"""Experimental ZCode plugin using native Coding Plan profiles or explicit API auth."""
from __future__ import annotations

import json
import os
from pathlib import Path

from openrua.agents.base import Agent, PreparedProfile, ProfileLogin, _copy_profile, read_api_key
from openrua.agents.conversation import Conversation
from openrua.plugins.agents.zcode_conversation import ZCodeConversation


class ZCode(Agent):
    def __init__(self, **fields):
        super().__init__(**fields)
        if self.version != "0.16.9":
            raise ValueError("this plugin supports ZCode CLI 0.16.9 only")

    def _native(self, source):
        try:
            paths = [source / 'v2/provider_config.json', source / '.zcode/v2/provider_config.json']
            present = [path for path in paths if path.is_file()]
            if len(present) != 1:
                raise ValueError('native ZCode profile path must identify exactly one provider_config.json')
            document = json.loads(present[0].read_text())
            if document.get('schemaVersion') != 1:
                raise ValueError('unsupported native ZCode provider schema; expected schemaVersion 1')
            config = document['config']
            selected = config['defaultModelSelection']
            rules = config['providerConfigRules']['providerRules']
            matches = [r for r in rules if r.get('providerId') == selected['providerId']]
            if len(matches) != 1 or matches[0].get('enabled') is False:
                raise ValueError('select an enabled native Coding Plan provider in ZCode first')
            provider = matches[0]['config']
            access, api = provider['access'], provider['api']
            if access.get('type') != 'zhipu-coding-plan-api-key':
                raise ValueError('native ZCode currently supports Coding Plan key profiles only; OAuth accounts and ordinary API providers are not imported')
            key = access.get('apiKey')
            if not isinstance(key, str) or not key.strip() or any(c.isspace() for c in key):
                raise ValueError('configure the Coding Plan key in native ZCode first')
            if (api.get('type') != 'anthropic-messages' or api.get('baseUrl', '').rstrip('/') not in
                    ('https://open.bigmodel.cn/api/anthropic', 'https://api.z.ai/api/anthropic')):
                raise ValueError('native ZCode requires an official Coding Plan endpoint')
            if selected['modelId'] != self.default_model:
                raise ValueError('set default_model in the external manifest to the model selected in native ZCode')
            # Resolve only a self-contained provider rule. Inherited templates
            # need explicit materialization by the native client first.
            return {"schemaVersion": 1, "config": {
                "providerConfigRules": {"providerRules": [{"providerId": selected['providerId'],
                    "enabled": True, "config": {"group": provider.get('group', 'standard-personal'),
                    "access": {"type": access['type'], "apiKey": key},
                    "api": {"type": api['type'], "baseUrl": api['baseUrl']},
                    "personalModelIds": [selected['modelId']]}}]},
                "modelConfigRules": {"providerModelRules": [], "manualProviderModelRules": []},
                "defaultModelSelection": selected}}, key
        except (OSError, UnicodeError, json.JSONDecodeError, KeyError, AttributeError, TypeError):
            raise ValueError('native ZCode needs a complete Coding Plan provider_config.json; configure it in ZCode first (OAuth reuse is not supported yet)') from None

    def inspect_login(self, source):
        try:
            self._native(source)
        except ValueError as error:
            return ProfileLogin(False, str(error))
        return ProfileLogin(True, 'native Coding Plan key profile found locally; subscription validity is not verified')

    def prepare_profile(self, source, dest, *, require_credentials=True):
        config, key = self._native(source)
        directory, _ = _copy_profile(source, dest, None, False, '')
        with open(directory / 'provider_config.json', 'x', encoding='utf-8',
                  opener=lambda path, flags: os.open(path, flags, 0o600)) as stream:
            json.dump(config, stream)
        return PreparedProfile(directory, self.sandbox_mounts(directory), lambda: [key])

    def login_hint(self, creds_home):
        return 'configure a native ZCode Coding Plan key profile; OAuth reuse is not supported; API use requires config set --auth api --api-key-file /absolute/path/to/key'

    def prepare_api_profile(self, key_file: Path, dest: Path) -> PreparedProfile:
        key = read_api_key(key_file)
        directory, _ = _copy_profile(key_file, dest, None, False, "")
        config = {"schemaVersion": 1, "config": {
            "providerConfigRules": {"providerRules": [{"providerId": "openrua", "enabled": True, "config": {
                "group": "standard-personal", "access": {"type": "api-key", "apiKey": key},
                "api": {"type": "openai-chat-completions", "baseUrl": self.default_options["api_base_url"]},
                "personalModelIds": [self.default_model]}}]},
            "modelConfigRules": {"providerModelRules": [], "manualProviderModelRules": []},
            "defaultModelSelection": {"providerId": "openrua", "modelId": self.default_model}}}
        with open(directory / "provider_config.json", "x", encoding="utf-8",
                  opener=lambda path, flags: os.open(path, flags, 0o600)) as stream:
            json.dump(config, stream)
        return PreparedProfile(directory, self.sandbox_mounts(directory), lambda: [key])

    def _env(self, proxy):
        return ["-e", "ZCODE_DATA_BASE_DIR=/zcode-home",
                "-e", "ZCODE_PERSONAL_PROVIDER_CONFIG_FILE=/zcode-home/provider_config.json",
                "-e", "ZCODE_BUILTIN_PROVIDER_CONFIG_FILE=/opt/openrua-zcode/source/config/provider/zcode-builtin.json",
                "-e", f"HTTPS_PROXY={proxy}", "-e", f"HTTP_PROXY={proxy}"]

    def _validate(self, model, options):
        if model != self.default_model:
            raise ValueError("set default_model in the external manifest before preparing the API profile")
        if options and any(self.default_options.get(k) != v for k, v in options.items()):
            raise ValueError("configure ZCode options in the external manifest before preparing the profile")

    def launch_argv(self, sandbox, prompt, model, max_turns, proxy, options=None,
                    session_id=None, resume=False, token_file=None, **unused):
        self._validate(model, options)
        if token_file:
            raise ValueError("use explicit API profile authentication for ZCode")
        if resume and not session_id:
            raise ValueError("ZCode resume requires an exact session ID")
        return [*self.exec_argv(sandbox, self._env(proxy)), self.binary, "--prompt", prompt,
                "--json", "--cwd", "/workspace", "--mode", "yolo",
                *(["--resume", session_id] if resume else [])]

    def interactive_argv(self, sandbox, model, proxy, options=None, prompt=None, **unused):
        self._validate(model, options)
        if prompt:
            raise ValueError("start without a prompt and enter it in the native ZCode terminal")
        return [*self.exec_argv(sandbox, self._env(proxy), interactive=True),
                self.binary, "--cwd", "/workspace"]

    def conversation(self, sandbox, model, proxy, options=None, session_id=None, **unused):
        self._validate(model, options)
        argv = [*self.exec_argv(sandbox, self._env(proxy), stdin=True), self.binary, "app-server"]
        return Conversation(argv, ZCodeConversation("/workspace", session_id))

    def sandbox_cli_check(self):
        return ("sandbox_cli_matches_pin", "bash -c 'zcode --version | grep -qF 0.16.9'")


HOOKS = ZCode
