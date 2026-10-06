"""Experimental ZCode 0.16.9 plugin using explicitly selected BigModel API auth."""
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

    def inspect_login(self, source):
        return ProfileLogin(False, "this experimental plugin requires explicit API configuration; native account reuse is not yet supported")

    def prepare_profile(self, source, dest, *, require_credentials=True):
        raise ValueError(self.login_hint(source))

    def login_hint(self, creds_home):
        return "select this manifest with config set --auth api --api-key-file /absolute/path/to/key; native subscription reuse is not yet supported"

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
