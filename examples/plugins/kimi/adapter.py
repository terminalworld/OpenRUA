"""Experimental Kimi Code plugin with explicit, isolated Moonshot API auth."""
from __future__ import annotations

import json
import os
from pathlib import Path

from openrua.agents.base import Agent, PreparedProfile, ProfileLogin, _copy_profile, read_api_key
from openrua.agents.conversation import Conversation
from openrua.plugins.agents.kimi_conversation import KimiConversation


class Kimi(Agent):
    def __init__(self, **fields):
        super().__init__(**fields)
        if self.version != '2.1.1':
            raise ValueError('this plugin supports Kimi Code 2.1.1 only')

    def inspect_login(self, source):
        return ProfileLogin(False, 'native subscription reuse is not validated for this experimental plugin; explicitly configure API auth')

    def login_hint(self, creds_home):
        return 'select this manifest with config set --auth api --api-key-file /absolute/path/to/key; native subscription reuse is not yet supported'

    def prepare_profile(self, source, dest, *, require_credentials=True):
        raise ValueError(self.login_hint(source))

    def prepare_api_profile(self, key_file, dest):
        key = read_api_key(key_file)
        directory, _ = _copy_profile(key_file, dest, None, False, '')
        q = json.dumps
        content = ('default_model = "openrua"\ntelemetry = false\nauto_session_title = false\nbuiltin_product_skills = false\n'
                   '[providers.openrua]\ntype = "kimi"\napi_key = ' + q(key) + '\nbase_url = ' + q(self.default_options['api_base_url']) + '\n'
                   '[models.openrua]\nprovider = "openrua"\nmodel = ' + q(self.default_model) + '\n'
                   'max_context_size = 131072\ncapabilities = ["tool_use", "image_in"]\n')
        with open(directory / 'config.toml', 'x', encoding='utf-8',
                  opener=lambda p, f: os.open(p, f, 0o600)) as stream:
            stream.write(content)
        return PreparedProfile(directory, self.sandbox_mounts(directory), lambda: [key])

    def _validate(self, model, options, token_file=None):
        if model != self.default_model:
            raise ValueError('set default_model in the manifest before preparing the API profile')
        if options and any(self.default_options.get(k) != v for k, v in options.items()):
            raise ValueError('configure options in the external manifest before preparing the profile')
        if token_file:
            raise ValueError('use explicit API profile authentication for Kimi')

    def _env(self, proxy):
        return ['-e', 'KIMI_CODE_HOME=/kimi-home', '-e', f'HTTPS_PROXY={proxy}', '-e', f'HTTP_PROXY={proxy}',
                '-e', 'NO_PROXY=localhost,127.0.0.1']

    def launch_argv(self, sandbox, prompt, model, max_turns, proxy, options=None,
                    session_id=None, resume=False, token_file=None, **unused):
        self._validate(model, options, token_file)
        if resume and not session_id:
            raise ValueError('Kimi resume requires an exact session ID')
        return [*self.exec_argv(sandbox, [*self._env(proxy), '-e', f'KIMI_LOOP_MAX_STEPS_PER_TURN={max_turns}']),
                self.binary, '--model', 'openrua', '--prompt', prompt, '--output-format', 'stream-json',
                *(['--session', session_id] if resume else [])]

    def interactive_argv(self, sandbox, model, proxy, options=None, prompt=None, **unused):
        self._validate(model, options)
        if prompt:
            raise ValueError('start without a prompt and enter it in the native Kimi terminal')
        return [*self.exec_argv(sandbox, self._env(proxy), interactive=True), self.binary, '--model', 'openrua']

    def conversation(self, sandbox, model, proxy, options=None, session_id=None, **unused):
        self._validate(model, options)
        # The standalone stdlib bridge runs next to Kimi inside the sandbox.
        # Pass its source explicitly; the image needs no second OpenRUA install.
        source = Path(__file__).with_name('bridge.py').read_text()
        command = [*self.exec_argv(sandbox, self._env(proxy), stdin=True), 'python3', '-c', source,
                   '--home', '/kimi-home', '--cwd', '/workspace', '--', self.binary]
        return Conversation(command, KimiConversation('/workspace', 'openrua', session_id))

    def sandbox_cli_check(self):
        return ('sandbox_cli_matches_pin', "bash -c 'kimi --version | grep -qF 2.1.1'")


HOOKS = Kimi
