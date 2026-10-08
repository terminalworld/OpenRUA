"""Experimental Kimi Code plugin with native OAuth or explicit Moonshot API auth."""
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

    def _native(self, source):
        try:
            import tomllib
        except ImportError:
            try:
                import tomli as tomllib
            except ImportError:
                raise ValueError('native Kimi profiles require Python 3.11+ or: pip install tomli') from None
        try:
            data = tomllib.loads((source / 'config.toml').read_text())
            models = data.get('models', {})
            chosen = models.get(data.get('default_model'), {})
            provider = data.get('providers', {}).get(chosen.get('provider'), {})
            oauth = provider.get('oauth', {})
            if (chosen.get('model') != self.default_model or
                    provider.get('type') != 'kimi' or
                    provider.get('base_url', '').rstrip('/') != 'https://api.kimi.com/coding/v1' or
                    oauth.get('storage') != 'file' or oauth.get('key') != 'oauth/kimi-code' or
                    oauth.get('oauth_host', 'https://auth.kimi.com').rstrip('/') != 'https://auth.kimi.com'):
                raise ValueError('native Kimi requires the selected official OAuth model; set default_model in the external manifest to its model ID')
            token_file = source / 'credentials/kimi-code.json'
            token = json.loads(token_file.read_text())
            if not isinstance(token, dict) or not all(isinstance(token.get(k), str) and token[k].strip()
                                                      for k in ('access_token', 'refresh_token')):
                raise ValueError('native Kimi OAuth credentials are incomplete; run kimi and /login')
            if (not isinstance(chosen.get('max_context_size', 131072), int) or
                    chosen.get('max_context_size', 131072) <= 0 or
                    not isinstance(chosen.get('capabilities', []), list) or
                    not all(isinstance(c, str) for c in chosen.get('capabilities', []))):
                raise ValueError('native Kimi model metadata is invalid')
            return chosen, token_file
        except (OSError, UnicodeError, tomllib.TOMLDecodeError, json.JSONDecodeError, AttributeError, TypeError):
            raise ValueError('native Kimi profile is missing or invalid; run kimi and /login') from None

    def inspect_login(self, source):
        try:
            self._native(source)
        except ValueError as error:
            return ProfileLogin(False, str(error))
        return ProfileLogin(True, 'native OAuth files found locally; subscription and refresh are not verified')

    def login_hint(self, creds_home):
        return 'run kimi and /login for native subscription access; API use requires config set --auth api --api-key-file /absolute/path/to/key'

    def prepare_profile(self, source, dest, *, require_credentials=True):
        chosen, token_file = self._native(source)
        directory, _ = _copy_profile(source, dest, None, False, '')
        q = json.dumps
        # Keep only the selected managed model, never a fallback API provider.
        content = ('default_model = "openrua"\ntelemetry = false\nauto_session_title = false\nbuiltin_product_skills = false\n'
                   '[providers."managed:kimi-code"]\ntype = "kimi"\nbase_url = "https://api.kimi.com/coding/v1"\napi_key = ""\n'
                   '[providers."managed:kimi-code".oauth]\nstorage = "file"\nkey = "oauth/kimi-code"\n'
                   '[models.openrua]\nprovider = "managed:kimi-code"\nmodel = ' + q(chosen['model']) + '\n'
                   'max_context_size = ' + str(int(chosen.get('max_context_size', 131072))) + '\n'
                   'capabilities = ' + q(chosen.get('capabilities', ['tool_use', 'image_in'])) + '\n')
        with open(directory / 'config.toml', 'x', encoding='utf-8',
                  opener=lambda p, f: os.open(p, f, 0o600)) as stream:
            stream.write(content)
        # Native refresh atomically replaces files; locks live under oauth/.
        # Share both directories, while sessions stay in the private home.
        (directory / 'credentials').mkdir(mode=0o700)
        (directory / 'oauth').mkdir(mode=0o700)
        (source / 'oauth').mkdir(mode=0o700, exist_ok=True)
        def secrets():
            current = json.loads(token_file.read_text())
            return [current[k] for k in ('access_token', 'refresh_token') if current.get(k)]
        return PreparedProfile(directory, (*self.sandbox_mounts(directory),
            f'{token_file.parent.resolve()}:/kimi-home/credentials',
            f"{(source / 'oauth').resolve()}:/kimi-home/oauth"), secrets)

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
