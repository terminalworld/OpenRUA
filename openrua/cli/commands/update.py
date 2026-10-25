"""Update one coding agent in an existing tool image."""
from __future__ import annotations

import fcntl
import hashlib
import json
import subprocess
from pathlib import Path

from openrua import agents
from openrua.cli.commands import build
from openrua.config import paths
from openrua.errors import UnavailableError, UsageError
from openrua.sandbox import SandboxError


def run(args) -> int:
    try:
        return _update(args)
    except (SandboxError, OSError, subprocess.SubprocessError, json.JSONDecodeError) as exc:
        raise UnavailableError(str(exc), hint=f'inspect docker image inspect {args.image} and the update log before retrying') from exc


def _update(args) -> int:
    target = agents.manifest(args.agent)
    if not target.fields.get('version'):
        raise UsageError('select an explicit target release: --agent <name>@<version>')
    try:
        adapter = agents.compose(target)
    except (ValueError, TypeError) as exc:
        raise UsageError(str(exc), hint='select a CLI version supported by this plugin') from exc
    check = adapter.installation_check()
    if not check or not adapter.version_argv or not target.install:
        raise UsageError('this plugin does not provide a verifiable versioned installation')
    locks = paths.home(args.home) / 'preparation'
    locks.mkdir(parents=True, exist_ok=True)
    identity = hashlib.sha256(args.image.encode()).hexdigest()[:16]
    with (locks / f'{identity}.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        found = subprocess.run(['docker', 'image', 'inspect', args.image, '--format', '{{json .}}'],
                               capture_output=True, text=True, timeout=30)
        if found.returncode:
            raise UnavailableError(f'cannot inspect {args.image}: {found.stderr.strip()}',
                                   hint='prepare this project before updating its agent')
        facts = json.loads(found.stdout)
        env = dict(item.split('=', 1) for item in facts['Config'].get('Env') or [] if '=' in item)
        print('[update] Checking the installed CLI', flush=True)
        ready = subprocess.run(['docker', 'run', '--rm', '--entrypoint', '', facts['Id'],
                                'bash', '-c', check[1]], capture_output=True, text=True)
        print('[update] Checking runtime dependencies; compatible tools are retained', flush=True)
        runtime_ready = True
        if adapter.prepare_check:
            runtime = subprocess.run(['docker', 'run', '--rm', '--entrypoint', '', facts['Id'],
                                      'bash', '-c', adapter.prepare_check], capture_output=True, text=True)
            print('[update] ' + ('Reusing compatible runtime dependencies' if runtime.returncode == 0
                                 else 'Preparing the runtime dependencies required by this component'), flush=True)
            runtime_ready = runtime.returncode == 0
        if ready.returncode == 0 and runtime_ready:
            print(f'[update] {adapter.title or adapter.name} {adapter.version} is already installed', flush=True)
            return 0
        print(f'[update] Installing {adapter.title or adapter.name} {adapter.version}', flush=True)
        tag, digest = build._sandbox([args.agent], None, env.get('ROS_DISTRO', 'jazzy'), None,
                                     args.image, args.home, base_image=facts['Id'], expected_base=facts['Id'])
        print(f'[update] Verified {adapter.title or adapter.name} {adapter.version}; ready for new sessions', flush=True)
        print(f'{tag} {digest}', flush=True)
    return 0


def add_parser(sub) -> None:
    parser = sub.add_parser('update', help='update one coding agent in an existing tool image')
    parser.add_argument('--agent', required=True, help='agent manifest or name with an explicit @version')
    parser.add_argument('--image', required=True, help='existing tool image; running containers retain their original image')
    parser.set_defaults(fn=run)
