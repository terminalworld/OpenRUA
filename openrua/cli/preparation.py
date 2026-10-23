"""Prepare missing images for an explicit new-session selection."""
from __future__ import annotations

import fcntl
import hashlib
from pathlib import Path
import shlex
import subprocess
import sys

from openrua import agents, config, proxy
from openrua.config import paths
from openrua.config.schema import sandbox_image
from openrua.cli.agent_settings import resolve
from openrua.errors import UnavailableError
from openrua.sandbox.check import cli_ready


def image_exists(image: str) -> bool:
    return subprocess.run(['docker', 'image', 'inspect', image],
                          capture_output=True, timeout=30).returncode == 0


def image_commands(composed, benchmark: str | None = None,
                   limits: dict | None = None) -> list[tuple[str, list[str]]]:
    """``(image, openrua build arguments)`` for every image this selection
    runs: the simulator's, or a real robot's driver images; the sandbox
    (a real robot's own when it installs packages into it); the proxy
    only when ``limits`` keep the agent to its model service."""
    backend = composed.cfg['machine']['backend']
    selected = composed.cfg['agent']
    agent = selected['name']
    if selected.get('version'):
        agent += '@' + selected['version']
    commands = []
    if backend['kind'] == 'sim':
        selector = ['--bench', benchmark or composed.benchmark] if composed.benchmark else ['--sim', composed.simulator]
        commands.append((backend['image'], ['build', *selector]))
    robot_file = None
    if backend['kind'] == 'real' and (backend.get('build') or backend.get('sandbox_install')):
        robot_file = str(paths.find('robots', composed.robot).resolve())
    for image in backend.get('build') or {}:
        commands.append((image, ['build', 'driver', '--robot', robot_file, '--image', image]))
    if backend.get('sandbox_install'):
        commands.append((backend['sandbox_image'], ['build', 'sandbox', '--robot', robot_file,
                                                    '--agent', agent]))
    else:
        commands.append((backend['sandbox_image'], ['build', 'sandbox', '--distro', backend['ros_distro'],
                                                    '--tag', backend['sandbox_image'], '--agent', agent]))
    if (limits or {}).get('internet') == 'model-only':
        commands.append((proxy.IMAGE, ['build', 'proxy']))
    return commands


def prepare(home: Path, values: dict, progress=print, limits: dict | None = None) -> None:
    """Build only missing images, with streamed output and retained failure logs.
    ``limits`` are the session's (a proxy image only when it needs one)."""
    composed = config.compose(values['robot'], values['sim'], values['bench'], home,
                              agent=values['agent'] or None, model=values['model'] or None)
    selected = composed.cfg['agent']
    adapter = agents.get(selected['name'], home, version=selected.get('version'))
    image = composed.cfg['machine']['backend']['sandbox_image']
    _prepare_images(home, image_commands(composed, values['bench'], limits), progress,
                    {image: _agent_probe(adapter)})


def prepare_connect(home: Path, agent: str | None = None, distro: str = 'jazzy',
                    progress=print) -> None:
    """Prepare the connecting terminal before a robot configuration exists."""
    adapter, selected = resolve(home, agent)
    image = sandbox_image(distro)
    name, _ = agents.split_pin(selected['name'])
    requested = name + ('@' + adapter.version if adapter.version else '')
    _prepare_images(home, [(image, ['build', 'sandbox', '--distro', distro,
                                  '--tag', image, '--agent', requested])], progress,
                    {image: _agent_probe(adapter)})


def _agent_probe(adapter) -> str | None:
    check = adapter.sandbox_cli_check()
    return check[1] if check else adapter.version_probe() or (
        'command -v ' + shlex.quote(adapter.binary) if adapter.binary else None)


def _ready(image: str, probe: str | None) -> bool:
    if not image_exists(image):
        return False
    if probe is None:
        return True
    return cli_ready(image, probe)


def _prepare_images(home: Path, commands: list[tuple[str, list[str]]], progress,
                    probes: dict[str, str | None] | None = None) -> None:
    """Share image locks, build output and failure logs across startup paths."""
    try:
        check = subprocess.run(['docker', 'info', '--format', '{{.ServerVersion}}'],
                               capture_output=True, text=True, timeout=30)
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise UnavailableError(f'Cannot access Docker: {exc}', hint='start Docker and retry startup') from exc
    if check.returncode:
        raise UnavailableError(check.stderr.strip() or 'Docker is unavailable',
                               hint='restore Docker access and retry startup')
    directory = paths.home(home) / 'preparation'
    directory.mkdir(parents=True, exist_ok=True)
    for image, command in commands:
        key = hashlib.sha256(image.encode()).hexdigest()[:16]
        with (directory / f'{key}.lock').open('a') as lock:
            # Another frontend may already be preparing the same image.
            progress(f'Checking {image}...')
            fcntl.flock(lock, fcntl.LOCK_EX)
            probe = (probes or {}).get(image)
            if _ready(image, probe):
                progress(f'Reusing {image}')
                continue
            log = directory / f'{key}.log'
            progress(f'Building {image}. First-time downloads may be large. Log: {log}')
            argv = [sys.executable, '-m', 'openrua', '--home', str(home), *command]
            with log.open('w') as stream:
                process = subprocess.Popen(argv, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                           text=True, bufsize=1)
                try:
                    for line in process.stdout:
                        stream.write(line)
                        stream.flush()
                        progress(line.rstrip())
                    code = process.wait()
                except BaseException:
                    process.terminate()
                    process.wait()
                    raise
            if code or not _ready(image, probe):
                hint = 'fix the build error shown above, then retry startup; completed images are reused'
                with log.open('rb') as stream:
                    stream.seek(0, 2)
                    stream.seek(max(0, stream.tell() - 8192))
                    tail = stream.read().decode(errors='replace')
                if 'cachemounts' in tail and 'no such file or directory' in tail:
                    hint = ('Docker reports a missing BuildKit working directory. Inspect docker info, '
                            'docker buildx ls and the data-root mount. Check docker ps before restarting '
                            'Docker to reinitialize the builder, then retry startup. '
                            'https://github.com/terminalworld/OpenRUA/blob/main/docs/connect.md')
                raise UnavailableError(f'Could not prepare {image}; build exit status {code}. Log: {log}',
                                       hint=hint)
            progress(f'Prepared {image}')
