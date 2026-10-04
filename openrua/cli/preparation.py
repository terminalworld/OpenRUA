"""Prepare missing images for an explicit new-session selection."""
from __future__ import annotations

import fcntl
import hashlib
from pathlib import Path
import subprocess
import sys

from openrua import config, proxy
from openrua.config import paths
from openrua.errors import UnavailableError


def image_exists(image: str) -> bool:
    return subprocess.run(['docker', 'image', 'inspect', image],
                          capture_output=True, timeout=30).returncode == 0


def image_commands(composed, benchmark: str | None = None) -> list[tuple[str, list[str]]]:
    backend = composed.cfg['machine']['backend']
    selected = composed.cfg['agent']
    agent = selected['name']
    if selected.get('version'):
        agent += '@' + selected['version']
    commands = []
    if backend['kind'] == 'sim':
        selector = ['--bench', benchmark or composed.benchmark] if composed.benchmark else ['--sim', composed.simulator]
        commands.append((backend['image'], ['build', *selector]))
    commands.extend([
        (backend['sandbox_image'], ['build', 'sandbox', '--distro', backend['ros_distro'],
                                    '--tag', backend['sandbox_image'], '--agent', agent]),
        (proxy.IMAGE, ['build', 'proxy', '--agent', agent]),
    ])
    return commands


def prepare(home: Path, values: dict, progress=print) -> None:
    """Build only missing images, with streamed output and retained failure logs."""
    composed = config.compose(values['robot'], values['sim'], values['bench'], home,
                              agent=values['agent'] or None, model=values['model'] or None)
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
    for image, command in image_commands(composed, values['bench']):
        key = hashlib.sha256(image.encode()).hexdigest()[:16]
        with (directory / f'{key}.lock').open('a') as lock:
            # Another frontend may already be preparing the same image.
            progress(f'Checking {image}...')
            fcntl.flock(lock, fcntl.LOCK_EX)
            if image_exists(image):
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
            if code or not image_exists(image):
                raise UnavailableError(f'Could not prepare {image}; build exit status {code}. Log: {log}',
                                       hint='fix the build error shown above, then choose Prepare and start again; completed images are reused')
            progress(f'Prepared {image}')
