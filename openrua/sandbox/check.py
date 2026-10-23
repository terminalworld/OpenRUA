"""Run a supplied CLI check in an image without project or network access."""

import subprocess
from uuid import uuid4

from openrua.sandbox import SandboxError


def cli_ready(image: str, command: str) -> bool:
    name = 'openrua-image-check-' + uuid4().hex
    try:
        result = subprocess.run(['docker', 'run', '--rm', '--pull', 'never', '--network', 'none',
                                 '--name', name, '--entrypoint', '', '-e', 'HOME=/tmp', image, 'bash', '-c', command],
                                capture_output=True, text=True, timeout=60)
    except subprocess.TimeoutExpired as exc:
        try:
            removed = subprocess.run(['docker', 'rm', '-f', name], capture_output=True, text=True, timeout=30)
        except (OSError, subprocess.TimeoutExpired) as cleanup:
            raise SandboxError(f'CLI check in {image} timed out and cleanup failed: {cleanup}',
                               hint=f'restore Docker access, then run docker rm -f {name}') from exc
        if removed.returncode and 'No such container' not in (getattr(removed, 'stderr', '') or ''):
            raise SandboxError(f'CLI check in {image} timed out; cleanup failed: {removed.stderr.strip()}',
                               hint=f'restore Docker access, then run docker rm -f {name}') from exc
        raise SandboxError(f'CLI check in {image} timed out; the image was not rebuilt.',
                           hint='inspect Docker and the agent CLI, then retry startup') from exc
    except OSError as exc:
        raise SandboxError(f'Cannot check CLI in {image}: {exc}', hint='check docker info, then retry startup') from exc
    if result.returncode == 125:
        raise SandboxError(result.stderr.strip() or result.stdout.strip() or f'Docker could not start the CLI check for {image}',
                           hint='restore Docker access and retry startup; the image was not rebuilt')
    return result.returncode == 0
