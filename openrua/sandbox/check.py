"""Run a supplied CLI check in an image without project or network access."""

import subprocess
from uuid import uuid4

from openrua.sandbox import SandboxError


def cli_result(image: str, command: str) -> subprocess.CompletedProcess:
    """Return a CLI check's output and exit status after runtime error handling."""
    name = 'openrua-image-check-' + uuid4().hex
    try:
        result = subprocess.run(['docker', 'run', '--rm', '--pull', 'never', '--network', 'none',
                                 '--name', name, '--entrypoint', '', '-e', 'HOME=/tmp', image, 'bash', '-c', command],
                                capture_output=True, text=True, timeout=60)
    except (subprocess.TimeoutExpired, KeyboardInterrupt) as exc:
        outcome = 'was interrupted' if isinstance(exc, KeyboardInterrupt) else 'timed out'
        try:
            removed = subprocess.run(['docker', 'rm', '-f', name], capture_output=True, text=True, timeout=30)
        except (OSError, subprocess.TimeoutExpired, KeyboardInterrupt) as cleanup:
            detail = str(cleanup) or type(cleanup).__name__
            raise SandboxError(f'CLI check in {image} {outcome} and cleanup failed: {detail}',
                               hint=f'restore Docker access, then run docker rm -f {name}') from exc
        if removed.returncode and 'No such container' not in (getattr(removed, 'stderr', '') or ''):
            raise SandboxError(f'CLI check in {image} {outcome}; cleanup failed: {removed.stderr.strip()}',
                               hint=f'restore Docker access, then run docker rm -f {name}') from exc
        if isinstance(exc, KeyboardInterrupt):
            raise
        raise SandboxError(f'CLI check in {image} timed out; the image was not rebuilt.',
                           hint='inspect Docker and the agent CLI, then retry startup') from exc
    except OSError as exc:
        raise SandboxError(f'Cannot check CLI in {image}: {exc}', hint='check docker info, then retry startup') from exc
    if result.returncode == 125:
        raise SandboxError(result.stderr.strip() or result.stdout.strip() or f'Docker could not start the CLI check for {image}',
                           hint='restore Docker access and retry startup; the image was not rebuilt')
    return result


def cli_ready(image: str, command: str) -> bool:
    return cli_result(image, command).returncode == 0
