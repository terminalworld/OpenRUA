"""Run a supplied CLI check in an image without project or network access."""

import subprocess


def cli_ready(image: str, command: str) -> bool:
    return subprocess.run(['docker', 'run', '--rm', '--pull', 'never', '--network', 'none',
                           '--entrypoint', '', '-e', 'HOME=/tmp', image, 'bash', '-c', command],
                          capture_output=True, timeout=60).returncode == 0
