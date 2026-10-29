"""Find an installed agent image without resolving a robot project."""

from datetime import datetime
import json
import os
import re
import subprocess

from openrua import agents
from openrua.errors import UnavailableError
from openrua.sandbox.check import cli_ready


def _docker(*args: str) -> str:
    try:
        result = subprocess.run(['docker', *args], capture_output=True, text=True, timeout=30)
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise UnavailableError(f'Cannot inspect installed agent images: {exc}',
                               hint='check docker info and retry model selection') from exc
    if result.returncode:
        raise UnavailableError(result.stderr.strip() or 'Docker could not inspect installed agent images.',
                               hint='restore Docker access and retry model selection')
    return result.stdout


def _created(value: str) -> datetime:
    # Docker timestamps can have nanoseconds. Normalize the fraction for
    # Python 3.10's ISO parser while preserving the timezone.
    value = re.sub(r'\.(\d+)', lambda match: '.' + match[1][:6].ljust(6, '0'), value)
    return datetime.fromisoformat(value.replace('Z', '+00:00'))


def find(adapter: agents.Agent, probe: str | None) -> str | None:
    """Use the newest matching, compatible image by its immutable ID."""
    label = f'openrua.agent.{adapter.name}.install_sha256={agents.fact_sha256(adapter, "install")}'
    ids = list(dict.fromkeys(_docker('image', 'ls', '--filter', f'label={label}',
                                    '--format', '{{.ID}}', '--no-trunc').split()))
    if not ids:
        return None
    raw = _docker('image', 'inspect', *ids)
    try:
        images = json.loads(raw)
        candidates = [facts for facts in images if any(
            '<none>' not in tag and '-candidate-' not in tag and not tag.startswith('openrua-build-base-')
            for tag in facts.get('RepoTags') or [])]
        candidates.sort(key=lambda facts: (_created(facts['Created']), facts['Id']), reverse=True)
    except (KeyError, TypeError, ValueError, AttributeError) as exc:
        raise UnavailableError(f'Cannot read installed agent image details: {exc}',
                               hint='inspect docker image ls and retry model selection') from exc
    command = f'test "$(id -u robot)" = "{os.getuid()}"'
    if probe:
        command += f' && ({probe})'
    for facts in candidates:
        if cli_ready(facts['Id'], command):
            return facts['Id']
    return None
