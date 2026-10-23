"""Connect native agent skill locations to persistent project and user files."""

from pathlib import Path

from openrua import agents, skills
from openrua.errors import UsageError


def project_directories() -> tuple[str, ...]:
    return tuple(dict.fromkeys(entry.agent.skills.project for entry in agents.available()
                               if entry.agent is not None and entry.agent.skills is not None))


def prepare(root: Path, home: Path, adapter, materials: Path) -> tuple[str, ...]:
    try:
        return _prepare(root, home, adapter, materials)
    except (ValueError, OSError) as exc:
        raise UsageError(f'cannot prepare project skills: {exc}',
                         hint='inspect openrua skills list --json and openrua skills policy') from exc


def _prepare(root: Path, home: Path, adapter, materials: Path) -> tuple[str, ...]:
    """Project-only setup; benchmark profiles never pass through here."""
    directories = project_directories()
    locations = getattr(adapter, 'skills', None)
    if locations is not None:
        directories = (*directories, locations.project)
    issues = skills.bridge(root, directories)
    materials.mkdir(parents=True, exist_ok=True)
    (materials / 'skills.md').write_text(skills.instructions(root, issues))
    instructions = root / 'AGENTS.md'
    # project.prepare has already validated and serialized initial instructions.
    import fcntl
    with (root / '.openrua' / 'initialize.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        if skills.GUIDANCE.strip() not in instructions.read_text():
            with instructions.open('a') as stream:
                stream.write('\n' + skills.GUIDANCE)
    if locations is None:
        return ()
    personal = home / 'skills'
    personal.mkdir(parents=True, exist_ok=True)
    return (f'{personal}:{locations.personal}',)
