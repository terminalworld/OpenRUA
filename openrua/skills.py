"""Ordinary skill files and project policy, independent of agents and robots."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import fcntl
import json
import os
from pathlib import Path
import shutil
import tempfile

import yaml

PROJECT_DIRECTORY = '.agents/skills'
MODES = ('auto', 'suggest', 'manual')


def _directory(path: Path, boundary: Path) -> None:
    if not path.resolve().is_relative_to(boundary.resolve()):
        raise ValueError(f'skill directory must stay inside {boundary}: {path}')
    if path.exists() and not path.is_dir():
        raise ValueError(f'skill directory is not a directory: {path}')
    path.mkdir(parents=True, exist_ok=True)


def bridge(project: Path, native_directories: tuple[str, ...]) -> list[str]:
    """Link native project directories without copying or replacing user files.

    When native directories already coexist, link missing names in both
    directions. Report independent same-name definitions for the user to resolve.
    """
    project = project.resolve()
    metadata = project / '.openrua'
    _directory(metadata, project)
    lock_path = metadata / 'skills.lock'
    if lock_path.is_symlink():
        raise ValueError(f'skill lock must be a regular file: {lock_path}')
    with lock_path.open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        common = project / PROJECT_DIRECTORY
        _directory(common, project)
        roots = [common]
        for relative in dict.fromkeys(native_directories):
            path = project / relative
            if path == common:
                continue
            _directory(path.parent, project)
            if not path.exists() and not path.is_symlink():
                path.symlink_to(os.path.relpath(common, path.parent), target_is_directory=True)
            _directory(path, project)
            if path.resolve() != common.resolve():
                roots.append(path)
        issues = []
        for source in roots:
            for item in sorted(source.iterdir()):
                if item.name.startswith('.') or not (item / 'SKILL.md').is_file():
                    continue
                if not item.resolve().is_relative_to(project):
                    raise ValueError(f'project skill points outside the project: {item}')
                for target in roots:
                    alias = target / item.name
                    if alias.exists() or alias.is_symlink():
                        if alias.resolve() != item.resolve():
                            message = f'conflicting skill {item.name}: {item} and {alias}'
                            if message not in issues:
                                issues.append(message)
                        continue
                    alias.symlink_to(os.path.relpath(item, target), target_is_directory=True)
        return issues


def mode(project: Path) -> str:
    path = project / '.openrua' / 'skills.json'
    if not path.exists():
        return 'auto'
    value = json.loads(path.read_text())
    if not isinstance(value, dict) or set(value) != {'accumulation'} or value['accumulation'] not in MODES:
        raise ValueError(f'invalid skills policy: {path}; use openrua skills policy auto|suggest|manual')
    return value['accumulation']


def set_mode(project: Path, value: str) -> None:
    if value not in MODES:
        raise ValueError(f'accumulation must be one of {", ".join(MODES)}')
    metadata = project / '.openrua'
    _directory(metadata, project)
    target = metadata / 'skills.json'
    if target.is_symlink():
        raise ValueError(f'skills policy must be a regular file: {target}')
    fd, temporary = tempfile.mkstemp(prefix='.skills-', dir=metadata)
    try:
        with os.fdopen(fd, 'w') as stream:
            json.dump({'accumulation': value}, stream, indent=2)
            stream.write('\n')
        os.replace(temporary, target)
    finally:
        Path(temporary).unlink(missing_ok=True)


@dataclass(frozen=True)
class Skill:
    name: str
    description: str
    path: str
    scope: str
    error: str | None = None

    def as_dict(self) -> dict:
        return asdict(self)


def catalog(roots: list[tuple[str, Path]], *, boundary: Path | None = None,
            max_bytes: int | None = None) -> list[Skill]:
    """Inspect file metadata, without claiming native availability or validation."""
    result = []
    seen = set()
    for scope, root in roots:
        if boundary is not None and not root.resolve().is_relative_to(boundary.resolve()):
            continue
        for path in sorted(root.glob('*/SKILL.md')):
            if boundary is not None and not path.resolve().is_relative_to(boundary.resolve()):
                continue
            if path.parent.name.startswith('.') or path.resolve() in seen:
                continue
            seen.add(path.resolve())
            name, description, error = path.parent.name, '', None
            try:
                with path.open() as stream:
                    text = stream.read(max_bytes + 1) if max_bytes is not None else stream.read()
                if max_bytes is not None and len(text.encode()) > max_bytes:
                    raise ValueError('SKILL.md exceeds the preview limit')
                lines = text.splitlines()
                if not lines or lines[0] != '---' or '---' not in lines[1:]:
                    raise ValueError('SKILL.md needs YAML front matter with name and description')
                end = lines.index('---', 1)
                fields = yaml.safe_load('\n'.join(lines[1:end]))
                if not isinstance(fields, dict) or not all(isinstance(fields.get(k), str) and fields[k].strip() for k in ('name', 'description')):
                    raise ValueError('name and description must be nonempty strings')
                name, description = fields['name'], fields['description']
            except (ValueError, OSError, yaml.YAMLError) as exc:
                error = str(exc)
            result.append(Skill(name, description, str(path), scope, error))
    return result


def install(source: Path, destination: Path) -> Path:
    """Copy one explicitly selected skill, refusing to overwrite existing work."""
    source = source.expanduser().resolve()
    if source.is_file():
        source = source.parent
    entries = catalog([('source', source.parent)])
    entry = next((entry for entry in entries if Path(entry.path).parent == source), None)
    if entry is None or entry.error:
        raise ValueError(f'not a valid skill directory: {source}' + (f': {entry.error}' if entry else ''))
    if any(path.is_symlink() for path in source.rglob('*')):
        raise ValueError(f'import requires ordinary files; resolve symlinks first: {source}')
    destination.mkdir(parents=True, exist_ok=True)
    target = destination / source.name
    if target.exists() or target.is_symlink():
        raise ValueError(f'skill already exists: {target}; review and edit it instead of overwriting')
    temporary = Path(tempfile.mkdtemp(prefix='.import-', dir=destination))
    try:
        shutil.copytree(source, temporary / source.name)
        # Reserve the name exclusively before moving files into it.
        target.mkdir()
        try:
            for item in (temporary / source.name).iterdir():
                item.rename(target / item.name)
        except BaseException:
            shutil.rmtree(target)
            raise
    finally:
        shutil.rmtree(temporary)
    return target


GUIDANCE = """\
## OpenRUA skills

Before using or saving a reusable method, read /robot/skills.md and the current
.openrua/skills.json policy if present. Keep skills as ordinary SKILL.md files
with optional scripts and references in .agents/skills/<name>/. Existing native
skill directories remain usable. Skills describe how to work; current robot
facts and observations must be checked again for each task.
"""


def instructions(project: Path, issues: list[str]) -> str:
    selected = mode(project)
    policy = {
        'auto': 'You may save or improve a useful project skill during normal work when the task provides evidence. Briefly report meaningful changes. Do not manufacture skills for routine work.',
        'suggest': 'Suggest useful skill additions or improvements, but obtain the user\'s agreement before saving them.',
        'manual': 'Create or improve skills only when the user explicitly asks. Existing skills may still be used.',
    }[selected]
    return f"""# Skills in this robot project

Accumulation policy: {selected}.
{policy}
Re-read .openrua/skills.json before writing skills; the user can change it while
this conversation is open. A missing policy file means auto.
If the user changes their accumulation preference, update that file with
{{"accumulation": "auto"}}, {{"accumulation": "suggest"}}, or
{{"accumulation": "manual"}} as requested and report the new preference.

Use your native skill discovery and creator when available. Save project
methods in .agents/skills/<name>/SKILL.md, with YAML name and description and
optional scripts or references. Describe when to use the method, dependencies,
machine assumptions, steps, checks, and the run or files supporting it. Use
relative paths for portable scripts. Improve an existing method instead of
making another copy. Preserve user instructions and agent-specific metadata.

Saving a file is not proof that its method works. Record actual checks and
unverified steps honestly. Do not operate a robot merely to verify a saved
skill. Re-observe the current scene before acting; old poses are observations,
not reusable constants. Distinguish method problems from a changed environment.

Personal methods are separate from this project's files. Promote a method to
personal scope only when the user requests it. Project and personal methods
can share a name; check the native agent's selection. Do not copy credentials into skills.
Use Git diff/history for maintained skills when this project uses Git.

Existing directory conflicts to review:
""" + ('\n'.join(f'- {issue}' for issue in issues) if issues else 'None.') + '\n'
