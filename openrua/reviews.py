"""Bounded input snapshots and review proposals, independent of execution."""

from __future__ import annotations

import base64
from datetime import datetime, timezone
import difflib
import hashlib
import json
import os
from pathlib import Path
import stat
import tempfile

from openrua.artifacts import WorkspaceFiles
from openrua.skills import catalog


def digest(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def write_json(path: Path, value: dict) -> None:
    fd, temporary = tempfile.mkstemp(prefix='.review-', dir=path.parent)
    try:
        with os.fdopen(fd, 'w') as stream:
            json.dump(value, stream, indent=2)
            stream.write('\n')
        os.replace(temporary, path)
    finally:
        Path(temporary).unlink(missing_ok=True)


def directory(path: Path, boundary: Path) -> None:
    """Create a project-owned directory without following metadata links."""
    boundary = boundary.resolve()
    relative = path.relative_to(boundary)
    current = boundary
    for part in relative.parts:
        current = current / part
        if current.is_symlink():
            raise ValueError(f'review directory must not be a symlink: {current}')
        current.mkdir(exist_ok=True)


def walk(reader: WorkspaceFiles, start: str = '', *, hidden: bool = True):
    """List bounded paths through the common symlink-safe artifact reader."""
    pending = [start]
    visited = 0
    while pending:
        parent = pending.pop()
        listing = reader.list(parent)
        if listing['truncated']:
            yield parent, 'truncated'
        for item in listing['entries']:
            visited += 1
            if visited > reader.max_entries:
                yield parent, 'truncated'
                return
            path = f'{parent}/{item["name"]}' if parent else item['name']
            if not hidden and (item['name'].startswith('.') or item['name'] in {'node_modules', '__pycache__', 'venv', 'env'}):
                continue
            if item['kind'] == 'directory':
                pending.append(path)
            else:
                yield path, item['kind']


class Snapshot:
    """Record every copied input and every omitted selected input."""

    def __init__(self, root: Path, *, max_bytes: int, max_file_bytes: int, max_files: int):
        if min(max_bytes, max_file_bytes, max_files) <= 0:
            raise ValueError('review input limits must be positive')
        self.root = root
        root.mkdir(parents=True)
        self.max_bytes, self.max_file_bytes, self.max_files = max_bytes, max_file_bytes, max_files
        self.bytes = 0
        self.files: list[dict] = []
        self.skipped: list[dict] = []
        self._paths: set[str] = set()

    def omit(self, source: str, reason: str) -> None:
        self.skipped.append({'source': source, 'reason': reason})

    def add(self, path: str, data: bytes, *, source: str, modified: str | None = None,
            mode: int = 0o644) -> bool:
        if path in self._paths:
            return False
        if len(data) > self.max_file_bytes:
            self.omit(source, 'file exceeds the per-file input limit')
            return False
        if self.bytes + len(data) > self.max_bytes or len(self.files) >= self.max_files:
            self.omit(source, 'review input budget reached')
            return False
        target = self.root / path
        if target.is_absolute() and not target.is_relative_to(self.root):
            raise ValueError('snapshot paths must be relative to the snapshot')
        if '..' in Path(path).parts:
            raise ValueError('snapshot paths must not contain ..')
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(data)
        target.chmod(mode & 0o777)
        self._paths.add(path)
        self.bytes += len(data)
        self.files.append({'path': path, 'source': source, 'bytes': len(data),
                           'sha256': digest(data), 'modified': modified, 'mode': mode & 0o777})
        return True

    def capture(self, reader: WorkspaceFiles, path: str, destination: str, *, text_only: bool) -> bool:
        try:
            item = reader.read(path)
            if text_only and item['kind'] != 'text':
                self.omit(str(reader.root / path), 'non-text project context')
                return False
            return self.add(destination, base64.b64decode(item['data']),
                            source=str(reader.root / path), modified=item['modified'], mode=item['mode'])
        except (OSError, ValueError) as exc:
            self.omit(str(reader.root / path), str(exc))
            return False

    def manifest(self) -> dict:
        return {'captured_at': datetime.now(timezone.utc).isoformat(), 'bytes': self.bytes,
                'files': self.files, 'skipped': self.skipped,
                'consistency': 'Each file is checked while read; files are not one atomic project snapshot.'}


def project_inputs(snapshot: Snapshot, project: Path, native_roots: tuple[str, ...],
                   context_paths: list[str] | None = None) -> list[str]:
    """Copy ordinary project text and real skill owners; never change aliases."""
    reader = WorkspaceFiles(project, snapshot.max_file_bytes, snapshot.max_files)
    context = ((path, 'file') for path in context_paths) if context_paths is not None else walk(reader, hidden=False)
    for path, kind in context:
        if kind == 'file':
            snapshot.capture(reader, path, 'project/' + path, text_only=True)
        else:
            snapshot.omit(str(project / path), f'unsupported project entry: {kind}')
        if len(snapshot.files) >= snapshot.max_files:
            snapshot.omit(str(project), 'remaining project files omitted at input limit')
            break
    skill_paths = []
    seen = set()
    for relative in dict.fromkeys(native_roots):
        root = project / relative
        if not root.exists():
            continue
        for method in sorted(root.iterdir()):
            if method.name.startswith('.') or not (method / 'SKILL.md').is_file():
                continue
            owner = method.resolve()
            if not owner.is_relative_to(project) or owner in seen:
                if not owner.is_relative_to(project):
                    snapshot.omit(str(method), 'skill points outside this project')
                continue
            seen.add(owner)
            owner_path = owner.relative_to(project).as_posix()
            for path, kind in walk(reader, owner_path):
                if kind == 'file':
                    destination = 'project/' + path
                    if destination in snapshot._paths or snapshot.capture(reader, path, destination, text_only=False):
                        skill_paths.append(path)
                else:
                    snapshot.omit(str(project / path), f'unsupported skill entry: {kind}')
    policy = '.openrua/skills.json'
    if (project / policy).exists():
        snapshot.capture(reader, policy, 'project/' + policy, text_only=True)
    return skill_paths


def changes(before: Path, after: Path) -> tuple[list[dict], str]:
    """Compare archived regular files; neither tree contains live links."""
    first = {p.relative_to(before).as_posix(): p.read_bytes() for p in before.rglob('*') if p.is_file()}
    second = {p.relative_to(after).as_posix(): p.read_bytes() for p in after.rglob('*') if p.is_file()}
    modes_before = {path: stat.S_IMODE((before / path).stat().st_mode) for path in first}
    modes_after = {path: stat.S_IMODE((after / path).stat().st_mode) for path in second}
    rows, diff = [], []
    for path in sorted(first.keys() | second.keys()):
        old, new = first.get(path), second.get(path)
        first_mode, second_mode = modes_before.get(path), modes_after.get(path)
        if old == new and first_mode == second_mode:
            continue
        rows.append({'path': path, 'kind': 'add' if old is None else 'delete' if new is None else 'modify',
                     'before_sha256': digest(old) if old is not None else None,
                     'after_sha256': digest(new) if new is not None else None,
                     'before_mode': first_mode, 'after_mode': second_mode})
        if first_mode != second_mode:
            diff.append(f'Mode {path}: {oct(first_mode) if first_mode is not None else "absent"} -> '
                        f'{oct(second_mode) if second_mode is not None else "absent"}\n')
        try:
            left, right = (old or b'').decode('utf-8'), (new or b'').decode('utf-8')
            diff.extend(difflib.unified_diff(left.splitlines(keepends=True), right.splitlines(keepends=True),
                                           fromfile='before/' + path, tofile='proposed/' + path))
        except UnicodeError:
            diff.append(f'Binary files differ: {path}\n')
    return rows, ''.join(diff)


def proposals(project: Path, baseline: Path, workspace: Path, output: Path, roots: tuple[str, ...],
              *, max_bytes: int, max_file_bytes: int, max_files: int,
              source_digests: dict[str, str] | None = None) -> dict:
    """Archive only bounded skill proposals after the worker has stopped."""
    source = workspace / 'proposed'
    archive = Snapshot(output / 'proposed', max_bytes=max_bytes, max_file_bytes=max_file_bytes, max_files=max_files)
    errors = []
    if source.is_symlink():
        errors.append('proposed must be an ordinary directory')
    elif source.is_dir():
        reader = WorkspaceFiles(source, max_file_bytes, max_files)
        for path, kind in walk(reader):
            allowed = any(Path(path).is_relative_to(root) and len(Path(path).relative_to(root).parts) >= 2
                          for root in roots)
            if kind != 'file' or not allowed:
                errors.append(f'unsupported proposal entry: {path} ({kind})')
            elif not archive.capture(reader, path, path, text_only=False):
                errors.append(f'proposal could not be archived: {path}')
            if len(archive.files) >= max_files:
                errors.append('proposal file limit reached; remaining files were not read')
                break
    entries = catalog([('proposed', archive.root / root) for root in roots])
    errors.extend(f'{entry.path}: {entry.error}' for entry in entries if entry.error)
    for path in archive.root.rglob('*'):
        if path.is_file() and not any(path.is_relative_to(Path(entry.path).parent) for entry in entries):
            errors.append(f'proposal resource has no SKILL.md: {path.relative_to(archive.root)}')
    work_reader = WorkspaceFiles(workspace, max_file_bytes, max_files)
    for path, kind in walk(work_reader):
        if kind == 'file' and path.endswith('/SKILL.md') and not path.startswith('proposed/'):
            errors.append(f'skill was written outside proposed/: {path}')
    rows, diff = changes(baseline, archive.root)
    current = WorkspaceFiles(project, max_file_bytes, max_files)
    for row in rows:
        expected = (source_digests or {}).get(row['path'], row['before_sha256'])
        try:
            item = current.read(row['path'])
            data = base64.b64decode(item['data'])
            row['source_changed'] = digest(data) != expected or item['mode'] != row['before_mode']
        except FileNotFoundError:
            row['source_changed'] = expected is not None
        except (OSError, ValueError):
            row['source_changed'] = True
    (output / 'changes.diff').write_text(diff)
    return {'changes': rows, 'validation_errors': errors, 'proposal_only': True,
            'proposed_skills': len(entries)}
