"""Project-bound skill views and explicit actions for a connected terminal."""

from __future__ import annotations

import asyncio
import base64
import hashlib
import json
import math
from pathlib import Path
import re
import tempfile

from openrua import reviews, skills
from openrua.artifacts import WorkspaceFiles
from openrua.cli import skill_tasks
from openrua.cli.commands import skill_review as review_command
from openrua.config import projects, paths
from openrua.config.schema import sandbox_image
from openrua.runner import skill_review
from openrua.runner.skills import project_directories
from openrua.errors import OpenRUAError

MAX_BYTES = 2 * 1024 * 1024


class ProjectSkills:
    def __init__(self, home: Path, project):
        self.home, self.project = home.resolve(), project

    async def action(self, body):
        return await asyncio.to_thread(self.dispatch, body)

    def root(self):
        root = self.project()
        if root is None:
            raise ValueError('Project skills are available after connecting to a robot project; use openrua skills from its folder.')
        return projects.resolve(root)

    def entries(self, root):
        entries = skills.catalog([('project', root / path) for path in
                                  dict.fromkeys((skills.PROJECT_DIRECTORY, *project_directories()))],
                                 boundary=root, max_bytes=MAX_BYTES)
        personal = self.home / 'skills'
        entries += skills.catalog([('personal', personal)], boundary=personal, max_bytes=MAX_BYTES)
        return {hashlib.sha256(f'{entry.scope}:{entry.path}'.encode()).hexdigest(): entry for entry in entries}

    def defaults(self, root, agent=None):
        selected, adapter, settings, model = review_command.selection(root, self.home, agent)
        effort = settings.get('options', {}).get('effort', adapter.default_options.get('effort', ''))
        return dict(defaults=dict(agent=selected, model=model or '', effort=effort or '',
                                  image=sandbox_image('jazzy'), wall_clock_min=5),
                    agent_identity=paths.agent_key(selected))

    def report(self, root, identity):
        if not isinstance(identity, str) or not re.fullmatch('[0-9a-f]{32}', identity):
            raise ValueError('invalid review ID')
        report = next((row for row in skill_review.reports(root) if row['id'] == identity), None)
        if report is None: raise ValueError('review not found in this project')
        return report, root / '.openrua' / 'skill-reviews' / identity

    def candidates(self, root, report, output):
        proposed = output / 'proposed'
        if not proposed.is_dir() or proposed.is_symlink(): return []
        reader = WorkspaceFiles(root, MAX_BYTES, 2000)
        prefix = proposed.relative_to(root).as_posix()
        try:
            manifest = json.loads(reader.read((output / 'manifest.json').relative_to(root).as_posix())['text'])
            originals = {row['path'].removeprefix('project/'): row.get('source_sha256', row['sha256'])
                         for row in manifest.get('files', []) if row.get('path', '').startswith('project/')}
        except FileNotFoundError:
            originals = {}
        found = []
        for absolute, kind in reviews.walk(reader, prefix):
            path = Path(absolute).relative_to(prefix).as_posix()
            if kind != 'file' or not path.endswith('/SKILL.md'): continue
            directory = proposed / Path(path).parent
            entries = skills.catalog([('proposal', directory.parent)], boundary=proposed, max_bytes=MAX_BYTES)
            entry = next((entry for entry in entries if Path(entry.path).parent == directory), None)
            if entry is None: continue
            rows = [row for row in report.get('changes', []) if isinstance(row.get('path'), str)
                    and Path(row['path']).is_relative_to(Path(path).parent)]
            if not rows: continue
            files = self.candidate_files(root, output, path)
            fingerprint = self.fingerprint(files)
            existing = [value for value in self.entries(root).values() if value.scope == 'project'
                        and (value.name == entry.name or Path(value.path).parent.name == directory.name)]
            source_changed = False
            for row in rows:
                expected = row.get('source_sha256', originals.get(row['path'], row.get('before_sha256')))
                try:
                    current = WorkspaceFiles(root, MAX_BYTES).read(row['path'])
                    changed = reviews.digest(base64.b64decode(current['data'])) != expected or current['mode'] != row.get('before_mode')
                except FileNotFoundError: changed = expected is not None
                except (OSError, ValueError): changed = True
                source_changed |= changed
            found.append(dict(key=path, name=entry.name, description=entry.description, error=entry.error,
                              fingerprint=fingerprint, existing=bool(existing), source_changed=source_changed,
                              can_import=report['status'] == 'completed' and not report.get('validation_errors')
                              and not entry.error and not existing))
        return found

    @staticmethod
    def candidate_files(root, output, path):
        relative = Path(path)
        if relative.is_absolute() or '..' in relative.parts or relative.name != 'SKILL.md' or len(relative.parts) < 3:
            raise ValueError('invalid candidate path')
        reader = WorkspaceFiles(root, MAX_BYTES, 2000)
        prefix = (output / 'proposed' / relative.parent).relative_to(root).as_posix()
        result = {}
        total = 0
        for item, kind in reviews.walk(reader, prefix):
            if kind != 'file': raise ValueError('candidate must contain only ordinary files')
            value = reader.read(item)
            total += value['size']
            if total > 32 * 1024 * 1024: raise ValueError('candidate exceeds the import size limit')
            result[Path(item).relative_to(prefix).as_posix()] = value
        return result

    @staticmethod
    def fingerprint(files):
        return reviews.digest(json.dumps(sorted((name, reviews.digest(base64.b64decode(value['data'])), value['mode'])
                                               for name, value in files.items())).encode())

    def dispatch(self, body):
        if not isinstance(body, dict) or not isinstance(body.get('operation'), str):
            raise ValueError('provide a project skills operation')
        op = body['operation']
        expected = {'overview': set(), 'skill': {'key'}, 'report': {'id'},
                    'start': {'request_id', 'options'}, 'task': {'id'},
                    'import': {'id', 'key', 'fingerprint'}, 'merge': {'id', 'key'},
                    'policy': {'mode'}, 'candidate': {'id', 'key'}, 'cancel': {'id'}, 'defaults': {'agent'}}
        if op not in expected or set(body) != expected[op] | {'operation'}:
            raise ValueError('incorrect project skills arguments')
        root = self.root()
        if op == 'defaults':
            if not isinstance(body['agent'], str) or not body['agent'].strip(): raise ValueError('provide an agent name')
            return self.defaults(root, body['agent'])
        if op == 'overview':
            try:
                selected = self.defaults(root)
                defaults = selected['defaults']
                review_error = None
            except (OSError, ValueError, RuntimeError, OpenRUAError) as exc:
                defaults, review_error = None, str(exc)
            return dict(project=str(root), entries=[dict(key=key, **entry.as_dict()) for key, entry in self.entries(root).items()],
                        policy=skills.mode(root), reports=[{key: value.get(key) for key in
                            ('id', 'status', 'started_at', 'agent', 'model', 'proposed_skills')} for value in skill_review.reports(root)[:20]],
                        tasks=skill_tasks.listing(self.home, root),
                        defaults=defaults, agent_identity=selected.get('agent_identity') if defaults else None, review_error=review_error)
        if op == 'skill':
            entry = self.entries(root).get(body['key'])
            if entry is None: raise ValueError('skill no longer exists; refresh the list')
            path = Path(entry.path).resolve()
            boundary = root if entry.scope == 'project' else (self.home / 'skills').resolve()
            return WorkspaceFiles(boundary, MAX_BYTES).read(path.relative_to(boundary).as_posix())
        if op == 'task': return skill_tasks.read(self.home, root, body['id'])
        if op == 'cancel': return skill_tasks.cancel(self.home, root, body['id'])
        if op in {'start', 'import', 'policy'} and not projects.trusted(root, self.home):
            raise ValueError('trust this project before starting a review or changing skills')
        if op == 'policy':
            skills.set_mode(root, body['mode'])
            return dict(policy=skills.mode(root))
        if op == 'start':
            options = body['options']
            if not isinstance(options, dict) or set(options) not in ({'agent', 'model', 'image', 'wall_clock_min'}, {'agent', 'model', 'effort', 'image', 'wall_clock_min'}):
                raise ValueError('provide agent, model, image and wall_clock_min')
            if 'effort' in options and (not isinstance(options['effort'], str) or len(options['effort']) > 128):
                raise ValueError('effort must be a native level or an empty default')
            if not all(isinstance(options[key], str) and len(options[key]) <= 4096 for key in ('agent', 'model', 'image')) or not all(options[key].strip() for key in ('agent', 'image')):
                raise ValueError('provide an agent, image and optional model')
            seconds = options['wall_clock_min']
            if type(seconds) not in (int, float) or not math.isfinite(seconds) or seconds <= 0:
                raise ValueError('review time limit must be finite and positive')
            return skill_tasks.start(self.home, root, body['request_id'], options)
        report, output = self.report(root, body['id'])
        candidates = self.candidates(root, report, output)
        if op == 'report':
            reader = WorkspaceFiles(root, MAX_BYTES)
            def text(name):
                try: return reader.read((output / name).relative_to(root).as_posix()).get('text', '')
                except FileNotFoundError: return ''
            return dict(report={key: report.get(key) for key in ('id', 'status', 'agent', 'model', 'effort', 'sources', 'reason', 'error', 'validation_errors')},
                        summary=text('report.md'), diff=text('changes.diff'), candidates=candidates)
        candidate = next((row for row in candidates if row['key'] == body['key']), None)
        if candidate is None: raise ValueError('candidate not found in this review')
        if op == 'candidate':
            return self.candidate_files(root, output, candidate['key'])['SKILL.md']
        if op == 'merge':
            relative = output.relative_to(root).as_posix()
            return dict(draft='Review this selected skill proposal and merge only the changes that still fit this project.\n'
                        f'Selected candidate: /workspace/{relative}/proposed/{candidate["key"]}\n'
                        f'Baseline: /workspace/{relative}/baseline/\n'
                        f'Diff and evidence: /workspace/{relative}/changes.diff and report.md\n'
                        'Read the current skill and compare it with the baseline and proposal. Preserve newer project edits; '
                        'if changes conflict, explain the conflict before editing. Use the native editing and approval workflow. '
                        'This request concerns skill files only; do not move the robot to validate them.')
        if not candidate['can_import']: raise ValueError('this candidate cannot be imported; inspect the report and merge existing skills with the agent')
        files = self.candidate_files(root, output, candidate['key'])
        if self.fingerprint(files) != body['fingerprint']: raise ValueError('candidate changed after preview; refresh and review it again')
        destination = root / skills.PROJECT_DIRECTORY
        if not destination.resolve().is_relative_to(root):
            raise ValueError('project skill destination must remain inside the project')
        # Copy the bytes just inspected to a private snapshot before importing.
        with tempfile.TemporaryDirectory(prefix='skill-import-') as directory:
            source = Path(directory) / Path(candidate['key']).parent.name
            source.mkdir()
            for name, value in files.items():
                target = source / name
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_bytes(base64.b64decode(value['data']))
                target.chmod(value['mode'] & 0o777)
            target = skills.install(source, destination)
        try: issues = skills.bridge(root, project_directories())
        except (OSError, ValueError) as exc: issues = [f'Skill saved; native directory setup needs attention: {exc}']
        return dict(imported=target.relative_to(root).as_posix(), name=candidate['name'], issues=issues)
