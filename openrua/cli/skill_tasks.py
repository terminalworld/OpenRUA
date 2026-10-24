"""Run and inspect one explicit skill review independently of the chat owner."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import fcntl
import hashlib
import math
import os
from pathlib import Path
import re
import subprocess
import sys

from openrua import reviews
from openrua.artifacts import WorkspaceFiles


def validate_options(options: dict) -> dict:
    if not isinstance(options, dict) or set(options) not in ({'agent', 'model', 'image', 'wall_clock_min'}, {'agent', 'model', 'effort', 'image', 'wall_clock_min'}):
        raise ValueError('provide agent, model, image and wall_clock_min')
    if 'effort' in options and (not isinstance(options['effort'], str) or len(options['effort']) > 128):
        raise ValueError('effort must be a native level or an empty default')
    if not all(isinstance(options[key], str) and len(options[key]) <= 4096 for key in ('agent', 'model', 'image')) or not all(options[key].strip() for key in ('agent', 'image')):
        raise ValueError('provide an agent, image and optional model')
    minutes = options['wall_clock_min']
    if type(minutes) not in (int, float) or not math.isfinite(minutes) or minutes <= 0:
        raise ValueError('review time limit must be finite and positive')
    return dict(options)


def location(home: Path, identity: str) -> Path:
    if not isinstance(identity, str) or not re.fullmatch('[0-9a-f]{32}', identity):
        raise ValueError('task ID must be 32 hexadecimal characters')
    root = home / 'skill-tasks'
    if root.is_symlink() or (root / identity).is_symlink():
        raise ValueError('skill task directories must be ordinary directories')
    return root / identity


def read(home: Path, project: Path, identity: str) -> dict:
    root = location(home, identity)
    reader = WorkspaceFiles(root, max_bytes=1024 * 1024)
    request = reader.read_json('request.json')
    if request.get('project') != str(project.resolve()) or request.get('home') != str(home.resolve()):
        raise ValueError('this task belongs to another project or user directory')
    result = {key: request[key] for key in ('id', 'project', 'started_at', 'options')}
    try:
        result.update(reader.read_json('status.json'))
    except FileNotFoundError:
        result['status'] = 'starting'
    if result['status'] in {'running', 'starting'}:
        lock_path = root / 'owner.lock'
        if lock_path.is_symlink():
            raise ValueError('task lock must be an ordinary file')
        if lock_path.is_file() and result['status'] == 'running':
            with lock_path.open() as lock:
                try: fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                except BlockingIOError: pass
                else: result.update(status='unknown', reason='Task owner exited without a final result; it was not restarted.')
        elif (datetime.now(timezone.utc) - datetime.fromisoformat(request['started_at'])).total_seconds() > 15:
            result.update(status='unknown', reason='Task startup was not confirmed; it was not restarted.')
    if result['status'] in {'running', 'starting'} and (root / 'cancel').exists():
        result['status'] = 'cancel_requested'
    return result


def listing(home: Path, project: Path) -> list[dict]:
    root = home / 'skill-tasks'
    if not root.exists(): return []
    if root.is_symlink(): raise ValueError('skill task directory must not be a symlink')
    results = []
    for directory in sorted(root.iterdir(), key=lambda path: path.name):
        if not re.fullmatch('[0-9a-f]{32}', directory.name): continue
        try: results.append(read(home, project, directory.name))
        except (OSError, ValueError, KeyError, TypeError): continue
    return sorted(results, key=lambda row: row['started_at'], reverse=True)[:20]


def start(home: Path, project: Path, identity: str, options: dict) -> dict:
    options = validate_options(options)
    home, project = home.resolve(), project.resolve()
    if home.is_relative_to(project): raise ValueError('review state must be outside the project')
    root = location(home, identity)
    root.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
    try: root.mkdir(mode=0o700)
    except FileExistsError:
        previous = read(home, project, identity)
        if previous['options'] != options: raise ValueError('task ID was already used with other options')
        return previous
    request = dict(id=identity, home=str(home), project=str(project), options=options,
                   started_at=datetime.now(timezone.utc).isoformat())
    reviews.write_json(root / 'request.json', request)
    with (root / 'process.log').open('xb') as log:
        os.chmod(log.name, 0o600)
        try:
            subprocess.Popen([sys.executable, '-m', 'openrua.cli.skill_tasks', '--home', str(home),
                              '--project', str(project), '--id', identity], stdin=subprocess.DEVNULL,
                             stdout=log, stderr=log, start_new_session=True, cwd=project)
        except Exception as exc:
            reviews.write_json(root / 'status.json', dict(status='failed', error=str(exc)))
            raise
    return read(home, project, identity)


def cancel(home: Path, project: Path, identity: str) -> dict:
    result = read(home, project, identity)
    if result['status'] not in {'running', 'starting'}: return result
    root = location(home, identity)
    (root / 'cancel').touch(exist_ok=True)
    return dict(id=identity, status='cancel_requested')


def run(home: Path, project: Path, identity: str) -> int:
    try: return _run(home, project, identity)
    except KeyboardInterrupt:
        reviews.write_json(location(home, identity) / 'status.json', dict(status='interrupted', reason='Review canceled.'))
        return 130


def _run(home: Path, project: Path, identity: str) -> int:
    from openrua.cli import build_parser
    from openrua.cli.commands import skill_review
    root = location(home, identity)
    request = read(home, project, identity)
    with (root / 'owner.lock').open('a') as owner:
        fcntl.flock(owner, fcntl.LOCK_EX | fcntl.LOCK_NB)
        if (root / 'status.json').exists():
            # A persisted attempt is never re-executed, even after a crash.
            return 0 if request['status'] in {'completed', 'skipped'} else 1
        if (root / 'cancel').exists():
            reviews.write_json(root / 'status.json', dict(status='interrupted', reason='Review canceled before starting.'))
            return 130
        # The project lock prevents two clients from spending on simultaneous reviews.
        project_key = hashlib.sha256(str(project.resolve()).encode()).hexdigest()
        with (root.parent / f'{project_key}.lock').open('a') as held:
            try: fcntl.flock(held, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                reviews.write_json(root / 'status.json', dict(status='busy', reason='Another review is running for this project.'))
                return 1
            reviews.write_json(root / 'status.json', dict(status='running'))
            options = request['options']
            argv = ['--home', str(home), 'skills', '-C', str(project), '--trust-workspace', 'review', '--json']
            for name in ('agent', 'model', 'effort', 'image', 'wall_clock_min'):
                if options.get(name) not in (None, ''):
                    argv.append('--' + name.replace('_', '-') + '=' + str(options[name]))
            def completed(report):
                reviews.write_json(root / 'status.json', {
                    'status': report['status'], 'review_id': report['id'],
                    **{key: report[key] for key in ('reason', 'error') if key in report}})
            try:
                if (root / 'cancel').exists(): raise KeyboardInterrupt
                return skill_review.run(build_parser().parse_args(argv), on_result=completed,
                                        cancelled=lambda: (root / 'cancel').exists())
            except Exception as exc:
                reviews.write_json(root / 'status.json', dict(status='failed', error=str(exc)))
                return 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--home', type=Path, required=True)
    parser.add_argument('--project', type=Path, required=True)
    parser.add_argument('--id', required=True)
    args = parser.parse_args()
    return run(args.home.resolve(), args.project.resolve(), args.id)


if __name__ == '__main__':
    raise SystemExit(main())
