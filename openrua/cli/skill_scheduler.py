"""Run one application-owned review scheduler per OpenRUA user directory."""

import argparse
from datetime import datetime, timezone
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
from uuid import uuid4

from apscheduler.events import EVENT_JOB_ERROR, EVENT_JOB_EXECUTED, EVENT_JOB_MISSED
from apscheduler.executors.pool import ThreadPoolExecutor
from apscheduler.schedulers.background import BackgroundScheduler

from openrua import __version__, reviews
from openrua.artifacts import WorkspaceFiles
from openrua.cli import skill_schedules as plans, skill_tasks


def directory(home):
    root = home / 'skill-scheduler'
    if root.is_symlink(): raise ValueError('scheduler directory must not be a symlink')
    return root


def status(home):
    root = directory(home)
    running = False
    lock = root / 'owner.lock'
    if lock.is_symlink(): raise ValueError('scheduler lock must be an ordinary file')
    instance = None
    if lock.is_file():
        with lock.open() as stream:
            try: fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError: running = True
            instance = stream.read().strip()
    try: value = json.loads(WorkspaceFiles(root, 65536).read('status.json')['text'])
    except FileNotFoundError: value = {}
    if instance != value.get('instance'): value = {}
    return {**value, 'running': running, 'state': 'stopping' if running and ((root / 'stop').exists() or value.get('state') == 'stopping')
            else 'running' if running and value else 'starting' if running else 'stopped', 'stop_requested': (root / 'stop').exists()}


def ensure(home, *, explicit=False):
    """Start only for enabled plans. Ordinary startup respects an explicit stop."""
    home = home.resolve()
    root = directory(home)
    current = status(home)
    if current['running'] and current['state'] != 'starting':
        if explicit and current['stop_requested']:
            raise RuntimeError('scheduler is stopping; wait for its current review, then start it again')
        return current
    if (root / 'stop').exists() and not explicit: return current
    if not any(row['enabled'] or row.get('active_task') for row in plans.listing(home)): return current
    root.mkdir(parents=True, exist_ok=True, mode=0o700)
    if explicit: (root / 'stop').unlink(missing_ok=True)
    process = None
    if not current['running']:
        with (root / 'service.log').open('ab') as log:
            os.chmod(log.name, 0o600)
            process = subprocess.Popen([sys.executable, '-m', 'openrua.cli.skill_scheduler', '--home', str(home)],
                                       stdin=subprocess.DEVNULL, stdout=log, stderr=log,
                                       cwd=root, start_new_session=True)
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        current = status(home)
        if current['running'] and current['state'] != 'starting': return current
        if process is not None and process.poll() is not None and not current['running']: break
        time.sleep(.02)
    raise RuntimeError(f'scheduler startup was not confirmed; inspect {root / "service.log"} and scheduler status before retrying')


def autostart(home):
    """A scheduler fault is visible without blocking the robot conversation."""
    try: return ensure(home)
    except (OSError, ValueError, RuntimeError) as exc:
        print(f'[skills] Could not restore background scheduling: {exc}', file=sys.stderr, flush=True)


def stop(home):
    root = directory(home)
    root.mkdir(parents=True, exist_ok=True, mode=0o700)
    (root / 'stop').touch(mode=0o600, exist_ok=True)
    return status(home)


def execute(home, identity, revision, due, active, stopped=None):
    """Adopt existing work or claim one occurrence, never replay a crashed task."""
    task_id = active
    try:
        if active:
            value = plans.load(home, identity)
            if value.get('active_task') != active: return
        else:
            if (directory(home) / 'stop').exists() or (stopped and stopped()): return
            value = plans.claim(home, identity, revision, due)
            if value is None: return
            task_id = value['active_task']
            skill_tasks.start(home, Path(value['project']), task_id, value['options'])
        while True:
            task = skill_tasks.read(home, Path(value['project']), task_id)
            if task['status'] not in plans.ACTIVE: break
            time.sleep(.25)
        plans.finish(home, identity, task)
    except Exception as exc:
        if task_id:
            plans.finish(home, identity, dict(id=task_id, status='unknown', error=str(exc)))
        print(f'[skills] Schedule {identity}: {exc}', file=sys.stderr, flush=True)


def serve(home, *, poll_seconds=.5, stopped=None):
    root = directory(home)
    root.mkdir(parents=True, exist_ok=True, mode=0o700)
    lock_path = root / 'owner.lock'
    if lock_path.is_symlink(): raise ValueError('scheduler lock must be an ordinary file')
    with lock_path.open('a') as owner:
        try: fcntl.flock(owner, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError: return 0
        if (root / 'stop').exists(): return 0
        instance = uuid4().hex
        owner.seek(0); owner.truncate(); owner.write(instance); owner.flush()
        scheduler = BackgroundScheduler(timezone=timezone.utc,
                                        executors={'default': ThreadPoolExecutor(max_workers=1)})
        pending, guard, closing = {}, threading.RLock(), threading.Event()
        def finished(event):
            with guard: pending.pop(event.job_id, None)
        scheduler.add_listener(finished, EVENT_JOB_EXECUTED | EVENT_JOB_ERROR | EVENT_JOB_MISSED)
        fact = dict(version=__version__, instance=instance, pid=os.getpid(), started_at=plans.now().isoformat())
        scheduler.start()
        reviews.write_json(root / 'status.json', {**fact, 'state': 'running'})
        try:
            while not (root / 'stop').exists() and not (stopped and stopped()):
                rows = [row for row in plans.listing(home) if row['enabled'] or row.get('active_task')]
                if not rows: break
                desired = {row['id']: row for row in rows}
                with guard:
                    for identity in list(pending):
                        row = desired.get(identity)
                        signature = (row['revision'], row['next_run'], row.get('active_task')) if row else None
                        if signature != pending[identity] and scheduler.get_job(identity) is not None:
                            scheduler.remove_job(identity)
                            pending.pop(identity)
                    for identity, row in desired.items():
                        if identity in pending: continue
                        signature = (row['revision'], row['next_run'], row.get('active_task'))
                        pending[identity] = signature
                        scheduler.add_job(execute, 'date', id=identity,
                                          run_date=plans.now() if row.get('active_task') else datetime.fromisoformat(row['next_run']),
                                          args=(home, identity, *signature, closing.is_set), misfire_grace_time=None, max_instances=1)
                time.sleep(poll_seconds)
        finally:
            # A stop prevents future runs and lets the bounded current review finish.
            closing.set()
            scheduler.pause()
            reviews.write_json(root / 'status.json', {**fact, 'state': 'stopping'})
            scheduler.shutdown(wait=True)
            reviews.write_json(root / 'status.json', {**fact, 'state': 'stopped'})
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--home', type=Path, required=True)
    args = parser.parse_args()
    stopped = threading.Event()
    for sig in (signal.SIGTERM, signal.SIGINT):
        signal.signal(sig, lambda *_: stopped.set())
    return serve(args.home.resolve(), stopped=stopped.is_set)


if __name__ == '__main__':
    raise SystemExit(main())
