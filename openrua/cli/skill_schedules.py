"""Project-scoped review plans, calendar rules and durable execution claims."""

from contextlib import contextmanager
from datetime import datetime, timedelta, timezone
import fcntl
import json
from pathlib import Path
import re
from uuid import NAMESPACE_URL, uuid5
from zoneinfo import ZoneInfo

from apscheduler.triggers.cron import CronTrigger

from openrua import reviews
from openrua.artifacts import FileChangedError, WorkspaceFiles
from openrua.cli import skill_tasks
from openrua.config import projects, paths

ACTIVE = {'starting', 'running', 'cancel_requested'}
ATTENTION = {'quota_limited', 'authentication_required', 'unknown'}
DAYS = ('mon', 'tue', 'wed', 'thu', 'fri', 'sat', 'sun')


def now():
    return datetime.now(timezone.utc)


def directory(home, identity):
    if not isinstance(identity, str) or not re.fullmatch('[0-9a-f]{32}', identity):
        raise ValueError('schedule ID must be 32 hexadecimal characters')
    root = home / 'skill-schedules'
    if root.is_symlink() or (root / identity).is_symlink():
        raise ValueError('schedule directories must be ordinary directories')
    return root / identity


def load(home, identity):
    source = WorkspaceFiles(directory(home, identity), 1024 * 1024)
    for attempt in range(3):
        try:
            value = json.loads(source.read('schedule.json')['text'])
            break
        except FileChangedError:
            if attempt == 2:
                raise
    if value['home'] != str(home.resolve()) or value['id'] != identity:
        raise ValueError('schedule belongs to another user directory')
    return value


def read(home, project, identity):
    value = load(home, identity)
    if value['project'] != str(project.resolve()):
        raise ValueError('schedule belongs to another project')
    return value


def listing(home, project=None):
    root = home / 'skill-schedules'
    if not root.exists(): return []
    if root.is_symlink(): raise ValueError('schedule directory must not be a symlink')
    result = []
    for path in sorted(root.iterdir()):
        if not re.fullmatch('[0-9a-f]{32}', path.name): continue
        value = load(home, path.name)
        if not value.get('deleted') and (project is None or value['project'] == str(project.resolve())):
            result.append(value)
    return sorted(result, key=lambda row: row['created_at'], reverse=True)


@contextmanager
def locked(home, identity):
    root = directory(home, identity)
    lock_path = root / 'config.lock'
    if lock_path.is_symlink(): raise ValueError('schedule lock must be an ordinary file')
    with lock_path.open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        yield load(home, identity)


def write(home, value):
    reviews.write_json(directory(home, value['id']) / 'schedule.json', value)


def trigger(calendar):
    if not isinstance(calendar, dict) or set(calendar) != {'time', 'days', 'timezone'}:
        raise ValueError('calendar requires time, days and timezone')
    clock = calendar['time']
    if not isinstance(clock, str) or not re.fullmatch(r'(?:[01]\d|2[0-3]):[0-5]\d', clock):
        raise ValueError('time must be HH:MM in 24-hour format')
    days = calendar['days']
    if not isinstance(days, str) or not days or any(day not in DAYS for day in days.split(',')):
        raise ValueError('days must be comma-separated mon,tue,wed,thu,fri,sat,sun')
    zone = calendar['timezone']
    if not isinstance(zone, str) or len(zone) > 128:
        raise ValueError('provide an IANA timezone, such as UTC or Asia/Shanghai')
    hour, minute = map(int, clock.split(':'))
    try: zone = ZoneInfo(zone)
    except (KeyError, ValueError) as exc: raise ValueError('unknown IANA timezone') from exc
    return CronTrigger(hour=hour, minute=minute, day_of_week=days, timezone=zone)


def following(calendar, at=None):
    value = trigger(calendar).get_next_fire_time(None, (at or now()) + timedelta(microseconds=1))
    return value.astimezone(timezone.utc).isoformat()


def definition(value, project):
    if not isinstance(value, dict) or set(value) != {'name', 'calendar', 'options'}:
        raise ValueError('provide schedule name, calendar and review options')
    if not isinstance(value['name'], str) or not value['name'].strip() or len(value['name']) > 160:
        raise ValueError('schedule name must contain 1..160 characters')
    trigger(value['calendar'])
    options = skill_tasks.validate_options(value['options'])
    options['agent'] = paths.agent_spec(options['agent'], project)
    return dict(name=value['name'].strip(), calendar=dict(value['calendar']), options=options)


def create(home, project, identity, value, *, enabled=False):
    home, project = home.resolve(), project.resolve()
    if home.is_relative_to(project): raise ValueError('schedule state must be outside the project')
    if not projects.trusted(project, home): raise ValueError('trust this project before scheduling reviews')
    fields = definition(value, project)
    root = directory(home, identity)
    root.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
    try: root.mkdir(mode=0o700)
    except FileExistsError:
        previous = read(home, project, identity)
        if any(previous[key] != val for key, val in fields.items()):
            raise ValueError('schedule ID already has a different definition')
        return previous
    value = dict(id=identity, project=str(project), home=str(home), **fields, enabled=enabled,
                 revision=1, created_at=now().isoformat(), next_run=following(fields['calendar']),
                 active_task=None, last_result=None, reason=None)
    write(home, value)
    return value


def edit(home, project, identity, value, revision):
    fields = definition(value, project)
    with locked(home, identity) as current:
        read(home, project, identity)
        if current['revision'] != revision: raise ValueError('schedule changed; refresh before editing')
        if current.get('deleted'): raise ValueError('schedule was removed')
        current.update(fields, revision=current['revision'] + 1, next_run=following(fields['calendar']))
        write(home, current)
        return current


def action(home, project, identity, action):
    if action not in {'pause', 'resume', 'remove'}: raise ValueError('choose pause, resume or remove')
    with locked(home, identity) as current:
        read(home, project, identity)
        if current.get('deleted'): raise ValueError('schedule was removed')
        if action == 'resume' and not projects.trusted(project, home):
            raise ValueError('trust this project before resuming reviews')
        current.update(enabled=action == 'resume', revision=current['revision'] + 1,
                       reason=None if action == 'resume' else 'Paused by the user.')
        if action == 'resume': current['next_run'] = following(current['calendar'])
        if action == 'remove': current.update(deleted=True, reason='Removed by the user.')
        write(home, current)
        return current


def claim(home, identity, revision, due):
    with locked(home, identity) as current:
        if not current['enabled'] or current.get('deleted') or current.get('active_task') \
                or current['revision'] != revision or current['next_run'] != due:
            return None
        project = Path(current['project'])
        if not project.is_dir() or not projects.trusted(project, home):
            current.update(enabled=False, reason='Project is unavailable or no longer trusted; inspect it before resuming.')
            write(home, current)
            return None
        if datetime.fromisoformat(due) > now(): return None
        # Claim before starting a task. A crash never replays an uncertain attempt.
        task = uuid5(NAMESPACE_URL, f"{identity}:{revision}:{due}").hex
        current.update(active_task=task, active_revision=revision, last_due=due,
                       last_started_at=now().isoformat(), next_run=following(current['calendar']))
        write(home, current)
        return current


def finish(home, identity, task):
    if task['status'] in ACTIVE: return
    with locked(home, identity) as current:
        if current.get('active_task') != task['id']: return
        result = {key: task[key] for key in ('id', 'status', 'review_id', 'reason', 'error') if key in task}
        current.update(last_result={**result, 'finished_at': now().isoformat()}, active_task=None)
        if task['status'] in ATTENTION and current['revision'] == current.get('active_revision'):
            current.update(enabled=False, reason=f"Review {task['status']}; inspect the task and resume explicitly.")
        write(home, current)
