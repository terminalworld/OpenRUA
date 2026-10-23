"""Create and manage project review schedules and their shared background owner."""

import json
from pathlib import Path
from uuid import uuid4

from openrua.cli import skill_scheduler as runtime, skill_schedules as plans
from openrua.cli.commands.skill_review import selection
from openrua.cli.workspace import choose
from openrua.config import projects, paths
from openrua.config.schema import sandbox_image
from openrua.errors import UsageError


def run(args):
    root = projects.resolve(args.workspace or Path.cwd())
    try:
        if args.action == 'scheduler':
            if args.scheduler_action == 'start': result = runtime.ensure(args.home, explicit=True)
            elif args.scheduler_action == 'stop': result = runtime.stop(args.home)
            else: result = runtime.status(args.home)
        elif args.schedule_action in {None, 'list'}:
            result = {'schedules': plans.listing(args.home, root), 'scheduler': runtime.status(args.home)}
        elif args.schedule_action == 'show': result = plans.read(args.home, root, args.id)
        else:
            root = choose(root, args.home, accept=args.trust_workspace)
            if root is None: return 0
            if args.schedule_action == 'add':
                selected, adapter, settings, model = selection(root, args.home, args.agent, args.model)
                options = dict(agent=selected, model=model or '',
                               effort=args.effort if args.effort is not None else settings.get('options', {}).get('effort', adapter.default_options.get('effort', '')),
                               image=args.image or sandbox_image('jazzy'), wall_clock_min=args.wall_clock_min)
                options['agent'] = paths.agent_spec(options['agent'])
                result = plans.create(args.home, root, args.id or uuid4().hex,
                    dict(name=args.name, calendar=dict(time=args.time, days=args.days, timezone=args.timezone), options=options),
                    enabled=args.enable)
            elif args.schedule_action == 'edit':
                current = plans.read(args.home, root, args.id)
                fields = {key: current[key] for key in ('name', 'calendar', 'options')}
                if args.name is not None: fields['name'] = args.name
                for key in ('time', 'days', 'timezone'):
                    if getattr(args, key) is not None: fields['calendar'][key] = getattr(args, key)
                if args.agent is not None and paths.agent_key(args.agent) != paths.agent_key(fields['options']['agent']):
                    selected, adapter, settings, model = selection(root, args.home, args.agent, args.model)
                    fields['options'].update(agent=selected, model=model or '',
                        effort=settings.get('options', {}).get('effort', adapter.default_options.get('effort', '')))
                for key in ('agent', 'model', 'effort', 'image', 'wall_clock_min'):
                    if getattr(args, key) is not None: fields['options'][key] = getattr(args, key)
                fields['options']['agent'] = paths.agent_spec(fields['options']['agent'])
                result = plans.edit(args.home, root, args.id, fields, current['revision'])
            else: result = plans.action(args.home, root, args.id, args.schedule_action)
            if result['enabled']: result = {**result, 'scheduler': runtime.ensure(args.home)}
        if args.json: print(json.dumps(result, indent=2))
        elif 'schedules' in result:
            for row in result['schedules']:
                print(f'{row["id"]}  {"enabled" if row["enabled"] else "paused"}  {row["name"]}  next: {row["next_run"]}')
            print('Scheduler: ' + result['scheduler']['state'])
        else: print(json.dumps(result, indent=2))
        return 0
    except (OSError, ValueError, RuntimeError, KeyError) as exc:
        raise UsageError(str(exc), hint='openrua skills schedule --help') from exc


def add_parsers(actions):
    service = actions.add_parser('scheduler', help='inspect, start or stop the shared background review scheduler')
    service.add_argument('scheduler_action', choices=('status', 'start', 'stop'), nargs='?', default='status')
    service.add_argument('--json', action='store_true')
    service.set_defaults(fn=run)
    parser = actions.add_parser('schedule', help='manage this project\'s optional periodic skill reviews')
    parser.add_argument('--json', action='store_true')
    parser.set_defaults(fn=run)
    verbs = parser.add_subparsers(dest='schedule_action')
    verbs.add_parser('list', help='list schedules in this project').add_argument('--json', action='store_true')
    add = verbs.add_parser('add', help='save a plan; --enable explicitly starts background scheduling')
    add.add_argument('--id', default=None, help='stable request ID for retrying an unconfirmed creation')
    add.add_argument('--name', required=True)
    add.add_argument('--time', required=True, help='local HH:MM in 24-hour format')
    add.add_argument('--days', required=True, help='comma-separated mon,tue,wed,thu,fri,sat,sun')
    add.add_argument('--timezone', required=True, help='IANA timezone, such as UTC or Asia/Shanghai')
    add.add_argument('--agent', default=None)
    add.add_argument('--model', default=None)
    add.add_argument('--effort', default=None)
    add.add_argument('--image', default=None)
    add.add_argument('--wall-clock-min', type=float, default=5)
    add.add_argument('--enable', action='store_true')
    add.add_argument('--json', action='store_true')
    edit = verbs.add_parser('edit', help='change a saved name, calendar or review settings')
    edit.add_argument('id')
    for key in ('name', 'time', 'days', 'timezone'): edit.add_argument('--' + key)
    for key in ('agent', 'model', 'effort', 'image'): edit.add_argument('--' + key)
    edit.add_argument('--wall-clock-min', type=float, default=None)
    edit.add_argument('--json', action='store_true')
    for name in ('show', 'pause', 'resume', 'remove'):
        item = verbs.add_parser(name, help={'show': 'inspect a plan and its last task',
            'pause': 'pause future reviews; the current review continues',
            'resume': 'resume future reviews after inspecting the project and account',
            'remove': 'remove the plan; keep reports and any current review'}[name])
        item.add_argument('id')
        item.add_argument('--json', action='store_true')
