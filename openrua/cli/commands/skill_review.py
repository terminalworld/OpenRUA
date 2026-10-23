"""The manual review entry point and project-scoped retained reports."""

from datetime import datetime, timezone
import json
import math
from pathlib import Path
import re
import sys

from openrua.cli import agent_settings
from openrua.artifacts import WorkspaceFiles
from openrua.cli.workspace import choose
from openrua.config import paths, projects
from openrua.config.schema import sandbox_image
from openrua.errors import UsageError
from openrua.runner import skill_review


def run(args) -> int:
    root = projects.resolve(args.workspace or Path.cwd())
    try:
        if args.action == 'reviews':
            if args.id and not re.fullmatch('[0-9a-f]{32}', args.id):
                raise ValueError('review ID must be the 32-character ID shown by openrua skills reviews')
            values = skill_review.reports(root)
            if args.id:
                values = [value for value in values if value['id'] == args.id]
                if not values:
                    raise ValueError('no review with that ID in this project; use openrua skills reviews')
            if args.input:
                if not args.id:
                    raise ValueError('--input requires a review ID')
                item = WorkspaceFiles(args.home / 'skill-reviews' / args.id / 'input').read(args.input)
                if args.json:
                    print(json.dumps(item, indent=2))
                elif item['kind'] == 'text':
                    print(item['text'])
                else:
                    raise ValueError('selected input is binary; use --json or inspect its retained file')
            elif args.json:
                print(json.dumps(values[0] if args.id else values, indent=2))
            elif args.id:
                print(WorkspaceFiles(root).read(f'.openrua/skill-reviews/{args.id}/report.md')['text'])
            else:
                for value in values:
                    print(f'{value["id"]}  {value["status"]}  {value.get("agent", "")}  {value["started_at"]}')
                if not values:
                    print('No skill reviews in this project. Use openrua skills review.')
            return 0
        if not math.isfinite(args.wall_clock_min) or args.wall_clock_min <= 0:
            raise ValueError('--wall-clock-min must be finite and positive')
        if not all(math.isfinite(value) and value > 0 for value in (args.max_input_mb, args.max_file_mb)):
            raise ValueError('input size limits must be finite and positive')
        if min(args.max_records, args.max_files, args.max_events, args.max_turns,
               int(args.max_input_mb * 1024 * 1024), int(args.max_file_mb * 1024 * 1024)) <= 0:
            raise ValueError('review limits must be positive')
        extra = tuple(Path(path).expanduser().resolve() for path in args.records)
        for path in extra:
            if not path.is_dir():
                raise ValueError(f'records directory does not exist: {path}')
        since = None
        if args.since:
            value = datetime.fromisoformat(args.since.replace('Z', '+00:00'))
            since = (value if value.tzinfo else value.replace(tzinfo=timezone.utc)).timestamp()
        root = choose(root, args.home, accept=args.trust_workspace)
        if root is None:
            return 0
        saved = projects.selection(root)
        requested = args.agent or saved.get('agent')
        adapter, settings = agent_settings.resolve(args.home, requested)
        model = args.model or agent_settings.project_model(saved, args.agent) or settings.get('model') or adapter.default_model
        print(f'[skills] Reviewing completed work in {root} with {adapter.name}.', file=sys.stderr, flush=True)
        result = skill_review.run(root, args.home, adapter, settings,
                                  image=args.image or sandbox_image('jazzy'), model=model, since=since,
                                  max_records=args.max_records, max_bytes=int(args.max_input_mb * 1024 * 1024),
                                  max_file_bytes=int(args.max_file_mb * 1024 * 1024), max_files=args.max_files,
                                  max_events=args.max_events, seconds=args.wall_clock_min * 60,
                                  max_turns=args.max_turns, extra_records=extra, force=args.force)
        if args.json:
            print(json.dumps(result, indent=2))
        else:
            print(f'{result["status"]}: {len(result["sources"])} completed records, '
                  f'{len(result.get("changes", []))} proposed file changes.\n{result["output"]}/report.md')
            if result.get('error') or result.get('reason'):
                print(result.get('error') or result['reason'])
        return 0 if result['status'] in {'completed', 'skipped'} else 130 if result['status'] == 'interrupted' else 1
    except (ValueError, OSError) as exc:
        raise UsageError(str(exc), hint='openrua skills review --help') from exc


def add_parsers(actions) -> None:
    review = actions.add_parser('review', help='review completed project records in isolation and save proposals')
    review.add_argument('--agent', default=None, help='agent name or manifest; default: project or user setting')
    review.add_argument('--model', default=None, help='model; default: project or selected agent setting')
    review.add_argument('--image', default=None, help='existing agent image; default: standard Jazzy sandbox')
    review.add_argument('--since', default=None, help='ISO date/time, UTC unless an offset is given')
    review.add_argument('--records', action='append', default=[], metavar='DIR', help='additional exec record root; ownership is still checked')
    review.add_argument('--max-records', type=int, default=20, help='newest eligible completed records to inspect')
    review.add_argument('--max-input-mb', type=float, default=32, help='total selected input size in MiB')
    review.add_argument('--max-file-mb', type=float, default=2, help='per-file or conversation export limit in MiB')
    review.add_argument('--max-files', type=int, default=2000, help='input and proposal file-count limit')
    review.add_argument('--max-events', type=int, default=10000, help='recent events per conversation; omitted events are reported')
    review.add_argument('--wall-clock-min', type=float, default=5, help='worker execution time cap in minutes')
    review.add_argument('--max-turns', type=int, default=100, help='requested native turn cap; adapters may not enforce it')
    review.add_argument('--force', action='store_true', help='review again even if identical inputs already have a completed review')
    review.add_argument('--json', action='store_true')
    review.set_defaults(fn=run)
    reports = actions.add_parser('reviews', help='list this project\'s review reports or read one')
    reports.add_argument('id', nargs='?', help='review ID from the list')
    reports.add_argument('--json', action='store_true')
    reports.add_argument('--input', default=None, metavar='PATH', help='read a relative path from this review\'s private input snapshot')
    reports.set_defaults(fn=run)
