"""Inspect ordinary skills and choose how a robot project accumulates methods."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from openrua import skills
from openrua.cli.workspace import choose
from openrua.config import projects
from openrua.errors import UsageError
from openrua.runner.skills import project_directories


def run(args) -> int:
    root = projects.resolve(args.workspace or Path.cwd())
    action = args.action or 'list'
    personal = getattr(args, 'personal', False)
    mutates = action == 'add' or (action == 'policy' and args.mode is not None)
    try:
        if mutates and not personal:
            root = choose(root, args.home, accept=args.trust_workspace)
            if root is None:
                return 0
        roots = [('project', root / directory) for directory in
                 dict.fromkeys((skills.PROJECT_DIRECTORY, *project_directories()))]
        if personal:
            roots = []
        roots.append(('personal', args.home / 'skills'))
        if action == 'policy':
            if args.mode is not None:
                skills.set_mode(root, args.mode)
            print(skills.mode(root))
        elif action == 'add':
            if not personal:
                skills.bridge(root, project_directories())
            destination = args.home / 'skills' if personal else root / skills.PROJECT_DIRECTORY
            print(skills.install(Path(args.source), destination))
        else:
            entries = skills.catalog(roots)
            if action == 'show':
                matches = [entry for entry in entries if entry.name == args.name]
                if len(matches) != 1:
                    raise ValueError(f'expected one skill named {args.name!r}, found {len(matches)}; use skills list --json to inspect paths')
                print(Path(matches[0].path).read_text(), end='')
            elif getattr(args, 'json', False):
                print(json.dumps([entry.as_dict() for entry in entries], indent=2))
            elif not entries:
                print('No saved skills. Ask your agent to save a reusable method, or use openrua skills add <directory>.')
            else:
                for entry in entries:
                    print(f'{entry.name} [{entry.scope}]  {entry.description}\n  {entry.path}' +
                          (f'\n  Invalid: {entry.error}' if entry.error else ''))
    except (ValueError, OSError) as exc:
        raise UsageError(str(exc), hint='openrua skills --help') from exc
    return 0


def add_parser(sub) -> None:
    p = sub.add_parser('skills', help='list reusable methods, import a skill, or set project accumulation')
    p.add_argument('-C', '--cd', '--workspace', dest='workspace', default=argparse.SUPPRESS,
                   help='project directory (default: current directory)')
    p.add_argument('--trust-workspace', action='store_true', default=argparse.SUPPRESS,
                   help='explicitly trust the selected project before writing')
    actions = p.add_subparsers(dest='action')
    listing = actions.add_parser('list', help='inspect saved file metadata; native availability may differ')
    listing.add_argument('--json', action='store_true')
    listing.add_argument('--personal', action='store_true', help='only personal OpenRUA skills')
    show = actions.add_parser('show', help='read one saved SKILL.md')
    show.add_argument('name')
    show.add_argument('--personal', action='store_true')
    add = actions.add_parser('add', help='import or promote one skill without overwriting existing files')
    add.add_argument('source', help='skill directory or SKILL.md')
    add.add_argument('--personal', action='store_true', help='save in the user directory for use across projects')
    policy = actions.add_parser('policy', help='read or change accumulation instructions; existing skills remain usable')
    policy.add_argument('mode', nargs='?', choices=skills.MODES)
    p.set_defaults(fn=run)
