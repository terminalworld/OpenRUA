"""View or save project settings, or personal preferences with --user."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


from openrua import agents, config
from openrua.config import paths, projects, settings
from openrua.cli import workspace
from openrua.errors import UsageError

# flag -> key path in the defaults file; an agent's facts go under
# agents.<name>, the agent being --agent, else the file's, else the package's
NAMES = {"robot": "robot", "sim": "simulator", "bench": "benchmark", "agent": "agent"}
FACTS = {"model": "model", "credentials_dir": "credentials_dir", "version": "version"}


def run(args) -> int:
    if not args.user and args.what != 'schema':
        return project_config(args)
    if args.user and any(getattr(args, key) is not None for key in ('robot', 'sim', 'bench')):
        raise UsageError('robot, simulator and benchmark choices belong to a project',
                         hint='run openrua config set in that project without --user')
    path = paths.config_path(args.home)
    if args.what == "schema":
        model = config.UserConfig if args.user else config.ResolvedConfig
        print(json.dumps(model.model_json_schema(), indent=2))
        return 0
    if args.what == "show":
        print(f"# {path}" + ("" if path.is_file() else " (absent: package defaults apply)"))
        if path.is_file():
            print(path.read_text(), end="")
        return 0
    names = {f: getattr(args, f) for f in NAMES if getattr(args, f) is not None}
    facts = {f: getattr(args, f) for f in FACTS if getattr(args, f) is not None}
    if not names and not facts and args.auth is None and args.api_key_file is None:
        raise UsageError("config set needs at least one flag",
                         hint="openrua config set --robot panda --sim robosuite --agent <name>")
    updates = {NAMES[flag]: None if value == "null" else value for flag, value in names.items()}
    agent_facts = {FACTS[flag]: None if value == "null" else value for flag, value in facts.items()}
    if args.auth is not None or args.api_key_file is not None:
        if args.auth != "api" and args.api_key_file is not None:
            raise UsageError("--api-key-file requires --auth api")
        if args.auth == "api" and not args.api_key_file:
            raise UsageError("--auth api requires --api-key-file")
        selected = updates.get("agent") or config.load_user_config(path).agent or config.load_user_config(paths.package_config_path()).agent
        auth = {"mode": args.auth}
        if args.api_key_file:
            auth["key_file"] = str(Path(args.api_key_file).expanduser().resolve())
        try:
            agents.api_key_file(agents.get(selected, args.home), auth)
        except ValueError as exc:
            raise UsageError(str(exc), hint="check --api-key-file, or select --auth native") from exc
        agent_facts["auth"] = auth
    data = settings.update(args.home, updates, agent_facts)
    written = list(updates.items())
    if agent_facts:
        agent = data.get("agent") or config.load_user_config(paths.package_config_path()).agent
        written.extend((f"agents.{agent}.{key}", value) for key, value in agent_facts.items())
    for key, value in written:
        print(f"{key}: {value}")
    return 0


def project_config(args) -> int:
    root = projects.resolve(args.workspace or Path.cwd())
    path = projects.settings_path(root)
    if args.what == 'show':
        print(f'# {path}')
        print(path.read_text() if path.exists() else '{}')
        return 0
    if any(getattr(args, key) is not None for key in ('credentials_dir', 'version', 'auth', 'api_key_file')):
        raise UsageError('login and personal agent settings belong to the user',
                         hint='openrua config set --user --agent <name> --auth native')
    overrides = {key: getattr(args, key) for key in projects.FIELDS}
    if all(value is None for value in overrides.values()):
        raise UsageError('config set needs at least one project setting')
    root = workspace.choose(root, args.home, accept=args.trust_workspace)
    if root is None:
        return 0
    chosen = projects.options(root, {key: '' if value == 'null' else value
                                    for key, value in overrides.items()}, relative_to=Path.cwd())
    if chosen.get('robot') or chosen.get('bench'):
        config.compose(chosen['robot'], chosen['sim'], chosen['bench'], args.home,
                       agent=chosen['agent'], model=chosen['model'])
    projects.save_selection(root, {key: value or '' for key, value in chosen.items()})
    print(f'Saved project settings: {path}')
    return 0


def add_parser(sub) -> None:
    p = sub.add_parser('config', help='view or save project settings; --user for personal preferences',
                       description='Save this project in .openrua/settings.json. Bundled environment choices '
                       'create maintained configuration files in configs/. --user selects personal agent '
                       'preferences and authentication in ~/.openrua/config.yaml.')
    p.add_argument("what", choices=("show", "set", "schema"), help="what to do")
    p.add_argument('--user', action='store_true', help='read or change personal agent preferences')
    p.add_argument('-C', '--cd', '--workspace', dest='workspace', default=argparse.SUPPRESS,
                   help='project directory (default: the current directory)')
    p.add_argument('--trust-workspace', action='store_true', default=argparse.SUPPRESS,
                   help='explicitly trust this project before saving')
    p.add_argument("--robot", default=None, help="robot a command uses when it names none")
    p.add_argument("--sim", default=None, help="simulator, likewise")
    p.add_argument("--bench", default=None, help="benchmark whose world a session loads by "
                   "default (null = the simulator's native scene)")
    p.add_argument("--agent", default=None, help="agent a command uses when it names none "
                   "(openrua agents)")
    p.add_argument("--model", default=None, help="project model; with --user, the personal model under agents.<agent>")
    p.add_argument("--version", default=None, metavar="VERSION",
                   help="pin the agent's CLI version (agents.<agent>.version)")
    p.add_argument("--credentials-dir", default=None, help="the agent's login profile "
                   "directory (agents.<agent>.credentials_dir)")
    p.add_argument("--auth", choices=("native", "api"), default=None,
                   help="authentication source; native login is the default, API use is explicit")
    p.add_argument("--api-key-file", default=None,
                   help="file containing a raw key, used only with --auth api")
    p.set_defaults(fn=run)
