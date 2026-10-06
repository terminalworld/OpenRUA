"""``openrua config show | set --robot R --sim S ... | schema``: your
defaults file and the schema.

The defaults file is ``~/.openrua/config.yaml``: the robot, simulator,
benchmark and agent a command uses when it names none; under
``agents.<name>`` what this machine knows about each agent (model,
version pin, login directory, options); the sandbox's docker flags.
``set`` writes the flags it is given into it (the same flags ``run``
takes; the agent facts under the agent named or defaulted), ``show``
prints it, ``schema`` prints every key of the resolved config with its
meaning.
"""

from __future__ import annotations

import json
from pathlib import Path


from openrua import agents, config
from openrua.config import paths, settings
from openrua.errors import UsageError

# flag -> key path in the defaults file; an agent's facts go under
# agents.<name>, the agent being --agent, else the file's, else the package's
NAMES = {"robot": "robot", "sim": "simulator", "bench": "benchmark", "agent": "agent"}
FACTS = {"model": "model", "credentials_dir": "credentials_dir", "version": "version"}


def run(args) -> int:
    path = paths.config_path(args.home)
    if args.what == "schema":
        print(json.dumps(config.ResolvedConfig.model_json_schema(), indent=2))
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


def add_parser(sub) -> None:
    p = sub.add_parser("config", help="your defaults (robot, simulator, benchmark, agent)",
                       description="Your defaults file, ~/.openrua/config.yaml: `config set` "
                       "writes the flags it is given into it (the same flags run takes), "
                       "`config show` prints it, `config schema` prints the resolved "
                       "config's JSON schema, every key with its meaning.")
    p.add_argument("what", choices=("show", "set", "schema"), help="what to do")
    p.add_argument("--robot", default=None, help="robot a command uses when it names none")
    p.add_argument("--sim", default=None, help="simulator, likewise")
    p.add_argument("--bench", default=None, help="benchmark whose world run / up load by "
                   "default (null = the simulator's native scene)")
    p.add_argument("--agent", default=None, help="agent a command uses when it names none "
                   "(openrua agents)")
    p.add_argument("--model", default=None, help="model the agent runs on this machine "
                   "(written under agents.<agent>)")
    p.add_argument("--version", default=None, metavar="VERSION",
                   help="pin the agent's CLI version (agents.<agent>.version)")
    p.add_argument("--credentials-dir", default=None, help="the agent's login profile "
                   "directory (agents.<agent>.credentials_dir)")
    p.add_argument("--auth", choices=("native", "api"), default=None,
                   help="authentication source; native login is the default, API use is explicit")
    p.add_argument("--api-key-file", default=None,
                   help="file containing a raw key, used only with --auth api")
    p.set_defaults(fn=run)
