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

import yaml

from openrua import config
from openrua.config import paths
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
    if not names and not facts:
        raise UsageError("config set needs at least one flag",
                         hint="openrua config set --robot panda --sim robosuite --agent <name>")
    config.load_user_config(path)                       # refuses the old agent: shape
    data = (config.load_yaml(path) if path.is_file() else {}) or {}
    written = []
    for flag, value in names.items():
        data[NAMES[flag]] = value
        written.append((NAMES[flag], value))
    if facts:
        agent = (names.get("agent") or data.get("agent")
                 or config.load_user_config(paths.package_config_path()).agent)
        node = data.setdefault("agents", {}).setdefault(agent, {})
        for flag, value in facts.items():
            node[FACTS[flag]] = value
            written.append((f"agents.{agent}.{FACTS[flag]}", value))
    config.validate(config.UserConfig, data, path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(data, sort_keys=False))
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
    p.set_defaults(fn=run)
