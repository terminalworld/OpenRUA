"""``openrua config show | set --robot R --sim S ... | schema``: your
defaults file and the schema.

The defaults file is ``~/.openrua/config.yaml``: the robot, simulator
and benchmark a command uses when it names none, the agent and model,
the sandbox's docker flags. ``set`` writes the flags it is given into
it (the same flags ``run`` takes), ``show`` prints it, ``schema`` prints
every key of the resolved config with its meaning.
"""

from __future__ import annotations

import json

import yaml

from openrua import config
from openrua.config import paths
from openrua.errors import UsageError

# flag -> key path in the defaults file
KEYS = {"robot": ("robot",), "sim": ("simulator",), "bench": ("benchmark",),
        "agent": ("agent", "name"), "model": ("agent", "model"),
        "credentials_dir": ("agent", "credentials_dir")}


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
    given = {flag: getattr(args, flag) for flag in KEYS if getattr(args, flag) is not None}
    if not given:
        raise UsageError("config set needs at least one flag",
                         hint="openrua config set --robot panda --sim robosuite --agent <name>")
    data = (config.load_yaml(path) if path.is_file() else {}) or {}
    for flag, value in given.items():
        node = data
        for part in KEYS[flag][:-1]:
            node = node.setdefault(part, {})
        node[KEYS[flag][-1]] = value
    config.validate(config.UserConfig, data, path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(data, sort_keys=False))
    for flag, value in given.items():
        print(f"{'.'.join(KEYS[flag])}: {value}")
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
    p.add_argument("--agent", default=None, help="agent (openrua agents)")
    p.add_argument("--model", default=None, help="model id for the agent")
    p.add_argument("--credentials-dir", default=None, help="the agent's login profile directory")
    p.set_defaults(fn=run)
