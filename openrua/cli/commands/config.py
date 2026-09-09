"""``openrua config show | set KEY VALUE ... | schema``: your defaults file
and the schema.

The defaults file is ``~/.openrua/config.yaml``: the robot, simulator
and benchmark a command uses when it names none, the agent's defaults,
the sandbox's docker flags. ``set`` writes keys into it (dotted paths,
YAML scalars), ``show`` prints it, ``schema`` prints every key of the
resolved config with its meaning.
"""

from __future__ import annotations

import json

import yaml

from openrua import config
from openrua.config import paths
from openrua.errors import UsageError


def _set(data: dict, key: str, value: str) -> None:
    node = data
    parts = key.split(".")
    for part in parts[:-1]:
        node = node.setdefault(part, {})
        if not isinstance(node, dict):
            raise UsageError(f"{key}: {part} is not a section")
    node[parts[-1]] = yaml.safe_load(value)


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
    if len(args.pairs) < 2 or len(args.pairs) % 2:
        raise UsageError("config set takes KEY VALUE pairs",
                         hint="openrua config set robot panda simulator robosuite")
    data = config.load_yaml(path) if path.is_file() else {}
    data = data or {}
    for key, value in zip(args.pairs[::2], args.pairs[1::2]):
        _set(data, key, value)
    config.validate(config.UserConfig, data, path)     # unknown keys are errors
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(data, sort_keys=False))
    for key, value in zip(args.pairs[::2], args.pairs[1::2]):
        print(f"{key}: {value}")
    return 0


def add_parser(sub) -> None:
    p = sub.add_parser("config", help="your defaults (robot, simulator, benchmark, agent)",
                       description="Your defaults file, ~/.openrua/config.yaml: `config set "
                       "robot panda simulator robosuite` writes keys into it (dotted paths "
                       "such as agent.model), `config show` prints it, `config schema` prints "
                       "the resolved config's JSON schema, every key with its meaning.")
    p.add_argument("what", choices=("show", "set", "schema"), help="what to do")
    p.add_argument("pairs", nargs="*", metavar="KEY VALUE", help="for set: key and value, "
                   "repeatable (robot, simulator, benchmark, agent.name, agent.model, ...)")
    p.set_defaults(fn=run)
