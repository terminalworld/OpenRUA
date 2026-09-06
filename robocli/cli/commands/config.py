"""``robocli config schema``: every config key and its meaning, as JSON schema."""

from __future__ import annotations

import json

from robocli import config


def run(args) -> int:
    if args.what == "schema":
        print(json.dumps(config.ResolvedConfig.model_json_schema(), indent=2))
    return 0


def add_parser(sub) -> None:
    p = sub.add_parser("config", help="the configuration schema",
                       description="Configuration: `robocli config schema` prints the "
                       "resolved config's JSON schema, every key with its meaning.")
    p.add_argument("what", choices=("schema",), help="what to show")
    p.set_defaults(fn=run)
