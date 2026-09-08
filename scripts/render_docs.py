"""Render the reference pages from the code: docs/cli.md from the
argument parsers, docs/config.md from the configuration schema.

    python scripts/render_docs.py          # write both pages
    python scripts/render_docs.py --check  # exit 1 when a page is stale

CI runs --check, so the pages cannot drift from the code they describe.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Any, get_args, get_origin

DOCS = Path(__file__).resolve().parents[1] / "docs"

CLI_HEAD = """---
summary: Every openrua verb, its arguments, and the exit codes
read_when:
  - You want the exact flags of a verb without running --help
  - A script needs to act on openrua's exit code
---

# Command line

Generated from the parsers by `scripts/render_docs.py`; edit the
`add_parser` of a verb, not this page.

"""

CONFIG_HEAD = """---
summary: Every key of the robot profiles, benchmark configs, agent manifests and the defaults file
read_when:
  - You are writing a robot profile, a benchmark config or an agent manifest
  - A config error names a key and you want its meaning and allowed values
---

# Configuration

Generated from the schema by `scripts/render_docs.py`; edit the
`Field(description=...)` in `openrua/config/schema.py`, not this page.
Unknown keys are errors everywhere. `openrua config schema` prints the
same information as JSON Schema.

"""


def render_cli() -> str:
    sys.argv[0] = "openrua"  # argparse derives every prog from it
    from openrua.cli import build_parser
    from openrua.errors import EXIT_CODES

    ap = build_parser()
    verbs = next(a for a in ap._actions if isinstance(a, argparse._SubParsersAction))
    # The verb table is laid out here, not by argparse: its column
    # widths differ between Python versions and the page must not.
    out = [CLI_HEAD, f"```\n{ap.format_usage().rstrip()}\n```\n\n| verb | does |\n|---|---|\n"]
    for action in verbs._choices_actions:
        out.append(f"| `{action.dest}` | {action.help} |\n")
    out.append("\nGlobal options: `--home` (the user directory, default `$OPENRUA_HOME` "
               "or `~/.openrua`), `--version`.\n")
    for name, sub in verbs.choices.items():
        out.append(f"\n## openrua {name}\n\n```\n{sub.format_help().rstrip()}\n```\n")
        nested = [a for a in sub._actions if isinstance(a, argparse._SubParsersAction)]
        for group in nested:
            for unit, p in group.choices.items():
                out.append(f"\n### openrua {name} {unit}\n\n```\n{p.format_help().rstrip()}\n```\n")
    out.append("\n## Exit codes\n\n| code | meaning |\n|---|---|\n")
    notes = {"ok": "done", "error": "any other failure", "usage": "bad arguments",
             "noinput": "a named robot, benchmark, agent or file does not exist",
             "unavailable": "docker, an image, a container or the simulator is missing",
             "noperm": "login or credentials", "config": "a config file that does not fit the schema"}
    for k, v in EXIT_CODES.items():
        out.append(f"| {v} | {notes[k]} |\n")
    return "".join(out)


def _type_name(tp: Any) -> str:
    from pydantic import BaseModel

    origin = get_origin(tp)
    if origin is None:
        if isinstance(tp, type) and issubclass(tp, BaseModel):
            return f"[{tp.__name__}](#{tp.__name__.lower()})"
        return getattr(tp, "__name__", str(tp)).replace("NoneType", "null")
    args = get_args(tp)
    if origin in (list, tuple, set):
        return f"list[{_type_name(args[0])}]" if args else "list"
    if origin is dict:
        return f"dict[{_type_name(args[0])}, {_type_name(args[1])}]"
    if str(origin) in ("typing.Union", "<class 'types.UnionType'>") or origin.__name__ == "UnionType":
        return " \\| ".join(_type_name(a) for a in args)
    if str(origin) == "typing.Literal":
        return " \\| ".join(repr(a) for a in args)
    if str(origin) == "typing.Annotated":
        return _type_name(args[0])
    return str(tp)


def _default(field) -> str:
    from pydantic import BaseModel
    from pydantic_core import PydanticUndefined

    if field.default_factory is not None:
        try:
            v = field.default_factory()
        except TypeError:
            return "(computed)"
        return "" if isinstance(v, BaseModel) else repr(v)
    if field.default is PydanticUndefined:
        return "**required**"
    return repr(field.default)


def render_config() -> str:
    from pydantic import BaseModel

    from openrua.config import schema

    roots = [
        (schema.RobotProfile, "robots/<name>.yaml (bundled or ~/.openrua/robots/)"),
        (schema.Benchmark, "benchmarks/<name>.yaml (bundled or ~/.openrua/benchmarks/)"),
        (schema.UserConfig, "~/.openrua/config.yaml and the package's configs/config.yaml"),
        (schema.AgentManifest, "agents/<name>.yaml (bundled or ~/.openrua/agents/)"),
        (schema.ResolvedConfig, "<trial>/config.yaml, the resolved view every party reads"),
    ]
    seen: list[type[BaseModel]] = []

    def collect(model: type[BaseModel]) -> None:
        if model in seen:
            return
        seen.append(model)
        for f in model.model_fields.values():
            stack = [f.annotation]
            while stack:
                t = stack.pop()
                if isinstance(t, type) and issubclass(t, BaseModel):
                    collect(t)
                else:
                    stack.extend(get_args(t))

    out = [CONFIG_HEAD, "## Files\n\n| file | schema |\n|---|---|\n"]
    for model, where in roots:
        out.append(f"| `{where}` | [{model.__name__}](#{model.__name__.lower()}) |\n")
        collect(model)
    for model in seen:
        # Whitespace collapsed: Python 3.13 dedents docstrings at compile
        # time and older versions do not; the page must not depend on
        # which interpreter rendered it.
        doc = " ".join((model.__doc__ or "").split())
        out.append(f"\n## {model.__name__}\n\n")
        if doc:
            out.append(doc + "\n\n")
        out.append("| key | type | default | meaning |\n|---|---|---|---|\n")
        for name, f in model.model_fields.items():
            desc = (f.description or "").replace("|", "\\|").replace("\n", " ")
            out.append(f"| `{name}` | {_type_name(f.annotation)} | {_default(f)} | {desc} |\n")
    return "".join(out)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--check", action="store_true",
                    help="compare with the files on disk instead of writing")
    args = ap.parse_args()
    pages = {DOCS / "cli.md": render_cli(), DOCS / "config.md": render_config()}
    stale = []
    for path, text in pages.items():
        if args.check:
            if not path.exists() or path.read_text() != text:
                stale.append(path)
        else:
            path.write_text(text)
            print(f"wrote {path.relative_to(DOCS.parent)}")
    if stale:
        print("stale: " + ", ".join(str(p.relative_to(DOCS.parent)) for p in stale)
              + "\nrun: python scripts/render_docs.py", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
