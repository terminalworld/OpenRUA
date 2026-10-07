"""Render the reference pages from the code: docs/cli.md from the
argument parsers, docs/config.md from the configuration schema.

    python scripts/render_docs.py          # write both pages
    python scripts/render_docs.py --check  # exit 1 when a page is stale

CI runs --check, so the pages cannot drift from the code they describe.
"""

from __future__ import annotations

import argparse
import copy
import sys
import textwrap
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


def _parser_reference(parser: argparse.ArgumentParser) -> str:
    # argparse versions measure nested command columns differently. Render
    # their help separately, using the same Markdown table as the top level.
    printable = copy.deepcopy(parser)
    groups = [a for a in printable._actions if isinstance(a, argparse._SubParsersAction)]
    tables = []
    for group in groups:
        descriptions = {a.dest: a.help for a in group._choices_actions}
        rows = ["\n| command | does |\n|---|---|\n"]
        for name in group.choices:
            rows.append(f"| `{name}` | {descriptions.get(name, '')} |\n")
        tables.append("".join(rows))
        group._choices_actions = []
        group.metavar = group.metavar or "COMMAND"
    if printable._mutually_exclusive_groups:
        # Python 3.13 permits wrapping inside exclusive groups, whereas older
        # argparse versions keep them together. Use one deterministic wrapping
        # rule for the reference; the interactive CLI keeps its native help.
        formatter = argparse.HelpFormatter(printable.prog, width=10000)
        formatter.add_usage(None, printable._actions, printable._mutually_exclusive_groups)
        usage = formatter.format_help().strip()
        wrapped = textwrap.fill(usage, width=78,
                                subsequent_indent=' ' * len(f'usage: {printable.prog} '),
                                break_long_words=False, break_on_hyphens=False)
        printable.usage = wrapped.removeprefix('usage: ')
    return f"```\n{printable.format_help().rstrip()}\n```\n" + "".join(tables)


def render_cli() -> str:
    sys.argv[0] = "openrua"  # argparse derives every prog from it
    from openrua.cli import build_parser
    from openrua.errors import EXIT_CODES

    ap = build_parser()
    verbs = next(a for a in ap._actions if isinstance(a, argparse._SubParsersAction))
    # The verb table is laid out here, not by argparse: its column
    # widths differ between Python versions and the page must not.
    out = [CLI_HEAD, _parser_reference(ap), "\n"]
    out.append("Without a subcommand, an unnamed launch opens setup with a new session ID. An explicit name reconnects "
               "to the named shared session. `--gui` selects the browser, `--cli` plain chat, "
               "`--resume [ID]` selects history, and `--setup` edits the shared defaults. With no interface selected, "
               "non-interactive invocations print help.\n")
    for name, sub in verbs.choices.items():
        out.append(f"\n## openrua {name}\n\n" + _parser_reference(sub))
        nested = [a for a in sub._actions if isinstance(a, argparse._SubParsersAction)]
        for group in nested:
            for unit, p in group.choices.items():
                out.append(f"\n### openrua {name} {unit}\n\n" + _parser_reference(p))
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
        (schema.RobotType, "robots/<type>.yaml (bundled or ~/.openrua/robots/): a robot type"),
        (schema.RobotInstance, "robots/<name>.yaml: a particular robot (type: + machine:)"),
        (schema.SimulatorProfile, "simulators/<engine>.yaml (bundled or ~/.openrua/simulators/)"),
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
            out.append(f"| `{f.alias or name}` | {_type_name(f.annotation)} | {_default(f)} | {desc} |\n")
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
