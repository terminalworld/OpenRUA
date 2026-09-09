# Contributing

## Setting up

```bash
git clone https://github.com/terminalworld/OpenRUA && cd OpenRUA
python -m venv .venv && .venv/bin/pip install -e ".[dev]"
.venv/bin/pytest -q && .venv/bin/lint-imports
```

Both commands must pass before a pull request. `pytest` includes the
layering contract (`tests/architecture/test_layering.py`) and the agent
boundary (`tests/agents/test_agent_boundary.py`); `lint-imports` checks
the same contract from `pyproject.toml`. Tests live in one directory
per unit (`tests/<unit>/`).

## Adding things

- **A robot**: a type under `openrua/configs/robots/<name>.yaml` (facts true
  of it anywhere) and, for simulation, an entry under the simulator's
  `robots:` (docs/your-own-robot.md, docs/simulation.md). `openrua doctor
  <name> --sim <engine>` must load it; `tests/config/test_config.py`
  validates every bundled file.
- **A simulator**: a file under `openrua/configs/simulators/<engine>.yaml` and
  a bridge backend that drives the engine. It embodies robots and loads a
  native scene; it knows no benchmark.
- **An agent**: a manifest under `openrua/configs/agents/<name>.yaml`
  and a module under `openrua/plugins/agents/<name>.py` named by its `entry_point`
  (docs/agents.md). `openrua.testing.check_manifest` must pass; the
  bundled agents run through it in `tests/agents/test_agents.py`. Nothing
  outside those two directories may name the agent (the boundary
  test lists the banned tokens).
- **A benchmark**: a config under `openrua/configs/benchmarks/<name>.yaml`
  naming its robot and simulator and bringing its world (`install:`,
  `scenes:`), plus a loader under `openrua/robot/sim/bridge/environments/`
  (how its scenes are built, reset and scored).
- **A config key**: add it to `openrua/config/schema.py` with a default and a
  description; unknown keys are errors everywhere, so the schema is
  the single place a key exists.

## Command-line conventions

- The main object is a positional argument; configuration is a flag:
  `openrua up panda --sim robosuite --ros-domain 7`, `openrua doctor ur5e --json`.
  An optional filter, mode or setting stays a flag even when it is
  usually given.
- Every argument has `help=`, and the help says where the default comes
  from (`default: the profile's`, `default: ~/.openrua`).
- Every command has both `help` (one line, in `openrua --help`) and
  `description` (a sentence or two, in `openrua <verb> --help`).
  Registration order in `build_parser` is the `--help` order.
- Listings take `--json`; `doctor` prints JSON whenever stdout is not
  a terminal.
- Errors are `openrua.errors` subclasses raised from library code with
  a `hint`; only the entry point prints and exits (sysexits codes).
  Never `SystemExit` from a library function.

## Naming

- Directories and modules take the word the ecosystem already uses for
  the role: `configs/`, `plugins/`, `runner/`, `preflight`, `backend`,
  `registry`, `loader`, `schema`, `manifest`, `hooks`, `operators`,
  `session`. A name must be understood without reading the code.
- No project-private metaphors in names or prose (no seats, houses,
  walls, occupants, conductors, examiners). The agent is the agent, the
  sandbox is the sandbox, the proxy is the proxy.
- One noun per thing: the file a trial resolves to is the resolved
  config (`config.yaml`), the checks before the agent starts are
  preflight, the simulator checkout is the simulator.

## Writing

- Comments and docstrings state the mechanism and the invariant it
  protects, and nothing else: no dates, ticket or audit identifiers,
  decision records, or the history of how a line came to be. That
  history belongs in git.
- Plain words, no dashes as punctuation, no emphasis in capitals.
- A docstring says what the unit does for its caller; a comment says
  why a line is the way it is when the code alone cannot.

## Docs

`docs/` is flat; the README's Documentation table lists every page in
reading order, users first, contributors last. Each page starts with
front matter: a one-line `summary` and a `read_when` list, so a reader
knows in two lines whether the page is for them. `docs/cli.md` and
`docs/config.md` are generated (`python scripts/render_docs.py`; CI
runs `--check`), so a verb's help text and a schema field's
`description` are the only places to write them. Worked examples live
in `examples/`.

## Style

- Every file runs on its own with parameters in and values or files
  out; units talk through parameters and files, never environment
  variables (the one exception is `OPENRUA_HOME`, read once at the
  CLI entry point).
- Knowledge lives in one place: an agent fact in its manifest or hooks
  module, a path rule in `config/paths.py`, a config key in
  `config/schema.py`.
- Fail loud with the fix in the message; never return a dead artifact
  silently.
