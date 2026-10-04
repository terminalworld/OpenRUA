# Contributing

## Setting up

```bash
git clone https://github.com/terminalworld/OpenRUA && cd OpenRUA
python -m venv .venv && .venv/bin/pip install -e ".[dev]"
.venv/bin/pytest -q && .venv/bin/lint-imports
```

Both checks must pass before a pull request. The default package includes the
terminal interface, so it is also exercised in local and CI tests. After pushing, check the
GitHub Actions run for that commit; local checks do not replace the remote
Python matrix. CI also runs browser tests in Chromium. To run them locally:

```bash
.venv/bin/pip install -e ".[browser-test]"
.venv/bin/python -m playwright install chromium
.venv/bin/pytest -q tests/web
```

The source-only [Pi terminal prototype](ui/terminal/README.md) has a separate
Node test job. With the Python development environment activated, run
`npm ci --prefix ui/terminal --ignore-scripts` and
`npm test --prefix ui/terminal`. Its tests use the actual session service and
controlled agent events without paid model calls.

The workspace template hash is pinned in
`tests/sandbox/test_workspace_template.py`. An intentional change to the
agent-facing documentation or tools changes the experimental artifact.
Review the template diff before updating that pin in the same commit;
never change it merely to silence a failing test. Historical trial hashes
remain unchanged.

`pytest` includes the layering contract (`tests/architecture/test_layering.py`) and the agent
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
  the engine module its `entry_point` names (under
  `openrua/robot/sim/bridge/engines/`, exposing `ENGINE`: how joints,
  poses and cameras are read and actions assembled; the test suite fails
  an engine with a name of `ENGINE_INTERFACE` missing). It embodies
  robots and loads a native scene; it knows no benchmark.
- **An agent**: a manifest under `openrua/configs/agents/<name>.yaml`
  and a module under `openrua/plugins/agents/<name>.py` named by its `entry_point`
  (docs/agents.md). `openrua.testing.check_manifest` must pass; the
  bundled agents run through it in `tests/agents/test_agents.py`. Nothing
  outside those two directories may name the agent (the boundary
  test lists the banned tokens).
- **A benchmark**: a config under `openrua/configs/benchmarks/<name>.yaml`
  naming its robot and simulator, its loader (`entry_point`, a module
  under `openrua/robot/sim/bridge/environments/` exposing `LOADER`: how
  its scenes are built, reset and scored) and its world (`install:` with
  pinned checkouts, a Python version and a requirements lock in the
  `<name>/` directory next to the yaml, so `openrua build --bench <name>`
  renders it into `openrua-sim-<name>`; `scenes:`).
  `tests/config/test_install.py` checks every bundled declaration is
  complete, its files ship and it renders.
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

Ship behavior changes with their command help, user tutorial, and reference
updates. Start from the user path affected by the change; keep installation
facts in `docs/install.md`, shared-session steps in `examples/shared-session.md`,
interface controls in `docs/terminal.md` or `docs/sessions.md`, and module
contracts in `docs/architecture.md`. Link to the owning page instead of copying
its details into every guide. The README introduces supported paths in the
released product; label work that is only available in a later version.

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

## Releasing

Each release archives a tested batch of useful changes. We currently use one
release channel; experimental features retain explicit documentation of their
limits. Add an entry to `CHANGELOG.md` with features, behavior changes, upgrade
instructions, and validation scope, then bump `version` in `pyproject.toml`.

Versions use `a.b.c`: increment `a` for a major product or core architecture
change, including incompatible upgrades; increment `b` for a substantial batch
of new features; increment `c` for fixes, small improvements, and compatibility
adjustments. Reset subsequent components to zero when incrementing an earlier
one. For example, guided terminal startup moves 0.1.0 to 0.2.0, while a fix to
that startup would move 0.2.0 to 0.2.1. Documentation-only maintenance can ship
with the next relevant package release.

Commit and push the release preparation, check its CI, then tag that commit
`v<version>` and push the tag. The Release workflow reruns the shared CI jobs
against the tagged commit, builds and checks the sdist and wheel, matches the
tag to the package version, and installs the wheel in an isolated environment.
It publishes the same artifacts to GitHub Releases and PyPI through trusted
publishing, using the changelog entry as the release notes. Verify both
publication jobs and installation from PyPI before declaring the release done.
Keep published tags immutable; subsequent fixes receive a new version.

The local artifact check is `python scripts/check_dist.py --dist dist
--version <version>` after `python -m build`. It installs the base package outside the checkout, checks bundled resources, and mounts the TUI
headlessly without a robot or model call.
