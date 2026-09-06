# Contributing

## Setting up

```bash
git clone https://github.com/terminalworld/RoboCLI && cd RoboCLI
python -m venv .venv && .venv/bin/pip install -e ".[dev]"
.venv/bin/pytest -q && .venv/bin/lint-imports
```

Both commands must pass before a pull request. `pytest` includes the
layering contract (`tests/test_layering.py`) and the agent boundary
(`tests/test_agent_boundary.py`); `lint-imports` checks the same
contract from `pyproject.toml`.

## Adding things

- **A robot**: a profile under `robocli/robots/<name>.yaml`
  (docs/your-own-robot.md). `robocli doctor <name>` must load it;
  `tests/test_config.py` validates every bundled profile.
- **An agent**: a module under `robocli/agents/<name>.py` exposing
  `AGENT` (docs/agents.md). `robocli.testing.check_agent` must pass;
  the bundled adapters are run through it in `tests/test_agents.py`.
  Nothing outside `robocli/agents/` may name the agent (the boundary
  test lists the banned tokens).
- **A benchmark**: a config under `robocli/benchmarks/<name>.yaml` and,
  if it needs a new simulator, a loader under
  `robocli/robot/onboard/environment/`.
- **A config key**: add it to `robocli/config.py` with a default and a
  description; unknown keys are errors everywhere, so the schema is
  the single place a key exists.

## Command-line conventions

- The main object is a positional argument; configuration is a flag:
  `robocli up panda-sim --ros-domain 7`, `robocli doctor ur5e --json`.
  An optional filter, mode or setting stays a flag even when it is
  usually given.
- Every argument has `help=`, and the help says where the default comes
  from (`default: the profile's`, `default: ~/.robocli`).
- Every command has both `help` (one line, in `robocli --help`) and
  `description` (a sentence or two, in `robocli <verb> --help`).
  Registration order in `build_parser` is the `--help` order.
- Listings take `--json`; `doctor` prints JSON whenever stdout is not
  a terminal.
- Errors are `robocli.errors` subclasses raised from library code with
  a `hint`; only the entry point prints and exits (sysexits codes).
  Never `SystemExit` from a library function.

## Style

- Every file runs on its own with parameters in and values or files
  out; units talk through parameters and files, never environment
  variables (the one exception is `ROBOCLI_HOME`, read once at the
  CLI entry point).
- Knowledge lives in one place: an agent fact in its adapter, a path
  rule in `paths.py`, a config key in `config.py`.
- Fail loud with the fix in the message; never return a dead artifact
  silently.
- Comments and docstrings: plain words, no dashes as punctuation.
