---
summary: Evidence required for guided setup choices, and automatic image preparation
read_when:
  - You want to know which combinations the startup menus offer
  - You are adding a robot or simulator to guided setup
---

# Guided setup validation

The guided robot, simulator and benchmark fields describe complete tested
combinations. A registered YAML file alone does not qualify an entry for these
menus. Select a robot first; downstream choices update to compatible entries.
The list currently contains:

| Robot | Simulator | Benchmark | Checked default scene |
|---|---|---|---|
| `panda` | `robosuite` | Native scene (no benchmark) | `Lift` |
| `panda` | `robosuite` | `capbench` | `capbench_lift` |
| `panda-omron` | `robosuite` | `robocasa365` | `CloseBlenderLid` |

The recorded check starts the declared robot and sandbox images in an isolated
network, resets the default scene, receives joint states and a camera frame,
and executes a trajectory to the current joint positions. It also checks agent executables using the version command from each selected
agent manifest. By default it checks the scene's configured agent; repeat
`--agent` to check others installed in the sandbox. These are CLI version checks,
not authentication or model-response tests. This is environment and interface evidence,
not proof of a successful model task, every benchmark task, physical deployment,
or every model/account configuration.

Evidence is stored under `openrua/configs/startup/`. It records image IDs, the
exact selection, suite, task, seed and a digest of bundled environment declarations.
Changing those declarations invalidates the guided entries until the checks are
rerun. Fast CI checks that every offered entry still composes and has current
evidence; CI without simulator images does not claim to rerun the physical
simulation checks.

To add or refresh an entry, build its declared images and run:

```sh
python scripts/check_environment.py --robot panda-omron --sim robosuite \
  --bench robocasa365 --agent claude-code --agent codex \
  --output openrua/configs/startup/robocasa365.json
```

For a native scene, omit `--bench`; a scene name is sufficient when there is no
benchmark task instruction.

This script refuses physical robots, uses unique temporary resource names,
makes no model calls, and writes passing evidence only after the checks and
resource shutdown complete. Review the evidence alongside the configuration
change. Other profiles can be tested through explicit CLI options without
claiming guided support.

## Preparation on the user's machine

Recorded evidence does not mean that images or credentials are already present
on another machine. **Prepare and start** validates the selected combination,
builds missing images through the existing `openrua build` commands, then runs
the preparation checks and starts the session. The simulator's ROS distribution
selects the sandbox image; the selected agent supplies its installation manifest.
Docker itself and agent login are prerequisites; OpenRUA reports how to resolve
them without installing host system software or collecting credentials in setup.

Build output is streamed and retained in `~/.openrua/preparation/`. A failed
build never launches the session. Retry reuses completed images and Docker's
build cache. **Save & check** does not download, build or start a robot. Existing
images are reused; declaration-change warnings still require an explicit rebuild.
The readiness report checks both the proxy image and its standing container,
since a rebuild cannot update a container already serving other sessions.
See [updating a shared proxy](install.md#updating-a-shared-proxy) for the repair
procedure. Network attachment failures retain Docker's error and stop startup.
