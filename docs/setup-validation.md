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
| `panda` | `robosuite` | `libero_pro` | `libero_goal_task`, task 0, seed 0 |
| `panda-omron` | `robosuite` | `robocasa365` | `CloseBlenderLid` |

LIBERO-PRO uses ROS Jazzy; the other listed entries use ROS Humble. Setup
selects the matching simulator and sandbox images automatically. Each entry
validates its default scene, not every suite or task in that benchmark.

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
sets up Docker when it is missing or not usable, builds missing images through
the existing `openrua build` commands, offers the agent's sign-in when it is
not signed in, then runs the preparation checks and starts the session. The simulator's ROS distribution
selects the sandbox image; the selected agent supplies its installation manifest.
A real robot's driver images (`backend.build`) and its own sandbox image (when it
declares `backend.sandbox_install`) are built the same way. The proxy image is
prepared only for a session limited to the model service (`--internet model-only`).
Docker and the agent's login are prerequisites. When either is missing, setup
shows the commands that set it up and runs them only on a yes
([Docker](install.md#docker), [your agent's login](install.md#your-agents-login));
the sign-in is the agent's own, so OpenRUA never handles the account's password.

Build output is streamed and retained in `~/.openrua/preparation/`. A failed
build never launches the session. Retry reuses completed images and Docker's
build cache. **Save and check preparation** does not download, build or start
a robot. Existing
images are reused; declaration-change warnings still require an explicit rebuild.
A proxy's whitelist is given when it starts and its name carries the hash of
those hosts, so switching agents starts another proxy instead of needing a
rebuild ([what the agent can reach](install.md#what-the-agent-can-reach)).
Network attachment failures retain Docker's error and stop startup.
