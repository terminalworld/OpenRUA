---
summary: Simulator files, the benchmarks' installs under ~/.openrua/simulators, GPU rendering
read_when:
  - openrua doctor says a simulator venv is missing
  - You keep simulators somewhere other than ~/.openrua/simulators
  - You want GPU rendering or a simulator of your own
---

# Simulators

Three kinds of file describe a simulated robot, and they depend in one
direction only: a benchmark names a simulator, a simulator embodies
robots, a robot knows neither.

- `simulators/<engine>.yaml` (`openrua simulators`): the engine, its
  install (a venv and the ROS distro that goes with its Python), the
  native scene `openrua run <robot> --sim <engine>` loads with no
  benchmark, and `robots:`, how the engine drives each robot type it
  embodies (controller, gains, joint-name map). The bundled `robosuite`
  embodies `panda` and loads its `Lift` scene; `maniskill` (SAPIEN 3,
  PhysX on the CPU, Vulkan rendering through lavapipe when there is no
  GPU) embodies `panda` and `widowx` and loads `PickCube-v1`; `robotwin`
  (RoboTwin's own harness on SAPIEN 3) embodies its dual-arm
  `aloha-agilex` and has no native scene; `calvin` (calvin_env on
  PyBullet, CPU rendering) embodies `panda` for the CALVIN benchmark.
- `benchmarks/<name>.yaml`: names its `robot:` and `simulator:` and
  brings its own world: `install:` (its venv and distro, over the
  simulator's) and `scenes:` (scene cameras, a default suite, and robot
  embodiments its assets add or adjust). LIBERO-PRO ships LIBERO's fork
  of robosuite 1.4 under Jazzy; CaP-Bench and RoboCasa365 run on
  robosuite 1.5 under Humble; RoboCasa's assets bring the `panda-omron`
  body robosuite's own lack.

## Installing

Each install is a checkout plus a Python venv, mounted into the robot
container at `up`. What it is made of is declared in the simulator file
(and a benchmark's `install:` over it): repositories at pinned commits
(`checkouts`, with submodules and a patch), the venv's `python` (3.12
goes with Jazzy, 3.10 with Humble), a `requirements` lock, checkouts
installed `editable`, and a `shell` tail for what those cannot say
(asset downloads). `openrua install` renders that to one bash script,
prints it, saves it and runs it:

```bash
openrua install --bench libero_pro     # or --sim robosuite; no flags: your default benchmark
```

Every step checks before it acts, so rerunning is cheap, and the saved
script (`~/.openrua/simulators/install-<name>.sh`) can be read or rerun
by hand. It needs `uv` and `git` on the host. The installs are large
(tens of GB with assets) and land under the user directory:

```
~/.openrua/simulators/
  cap-x/            LIBERO, LIBERO-PRO and CaP-Bench (.venv-libero, .venv-capbench); also robosuite's own scenes
  libero-plus/      LIBERO-Plus (.venv; its fork and 6.4 GB of assets)
  libero-mem/       LIBERO-Mem (.venv; its fork with its own robosuite and robomimic)
  robocerebra/      RoboCerebra (.venv; its LIBERO fork and the RoboCerebra_Bench cases)
  robocasa/         RoboCasa v0.2 on robosuite 1.5.0 (.venv; its kitchen assets)
  robocasa365/      RoboCasa365 (.venv-robocasa)
  maniskill/        ManiSkill 3 and SimplerEnv's Bridge tasks (.venv; assets they download land in data/)
  mikasa/           MIKASA-Robo (.venv; its checkout over ManiSkill 3.0.1, the YCB objects in data/)
  robotwin/         RoboTwin 2.0 (.venv on SAPIEN 3.0.3; its checkout with 9 GB of objects and embodiments)
  calvin/           CALVIN (.venv; the calvin checkout with calvin_env, py3.10 == Humble)
  install-*.sh      the rendered scripts
```

Every LIBERO-family benchmark installs its fork as the `libero`
package, so each fork gets a venv of its own; one loader serves them
all (`entry_point: libero`).

`install.venv` is relative to that directory (`cap-x/.venv-libero`); an
absolute path or `~` works too if you keep simulators elsewhere. The
venv has `openrua` installed by the script (the container starts the
bridge with `python -m openrua.robot.sim.bridge.main` from it): an
editable install of your checkout when the package runs from one, else
the released package at the same version. `openrua doctor <robot>
--bench <benchmark>` (or `--sim <engine>`) compares what is on disk with
the declaration: the venv and its Python, the package in it, each
checkout's commit.

## Rendering

Rendering is EGL; with no GPU it falls back to llvmpipe (slow but
correct, and what the reported runs used). `install.gpus: true` (in the
simulator file or a benchmark's `install:`) injects the NVIDIA driver
when the host has nvidia-container-toolkit (under podman the same flag
resolves through CDI: `nvidia-ctk cdi generate` once, see
docs/podman.md).

## A simulator or benchmark of your own

A new benchmark on an existing engine is a benchmark file plus the
loader its `entry_point` names (how its scenes are built, reset and
scored, and what tasks each suite holds: a module exposing `LOADER`
with `create`, `init_state`, `reset`, `success`, `task_info` and
`tasks`, see [architecture.md](architecture.md); the test suite fails a
loader with a name missing, and `openrua benchmarks <name>` prints
what `tasks` returns). Bundled, the two live under
`openrua/configs/benchmarks/` and `openrua/robot/sim/bridge/environments/`;
yours sit side by side (`entry_point: ./my_bench.py`) and are passed
with `--bench ./my-bench.yaml`. A new engine is a simulator file plus
the engine module its `entry_point` names: a module exposing `ENGINE`,
a callable `ENGINE(env, cfg)` whose result answers every name in
`ENGINE_INTERFACE` (`openrua/robot/sim/bridge/engines/__init__.py`
lists them with their units and conventions: joint addressing, poses,
camera renders and intrinsics, action assembly, FK). Bundled engines
live under `openrua/robot/sim/bridge/engines/` (`robosuite`); yours
sits next to your simulator file (`entry_point: ./my_engine.py`) and
is passed with `--sim ./my-sim.yaml`. A robot the engine's own assets
lack is declared under the benchmark's `scenes.robots`, as RoboCasa365
does for `panda-omron`.
