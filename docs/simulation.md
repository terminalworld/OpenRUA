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
  embodies `panda` and loads its `Lift` scene.
- `benchmarks/<name>.yaml`: names its `robot:` and `simulator:` and
  brings its own world: `install:` (its venv and distro, over the
  simulator's) and `scenes:` (scene cameras, a default suite, and robot
  embodiments its assets add or adjust). LIBERO-PRO ships LIBERO's fork
  of robosuite 1.4 under Jazzy; CaP-Bench and RoboCasa365 run on
  robosuite 1.5 under Humble; RoboCasa's assets bring the `panda-omron`
  body robosuite's own lack.

## Where the installs live

Each install is a checkout plus a Python venv, mounted into the robot
container at `up`. They are large (tens of GB with assets) and are built
locally under the user directory:

```
~/.openrua/simulators/
  cap-x/            LIBERO-PRO and CaP-Bench (.venv-libero, .venv-capbench); also robosuite's own scenes
  robocasa365/      RoboCasa365 (.venv-robocasa)
  <engine>.yaml     your own simulator files, looked up after the bundled ones
```

`install.venv` is relative to that directory (`cap-x/.venv-libero`); an
absolute path or `~` works too if you keep simulators elsewhere. The
venv must have `openrua` installed (the container starts the bridge
with `python -m openrua.robot.sim.bridge.main` from it).

A checkout is the benchmark's own repository at a known commit, and
the venv is one the robot container can run: its Python matches the
image's ROS distro (3.12 for Jazzy, 3.10 for Humble), it is made with
`uv` (the container mounts uv's interpreter store), the benchmark's
packages are installed in it, and so is `openrua`. `openrua doctor
<robot> --bench <benchmark>` (or `--sim <engine>`) reports which venv is
missing and where it looked.

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
scored: a module exposing `LOADER`, see
[architecture.md](architecture.md)). Bundled, the two live under
`openrua/configs/benchmarks/` and `openrua/robot/sim/bridge/environments/`;
yours sit side by side (`entry_point: ./my_bench.py`) and are passed
with `--bench ./my-bench.yaml`. A new engine is a simulator file plus a
bridge backend for it. A robot the engine's
own assets lack is declared under the benchmark's `scenes.robots`, as
RoboCasa365 does for `panda-omron`.
