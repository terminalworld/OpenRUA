---
summary: Simulator files, the simulator images openrua build makes from them, GPU rendering
read_when:
  - openrua doctor says a robot image is missing or stale
  - You want to know what a simulator image holds, or how big it is
  - You want GPU rendering or a simulator of your own
---

# Simulators

Three kinds of file describe a simulated robot, and they depend in one
direction only: a benchmark names a simulator, a simulator embodies
robots, a robot knows neither.

- `simulators/<engine>.yaml` (`openrua simulators`): the engine, its
  install (what its image is made of, and the ROS distro that goes
  with its Python), the native scene `openrua run <robot> --sim
  <engine>` loads with no benchmark, and `robots:`, how the engine
  drives each robot type it embodies (controller, gains, joint-name
  map). The bundled `robosuite` embodies `panda` and loads its `Lift`
  scene; `maniskill` (SAPIEN 3, PhysX on the CPU, Vulkan rendering
  through lavapipe when there is no GPU) embodies `panda` and `widowx`
  and loads `PickCube-v1`; `robotwin` (RoboTwin's own harness on SAPIEN
  3) embodies its dual-arm `aloha-agilex` and has no native scene;
  `calvin` (calvin_env on PyBullet, CPU rendering) embodies `panda` for
  the CALVIN benchmark; `vlabench` (dm_control on MuJoCo) embodies
  `panda` as VLABench's `franka`.
- `benchmarks/<name>.yaml`: names its `robot:` and `simulator:` and
  brings its own world: `install:` (its own image contents and distro,
  over the simulator's) and `scenes:` (scene cameras, a default suite,
  and robot embodiments its assets add or adjust). LIBERO-PRO ships
  LIBERO's fork of robosuite 1.4 under Jazzy; CaP-Bench and RoboCasa365
  run on robosuite 1.5 under Humble; RoboCasa's assets bring the
  `panda-omron` body robosuite's own lack.

## The image

A simulated robot runs in one image that holds its whole environment:
the ROS 2 base (ROS, MoveIt, software rendering), the simulator
checkouts at their pinned commits with their patches applied, the
assets, a Python venv with the requirements lock and the checkouts
installed, and this package. What goes in is declared in the simulator
file (and a benchmark's `install:` over it, key by key): repositories
at pinned commits (`checkouts`, with submodules and a patch), the
`python` the image runs (3.12 goes with Jazzy, 3.10 with Humble), a
`requirements` lock, checkouts installed `editable`, and a `shell` tail
for what those cannot say (asset downloads). `openrua build` renders
that into a Dockerfile over the base image and builds it:

```bash
openrua build --bench libero_pro     # openrua-sim-libero_pro; --sim robosuite: the engine's own image
openrua build --all                  # every bundled benchmark
```

An image is named after the declaration it came from,
`openrua-sim-<name>`. A benchmark that writes no install key of its
own runs in its simulator's image (`capbench` in `openrua-sim-robosuite`,
`calvin` in `openrua-sim-calvin`); one that brings anything, a fork or
an asset download, gets its own. Every LIBERO-family benchmark installs
its fork as the `libero` package, so each has an image of its own, and
one loader serves them all (`entry_point: libero`).

Inside the image the checkouts live under `/opt/openrua/simulators/`
(`{root}` in a shell tail) and the venv at `/opt/openrua/venv/`
(`{venv}`); a file the shell needs from the declaration's own
directory, `<name>/` next to `<name>.yaml`, is copied in as `{here}`.
The package in the venv is your checkout when `openrua` runs from one,
else the release at the same version from PyPI. The container mounts
one host directory, the trial's own (config, logs, frames); nothing
else crosses the boundary.

Builds are large: the LIBERO family and ManiSkill a few GB each,
RoboTwin about 12 GB, VLABench about 9 GB, RoboCasa365 about 25 GB
with its kitchens. Docker's layer cache keeps a rebuild cheap when the
declaration has not changed, and the requirements layer is cached
across images (`uv`'s cache mount), so the second LIBERO fork builds
faster than the first. A download that breaks mid-way fails the build
loudly; rerun it.

Each image carries a label with the fingerprint of the Dockerfile and
files it was rendered from (`openrua.install_sha256`), the package
version and, from a checkout, the commit. `openrua doctor <robot>
--bench <benchmark>` renders the declaration again and compares: a
changed lock, patch or shell tail is reported as a stale image with
the build command that refreshes it. A trial's `provenance.json`
records the image's name and digest, which pins the simulator, its
patches and its assets in one number.

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
with `--bench ./my-bench.yaml`, and `openrua build --bench
./my-bench.yaml` makes `openrua-sim-my-bench` from its `install:`
(its lock, patches and `my-bench/` directory next to the yaml). A new
engine is a simulator file plus the engine module its `entry_point`
names: a module exposing `ENGINE`, a callable `ENGINE(env, cfg)` whose
result answers every name in `ENGINE_INTERFACE`
(`openrua/robot/sim/bridge/engines/__init__.py` lists them with their
units and conventions: joint addressing, poses, camera renders and
intrinsics, action assembly, FK). Bundled engines live under
`openrua/robot/sim/bridge/engines/` (`robosuite`); yours sits next to
your simulator file (`entry_point: ./my_engine.py`) and is passed with
`--sim ./my-sim.yaml`. A robot the engine's own assets lack is
declared under the benchmark's `scenes.robots`, as RoboCasa365 does
for `panda-omron`.
