---
summary: Where the simulator checkouts live and how a profile names them
read_when:
  - openrua doctor says a simulator venv is missing
  - You keep simulators somewhere other than ~/.openrua/simulators
  - You want GPU rendering
---

# Simulators

The simulated robots run the benchmarks' own simulators (robosuite /
MuJoCo, LIBERO-PRO, CaP-Bench, RoboCasa). Each is a checkout plus a
Python venv, mounted into the robot container at `up`. They are large
(tens of GB with assets) and are built locally under the user
directory:

```
~/.openrua/simulators/
  cap-x/            LIBERO-PRO and CaP-Bench (.venv-libero, .venv-capbench)
  robocasa365/      RoboCasa365 (.venv-robocasa)
```

Each robot profile names its venv under `machine.backend.simulator.venv`,
relative to that directory (`cap-x/.venv-libero`); an absolute path or
`~` works too if you keep simulators elsewhere. The venv must have
`openrua` installed (the container starts the bridge with `python -m
openrua.robot.sim.bridge.main` from it).

A checkout is the benchmark's own repository at a known commit, and
the venv is one the robot container can run: its Python matches the
image's ROS distro (3.12 for Jazzy, 3.10 for Humble), it is made with
`uv` (the container mounts uv's interpreter store), the benchmark's
packages are installed in it, and so is `openrua`. `openrua doctor
<robot>` reports which venv is missing and where it looked.

Rendering is EGL; with no GPU it falls back to llvmpipe (slow but
correct, and what the reported runs used). `machine.backend.gpus: true`
injects the NVIDIA driver when the host has nvidia-container-toolkit
(under podman the same flag resolves through CDI: `nvidia-ctk cdi
generate` once, see docs/podman.md).
