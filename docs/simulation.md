---
summary: Where the simulator checkouts live and how a profile names them
read_when:
  - robocli doctor says a simulator venv is missing
  - You keep simulators somewhere other than ~/.robocli/simulators
  - You want GPU rendering
---

# Simulators

The simulated robots run the benchmarks' own simulators (robosuite /
MuJoCo, LIBERO-PRO, CaP-Bench, RoboCasa). Each is a checkout plus a
Python venv, mounted into the robot container at `up`. They are large
(tens of GB with assets) and are built locally under the user
directory:

```
~/.robocli/simulators/
  cap-x/            LIBERO-PRO and CaP-Bench (.venv-libero, .venv-capbench)
  robocasa365/      RoboCasa365 (.venv-robocasa)
```

Each robot profile names its venv under `machine.backend.simulator.venv`,
relative to that directory (`cap-x/.venv-libero`); an absolute path or
`~` works too if you keep simulators elsewhere. The venv must have
`robocli` installed (the container starts the bridge with `python -m
robocli.robot.sim.bridge.main` from it).

A checkout is the benchmark's own repository at a known commit, and
the venv is one the robot container can run: its Python matches the
image's ROS distro (3.12 for Jazzy, 3.10 for Humble), it is made with
`uv` (the container mounts uv's interpreter store), the benchmark's
packages are installed in it, and so is `robocli-harness`. The exact
commits, patches and locks the reported runs used ship with the
paper's experiment repository as one setup script. `robocli doctor
<robot>` reports which venv is missing and where it looked.

Rendering is EGL; with no GPU it falls back to llvmpipe (slow but
correct, and what the reported runs used). `machine.backend.gpus: true`
injects the NVIDIA driver when the host has nvidia-container-toolkit
(under podman the same flag resolves through CDI: `nvidia-ctk cdi
generate` once, see docs/podman.md).
