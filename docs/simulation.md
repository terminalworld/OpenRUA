---
summary: Where the simulator checkouts live and how a profile names them
read_when:
  - robocli doctor says a substrate venv is missing
  - You keep simulators somewhere other than ~/.robocli/substrates
  - You want GPU rendering
---

# Simulation substrates

The simulated bodies run the benchmarks' own simulators (robosuite /
MuJoCo, LIBERO-PRO, CaP-Bench, RoboCasa). Those live in a *substrate*:
a checkout plus a Python venv, mounted into the robot container at
`up`. Substrates are large (tens of GB with assets) and are built
locally under the user directory:

```
~/.robocli/substrates/
  cap-x/            LIBERO-PRO and CaP-Bench (.venv-libero, .venv-capbench)
  robocasa365/      RoboCasa365 (.venv-robocasa)
```

Each robot profile names its venv under `machine.body.substrate.venv`,
relative to that directory (`cap-x/.venv-libero`); an absolute path or
`~` works too if you keep substrates elsewhere. The venv must have
`robocli` installed (the body boots with `python -m
robocli.robot.onboard.boot` from it).

Build recipes for each substrate are being written up here; until
then `robocli doctor <robot>` reports which venv is missing and where
it looked.

Rendering is EGL; with no GPU it falls back to llvmpipe (slow but
correct, and what the reported runs used). `machine.body.gpus: true`
injects the NVIDIA driver when the host has nvidia-container-toolkit.
