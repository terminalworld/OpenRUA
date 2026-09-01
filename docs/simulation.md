# Simulation substrates

The simulated bodies run the benchmarks' own simulators (robosuite /
MuJoCo, LIBERO-PRO, CaP-Bench, RoboCasa). Those live in a *substrate*:
a checkout plus a Python venv, mounted into the robot container at
`up`. Substrates are large (tens of GB with assets) and are built
locally under `substrates/` (gitignored):

```
substrates/
  cap-x/            LIBERO-PRO and CaP-Bench (.venv-libero, .venv-capbench)
  robocasa365/      RoboCasa365 (.venv-robocasa)
```

Each robot profile names its venv under `machine.body.substrate.venv`;
an absolute path or `~` works too if you keep substrates elsewhere.

Build recipes for each substrate are being written up here; until
then `robocli doctor` reports which venv is missing.

Rendering is EGL; with no GPU it falls back to llvmpipe (slow but
correct, and what the reported runs used). `machine.body.gpus: true`
injects the NVIDIA driver when the host has nvidia-container-toolkit.
