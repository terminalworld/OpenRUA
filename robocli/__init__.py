"""RoboCLI: operating robots through their native command-line interface.

Folders are units; top-level files compose or are shared (architecture
contract in pyproject.toml, enforced by tests/test_layering.py):

- ``robot``       the machine the agent operates, provided by a backend:
                  sim/ (host side + bridge/, the simulated robot's own
                  software, container) or real/ (a launch command and a
                  handle that waits for the graph)
- ``sandbox``     the agent's terminal: reachable machine + config ->
                  live container (native CLI/rclpy inside)
- ``proxy``       one shared whitelist proxy to the internet
- ``agents``      the agent contract, the registry (manifests + hooks),
                  the launcher, the prompts
- ``runner``      running trials: bring-up, preflight (every promise the
                  workspace docs make, verified before the agent starts;
                  red = trial refused), the operator, the verdict, the
                  record (sole runs/ writer), the lock
- ``cli``, ``doctor``  the command line and the install check
- ``config``, ``errors``, ``testing``  shared leaves

robocli.robot.sim.bridge is container-side (rclpy); host code never
imports it and reaches it only over DDS (the agent) or its stdio line
(the runner).
"""
from importlib.metadata import PackageNotFoundError, version as _version

try:
    __version__ = _version("robocli-harness")
except PackageNotFoundError:  # a checkout that was never installed
    __version__ = "0+unknown"
