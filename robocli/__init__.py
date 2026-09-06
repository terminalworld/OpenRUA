"""RoboCLI: operating robots through their native command-line interface.

Folders are single functions; top-level files compose or are shared
(architecture contract in pyproject.toml, machine-enforced by
tests/test_layering.py):

- ``robot``       the machine the agent operates, provided by a backend:
                  sim/ (host side + bridge/, the simulated robot's own
                  software, container) or real/ (a launch command and a
                  handle that waits for the graph)
- ``sandbox``     the house: reachable machine + config -> live sandbox
                  container (native CLI/rclpy inside)
- ``proxy``       the gatehouse: one shared whitelist wall to the internet
- ``agents``      the occupants: adapters + launcher + opening prompt
- ``runner``      running trials: bring-up, preflight (every manual
                  promise verified before the agent starts; red = trial
                  refused), the operator, the verdict, the record (sole
                  runs/ writer), the lock

robocli.robot.sim.bridge is container-side (rclpy); host code never
imports it and reaches it only over DDS (the agent) or its stdio line
(the examiner, via run). Every folder is one function; run.py conducts (and owns the
config view: per-suite resolution, computed once, sent as data).
"""

from importlib.metadata import PackageNotFoundError, version as _version

try:
    __version__ = _version("robocli")
except PackageNotFoundError:  # a checkout that was never installed
    __version__ = "0+unknown"
