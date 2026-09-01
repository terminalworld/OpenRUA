"""RoboCLI: operating robots through their native command-line interface.

Folders are single functions; top-level files compose or are shared
(architecture contract in pyproject.toml, machine-enforced by
tests/test_layering.py):

- ``robot``       the machine the agent operates: ground-side verbs
                  (build/up/down, host) + onboard (the simulated robot's
                  own software, container; a real robot needs only the
                  ground verbs)
- ``sandbox``     the house: reachable machine + config -> live sandbox
                  container (native CLI/rclpy inside)
- ``proxy``       the gatehouse: one shared whitelist wall to the internet
- ``agents``      the occupants: adapters + launcher + opening prompt
- ``precheck.py`` the examiner's gate: every manual promise verified
                  before the agent boards; red = trial refused
- ``record.py``   the examiner's ledger: what trial files say, secret
                  scrubbing, evidence extraction (sole runs/ writer)

robocli.robot.onboard is container-side (rclpy); host code never
imports it and reaches it only over DDS (the agent) or its stdio line
(the examiner, via run). Every folder is one function; run.py conducts (and owns the
config view: per-suite resolution, computed once, sent as data).
"""

__version__ = "0.1.0"
