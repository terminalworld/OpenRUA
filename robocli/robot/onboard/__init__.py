"""Onboard: the simulated robot's own software. Runs INSIDE the body.

- ``environment/``  per-benchmark loaders behind one plug shape:
                  build the env through the benchmark's own factory,
                  speak its init-state/reset protocol, call its
                  original predicate (zero ROS)
- ``ros_graph/``  live env -> the robot's ROS 2 surface (zero truth);
                  the simulation-side stand-in for a vendor driver
- ``monitor/``    watches every step (success latch) and answers the
                  host's stdio questions
- ``boot.py``     the body's first process
                  (``python -m robocli.robot.onboard.boot``)

Self-contained by machine-enforced contract: nothing here imports any
other part of robocli (the per-trial config arrives ALREADY RESOLVED as
a file parameter; the peers profile arrives rendered). All internal
imports are relative, so at release this folder bakes into the body
image as a top-level package (``python -m onboard.boot``) unchanged.
environment, ros_graph and monitor never see each other: env, loader,
and sim-thread flow as plain parameters, passed by boot.py.
"""
