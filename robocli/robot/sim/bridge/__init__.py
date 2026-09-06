"""The bridge: the simulated robot's own software. Runs inside the container.

- ``environments/``  per-benchmark loaders behind one plug shape (build
                     the env through the benchmark's own factory, speak its
                     init-state/reset protocol, call its original predicate;
                     zero ROS) and ``worker.py``, the single env-owner thread
- ``ros/``           live env -> the robot's ROS 2 surface (zero truth); the
                     simulation-side stand-in for a vendor driver
- ``rpc.py``         the host's control line: per-step success latch and
                     the answering verbs
- ``main.py``        the container's first process (``robocli-bridge``)

Self-contained by machine-enforced contract: nothing here imports any
other part of robocli (the per-trial config arrives already resolved as
a file parameter; the peers profile arrives rendered). All internal
imports are relative. environments, ros and rpc never see each other:
env, loader and worker flow as plain parameters, passed by main.py.
"""
