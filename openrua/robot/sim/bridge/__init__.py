"""The bridge: the simulated robot's own software. Runs inside the container.

- ``environments/``  per-benchmark loaders behind one plug shape (build
                     the env through the benchmark's own factory, speak its
                     init-state/reset protocol, call its original predicate;
                     zero ROS) and ``worker.py``, the single env-owner thread
- ``engines/``       per-engine views of the world behind one plug shape
                     (joint addressing, poses, cameras, action assembly,
                     FK; zero ROS); the one place a physics engine is named
- ``ros/``           bound engine -> the robot's ROS 2 surface (zero truth);
                     the simulation-side stand-in for a vendor driver
- ``rpc.py``         the host's control line: per-step success latch and
                     the answering verbs
- ``plug.py``        how a resolved config's loader / engine line is imported
- ``main.py``        the container's first process (``openrua-bridge``)

Self-contained by machine-enforced contract: nothing here imports any
other part of openrua (the per-trial config arrives already resolved as
a file parameter; the peers profile arrives rendered). All internal
imports are relative. environments, engines, ros and rpc never see each
other: env, loader, engine and worker flow as plain parameters, passed
by main.py.
"""
