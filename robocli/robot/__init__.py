"""The robot: the machine the agent operates.

Same shape as the sibling units (sandbox, proxy): birth verbs and the
blueprint at the package top, plus one sealed capsule the others don't
need. HOST-SIDE at the top, ROBOT-SIDE inside ``onboard/``:

- ``build.py``  blueprint (sim-<distro>.Dockerfile) -> body image,
                self-describing labels; ``up`` never builds.
- ``up.py``     image + per-trial parameters -> live body whose first
                process is onboard's boot; returns the three pipes.
- ``down.py``   the e-stop (conversation-dead teardown only).
- ``onboard/``  the simulated robot's own software, runs inside the
                container: world (environment), surface (ros_graph),
                measurement (monitor), and boot. Imports NOTHING from
                the rest of robocli (machine-enforced), so it bakes
                into the body image as a self-contained package at
                release.

When the physical backend arrives, its ground verbs (connect /
vendor-launch wiring / disconnect) join the package top -- a real robot
needs ONLY ground verbs, because its onboard software ships from the
vendor (franka_ros2 + MoveIt publish the same graph shape onboard/
mirrors verbatim). That asymmetry is the paper's thesis in tree form:
no adapter code exists for real hardware because none is needed.

Host code may import the top verbs (main conducts with up/down); nobody
host-side imports onboard -- the agent reaches the robot over DDS, the
examiner over the stdio pipes ``up`` handed to the conductor. Everything
host-side here consumes DATA handed in by the conductor (the resolved
per-trial config, the rendered DDS peers profile).
"""
