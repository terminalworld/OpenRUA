"""ROS graph: a live simulator env -> the robot's ROS 2 surface.

The agent-facing side of the machine: joint states, cameras, TF, wrench,
the command ports (FollowJointTrajectory, twist servo, gripper action),
and the paused-clock stepping that ties graph commands to sim steps.
Graph shape copies franka_ros2 naming verbatim; zero invention.

Zero truth, zero evaluation: no success predicate, no reset, no episode
state lives here. Zero knowledge of the sibling environments and engines packages: the
bound engine, the sim-owner thread, and the config arrive as plain
constructor parameters (main.py does the passing); nothing here names
a physics engine.
"""
