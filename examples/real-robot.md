# A real robot

1. Write a profile (see `docs/your-own-robot.md`) with the joints,
   frames, gripper and ports your robot serves.
2. Make the robot's ROS 2 graph reachable from the host running
   RoboCLI (same LAN, same `ROS_DOMAIN_ID`).
3. `robocli up ./my-robot.yaml --ros-domain <id>`
4. `robocli agent "..."`

The preflight runs the same checks as in simulation: the ports you
listed must be served, joint names must match the manual, TF and
camera frames must flow. If a check fails, `up` refuses and says which
promise the robot did not keep.
