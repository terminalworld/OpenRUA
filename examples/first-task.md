# First task in simulation

```bash
robocli doctor panda-sim             # Docker, images, simulator, login
robocli up panda-sim                 # terminal 1: stays up
robocli agent "put the bowl on the plate"    # terminal 2
```

What the agent finds in `/workspace`:

```
README.md          start here: map of the docs, a 30-second start
machine.yaml       this robot's facts: joints, limits, frames, ports
docs/10-machine.md ... 40-patterns.md
tools/             cam_snap.py, px2world.py, fjt_send.py, gripper_cmd.py, ik_move.py
```

A typical session: `ros2 topic list`, `ros2 topic echo /joint_states
--once`, a camera snapshot, a pixel-to-world lookup, an IK move, a
gripper close, another snapshot to confirm. All of it is ROS 2; the
tools are thin conveniences the agent may ignore.

`Ctrl-C` in terminal 1 powers the robot off. The workspace the agent
left behind stays under `~/.robocli/workspaces/robocli/`.
