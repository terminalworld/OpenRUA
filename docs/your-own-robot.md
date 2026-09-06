---
summary: Describe your robot in one YAML profile and bring it up
read_when:
  - You have a ROS 2 robot (real or simulated) that is not one of the bundled profiles
  - You want to know what the agent's machine.yaml is generated from
---

# Use your own robot

A robot is a YAML profile. The bundled ones (`robocli robots`) are
simulated; a real robot's profile is the same file minus the simulator
body. Put yours in `~/.robocli/robots/<name>.yaml` and it is found by
name, or pass its path.

## What the agent reads

`robocli up` generates the agent's `machine.yaml` from the profile's
`machine:` section. The agent's docs explain how to read it; you supply
the facts:

```yaml
machine:
  robot:
    model: Universal Robots UR5e
    description: 6-joint arm with a Robotiq 2F-85 gripper
  frames: {world: world, base: base_link, hand: tool0}
  arm:
    joints: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint,
             wrist_1_joint, wrist_2_joint, wrist_3_joint]
    limits_rad: [[-6.28, 6.28], [-6.28, 6.28], [-3.14, 3.14],
                 [-6.28, 6.28], [-6.28, 6.28], [-6.28, 6.28]]
  gripper: {open_m: 0.085, closed_m: 0.0, max_effort: 100.0,
            stops_at: [open, closed]}
  planning: {moveit: true, move_action: /move_action,
             ik_service: /compute_ik, group: ur_manipulator,
             planning_frame: base_link}
  ports:
    trajectory: /scaled_joint_trajectory_controller/follow_joint_trajectory
    gripper: /robotiq_gripper_controller/gripper_cmd
    twist: /servo_node/delta_twist_cmds
    wrench: /force_torque_sensor_broadcaster/wrench
  cameras:
    list: [wrist_camera]
  workspace_template: workspace
```

Every key is checked against the schema (`robocli config schema`
prints all of them with their meaning); a misspelled key is an error,
not a silent no-op. Every port you list becomes a promise: `preflight`
verifies it is served before an agent boards, and the manual describes
it to the agent. List only what the robot actually serves.

## Bringing it up

```bash
cp ur5e.yaml ~/.robocli/robots/
robocli doctor ur5e                # images, login, and that the profile loads
robocli up ur5e --ros-domain 7     # or: robocli up ./ur5e.yaml
```

The sandbox joins the robot's DDS domain. The robot's ROS 2 graph must
be reachable from the host running RoboCLI (same network, or a DDS
discovery server / static peers, which the sandbox `up` accepts as
`--static-peer`). Then, as always:

```bash
robocli agent "move the arm to the home pose and open the gripper"
```

Real-robot support is being brought up profile by profile; the
simulated profiles (`robocli/configs/robots/`) are the reference for what a
complete `machine:` section looks like.
