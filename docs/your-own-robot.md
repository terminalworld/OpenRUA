# Use your own robot

A robot is a YAML profile. The shipped ones under `robots/` are
simulated; a real robot's profile is the same file minus the simulator
body.

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

Every port you list becomes a promise: `precheck` verifies it is
served before an agent boards, and the manual describes it to the
agent. List only what the robot actually serves.

## Bringing it up

```bash
robocli up --robot ./ur5e.yaml --ros-domain 7
```

The sandbox joins the robot's DDS domain. The robot's ROS 2 graph must
be reachable from the host running RoboCLI (same network, or a DDS
discovery server / static peers, which the sandbox `up` accepts as
`--static-peer`). Then, as always:

```bash
robocli agent "move the arm to the home pose and open the gripper"
```

Real-robot support is being brought up profile by profile; the
simulated profiles are the reference for what a complete `machine:`
section looks like.
