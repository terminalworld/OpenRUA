---
summary: Describe your robot in one YAML profile and bring it up
read_when:
  - You have a ROS 2 robot (real or simulated) that is not one of the bundled profiles
  - You want to know what the agent's machine.yaml is generated from
---

# Use your own robot

A robot is a YAML profile. The bundled ones (`openrua robots`) are
simulated; a real robot's profile is the same file with a `real`
backend instead of a simulator. Put yours in
`~/.openrua/robots/<name>.yaml` and it is found by name, or pass its
path.

## Draft it from the graph

Most of the profile is already on the robot's ROS 2 graph. With the
robot's stack running and reachable from this host:

```bash
openrua probe --host > ur5e.yaml            # multicast on the host network
openrua probe --static-peers 192.168.1.20 > ur5e.yaml
openrua probe --discovery-server 192.168.1.20:11811 > ur5e.yaml
```

`probe` starts a throwaway sandbox that can see the graph, reads the
topics, actions and services and `/robot_description`, and prints a
profile: joint names and limits, base and hand frames, the ports it
recognised, cameras, and whether MoveIt is up. Lines marked `TODO` need
you: the model name, a one-line description, the planning group, and
which of the found ports to keep. Delete any port the robot does not
actually serve. A robot that is already up under OpenRUA can be
probed from its own sandbox: `openrua probe --name openrua`.

## What the agent reads

`openrua up` generates the agent's `machine.yaml` from the profile's
`machine:` section. The agent's docs explain how to read it; you supply
the facts:

```yaml
machine:
  backend:
    kind: real
    ros_distro: humble       # what the robot runs; the sandbox image follows (openrua-sandbox-humble)
    launch: ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.1.20   # optional; omit if the graph is already up
    image: null              # or a docker image the launch command runs in (host network), for a driver on another ROS release
    discovery:
      network: host          # or static_peers: [192.168.1.20] / discovery_server: 192.168.1.20:11811
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

Every key is checked against the schema (`openrua config schema`
prints all of them with their meaning); a misspelled key is an error,
not a silent no-op. Every port you list becomes a promise: `preflight`
verifies it is served before the agent starts, and the workspace docs
describe it to the agent. List only what the robot actually serves.

Two shapes the example does not show: a machine with several arms
lists them under `arms:` (one entry each with its own joints, limits,
gripper and ports) instead of `arm:`/`gripper:`/`ports:`, and a mobile
manipulator adds `base:` (its `cmd_vel` and `odom` topics) so the
workspace docs describe driving as well as reaching. Every key and its
meaning is in [config.md](config.md#machine).

## Bringing it up

```bash
cp ur5e.yaml ~/.openrua/robots/
openrua doctor ur5e                # images, login, and that the profile loads
openrua up ur5e --ros-domain 7 --task "move the arm to the home pose"
```

The sandbox joins the host network and reaches the graph the way
`backend.discovery` says. On a real robot there is no simulator to ask,
so the task sentence comes from `--task` (on `up` and on `run`), and a
trial's `result.json` records `success: null` with `verdict:
not_applicable`; preflight, the agent, the transcript and the provenance
are the same as in simulation. Then, as always:

```bash
openrua agent "move the arm to the home pose and open the gripper"
```

One difference from simulation to know about: a sandbox on the host
network is not on an internal docker network, so the proxy is the route
the agent is told to use, not a wall it cannot get around. For a scored
campaign, simulation keeps that isolation; on hardware the point of the
sandbox is the same toolchain and the same workspace, not containment.

## A simulated robot of your own

The same profile with a `sim` backend names a simulator venv and its
`ros_distro` (the robot and sandbox images follow from it); the bundled `panda-sim`, `panda-sim-humble` and
`panda-omron-sim` are the templates, and [simulation.md](simulation.md)
says where the venvs live. Adding a benchmark the bridge does not know
is a loader under `robot/sim/bridge/environments/`, described in
[architecture.md](architecture.md).
