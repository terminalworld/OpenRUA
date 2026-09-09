---
summary: Describe your robot in one YAML file and bring it up
read_when:
  - You have a ROS 2 robot (real or simulated) that is not one of the bundled ones
  - You want to know what the agent's machine.yaml is generated from
---

# Use your own robot

A robot is a YAML file under `robots/`. The bundled ones (`openrua
robots`) are robot *types*: the facts true of a Franka Panda wherever it
runs, with no simulator in them; a simulator file says how it embodies
them. Your real robot is an *instance*: the same kind of file with a
`machine:` section carrying its `real` backend and, when it is a bundled
type, `type: panda` on top so you only write what differs. Put yours in
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

`openrua up` generates the agent's `machine.yaml` from the assembled
`machine:` section. The agent's docs explain how to read it; you supply
the facts:

```yaml
# type: panda            # an instance of a bundled type: its facts come first,
                         # and this file writes over them (then delete the
                         # facts below that the type already supplies)
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
openrua doctor ur5e                # images, login, and that the file loads
openrua run ur5e --ros-domain 7 "move the arm to the home pose"      # or up + agent + down
```

The sandbox joins the host network and reaches the graph the way
`backend.discovery` says. On a real robot there is no simulator to ask,
so the task sentence is the one you give (`run`'s prompt, or `--task`
on `up` and `bench`), and a trial's `result.json` records `success: null`
with `verdict: not_applicable`; preflight, the agent, the transcript and
the provenance are the same as in simulation. A real robot takes no
`--sim`; it may take `--bench` to run a benchmark's task list on it.

One difference from simulation to know about: a sandbox on the host
network is not on an internal docker network, so the proxy is the route
the agent is told to use, not a wall it cannot get around. For a scored
campaign, simulation keeps that isolation; on hardware the point of the
sandbox is the same toolchain and the same workspace, not containment.

## A simulated robot of your own

A simulated robot is a robot type plus an entry under a simulator's
`robots:` (how the engine drives it: controller, gains, joint-name
map). Copy `openrua/configs/robots/panda.yaml` for the type and add
your robot to `~/.openrua/simulators/<engine>.yaml`; a benchmark whose
assets bring the body declares it under `scenes.robots` instead, as
`robocasa365` does for `panda-omron`. [simulation.md](simulation.md)
has the files and where the installs live.
