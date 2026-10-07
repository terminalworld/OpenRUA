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
type, `type: panda` on top so you only write what differs. Pass its
path wherever a robot name is expected; the bundled names are the
only ones looked up by name.

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
not a silent no-op. Listed ports describe the robot to the agent through
`machine.yaml`; list only what the robot actually serves. Benchmark trials
check declared interfaces through `preflight` before starting the agent.
Interactive sessions wait for graph visibility, without running the full trial
gate. Neither check establishes safe physical operation.

For a real robot, set `cameras.list: []` when the profile declares no cameras.
This omits camera
entries from `machine.yaml` and camera checks from trial preflight; it does not
disable sensors or restrict the agent's access to the graph. `null` retains
camera discovery using the documented topic convention. Real-driver trials do
not require the simulator's `/clock` or forbid unlisted gripper interfaces.

Startup waits for ROS graph visibility for up to five minutes. A stalled
readiness command is stopped at that deadline, and its last output is included
in the error. Failed startup cleans up resources created for that attempt;
an already-running external robot driver remains under its original owner.

Two shapes the example does not show: a machine with several arms
lists them under `arms:` (one entry each with its own joints, limits,
gripper and ports) instead of `arm:`/`gripper:`/`ports:`, and a mobile
manipulator adds `base:` (its `cmd_vel` and `odom` topics) so the
workspace docs describe driving as well as reaching. Every key and its
meaning is in [config.md](config.md#machine).

## Bringing it up

To open OpenRUA's shared terminal chat with your completed profile:

```bash
openrua --robot ./ur5e.yaml --sim '' --bench '' --ros-domain 7
```

The empty simulator and benchmark selections override any saved simulation
defaults. OpenRUA validates the profile and prepares missing agent images before
opening chat. It preserves your profile instead of sending it through the
simulation setup menu. This is a software entry point; physical robot operation
still requires validation on your hardware.

For the native coding agent terminal or explicit resource commands:

```bash
openrua doctor ./ur5e.yaml         # images, login, and that the file loads
openrua run ./ur5e.yaml --ros-domain 7 "move the arm to the home pose"   # or up + agent + down
openrua config set --robot ./ur5e.yaml                                    # or make it the default
```

The sandbox joins the host network and reaches the graph the way
`backend.discovery` says. `--ros-domain` selects the same ROS domain for the
sandbox and a driver started by `backend.launch`, whether on the host or in
`backend.image`. For an already-running external driver, select its existing
domain; OpenRUA does not reconfigure it. The default domain is 0. On a real robot there is no simulator to ask,
so the task sentence is the one you give (`run`'s prompt, or `--task`
on `up` and `bench`), and a trial's `result.json` records `success: null`
with `verdict: not_applicable`; preflight, the agent, the transcript and
the provenance are the same as in simulation. A real robot takes no
`--sim`; it may take `--bench` to run a benchmark's task list on it.

For an independently controlled native terminal, keep `openrua up` running and
use `openrua agent --name <name>` in another terminal. `openrua down --name <name>`
asks that resource owner to stop the sandbox and any driver it launched. The
same external stop works while `openrua run` is open. Driver launch commands
must stay in the foreground so their owner can wait for shutdown; omit `launch`
to join an externally managed graph, which OpenRUA leaves running.

Stopping retains the workspace and native profile. A failed shutdown is reported
and can be retried; it is not marked stopped. If the owner has crashed, inspect
and stop its remaining resources explicitly. OpenRUA does not signal a saved PID
or infer that a host driver stopped just because the sandbox container is gone.
Delete retained materials separately with `openrua clean --name <name>` after
shutdown. This lifecycle handling does not constitute a physical emergency stop.

One difference from simulation to know about: a sandbox on the host
network is not on an internal docker network, so the proxy is the route
the agent is told to use, not a wall it cannot get around. For a scored
campaign, simulation keeps that isolation; on hardware the point of the
sandbox is the same toolchain and the same workspace, not containment.

## Same-host Fast DDS discovery

If a driver sees its own ROS node but the sandbox cannot, first check that both
use the same ROS domain. With host networking, Fast DDS may also select shared
memory even though the containers have separate IPC namespaces; see the
[native Docker guidance](https://fast-dds.docs.eprosima.com/en/3.4.x/docker/shm_docker.html).

To retain separate IPC namespaces, a Fast DDS driver can use the
[UDP-only profile](../examples/ros/fastdds-udp.xml). Place the file where the
driver runs and prefix its launch command with:

```sh
FASTRTPS_DEFAULT_PROFILES_FILE=/absolute/path/to/fastdds-udp.xml ros2 launch ...
```

For `backend.image`, the file must exist inside that image. For an external
driver, configure it through its existing deployment process. OpenRUA does not
change an external driver's middleware or enable host IPC automatically. The
profile was checked with software ROS nodes on Humble, including both owned and
external driver lifecycles; it is not physical-robot validation.

## A simulated robot of your own

A simulated robot is a robot type plus an entry under a simulator's
`robots:` (how the engine drives it: controller, gains, joint-name
map). Copy `openrua/configs/robots/panda.yaml` for the type and add
your robot to the simulator file (`openrua/configs/simulators/<engine>.yaml`
in a pull request, or your own copy passed with `--sim ./<engine>.yaml`); a benchmark whose
assets bring the body declares it under `scenes.robots` instead, as
`robocasa365` does for `panda-omron`. [simulation.md](simulation.md)
has the files and what the images hold.
