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
topics, actions and services with their types, the joints each
trajectory and gripper controller drives, `/robot_description` and
`/robot_description_semantic`, and prints a profile: the arm's joints in
its controller's order with their limits, the gripper's joint, base and
hand frames, the planning group, the ports it recognised, and one entry
per camera that publishes a CameraInfo. Lines marked `TODO` need you: the
model name, a one-line description, the world frame, which end of the
gripper's travel is open and what its driver does with a goal, each
camera's type and mount, and which of the found ports to keep. A `TODO`
is not a valid value, so the profile does not load until you decide.
Delete any port the robot does not actually serve. A robot that is already up under OpenRUA can be
probed from its own sandbox: `openrua probe --name openrua`.

Use `--ros-domain` and `--distro` to match the running robot, for example
`openrua probe --host --ros-domain 7 --distro humble > ur5e.yaml`.
The draft preserves this ROS distribution. Probing requires the matching sandbox
image and access to the ROS graph; it does not require a model login or the
OpenRUA proxy. The selected domain is used for this probe; pass it again when
starting a session. With `--name`, the probe reuses the existing sandbox and
leaves the deployment distribution as a TODO to review in the draft.
A failed topic, action or service query stops the probe and reports the native
error instead of producing a profile from incomplete query results. A missing
robot description is optional: the draft leaves joint and frame facts for you
to complete. Published URDFs are parsed from the native topic output, excluding
its trailing document separator. Check the command's exit status before using the redirected file.

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
  gripper:                   # the GripperCommand interface, then what the driver does with a goal
    joint: robotiq_85_left_knuckle_joint   # reports the gripper's position in /joint_states
    unit: rad                # this driver commands the knuckle angle, not a width
    open: 0.0
    closed: 0.7929           # which way closes follows from open and closed
    max_effort: 100.0
    holds_partway: true      # try a position between the two and watch
    effort_honoured: false   # try a low max_effort on something soft
  planning: {moveit: true, move_action: /move_action,
             ik_service: /compute_ik, group: ur_manipulator,
             planning_frame: base_link}
  ports:
    trajectory: /scaled_joint_trajectory_controller/follow_joint_trajectory
    gripper: /robotiq_gripper_controller/gripper_cmd
    twist: /servo_node/delta_twist_cmds
    wrench: /force_torque_sensor_broadcaster/wrench
  cameras:
    list:                    # each camera's name here, its type, and where it sits
      - {name: wrist_camera, type: ./cameras/realsense-d435.yaml, mount: wrist}
  workspace_template: workspace
```

The agent can only call a service or read a topic whose type definition
is installed in its sandbox. When the robot speaks a vendor's own
messages (a Franka's `franka_msgs` for its robot mode and error
recovery, say), declare how to install them, one line run as root when
the sandbox image is built; a ROS workspace built under
`/opt/openrua/overlays/<name>` is sourced in every shell:

```yaml
  backend:
    sandbox_install: >-
      apt-get update && apt-get install -y --no-install-recommends git
      ros-humble-controller-manager-msgs ros-humble-ros2controlcli
      && git clone https://github.com/vendor/driver /tmp/src/driver
      && . /opt/ros/humble/setup.sh && cd /tmp/src
      && colcon build --packages-select vendor_msgs --install-base /opt/openrua/overlays/vendor
```

Then `openrua build sandbox --robot ./ur5e.yaml` builds that robot's own
sandbox image (its name carries a hash of the install), and `doctor`
says when the install has changed since.

A camera type is a file of its own, next to the robot's, saying what one
camera with one driver publishes; `{name}` stands for the camera's name
on the machine:

```yaml
# cameras/realsense-d435.yaml
model: Intel RealSense D435
driver: realsense2_camera (align_depth.enable:=true)
topics:
  color: /{name}/color/image_raw
  info: /{name}/color/camera_info
  depth: /{name}/aligned_depth_to_color/image_raw
depth: {encoding: 16UC1, aligned_to_color: true}   # REP 118: 16UC1 is millimetres
```

OpenRUA bundles only its simulation bridge's camera type; a real camera's
type lives with the robot that uses it.

Every key is checked against the schema (`openrua config schema`
prints all of them with their meaning); a misspelled key is an error,
not a silent no-op. Listed ports describe the robot to the agent through
`machine.yaml`; list only what the robot actually serves. Benchmark trials
check declared interfaces through `preflight` before starting the agent.
Interactive sessions wait for graph visibility, without running the full trial
gate. Neither check establishes safe physical operation.

A real robot lists its cameras, each with its type; `cameras.list: []`
declares none. This omits camera entries from `machine.yaml` and camera
checks from trial preflight; it does not disable sensors or restrict the
agent's access to the graph. Real-driver trials do
not require the simulator's `/clock` or forbid unlisted gripper interfaces.

What no field can say goes under `notes:`, by the manifest kind it
concerns (`timing`, `joint_trajectory`, `gripper`, `camera`, `planning`,
...): how close the arm gets to a target, that the gripper reports a
grasp as stalled, that the planning scene has no table. They reach the
agent in `machine.yaml`, for the kinds the robot has.

The agent is told that these facts are what you declared, to check
them against the running graph, and to write any difference it finds to
`machine-discrepancies.yaml` in its workspace (the fact's key, what you
declared, what it observed, and the evidence) without editing
`machine.yaml`. Read that file after a session and fold what is right
into the profile.

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
openrua --robot ./ur5e.yaml --ros-domain 7
```

A real robot takes no saved simulator or benchmark default; name a benchmark
with `--bench` to run its task on the robot. OpenRUA validates the profile and
prepares missing agent images before opening chat.

The launch command runs on the host in the directory of the robot's file
(`backend.cwd` to choose another), so it may name the driver's files relative
to it. Ending the session sends it SIGINT, then SIGTERM, then SIGKILL,
waiting `backend.stop_timeout_s` (30 s by default) after each of the first
two: give a driver that releases the robot before exiting the time it needs.
If the launch command exits while the session runs (a driver that crashed
after an emergency stop, say), the session says so in its service log and
the terminal, and holds the queued instructions until you look.

Leaving the terminal (Ctrl+D, Ctrl+C, /quit) does not end the session: the
robot stays under its driver and the agent keeps its sandbox. The terminal
says how to reconnect (`openrua --name <session>`) and how to end it
(`/end`, or `openrua session --name <session> end`). It preserves your profile instead of sending it through the
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
