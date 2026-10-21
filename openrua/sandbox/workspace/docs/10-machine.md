# 10. The machine

Top-down: where the facts live → software stack → runtime environment →
timing.

## 1. Machine facts live in `machine.yaml`

This doc set is generic; everything specific to THIS machine; robot
model, joint names and limits, port names, frames, planning setup; is
in **`machine.yaml`** at the workspace root. Read it first. Structure:

- `robot`; model and one-line description
- `frames`; the TF names for `world`, the arm `base`, the `hand`
- `actuators`; a list of typed write-ports, each entry:
  `kind` (`joint_trajectory` / `cartesian_twist` / `gripper` / …),
  `port`, message `type`, plus kind-specific facts (joint names,
  limits, open/closed positions)
- `sensors`; a list of typed read-ports (`joint_state`, `wrench`,
  `camera_set`, …)
- `simulation`; only on a simulated machine: how its clock, cameras and
  controllers behave where a real robot's would not
- `notes`; observations about this machine that no field can say,
  grouped by the kind they concern (`timing`, `gripper`, `planning`, …)

Every key in it is defined in a comment on its own line, units included;
`machine.yaml` explains itself and needs nothing from this doc.
- `planning`; MoveIt entry points, if present

Each `kind` has a how-to section in [20-perception.md](20-perception.md)
or [30-action.md](30-action.md). The manifest is an index of the ROS
graph; `ros2 topic list` / `ros2 action list` show the same ports live
(and anything beyond the manifest).

The manifest is what the machine's owner declared, and an owner can be
wrong. Check a fact against the running system before relying on it
(`ros2 topic info`, `ros2 interface show`, `ros2 param get`, a frame, a
reading). When they disagree, work with what the system shows and
record the difference in `machine-discrepancies.yaml` at the workspace
root, one entry per fact, so a person can correct the config:

```yaml
- key: sensors[kind=camera, name=d405].depth     # where in machine.yaml
  declared: /d405/depth/image_raw
  observed: /d405/aligned_depth_to_color/image_raw
  evidence: ros2 topic list shows no /d405/depth/image_raw; the aligned
    topic publishes 16UC1 at 30 Hz
```

Leave `machine.yaml` itself as it is.

## 2. Software stack

- **ROS 2** is the control middleware: everything is a topic, service
  or action. **`ros2` CLI** for shell access, **`rclpy`** for Python.
- The `ros2` CLI verbs that cover daily work:

  | Verb | Does |
  |---|---|
  | `ros2 topic list` / `info <t>` / `echo <t> --once` | find, inspect, read topics |
  | `ros2 topic pub [--rate N] <t> <type> '{...}'` | publish (once or streaming) |
  | `ros2 action list` / `send_goal <a> <type> '{...}'` | find and call actions |
  | `ros2 service list` / `call <s> <type> '{...}'` | find and call services |
  | `ros2 node list` | who is running |
  | `ros2 interface show <type>` | field layout of any message type |
  | `ros2 run tf2_ros tf2_echo <from> <to>` | read a TF transform |

- Read `machine.yaml` in Python:
  ```python
  import yaml
  M = yaml.safe_load(open("machine.yaml"))
  fjt = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
  ```

## 3. Runtime environment

- ROS is **already sourced** in every shell (bash and sh alike);
  no need to `source` manually.
- Python 3 has `rclpy`, `numpy`, `scipy`, `cv2`, `PIL`, `cv_bridge`,
  `tf2_ros`, `control_msgs`, `moveit_msgs`, `PyKDL`, `urdf_parser_py`.
- **This directory (`/workspace`) is yours**: write scripts, images,
  notes here.
- **No internet.** `pip install` and web access fail; everything needed
  is installed.

## 4. Timing

How time passes on this machine is in `machine.yaml` (`simulation.clock`
on a simulated one, `notes.timing` when it has notes). Either way:

- confirm every motion by re-reading sensors after the command
  completes;
- a client-side timeout is not a motion failure; read the joint state
  before sending the motion again.
