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

Every key in it is defined in a comment on its own line, units included;
`machine.yaml` explains itself and needs nothing from this doc.
- `planning`; MoveIt entry points, if present

Each `kind` has a how-to section in [20-perception.md](20-perception.md)
or [30-action.md](30-action.md). The manifest is an index of the ROS
graph; `ros2 topic list` / `ros2 action list` show the same ports live
(and anything beyond the manifest).

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

The system clock (`/clock`) advances with robot activity, not wall time.
Consequences:

- `sleep` does not let the physical world settle;
- confirm every motion by re-reading sensors after the command
  completes, not by waiting.

## Timing facts (paused-clock machine)

- At session start the scene may not have settled yet (objects can sit
  slightly above their supports); the world only advances while commands
  execute; waiting wall-clock time changes nothing.
- A few sim-seconds of trajectory can take minutes of wall time to
  return under load. A client-side timeout is NOT a motion failure;
  verify against `/joint_states` before re-sending.
