# 30. Action

Ways to move, coarse to fine, keyed by actuator `kind` in `machine.yaml`
(`actuators` list; a dual-arm machine simply has two `joint_trajectory`
entries; check what THIS machine has). Every section: what → port →
CLI → rclpy → tool → notes.

## A. Joint trajectory (kind `joint_trajectory` (the workhorse))

- **What**: track a joint-space trajectory; the primary way to move.
- **Port**: `machine.yaml → actuators[kind=joint_trajectory]` gives the
  action name, the joint names (in order) and their limits.
- **Tool**: `python3 tools/action/fjt_send.py <p1,...,pN> <seconds>`
- **rclpy**: `ActionClient` (`control_msgs/action/FollowJointTrajectory`)
  sending a trajectory whose points have `positions` (one per joint,
  manifest order) and `time_from_start`; several points = one smooth
  pass (see `fjt_send.py` for the skeleton).
- **CLI**: possible via `ros2 action send_goal`, but the YAML is
  unwieldy; prefer the tool/rclpy.
- **Notes**:
  - 2–4 s for large moves; too-short durations overshoot;
  - action result returning ≠ target reached exactly; re-read the
    joint state and compare;
  - nonzero `error_code` = tracking problem (see 40-patterns.md).

## B. Cartesian servo (kind `cartesian_twist`)

- **What**: stream end-effector velocities for small precise motion.
- **Port**: `machine.yaml → actuators[kind=cartesian_twist]`; topic,
  and the `frame` to stamp commands with (linear is m/s).
- **CLI**: `ros2 topic pub --rate 20 <port> geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: <frame>}, twist: {linear: {z: -0.05}}}"`;
  Ctrl-C to stop the stream.
- **rclpy** (the world only advances while commands stream; publish in
  a loop, then re-check sensors):
  ```python
  import rclpy, yaml
  from geometry_msgs.msg import TwistStamped
  tw = next(a for a in yaml.safe_load(open("machine.yaml"))["actuators"]
            if a["kind"] == "cartesian_twist")
  rclpy.init(); node = rclpy.create_node("nudge")
  pub = node.create_publisher(TwistStamped, tw["port"], 10)
  msg = TwistStamped(); msg.header.frame_id = tw["frame"]
  msg.twist.linear.z = -0.05                 # 5 cm/s downward
  for _ in range(20):                        # ~1 s of motion
      pub.publish(msg); rclpy.spin_once(node, timeout_sec=0.05)
  ```
- **Notes**: burst → sense → correct beats one long blind stream.

## C. Gripper (kind `gripper`)

- **What**: open/close the fingers. The manifest entry carries this
  gripper's facts (`open_m`, `closed_m`, `stops_at`, `max_effort`),
  each with its definition written on the line.
- **Port**: `machine.yaml → actuators[kind=gripper]`
  (`control_msgs/action/GripperCommand`).
- **Tool**: `python3 tools/action/gripper_cmd.py <width_m>`
- **CLI** (fill `<port>`, `open_m`/`closed_m`, `max_effort` from the
  manifest entry):
  `ros2 action send_goal <port> control_msgs/action/GripperCommand \
  "{command: {position: <m>, max_effort: <n>}}"`
- **Notes**: after closing on something, finger positions in the joint
  state stop above `closed_m`; that gap is your grasp evidence.

## D. Mobile base (kind `base_twist` (only on mobile machines))

- **What**: drive the base with body-frame velocities. Absent from the
  manifest = this machine's base is fixed; skip this section.
- **Port**: `machine.yaml → actuators[kind=base_twist]`; plain
  `geometry_msgs/msg/Twist` (`/cmd_vel` convention): `linear.x` forward,
  `linear.y` left (holonomic bases only; check `type`), `angular.z`
  counter-clockwise yaw.
- **Tool**: `python3 tools/action/base_goto.py <x> <y> [yaw]`;
  closed-loop on odometry; exits with the last pose as fact on
  stall.
- **CLI**: `ros2 topic pub --rate 10 <port> geometry_msgs/msg/Twist \
  "{linear: {x: 0.3}}"`; Ctrl-C to stop.
- **Semantics**: **one message = one control tick** of motion at that
  velocity. Messages queue up to a small depth (4); anything beyond is
  dropped, so flooding buys nothing. Queued ticks still execute after
  you stop publishing; pace yourself by odometry feedback, not by
  message count or wall time.
- **Notes**:
  - closed-loop is the only reliable pattern: publish a few ticks →
    read odometry (20-perception A4) → correct → repeat;
  - drive and arm motion are separate commands; position the base
    first, then work the arm;
  - operational norm for go-to/docking jobs: park the base directly in
    front of the target, within arm's reach (close enough to work on it
    without re-driving), squarely facing it. "Near" is not parked.

## E. Planning (MoveIt): `machine.yaml → planning`

- **What**: collision-aware planning and IK when raw trajectories are
  not enough (present when `planning.moveit` is true).
- **Ports**: `planning.move_action` (plan+execute,
  `moveit_msgs/action/MoveGroup`, group `planning.group`);
  `planning.ik_service` (`moveit_msgs/srv/GetPositionIK`); Cartesian
  pose → joint positions.
- **Tool**: `python3 tools/action/ik_move.py <x> <y> <z> <qx> <qy> <qz>
  <qw> [seconds] [--at tcp]`; IK then trajectory, loud on failure.
- **Pattern**: IK for the target pose, then A (trajectory) to go there;
  or `move_action` end-to-end when obstacles matter.
- **Notes**: check the returned `error_code` (1 = success); frame
  behavior is NOT what headers suggest; see "Planning facts" below.

## Actuation facts

- Gripper commands are PER-FINGER positions: the opening between the
  fingers is twice the commanded value (nominals in machine.yaml).
  What this machine does with the commanded value is its own fact, not
  a general rule: read `stops_at` on the manifest's gripper entry. Gap
  readings need a few settle ticks; the measured maximum opening can sit
  slightly under nominal.
- A trajectory tolerance-violation result on a long goal is often
  controller lag, not an obstruction: resending the same goal typically
  converges.
- Arm contact and reaction forces can displace the mobile base by
  centimeters; re-read odometry after arm work before trusting stored
  base poses.

## Planning facts (MoveIt)

- The planner's model frame is the ARM BASE even where message headers
  say "world" (machine.yaml planning.planning_frame). Leave `frame_id`
  EMPTY in IK requests; a base-frame label is rejected (-21).
- The planning scene starts EMPTY; no world collision objects exist
  until you publish them; collision meshes carry roughly a centimeter
  of padding, so trust measurements over nominals.
- IK/FK requests HANG if their robot_state carries non-arm joints
  (finger joints included): seed with the arm joints only.
- IK can fail silently (no error, no motion) and non-monotonically
  (nearby poses flip feasibility). Seed requests with the current
  configuration to keep branch continuity, and verify motion actually
  happened from `/joint_states`.
