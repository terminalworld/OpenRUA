# 20. Perception

Two families: **A. proprioception** (the robot's own state) and
**B. vision** (cameras). Concrete port names come from `machine.yaml`
(`sensors` list); sections below are keyed by sensor `kind`. Every
section: what → port → CLI → rclpy → tool → notes.

## A. Proprioception

### A1. Joint state (kind `joint_state`)

- **What**: positions/velocities/efforts for every joint.
- **Port**: `machine.yaml → sensors[kind=joint_state].port`
  (`sensor_msgs/msg/JointState`).
- **CLI**: `ros2 topic echo <port> --once`
- **rclpy**: subscribe once;
  ```python
  import rclpy, yaml
  from sensor_msgs.msg import JointState
  port = next(s for s in yaml.safe_load(open("machine.yaml"))["sensors"]
              if s["kind"] == "joint_state")["port"]
  rclpy.init(); node = rclpy.create_node("js")
  got = []
  node.create_subscription(JointState, port, got.append, 1)
  while not got: rclpy.spin_once(node, timeout_sec=0.5)
  print(dict(zip(got[0].name, got[0].position)))
  ```
- **Notes**: message order is not fixed; always match by joint `name`
  (arm joint names: `machine.yaml → actuators[kind=joint_trajectory].joints`).

### A2. Pose (kinematics via TF)

- **What**: where any frame is relative to any other (`frames` in
  `machine.yaml` names the important ones: `world`, `base`, `hand`).
- **CLI**: `ros2 run tf2_ros tf2_echo <world> <hand>` (frame names from
  `machine.yaml → frames`; prints once a second, Ctrl-C after the first).
- **rclpy**: `tf2_ros.Buffer` + `TransformListener`, then
  `buffer.lookup_transform(world, hand, Time())`.
- **Notes**: TF gives you where the ROBOT is; object locations you
  estimate from camera images (B) + geometry.

### A3. Wrist force/torque (kind `wrench`)

- **What**: external wrench at the end effector; contact evidence.
- **Port**: `machine.yaml → sensors[kind=wrench].port`
  (`geometry_msgs/msg/WrenchStamped`).
- **CLI**: `ros2 topic echo <port> --once`
- **Notes**: the baseline is not zero; watch for *changes* on contact,
  not absolute values.

### A4. Odometry (kind `odometry` (only on mobile machines))

- **What**: the mobile base's pose; absent from the manifest = fixed
  base, skip.
- **Port**: `machine.yaml → sensors[kind=odometry].port`
  (`nav_msgs/msg/Odometry`; pose in the `world` frame, `child_frame_id`
  from the manifest entry's `frame`).
- **CLI**: `ros2 topic echo <port> --once`
- **Notes**: read before and after driving (30-action D); displacement
  is your evidence the base actually moved.

## B. Vision (kind `camera`, or `camera_set` when the scene decides)

### B1. Find the cameras

- Each `camera` entry in the manifest is one camera: its `name`, `model`,
  `mount` (wrist or fixed) and the exact `color`, `info` and `depth`
  topics, with the depth encoding and whether depth is aligned to color.
- A `camera_set` entry instead gives a topic pattern and a command that
  lists the cameras the scene has.

### B2. Grab a color frame

- **Tool**: `python3 tools/perception/cam_snap.py <name>` → `<name>.png`,
  then open the PNG (read the file).
- **rclpy**: subscribe to the entry's `color` topic, convert with
  `cv_bridge.CvBridge().imgmsg_to_cv2(msg, "bgr8")`.

### B3. Depth and intrinsics

- Depth: the entry's `depth` topic (`cam_snap.py <name> --depth`). ROS
  REP 118 defines `32FC1` depth in meters and `16UC1` depth in
  millimeters; the entry's `depth_encoding` says which. `cam_snap.py`
  saves native values unchanged in `.npy`. Convert with
  `imgmsg_to_cv2(msg, "passthrough")`.
- Intrinsics: `ros2 topic echo <info> --once` (`k` = 3×3 camera matrix;
  `header.frame_id` = the optical frame the pixels are in). With depth
  aligned to color, pixels + depth → 3D points in that frame, then TF
  into `frames.world`.

## C. Graph introspection (beyond the manifest)

- `ros2 topic list` / `ros2 service list` / `ros2 action list` /
  `ros2 node list`
- `ros2 topic info <topic>` and `ros2 interface show <type>` explain any
  port you find.

## Sensor-surface facts

- A TF buffer kept from before a motion can serve stale poses; look up
  after the motion with a fresh listener. When the planner offers an FK
  service (`ros2 service list | grep fk`), it gives the hand pose for
  given joints.
- `/robot_description` publishes with TRANSIENT_LOCAL durability; a
  plain subscription misses it; request that QoS.
- This machine's own sensor observations are under `notes` in
  machine.yaml.

**Tool**: `python3 tools/perception/px2world.py <camera> <u> <v>`;
depth + intrinsics + TF for one pixel, printed as x y z in `frames.world`,
metres. It reads the camera's topics from its manifest entry and the
optical frame from its CameraInfo; it needs depth aligned to color, and
TF has to connect the camera to the world frame (a camera nobody
calibrated into the robot's TF tree has no such path).
