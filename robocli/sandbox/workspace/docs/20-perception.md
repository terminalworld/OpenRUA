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
- **Notes**: baseline is not zero (gravity); watch for *changes* on
  contact, not absolute values.

### A4. Odometry (kind `odometry` (only on mobile machines))

- **What**: the mobile base's pose; absent from the manifest = fixed
  base, skip.
- **Port**: `machine.yaml → sensors[kind=odometry].port`
  (`nav_msgs/msg/Odometry`; pose in the `world` frame, `child_frame_id`
  from the manifest entry's `frame`).
- **CLI**: `ros2 topic echo <port> --once`
- **Notes**: read before and after driving (30-action D); displacement
  is your evidence the base actually moved.

## B. Vision (kind `camera_set`)

### B1. Discover the cameras

- The manifest entry gives the topic pattern and the discovery command:
  `ros2 topic list | grep image_raw`. Each camera `<name>` publishes
  color, depth and intrinsics topics per that pattern.

### B2. Grab a color frame

- **Tool**: `python3 tools/perception/cam_snap.py <name>` → `<name>.png`,
  then open the PNG (read the file).
- **rclpy**: subscribe to the topic, convert with
  `cv_bridge.CvBridge().imgmsg_to_cv2(msg, "bgr8")`.

### B3. Depth and intrinsics

- Depth: same pattern on the depth topic (`cam_snap.py` accepts a full
  topic path; depth is 32-bit float meters; convert with
  `imgmsg_to_cv2(msg, "passthrough")`).
- Intrinsics: `ros2 topic echo /<name>/color/camera_info --once`
  (`k` = 3×3 camera matrix); pixels + depth → 3D points in the camera
  optical frame, then TF into `world`.

## C. Graph introspection (beyond the manifest)

- `ros2 topic list` / `ros2 service list` / `ros2 action list` /
  `ros2 node list`
- `ros2 topic info <topic>` and `ros2 interface show <type>` explain any
  port you find.

## Sensor-surface facts

- The TF arm chain may be DISCONNECTED from the world tree (lookups
  like world->hand fail with "unconnected trees"). For the hand pose
  use the FK service; TF stamps are sim-time and a cached buffer can
  serve stale poses after motion.
- `/robot_description` publishes with TRANSIENT_LOCAL durability; a
  plain subscription misses it; request that QoS.
- Camera topics may appear in the graph lazily; the manifest's
  camera_set entry is authoritative, `ros2 node info` shows the full set.
- Oblique (side-view) cameras systematically overestimate the height of
  low, flat objects near their edges; cross-view disagreement on such
  objects usually traces to this.
- The odometry topic publishes only as the sim advances.

**Tool**: `python3 tools/perception/px2world.py <camera> <u> <v>`;
depth + intrinsics + TF for one pixel, printed as world x y z.
