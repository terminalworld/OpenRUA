#!/usr/bin/env python3
"""Print camera poses in world, world->panda_link0, and hand FK."""
import rclpy, yaml
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState

rclpy.init()
node = rclpy.create_node("survey")
buf = Buffer()
TransformListener(buf, node)
frames = set()
def on_tf(m):
    for t in m.transforms:
        frames.add((t.header.frame_id, t.child_frame_id))
qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)
node.create_subscription(TFMessage, "/tf_static", on_tf, qos)
node.create_subscription(TFMessage, "/tf", on_tf, 100)
js = {}
node.create_subscription(JointState, "/joint_states", lambda m: js.setdefault("m", m), 1)
for _ in range(20):
    rclpy.spin_once(node, timeout_sec=0.2)
print("TF edges:")
for e in sorted(frames):
    print("  ", e)
for cam in ["agentview", "birdview", "frontview", "sideview", "robot0_eye_in_hand", "robot0_robotview", "galleryview", "paperview"]:
    f = f"{cam}_optical_frame"
    try:
        t = buf.lookup_transform("world", f, Time())
        tr, q = t.transform.translation, t.transform.rotation
        print(f"{cam}: t=({tr.x:.4f},{tr.y:.4f},{tr.z:.4f}) q=({q.x:.4f},{q.y:.4f},{q.z:.4f},{q.w:.4f})")
    except Exception as ex:
        print(f"{cam}: {type(ex).__name__}: {ex}")
for f in ["panda_link0", "panda_hand"]:
    try:
        t = buf.lookup_transform("world", f, Time())
        tr, q = t.transform.translation, t.transform.rotation
        print(f"world->{f}: t=({tr.x:.4f},{tr.y:.4f},{tr.z:.4f}) q=({q.x:.4f},{q.y:.4f},{q.z:.4f},{q.w:.4f})")
    except Exception as ex:
        print(f"world->{f}: {type(ex).__name__}: {ex}")

# FK
cli = node.create_client(GetPositionFK, "/compute_fk")
if cli.wait_for_service(timeout_sec=10) and "m" in js:
    req = GetPositionFK.Request()
    req.header.frame_id = ""
    req.fk_link_names = ["panda_hand"]
    arm = [f"panda_joint{i}" for i in range(1, 8)]
    seed = JointState()
    for n, p in zip(js["m"].name, js["m"].position):
        if n in arm:
            seed.name.append(n); seed.position.append(p)
    req.robot_state.joint_state = seed
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=30)
    r = fut.result()
    if r:
        for ps in r.pose_stamped:
            p, q = ps.pose.position, ps.pose.orientation
            print(f"FK panda_hand (frame '{ps.header.frame_id}'): p=({p.x:.4f},{p.y:.4f},{p.z:.4f}) q=({q.x:.4f},{q.y:.4f},{q.z:.4f},{q.w:.4f}) err={r.error_code.val}")
rclpy.shutdown()
