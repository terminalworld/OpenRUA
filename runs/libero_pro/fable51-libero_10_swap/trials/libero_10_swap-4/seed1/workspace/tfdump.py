#!/usr/bin/env python3
"""Dump all TF frames (world -> X) once, plus FK of the hand via /compute_fk."""
import rclpy, yaml
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState

rclpy.init()
node = rclpy.create_node("tfdump")
buf = Buffer(); TransformListener(buf, node)
for _ in range(20):
    rclpy.spin_once(node, timeout_sec=0.2)
frames = yaml.safe_load(buf.all_frames_as_yaml()) or {}
print("frames:", sorted(frames.keys()))
for f in sorted(frames):
    for root in ("world", "panda_link0"):
        try:
            t = buf.lookup_transform(root, f, Time())
            tr, q = t.transform.translation, t.transform.rotation
            print(f"{root}->{f}: t=({tr.x:.4f},{tr.y:.4f},{tr.z:.4f}) q=({q.x:.4f},{q.y:.4f},{q.z:.4f},{q.w:.4f})")
        except Exception as e:
            print(f"{root}->{f}: FAIL {type(e).__name__}")

# FK of the hand
js = {}
node.create_subscription(JointState, "/joint_states", lambda m: js.setdefault("m", m), 1)
while "m" not in js:
    rclpy.spin_once(node, timeout_sec=0.2)
cli = node.create_client(GetPositionFK, "/compute_fk")
cli.wait_for_service(timeout_sec=10)
req = GetPositionFK.Request()
req.fk_link_names = ["panda_hand", "panda_link8"]
arm = [f"panda_joint{i}" for i in range(1, 8)]
seed = JointState()
for n, p in zip(js["m"].name, js["m"].position):
    if n in arm:
        seed.name.append(n); seed.position.append(p)
req.robot_state.joint_state = seed
fut = cli.call_async(req)
rclpy.spin_until_future_complete(node, fut, timeout_sec=30)
res = fut.result()
if res:
    for n, ps in zip(res.fk_link_names, res.pose_stamped):
        p, q = ps.pose.position, ps.pose.orientation
        print(f"FK {n} [{ps.header.frame_id}]: p=({p.x:.4f},{p.y:.4f},{p.z:.4f}) q=({q.x:.4f},{q.y:.4f},{q.z:.4f},{q.w:.4f})")
rclpy.shutdown()
