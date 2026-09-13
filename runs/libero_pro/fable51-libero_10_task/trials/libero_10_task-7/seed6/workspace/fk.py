#!/usr/bin/env python3
"""Print hand pose via /compute_fk (arm joints only) plus finger positions."""
import time
import rclpy
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState

ARM = [f"panda_joint{i}" for i in range(1, 8)]
rclpy.init()
node = rclpy.create_node("fk")
js = {}
node.create_subscription(JointState, "/joint_states", lambda m: js.setdefault("m", m), 1)
end = time.time() + 15
while "m" not in js and time.time() < end:
    rclpy.spin_once(node, timeout_sec=0.2)
m = js["m"]
d = dict(zip(m.name, m.position))
print("joints:", ", ".join(f"{d[j]:.4f}" for j in ARM))
print("fingers:", d.get("panda_finger_joint1"), d.get("panda_finger_joint2"))
cli = node.create_client(GetPositionFK, "/compute_fk")
cli.wait_for_service(10)
req = GetPositionFK.Request()
req.header.frame_id = ""
req.fk_link_names = ["panda_hand"]
req.robot_state.joint_state.name = ARM
req.robot_state.joint_state.position = [d[j] for j in ARM]
fut = cli.call_async(req)
rclpy.spin_until_future_complete(node, fut, timeout_sec=30)
r = fut.result()
if r is None:
    print("FK no answer")
else:
    print("fk error", r.error_code.val)
    for ps in r.pose_stamped:
        p, q = ps.pose.position, ps.pose.orientation
        print(f"frame '{ps.header.frame_id}' pos ({p.x:.4f},{p.y:.4f},{p.z:.4f}) quat ({q.x:.4f},{q.y:.4f},{q.z:.4f},{q.w:.4f})")
rclpy.shutdown()
