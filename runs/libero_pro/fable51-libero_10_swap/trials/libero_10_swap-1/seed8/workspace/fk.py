import rclpy, sys
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState
rclpy.init(); n = rclpy.create_node("fk")
js = {}
n.create_subscription(JointState, "/joint_states", lambda m: js.setdefault("m", m), 1)
while "m" not in js: rclpy.spin_once(n, timeout_sec=0.2)
cli = n.create_client(GetPositionFK, "/compute_fk"); cli.wait_for_service(10)
req = GetPositionFK.Request()
req.fk_link_names = sys.argv[1:] or ["panda_hand", "panda_link8", "panda_hand_tcp"]
seed = JointState()
for nm, p in zip(js["m"].name, js["m"].position):
    if nm.startswith("panda_joint"): seed.name.append(nm); seed.position.append(p)
req.robot_state.joint_state = seed
f = cli.call_async(req); rclpy.spin_until_future_complete(n, f, timeout_sec=30)
r = f.result()
print("error", r.error_code.val)
for nm, ps in zip(r.fk_link_names, r.pose_stamped):
    p, q = ps.pose.position, ps.pose.orientation
    print(nm, ps.header.frame_id, f"{p.x:.4f} {p.y:.4f} {p.z:.4f}", f"q {q.x:.3f} {q.y:.3f} {q.z:.3f} {q.w:.3f}")
