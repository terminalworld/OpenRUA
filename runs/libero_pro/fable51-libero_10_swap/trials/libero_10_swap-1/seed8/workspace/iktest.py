import rclpy, sys
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
rclpy.init(); n = rclpy.create_node("iktest")
js = {}
n.create_subscription(JointState, "/joint_states", lambda m: js.setdefault("m", m), 1)
while "m" not in js: rclpy.spin_once(n, timeout_sec=0.2)
cli = n.create_client(GetPositionIK, "/compute_ik"); cli.wait_for_service(10)
cur = {nm: p for nm, p in zip(js["m"].name, js["m"].position) if nm.startswith("panda_joint")}
def ik(x, y, z, q, link=""):
    req = GetPositionIK.Request()
    req.ik_request.group_name = "panda_arm"
    req.ik_request.pose_stamped.header.frame_id = ""
    if link: req.ik_request.ik_link_name = link
    p = req.ik_request.pose_stamped.pose
    p.position.x, p.position.y, p.position.z = x, y, z
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q)
    seed = JointState(); seed.name = list(cur); seed.position = list(cur.values())
    req.ik_request.robot_state.joint_state = seed
    req.ik_request.timeout = Duration(sec=2)
    f = cli.call_async(req); rclpy.spin_until_future_complete(n, f, timeout_sec=60)
    r = f.result()
    if r is None: print("  no answer"); return
    sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
    diff = max(abs(sol[j]-cur[j]) for j in cur) if r.error_code.val == 1 else None
    print(f"  code={r.error_code.val} maxdiff_from_current={diff}")
q8 = (0.924, -0.383, -0.026, 0.011)
print("link8 world coords:"); ik(-0.053, 0.0, 0.7776, q8)
print("link8 base coords:"); ik(0.457, 0.0, 0.3576, q8)
print("hand-orientation q=(1,0,0,0) world coords (expected wrong tip orientation):"); ik(-0.053, 0.0, 0.7776, (1.0,0,0,0))
