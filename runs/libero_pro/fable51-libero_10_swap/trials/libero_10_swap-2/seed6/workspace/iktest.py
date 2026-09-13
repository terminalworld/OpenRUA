import rclpy, yaml
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from sensor_msgs.msg import JointState
M = yaml.safe_load(open("/workspace/machine.yaml")); arm = M["actuators"][0]["joints"]
rclpy.init(); n = rclpy.create_node("iktest")
js = {}
n.create_subscription(JointState, "/joint_states", lambda m: js.setdefault("m", m), 1)
while "m" not in js: rclpy.spin_once(n, timeout_sec=0.2)
d = dict(zip(js["m"].name, js["m"].position))
fk = n.create_client(GetPositionFK, "/compute_fk"); fk.wait_for_service()
req = GetPositionFK.Request(); req.fk_link_names = ["panda_hand"]
req.robot_state.joint_state.name = arm; req.robot_state.joint_state.position = [d[j] for j in arm]
f = fk.call_async(req); rclpy.spin_until_future_complete(n, f, timeout_sec=30)
ps = f.result().pose_stamped[0]
print("FK frame:", repr(ps.header.frame_id), ps.pose.position)
ik = n.create_client(GetPositionIK, "/compute_ik"); ik.wait_for_service()
for fid in ["", "world", "panda_link0"]:
    r = GetPositionIK.Request(); r.ik_request.group_name = "panda_arm"
    r.ik_request.pose_stamped.header.frame_id = fid
    r.ik_request.pose_stamped.pose = ps.pose
    r.ik_request.robot_state.joint_state.name = arm
    r.ik_request.robot_state.joint_state.position = [d[j] for j in arm]
    r.ik_request.timeout.sec = 2
    f = ik.call_async(r); rclpy.spin_until_future_complete(n, f, timeout_sec=60)
    res = f.result()
    sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
    print(f"frame_id={fid!r}: code={res.error_code.val}", [round(sol.get(j, float('nan')),3) for j in arm])
