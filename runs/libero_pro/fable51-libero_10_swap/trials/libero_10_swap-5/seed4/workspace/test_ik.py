from rob import *
r = Robot()
q0 = r.arm_q()
req = GetPositionFK.Request(); req.fk_link_names = ["panda_hand"]; req.robot_state.joint_state = r._seed(q0)
fut = r.fk.call_async(req); rclpy.spin_until_future_complete(r.node, fut, timeout_sec=30)
p = fut.result().pose_stamped[0].pose
raw = np.array([p.position.x, p.position.y, p.position.z]); quat = np.array([p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w])
print("raw FK", raw)
def fk_raw(q):
    req = GetPositionFK.Request(); req.fk_link_names = ["panda_hand"]; req.robot_state.joint_state = r._seed(q)
    fut = r.fk.call_async(req); rclpy.spin_until_future_complete(r.node, fut, timeout_sec=30)
    p = fut.result().pose_stamped[0].pose
    return np.array([p.position.x, p.position.y, p.position.z]), np.array([p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w])
def ik_raw(pos, quat):
    req = GetPositionIK.Request(); rr = req.ik_request
    rr.group_name = "panda_arm"; rr.pose_stamped.header.frame_id = ""
    rr.pose_stamped.pose.position.x, rr.pose_stamped.pose.position.y, rr.pose_stamped.pose.position.z = map(float, pos)
    rr.pose_stamped.pose.orientation.x, rr.pose_stamped.pose.orientation.y, rr.pose_stamped.pose.orientation.z, rr.pose_stamped.pose.orientation.w = map(float, quat)
    rr.robot_state.joint_state = r._seed(q0); rr.avoid_collisions = False; rr.timeout.sec = 5
    fut = r.ik.call_async(req); rclpy.spin_until_future_complete(r.node, fut, timeout_sec=60)
    res = fut.result()
    if res.error_code.val != 1: return None, res.error_code.val
    sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
    return np.array([sol[j] for j in ARM]), 1
qd = topdown_quat(-np.pi/2)
for name, pos in [("raw fk", raw), ("raw+BASE", raw + BASE_W), ("raw-BASE", raw - BASE_W),
                  ("book hand world", [-0.093, -0.019, 1.20]), ("book hand base", np.array([-0.093, -0.019, 1.20]) - BASE_W)]:
    q, c = ik_raw(pos, qd)
    if q is None: print(name, pos, "IK fail", c); continue
    fp, fq = fk_raw(q)
    print(name, "target", np.round(pos,3), "-> FK(sol)", np.round(fp,3), "q", np.round(q,3))
