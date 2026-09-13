from arm import *
a = Arm()
req = GetPositionFK.Request(); req.fk_link_names = ["panda_link0", "panda_hand"]
req.robot_state.joint_state = a.arm_state()
fut = a.fk.call_async(req); rclpy.spin_until_future_complete(a.node, fut, timeout_sec=30)
for ps in fut.result().pose_stamped:
    print(ps.header.frame_id, ps.pose.position)
# IK test: hand world pose from TF
q = (1.0, 0.0, -0.0284, 0.0)
for label, xyz in [("world-coords", (-0.053, 0.0, 0.778)), ("base-coords", (0.457, 0.0, 0.358))]:
    try:
        req = GetPositionIK.Request(); req.ik_request.group_name = "panda_arm"
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = xyz
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = q
        req.ik_request.robot_state.joint_state = a.arm_state()
        fut = a.ik.call_async(req); rclpy.spin_until_future_complete(a.node, fut, timeout_sec=60)
        res = fut.result()
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        print(label, res.error_code.val, [round(sol.get(j, float('nan')), 3) for j in JOINTS])
    except Exception as e:
        print(label, "err", e)
