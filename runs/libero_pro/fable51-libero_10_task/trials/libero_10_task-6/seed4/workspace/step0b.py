from robot import *
r = Robot()
q,_ = r.joints()
req = GetPositionFK.Request(); req.fk_link_names = ["panda_link0", "panda_hand", "panda_leftfinger"]
req.robot_state.joint_state = r._seed(q)
fut = r.fk.call_async(req); rclpy.spin_until_future_complete(r.node, fut, timeout_sec=30)
res = fut.result()
for n, ps in zip(res.fk_link_names, res.pose_stamped):
    p = ps.pose.position; print(n, "frame:", repr(ps.header.frame_id), "pos", round(p.x,4), round(p.y,4), round(p.z,4))
