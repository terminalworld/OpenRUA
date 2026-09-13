from rob import *
r = R()
pos,q = r.fk(); Rm = quat_R(*q)
print("hand pos", pos.round(4), "q", np.round(q,4))
print("hand x in world", Rm[:,0].round(3)); print("hand y (finger axis) in world", Rm[:,1].round(3)); print("hand z", Rm[:,2].round(3))
# also FK of the fingers
req = GetPositionFK.Request(); req.header.frame_id=""
req.fk_link_names = ["panda_leftfinger","panda_rightfinger","panda_link8"]
req.robot_state.joint_state = r._seed()
fut = r.fkc.call_async(req); rclpy.spin_until_future_complete(r.node, fut, timeout_sec=30)
for n,ps in zip(fut.result().fk_link_names, fut.result().pose_stamped): print(n, np.round([ps.pose.position.x,ps.pose.position.y,ps.pose.position.z],4))
