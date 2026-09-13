from arm import *
a=Arm()
req=GetPositionFK.Request(); req.fk_link_names=["panda_link8","panda_hand"]
req.robot_state.joint_state.name=ARM; req.robot_state.joint_state.position=a.joints()
f=a.fk.call_async(req); rclpy.spin_until_future_complete(a.node,f,timeout_sec=60)
for ps in f.result().pose_stamped:
    p=ps.pose.position; q=ps.pose.orientation; print(ps.header.frame_id,(round(p.x,4),round(p.y,4),round(p.z,4)),(round(q.x,4),round(q.y,4),round(q.z,4),round(q.w,4)))
print(np.round(a.solve_ik_base([-0.053,0,0.7776],(0.9239,-0.3827,0,0),at_tcp=False),4))
