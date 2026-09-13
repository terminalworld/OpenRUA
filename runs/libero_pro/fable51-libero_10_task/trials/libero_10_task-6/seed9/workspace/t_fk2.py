from ctl import *
c = Ctl()
p,q = c.hand_world(); print("TF  hand", p.round(4), np.round(q,4))
for link in ["panda_hand","panda_link8","panda_leftfinger","panda_rightfinger"]:
    req = GetPositionFK.Request(); req.fk_link_names=[link]; s=JointState(); s.name=list(JOINTS); s.position=c.arm_q(); req.robot_state.joint_state=s
    f=c.fk.call_async(req); rclpy.spin_until_future_complete(c.n,f,timeout_sec=30); r=f.result()
    pp=r.pose_stamped[0].pose.position; o=r.pose_stamped[0].pose.orientation
    print("FK ", link, round(pp.x,4),round(pp.y,4),round(pp.z,4), "q", np.round([o.x,o.y,o.z,o.w],4))
for fr in ["panda_link8","panda_leftfinger","panda_rightfinger"]:
    t=c.tf.lookup_transform("world",fr,rclpy.time.Time()); tr=t.transform.translation; qq=t.transform.rotation
    print("TF ", fr, round(tr.x,4),round(tr.y,4),round(tr.z,4), "q", np.round([qq.x,qq.y,qq.z,qq.w],4))
