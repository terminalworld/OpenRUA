from robot import *
r = Robot()
quat = Robot.quat_topdown(90)
sol = r.ik_world([-0.22, -0.125, 0.62], quat)
for link in ["panda_link8", "panda_hand"]:
    req = GetPositionFK.Request(); req.header.frame_id=""; req.fk_link_names=[link]
    js = JointState(); js.name=list(JOINTS); js.position=[float(v) for v in sol]
    req.robot_state.joint_state=js
    fut=r.fk.call_async(req); rclpy.spin_until_future_complete(r.node, fut, timeout_sec=30)
    p=fut.result().pose_stamped[0].pose
    print(link, [round(v,4) for v in (p.position.x,p.position.y,p.position.z)], [round(v,4) for v in (p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w)])
print("requested", quat.round(4))
