from arm import *
r = Robot()
req = GetPositionFK.Request(); req.fk_link_names = ["panda_link0", "panda_link1", "panda_hand"]
req.robot_state.joint_state.name = JOINTS; req.robot_state.joint_state.position = r.arm_q()
res = r._call(r.fk, req)
for n, ps in zip(res.fk_link_names, res.pose_stamped):
    p = ps.pose.position; print(n, ps.header.frame_id, round(p.x,4), round(p.y,4), round(p.z,4))
req.header.frame_id = "world"
res = r._call(r.fk, req)
for n, ps in zip(res.fk_link_names, res.pose_stamped):
    p = ps.pose.position; print("hdr=world", n, ps.header.frame_id, round(p.x,4), round(p.y,4), round(p.z,4))
