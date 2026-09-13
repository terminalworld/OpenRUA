from rob import *
r = Robot("fkf")
req = GetPositionFK.Request(); req.fk_link_names = ["panda_link0","panda_hand"]
req.robot_state.joint_state.name = list(JOINTS); req.robot_state.joint_state.position = r.joints()
res = r._call(r.fk, req)
for ps in res.pose_stamped: print(ps.header.frame_id, ps.pose.position)
