import numpy as np
from rob import *
from moveit_msgs.srv import GetPositionFK
r = Robot("t2")
req = GetPositionFK.Request(); req.header.frame_id = ""
req.fk_link_names = ["panda_link0", "panda_link8", "panda_hand"]
s = JointState(); s.name = list(ARM); s.position = r.joints()
req.robot_state.joint_state = s
fut = r.fk.call_async(req); rclpy.spin_until_future_complete(r.node, fut, timeout_sec=30)
res = fut.result()
poses = {}
for n, ps in zip(res.fk_link_names, res.pose_stamped):
    p, o = ps.pose.position, ps.pose.orientation
    poses[n] = ((p.x, p.y, p.z), (o.x, o.y, o.z, o.w))
    print(n, ps.header.frame_id, np.round([p.x, p.y, p.z], 4), np.round([o.x, o.y, o.z, o.w], 4))
# IK with link8 pose, raw (no base offset)
req = GetPositionIK.Request()
req.ik_request.group_name = "panda_arm"; req.ik_request.pose_stamped.header.frame_id = ""
(px, py, pz), (qx, qy, qz, qw) = poses["panda_link8"]
pp = req.ik_request.pose_stamped.pose
pp.position.x, pp.position.y, pp.position.z = px, py, pz
pp.orientation.x, pp.orientation.y, pp.orientation.z, pp.orientation.w = qx, qy, qz, qw
req.ik_request.robot_state.joint_state = s
fut = r.ik.call_async(req); rclpy.spin_until_future_complete(r.node, fut, timeout_sec=60)
res = fut.result(); print("err", res.error_code.val)
sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
print("IK(link8 pose) ->", np.round([sol[j] for j in ARM], 4))
print("current        ->", np.round(r.joints(), 4))
