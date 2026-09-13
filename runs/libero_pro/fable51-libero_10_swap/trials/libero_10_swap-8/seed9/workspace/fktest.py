import numpy as np
from rob import *
import rclpy
from moveit_msgs.srv import GetPositionFK
r = Robot("fktest")
q0 = r.arm_q()
for fid in ["", "world", "panda_link0"]:
    req = GetPositionFK.Request(); req.header.frame_id = fid
    req.fk_link_names = ["panda_link0","panda_hand"]
    req.robot_state.joint_state = r._seed(q0)
    fut = r.fk.call_async(req); rclpy.spin_until_future_complete(r.node, fut, timeout_sec=30)
    res = fut.result()
    print(repr(fid), res.error_code.val, [(ps.header.frame_id, np.round([ps.pose.position.x,ps.pose.position.y,ps.pose.position.z],3)) for ps in res.pose_stamped])
# IK in world coords for pot A above
for quat in [(0.7071,0.7071,0,0),(0.7071,-0.7071,0,0),(1,0,0,0)]:
    R = quat_to_R(*quat)
    hand = np.array([-0.196,-0.200,1.05]) - R[:,2]*TCP
    sol = r.ik_base(hand, quat, seed=q0)
    print(quat, sol if sol is None else np.round(sol,3))
