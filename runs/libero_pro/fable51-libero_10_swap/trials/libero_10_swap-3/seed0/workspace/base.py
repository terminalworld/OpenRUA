from robot import *
from moveit_msgs.srv import GetPositionFK
r = Robot("base")
q = r.arm_q()
req = GetPositionFK.Request(); req.fk_link_names = ["panda_link0","panda_link4","panda_link6","panda_link7","panda_hand"]
req.robot_state.joint_state.name = JOINTS; req.robot_state.joint_state.position = [float(v) for v in q]
fut = r.fk.call_async(req); rclpy.spin_until_future_complete(r.node, fut, timeout_sec=30)
for n, ps in zip(fut.result().fk_link_names, fut.result().pose_stamped):
    p = ps.pose.position; print(n, ps.header.frame_id, round(p.x,4), round(p.y,4), round(p.z,4))
