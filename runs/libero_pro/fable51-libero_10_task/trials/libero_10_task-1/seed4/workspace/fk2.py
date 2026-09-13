import numpy as np, rclpy
from ctl import Ctl
from moveit_msgs.srv import GetPositionFK
c=Ctl()
req=GetPositionFK.Request(); req.fk_link_names=['panda_link8','panda_hand']
req.robot_state.joint_state=c._seed()
fut=c.fk.call_async(req); rclpy.spin_until_future_complete(c.n,fut,timeout_sec=30)
for ps in fut.result().pose_stamped:
    o=ps.pose.orientation; print(ps.header.frame_id, np.round([ps.pose.position.x,ps.pose.position.y,ps.pose.position.z],4), 'q', np.round([o.x,o.y,o.z,o.w],4))
print(fut.result().fk_link_names)
c.close()
