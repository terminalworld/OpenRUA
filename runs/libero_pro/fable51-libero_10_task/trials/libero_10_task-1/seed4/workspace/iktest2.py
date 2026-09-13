import numpy as np
from ctl import Ctl, quat_R, BASE, TCP
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState
c=Ctl()
q0=np.array(c.arm_q())
xyz,quat=c.hand_pose(); hand_raw = xyz-BASE
tcp = hand_raw + TCP*quat_R(*quat)[:,2] + BASE   # so solve_ik passes hand_raw
sol=np.array(c.solve_ik(tcp, quat))
print('current ', np.round(q0,3)); print('solution', np.round(sol,3))
# FK of solution
req=GetPositionFK.Request(); req.fk_link_names=['panda_hand']
js=JointState(); js.name=[f'panda_joint{i}' for i in range(1,8)]; js.position=[float(x) for x in sol]
req.robot_state.joint_state=js
import rclpy
fut=c.fk.call_async(req); rclpy.spin_until_future_complete(c.n,fut,timeout_sec=30)
p=fut.result().pose_stamped[0]
print('FK(sol) frame', p.header.frame_id, np.round([p.pose.position.x,p.pose.position.y,p.pose.position.z],4))
c.close()
