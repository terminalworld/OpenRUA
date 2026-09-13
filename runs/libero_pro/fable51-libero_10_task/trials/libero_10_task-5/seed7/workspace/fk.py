import numpy as np, rclpy
from ctl import Ctl, JOINTS, TCP
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState
from scipy.spatial.transform import Rotation as Rot
c = Ctl()
cl = c.node.create_client(GetPositionFK, "/compute_fk"); cl.wait_for_service(10)
req = GetPositionFK.Request(); req.fk_link_names = ["panda_hand"]
s = JointState(); s.name = list(JOINTS); s.position = c.arm_q(); req.robot_state.joint_state = s
f = cl.call_async(req); rclpy.spin_until_future_complete(c.node, f, timeout_sec=30)
p = f.result().pose_stamped[0].pose
R = Rot.from_quat([p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w])
hand = np.array([p.position.x,p.position.y,p.position.z]); tcp = hand + TCP*R.as_matrix()[:,2]
print("hand", hand.round(4), "tcp", tcp.round(4), "hand_z", R.as_matrix()[:,2].round(3), "hand_y", R.as_matrix()[:,1].round(3))
