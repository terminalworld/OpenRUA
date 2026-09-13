import rclpy, sys, numpy as np
from moveit_msgs.srv import GetPositionFK
arm = [f"panda_joint{i}" for i in range(1,8)]
rclpy.init(); n = rclpy.create_node("fkof")
fk = n.create_client(GetPositionFK, "/compute_fk"); fk.wait_for_service()
req = GetPositionFK.Request(); req.fk_link_names = ["panda_hand"]
req.robot_state.joint_state.name = arm; req.robot_state.joint_state.position = [float(x) for x in sys.argv[1].split(",")]
f = fk.call_async(req); rclpy.spin_until_future_complete(n, f, timeout_sec=30)
p = f.result().pose_stamped[0].pose
q = p.orientation; x,y,z,w = q.x,q.y,q.z,q.w
R = np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
hw = np.array([p.position.x, p.position.y, p.position.z]); tcp = hw + 0.1034*R[:,2]
print("hand world", hw.round(4), "quat", np.array([x,y,z,w]).round(4), "tcp", tcp.round(4))
print("axes x", R[:,0].round(3), "y", R[:,1].round(3), "z", R[:,2].round(3))
