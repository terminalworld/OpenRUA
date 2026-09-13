"""Print current hand pose (panda_link0 and world) via /compute_fk."""
import rclpy, yaml, sys, time
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState
import numpy as np
M = yaml.safe_load(open("/workspace/machine.yaml"))
arm = M["actuators"][0]["joints"]
rclpy.init(); n = rclpy.create_node("fk")
js = {}
n.create_subscription(JointState, "/joint_states", lambda m: js.setdefault("m", m), 1)
while "m" not in js: rclpy.spin_once(n, timeout_sec=0.2)
cli = n.create_client(GetPositionFK, "/compute_fk"); cli.wait_for_service()
req = GetPositionFK.Request()
req.fk_link_names = ["panda_hand"]
d = dict(zip(js["m"].name, js["m"].position))
req.robot_state.joint_state.name = arm
req.robot_state.joint_state.position = [d[j] for j in arm]
f = cli.call_async(req); rclpy.spin_until_future_complete(n, f, timeout_sec=30)
r = f.result()
p = r.pose_stamped[0].pose
print("joints", [round(d[j],4) for j in arm], "fingers", round(d["panda_finger_joint1"],4), round(d["panda_finger_joint2"],4))
print("hand in base: pos", round(p.position.x,4), round(p.position.y,4), round(p.position.z,4), "quat", round(p.orientation.x,4), round(p.orientation.y,4), round(p.orientation.z,4), round(p.orientation.w,4))
q = p.orientation; x,y,z,w = q.x,q.y,q.z,q.w
R = np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
base = np.array([-0.66, 0.0, 0.912])
hw = base + np.array([p.position.x, p.position.y, p.position.z])
tcp = hw + 0.1034*R[:,2]
print("hand in world:", hw.round(4), " tcp in world:", tcp.round(4))
print("hand axes in world: x", R[:,0].round(3), "y", R[:,1].round(3), "z", R[:,2].round(3))
