import numpy as np, rclpy, sys
sys.path.insert(0,"/workspace")
from arm import Arm, JOINTS
from moveit_msgs.srv import GetPositionFK
a = Arm()
req = GetPositionFK.Request(); req.header.frame_id = ""
req.fk_link_names = ["panda_link0","panda_link8","panda_hand"]
req.robot_state.joint_state.name = JOINTS
req.robot_state.joint_state.position = list(a.q())
fut = a.fk.call_async(req); rclpy.spin_until_future_complete(a.node, fut, timeout_sec=30)
r = fut.result()
print("code", r.error_code.val)
for ps in r.pose_stamped:
    p = ps.pose; print(ps.header.frame_id, p.position.x, p.position.y, p.position.z, "|", p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
# TF chain
from tf2_ros import Buffer, TransformListener
import rclpy.time
buf = Buffer(); TransformListener(buf, a.node)
import time
t0=time.time()
while time.time()-t0<5: rclpy.spin_once(a.node, timeout_sec=0.1)
for f in ["panda_link0","panda_hand","robot0_eye_in_hand_optical_frame"]:
    try:
        t = buf.lookup_transform("world", f, rclpy.time.Time()).transform
        print("TF world->",f, t.translation.x, t.translation.y, t.translation.z, "|", t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w)
    except Exception as e: print("TF fail", f, e)
