import rclpy, numpy as np, time
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy
from rob import quat_R
rclpy.init(); node = rclpy.create_node("tfchk")
seen = {}
def cb(msg):
    for t in msg.transforms: seen[(t.header.frame_id, t.child_frame_id)] = t.transform
node.create_subscription(TFMessage, "/tf", cb, 100)
node.create_subscription(TFMessage, "/tf_static", cb, QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))
t0=time.time()
while time.time()-t0<3: rclpy.spin_once(node, timeout_sec=0.2)
tr = seen[("world","robot0_eye_in_hand_optical_frame")]
q = tr.rotation; R = quat_R(q.x,q.y,q.z,q.w)  # cam->world
print("cam pos", tr.translation)
# image directions: cam x = image right, cam y = image down, cam z = optical
for name, wv in [("world x", [1,0,0]), ("world y",[0,1,0]), ("world z",[0,0,1])]:
    cv = R.T @ np.array(wv)
    print(name, "-> cam (right,down,fwd) =", cv.round(3))
