import rclpy, sys
from tf2_ros import Buffer, TransformListener
rclpy.init(); node = rclpy.create_node("tfdump")
buf = Buffer(); TransformListener(buf, node)
frames = sys.argv[1:] or ["agentview_optical_frame","birdview_optical_frame","frontview_optical_frame","sideview_optical_frame","robot0_eye_in_hand_optical_frame","robot0_robotview_optical_frame","panda_link0","panda_hand","panda_leftfinger","panda_rightfinger"]
import time
t0=time.time()
while time.time()-t0 < 15 and not all(buf.can_transform("world", f, rclpy.time.Time()) for f in frames):
    rclpy.spin_once(node, timeout_sec=0.2)
for f in frames:
    try:
        t = buf.lookup_transform("world", f, rclpy.time.Time())
        tr, q = t.transform.translation, t.transform.rotation
        print(f, [round(v,4) for v in (tr.x,tr.y,tr.z)], [round(v,4) for v in (q.x,q.y,q.z,q.w)])
    except Exception as e:
        print(f, "ERR", e)
