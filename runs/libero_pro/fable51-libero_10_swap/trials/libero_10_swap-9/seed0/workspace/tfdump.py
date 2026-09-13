import rclpy, sys
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
rclpy.init(); n = rclpy.create_node("tfdump")
buf = Buffer(); TransformListener(buf, n)
for _ in range(20): rclpy.spin_once(n, timeout_sec=0.2)
print(buf.all_frames_as_string())
pairs = [("world","panda_link0"),("world","birdview_optical_frame"),("world","agentview_optical_frame"),("world","sideview_optical_frame"),("world","frontview_optical_frame"),("panda_link0","panda_hand")]
for a,b in pairs:
    try:
        t = buf.lookup_transform(a,b,Time())
        tr=t.transform.translation; q=t.transform.rotation
        print(a,"->",b, f"t=({tr.x:.4f},{tr.y:.4f},{tr.z:.4f}) q=({q.x:.4f},{q.y:.4f},{q.z:.4f},{q.w:.4f})")
    except Exception as e: print(a,"->",b,"FAIL",e)
