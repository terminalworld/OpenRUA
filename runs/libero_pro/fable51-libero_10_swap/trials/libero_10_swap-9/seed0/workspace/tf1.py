import rclpy, sys
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
rclpy.init(); n = rclpy.create_node("tf1")
buf = Buffer(); TransformListener(buf, n)
import time
while not buf.can_transform("world","panda_hand",Time()) or not buf.can_transform("world","robot0_robotview_optical_frame",Time()): rclpy.spin_once(n, timeout_sec=0.2)
for b in sys.argv[1:]:
    t = buf.lookup_transform("world",b,Time()); tr=t.transform.translation; q=t.transform.rotation
    print(b, f"t=({tr.x:.4f},{tr.y:.4f},{tr.z:.4f}) q=({q.x:.4f},{q.y:.4f},{q.z:.4f},{q.w:.4f})")
