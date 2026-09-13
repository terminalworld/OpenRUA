import sys, rclpy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
rclpy.init(); n = rclpy.create_node("tfq"); b = Buffer(); TransformListener(b, n)
pairs = [tuple(a.split(":")) for a in sys.argv[1:]]
import time
for _ in range(40):
    rclpy.spin_once(n, timeout_sec=0.1)
for a, c in pairs:
    try:
        t = b.lookup_transform(a, c, Time())
        tr, q = t.transform.translation, t.transform.rotation
        print(f"{a}->{c}: xyz=({tr.x:.4f},{tr.y:.4f},{tr.z:.4f}) q=({q.x:.4f},{q.y:.4f},{q.z:.4f},{q.w:.4f})")
    except Exception as e:
        print(f"{a}->{c}: FAIL {e}")
print(b.all_frames_as_string())
