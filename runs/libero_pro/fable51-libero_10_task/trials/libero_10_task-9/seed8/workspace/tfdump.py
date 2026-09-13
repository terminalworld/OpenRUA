import rclpy, sys
from tf2_ros import Buffer, TransformListener
rclpy.init(); node = rclpy.create_node("tfdump")
buf = Buffer(); TransformListener(buf, node)
import time
for _ in range(30): rclpy.spin_once(node, timeout_sec=0.2)
print(buf.all_frames_as_string())
for f in sys.argv[1:]:
    try:
        t = buf.lookup_transform("world", f, rclpy.time.Time())
        tr, q = t.transform.translation, t.transform.rotation
        print(f, "xyz=(%.4f %.4f %.4f) q=(%.4f %.4f %.4f %.4f)"%(tr.x,tr.y,tr.z,q.x,q.y,q.z,q.w))
    except Exception as e: print(f, "ERR", e)
rclpy.shutdown()
