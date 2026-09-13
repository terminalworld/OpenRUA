import rclpy, sys
from tf2_ros import Buffer, TransformListener
rclpy.init(); n = rclpy.create_node("tfd")
buf = Buffer(); TransformListener(buf, n)
import time
for _ in range(30): rclpy.spin_once(n, timeout_sec=0.2)
print(buf.all_frames_as_string())
for f in sys.argv[1:]:
    try:
        t = buf.lookup_transform("world", f, rclpy.time.Time())
        tr, q = t.transform.translation, t.transform.rotation
        print(f, "t=", round(tr.x,4), round(tr.y,4), round(tr.z,4), "q=", round(q.x,4), round(q.y,4), round(q.z,4), round(q.w,4))
    except Exception as e:
        print(f, "ERR", e)
