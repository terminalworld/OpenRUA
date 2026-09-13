import rclpy, yaml
from tf2_ros import Buffer, TransformListener
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
rclpy.init(); node = rclpy.create_node("tfdump")
buf = Buffer(); TransformListener(buf, node)
frames_seen = set()
def cb(m):
    for t in m.transforms: frames_seen.add((t.header.frame_id, t.child_frame_id))
qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
node.create_subscription(TFMessage, "/tf_static", cb, qos)
node.create_subscription(TFMessage, "/tf", cb, 100)
import time
end=time.time()+4
while time.time()<end: rclpy.spin_once(node, timeout_sec=0.1)
for p,c in sorted(frames_seen): print(p,"->",c)
print("----")
for f in sorted({c for _,c in frames_seen}):
    try:
        t = buf.lookup_transform("world", f, rclpy.time.Time())
        tr=t.transform.translation; q=t.transform.rotation
        print(f"{f:40s} xyz=({tr.x:.3f},{tr.y:.3f},{tr.z:.3f}) q=({q.x:.3f},{q.y:.3f},{q.z:.3f},{q.w:.3f})")
    except Exception as e:
        print(f"{f:40s} ERR {str(e)[:60]}")
