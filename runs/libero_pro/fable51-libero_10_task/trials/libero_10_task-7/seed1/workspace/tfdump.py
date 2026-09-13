import rclpy, yaml
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
rclpy.init(); node = rclpy.create_node("tfdump")
seen = {}
def cb(m):
    for t in m.transforms:
        tr=t.transform.translation; r=t.transform.rotation
        seen[(t.header.frame_id,t.child_frame_id)] = (round(tr.x,4),round(tr.y,4),round(tr.z,4),round(r.x,4),round(r.y,4),round(r.z,4),round(r.w,4))
qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
node.create_subscription(TFMessage, "/tf_static", cb, qos)
node.create_subscription(TFMessage, "/tf", cb, 100)
import time
end=time.time()+5
while time.time()<end: rclpy.spin_once(node, timeout_sec=0.2)
for k,v in sorted(seen.items()): print(k, v)
