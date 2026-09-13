import rclpy, yaml
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
rclpy.init(); node=rclpy.create_node("tfdump")
seen={}
def cb(m):
    for t in m.transforms:
        seen[(t.header.frame_id,t.child_frame_id)]=t.transform
qos=QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)
node.create_subscription(TFMessage,"/tf_static",cb,qos)
node.create_subscription(TFMessage,"/tf",cb,10)
import time
for _ in range(40): rclpy.spin_once(node,timeout_sec=0.1)
for k,v in sorted(seen.items()):
    print(k, f"t=({v.translation.x:.4f},{v.translation.y:.4f},{v.translation.z:.4f}) q=({v.rotation.x:.4f},{v.rotation.y:.4f},{v.rotation.z:.4f},{v.rotation.w:.4f})")
