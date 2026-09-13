import rclpy, yaml
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
rclpy.init(); node=rclpy.create_node('tfdump')
seen={}
def cb(msg):
    for t in msg.transforms:
        seen[(t.header.frame_id,t.child_frame_id)]=t.transform
qos=QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
node.create_subscription(TFMessage,'/tf_static',cb,qos)
node.create_subscription(TFMessage,'/tf',cb,10)
import time
for _ in range(30): rclpy.spin_once(node,timeout_sec=0.2)
for k,v in sorted(seen.items()):
    print(k, 'xyz=(%.4f %.4f %.4f) q=(%.4f %.4f %.4f %.4f)'%(v.translation.x,v.translation.y,v.translation.z,v.rotation.x,v.rotation.y,v.rotation.z,v.rotation.w))
