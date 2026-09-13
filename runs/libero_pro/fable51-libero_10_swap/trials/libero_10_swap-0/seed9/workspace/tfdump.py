import rclpy, yaml
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
rclpy.init(); node=rclpy.create_node("tfdump")
got={}
qos=QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
def cb(m, key):
    for t in m.transforms:
        got[(key,t.header.frame_id,t.child_frame_id)]=t
node.create_subscription(TFMessage,"/tf_static",lambda m: cb(m,"static"),qos)
node.create_subscription(TFMessage,"/tf",lambda m: cb(m,"dyn"),10)
import time
end=time.time()+5
while time.time()<end: rclpy.spin_once(node,timeout_sec=0.2)
for k,t in sorted(got.items()):
    tr=t.transform.translation; q=t.transform.rotation
    print(k, f"t=({tr.x:.4f},{tr.y:.4f},{tr.z:.4f}) q=({q.x:.4f},{q.y:.4f},{q.z:.4f},{q.w:.4f})")
