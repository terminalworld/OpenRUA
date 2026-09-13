import rclpy, time
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
rclpy.init(); node=rclpy.create_node('tfdump')
frames={}
def cb(msg):
    for t in msg.transforms:
        frames[(t.header.frame_id,t.child_frame_id)]=t.transform
qos=QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
node.create_subscription(TFMessage,'/tf_static',cb,qos)
node.create_subscription(TFMessage,'/tf',cb,10)
t0=time.time()
while time.time()-t0<5: rclpy.spin_once(node,timeout_sec=0.2)
for k,v in sorted(frames.items()):
    print(k, f"t=({v.translation.x:.4f},{v.translation.y:.4f},{v.translation.z:.4f}) q=({v.rotation.x:.4f},{v.rotation.y:.4f},{v.rotation.z:.4f},{v.rotation.w:.4f})")
