import rclpy, time
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
rclpy.init(); n=rclpy.create_node('tfdump')
seen={}
def cb(m):
    for t in m.transforms:
        seen[(t.header.frame_id,t.child_frame_id)]=t.transform
qos=QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
n.create_subscription(TFMessage,'/tf_static',cb,qos)
n.create_subscription(TFMessage,'/tf',cb,10)
t0=time.time()
while time.time()-t0<5: rclpy.spin_once(n,timeout_sec=0.2)
for k,v in sorted(seen.items()):
    print(k, "t=(%.4f %.4f %.4f) q=(%.4f %.4f %.4f %.4f)"%(v.translation.x,v.translation.y,v.translation.z,v.rotation.x,v.rotation.y,v.rotation.z,v.rotation.w))
