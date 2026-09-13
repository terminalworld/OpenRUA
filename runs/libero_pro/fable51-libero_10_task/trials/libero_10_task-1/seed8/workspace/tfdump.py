import rclpy, yaml
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
rclpy.init(); n=rclpy.create_node("tfd")
got=[]
qos=QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
n.create_subscription(TFMessage,"/tf_static",lambda m:got.append(("static",m)),qos)
n.create_subscription(TFMessage,"/tf",lambda m:got.append(("dyn",m)),10)
import time
t=time.time()
while time.time()-t<4: rclpy.spin_once(n,timeout_sec=0.2)
seen={}
for k,m in got:
    for tr in m.transforms:
        seen[(k,tr.header.frame_id,tr.child_frame_id)]=tr.transform
for (k,p,c),t in seen.items():
    print(k,p,"->",c,"t=(%.3f %.3f %.3f) q=(%.3f %.3f %.3f %.3f)"%(t.translation.x,t.translation.y,t.translation.z,t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w))
