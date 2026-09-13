import numpy as np, rclpy
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy
exec(open("pxw.py").read().split("def grab")[0])
rclpy.init(); node=rclpy.create_node("tfc")
got={}
def cb(m):
    for t in m.transforms: got[(t.header.frame_id,t.child_frame_id)]=t.transform
qos=QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)
node.create_subscription(TFMessage,"/tf_static",cb,qos); node.create_subscription(TFMessage,"/tf",cb,100)
for _ in range(20): rclpy.spin_once(node,timeout_sec=0.2)
def M(t):
    T=np.eye(4); T[:3,:3]=R_of((t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w)); T[:3,3]=[t.translation.x,t.translation.y,t.translation.z]; return T
chain=["world","panda_link0"]+[f"panda_link{i}" for i in range(1,9)]+["panda_hand"]
T=np.eye(4)
for a,b in zip(chain,chain[1:]):
    T=T@M(got[(a,b)])
    print(b, np.round(T[:3,3],4))
print("hand R:\n",np.round(T[:3,:3],3))
rclpy.shutdown()
