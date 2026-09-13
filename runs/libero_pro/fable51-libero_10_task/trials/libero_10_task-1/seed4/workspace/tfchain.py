import rclpy, time, numpy as np
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy
from ctl import quat_R
rclpy.init(); n=rclpy.create_node('tfc')
got={}
def cb(m):
    for t in m.transforms: got[(t.header.frame_id,t.child_frame_id)]=t.transform
n.create_subscription(TFMessage,'/tf_static',cb,QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
n.create_subscription(TFMessage,'/tf',cb,10)
end=time.time()+4
while time.time()<end: rclpy.spin_once(n,timeout_sec=0.2)
chain=['world','panda_link0','panda_link1','panda_link2','panda_link3','panda_link4','panda_link5','panda_link6','panda_link7','panda_link8','panda_hand']
T=np.eye(4)
for a,b in zip(chain,chain[1:]):
    tr=got[(a,b)]; M=np.eye(4); M[:3,:3]=quat_R(tr.rotation.x,tr.rotation.y,tr.rotation.z,tr.rotation.w)
    M[:3,3]=[tr.translation.x,tr.translation.y,tr.translation.z]; T=T@M
    print(b, np.round(T[:3,3],4))
print('hand R', np.round(T[:3,:3],3))
