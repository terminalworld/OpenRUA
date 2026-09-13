import numpy as np, rclpy, time
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy
def qR(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
rclpy.init(); node=rclpy.create_node("chain")
seen={}
def cb(m):
    for t in m.transforms: seen[t.child_frame_id]=t
node.create_subscription(TFMessage,"/tf_static",cb,QoSProfile(depth=100,durability=DurabilityPolicy.TRANSIENT_LOCAL))
node.create_subscription(TFMessage,"/tf",cb,100)
end=time.time()+4
while time.time()<end: rclpy.spin_once(node,timeout_sec=0.2)
def T(child):
    t=seen[child]; M=np.eye(4); r=t.transform.rotation; tr=t.transform.translation
    M[:3,:3]=qR(r.x,r.y,r.z,r.w); M[:3,3]=[tr.x,tr.y,tr.z]; return M
M=np.eye(4)
for c in ["panda_link1","panda_link2","panda_link3","panda_link4","panda_link5","panda_link6","panda_link7","panda_link8","panda_hand"]:
    M=M@T(c)
print("base->hand pos", M[:3,3].round(4)); print("R=\n",M[:3,:3].round(3))
W=T("panda_link0")@M
print("world->hand pos", W[:3,3].round(4))
