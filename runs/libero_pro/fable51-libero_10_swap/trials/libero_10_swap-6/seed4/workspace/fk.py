"""FK of panda_hand and TCP in world from current /joint_states via TF chain (numpy)."""
import numpy as np, rclpy, time, sys
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy
from scipy.spatial.transform import Rotation as Rot
def get_chain():
    rclpy.init(); node = rclpy.create_node("fk")
    got = {}
    qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    def cb(m):
        for t in m.transforms: got[(t.header.frame_id, t.child_frame_id)] = t.transform
    node.create_subscription(TFMessage, "/tf_static", cb, qos)
    node.create_subscription(TFMessage, "/tf", cb, 100)
    end=time.time()+5
    while time.time()<end: rclpy.spin_once(node, timeout_sec=0.2)
    rclpy.shutdown(); return got
def T_of(t):
    M=np.eye(4); M[:3,:3]=Rot.from_quat([t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w]).as_matrix()
    M[:3,3]=[t.translation.x,t.translation.y,t.translation.z]; return M
if __name__=="__main__":
    g=get_chain()
    chain=["world","panda_link0","panda_link1","panda_link2","panda_link3","panda_link4","panda_link5","panda_link6","panda_link7","panda_link8","panda_hand"]
    M=np.eye(4)
    for a,b in zip(chain,chain[1:]): M=M@T_of(g[(a,b)])
    tcp=M@np.array([0,0,0.1034,1])
    print("hand pos world", M[:3,3].round(4), "quat xyzw", Rot.from_matrix(M[:3,:3]).as_quat().round(4))
    print("hand axes in world: x", M[:3,0].round(3), "y", M[:3,1].round(3), "z", M[:3,2].round(3))
    print("TCP world", tcp[:3].round(4))
    for i in range(1,8):
        t=g[(f"panda_link{i-1}",f"panda_link{i}")]
