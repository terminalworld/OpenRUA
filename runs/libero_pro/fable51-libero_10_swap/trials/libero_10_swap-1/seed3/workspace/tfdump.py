import rclpy, time
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
rclpy.init(); n=rclpy.create_node('tfdump')
got={}
def cb(m, static):
    for t in m.transforms:
        got[(t.header.frame_id,t.child_frame_id)] = (t.transform.translation, t.transform.rotation, static)
qos=QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
n.create_subscription(TFMessage,'/tf_static',lambda m: cb(m,True),qos)
n.create_subscription(TFMessage,'/tf',lambda m: cb(m,False),100)
t0=time.time()
while time.time()-t0<4: rclpy.spin_once(n,timeout_sec=0.2)
for k,(tr,q,s) in sorted(got.items()):
    print(('S ' if s else 'D '),k, f"t=({tr.x:.4f},{tr.y:.4f},{tr.z:.4f}) q=({q.x:.4f},{q.y:.4f},{q.z:.4f},{q.w:.4f})")
