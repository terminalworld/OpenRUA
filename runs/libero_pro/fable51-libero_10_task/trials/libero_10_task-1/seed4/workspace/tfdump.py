import rclpy, time
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy
rclpy.init(); n=rclpy.create_node('tfdump')
got={}
def cb(m):
    for t in m.transforms:
        got[(t.header.frame_id,t.child_frame_id)]=t.transform
qos=QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
n.create_subscription(TFMessage,'/tf_static',cb,qos)
n.create_subscription(TFMessage,'/tf',cb,10)
end=time.time()+5
while time.time()<end: rclpy.spin_once(n,timeout_sec=0.2)
for k,v in sorted(got.items()):
    print(k, f"t=({v.translation.x:.3f},{v.translation.y:.3f},{v.translation.z:.3f}) q=({v.rotation.x:.3f},{v.rotation.y:.3f},{v.rotation.z:.3f},{v.rotation.w:.3f})")
