import rclpy
from tf2_ros import Buffer, TransformListener
rclpy.init(); node=rclpy.create_node("hp"); buf=Buffer(); TransformListener(buf,node)
import rclpy.time
for _ in range(40):
    rclpy.spin_once(node,timeout_sec=0.1)
    if buf.can_transform("panda_link0","panda_hand",rclpy.time.Time()) and buf.can_transform("world","panda_hand",rclpy.time.Time()): break
for a,b in [("panda_link0","panda_hand"),("world","panda_hand"),("world","panda_leftfinger"),("world","panda_rightfinger")]:
    t=buf.lookup_transform(a,b,rclpy.time.Time()).transform
    print(f"{a}->{b}: t=({t.translation.x:.4f},{t.translation.y:.4f},{t.translation.z:.4f}) q=({t.rotation.x:.4f},{t.rotation.y:.4f},{t.rotation.z:.4f},{t.rotation.w:.4f})")
