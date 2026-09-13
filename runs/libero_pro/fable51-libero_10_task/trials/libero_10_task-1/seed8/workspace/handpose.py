import rclpy, time
from tf2_ros import Buffer, TransformListener
rclpy.init(); n=rclpy.create_node("hp"); b=Buffer(); TransformListener(b,n)
t0=time.time()
while time.time()-t0<5: rclpy.spin_once(n,timeout_sec=0.1)
for a,c in [("panda_link0","panda_hand"),("world","panda_hand"),("world","panda_link0")]:
    try:
        t=b.lookup_transform(a,c,rclpy.time.Time()).transform
        print(a,"->",c,"t=(%.4f %.4f %.4f) q=(%.4f %.4f %.4f %.4f)"%(t.translation.x,t.translation.y,t.translation.z,t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w))
    except Exception as e: print(a,c,"ERR",e)
