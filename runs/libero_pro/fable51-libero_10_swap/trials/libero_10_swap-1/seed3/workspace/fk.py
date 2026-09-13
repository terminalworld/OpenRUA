import rclpy, time
from tf2_ros import Buffer, TransformListener
rclpy.init(); n=rclpy.create_node('fk')
b=Buffer(); TransformListener(b,n)
t0=time.time()
while time.time()-t0<5 and not (b.can_transform('world','panda_hand',rclpy.time.Time()) and b.can_transform('panda_link0','panda_hand',rclpy.time.Time())):
    rclpy.spin_once(n,timeout_sec=0.2)
for parent in ['world','panda_link0']:
    t=b.lookup_transform(parent,'panda_hand',rclpy.time.Time())
    tr=t.transform.translation; q=t.transform.rotation
    print(f"{parent}->panda_hand t=({tr.x:.4f},{tr.y:.4f},{tr.z:.4f}) q=({q.x:.4f},{q.y:.4f},{q.z:.4f},{q.w:.4f})")
