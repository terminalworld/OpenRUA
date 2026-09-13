import numpy as np, rclpy, time
from tf2_ros import Buffer, TransformListener
d=np.load("snaps/agentview_depth.npy"); fx=579.4112549695428; cx=320; cy=240
rclpy.init(); n=rclpy.create_node("c"); b=Buffer(); TransformListener(b,n)
end=time.time()+5
while time.time()<end and not b.can_transform("world","agentview_optical_frame",rclpy.time.Time()): rclpy.spin_once(n,timeout_sec=0.1)
t=b.lookup_transform("world","agentview_optical_frame",rclpy.time.Time()).transform
q=t.rotation; x,y,z,w=q.x,q.y,q.z,q.w
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
T=np.array([t.translation.x,t.translation.y,t.translation.z])
def W(u,v): Z=d[v,u]; return R@np.array([(u-cx)*Z/fx,(v-cy)*Z/fx,Z])+T
np.set_printoptions(precision=3, suppress=True, linewidth=200)
for v in range(160,340,10):
    print(v, " ".join(f"[{u}:{W(u,v)[0]:.2f},{W(u,v)[1]:.2f},{W(u,v)[2]:.2f}]" for u in range(440,530,10)))
rclpy.shutdown()
