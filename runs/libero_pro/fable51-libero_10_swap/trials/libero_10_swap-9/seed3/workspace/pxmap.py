import sys, struct, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
cam=sys.argv[1]; pts=[tuple(map(int,a.split(','))) for a in sys.argv[2:]]
rclpy.init(); node=rclpy.create_node("pxmap"); buf=Buffer(); TransformListener(buf,node)
got={}
node.create_subscription(Image,f"/{cam}/depth/image_raw",lambda m:got.setdefault("d",m),1)
node.create_subscription(CameraInfo,f"/{cam}/color/camera_info",lambda m:got.setdefault("i",m),1)
while len(got)<2 or not buf.can_transform("world",f"{cam}_optical_frame",rclpy.time.Time()): rclpy.spin_once(node,timeout_sec=0.2)
d=got["d"]; info=got["i"]
depth=np.frombuffer(d.data,dtype=np.float32).reshape(d.height,d.width)
np.save(f"{cam}_depth.npy",depth)
fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
t=buf.lookup_transform("world",f"{cam}_optical_frame",rclpy.time.Time()); q=t.transform.rotation
x,y,z,w=q.x,q.y,q.z,q.w
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
T=np.array([t.transform.translation.x,t.transform.translation.y,t.transform.translation.z])
for u,v in pts:
    Z=depth[v,u]; p=R@np.array([(u-cx)*Z/fx,(v-cy)*Z/fy,Z])+T
    print(f"px({u},{v}) depth={Z:.3f} world=({p[0]:.3f},{p[1]:.3f},{p[2]:.3f})")
