"""Usage: python3 pxw.py <cam> u,v u,v ...   -> world xyz per pixel; also saves depth npy"""
import sys, struct, numpy as np, rclpy, time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge
cam=sys.argv[1]; pts=[tuple(map(int,a.split(","))) for a in sys.argv[2:]]
rclpy.init(); n=rclpy.create_node("pxw"); b=Buffer(); TransformListener(b,n)
got={}
n.create_subscription(Image,f"/{cam}/depth/image_raw",lambda m:got.setdefault("d",m),1)
n.create_subscription(CameraInfo,f"/{cam}/color/camera_info",lambda m:got.setdefault("i",m),1)
end=time.time()+30
while time.time()<end and not ("d" in got and "i" in got and b.can_transform("world",f"{cam}_optical_frame",rclpy.time.Time())):
    rclpy.spin_once(n,timeout_sec=0.1)
d=CvBridge().imgmsg_to_cv2(got["d"],"passthrough").astype(np.float32); np.save(f"snaps/{cam}_depth.npy",d)
k=got["i"].k; fx,fy,cx,cy=k[0],k[4],k[2],k[5]
t=b.lookup_transform("world",f"{cam}_optical_frame",rclpy.time.Time()).transform
q=t.rotation; x,y,z,w=q.x,q.y,q.z,q.w
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
T=np.array([t.translation.x,t.translation.y,t.translation.z])
print("cam",cam,"size",d.shape,"fx",fx,"cx",cx,"cy",cy)
for u,v in pts:
    Z=d[v,u]; p=R@np.array([(u-cx)*Z/fx,(v-cy)*Z/fy,Z])+T
    print(f"px({u},{v}) depth={Z:.3f} world=({p[0]:.3f},{p[1]:.3f},{p[2]:.3f})")
rclpy.shutdown()
