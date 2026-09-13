import sys, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
cam = sys.argv[1]
rclpy.init(); node = rclpy.create_node("seg"); tfbuf=Buffer(); TransformListener(tfbuf,node)
def grab(topic, T):
    got={}; s=node.create_subscription(T, topic, lambda m: got.setdefault("m",m),1)
    while "m" not in got: rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(s); return got["m"]
depth=grab(f"/{cam}/depth/image_raw", Image); info=grab(f"/{cam}/color/camera_info", CameraInfo)
D=np.frombuffer(depth.data, dtype=np.float32).reshape(depth.height, depth.width)
frame=f"{cam}_optical_frame"
while not tfbuf.can_transform("world", frame, rclpy.time.Time()): rclpy.spin_once(node, timeout_sec=0.2)
t=tfbuf.lookup_transform("world", frame, rclpy.time.Time()); q=t.transform.rotation
x,y,z,w=q.x,q.y,q.z,q.w
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
T=np.eye(4); T[:3,:3]=R; T[:3,3]=[t.transform.translation.x,t.transform.translation.y,t.transform.translation.z]
fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
H,W=D.shape; vv,uu=np.mgrid[0:H,0:W]
P=np.stack([(uu-cx)*D/fx,(vv-cy)*D/fy,D,np.ones_like(D)],-1).reshape(-1,4)@T.T
P=P[:,:3].reshape(H,W,3)
np.save(f"snaps/{cam}_world.npy", P)
print("cam pos", T[:3,3], "R", R.round(3).tolist())
# report by ranges
zs=P[...,2]
print("z hist", np.histogram(zs[np.isfinite(zs)], bins=20)[0], np.histogram(zs[np.isfinite(zs)], bins=20)[1].round(3))
