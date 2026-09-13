import sys, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
cam=sys.argv[1]
def grab(node, topic, T):
    got={}; s=node.create_subscription(T, topic, lambda m: got.setdefault("m",m),1)
    while "m" not in got: rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(s); return got["m"]
rclpy.init(); node=rclpy.create_node("cloud"); buf=Buffer(); TransformListener(buf,node)
d=grab(node,f"/{cam}/depth/image_raw",Image); info=grab(node,f"/{cam}/color/camera_info",CameraInfo)
D=np.frombuffer(d.data,dtype=np.float32).reshape(d.height,d.width)
fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
frame=f"{cam}_optical_frame"
while not buf.can_transform("world",frame,rclpy.time.Time()): rclpy.spin_once(node,timeout_sec=0.2)
t=buf.lookup_transform("world",frame,rclpy.time.Time()); q=t.transform.rotation
x,y,zz,w=q.x,q.y,q.z,q.w
R=np.array([[1-2*(y*y+zz*zz),2*(x*y-zz*w),2*(x*zz+y*w)],[2*(x*y+zz*w),1-2*(x*x+zz*zz),2*(y*zz-x*w)],[2*(x*zz-y*w),2*(y*zz+x*w),1-2*(x*x+y*y)]])
T=np.eye(4); T[:3,:3]=R; T[:3,3]=[t.transform.translation.x,t.transform.translation.y,t.transform.translation.z]
v,u=np.mgrid[0:d.height,0:d.width]
Z=D; ok=np.isfinite(Z)&(Z>0.05)
P=np.stack([(u-cx)*Z/fx,(v-cy)*Z/fy,Z,np.ones_like(Z)],-1)
W=(P@T.T)[...,:3]
np.save(f"{cam}_world.npy", W); np.save(f"{cam}_ok.npy", ok)
print("cam at", T[:3,3], "saved", W.shape)
rclpy.shutdown()
