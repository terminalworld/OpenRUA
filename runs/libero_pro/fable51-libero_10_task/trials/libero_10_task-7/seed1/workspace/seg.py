import sys, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy
from scipy import ndimage
def grab(node, topic, T, qos=1):
    got={}
    s=node.create_subscription(T, topic, lambda m: got.setdefault("m",m), qos)
    while "m" not in got: rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(s); return got["m"]
def qR(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
cam=sys.argv[1]; zmin=float(sys.argv[2]) if len(sys.argv)>2 else 0.44
zmax=float(sys.argv[3]) if len(sys.argv)>3 else 0.75
roi=[int(x) for x in sys.argv[4].split(",")] if len(sys.argv)>4 else None
rclpy.init(); node=rclpy.create_node("seg")
depth=grab(node,f"/{cam}/depth/image_raw",Image)
info=grab(node,f"/{cam}/color/camera_info",CameraInfo)
frame=f"{cam}_optical_frame"
qos=QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)
seen={}
def cb(m):
    for t in m.transforms: seen[t.child_frame_id]=t
node.create_subscription(TFMessage,"/tf_static",cb,qos)
node.create_subscription(TFMessage,"/tf",cb,100)
import time; end=time.time()+4
while time.time()<end and frame not in seen: rclpy.spin_once(node,timeout_sec=0.2)
tf=seen[frame]
D=np.frombuffer(depth.data,dtype=np.float32).reshape(depth.height,depth.width)
fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
R=qR(tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z,tf.transform.rotation.w)
t=np.array([tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z])
v,u=np.mgrid[0:depth.height,0:depth.width]
P=np.stack([(u-cx)*D/fx,(v-cy)*D/fy,D],-1)@R.T+t
X,Y,Z=P[...,0],P[...,1],P[...,2]
mask=(Z>zmin)&(Z<zmax)&(X>-0.35)&(X<0.45)&(np.abs(Y)<0.5)&np.isfinite(D)
if roi: mask&=(u>=roi[0])&(u<=roi[1])&(v>=roi[2])&(v<=roi[3])
lab,n=ndimage.label(mask)
for i in range(1,n+1):
    m=lab==i
    if m.sum()<4: continue
    print(f"blob{i}: px={m.sum()} u={u[m].mean():.0f} v={v[m].mean():.0f} x[{X[m].min():.3f},{X[m].max():.3f}] y[{Y[m].min():.3f},{Y[m].max():.3f}] z[{Z[m].min():.3f},{Z[m].max():.3f}] centroid=({X[m].mean():.3f},{Y[m].mean():.3f})")
