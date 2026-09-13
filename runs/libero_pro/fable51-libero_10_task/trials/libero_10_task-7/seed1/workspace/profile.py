import sys, numpy as np, rclpy, time
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy
def grab(node, topic, T, qos=1):
    got={}
    s=node.create_subscription(T, topic, lambda m: got.setdefault("m",m), qos)
    while "m" not in got: rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(s); return got["m"]
def qR(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
cam=sys.argv[1]; x0,x1,y0,y1=map(float,sys.argv[2:6]); zmin=float(sys.argv[6])
rclpy.init(); node=rclpy.create_node("prof")
depth=grab(node,f"/{cam}/depth/image_raw",Image); info=grab(node,f"/{cam}/color/camera_info",CameraInfo)
frame=f"{cam}_optical_frame"; seen={}
def cb(m):
    for t in m.transforms: seen[t.child_frame_id]=t
node.create_subscription(TFMessage,"/tf_static",cb,QoSProfile(depth=100,durability=DurabilityPolicy.TRANSIENT_LOCAL))
node.create_subscription(TFMessage,"/tf",cb,100)
end=time.time()+4
while time.time()<end and frame not in seen: rclpy.spin_once(node,timeout_sec=0.2)
tf=seen[frame]
D=np.frombuffer(depth.data,dtype=np.float32).reshape(depth.height,depth.width)
fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
R=qR(tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z,tf.transform.rotation.w)
t=np.array([tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z])
v,u=np.mgrid[0:depth.height,0:depth.width]
P=np.stack([(u-cx)*D/fx,(v-cy)*D/fy,D],-1)@R.T+t
X,Y,Z=P[...,0],P[...,1],P[...,2]
m=(X>x0)&(X<x1)&(Y>y0)&(Y<y1)&(Z>zmin)&np.isfinite(D)
print("overall x[%.3f,%.3f] y[%.3f,%.3f] zmax %.3f n=%d"%(X[m].min(),X[m].max(),Y[m].min(),Y[m].max(),Z[m].max(),m.sum()))
for xa in np.arange(x0,x1,0.01):
    mm=m&(X>=xa)&(X<xa+0.01)
    if mm.sum()>3: print(f"x={xa:.2f}: n={mm.sum():4d} y[{Y[mm].min():.3f},{Y[mm].max():.3f}] ymid={0.5*(Y[mm].min()+Y[mm].max()):.3f} zmax={Z[mm].max():.3f}")
# PCA of xy
pts=np.stack([X[m],Y[m]],1); c=pts.mean(0); w,vec=np.linalg.eigh(np.cov((pts-c).T))
print("centroid",c.round(4),"axis",vec[:,1].round(3),"angle_deg",np.degrees(np.arctan2(vec[1,1],vec[0,1])).round(1))
