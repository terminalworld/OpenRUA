#!/usr/bin/env python3
"""scan.py cam zmin zmax xmin xmax ymin ymax [grid=0.02] -> occupancy grid of world points in the box (from cam depth)."""
import sys, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from scipy.spatial.transform import Rotation as R
cam=sys.argv[1]; zmin,zmax,xmin,xmax,ymin,ymax=map(float,sys.argv[2:8]); g=float(sys.argv[8]) if len(sys.argv)>8 else 0.02
rclpy.init(); node=rclpy.create_node("scan"); buf=Buffer(); TransformListener(buf,node)
got={}
node.create_subscription(Image,f"/{cam}/depth/image_raw",lambda m: got.setdefault("d",m),1)
node.create_subscription(CameraInfo,f"/{cam}/color/camera_info",lambda m: got.setdefault("i",m),1)
while not("d" in got and "i" in got and buf.can_transform("world",f"{cam}_optical_frame",rclpy.time.Time())):
    rclpy.spin_once(node,timeout_sec=0.2)
d=np.frombuffer(got["d"].data,dtype=np.float32).reshape(got["d"].height,got["d"].width)
k=got["i"].k; fx,fy,cx,cy=k[0],k[4],k[2],k[5]
t=buf.lookup_transform("world",f"{cam}_optical_frame",rclpy.time.Time())
q=t.transform.rotation; tr=t.transform.translation
Rm=R.from_quat([q.x,q.y,q.z,q.w]).as_matrix(); T=np.array([tr.x,tr.y,tr.z])
v,u=np.mgrid[0:d.shape[0],0:d.shape[1]]
ok=np.isfinite(d)&(d>0.01)
P=np.stack([(u-cx)*d/fx,(v-cy)*d/fy,d],-1)[ok]@Rm.T+T
sel=(P[:,2]>zmin)&(P[:,2]<zmax)&(P[:,0]>xmin)&(P[:,0]<xmax)&(P[:,1]>ymin)&(P[:,1]<ymax)
pts=P[sel]; print("n",len(pts))
xs=np.arange(xmin,xmax,g); ys=np.arange(ymin,ymax,g)
H=np.zeros((len(ys),len(xs))); Z=np.zeros_like(H)
for p in pts:
    i=int((p[1]-ymin)/g); j=int((p[0]-xmin)/g); H[i,j]+=1; Z[i,j]=max(Z[i,j],p[2])
print("rows=y (top=ymin) cols=x (left=xmin); cell = max z (cm) or . if empty")
print("      "+" ".join(f"{x*100:4.0f}" for x in xs))
for i,y in enumerate(ys):
    print(f"{y*100:5.0f} "+" ".join(f"{Z[i,j]*100:4.1f}" if H[i,j]>0 else "   ." for j in range(len(xs))))
