#!/usr/bin/env python3
"""lid.py zmin zmax [cam] -> centroid (world) of wrist-camera depth pixels whose world z is in [zmin,zmax]."""
import sys, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from scipy.spatial.transform import Rotation as R
zmin,zmax=float(sys.argv[1]),float(sys.argv[2]); cam=sys.argv[3] if len(sys.argv)>3 and sys.argv[3]!="-" else "robot0_eye_in_hand"
rclpy.init(); node=rclpy.create_node("lid"); buf=Buffer(); TransformListener(buf,node)
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
sel=(P[:,2]>zmin)&(P[:,2]<zmax)&(abs(P[:,0]-float(sys.argv[4]))<0.06)&(abs(P[:,1]-float(sys.argv[5]))<0.06)
pts=P[sel]
print("n",sel.sum())
if sel.sum():
    print("centroid",np.round(pts.mean(0),4).tolist(),"x",np.round([pts[:,0].min(),pts[:,0].max()],4).tolist(),"y",np.round([pts[:,1].min(),pts[:,1].max()],4).tolist(),"z",np.round([pts[:,2].min(),pts[:,2].max()],4).tolist())
