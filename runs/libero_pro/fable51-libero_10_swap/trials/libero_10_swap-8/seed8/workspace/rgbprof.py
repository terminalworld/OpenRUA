import numpy as np, rclpy, cv2
from rob import *
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener
r=Rob("rgbprof"); buf=Buffer(); TransformListener(buf,r.node)
cam="sideview"; fr=f"{cam}_optical_frame"
for _ in range(50):
    r.spin(0.1)
    if buf.can_transform("world",fr,rclpy.time.Time()): break
t=buf.lookup_transform("world",fr,rclpy.time.Time()).transform
tt=np.array([t.translation.x,t.translation.y,t.translation.z]); q=[t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w]
img=cv2.imread(r.snap(cam,"/workspace/sv.png"))
P=r.depth_world(cam,tt,q)
B=np.array([-0.064,0.234])
# pot pixels: world points within 6cm of B in xy and above table
d=np.linalg.norm(P[:,:,:2]-B,axis=2); mask=(d<0.06)&(P[:,:,2]>0.9)&(P[:,:,2]<1.06)
vs,us=np.where(mask); print("depth-mask bbox u",us.min(),us.max(),"v",vs.min(),vs.max())
# RGB segmentation inside a padded bbox: not wood (wood is warm: R>G>B strongly)
u0,u1,v0,v1=us.min()-15,us.max()+15,vs.min()-10,vs.max()+10
sub=img[v0:v1,u0:u1].astype(int); b,g,rr=sub[...,0],sub[...,1],sub[...,2]
wood=(rr-b>25)                       # table is orange-ish
pot=~wood
Rw=quat_to_R(q); fx=None
from sensor_msgs.msg import CameraInfo
got={}
sub_=r.node.create_subscription(CameraInfo,f"/{cam}/color/camera_info",lambda m:got.setdefault("m",m),1)
while "m" not in got: r.spin(0.2)
fx=got["m"].k[0]; fy=got["m"].k[4]; cx=got["m"].k[2]; cy=got["m"].k[5]
Zc=np.linalg.norm(np.array([B[0],B[1],0.98])-tt)   # distance to pot centre
print("fx",fx,"Zc",Zc, "mm/px", 1000*Zc/fx)
cv2.imwrite("/workspace/sv_crop.png", np.where(pot[...,None],sub,sub//3).astype(np.uint8))
for v in range(0,v1-v0):
    row=pot[v]; 
    if row.sum()<3: continue
    uu=np.where(row)[0]; w=(uu.max()-uu.min()+1)*Zc/fx
    zrow=P[v0+v, u0+uu.min():u0+uu.max()+1, 2]; zrow=zrow[np.isfinite(zrow)]
    zz=np.median(zrow) if zrow.size else float('nan')
    print(f"row {v0+v}: width={100*w:4.1f}cm  z~{zz:.3f}  u=[{u0+uu.min()},{u0+uu.max()}]")
