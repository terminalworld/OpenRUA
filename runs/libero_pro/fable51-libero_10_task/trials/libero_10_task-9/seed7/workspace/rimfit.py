import numpy as np, rclpy, time, sys
from tf2_ros import Buffer, TransformListener
cam="robot0_eye_in_hand"
d=np.load(f"snaps/{cam}_depth.npy"); fx=312.77408948188935; cx=320; cy=240
rclpy.init(); n=rclpy.create_node("c"); b=Buffer(); TransformListener(b,n)
end=time.time()+5
while time.time()<end and not b.can_transform("world",f"{cam}_optical_frame",rclpy.time.Time()): rclpy.spin_once(n,timeout_sec=0.1)
t=b.lookup_transform("world",f"{cam}_optical_frame",rclpy.time.Time()).transform
q=t.rotation; x,y,z,w=q.x,q.y,q.z,q.w
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
T=np.array([t.translation.x,t.translation.y,t.translation.z])
print("cam pos",T)
vs,us=np.mgrid[0:480,0:640]
Z=d; P=np.stack([(us-cx)*Z/fx,(vs-cy)*Z/fx,Z],-1)@R.T+T
zw=P[...,2]
m=(zw>1.0)&(zw<1.02)&(vs<400)
pts=P[m][:,:2]; print("rim pts",len(pts))
# algebraic circle fit
A=np.c_[2*pts, np.ones(len(pts))]; bb=(pts**2).sum(1)
c=np.linalg.lstsq(A,bb,rcond=None)[0]; cx_,cy_=c[0],c[1]; r_=np.sqrt(c[2]+cx_**2+cy_**2)
print(f"rim circle centre=({cx_:.4f},{cy_:.4f}) r={r_:.4f}")
