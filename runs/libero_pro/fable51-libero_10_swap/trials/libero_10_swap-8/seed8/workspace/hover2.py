import sys, numpy as np, rclpy
from rob import *
from tf2_ros import Buffer, TransformListener
r=Rob("hover2")
x,y,z,tx,ty,hyaz=map(float,sys.argv[1:7]); tag=sys.argv[7]
hz=np.array([tx,ty,-np.sqrt(1-tx*tx-ty*ty)]); hy=np.array([np.cos(np.radians(hyaz)),np.sin(np.radians(hyaz)),0.0])
R=R_from_axes(hz,hy); q=r.ik([x,y,z],R,at_tcp=True)
if q is None: raise SystemExit("IK FAIL")
dur=max(3.0,np.abs(q-r.arm_q()).max()/0.15); print(r.move_q(q,dur)); p,Rn=r.tcp(); print("tcp",p.round(3),"hz",Rn[:,2].round(2))
r.snap("robot0_eye_in_hand",f"/workspace/{tag}_eye.png")
buf=Buffer(); TransformListener(buf,r.node); fr="robot0_eye_in_hand_optical_frame"
while not buf.can_transform("world",fr,rclpy.time.Time()): r.spin(0.2)
t=buf.lookup_transform("world",fr,rclpy.time.Time()); tr=t.transform.translation; ro=t.transform.rotation
P=r.depth_world("robot0_eye_in_hand",[tr.x,tr.y,tr.z],[ro.x,ro.y,ro.z,ro.w]); np.save(f"/workspace/{tag}_eye_world.npy",P)
print("cam at",np.round([tr.x,tr.y,tr.z],3))
