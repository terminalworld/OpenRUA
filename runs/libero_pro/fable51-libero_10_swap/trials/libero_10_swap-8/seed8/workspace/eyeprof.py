import numpy as np, rclpy, sys
from rob import *
from tf2_ros import Buffer, TransformListener
r=Rob("eyeprof"); buf=Buffer(); TransformListener(buf,r.node)
sols=np.load("sols.npy",allow_pickle=True).item()
if "back" in sys.argv:
    r.gripper(0.04); r.move_q(sols["A_pre"],3.0)
cam="robot0_eye_in_hand"; fr=f"{cam}_optical_frame"
for _ in range(50):
    r.spin(0.1)
    if buf.can_transform("world",fr,rclpy.time.Time()): break
t=buf.lookup_transform("world",fr,rclpy.time.Time()).transform
tt=[t.translation.x,t.translation.y,t.translation.z]; q=[t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w]
print("cam pos",np.round(tt,3),"tcp",r.tcp()[0].round(3))
P=r.depth_world(cam,tt,q); np.save("eye_world.npy",P); P=P.reshape(-1,3); P=P[np.isfinite(P).all(1)]
A=np.array([-0.195,-0.200])
d=P[:,:2]-A; rr=np.linalg.norm(d,axis=1); m=(rr<0.09)&(P[:,2]>0.895)&(P[:,2]<1.07)
Q=P[m]; dd=d[m]; az=np.degrees(np.arctan2(dd[:,1],dd[:,0]))
print("n",m.sum())
for z0 in np.arange(0.895,1.06,0.005):
    mm=(Q[:,2]>=z0)&(Q[:,2]<z0+0.005)
    if mm.sum()>3:
        # radius in the visible az range, and lateral width along 45deg axis (finger axis)
        lat=dd[mm]@np.array([0.7071,0.7071]); ap=dd[mm]@np.array([0.7071,-0.7071])
        print(f"  z={z0:.3f} n={mm.sum():4d} r_max={100*rr[m][mm].max():4.1f} lat=[{100*lat.min():5.1f},{100*lat.max():5.1f}] near_face={100*ap.min():5.1f} az=[{az[mm].min():4.0f},{az[mm].max():4.0f}]")
