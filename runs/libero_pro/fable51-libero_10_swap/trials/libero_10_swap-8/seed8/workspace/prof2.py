import numpy as np, rclpy
from rob import *
from tf2_ros import Buffer, TransformListener
r=Rob("prof2"); buf=Buffer(); TransformListener(buf,r.node)
B=np.array([-0.064,0.234])
for cam in ("sideview","frontview","agentview"):
    fr=f"{cam}_optical_frame"
    for _ in range(50):
        r.spin(0.1)
        if buf.can_transform("world",fr,rclpy.time.Time()): break
    t=buf.lookup_transform("world",fr,rclpy.time.Time()).transform
    tt=[t.translation.x,t.translation.y,t.translation.z]; q=[t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w]
    print(cam,"cam pos",np.round(tt,3))
    P=r.depth_world(cam,tt,q).reshape(-1,3); P=P[np.isfinite(P).all(1)]
    d=P[:,:2]-B; m=(np.linalg.norm(d,axis=1)<0.08)&(P[:,2]>0.895)&(P[:,2]<1.07)
    Q=P[m]; dd=d[m]
    # view direction in xy from camera to pot; lateral axis perpendicular
    v=B-np.array(tt[:2]); v/=np.linalg.norm(v); lat=np.array([-v[1],v[0]])
    l=dd@lat; a=dd@v
    for z0 in np.arange(0.895,1.06,0.005):
        mm=(Q[:,2]>=z0)&(Q[:,2]<z0+0.005)
        if mm.sum()>3:
            print(f"  z={z0:.3f} width={100*(l[mm].max()-l[mm].min()):4.1f}cm  near_face_dist={100*a[mm].min():5.1f}  n={mm.sum()}")
