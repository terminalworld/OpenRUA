import numpy as np, rclpy
from rob import *
from tf2_ros import Buffer, TransformListener
r=Rob("handgeom"); buf=Buffer(); TransformListener(buf,r.node)
def cloud(cam):
    fr=f"{cam}_optical_frame"
    while not buf.can_transform("world",fr,rclpy.time.Time()): r.spin(0.2)
    t=buf.lookup_transform("world",fr,rclpy.time.Time()); tr=t.transform.translation; ro=t.transform.rotation
    P=r.depth_world(cam,[tr.x,tr.y,tr.z],[ro.x,ro.y,ro.z,ro.w]).reshape(-1,3); return P[np.isfinite(P).all(1)]
p,R=r.fk(); pt=p+TCP*R[:,2]
print("hand origin",p.round(3),"tcp",pt.round(3)); print("R\n",R.round(2))
for cam in ("sideview","agentview","birdview"):
    P=cloud(cam)
    L=(P-pt)@R   # coords in hand frame relative to TCP: columns hx,hy,hz
    m=(np.abs(L[:,0])<0.12)&(np.abs(L[:,1])<0.15)&(L[:,2]>-0.25)&(L[:,2]<0.03)&(P[:,2]>1.0)
    Q=L[m]; print(cam,"n",m.sum())
    for z0 in np.arange(-0.25,0.03,0.01):
        mm=(Q[:,2]>=z0)&(Q[:,2]<z0+0.01)
        if mm.sum()>3: print(f"  hz[{z0:+.2f},{z0+0.01:+.2f}] n={mm.sum():4d} hx[{Q[mm,0].min():+.3f},{Q[mm,0].max():+.3f}] hy[{Q[mm,1].min():+.3f},{Q[mm,1].max():+.3f}]")
