from rob import *
from tf2_ros import Buffer, TransformListener
import time
def bird_points(r):
    tfbuf=Buffer(); TransformListener(tfbuf, r.node)
    t0=time.time()
    while time.time()-t0<10 and not tfbuf.can_transform('world','birdview_optical_frame',rclpy.time.Time()): r.spin(0.2)
    t=tfbuf.lookup_transform('world','birdview_optical_frame',rclpy.time.Time())
    q=t.transform.rotation; R=quat_to_R(q.x,q.y,q.z,q.w); T=np.array([t.transform.translation.x,t.transform.translation.y,t.transform.translation.z])
    d=r.depth('birdview'); K=r.K('birdview'); fx,fy,cx,cy=K[0,0],K[1,1],K[0,2],K[1,2]
    vs,us=np.mgrid[0:480,0:640]
    return (np.stack([(us-cx)*d/fx,(vs-cy)*d/fy,d],-1).reshape(-1,3)@R.T+T).reshape(480,640,3)
def pot_report(P,c,rad=0.06,zmin=1.02):
    m=(np.abs(P[...,0]-c[0])<rad)&(np.abs(P[...,1]-c[1])<rad)&(P[...,2]>zmin)
    if m.sum()==0: return None
    pts=P[m]; top=pts[pts[:,2]>pts[:,2].max()-0.03]
    return dict(n=int(m.sum()), zmax=round(float(pts[:,2].max()),3), top_xy=np.round(top[:,:2].mean(0),4))
if __name__=='__main__':
    import sys
    r=Robot(); P=bird_points(r)
    for arg in sys.argv[1:]:
        x,y=map(float,arg.split(',')); print(arg, pot_report(P,(x,y)))
