import numpy as np
from rob import *
from goto import ik_checked, margin
from coll import fk_all, CAPS
r = Robot("s4")
q0 = r.arm_q()
BOT = np.array([-0.166, 0.073]); ZG = 1.045
def dist_boards(p, rad):
    # boards AABB: x[-0.26,-0.03] y[-0.36,-0.19] z[0.90,1.22]
    lo=np.array([-0.26,-0.36,0.90]); hi=np.array([-0.03,-0.19,1.22])
    d=np.linalg.norm(np.maximum(0,np.maximum(lo-p,p-hi))); return d-rad
def dist_bowl(p, rad):
    if p[2]-rad > 1.03: return 9
    return np.hypot(p[0]+0.01,p[1]+0.075)-0.085-rad
def dist_cab(p, rad):
    lo=np.array([-0.18,0.22,0.90]); hi=np.array([0.14,0.42,1.13])
    return np.linalg.norm(np.maximum(0,np.maximum(lo-p,p-hi)))-rad
def check(q):
    P=fk_all(r,q); worst=(9,None)
    for a,b,rad in CAPS:
        for t in np.linspace(0,1,6):
            p=P[a]+t*(P[b]-P[a])
            for nm,f in (("boards",dist_boards),("bowl",dist_bowl),("cab",dist_cab)):
                d=f(p,rad)
                if d<worst[0]: worst=(d,(nm,a,np.round(p,3)))
            d=p[2]-rad-0.902
            if d<worst[0]: worst=(d,("table",a,np.round(p,3)))
    return worst
print("current", check(q0))
res=[]
for az_deg in [75,90,105,120]:
    for tilt_deg in [15,25,35]:
        az=np.radians(az_deg); tilt=np.radians(tilt_deg)
        d=np.array([np.cos(az),np.sin(az),0.0])
        Z=np.array([np.cos(tilt)*d[0],np.cos(tilt)*d[1],-np.sin(tilt)]); Y=np.array([-d[1],d[0],0.0]); X=np.cross(Y,Z)
        R=np.column_stack([X,Y,Z]); g=np.array([BOT[0],BOT[1],ZG]); pre=g-0.08*Z
        qg=ik_checked(r,g,R,q0,max_dist=3.0)
        if qg is None: print(az_deg,tilt_deg,"no IK grasp"); continue
        qp=ik_checked(r,pre,R,qg,max_dist=0.6)
        if qp is None: print(az_deg,tilt_deg,"no IK pre"); continue
        cg=check(qg); cp=check(qp)
        print(f"az={az_deg} tilt={tilt_deg} margin={margin(qg):.2f}/{margin(qp):.2f} dq0={np.abs(qg-q0).max():.2f}\n   grasp {cg}\n   pre   {cp}")
        res.append((az_deg,tilt_deg,qg,qp,R,cg[0],cp[0]))
np.save("snaps/search4.npy", np.array(res,dtype=object), allow_pickle=True)
