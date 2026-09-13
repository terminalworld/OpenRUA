import numpy as np, sys
from handcheck import *
mp=np.load('mugpose.npy'); c=mp[0:3]; a=mp[3:6]; perp=np.array([-a[1],a[0],0]); Z=np.array([0,0,1.0])
def boxdist(x,lo,hi):
    ce=(lo+hi)/2; hf=(hi-lo)/2; d=np.maximum(0,np.abs(x-ce)-hf); return np.linalg.norm(d,axis=1)
def run(ELEV,ARMS=(0.020,),verbose=False):
    e=np.radians(ELEV); rho=np.cos(e)*perp+np.sin(e)*Z; tau=-np.sin(e)*perp+np.cos(e)*Z; mug=(c,a,perp,rho)
    def handle_clear(Pw,lab):
        d=Pw-c; t=d@a; q=d@rho; s=d@tau; X=np.stack([t,q,s],1)
        dist=np.minimum.reduce([boxdist(X,np.array([-0.038,0.03,-0.006]),np.array([-0.028,0.086,0.006])),
                                boxdist(X,np.array([0.015,0.03,-0.006]),np.array([0.025,0.086,0.006])),
                                boxdist(X,np.array([-0.04,0.074,-0.006]),np.array([0.027,0.086,0.006]))])
        return {l:dist[lab==l].min() for l in set(lab)}
    res=[]
    for arm in ARMS:
      for dtc in (0.006,0.008,0.010,0.012):
        tc=arm+dtc if arm>0 else arm-dtc
        for RP in (0.050,0.052,0.054,0.056,0.058,0.060):
          for tilt in range(-40,50,5):
            th=np.radians(tilt); zh=-np.cos(th)*Z-np.sin(th)*perp
            Rm=pose_from(zh,a); pad=c+tc*a+RP*rho; H=pad-0.093*zh
            P,lab=hand_points(0.04); Pw=(Rm@P.T).T+H
            dep,what=penetration(Pw,*mug)
            if dep.max()>0: continue
            cl=clearance(Pw,lab,c,a,perp); hc=handle_clear(Pw,lab)
            m=min(min(cl.values()),min(hc.values()))
            res.append((round(m,4),arm,round(tc,3),RP,tilt,{k:round(v,3) for k,v in cl.items()},{k:round(v,3) for k,v in hc.items()},np.round(H,3)))
    res.sort(key=lambda r:-r[0])
    return res
for ELEV in (20,26,30,40,50,60,70,80,90):
    res=run(ELEV,ARMS=(0.020,-0.033))
    print(ELEV, res[0] if res else None)
