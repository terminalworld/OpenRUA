import numpy as np, sys
from handcheck import *
mp=np.load('mugpose.npy'); c=mp[0:3]; a=mp[3:6]; perp=np.array([-a[1],a[0],0]); Z=np.array([0,0,1.0])
ELEV=float(sys.argv[1]) if len(sys.argv)>1 else 26.0
e=np.radians(ELEV); rho=np.cos(e)*perp+np.sin(e)*Z; tau=-np.sin(e)*perp+np.cos(e)*Z; mug=(c,a,perp,rho)
def boxdist(x,lo,hi):
    ce=(lo+hi)/2; hf=(hi-lo)/2; d=np.maximum(0,np.abs(x-ce)-hf); return np.linalg.norm(d,axis=1)
def handle_clear(Pw,lab):
    d=Pw-c; t=d@a; q=d@rho; s=d@tau; X=np.stack([t,q,s],1)
    dist=np.minimum.reduce([boxdist(X,np.array([-0.038,0.03,-0.006]),np.array([-0.028,0.086,0.006])),
                            boxdist(X,np.array([0.015,0.03,-0.006]),np.array([0.025,0.086,0.006])),
                            boxdist(X,np.array([-0.04,0.074,-0.006]),np.array([0.027,0.086,0.006]))])
    return {l:dist[lab==l].min() for l in set(lab)}
res=[]
P,lab=hand_points(0.04)
for tc in (0.026,0.028,0.030):
  for RP in np.arange(0.048,0.072,0.002):
    for soff in np.arange(-0.012,0.0121,0.004):
      for phi in range(-90,91,5):   # z_h = -(cos(phi) s_hat + sin(phi) q_hat)?? param: direction from pad toward tips
        ph=np.radians(phi); zh=-np.cos(ph)*tau-np.sin(ph)*rho   # phi=0: fingers point -tau (from +tau side); phi=90: fingers point -rho (toward axis)
        if zh[2]>-0.1: continue  # keep fingers pointing somewhat downward
        Rm=pose_from(zh,a); pad=c+tc*a+RP*rho+soff*tau; H=pad-0.093*zh
        Pw=(Rm@P.T).T+H
        dep,what=penetration(Pw,*mug)
        if dep.max()>0: continue
        cl=clearance(Pw,lab,c,a,perp); hc=handle_clear(Pw,lab)
        # the pad must overlap the arm face: pad centre q in 0.048..0.070 approx (pad 2cm long along zh)
        m=min(min(cl.values()),min(hc.values()))
        res.append((round(m,4),tc,round(RP,3),round(soff,3),phi,{k:round(v,3) for k,v in cl.items()},{k:round(v,3) for k,v in hc.items()},np.round(H,3),np.round(zh,2)))
res.sort(key=lambda r:-r[0])
for r in res[:15]: print(r)
