import numpy as np
from rob import *
r=Rob("state2")
P=r.depth_world("birdview",*BIRD).reshape(-1,3); P=P[np.isfinite(P).all(1)]
np.save("bird_now.npy",P)
def blob(name,xr,yr,zmin):
    m=(P[:,0]>xr[0])&(P[:,0]<xr[1])&(P[:,1]>yr[0])&(P[:,1]<yr[1])&(P[:,2]>zmin)
    Q=P[m]; c=Q[:,:2].mean(0); U,S,Vt=np.linalg.svd(Q[:,:2]-c,full_matrices=False)
    ax=Vt[0]; proj=(Q[:,:2]-c)@ax; perp=(Q[:,:2]-c)@Vt[1]
    print(f"{name}: n={len(Q)} center={c.round(3)} axis={ax.round(3)} (az {np.degrees(np.arctan2(ax[1],ax[0])):.0f}) len={proj.max()-proj.min():.3f} width={perp.max()-perp.min():.3f} zmax={Q[:,2].max():.3f}")
    for s in np.arange(proj.min(),proj.max(),0.015):
        mm=(proj>=s)&(proj<s+0.015)
        if mm.sum(): print(f"   along {s:+.3f}: n={mm.sum():3d} zmax={Q[mm,2].max():.3f} width={perp[mm].max()-perp[mm].min():.3f} perp_c={perp[mm].mean():+.3f}")
blob("B",(-0.35,0.05),(0.1,0.45),0.91)
blob("A",(0.05,0.35),(-0.1,0.2),0.94)
for cam in ("agentview","sideview","frontview"): r.snap(cam,f"/workspace/now_{cam}.png")
