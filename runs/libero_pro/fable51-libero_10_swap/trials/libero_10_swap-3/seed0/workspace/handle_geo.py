import numpy as np
for cam in ["agentview","frontview","birdview","sideview"]:
    d = np.load(f"{cam}_cloud.npz"); pw = d["pw"]; z = pw[...,2]; x = pw[...,0]; y = pw[...,1]; ok=np.isfinite(z)
    m = ok&(y>-0.219)&(y<-0.12)&(z>1.0)&(z<1.135)&(x>-0.22)&(x<0.02)
    print("==", cam, "pts in front of cabinet face, z>1.0:", m.sum())
    for lo in np.arange(1.0,1.135,0.01):
        mm = m&(z>=lo)&(z<lo+0.01)
        if mm.sum()>3: print(f"   z {lo:.3f}: n={mm.sum()} x[{x[mm].min():.3f},{x[mm].max():.3f}] y[{y[mm].min():.3f},{y[mm].max():.3f}]")
    # cabinet face y position: points with z in [1.03,1.12], x in [-0.2,0], y>-0.3
    m = ok&(z>1.03)&(z<1.12)&(x>-0.2)&(x<0.0)&(y>-0.3)&(y<-0.15)
    if m.sum(): print("   face-ish pts y percentiles 5/50/95:", np.percentile(y[m],[5,50,95]).round(4))
