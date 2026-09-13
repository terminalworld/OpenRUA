import numpy as np
for cam in ["agentview","birdview","sideview"]:
    d = np.load(f"{cam}_cloud.npz"); pw = d["pw"]
    z = pw[...,2]; x = pw[...,0]; y = pw[...,1]; ok=np.isfinite(z)
    # bowl rim: points z in [0.945,0.97] near (-0.138,0.04)
    r = np.hypot(x+0.138, y-0.040)
    m = ok&(r<0.09)&(z>0.945)&(z<0.975)
    if m.sum(): print(cam, "rim n", m.sum(), "center", round(x[m].mean(),4), round(y[m].mean(),4), "x rng", round(x[m].min(),3), round(x[m].max(),3), "y rng", round(y[m].min(),3), round(y[m].max(),3), "zmax", round(z[m].max(),3))
    # drawer bottom (z 0.905-0.93) and drawer walls/front
    m = ok&(x>-0.28)&(x<0.05)&(y>-0.25)&(y<0.0)&(z>0.905)&(z<1.0)
    for lo,hi in [(0.905,0.93),(0.93,0.95),(0.95,0.97),(0.97,0.99),(0.99,1.0)]:
        mm = m&(z>=lo)&(z<hi)
        if mm.sum()>10: print(f"   {cam} drawer z {lo:.3f}-{hi:.3f}: n={mm.sum()} x[{x[mm].min():.3f},{x[mm].max():.3f}] y[{y[mm].min():.3f},{y[mm].max():.3f}]")
    # bottle
    m = ok&(x>-0.02)&(x<0.12)&(y>-0.15)&(y<0.0)&(z>0.93)&(z<1.2)
    if m.sum(): print("   bottle x", round(x[m].min(),3), round(x[m].max(),3), "y", round(y[m].min(),3), round(y[m].max(),3), "zmax", round(z[m].max(),3))
