import numpy as np
for cam in ["agentview","frontview","sideview","birdview"]:
    d = np.load(f"{cam}_cloud.npz"); pw = d["pw"]; z = pw[...,2]; x = pw[...,0]; y = pw[...,1]; ok=np.isfinite(z)
    # bowl rim: near (-0.08,-0.156), z 0.98..1.05, r<0.09
    r = np.hypot(x+0.08, y+0.156)
    m = ok&(r<0.10)&(z>0.975)&(z<1.045)&(x>-0.2)
    if m.sum()>10:
        print(f"{cam}: bowl-ish pts n={m.sum()} x[{x[m].min():.3f},{x[m].max():.3f}] y[{y[m].min():.3f},{y[m].max():.3f}] z[{z[m].min():.3f},{z[m].max():.3f}]")
        for lo in np.arange(0.975,1.045,0.01):
            mm = m&(z>=lo)&(z<lo+0.01)
            if mm.sum()>3: print(f"    z {lo:.3f}: n={mm.sum()} x[{x[mm].min():.3f},{x[mm].max():.3f}] y[{y[mm].min():.3f},{y[mm].max():.3f}]")
