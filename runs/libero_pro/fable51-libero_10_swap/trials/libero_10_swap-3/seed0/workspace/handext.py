import numpy as np
for cam in ["agentview","frontview","birdview"]:
    d = np.load(f"{cam}_cloud.npz"); pw = d["pw"]; z = pw[...,2]; x = pw[...,0]; y = pw[...,1]; ok=np.isfinite(z)
    for lo,hi in [(1.075,1.10),(1.10,1.13),(1.13,1.16),(1.16,1.20),(1.20,1.25)]:
        m = ok&(z>=lo)&(z<hi)&(x>-0.35)&(x<0.0)&(y>-0.1)&(y<0.2)
        if m.sum()>5: print(f"{cam} z {lo}-{hi}: n={m.sum()} x[{x[m].min():.3f},{x[m].max():.3f}] y[{y[m].min():.3f},{y[m].max():.3f}]")
