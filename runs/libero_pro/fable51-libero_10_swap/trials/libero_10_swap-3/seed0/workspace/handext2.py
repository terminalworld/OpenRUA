import numpy as np
for cam in ["agentview","frontview","sideview","birdview"]:
    d = np.load(f"{cam}_cloud.npz"); pw = d["pw"]; z = pw[...,2]; x = pw[...,0]; y = pw[...,1]; ok=np.isfinite(z)
    print("==", cam, "(TCP -0.1595,-0.0504,1.08; hand frame z=1.183)")
    for lo in np.arange(1.10,1.30,0.01):
        m = ok&(z>=lo)&(z<lo+0.01)&(x>-0.30)&(x<-0.03)&(y>-0.18)&(y<0.15)
        if m.sum()>5: print(f"   z {lo:.2f}: n={m.sum():4d} x[{x[m].min():.3f},{x[m].max():.3f}] y[{y[m].min():.3f},{y[m].max():.3f}]  (y rel TCP: {y[m].min()+0.0504:+.3f} .. {y[m].max()+0.0504:+.3f})")
