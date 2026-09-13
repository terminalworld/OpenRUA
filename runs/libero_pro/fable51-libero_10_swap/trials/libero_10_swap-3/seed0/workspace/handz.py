import numpy as np
TCP=np.array([-0.1595,-0.0504,1.08])
for cam in ["frontview","sideview"]:
    d=np.load(f"{cam}_cloud.npz"); pw=d["pw"]; x,y,z=pw[...,0],pw[...,1],pw[...,2]; ok=np.isfinite(z)
    # column around the hand/bowl: x in [-0.30,0.0], y in [-0.12,0.10], z>0.95
    m=ok&(x>-0.30)&(x<0.0)&(y>-0.12)&(y<0.12)&(z>0.95)&(z<1.35)
    print(cam, m.sum())
    for lo in np.arange(0.95,1.35,0.01):
        mm=m&(z>=lo)&(z<lo+0.01)
        if mm.sum()>3: print(f"  z {lo:.2f} ({lo-1.08:+.3f}): n={mm.sum():4d} x[{x[mm].min():.3f},{x[mm].max():.3f}] y[{y[mm].min():.3f},{y[mm].max():.3f}]")
