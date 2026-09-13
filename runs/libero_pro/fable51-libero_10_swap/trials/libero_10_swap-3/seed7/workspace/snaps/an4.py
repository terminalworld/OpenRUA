import numpy as np
for cam in ["frontview","agentview"]:
    P=np.load(f"snaps/{cam}_world.npy"); X,Y,Z=P[...,0],P[...,1],P[...,2]
    m=(np.hypot(X+0.143,Y-0.058)<0.09)&(Z>0.905)&(Z<1.1)
    print(cam)
    for zlo in np.arange(0.90,1.03,0.01):
        mm=m&(Z>=zlo)&(Z<zlo+0.01)
        if mm.sum(): print(f"  z{zlo:.2f}: n={mm.sum():4d} x[{X[mm].min():.3f},{X[mm].max():.3f}] y[{Y[mm].min():.3f},{Y[mm].max():.3f}] r={np.hypot(X[mm]+0.143,Y[mm]-0.058).mean():.3f}")
