import numpy as np
for cam in ["agentview","frontview"]:
    P=np.load(f"snaps/{cam}_world.npy"); X,Y,Z=P[...,0],P[...,1],P[...,2]
    m=(X>-0.05)&(X<0.2)&(Y>-0.2)&(Y<0.0)&(Z>0.905)&np.isfinite(Z)
    print(cam, m.sum())
    for zlo in np.arange(0.9,1.3,0.05):
        mm=m&(Z>=zlo)&(Z<zlo+0.05)
        if mm.sum()>5: print(f"  z{zlo:.2f}: n={mm.sum():5d} x[{X[mm].min():.3f},{X[mm].max():.3f}] y[{Y[mm].min():.3f},{Y[mm].max():.3f}] cx={X[mm].mean():.3f} cy={Y[mm].mean():.3f}")
