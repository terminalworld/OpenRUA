import numpy as np
for cam in ["agentview","birdview","frontview"]:
    P=np.load(f"snaps/{cam}_world.npy"); X,Y,Z=P[...,0],P[...,1],P[...,2]
    m=(Y<-0.05)&(Y>-0.5)&(X>-0.4)&(X<0.15)&(Z>0.905)&(Z<1.3)&np.isfinite(Z)
    print(cam)
    for ylo in np.arange(-0.45,-0.05,0.02):
        mm=m&(Y>=ylo)&(Y<ylo+0.02)
        if mm.sum()>5: print(f"  y{ylo:+.2f}: n={mm.sum():5d} x[{X[mm].min():.3f},{X[mm].max():.3f}] z[{Z[mm].min():.3f},{Z[mm].max():.3f}]")
