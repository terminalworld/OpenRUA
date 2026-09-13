import numpy as np
P=np.load("sideview_xyz.npy"); X,Y,Z=P[...,0],P[...,1],P[...,2]
# target pot region
m=(X>0.0)&(X<0.14)&(Y>-0.05)&(Y<0.13)&(Z>0.905)&(Z<1.07)
for zlo in np.arange(0.905,1.06,0.01):
    mm=m&(Z>=zlo)&(Z<zlo+0.01)
    if mm.sum(): print(f"z {zlo:.3f}: n={mm.sum():3d} X[{X[mm].min():.3f},{X[mm].max():.3f}] w={X[mm].max()-X[mm].min():.3f}  Y[{Y[mm].min():.3f},{Y[mm].max():.3f}]")
