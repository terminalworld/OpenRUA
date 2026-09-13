import numpy as np
P=np.load("frontview_xyz.npy"); X,Y,Z=P[...,0],P[...,1],P[...,2]
m=(X>0.0)&(X<0.15)&(Y>-0.06)&(Y<0.03)&(Z>0.905)&(Z<1.07)
print("handle pts",m.sum())
for zlo in np.arange(0.905,1.06,0.005):
    mm=m&(Z>=zlo)&(Z<zlo+0.005)
    if mm.sum(): print(f"z {zlo:.3f}: n={mm.sum():3d} X[{X[mm].min():.3f},{X[mm].max():.3f}] Y[{Y[mm].min():.3f},{Y[mm].max():.3f}]")
# whole pot from frontview: Y extents by z (body)
print("--- body Y extents by z (frontview)")
m=(X>0.0)&(X<0.15)&(Y>-0.06)&(Y<0.15)&(Z>0.905)&(Z<1.07)
for zlo in np.arange(0.905,1.06,0.01):
    mm=m&(Z>=zlo)&(Z<zlo+0.01)
    if mm.sum(): print(f"z {zlo:.3f}: n={mm.sum():3d} Y[{Y[mm].min():.3f},{Y[mm].max():.3f}] w={Y[mm].max()-Y[mm].min():.3f} Xmin={X[mm].min():.3f}")
