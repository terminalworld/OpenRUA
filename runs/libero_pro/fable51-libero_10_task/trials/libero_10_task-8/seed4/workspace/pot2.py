import numpy as np
P=np.load("bird_xyz.npy"); X,Y,Z=P[...,0],P[...,1],P[...,2]
box=(X>0.0)&(X<0.15)&(Y>-0.05)&(Y<0.15)
for zlo,zhi in [(0.905,0.95),(0.95,1.0),(1.0,1.03),(1.03,1.045),(1.045,1.1)]:
    mm=box&(Z>zlo)&(Z<=zhi)
    if mm.sum(): print(f"Z in ({zlo},{zhi}]: n={mm.sum()} X[{X[mm].min():.3f},{X[mm].max():.3f}] Y[{Y[mm].min():.3f},{Y[mm].max():.3f}]")
# top lid excluding handle/spout: restrict Y to central body
top=box&(Z>1.03)
# per-row (X) Y extents
vs=np.unique(np.where(top)[0])
for v in vs:
    m=top[v]
    ys=Y[v][m]; print(f"X={X[v][m].mean():.3f} Y[{ys.min():.3f},{ys.max():.3f}] w={ys.max()-ys.min():.3f} zmax={Z[v][m].max():.3f}")
