import numpy as np
P=np.load("bird_xyz.npy"); X,Y,Z=P[...,0],P[...,1],P[...,2]
m=(Z>0.905)&(X>0.0)&(X<0.15)&(Y>-0.05)&(Y<0.15)
for zlo in [0.91,0.95,1.0,1.03,1.05]:
    mm=m&(Z>zlo)
    if mm.sum(): print(f"Z>{zlo}: n={mm.sum()} X[{X[mm].min():.3f},{X[mm].max():.3f}] Y[{Y[mm].min():.3f},{Y[mm].max():.3f}] c=({X[mm].mean():.3f},{Y[mm].mean():.3f})")
# ascii of heights
vs,us=np.where(m)
for v in range(vs.min(),vs.max()+1):
    row=""
    for u in range(us.min(),us.max()+1):
        z=Z[v,u]
        row+= "." if z<0.905 else str(min(9,int((z-0.9)*60)))
    print(f"{v:3d} {row}")
print("u range",us.min(),us.max())
