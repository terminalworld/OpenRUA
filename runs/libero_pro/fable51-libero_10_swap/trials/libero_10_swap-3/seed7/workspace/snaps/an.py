import numpy as np
P=np.load("snaps/birdview_world.npy")
X,Y,Z=P[...,0],P[...,1],P[...,2]
# bowl region: around (-0.18,0.05); mask z>0.91 within a 0.15 radius
r=np.hypot(X+0.17,Y-0.05)
for name,m in [("bowl", (r<0.12)&(Z>0.905)&(Z<1.1))]:
    print(name, m.sum(), "x",X[m].min().round(3),X[m].max().round(3),"y",Y[m].min().round(3),Y[m].max().round(3),"z",Z[m].min().round(3),Z[m].max().round(3), "centroid", X[m].mean().round(4), Y[m].mean().round(4))
    # rim: highest points
    top=m&(Z>Z[m].max()-0.01)
    print(" rim centroid", X[top].mean().round(4), Y[top].mean().round(4), "n",top.sum())
# table
m=(Z>0.89)&(Z<0.905)&(np.abs(X)<0.6)&(np.abs(Y)<0.6)
print("table z", Z[m].mean().round(4), m.sum())
# cabinet region: y<-0.1, x in -0.35..0.15
m=(Y<-0.1)&(Y>-0.5)&(X>-0.4)&(X<0.2)&(Z>0.91)
print("cabinet pts", m.sum(), "x",X[m].min().round(3),X[m].max().round(3),"y",Y[m].min().round(3),Y[m].max().round(3),"z",Z[m].min().round(3),Z[m].max().round(3))
for zlo,zhi in [(0.91,0.95),(0.95,1.0),(1.0,1.05),(1.05,1.1),(1.1,1.15),(1.15,1.2)]:
    mm=m&(Z>=zlo)&(Z<zhi)
    if mm.sum(): print(f" z{zlo}-{zhi}: n={mm.sum()} x[{X[mm].min():.3f},{X[mm].max():.3f}] y[{Y[mm].min():.3f},{Y[mm].max():.3f}]")
