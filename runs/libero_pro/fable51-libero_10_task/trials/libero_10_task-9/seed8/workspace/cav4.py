import numpy as np
d = np.load("robot0_eye_in_hand_cloud.npz"); pw = d["pw"]; dep=d["depth"]
X,Y,Z = pw[...,0], pw[...,1], pw[...,2]
ok = np.isfinite(dep)&(dep>0.05)
m = ok&(Y>0.25)&(Y<0.299)&(X>-0.14)&(X<0.05)&(Z>0.9)&(Z<1.15)
xs,ys,zs=X[m],Y[m],Z[m]
print("front-plane pts within opening x-range:", m.sum())
for z in np.arange(0.90,1.12,0.01):
    mm=(zs>=z)&(zs<z+0.01)
    if mm.sum(): print(f"  z[{z:.2f}] n={mm.sum():5d} y[{ys[mm].min():.3f},{ys[mm].max():.3f}] x[{xs[mm].min():.3f},{xs[mm].max():.3f}]")
# x-extent of the opening at mid height: front plane points at z in [0.98,1.04]
m2 = ok&(Y>0.25)&(Y<0.299)&(Z>0.98)&(Z<1.04)&(X>-0.3)&(X<0.3)
print("front plane at mid-height, x hist:")
xs2=X[m2]
for x in np.arange(-0.3,0.3,0.02):
    mm=(xs2>=x)&(xs2<x+0.02)
    if mm.sum(): print(f"  x[{x:.2f}] n={mm.sum()}")
# also depth along the y direction between 0.267 and 0.30 : any points at all with x in cavity range?
m3 = ok&(Y>0.27)&(Y<0.30)&(X>-0.14)&(X<0.05)&(Z>0.9)&(Z<1.15)
print("pts between front plane and interior:", m3.sum(), "z", Z[m3].min() if m3.sum() else None, Z[m3].max() if m3.sum() else None)
