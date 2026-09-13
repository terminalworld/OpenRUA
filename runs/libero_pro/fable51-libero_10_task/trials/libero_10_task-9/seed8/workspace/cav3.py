import numpy as np
d = np.load("robot0_eye_in_hand_cloud.npz"); pw = d["pw"]; dep=d["depth"]
X,Y,Z = pw[...,0], pw[...,1], pw[...,2]
ok = np.isfinite(dep)&(dep>0.05)
# front face of microwave: y in [0.25,0.29]
m = ok&(Y>0.25)&(Y<0.295)&(X>-0.25)&(X<0.25)&(Z>0.9)&(Z<1.15)
print("front face pts", m.sum(), "y median", np.median(Y[m]).round(4))
# interior: y>0.30
mi = ok&(Y>0.30)&(X>-0.2)&(X<0.2)&(Z>0.9)&(Z<1.15)
print("interior pts", mi.sum(), "y range", Y[mi].min().round(3), Y[mi].max().round(3))
xs,ys,zs = X[mi],Y[mi],Z[mi]
print("interior z range", zs.min().round(4), zs.max().round(4), " x range", xs.min().round(4), xs.max().round(4))
# ceiling: points with z> 1.03 -> distribution
for z in np.arange(1.03,1.12,0.005):
    mm=(zs>=z)&(zs<z+0.005)
    if mm.sum(): print(f"  z[{z:.3f}] n={mm.sum():5d} x[{xs[mm].min():.3f},{xs[mm].max():.3f}] y[{ys[mm].min():.3f},{ys[mm].max():.3f}]")
print("floor:")
for z in np.arange(0.93,0.97,0.005):
    mm=(zs>=z)&(zs<z+0.005)
    if mm.sum(): print(f"  z[{z:.3f}] n={mm.sum():5d} x[{xs[mm].min():.3f},{xs[mm].max():.3f}] y[{ys[mm].min():.3f},{ys[mm].max():.3f}]")
print("walls (x hist for interior, z in 0.96..1.04):")
mm=(zs>0.96)&(zs<1.04)
for x in np.arange(-0.2,0.2,0.01):
    m2=mm&(xs>=x)&(xs<x+0.01)
    if m2.sum(): print(f"  x[{x:.2f}] n={m2.sum():5d} y[{ys[m2].min():.3f},{ys[m2].max():.3f}]")
print("back wall y:", np.percentile(ys, [95,99,99.9]).round(3))
