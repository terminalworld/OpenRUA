import numpy as np
P=np.load("snaps/eih_P.npy"); x,y,z=P[...,0],P[...,1],P[...,2]
# find pixels with x close to -0.17 and print (y,z) profile sorted by y
m=(abs(x+0.17)<0.004)&np.isfinite(z)
ys,zs=y[m],z[m]
o=np.argsort(ys)
ys,zs=ys[o],zs[o]
# bin by y
for lo in np.arange(-0.60,-0.15,0.01):
    mm=(ys>=lo)&(ys<lo+0.01)
    if mm.sum(): print(f"y[{lo:.2f}] z min {zs[mm].min():.3f} max {zs[mm].max():.3f} n={mm.sum()}")
