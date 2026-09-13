import numpy as np
d = np.load("agentview_cloud.npz"); pw = d["pw"]; col=d["color"]
X,Y,Z = pw[...,0], pw[...,1], pw[...,2]
# microwave body front region: x in [-0.17,0.17], y in [0.25,0.5]
m = (X>-0.18)&(X<0.18)&(Y>0.2)&(Y<0.6)&(Z>0.89)&(Z<1.15)
print("pts", m.sum())
xs,ys,zs = X[m],Y[m],Z[m]
print("y bins:")
for y in np.arange(0.2,0.6,0.02):
    mm=(ys>=y)&(ys<y+0.02)
    if mm.sum(): print(f"  y[{y:.2f}] n={mm.sum():5d} x[{xs[mm].min():.3f},{xs[mm].max():.3f}] z[{zs[mm].min():.3f},{zs[mm].max():.3f}]")
print("z bins (y>0.3):")
for z in np.arange(0.89,1.15,0.01):
    mm=(zs>=z)&(zs<z+0.01)&(ys>0.3)
    if mm.sum(): print(f"  z[{z:.2f}] n={mm.sum():5d} x[{xs[mm].min():.3f},{xs[mm].max():.3f}] y[{ys[mm].min():.3f},{ys[mm].max():.3f}]")
