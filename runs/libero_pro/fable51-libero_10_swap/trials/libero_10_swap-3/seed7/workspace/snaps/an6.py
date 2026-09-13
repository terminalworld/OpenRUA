import numpy as np
P=np.load("snaps/agentview_world.npy"); X,Y,Z=P[...,0],P[...,1],P[...,2]
# tall stuff not cabinet body
m=(Z>1.05)&(Y>-0.22)&(Y<0.3)&(X>-0.4)&(X<0.4)&np.isfinite(Z)
print("tall pts", m.sum(), "x",X[m].min().round(3),X[m].max().round(3),"y",Y[m].min().round(3),Y[m].max().round(3),"z",Z[m].max().round(3))
for xlo in np.arange(-0.4,0.4,0.05):
    mm=m&(X>=xlo)&(X<xlo+0.05)
    if mm.sum()>5: print(f"  x{xlo:+.2f}: n={mm.sum():5d} y[{Y[mm].min():.3f},{Y[mm].max():.3f}] z[{Z[mm].min():.3f},{Z[mm].max():.3f}]")
# drawer front panel: y in [-0.13,-0.06], z 0.92..1.05, by x
m=(Y>-0.13)&(Y<-0.06)&(Z>0.905)&(Z<1.05)
print("front panel region")
for xlo in np.arange(-0.3,0.1,0.02):
    mm=m&(X>=xlo)&(X<xlo+0.02)
    if mm.sum()>5: print(f"  x{xlo:+.2f}: n={mm.sum():5d} y[{Y[mm].min():.3f},{Y[mm].max():.3f}] z[{Z[mm].min():.3f},{Z[mm].max():.3f}]")
