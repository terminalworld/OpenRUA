import numpy as np
P=np.load("robot0_eye_in_hand_xyz.npy"); X,Y,Z=P[...,0],P[...,1],P[...,2]
ok=np.isfinite(Z)&(Z>0.5)
h=ok&(X>0.03)&(X<0.12)&(Y>-0.04)&(Y<0.02)&(Z>0.95)&(Z<1.05)
for ylo in np.arange(-0.03,0.02,0.005):
    mm=h&(Y>=ylo)&(Y<ylo+0.005)
    if mm.sum(): print(f"Y {ylo:+.3f}: n={mm.sum():3d} X[{X[mm].min():.4f},{X[mm].max():.4f}] c={(X[mm].min()+X[mm].max())/2:.4f} Z[{Z[mm].min():.4f},{Z[mm].max():.4f}]")
# finger pads: Z near TCP 1.10..1.12
f=ok&(Z>1.098)&(Z<1.125)&(Y>-0.05)&(Y<0.05)
for side,mm in [("left(-X)",f&(X<0.07)),("right(+X)",f&(X>0.07))]:
    if mm.sum(): print(f"finger {side}: X[{X[mm].min():.4f},{X[mm].max():.4f}] Y[{Y[mm].min():.4f},{Y[mm].max():.4f}] Z[{Z[mm].min():.4f},{Z[mm].max():.4f}]")
