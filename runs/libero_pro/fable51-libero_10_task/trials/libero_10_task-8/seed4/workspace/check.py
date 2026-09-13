import numpy as np
P=np.load("robot0_eye_in_hand_xyz.npy"); X,Y,Z=P[...,0],P[...,1],P[...,2]
ok=np.isfinite(Z)&(Z>0.5)
lid=ok&(Z>1.028)&(Z<1.046)&(Y>0.02)&(Y<0.10)&(X>0.0)&(X<0.15)
print(f"lid rim X[{X[lid].min():.4f},{X[lid].max():.4f}] center {(X[lid].min()+X[lid].max())/2:.4f}  Y[{Y[lid].min():.4f},{Y[lid].max():.4f}]")
knob=ok&(Z>1.046)&(Z<1.07)&(X>0.0)&(X<0.15)
print(f"knob c=({X[knob].mean():.4f},{Y[knob].mean():.4f}) ztop={Z[knob].max():.4f}")
# fingers: points at Z between 1.10 and 1.20 (fingertips at TCP 1.12 up to hand)
f=ok&(Z>1.10)&(Z<1.19)
print("finger pts",f.sum())
for side,mm in [("left(-X)",f&(X<0.07)),("right(+X)",f&(X>0.07))]:
    if mm.sum(): print(f"{side}: X[{X[mm].min():.4f},{X[mm].max():.4f}] Y[{Y[mm].min():.4f},{Y[mm].max():.4f}] Z[{Z[mm].min():.4f},{Z[mm].max():.4f}]")
# body profile below lid, by Z
body=ok&(X>0.0)&(X<0.15)&(Y>0.02)&(Y<0.10)&(Z>0.95)&(Z<1.05)
for zlo in np.arange(0.95,1.05,0.01):
    mm=body&(Z>=zlo)&(Z<zlo+0.01)
    if mm.sum(): print(f"z {zlo:.2f}: n={mm.sum():4d} X[{X[mm].min():.4f},{X[mm].max():.4f}] w={X[mm].max()-X[mm].min():.4f}")
