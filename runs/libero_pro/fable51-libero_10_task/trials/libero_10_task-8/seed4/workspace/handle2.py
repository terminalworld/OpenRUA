import numpy as np
P=np.load("robot0_eye_in_hand_xyz.npy"); X,Y,Z=P[...,0],P[...,1],P[...,2]
ok=np.isfinite(Z)
m=ok&(X>0.0)&(X<0.15)&(Y>-0.06)&(Y<0.15)&(Z>0.905)&(Z<1.07)
print("pot pts",m.sum())
# lid extents
lid=m&(Z>1.03)&(Y>0.02)
print(f"lid X[{X[lid].min():.4f},{X[lid].max():.4f}] Y[{Y[lid].min():.4f},{Y[lid].max():.4f}] zmax {Z[lid].max():.4f}")
knob=m&(Z>1.046)
print(f"knob X[{X[knob].min():.4f},{X[knob].max():.4f}] Y[{Y[knob].min():.4f},{Y[knob].max():.4f}] c=({X[knob].mean():.4f},{Y[knob].mean():.4f})")
h=m&(Y<0.018)
print("handle region pts",h.sum())
for ylo in np.arange(-0.035,0.02,0.005):
    mm=h&(Y>=ylo)&(Y<ylo+0.005)
    if mm.sum(): print(f"Y {ylo:.3f}: n={mm.sum():3d} X[{X[mm].min():.4f},{X[mm].max():.4f}] w={X[mm].max()-X[mm].min():.4f} Z[{Z[mm].min():.4f},{Z[mm].max():.4f}]")
