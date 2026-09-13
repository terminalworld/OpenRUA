import numpy as np
P = np.load("robot0_eye_in_hand_xyz.npy")
Z = P[...,2]
# bottle: anything above table+5cm within x in [-0.3,0], y in [-0.1,0.15], excluding cabinet (x>0)
m = (Z > 0.95) & (P[...,0] > -0.3) & (P[...,0] < 0.0) & (P[...,1] > -0.1) & (P[...,1] < 0.15)
pts = P[m]
print("bottle pts", pts.shape)
for lo,hi in [(0.95,1.0),(1.0,1.03),(1.03,1.06),(1.06,1.1),(1.1,1.2)]:
    s = pts[(pts[:,2]>=lo)&(pts[:,2]<hi)]
    if len(s): print(f"z[{lo},{hi}) n={len(s)} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}] cx={s[:,0].mean():.3f} cy={s[:,1].mean():.3f}")
# table height under eye-in-hand
t = P[(Z>0.85)&(Z<0.95)&(P[...,0]<0.0)]
print("table z", np.median(t[:,2]))
# cabinet/drawer region: x>0
c = P[(P[...,0]>0.0)&(Z>0.905)]
print("cabinet pts", c.shape)
for lo,hi in [(0.905,0.95),(0.95,1.0),(1.0,1.05),(1.05,1.1),(1.1,1.2),(1.2,1.4)]:
    s = c[(c[:,2]>=lo)&(c[:,2]<hi)]
    if len(s): print(f"z[{lo},{hi}) n={len(s)} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]")
