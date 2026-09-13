import numpy as np
P = np.load("snaps/peek_xyz.npy"); X,Y,Z = P[...,0],P[...,1],P[...,2]
fl = (np.abs(Z-0.925)<0.006)&(X>-0.06)&(X<0.06)
print("floor (|x|<6cm): y", Y[fl].min(), Y[fl].max())
fl2 = (np.abs(Z-0.925)<0.006)&(Y>0.12)&(Y<0.20)
print("floor (y .12-.20): x", X[fl2].min(), X[fl2].max())
# back wall: points y>0.2 with z 0.93-0.98
bw = (Z>0.935)&(Z<0.975)&(Y>0.18)&(Y<0.30)&(X>-0.06)&(X<0.06)
print("back wall pts", bw.sum(), "y range", np.percentile(Y[bw],[5,50,95]))
# wall tops
for lab,m in [("front rim",(Z>0.975)&(Z<0.995)&(Y<0.12)&(X>-0.06)&(X<0.06)),("back rim",(Z>0.975)&(Z<1.0)&(Y>0.18)&(Y<0.3)&(X>-0.06)&(X<0.06)),("left rim",(Z>0.975)&(Z<0.995)&(X<-0.08)&(Y>0.1)&(Y<0.22)),("right rim",(Z>0.975)&(Z<0.995)&(X>0.08)&(Y>0.1)&(Y<0.22))]:
    if m.any(): print(lab, m.sum(), "x", np.round(np.percentile(X[m],[2,98]),3), "y", np.round(np.percentile(Y[m],[2,98]),3), "z", np.round(np.percentile(Z[m],[2,98]),3))
# cabinet opening: points above back rim at y~0.22-0.24, z 0.99-1.06
op = (Y>0.21)&(Y<0.26)&(Z>0.99)&(Z<1.08)&(X>-0.06)&(X<0.06)
print("cabinet face above opening: n",op.sum(), "z min", Z[op].min() if op.any() else None, "y", np.round(np.percentile(Y[op],[5,50,95]),3) if op.any() else None)
for zlo in np.arange(0.98,1.06,0.01):
    m=(Y>0.21)&(Y<0.26)&(Z>=zlo)&(Z<zlo+0.01)&(X>-0.06)&(X<0.06)
    if m.any(): print(f"  z {zlo:.2f}: n={m.sum()} y med {np.median(Y[m]):.3f}")
