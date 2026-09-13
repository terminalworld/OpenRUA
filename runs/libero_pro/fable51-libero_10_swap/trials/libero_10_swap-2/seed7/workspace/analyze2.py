import numpy as np
P = np.load("/workspace/P_birdview.npy"); Z = P[...,2]
reg = (P[...,1] < -0.1) & (P[...,1] > -0.4) & (P[...,0] > -0.2) & (P[...,0] < 0.1)
for zt in [0.92, 0.95, 0.98, 1.0, 1.02, 1.04, 1.05]:
    m = reg & (Z > zt)
    if m.sum()==0: continue
    print(f"z>{zt}: n={m.sum()} x[{P[m][:,0].min():.3f},{P[m][:,0].max():.3f}] y[{P[m][:,1].min():.3f},{P[m][:,1].max():.3f}] cx={P[m][:,0].mean():.3f} cy={P[m][:,1].mean():.3f}")
# row/col profile at top: for each y-bin, print max z
m = reg & (Z > 0.91)
ys = P[m][:,1]; zs = Z[m]; xs = P[m][:,0]
for yb in np.arange(-0.33, -0.17, 0.01):
    s = (ys >= yb) & (ys < yb+0.01)
    if s.sum(): print(f"y={yb:.2f}: zmax={zs[s].max():.3f} xrange[{xs[s].min():.3f},{xs[s].max():.3f}] n={s.sum()}")
