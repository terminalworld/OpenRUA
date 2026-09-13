import numpy as np
from scipy.optimize import least_squares
d=np.load("agentview_cloud.npz"); pc=d["pc"]; col=d["col"]; z=pc[...,2]
for zl,zh in [(0.46,0.50),(0.50,0.54),(0.54,0.57)]:
    m=np.isfinite(z)&(pc[...,0]>-0.3)&(pc[...,0]<0.0)&(np.abs(pc[...,1])<0.15)&(z>zl)&(z<zh)
    P=pc[m][:,:2]
    # exclude handle: keep y>-0.03 for fit
    Q=P[P[:,1]>-0.025]
    f=lambda c: np.hypot(Q[:,0]-c[0],Q[:,1]-c[1])-c[2]
    r=least_squares(f,[-0.2,0.01,0.045])
    print(f"z[{zl},{zh}] n={len(Q)} center=({r.x[0]:.3f},{r.x[1]:.3f}) radius={r.x[2]:.3f} rms={np.sqrt(np.mean(r.fun**2)):.4f}  handle-side y min {P[:,1].min():.3f}")
