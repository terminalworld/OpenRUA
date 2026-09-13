import numpy as np, sys
cam = sys.argv[1] if len(sys.argv)>1 else "agentview"
P = np.load(f"{cam}_xyz.npy")
x,y,z = P[...,0].ravel(), P[...,1].ravel(), P[...,2].ravel()
m = np.isfinite(z)&(x>-0.12)&(x<0.12)&(y>-0.02)&(y<0.26)&(z>0.912)&(z<1.05)
x,y,z = x[m],y[m],z[m]
for ys in np.arange(-0.02,0.26,0.01):
    s=(y>=ys)&(y<ys+0.01)
    if s.sum()>3: print(f"  y[{ys:+.2f}] n={s.sum():4d} x {x[s].min():+.3f}..{x[s].max():+.3f} ztop {z[s].max():.3f} zmed {np.median(z[s]):.3f}")
