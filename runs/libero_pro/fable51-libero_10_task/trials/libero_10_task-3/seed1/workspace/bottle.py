import numpy as np
for cam in ("agentview","birdview"):
    P = np.load(f"{cam}_xyz.npy")
    x,y,z = P[...,0].ravel(), P[...,1].ravel(), P[...,2].ravel()
    m = np.isfinite(z)&(x>-0.45)&(x<-0.15)&(y>-0.15)&(y<0.15)&(z>0.912)&(z<1.0)
    x,y,z = x[m],y[m],z[m]
    print("==",cam, len(z), "ztop", z.max().round(3))
    # PCA axis of top points
    pts = np.c_[x,y]
    c = pts.mean(0); u,s,vt = np.linalg.svd(pts-c, full_matrices=False)
    ax = vt[0]; 
    if ax[0]<0: ax=-ax
    t = (pts-c)@ax; w = (pts-c)@vt[1]
    print("center", c.round(3), "axis", ax.round(3), "angle deg", np.degrees(np.arctan2(ax[1],ax[0])).round(1), "len", (t.max()-t.min()).round(3), "t range", t.min().round(3), t.max().round(3))
    for ts in np.arange(t.min(), t.max(), 0.02):
        s=(t>=ts)&(t<ts+0.02)
        if s.sum()>3: print(f"  t[{ts:+.3f}] n={s.sum():3d} width {w[s].max()-w[s].min():.3f} ztop {z[s].max():.3f} wc {w[s].mean():+.3f}")
    # body only (z>0.93) center
    b = z>0.932
    print("body(z>0.932) center", pts[b].mean(0).round(3), "t range", t[b].min().round(3), t[b].max().round(3))
