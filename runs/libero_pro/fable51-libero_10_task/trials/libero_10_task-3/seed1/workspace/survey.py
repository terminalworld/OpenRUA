import numpy as np
def hmap(P, xr, yr, cell=0.01, zmin=0.91):
    x,y,z = P[...,0].ravel(), P[...,1].ravel(), P[...,2].ravel()
    m = np.isfinite(z) & (x>xr[0])&(x<xr[1])&(y>yr[0])&(y<yr[1])&(z>zmin)
    return x[m],y[m],z[m]
for cam in ("birdview","agentview"):
    P = np.load(f"{cam}_xyz.npy")
    print("==", cam)
    # bottle region
    x,y,z = hmap(P, (-0.45,-0.15), (-0.15,0.12))
    if len(z):
        print("bottle pts", len(z), "x", x.min().round(3), x.max().round(3), "y", y.min().round(3), y.max().round(3), "ztop", z.max().round(3))
        # body vs neck: body where z>0.925
        b = z>0.925
        print(" body x", x[b].min().round(3), x[b].max().round(3), "y", y[b].min().round(3), y[b].max().round(3), "ztop", z[b].max().round(3), "ymean", y[b].mean().round(3))
        # per x-slice y-center & top
        for xs in np.arange(-0.40,-0.20,0.02):
            s = (x>=xs)&(x<xs+0.02)
            if s.sum()>3: print(f"  x[{xs:.2f}] n={s.sum():3d} y {y[s].min():.3f}..{y[s].max():.3f} yc {y[s].mean():.3f} ztop {z[s].max():.3f}")
    # drawer region: front panel & handle
    x,y,z = hmap(P, (-0.12,0.12), (0.0,0.26), zmin=0.915)
    print("drawer region pts", len(z))
    for ys in np.arange(0.0,0.26,0.01):
        s=(y>=ys)&(y<ys+0.01)
        if s.sum()>3: print(f"  y[{ys:.2f}] n={s.sum():3d} x {x[s].min():.3f}..{x[s].max():.3f} ztop {z[s].max():.3f} zmed {np.median(z[s]):.3f}")
