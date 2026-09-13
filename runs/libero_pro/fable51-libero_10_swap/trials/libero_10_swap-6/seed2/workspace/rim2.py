import numpy as np, sys
P = np.load(f"snaps/{sys.argv[1]}_P.npy")
Z = P[...,2]
def fit_circle(xy):
    x,y = xy[:,0], xy[:,1]
    A = np.c_[2*x, 2*y, np.ones_like(x)]
    c, *_ = np.linalg.lstsq(A, x*x+y*y, rcond=None)
    return c[0], c[1], np.sqrt(c[2] + c[0]**2 + c[1]**2)
for name, box, zlo, zhi in [("white mug rim", (-0.22,-0.05,-0.25,-0.04), 0.54, 0.65),
                       ("white mug all", (-0.22,-0.05,-0.25,-0.04), 0.44, 0.65),
                       ("red mug rim", (-0.12,0.05,0.0,0.2), 0.54, 0.65),
                       ("plate", (0.03,0.25,-0.12,0.12), 0.43, 0.47),
                       ("pudding", (-0.3,-0.15,-0.05,0.08), 0.44, 0.50)]:
    m = (P[...,0]>box[0])&(P[...,0]<box[1])&(P[...,1]>box[2])&(P[...,1]<box[3])&(Z>zlo)&(Z<zhi)&np.isfinite(Z)
    pts = P[m]
    if len(pts)==0: print(name, "none"); continue
    cx,cy,r = fit_circle(pts[:,:2])
    print(f"{name}: n={len(pts)} circle c=({cx:.3f},{cy:.3f}) r={r:.3f} | xr=({pts[:,0].min():.3f},{pts[:,0].max():.3f}) yr=({pts[:,1].min():.3f},{pts[:,1].max():.3f}) z=({pts[:,2].min():.3f},{pts[:,2].max():.3f})")
