import numpy as np
P = np.load("birdview_world.npy")
X,Y,Z = P[...,0],P[...,1],P[...,2]
for name,(x0,x1,y0,y1) in {"white":(-0.2,0.0,-0.4,-0.15),"yellow":(-0.15,0.1,-0.1,0.12)}.items():
    reg = (X>x0)&(X<x1)&(Y>y0)&(Y<y1)&(Z>0.91)
    pts = P[reg]
    print(f"{name}: n={len(pts)} x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] z[{pts[:,2].min():.3f},{pts[:,2].max():.3f}]")
    h,e = np.histogram(pts[:,2], bins=np.arange(0.9,1.1,0.01))
    print("  z hist:", [(f"{e[i]:.2f}",int(h[i])) for i in range(len(h)) if h[i]>0])
    zmax = pts[:,2].max()
    rim = reg & (Z > zmax-0.015)
    pr = P[rim]
    print(f"  rim: center ({pr[:,0].mean():.3f},{pr[:,1].mean():.3f}) x[{pr[:,0].min():.3f},{pr[:,0].max():.3f}] y[{pr[:,1].min():.3f},{pr[:,1].max():.3f}]")
    # body-only rim excluding handle: fit circle to rim points via least squares
    A = np.c_[2*pr[:,0], 2*pr[:,1], np.ones(len(pr))]
    b = pr[:,0]**2 + pr[:,1]**2
    cx, cy, c = np.linalg.lstsq(A, b, rcond=None)[0]
    r = np.sqrt(c + cx**2 + cy**2)
    print(f"  circle fit: center ({cx:.3f},{cy:.3f}) r={r:.3f}")
