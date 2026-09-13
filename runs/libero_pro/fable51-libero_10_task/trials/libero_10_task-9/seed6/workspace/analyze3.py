import numpy as np
P = np.load("birdview_world.npy")
X,Y,Z = P[...,0],P[...,1],P[...,2]
for name,(x0,x1,y0,y1) in {"white":(-0.2,0.0,-0.4,-0.15),"yellow":(-0.15,0.1,-0.1,0.12)}.items():
    reg = (X>x0)&(X<x1)&(Y>y0)&(Y<y1)
    for zlo in [0.93, 1.0, 1.04, 1.06, 1.08]:
        m = reg & (Z>zlo)
        if m.sum()==0: continue
        pts = P[m]
        print(f"{name} z>{zlo}: n={m.sum()} x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] (w={pts[:,0].max()-pts[:,0].min():.3f}) y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] (w={pts[:,1].max()-pts[:,1].min():.3f})")
    # inside floor
    m = reg & (Z>0.93)&(Z<1.0)
    pts=P[m]; print(f"{name} inner floor z mean {pts[:,2].mean():.3f}, center ({pts[:,0].mean():.3f},{pts[:,1].mean():.3f}), n={len(pts)}")
    # rim ring center
    m = reg & (Z>1.06)
    pts=P[m]; print(f"{name} rim center ({pts[:,0].mean():.3f},{pts[:,1].mean():.3f}) zmax={pts[:,2].max():.3f}")
