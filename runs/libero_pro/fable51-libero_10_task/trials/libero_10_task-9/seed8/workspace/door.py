import numpy as np
for cam in ["birdview","agentview"]:
    d = np.load(f"{cam}_cloud.npz"); pw = d["pw"]
    X,Y,Z = pw[...,0], pw[...,1], pw[...,2]
    m = (Z>0.95)&(Z<1.13)&(X<-0.16)&(X>-0.6)&(Y>-0.1)&(Y<0.3)
    print(cam, "door-cand pts", m.sum())
    if m.sum()==0: continue
    xs, ys, zs = X[m], Y[m], Z[m]
    # bin by x
    for x in np.arange(-0.55,-0.15,0.02):
        mm = (xs>=x)&(xs<x+0.02)
        if mm.sum(): print(f"  x[{x:.2f}] n={mm.sum():4d} y[{ys[mm].min():.3f},{ys[mm].max():.3f}] z[{zs[mm].min():.3f},{zs[mm].max():.3f}]")
