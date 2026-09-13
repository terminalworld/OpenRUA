import numpy as np
for cam in ["agentview","frontview","birdview"]:
    d = np.load(f"{cam}_cloud.npz"); pw = d["pw"]
    X,Y,Z = pw[...,0], pw[...,1], pw[...,2]
    m = (np.abs(X+0.0985)<0.08)&(Y>-0.205)&(Y<-0.12)&(Z>0.905)&(Z<1.02)
    xs,ys,zs=X[m],Y[m],Z[m]
    print(cam, "handle pts", m.sum())
    for z in np.arange(0.90,1.02,0.01):
        mm=(zs>=z)&(zs<z+0.01)
        if mm.sum()>2: print(f"  z[{z:.2f}] n={mm.sum():4d} x[{xs[mm].min():.3f},{xs[mm].max():.3f}] xc={np.median(xs[mm]):.3f} y[{ys[mm].min():.3f},{ys[mm].max():.3f}]")
    # y-profile at mid height
    mm=(zs>0.94)&(zs<0.97)
    for y in np.arange(-0.205,-0.17,0.005):
        k=mm&(ys>=y)&(ys<y+0.005)
        if k.sum()>0: print(f"    y[{y:.3f}] n={k.sum()} x[{xs[k].min():.3f},{xs[k].max():.3f}] z[{zs[k].min():.3f},{zs[k].max():.3f}]")
