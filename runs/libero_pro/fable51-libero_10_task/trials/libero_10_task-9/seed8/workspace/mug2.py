import numpy as np
for cam in ["agentview","frontview","birdview"]:
    d = np.load(f"{cam}_cloud.npz"); pw = d["pw"]
    X,Y,Z = pw[...,0], pw[...,1], pw[...,2]
    m = (np.abs(X+0.10)<0.08)&(np.abs(Y+0.255)<0.08)&(Z>0.905)&(Z<1.0)
    xs,ys,zs=X[m],Y[m],Z[m]
    print(cam, "white mug pts", m.sum())
    for z in np.arange(0.90,1.0,0.01):
        mm=(zs>=z)&(zs<z+0.01)
        if mm.sum()>3: print(f"  z[{z:.2f}] n={mm.sum():4d} x[{xs[mm].min():.3f},{xs[mm].max():.3f}] y[{ys[mm].min():.3f},{ys[mm].max():.3f}] xc={(xs[mm].min()+xs[mm].max())/2:.3f}")
