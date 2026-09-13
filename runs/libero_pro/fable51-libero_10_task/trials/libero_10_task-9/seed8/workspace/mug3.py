import numpy as np
for cam in ["agentview","frontview","birdview"]:
    d = np.load(f"{cam}_cloud.npz"); pw = d["pw"]
    X,Y,Z = pw[...,0], pw[...,1], pw[...,2]
    for name,(x0,y0) in {"white":(-0.09,-0.255),"yellow":(0.024,-0.003)}.items():
        m = (np.abs(X-x0)<0.08)&(np.abs(Y-y0)<0.08)&(Z>0.905)&(Z<1.25)
        xs,ys,zs=X[m],Y[m],Z[m]
        print(cam, name, "pts", m.sum())
        for z in np.arange(0.90,1.15,0.01):
            mm=(zs>=z)&(zs<z+0.01)
            if mm.sum()>3: print(f"  z[{z:.2f}] n={mm.sum():4d} x[{xs[mm].min():.3f},{xs[mm].max():.3f}] y[{ys[mm].min():.3f},{ys[mm].max():.3f}] xc={(xs[mm].min()+xs[mm].max())/2:.3f} yc={(ys[mm].min()+ys[mm].max())/2:.3f}")
