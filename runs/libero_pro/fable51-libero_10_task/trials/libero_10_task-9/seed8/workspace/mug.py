import numpy as np
d = np.load("birdview_cloud.npz"); pw = d["pw"]; col=d["color"]
X,Y,Z = pw[...,0], pw[...,1], pw[...,2]
for name,(x0,y0) in {"white":(-0.11,-0.25),"yellow":(0.01,0.0)}.items():
    m = (np.abs(X-x0)<0.09)&(np.abs(Y-y0)<0.09)&(Z>0.905)
    xs,ys,zs=X[m],Y[m],Z[m]
    print(name, "pts", m.sum(), "z range", zs.min(), zs.max())
    for z in np.arange(0.90,1.0,0.01):
        mm=(zs>=z)&(zs<z+0.01)
        if mm.sum(): print(f"  z[{z:.2f}] n={mm.sum():4d} x[{xs[mm].min():.3f},{xs[mm].max():.3f}] y[{ys[mm].min():.3f},{ys[mm].max():.3f}]")
    top = zs>zs.max()-0.01
    print("  top-rim centroid", xs[top].mean(), ys[top].mean(), "extent x", xs[top].max()-xs[top].min(), "y", ys[top].max()-ys[top].min())
