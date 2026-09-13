import numpy as np
from cloud import cloud
for cam in ("sideview", "frontview"):
    X, Y, Z, _ = cloud(cam)
    m = (Y > 0.045) & (Y < 0.15) & (X > 0.08) & (X < 0.40) & (Z > 0.94) & (Z < 1.30)
    print(cam, "pot B pts", m.sum(), "x[%.3f,%.3f] z[%.3f,%.3f]" % (X[m].min(), X[m].max(), Z[m].min(), Z[m].max()))
    for z0 in np.arange(0.94, 1.30, 0.02):
        mm = m & (Z >= z0) & (Z < z0 + 0.02)
        if mm.sum() < 3: continue
        print(f"  z {z0:.2f}: n={mm.sum():4d} x[{X[mm].min():.3f},{X[mm].max():.3f}] y[{Y[mm].min():.3f},{Y[mm].max():.3f}]")
