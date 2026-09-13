import numpy as np
from cloud import cloud
X, Y, Z, _ = cloud("frontview")
m = (X > -0.16) & (X < 0.0) & (Y > 0.22) & (Y < 0.32) & (Z > 0.905) & (Z < 1.05)
print("pts", m.sum())
for y0 in np.arange(0.22, 0.30, 0.005):
    mm = m & (Y >= y0) & (Y < y0 + 0.005)
    if mm.sum() < 2: continue
    print(f"  y {y0:.3f}: n={mm.sum():4d} x[{X[mm].min():.3f},{X[mm].max():.3f}] z[{Z[mm].min():.3f},{Z[mm].max():.3f}]  zc={0.5*(Z[mm].min()+Z[mm].max()):.3f}")
