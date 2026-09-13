from cloud import cloud
import numpy as np
X, Y, Z, _ = cloud('sideview')
m = (X > -0.12) & (X < 0.08) & (Y > 0.16) & (Y < 0.26) & (Z > 0.90) & (Z < 1.10)
print("pts", m.sum())
# for each x bin, list z clusters of points near the pot mid-plane (y within 1.5 cm of axis 0.207)
mm = m & (np.abs(Y - 0.207) < 0.015)
for x0 in np.arange(-0.10, 0.06, 0.01):
    s = mm & (X >= x0) & (X < x0 + 0.01)
    if not s.sum(): continue
    zs = np.sort(Z[s]); gaps = np.where(np.diff(zs) > 0.008)[0]
    segs = np.split(zs, gaps + 1)
    print(f"x {x0:+.2f}: " + "  ".join(f"[{sg.min():.3f}-{sg.max():.3f}]n{len(sg)}" for sg in segs))
