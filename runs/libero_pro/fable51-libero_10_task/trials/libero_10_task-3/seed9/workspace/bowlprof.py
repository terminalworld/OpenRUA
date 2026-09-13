import numpy as np
from rob import *
r = Robot("bp")
pts = np.vstack([r.cloud(c)[1].reshape(-1, 3) for c in ["agentview", "birdview", "frontview"]])
pts = pts[np.isfinite(pts).all(1)]
d = pts[:, :2] - np.array([-0.01, -0.075]); rho = np.hypot(d[:, 0], d[:, 1])
m = (rho < 0.13) & (pts[:, 2] > 0.90) & (pts[:, 2] < 1.06)
p = pts[m]; rho = rho[m]
for lo in np.arange(0, 0.13, 0.01):
    s = (rho >= lo) & (rho < lo + 0.01)
    if s.sum(): print(f"rho {lo:.2f}-{lo+0.01:.2f}: n={s.sum():4d} z min {p[s,2].min():.3f} max {p[s,2].max():.3f} p90 {np.percentile(p[s,2],90):.3f}")
print("--- high pts near bowl")
s = (rho > 0.10) & (p[:, 2] > 0.99)
print(np.round(np.percentile(p[s], [0, 50, 100], axis=0), 3))
s = (p[:, 2] > 0.94) & (rho < 0.10)
print("rim-ish pts z>0.94:", s.sum(), "x", np.round(np.percentile(p[s,0],[0,100]),3), "y", np.round(np.percentile(p[s,1],[0,100]),3))
# bowl outline by angle
ang = np.degrees(np.arctan2(d[m][:,1], d[m][:,0]))
for a0 in range(-180, 180, 45):
    s = (ang >= a0) & (ang < a0+45) & (p[:,2] > 0.93) & (rho < 0.11)
    if s.sum(): print(f"ang {a0:4d}: rho max {rho[s].max():.3f} z max {p[s,2].max():.3f}")
