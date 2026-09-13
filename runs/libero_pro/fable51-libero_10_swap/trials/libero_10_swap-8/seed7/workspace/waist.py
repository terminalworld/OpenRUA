from potpose import measure
import numpy as np
wx, wy, wz = measure(fresh=True)
h = wz - 0.899
m = (wx > 0.0) & (wx < 0.20) & (wy > 0.13) & (wy < 0.32) & (h > 0.045) & ~((wx > 0.086) & (wy < 0.14))
P = np.stack([wx[m], wy[m]], 1); hh = h[m]
c = P.mean(0); U, S, Vt = np.linalg.svd(P - c); ax = Vt[0]
if ax[1] < 0: ax = -ax          # +a toward +y (base end)
a = (P - c) @ ax; s = (P - c) @ np.array([-ax[1], ax[0]])
print("centroid", np.round(c,4), "axis dir", np.round(ax,3), "angle deg", round(np.degrees(np.arctan2(ax[1], ax[0])),1))
print("a range", a.min().round(3), a.max().round(3))
for a0 in np.arange(a.min(), a.max(), 0.005):
    sl = (a >= a0) & (a < a0+0.005)
    if sl.sum() > 2:
        print(f"a {a0:+.3f}: n{sl.sum():3d} width {s[sl].max()-s[sl].min():.3f} (s {s[sl].min():+.3f}..{s[sl].max():+.3f}) hmax {hh[sl].max():.3f} s@hmax {s[sl][np.argmax(hh[sl])]:+.3f}")
np.save('waist_axis.npy', np.array([c[0], c[1], ax[0], ax[1]]))
