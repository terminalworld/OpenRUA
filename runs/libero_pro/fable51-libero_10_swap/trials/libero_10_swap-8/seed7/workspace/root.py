from potpose import measure
import numpy as np
wx, wy, wz = measure(fresh=False)
wx, wy, wz = wx.ravel(), wy.ravel(), wz.ravel(); h = wz - 0.899
c = np.array([0.088, 0.2038]); ax = np.array([0.589, 0.808]); sv = np.array([-ax[1], ax[0]])
P = np.stack([wx, wy], 1); a = (P - c) @ ax; s = (P - c) @ sv
for s0 in np.arange(-0.11, -0.03, 0.01):
    m = (s >= s0) & (s < s0+0.01) & (h > 0.036) & (h < 0.06) & (a > -0.1) & (a < 0.1)
    if m.sum():
        aa = np.sort(a[m]); print(f"s {s0:+.2f}: n{m.sum():3d} a range {aa.min():+.3f}..{aa.max():+.3f}  a-hist", np.histogram(aa, bins=np.arange(-0.09,0.09,0.01))[0])
