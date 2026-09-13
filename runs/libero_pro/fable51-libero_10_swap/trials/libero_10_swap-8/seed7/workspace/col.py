from potpose import measure
import numpy as np
wx, wy, wz = measure(fresh=False)
wx, wy, wz = wx.ravel(), wy.ravel(), wz.ravel(); h = wz - 0.899
c = np.array([0.088, 0.2038]); ax = np.array([0.589, 0.808]); sv = np.array([-ax[1], ax[0]])
P = np.stack([wx, wy], 1); a = (P - c) @ ax; s = (P - c) @ sv
for lo, hi, lab in [(-0.048, -0.032, "handle-side finger column"), (0.032, 0.048, "far-side finger column")]:
    m = (s >= lo) & (s < hi) & (h > 0.02) & (a > -0.1) & (a < 0.1)
    print(lab)
    for a0 in np.arange(-0.09, 0.08, 0.01):
        mm = m & (a >= a0) & (a < a0 + 0.01)
        if mm.sum(): print(f"  a {a0:+.2f}: n{mm.sum():2d} h {h[mm].min():.3f}..{h[mm].max():.3f}  s@hmax {s[mm][np.argmax(h[mm])]:+.3f}")
