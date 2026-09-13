from potpose import measure
import numpy as np
wx, wy, wz = measure(fresh=False)
h = wz - 0.899
c = np.array([0.088, 0.2038]); ax = np.array([0.589, 0.808]); sv = np.array([-ax[1], ax[0]])
wx, wy, wz = wx.ravel(), wy.ravel(), wz.ravel(); h = h.ravel(); P = np.stack([wx, wy], 1); a = (P - c) @ ax; s = (P - c) @ sv
sel = (a > -0.01) & (a < 0.025) & (h > 0.003)
for s0 in np.arange(-0.09, 0.06, 0.01):
    m = sel & (s >= s0) & (s < s0 + 0.01)
    if m.sum(): print(f"s {s0:+.2f}: n{m.sum():3d} h {h[m].min():.3f}..{h[m].max():.3f}")
print("-- table check near (-0.06,0.36), (-0.04,0.25), (0.0,0.40):")
for px, py in [(-0.06, 0.36), (-0.04, 0.25), (0.0, 0.40), (-0.1, 0.38)]:
    m = (np.abs(wx - px) < 0.02) & (np.abs(wy - py) < 0.02)
    print(px, py, "n", m.sum(), "z", (wz[m].min().round(3), wz[m].max().round(3)) if m.sum() else None)
