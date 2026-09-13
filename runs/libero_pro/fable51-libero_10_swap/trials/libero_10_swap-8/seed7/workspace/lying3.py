from potpose import measure
import numpy as np
wx, wy, wz = measure(fresh=True)
h = wz - 0.899
m = (wx > -0.20) & (wx < 0.12) & (wy > 0.12) & (wy < 0.30) & (h > 0.02)
print("pts", m.sum(), "hmax", h[m].max().round(4), "x range", wx[m].min().round(3), wx[m].max().round(3), "y range", wy[m].min().round(3), wy[m].max().round(3))
for x0 in np.arange(-0.14, 0.09, 0.01):
    s = m & (wx >= x0) & (wx < x0+0.01)
    if s.sum(): print(f"x {x0:+.2f}: hmax {h[s].max():.3f} y@max {wy[s][np.argmax(h[s])]:.3f} yrange {wy[s].min():.3f}..{wy[s].max():.3f}")
