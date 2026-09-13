from potpose import measure
import numpy as np
wx, wy, wz = measure(fresh=True)
h = wz - 0.899
m = (wx > -0.14) & (wx < -0.01) & (wy > 0.08) & (wy < 0.30) & (h > 0.02)
print("pot B pts", m.sum(), "hmax", h[m].max().round(4))
top = m & (h > h[m].max() - 0.012)
print("ridge x range", wx[top].min().round(3), wx[top].max().round(3), "y range", wy[top].min().round(3), wy[top].max().round(3), "mean", wx[top].mean().round(4), wy[top].mean().round(4))
for y0 in np.arange(0.09, 0.28, 0.01):
    s = m & (wy >= y0) & (wy < y0+0.01)
    if s.sum(): print(f"y {y0:.2f}: hmax {h[s].max():.3f} x@max {wx[s][np.argmax(h[s])]:.3f} xrange {wx[s].min():.3f}..{wx[s].max():.3f}")
