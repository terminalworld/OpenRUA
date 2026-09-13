from potpose import measure
import numpy as np
wx, wy, wz = measure(fresh=True)
h = wz - 0.899
m = (wx > -0.05) & (wx < 0.20) & (wy > 0.05) & (wy < 0.40) & (h > 0.02)
# exclude stove plate (h 0.031) and knob object region
print("pts", m.sum(), "hmax", h[m].max().round(4))
for y0 in np.arange(0.05, 0.36, 0.01):
    s = m & (wy >= y0) & (wy < y0+0.01)
    if s.sum(): print(f"y {y0:.2f}: hmax {h[s].max():.3f} x@max {wx[s][np.argmax(h[s])]:.3f} xrange {wx[s].min():.3f}..{wx[s].max():.3f}  n{s.sum()}")
