import numpy as np
from potpose import measure
wx, wy, wz = measure(fresh=True)
S = 0.93
m = (wx>0.09)&(wx<0.30)&(wy>0.04)&(wy<0.15)&(wz>S+0.01)
h = wz - S
print("lying pot B: x[%.3f,%.3f] y[%.3f,%.3f] hmax=%.3f" % (wx[m].min(), wx[m].max(), wy[m].min(), wy[m].max(), h[m].max()))
for x0 in np.arange(0.10, 0.29, 0.01):
    mm = m & (wx>=x0) & (wx<x0+0.01)
    if mm.sum()<2: continue
    ridge = mm & (h > h[mm].max()-0.01)
    print(f"  x {x0:.2f}: n={mm.sum():3d} y[{wy[mm].min():.3f},{wy[mm].max():.3f}] width={100*(wy[mm].max()-wy[mm].min()):.1f}cm hmax={h[mm].max()*100:.1f}cm ridge_y={wy[ridge].mean():.3f}")
