import numpy as np
from potpose import measure
wx, wy, wz = measure(fresh=True)
S = 0.899
m = (wx>-0.20)&(wx<0.05)&(wy>0.05)&(wy<0.45)&(wz>S+0.01)
h = wz - S
print("pot B on table: x[%.3f,%.3f] y[%.3f,%.3f] hmax=%.3f" % (wx[m].min(), wx[m].max(), wy[m].min(), wy[m].max(), h[m].max()))
for y0 in np.arange(0.08, 0.36, 0.01):
    mm = m & (wy>=y0) & (wy<y0+0.01)
    if mm.sum()<2: continue
    ridge = mm & (h > h[mm].max()-0.01)
    print(f"  y {y0:.2f}: n={mm.sum():3d} x[{wx[mm].min():.3f},{wx[mm].max():.3f}] width={100*(wx[mm].max()-wx[mm].min()):.1f}cm hmax={h[mm].max()*100:.1f}cm ridge_x={wx[ridge].mean():.3f}")
