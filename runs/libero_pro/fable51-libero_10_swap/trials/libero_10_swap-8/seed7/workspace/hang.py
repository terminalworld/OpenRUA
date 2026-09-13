import numpy as np
from potpose import measure
wx, wy, wz = measure(fresh=True)
m = (wx>0.05)&(wx<0.40)&(wy>0.05)&(wy<0.145)&(wz>0.95)&(wz<1.235)
print("hanging B footprint: x[%.3f,%.3f] y[%.3f,%.3f] ztop=%.3f zmin=%.3f" % (wx[m].min(), wx[m].max(), wy[m].min(), wy[m].max(), wz[m].max(), wz[m].min()))
for x0 in np.arange(0.05, 0.40, 0.01):
    mm = m & (wx>=x0) & (wx<x0+0.01)
    if mm.sum()<2: continue
    print(f"  x {x0:.2f}: n={mm.sum():3d} y[{wy[mm].min():.3f},{wy[mm].max():.3f}] zmax={wz[mm].max():.3f} zmin={wz[mm].min():.3f}")
