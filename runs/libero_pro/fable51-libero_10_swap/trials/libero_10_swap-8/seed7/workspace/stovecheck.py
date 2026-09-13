import numpy as np
from potpose import measure, pot_pose
wx, wy, wz = measure(fresh=True)
for name,(x,y) in {"A":(0.22,-0.004),"B":(0.13,0.097)}.items():
    print(name, pot_pose(wx, wy, wz, x, y, 0.93))
m = (wx>0.05)&(wx<0.32)&(wy>-0.08)&(wy<0.17)&(wz>0.93+0.01)
for h0,h1 in [(0.01,0.05),(0.05,0.10),(0.10,0.13),(0.13,0.145),(0.145,0.2)]:
    mm = m & (wz-0.93>=h0)&(wz-0.93<h1)
    if mm.sum(): print(f"  h{h0*100:.0f}-{h1*100:.0f}: n={mm.sum()} x[{wx[mm].min():.3f},{wx[mm].max():.3f}] y[{wy[mm].min():.3f},{wy[mm].max():.3f}]")
