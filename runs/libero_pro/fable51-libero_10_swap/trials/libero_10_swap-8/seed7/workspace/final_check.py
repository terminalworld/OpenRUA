"""Birdview check: find the two pots, report apex, footprint, and whether they stand on the stove plate."""
import numpy as np
from potpose import measure
wx, wy, wz = [a.ravel() for a in measure(True)]
ok = np.isfinite(wz)
wx, wy, wz = wx[ok], wy[ok], wz[ok]
PLATE = dict(x=(0.086, 0.276), y=(-0.047, 0.140), top=0.93)
scene = (wz > 0.95) & (wx > -0.45) & (wx < 0.45) & (wy > -0.45) & (wy < 0.45)
print("points above 0.95:", scene.sum())
# cluster by simple flood on a 1 cm grid
g = {}
for x, y, z in zip(wx[scene], wy[scene], wz[scene]):
    g.setdefault((round(x/0.01), round(y/0.01)), []).append(z)
seen = set(); clusters = []
for k in g:
    if k in seen: continue
    stack = [k]; comp = []
    while stack:
        c = stack.pop()
        if c in seen: continue
        seen.add(c); comp.append(c)
        for dx in (-1,0,1):
            for dy in (-1,0,1):
                n = (c[0]+dx, c[1]+dy)
                if n in g and n not in seen: stack.append(n)
    clusters.append(comp)
for comp in sorted(clusters, key=len, reverse=True):
    if len(comp) < 20: continue
    xs = np.array([c[0]*0.01 for c in comp]); ys = np.array([c[1]*0.01 for c in comp])
    zs = np.array([max(g[c]) for c in comp])
    top = zs.max(); apex = np.array([xs[zs > top-0.01].mean(), ys[zs > top-0.01].mean()])
    body = zs > 1.0  # lid/upper chamber region
    cx, cy = xs[body].mean(), ys[body].mean()
    ext_x = (xs[body].min(), xs[body].max()); ext_y = (ys[body].min(), ys[body].max())
    on = PLATE["x"][0] < cx-0.035 and cx+0.035 < PLATE["x"][1] and PLATE["y"][0] < cy-0.035 and cy+0.035 < PLATE["y"][1]
    print(f"cluster n={len(comp)} top z={top:.3f} apex=({apex[0]:.3f},{apex[1]:.3f}) body centre=({cx:.3f},{cy:.3f}) "
          f"x[{ext_x[0]:.2f},{ext_x[1]:.2f}] y[{ext_y[0]:.2f},{ext_y[1]:.2f}]  upright={abs(top-1.084)<0.01}  base within plate={on}")
