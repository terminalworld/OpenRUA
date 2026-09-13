import numpy as np, math
from pushmodel import *
from panda import *
from collide import check, check_path
import panda
r = Robot(); q0 = r.q()
deg, zc, x = 80, 0.945, -0.08
th = math.radians(deg)
a = np.array([0, math.sin(th), -math.cos(th)]); xh = np.array([0, math.cos(th), math.sin(th)])
R = R_from_axes(a, y=[1, 0, 0]); off = 0.0116*a - 0.012*xh
qt = np.load(f'plan19_{deg}_{zc}.npy')[0]
Tpre = make_T(np.array([x, 0.15, 1.10]) - off, R)
qpre = ik(Tpre, qt); print('qpre', qpre.round(3), check(qpre, ignore=ign, extra=extras(0.175)))
print('direct', check_path([q0, qpre], ignore=ign, extra=extras(0.175)))
rng = np.random.default_rng(3); best=None
for t in range(400):
    qm = rng.uniform(LIM[:,0]+0.1, LIM[:,1]-0.1)
    # bias: interpolate then perturb
    s = rng.uniform(0.2,0.8); qm = q0*(1-s)+qpre*s + rng.normal(0,0.5,7); qm = np.clip(qm, LIM[:,0]+0.05, LIM[:,1]-0.05)
    c1 = check_path([q0, qm], ignore=ign, extra=extras(0.175), steps=8); c2 = check_path([qm, qpre], ignore=ign, extra=extras(0.175), steps=8)
    w = min(c1[0], c2[0])
    if best is None or w > best[0]: best=(w, qm, c1, c2)
print('best via', best[0], best[1].round(3), best[2], best[3])
np.save('via19.npy', np.array([best[1], qpre]))
