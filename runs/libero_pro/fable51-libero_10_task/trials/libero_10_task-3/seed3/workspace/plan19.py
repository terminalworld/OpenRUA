import numpy as np, math
import collide
from panda import *
from collide import check, OBST
np.set_printoptions(precision=3, suppress=True)
def hand_points_closed(Th):
    pts = []
    for x in (-0.03, 0.03):
        for y in np.linspace(-0.10, 0.10, 9):
            for z in (-0.04, 0.0, 0.035, 0.07):
                pts.append([x, y, z, 1])
    for x in (-0.012, 0.012):
        for y in (-0.01, 0.0, 0.01):
            for z in (0.075, 0.09, 0.105):   # finger body; tips (0.115) excluded -> they touch the panel
                pts.append([x, y, z, 1])
    return (Th @ np.array(pts).T).T[:, :3]
collide.hand_points = hand_points_closed
OBST['tophandle'] = ([-0.05, 0.19, 1.005], [0.06, 0.23, 1.11])
OBST['cabinet'] = ([-0.14, 0.24, 0.90], [0.145, 0.44, 1.14])       # body behind panel plane
OBST['lintel'] = ([-0.14, 0.228, 0.99], [0.145, 0.44, 1.14])       # frame above bottom drawer opening
ign = ('drawer','bottle')
def extras(yp):
    return {'bar': ([-0.045, yp-0.033, 0.938], [0.055, yp-0.009, 0.962]),
            'postL': ([-0.037, yp-0.02, 0.938], [-0.023, yp, 0.962]),
            'postR': ([0.028, yp-0.02, 0.938], [0.043, yp, 0.962]),
            'paneltop': ([-0.11, yp, 0.983], [0.12, yp+0.02, 0.99])}
x = -0.08
for deg in (70, 75, 80, 85):
  for zc in (0.945, 0.955):
    th = math.radians(deg)
    a = np.array([0, math.sin(th), -math.cos(th)]); xh = np.array([0, math.cos(th), math.sin(th)])
    R = R_from_axes(a, y=[1, 0, 0])
    off = 0.0116*a - 0.012*xh
    rng = np.random.default_rng(1); best=None
    for t in range(40):
        q0 = rng.uniform(LIM[:,0]+0.1, LIM[:,1]-0.1); qs=[]; qp=q0; ok=True; cs=[]
        for yp in (0.165, 0.20, 0.228):
            p = np.array([x, yp, zc]) - off
            q = ik(make_T(p, R), qp)
            if q is None: ok=False; break
            qs.append(q); qp=q; cs.append(check(q, ignore=ign, extra=extras(yp)))
        if not ok: continue
        worst = min(c[0] for c in cs)
        if best is None or worst > best[0]: best=(worst, cs, qs)
    print(deg, zc, 'best worst %.3f'%best[0] if best else None, [(round(float(c[0]),3), c[1]) for c in best[1]] if best else '')
    if best and best[0] > 0.003: np.save(f'plan19_{deg}_{zc}.npy', np.array(best[2]))
