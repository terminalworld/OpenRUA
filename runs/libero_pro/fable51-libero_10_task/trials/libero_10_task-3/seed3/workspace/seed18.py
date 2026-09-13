import numpy as np, math
from panda import *
from collide import check, check_path
np.set_printoptions(precision=3, suppress=True)
ign = ('drawer','bottle')
for deg in (52, 55):
    x, z = 0.0, 0.959
    th = math.radians(deg)
    R = R_from_axes([0, math.cos(th), -math.sin(th)], y=[1, 0, 0])
    P = [[x, 0.025, 1.15], [x, 0.025, z], [x, 0.10, z], [x, 0.185, z]]
    rng = np.random.default_rng(0); best = None
    for t in range(60):
        q0 = rng.uniform(LIM[:,0]+0.1, LIM[:,1]-0.1)
        qs = []; qp = q0; ok = True
        for p in P:
            q = ik(make_T(p, R), qp)
            if q is None: ok = False; break
            qs.append(q); qp = q
        if not ok: continue
        c = check_path(qs, ignore=ign)
        if best is None or c[0] > best[0][0]: best = (c, qs)
    print(deg, 'best', best[0] if best else None, [(round(float(check(q, ignore=ign)[0]),3), check(q, ignore=ign)[1]) for q in best[1]] if best else '')
    if best: np.save(f'seed18_{deg}.npy', np.array(best[1]))
