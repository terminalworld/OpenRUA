import math, sys, numpy as np
from arm import Arm, HOME, R_quat
from stage8 import a, n, Z, G, R_from
arm = Arm("probe8")
cur = np.array(arm.arm_q())
for beta in [35, 45, 55]:
    for gam in [0, 20, 35, 50, 70]:
        b, g = math.radians(beta), math.radians(gam)
        h = math.cos(g) * (-a) + math.sin(g) * (-n)   # horizontal approach dir, yawed toward -n (pointing -y-ish)
        d = math.cos(b) * h - math.sin(b) * Z
        f = (Z - (Z @ d) * d); f /= np.linalg.norm(f)
        best = None
        for fs in (f, -f):
            R = R_from(d, fs)
            for seed in (cur, np.array(HOME)):
                q = arm.ik_world(G, R_quat(R), seed=list(seed), tries=1)
                if q is not None:
                    dq = np.abs(np.array(q) - cur).max()
                    if best is None or dq < best[0]:
                        best = (dq, np.round(q, 2), 'f+' if fs is f else 'f-')
        print(f"beta={beta} gam={gam} ->", best, flush=True)
