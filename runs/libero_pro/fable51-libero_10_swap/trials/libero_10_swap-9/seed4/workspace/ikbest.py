import numpy as np
from rob import LIMITS

def ik_best(r, pos, quat, ref, n=12, extra_seeds=()):
    """collect IK solutions from several seeds, return the one closest (weighted L-inf/L2) to ref."""
    sols = []
    seeds = [np.array(ref)] + [np.array(s) for s in extra_seeds]
    rng = np.random.default_rng(0)
    for i in range(n):
        seeds.append(np.clip(np.array(ref) + rng.uniform(-0.6, 0.6, 7), LIMITS[:, 0], LIMITS[:, 1]))
    for s in seeds:
        q = r.solve_ik(pos, quat, seed=s, tries=1)
        if q is not None:
            sols.append(q)
    if not sols:
        return None, None
    ref = np.array(ref)
    d = [np.linalg.norm(q - ref) + 2 * np.abs(q - ref).max() for q in sols]
    i = int(np.argmin(d))
    return sols[i], d[i]
