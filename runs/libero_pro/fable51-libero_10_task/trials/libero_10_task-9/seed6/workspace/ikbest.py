import numpy as np
from rob import *
def best_ik(r, tcp, R, seeds=None, tries=8, weights=None):
    """Try several seeds, return the solution closest (weighted) to current joints."""
    cur = np.array(r.joints()); sols = []
    lim = np.array(ARM["limits_rad"])
    seeds = [cur] + (seeds or [])
    rng = np.random.default_rng(0)
    for i in range(tries):
        s = seeds[i] if i < len(seeds) else np.clip(cur + rng.normal(0, 0.6, 7), lim[:,0]+0.05, lim[:,1]-0.05)
        q = r.solve_ik(hand_pose_from_tcp(tcp, R), R, seed=list(s), timeout=2.0)
        if q is not None: sols.append(np.array(q))
    if not sols: return None
    w = np.array(weights if weights is not None else [1,1,1,1,0.5,0.5,0.3])
    d = [np.sum(w*np.abs(q-cur)) for q in sols]
    return list(sols[int(np.argmin(d))])
