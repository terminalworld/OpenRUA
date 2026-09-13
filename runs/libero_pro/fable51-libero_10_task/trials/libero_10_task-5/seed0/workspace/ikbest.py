import numpy as np
LO = np.array([-2.8973,-1.7628,-2.8973,-3.0718,-2.8973,-0.0175,-2.8973])
HI = np.array([ 2.8973, 1.7628, 2.8973,-0.0698, 2.8973, 3.7525, 2.8973])
# q6 > ~2.9 folds the hand back onto the forearm (self-collision): treat as a hard limit
HI_M = HI.copy(); HI_M[5] = 2.9
def margin(q):
    q = np.array(q); return float(np.min(np.minimum(q-LO, HI_M-q)))
def ik_best(r, pos, quat, n=25, seed0=None, timeout=0.3, rng=np.random.default_rng(0)):
    sols = []
    seeds = [seed0] if seed0 is not None else []
    seeds += [[0,-0.785,0,-2.356,0,1.571,0.785]]
    seeds += [list(rng.uniform(LO+0.3, HI-0.3)) for _ in range(n)]
    for s in seeds:
        sol = r.ik(pos, quat, seed=s, timeout=timeout)
        if sol: sols.append((margin(sol), np.array(sol)))
    if not sols: return None, []
    sols.sort(key=lambda t: -t[0])
    return sols[0][1], sols
