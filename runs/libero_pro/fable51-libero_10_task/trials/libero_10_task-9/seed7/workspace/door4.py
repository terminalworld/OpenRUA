import sys
from door2 import *
rho, press, zt = 0.19, 0.0, 1.085
cur = r.joints(); print("cur", np.round(cur,2))
for dg in [97, 81, 65, 49, 33, 17, 0]:
    tcp, R = pose(dg, rho, press, zt)
    best=None
    for k in range(6):
        seed = np.array(cur) + np.random.default_rng(k).normal(0, 0.3, 7)*(k>0)
        q = r.ik_world(hand_pose_from_tcp(tcp, R), quat_from_R(R), seed=list(seed), attempts=1, timeout=0.5)
        if q is not None:
            d = max(abs(a-b) for a,b in zip(q,cur))
            if best is None or d < best[0]: best=(d, margin(q), np.round(q,2))
    print(dg, np.round(tcp,3), best)
