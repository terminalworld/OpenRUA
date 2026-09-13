import sys
from door2 import *
rho, press, zt = 0.19, 0.0, 1.085
rng = np.random.default_rng(5)
tcp, R = pose(97, rho, press, zt); hand = hand_pose_from_tcp(tcp, R)
sols = []
base = np.array([0.1, 0.3, 0.0, -2.5, 0.0, 2.8, 0.8])
for i in range(40):
    seed = np.clip(base + rng.normal(0, 0.6, 7), lim[:,0]+0.05, lim[:,1]-0.05)
    q = r.ik_world(hand, quat_from_R(R), seed=seed, attempts=1, timeout=0.5)
    if q is not None and q[1] > 0: sols.append((margin(q), np.round(q,2)))
sols.sort(key=lambda s: -s[0])
for s in sols[:6]: print(s)
if sols:
    seed = list(sols[0][1])
    for dg in list(np.arange(97, 0, -8)) + [0]:
        tcp, R = pose(dg, rho, press, zt); q = r.ik_world(hand_pose_from_tcp(tcp, R), quat_from_R(R), seed=seed, attempts=3)
        if q is None: print("fail", dg); break
        print(f"th={dg:5.1f} tcp={np.round(tcp,3)} jump={max(abs(a-b) for a,b in zip(q,seed)):.3f} margin={margin(q):.2f} q={np.round(q,2)}"); seed = q
