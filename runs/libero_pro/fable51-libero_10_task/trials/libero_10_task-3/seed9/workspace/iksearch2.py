import numpy as np
from rob import *
r = Robot("iks2")
q0 = r.arm_q()
BOT = np.array([-0.166, 0.073]); ZG = 1.04
lim = np.array(FJT["limits_rad"])
def R_approach(az, tilt):
    Z = np.array([np.cos(az)*np.cos(tilt), np.sin(az)*np.cos(tilt), -np.sin(tilt)])
    Y = np.array([-np.sin(az), np.cos(az), 0.0]); X = np.cross(Y, Z)
    return np.stack([X, Y, Z], 1)
rng = np.random.default_rng(1)
seeds = [q0] + [rng.uniform(lim[:,0]*0.8, lim[:,1]*0.8) for _ in range(30)]
for az in [np.radians(45), np.radians(30), np.radians(60)]:
  for tilt in [0.0, np.radians(10)]:
    R = R_approach(az, tilt)
    sols=[]
    for s in seeds:
        q = r.ik_tcp([BOT[0]-0.07*np.cos(az), BOT[1]-0.07*np.sin(az), ZG], R, seed=s)
        if q is None or any(np.abs(q-x).max()<0.05 for x in sols): continue
        sols.append(q)
    good=[]
    for q in sols:
        marg = np.minimum(q-lim[:,0], lim[:,1]-q).min()
        if marg > 0.2:
            # check grasp & lift reachable from this branch
            qg = r.ik_tcp([BOT[0], BOT[1], ZG], R, seed=q); ql = None if qg is None else r.ik_tcp([BOT[0], BOT[1], ZG+0.11], R, seed=qg)
            ok = qg is not None and ql is not None and np.abs(qg-q).max()<0.6 and np.abs(ql-qg).max()<0.8
            good.append((marg, q, ok))
    print(f"az {np.degrees(az):.0f} tilt {np.degrees(tilt):.0f}: {len(sols)} sols, {len(good)} with margin>0.2")
    for marg,q,ok in sorted(good, key=lambda t:-t[0])[:4]:
        print("   margin", round(marg,2), "chain ok", ok, "q", np.round(q,2))
