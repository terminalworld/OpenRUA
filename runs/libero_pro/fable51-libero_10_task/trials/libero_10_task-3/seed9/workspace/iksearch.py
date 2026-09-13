import numpy as np
from rob import *
r = Robot("iks")
q0 = r.arm_q()
BOT = np.array([-0.166, 0.073]); ZG = 1.04
lim = np.array(FJT["limits_rad"])
def R_side(tilt=0.0):
    Z = np.array([np.cos(tilt), 0, -np.sin(tilt)]); Y = np.array([0,1,0.]); X = np.cross(Y, Z)
    return np.stack([X, Y, Z], 1)
rng = np.random.default_rng(0)
seeds = [q0, np.array([0, 0.5, 0, -2.5, 0, 3.0, 0.8]), np.array([0, 0.9, 0, -2.2, 0, 3.1, 0.8]), np.array([0,-0.2,0,-2.4,0,2.2,0.8])]
seeds += [rng.uniform(lim[:,0]*0.8, lim[:,1]*0.8) for _ in range(25)]
for tilt in [0.0, np.radians(15)]:
    R = R_side(tilt)
    sols=[]
    for s in seeds:
        q = r.ik_tcp([BOT[0]-0.07, BOT[1], ZG], R, seed=s)
        if q is None: continue
        if any(np.abs(q-x).max()<0.05 for x in sols): continue
        sols.append(q)
    print(f"tilt {np.degrees(tilt):.0f}: {len(sols)} distinct solutions")
    for q in sols:
        # score: margin to limits, distance to q0
        marg = np.minimum(q-lim[:,0], lim[:,1]-q).min()
        print("  q", np.round(q,2), "limit margin", round(marg,2), "dist q0", round(np.abs(q-q0).max(),2))
