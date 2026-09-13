import sys
from door2 import *
rho, press, zt = 0.19, 0.0, 1.085
rng = np.random.default_rng(7)
dg0 = float(sys.argv[1]) if len(sys.argv)>1 else 97
tcp, R = pose(dg0, rho, press, zt); hand = hand_pose_from_tcp(tcp, R)
sols=[]
for base in ([0.1,0.3,0.0,-2.7,0.0,3.0,0.8],[0.1,0.1,0.0,-2.9,0.0,3.1,0.8],[0.0,0.6,0.0,-2.4,0.0,3.0,0.8],[-0.3,0.2,0.3,-2.8,0.0,3.0,0.0]):
    for i in range(15):
        seed = np.clip(np.array(base) + rng.normal(0, 0.3, 7)*(i>0), lim[:,0]+0.05, lim[:,1]-0.05)
        q = r.ik_world(hand, quat_from_R(R), seed=list(seed), attempts=1, timeout=0.5)
        if q is not None and abs(q[0])<1.0 and abs(q[2])<1.0: sols.append((round(margin(q),2), np.round(q,2)))
sols.sort(key=lambda s:-s[0])
for s in sols[:5]: print(s)
print("n", len(sols))
