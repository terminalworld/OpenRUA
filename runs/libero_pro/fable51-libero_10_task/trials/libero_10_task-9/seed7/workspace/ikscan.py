import sys
from cart import *
r = Robot()
Rf = np.array([[-1,0,0],[0,0,1],[0,1,0]], float)
tcp = np.array([float(v) for v in sys.argv[1:4]])
hand = hand_pose_from_tcp(tcp, Rf); quat = quat_from_R(Rf)
lim = np.array(FJT["limits_rad"])
rng = np.random.default_rng(0)
sols = []
for i in range(25):
    seed = rng.uniform(lim[:,0], lim[:,1])
    q = r.ik_world(hand, quat, seed=seed, attempts=1, timeout=1.0)
    if q is not None:
        q = np.round(q, 2)
        if not any(np.allclose(q, s, atol=0.05) for s in sols):
            sols.append(q); print(q, "margin j7", round(2.9-abs(q[6]),2))
print(len(sols), "distinct")
