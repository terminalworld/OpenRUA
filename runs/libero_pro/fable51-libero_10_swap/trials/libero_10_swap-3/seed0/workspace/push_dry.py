from robot import *
r = Robot("pushdry")
Qp = (0.5,-0.5,0.5,0.5)
qb = np.load("qb.npy"); seed = list(qb)
path = [(-0.19,0.05,1.10)]
for z in np.arange(1.08, 0.96, -0.02): path.append((-0.19, 0.05, z))
path.append((-0.19,0.05,0.965))
for y in np.arange(0.03, -0.235, -0.02): path.append((-0.19, y, 0.965))
qs=[]
for p in path:
    q = r.ik_tcp(p, Qp, seed)
    if q is None: print("IK FAIL", p); break
    if np.abs(np.array(q)-np.array(seed)).max()>1.0: print("JUMP", p, np.round(q,3)); break
    qs.append(q); seed=q
    print(np.round(p,3), np.round(q,3))
