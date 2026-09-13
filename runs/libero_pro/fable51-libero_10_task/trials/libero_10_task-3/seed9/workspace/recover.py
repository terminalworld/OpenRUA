import numpy as np
from rob import *
r = Robot("rec")
q0 = r.arm_q(); pos, R = r.fk_hand()
best=None
for dz in [0.15, 0.12, 0.10]:
    for _ in range(5):
        q = r.ik_hand(pos + [0,0,dz], R, seed=q0)
        if q is not None:
            d = np.abs(q-q0).max(); print("dz",dz,"joint dist",round(d,3))
            if d < 1.0: best=q; break
    if best is not None: break
if best is None: raise SystemExit("no close IK")
r.move_q(best, 4.0)
t,_=r.tcp(); print("tcp", np.round(t,3), "force", np.round(r.force(),2))
