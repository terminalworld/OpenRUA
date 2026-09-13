import numpy as np
from rob import *
from goto import ik_checked, margin
from scene import check, R_for
r = Robot("rd"); q0 = r.arm_q(); t0, R0 = r.tcp()
Rp = np.load("snaps/R_place.npy")
print("q0", np.round(q0,2))
for t in [0.25,0.5,0.75,1.0]:
    R = slerp_R(R0, Rp, t)
    for md in [0.6, 1.5, 4.0]:
        q = ik_checked(r, t0, R, q0, max_dist=md, tries=8)
        if q is not None:
            print(f"t={t} md={md} dq={np.round(q-q0,2)} margin={margin(q):.2f} clear={check(r,q)[0]:.3f}"); break
    else: print(f"t={t} no IK")
