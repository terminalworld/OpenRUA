from lib import *
r = Robot("t0")
q = r.arm_q(); print("q", np.round(q,4), "gap", r.finger_gap())
p, R = r.fk(q); print("FK hand world", np.round(p,4)); print(np.round(R,3))
p8, R8 = r.fk(q, "panda_link8"); print("FK link8", np.round(p8,4)); print(np.round(R8,3))
# IK sanity: solve for the current pose
sol = r.ik(p, R, seed=[v+0.05 for v in q]); print("IK back:", None if sol is None else np.round(sol,4))
