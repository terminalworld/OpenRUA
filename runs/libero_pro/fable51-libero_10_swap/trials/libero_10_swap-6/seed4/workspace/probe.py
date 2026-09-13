from arm import *
a = Arm()
print("joints", np.round(a.arm_q(),4))
print("fk", a.fk_hand())
q = a.solve_ik([0.10,0,0.60],[1,0,0,0])
print("ik sol for (0.10,0,0.60):", None if q is None else np.round(q,4))
