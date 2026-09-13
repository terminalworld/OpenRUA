import math, numpy as np
from arm import Arm, HOME, R_quat
from stage8 import G, PRE, LIFT, R1, rotated, STEPS, TARGET_AXIS, TABLE
arm = Arm("probe8c")
cur = np.array(arm.arm_q())
R = R1
HI = PRE + np.array([0, 0, 0.16])
qHI = arm.ik_best(HI, R_quat(R)); print("HI", None if qHI is None else (np.round(qHI, 2), round(float(np.abs(qHI - cur).max()), 2)))
seed = qHI
Rf = rotated(R, -math.pi / 2)
P = np.array([TARGET_AXIS[0], TARGET_AXIS[1] - 0.051, TABLE + 0.123])
chain = [("pre", PRE, R_quat(R)), ("G", G, R_quat(R)), ("lift", LIFT, R_quat(R))]
chain += [(f"rot{k}", LIFT, R_quat(rotated(R, -k * math.pi / 2 / STEPS))) for k in range(1, STEPS + 1)]
chain += [("over", np.array([P[0], P[1], LIFT[2]]), R_quat(Rf)), ("place", P, R_quat(Rf)), ("up", P + [0, 0, 0.15], R_quat(Rf))]
for name, xyz, quat in chain:
    s = arm.ik_world(xyz, quat, seed=seed, tries=2)
    if s is None:
        print(name, "IK FAIL", np.round(xyz, 3)); break
    print(name, np.round(xyz, 3), "dq=%.3f" % np.abs(np.array(s) - np.array(seed)).max(), np.round(s, 2))
    seed = s
