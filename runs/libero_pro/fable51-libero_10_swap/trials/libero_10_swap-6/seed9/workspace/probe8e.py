import math, numpy as np
from arm import Arm, HOME, R_quat
from stage8 import G, PRE, LIFT, R1, R1b, rotated, STEPS, TARGET_AXIS, TABLE
arm = Arm("probe8e")
cur = np.array(arm.arm_q())
for name0, R in [("R1b", R1b), ("R1", R1)]:
    print("=== ", name0)
    qPRE = arm.ik_best(PRE, R_quat(R), n=8); 
    if qPRE is None: print("PRE fail"); continue
    print("PRE", np.round(qPRE, 2))
    HI = PRE + np.array([0, 0, 0.27])
    qHI = arm.ik_world(HI, R_quat(R), seed=qPRE, tries=2); print("HI", None if qHI is None else (np.round(qHI, 2), "dq from PRE %.2f" % np.abs(np.array(qHI) - np.array(qPRE)).max(), "dq from cur %.2f" % np.abs(np.array(qHI) - cur).max()))
    seed = qPRE
    Rf = rotated(R, -math.pi / 2)
    P = np.array([TARGET_AXIS[0], TARGET_AXIS[1] + 0.05, TABLE + 0.133])
    chain = [("G", G, R_quat(R)), ("lift", LIFT, R_quat(R))]
    chain += [(f"rot{k}", LIFT, R_quat(rotated(R, -k * math.pi / 2 / STEPS))) for k in range(1, STEPS + 1)]
    chain += [("over", np.array([P[0], P[1], LIFT[2]]), R_quat(Rf)), ("place", P, R_quat(Rf)), ("up", P + [0, 0, 0.15], R_quat(Rf))]
    for name, xyz, quat in chain:
        s = arm.ik_world(xyz, quat, seed=seed, tries=2)
        if s is None:
            print(name, "IK FAIL", np.round(xyz, 3)); break
        print(name, np.round(xyz, 3), "dq=%.3f" % np.abs(np.array(s) - np.array(seed)).max(), np.round(s, 2))
        seed = s
