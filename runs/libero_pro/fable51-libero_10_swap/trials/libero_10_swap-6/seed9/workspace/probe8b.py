import math, sys, numpy as np
from arm import Arm, HOME, R_quat
from stage8 import a, n, Z, G, R_from, rotated, place_tcp, STEPS, d1, LIFT, R1
import stage8
arm = Arm("probe8b")
cur = np.array(arm.arm_q())
R = R1
qG = arm.ik_world(G, R_quat(R), seed=list(cur), tries=2)
print("G", np.round(qG, 2))
for name, P in [("pre-0.04d", G - 0.04 * d1), ("pre-0.07d", G - 0.07 * d1), ("pre-up6-0.02d", G + np.array([0, 0, 0.06]) - 0.02 * d1), ("pre-up8", G + np.array([0, 0, 0.08]))]:
    q = arm.ik_world(P, R_quat(R), seed=qG, tries=2)
    print(name, P.round(3), None if q is None else (np.round(q, 2), round(float(np.abs(np.array(q) - qG).max()), 3)))
seed = qG
chain = [("lift", LIFT, R_quat(R))]
for k in range(1, STEPS + 1):
    chain.append((f"rot{k}", LIFT, R_quat(rotated(R, -k * math.pi / 2 / STEPS))))
Rf = rotated(R, -math.pi / 2)
P = place_tcp(Rf)
chain.append(("over", np.array([P[0], P[1], LIFT[2]]), R_quat(Rf)))
chain.append(("place", P, R_quat(Rf)))
for name, xyz, quat in chain:
    s = arm.ik_world(xyz, quat, seed=seed, tries=2)
    if s is None:
        print(name, "IK FAIL", np.round(xyz, 3)); break
    print(name, np.round(xyz, 3), "dq=%.3f" % np.abs(np.array(s) - np.array(seed)).max(), np.round(s, 2))
    seed = s
