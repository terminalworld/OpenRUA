import numpy as np, kin, sys
from ctl import Ctl
c = Ctl()
q0 = np.array(c.arm_q())
q_home = np.array([0.0, -0.161, 0.0, -2.4446, 0.0, 2.2268, 0.7854])
ph, Rh = kin.hand(q0)
stages = [q0]
# stage 1: lift 15 cm
q1 = kin.ik(ph + [0,0,0.15], Rh, q0, use_tcp=False); stages.append(q1)
# stage 2: move hand toward home hand pose orientation gradually? just check direct to home
stages.append(q_home)
ok = True
for a, b in zip(stages, stages[1:]):
    z, k, (t, p) = kin.path_min_z(a, b, extra=kin.HAND_PTS)
    print(f"{a.round(2)} -> {b.round(2)}\n   min z {z:.3f} at {k} t={t:.2f} p={p.round(3)}")
    ok &= z > 0.99
print("all ok", ok)
if ok and "--go" in sys.argv:
    for b in stages[1:]:
        print("moving to", b.round(3), c.movej(list(b), 5.0))
        print("now", np.array(c.arm_q()).round(3))
c.node.destroy_node()
