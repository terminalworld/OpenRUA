import numpy as np, kin, sys
from ctl import Ctl
c = Ctl()
q0 = np.array(c.arm_q())
q_home = np.array([0.0, -0.161, 0.0, -2.4446, 0.0, 2.2268, 0.7854])
ph, Rh = kin.hand(q0)
q1 = kin.ik(ph + [0,0,0.15], Rh, q0, use_tcp=False)
# lift further to 1.40 and level the hand (top-down) keeping near q1
Rdown = kin.Rot.from_euler("xyz", [np.pi, 0, 0]).as_matrix()  # Z down, X along +x
cands = {}
for z in (1.35, 1.40):
    for yaw in (0, 45, 90, 135, 180, -45, -90, -135):
        R = kin.Rot.from_euler("z", np.radians(yaw)).as_matrix() @ Rdown
        try:
            q2 = kin.ik(ph + [0,0,z-ph[2]], R, q1, use_tcp=False, null_bias=0.1)
        except RuntimeError as e:
            continue
        cands[(z,yaw)] = q2
print(len(cands), "candidates")
best = None
for key, q2 in cands.items():
    q_mid = q_home.copy(); q_mid[0] = q2[0]
    st = [q0, q1, q2, q_mid, q_home]
    zs = [kin.path_min_z(a, b, extra=kin.HAND_PTS)[0] for a, b in zip(st, st[1:])]
    m = min(zs)
    print(key, np.round(zs,3), q2.round(2))
    if best is None or m > best[0]: best = (m, key, st)
print("best", best[0], best[1])
if best[0] > 0.99 and "--go" in sys.argv:
    for b in best[2][1:]:
        print("moving to", np.round(b,3), c.movej(list(b), 5.0))
        print("now", np.array(c.arm_q()).round(3))
c.node.destroy_node()
