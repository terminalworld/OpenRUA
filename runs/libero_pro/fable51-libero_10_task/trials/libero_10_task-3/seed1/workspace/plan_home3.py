import numpy as np, kin, sys
from ctl import Ctl
c = Ctl()
q0 = np.array(c.arm_q())
q_home = np.array([0.0, -0.161, 0.0, -2.4446, 0.0, 2.2268, 0.7854])
ph, Rh = kin.hand(q0)
q1 = kin.ik(ph + [0,0,0.15], Rh, q0, use_tcp=False)
Rdown = kin.Rot.from_euler("xyz", [np.pi, 0, 0]).as_matrix()
R = kin.Rot.from_euler("z", np.radians(135)).as_matrix() @ Rdown
q2 = kin.ik(ph + [0,0,1.35-ph[2]], R, q1, use_tcp=False, null_bias=0.1)
q_mid = q_home.copy(); q_mid[0] = q2[0]
st = [q0, q1, q2, q_mid, q_home]
allok = True
for a, b in zip(st, st[1:]):
    h = kin.collisions(a, b)
    print(np.round(b,2), "hits:", len(h), h[:3])
    allok &= not h
print("home hand", kin.hand(q_home)[0].round(3))
print("all ok", allok)
if allok and "--go" in sys.argv:
    for b in st[1:]:
        print("moving to", np.round(b,3), c.movej(list(b), 5.0))
        print("now", np.array(c.arm_q()).round(3))
c.node.destroy_node()
