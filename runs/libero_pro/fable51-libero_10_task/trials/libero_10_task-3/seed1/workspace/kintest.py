import numpy as np, kin
from ctl import Ctl, fk_links
c = Ctl()
q = np.array(c.arm_q()); print("q", q.round(3))
p, R = c.hand_pose(q); pl, Rl = kin.hand(q)
print("hand svc", p.round(4), "local", pl.round(4), "rot diff deg", np.degrees(np.arccos(np.clip((np.trace(R.as_matrix().T@Rl)-1)/2,-1,1))).round(2))
svc = fk_links(c, q)
for k,v in kin.fk(q).items():
    if k in svc: print(k, np.round(v[0]-svc[k],4))
# IK round trip: lift hand 15 cm
q2 = kin.ik(pl + [0,0,0.15], Rl, q, use_tcp=False)
print("q2", q2.round(3), "diff", np.abs(q2-q).max().round(3))
print(kin.hand(q2)[0].round(4))
c.node.destroy_node()
