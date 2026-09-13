import numpy as np, kin
from ctl import Ctl
c = Ctl()
q0 = np.array(c.arm_q())
q_home = np.array([0.0, -0.161, 0.0, -2.4446, 0.0, 2.2268, 0.7854])
h = kin.collisions(q0, q_home); print("hits", h[:3])
if not h:
    secs = max(3.0, np.abs(q_home - q0).max() / 0.15)
    print("secs", round(secs,1), c.movej(list(q_home), secs))
    print("now", np.array(c.arm_q()).round(3))
c.node.destroy_node()
