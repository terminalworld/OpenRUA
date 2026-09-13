import numpy as np, kin, motion
from ctl import Ctl
kin.OBST.pop("bottle_fallen", None); kin.OBST.pop("drawer", None)
kin.OBST["handles_up"] = (-0.05, 0.06, 0.19, 0.22, 0.99, 1.11)
c = Ctl()
q_home = np.array([0.0, -0.161, 0.0, -2.4446, 0.0, 2.2268, 0.7854])
q = np.array(c.arm_q())
# lift first, then joint move home
p, R = kin.tcp(q)
motion.goto(c, [p[0], p[1]-0.08, 1.25], R, label="lift", ignore=("handles_up",), skip_pts=("tcp","finger+y","finger-y"))
motion.gotoj(c, q_home, label="home")
print("final joints", np.array(c.arm_q()).round(3), "fingers", c.fingers())
c.node.destroy_node()
