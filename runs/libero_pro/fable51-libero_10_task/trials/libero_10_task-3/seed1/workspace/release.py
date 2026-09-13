import numpy as np, kin, motion
from ctl import Ctl
kin.OBST.pop("bottle_fallen", None)
kin.OBST["drawer"] = (-0.11, 0.11, 0.07, 0.24, 0.9, 0.99)
kin.OBST["handles_up"] = (-0.05, 0.06, 0.19, 0.22, 0.99, 1.11)
c = Ctl()
print("open", c.gripper(0.04))
q = np.array(c.arm_q()); p, R = kin.tcp(q)
motion.goto(c, p + [0, -0.05, 0.12], R, label="retreat", ignore=("drawer",), skip_pts=("tcp","finger+y","finger-y"))
c.node.destroy_node()
