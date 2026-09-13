import numpy as np, rclpy
from ctl import *
c = Ctl()
q0 = np.array(c.arm_q())
pos_fk_raw, R = c.hand_pose(); pos_fk_raw = pos_fk_raw - BASE_IN_WORLD
print("FK raw:", pos_fk_raw.round(4))
for label, p in [("raw(FK frame)", pos_fk_raw), ("raw - base", pos_fk_raw - BASE_IN_WORLD), ("raw + base", pos_fk_raw + BASE_IN_WORLD)]:
    try:
        # bypass BASE subtraction: pass p + BASE so solve_ik subtracts it back
        q = c.solve_ik(p + BASE_IN_WORLD, R, seed=q0, tries=1)
        print(label, "-> sol err", np.abs(np.array(q)-q0).max().round(4))
    except Exception as e:
        print(label, "->", e)
rclpy.shutdown()
