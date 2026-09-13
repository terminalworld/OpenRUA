"""White mug: grasp handle bar and lift. Stops after lift for visual check."""
import numpy as np
from rob import *
from plan import R_G

HX, HY, HZ = -0.124, -0.172, 0.945
grasp = np.array([HX, HY, HZ])
pregrasp = grasp - 0.09 * R_G[:, 2]
lift = np.array([HX, HY, 1.13])

r = Rob("white_a")
q0, _ = r.joints()
print("start joints", np.round(q0, 3).tolist())
print("start tcp", np.round(r.fk()[0], 4).tolist(), "fingers", r.finger_gap())

r.gripper(0.04)
q = r.move_tcp(pregrasp, R_G, secs=8.0)
assert q is not None
q = r.move_tcp(grasp, R_G, secs=3.0, seed=q)
assert q is not None
gap = r.gripper(0.0)
print("GAP after close:", gap)
q = r.move_tcp(lift, R_G, secs=3.0, seed=q)
print("fingers after lift:", r.finger_gap())
print("DONE_A")
rclpy.shutdown()
