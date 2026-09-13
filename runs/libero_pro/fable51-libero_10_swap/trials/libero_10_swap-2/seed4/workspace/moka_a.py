# stage A: reorient high, descend to pre-grasp
from moka_plan import *
m = Mover("moka_a")
print("wrench", wrench(m))
print("open gripper"); m.gripper(0.04)
pre_high = HAND_PRE + np.array([0, 0, 0.25])
print("-> pre_high", np.round(pre_high,3)); 
if move_interp(m, pre_high, Q_GRASP, n=8, seconds=6.0) is None: raise SystemExit("abort")
print("-> pre", np.round(HAND_PRE,3))
if move_interp(m, HAND_PRE, Q_GRASP, n=6, seconds=4.0) is None: raise SystemExit("abort")
print("wrench", wrench(m)); print("fingers", m.fingers())
