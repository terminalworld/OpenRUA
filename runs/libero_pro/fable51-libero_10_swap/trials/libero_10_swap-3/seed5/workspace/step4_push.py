from rlib import *
from scene import *
r = Robot("s4")
th = np.deg2rad(30)
Rpush = hand_R([1, 0, 0], [0, -np.cos(th), -np.sin(th)])
xc, ztip = -0.11, 0.953
r.report()
r.gripper(0.0)
q_hi = r.ik([xc, 0.0, 1.10], Rpush)
r.move_q(q_hi); r.report()
r.move_cart([([xc, 0.0, ztip], Rpush)]); r.report()
w0 = r.wrench(); print("wrench0", np.round(w0, 2))
r.move_cart([([xc, -0.10, ztip], Rpush)]); r.report()
print("wrench", np.round(r.wrench() - w0, 2))
r.move_cart([([xc, -0.172, ztip], Rpush)]); r.report()
print("wrench", np.round(r.wrench() - w0, 2))
# retreat
r.move_cart([([xc, -0.10, ztip + 0.02], Rpush), ([xc, 0.0, 1.10], Rpush)]); r.report()
