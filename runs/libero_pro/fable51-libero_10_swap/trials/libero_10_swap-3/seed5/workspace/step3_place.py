from rlib import *
from scene import *
r = Robot("s3")
sc = Scene(r.node)
th = np.deg2rad(25)
Rp = hand_R([1, 0, 0], [0, -np.sin(th), -np.cos(th)])
xh, yp = -0.14, -0.155
pre = [xh, yp, 1.15]
place = [xh, yp, 0.975]
r.report()
q_pre = r.ik(pre, Rp)
print("pre valid", sc.check(ARM, q_pre, 0.004))
tp, Rk = r.tcp(q_pre); print("pre tcp", np.round(tp, 4), "Z", np.round(Rk[:, 2], 3), "Y", np.round(Rk[:, 1], 3))
r.move_q(q_pre); r.report()
print("finger", r.finger())
pts, frac = r.cartesian([(place, Rp)])
print("descend frac", frac, "final valid", sc.check(ARM, pts[-1], 0.004))
r.move_q(pts); r.report()
print("finger", r.finger(), "wrench", np.round(r.wrench(), 2))
r.gripper(0.04)
r.spin(1.0)
r.move_cart([(pre, Rp)]); r.report()
