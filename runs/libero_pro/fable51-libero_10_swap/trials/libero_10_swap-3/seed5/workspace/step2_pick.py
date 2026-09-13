from rlib import *
from scene import *
r = Robot("s2")
sc = Scene(r.node)
sc.publish(objects(with_bowl=False), remove=["bowl"])
xb, yb = -0.138, 0.063
OFF = 0.047
R = top_down_R(0.0)
pre = [xb - OFF, yb, 1.05]
grasp = [xb - OFF, yb, 0.925]
r.report()
if r.finger() < 0.035:
    r.gripper(0.04)
q_pre = r.ik(pre, R)
print("pre valid", sc.check(ARM, q_pre, 0.04))
r.move_q(q_pre); r.report()
pts, frac = r.cartesian([(grasp, R)])
print("descend frac", frac, "final valid", sc.check(ARM, pts[-1], 0.04))
w0 = r.wrench()
r.move_q(pts); r.report()
print("wrench delta", np.round(r.wrench() - w0, 2))
r.gripper(0.0)
r.spin(1.0)
print("finger after close", r.finger())
r.move_cart([(pre, R)]); r.report()
print("finger after lift", r.finger(), "wrench", np.round(r.wrench(), 2))
