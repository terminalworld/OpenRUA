from rlib import *
from scene import *
r = Robot("s4c")
sc = Scene(r.node)
th = np.deg2rad(30)
Rpush = hand_R([1, 0, 0], [0, -np.cos(th), -np.sin(th)])
xc, ztip = -0.11, 0.953
seed = r.arm_q()
sc.publish(objects(drawer_front=-0.085, bowl=(-0.08, -0.15), with_bowl=True))
for name, y in [("pre", 0.0), ("start_contact", -0.03), ("mid", -0.10)]:
    q = r.ik([xc, y, ztip], Rpush, seed=seed); seed = q
    tp, Rk = r.tcp(q)
    print(name, "tcp", np.round(tp, 4), "hand", np.round(r.fk(q)[0], 4), "valid", sc.check(ARM, q, 0.0), "q", np.round(q, 3))
# closed-drawer scene for the end pose
sc.publish(objects(drawer_front=-0.225, bowl=(-0.08, -0.29), with_bowl=True))
for name, y in [("end", -0.172), ("end_deeper", -0.19)]:
    q = r.ik([xc, y, ztip], Rpush, seed=seed); seed = q
    tp, Rk = r.tcp(q)
    print(name, "tcp", np.round(tp, 4), "hand", np.round(r.fk(q)[0], 4), "valid", sc.check(ARM, q, 0.0), "q", np.round(q, 3))
