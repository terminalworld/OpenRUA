import sys, numpy as np, kin, motion
from ctl import Ctl
c = Ctl()
ax = np.array([0.559, -0.829, 0.0])          # bottle axis (neck toward -ax)
perp = np.array([0.829, 0.559, 0.0])
center = np.array([-0.288, 0.007])
q = np.array(c.arm_q())
best = None
for sgn in (1, -1):
    R = motion.R_from_axes([0,0,-1], sgn*perp)
    try:
        qs = motion.cart_path(q, [center[0], center[1], 1.05], R)
    except RuntimeError as e:
        print("ik fail", sgn, e); continue
    hits = motion.check(qs, q)
    d = np.abs(qs[-1]-q).max()
    print(f"sgn {sgn}: max dq {d:.2f} hits {len(hits)} X={R[:,0].round(2)} q_end {qs[-1].round(2)}")
    if not hits and (best is None or d < best[0]): best = (d, sgn, R, qs)
d, sgn, R, qs_pre = best
q1 = qs_pre[-1]
qs_down = motion.cart_path(q1, [center[0], center[1], 0.913], R)
skip = ("tcp","finger+y","finger-y")
print("down hits", motion.check(qs_down, q1, ignore=("bottle_fallen",), skip_pts=skip)[:3])
q2 = qs_down[-1]
qs_up = motion.cart_path(q2, [center[0], center[1], 1.10], R)
print("up hits", motion.check(qs_up, q2, ignore=("bottle_fallen",), skip_pts=skip)[:3])
if "--go" in sys.argv:
    print("open", c.gripper(0.04))
    motion.execute(c, qs_pre, label="pre")
    motion.execute(c, qs_down, ignore=("bottle_fallen",), skip_pts=skip, label="down")
    f = c.gripper(0.0); print("close ->", f)
    motion.execute(c, qs_up, ignore=("bottle_fallen",), skip_pts=skip, label="up")
    print("fingers after lift", c.fingers())
c.node.destroy_node()
