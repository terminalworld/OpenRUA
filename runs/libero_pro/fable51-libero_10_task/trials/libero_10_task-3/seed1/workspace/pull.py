import sys, numpy as np, kin, motion
from ctl import Ctl
c = Ctl()
th = np.radians(20)
R = motion.R_from_axes([0, np.sin(th), -np.cos(th)], [-1, 0, 0])
print("R cols X,Y,Z:", R[:,0].round(3), R[:,1].round(3), R[:,2].round(3))
X0 = 0.0
pre   = np.array([X0, 0.18, 1.12])
down  = np.array([X0, 0.18, 0.955])
pulled= np.array([X0, 0.12, 0.955])
up    = np.array([X0, 0.12, 1.12])
q = np.array(c.arm_q())
paths = []
for name, tgt, ign in [("pre", pre, ()), ("down", down, ("drawer",)), ("pull", pulled, ("drawer",)), ("up", up, ("drawer",))]:
    qs = motion.cart_path(q, tgt, R)
    hits = motion.check(qs, q, ignore=ign, skip_pts=("tcp","finger+y","finger-y") if ign else ())
    print(f"{name}: {len(qs)} wps, max dq {np.abs(qs[-1]-q).max():.2f}, hits {len(hits)} {hits[:2]}")
    print("   q_end", qs[-1].round(2), "palm pts:", [ (k, (kin.hand(qs[-1])[0] + kin.hand(qs[-1])[1] @ np.array(o)).round(3)) for k,o in kin.HAND_PTS[:2]])
    paths.append((name, qs, ign)); q = qs[-1]
if "--go" in sys.argv:
    print("fingers", c.gripper(0.0))
    for name, qs, ign in paths:
        motion.execute(c, qs, ignore=ign, skip_pts=("tcp","finger+y","finger-y") if ign else (), label=name)
c.node.destroy_node()
