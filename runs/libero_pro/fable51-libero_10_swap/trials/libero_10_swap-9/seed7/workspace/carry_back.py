import sys, numpy as np
from ctl import Robot, quat_from_axes, log
from geom import a_of
def Q(th): return quat_from_axes(a_of(th), [1.0, 0, 0])
r = Robot("cb")
seed = r.joints()
steps = [((-0.13, -0.44, 1.28), 65), ((-0.13, -0.30, 1.28), 65), ((-0.05, -0.05, 1.28), 65), ((-0.05, 0.13, 1.28), 55), ((-0.05, 0.13, 1.15), 45)]
wps = []
for p, th in steps:
    s = r.ik(p, Q(th), seed=seed, at_tcp=True)
    log(p, th, None if s is None else np.round(s, 2), None if s is None else round(float(np.abs(np.array(s)-np.array(seed)).max()),2))
    if s is None: sys.exit("IK fail")
    wps.append(s); seed = s
if "dry" in sys.argv: sys.exit(0)
r.move_joints(wps, [5.0, 9.0, 14.0, 18.0, 22.0])
log("hand", np.round(r.hand_pose()[0], 3), "fingers", r.fingers())
