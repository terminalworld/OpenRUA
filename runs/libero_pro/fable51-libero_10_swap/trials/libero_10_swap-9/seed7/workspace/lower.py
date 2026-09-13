import sys, numpy as np
from ctl import Robot, quat_from_axes, log
from geom import a_of
def Q(th): return quat_from_axes(a_of(th), [1.0, 0, 0])
r = Robot("lw")
seed = r.joints()
zs = [float(v) for v in sys.argv[1:]]
wps=[]
for z in zs:
    s = r.ik((-0.05, 0.13, z), Q(45), seed=seed, at_tcp=True); assert s is not None
    wps.append(s); seed=s
r.move_joints(wps, [3.0*(i+1) for i in range(len(wps))])
log("hand", np.round(r.hand_pose()[0], 3), "fingers", r.fingers())
