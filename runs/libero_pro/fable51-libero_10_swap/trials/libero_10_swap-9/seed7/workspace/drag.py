import sys, numpy as np
from ctl import Robot, quat_down, log
r = Robot("dg")
Q = quat_down(90)  # fingers along world x
XH, YH, ZG = -0.055, 0.195, 1.005
def ik(p, seed): 
    s = r.ik(p, Q, seed=seed, at_tcp=True)
    if s is None: sys.exit(f"IK fail {p}")
    return s
seed = r.joints()
r.gripper(0.04)
s1 = ik((XH, YH, 1.15), seed); s2 = ik((XH, YH, 1.04), s1); s3 = ik((XH, YH, ZG), s2)
r.move_joints([s1, s2, s3], [5.0, 8.0, 10.0])
f = r.gripper(0.0)
if not (0.005 < f[0] < 0.015): sys.exit(f"bad grasp {f}")
seed = s3
wps = [ik((XH, y, ZG), seed) for y in (0.10, 0.0)]
r.move_joints(wps, [5.0, 10.0])
log("hand", np.round(r.hand_pose()[0],3), "fingers", r.fingers())
r.gripper(0.04)
s = ik((XH, 0.0, 1.15), r.joints()); r.move_joints([s],[4.0])
