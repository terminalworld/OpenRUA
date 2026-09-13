import math, numpy as np, sys
from rb import Robot, R_quat, quat_R
r = Robot("place")
def quat_from_xh(yaw_deg):
    xh = np.array([math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg)), 0.0])
    zh = np.array([0,0,-1.0]); yh = np.cross(zh, xh)
    return R_quat(np.column_stack([xh, yh, zh]))
def go(p, qq, sec=3, via=None, **kw):
    code = r.move_pose(p, qq, seconds=sec, at_tcp=True, via=via, **kw)
    if code is None: sys.exit("move failed")
    print("  fingers", np.round(r.fingers(),4))
    return code
p0, q0 = r.fk()
go([-0.18, 0.05, 1.15], q0, 4)
go([-0.18, 0.05, 1.15], quat_from_xh(75), 4)
for yaw in (20, -35, -90):
    go([-0.18, 0.05, 1.15], quat_from_xh(yaw), 5)
r.report()
np.save("q_place.npy", np.array(r.arm_q()))
