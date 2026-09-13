import math, numpy as np, sys
from rb import Robot, R_quat, quat_R
r = Robot("gb")
ang = math.radians(15); yh0 = np.array([math.cos(ang), -math.sin(ang), 0.0]); b = math.radians(20)
yh = math.cos(b)*yh0 + math.sin(b)*np.array([0,0,1.0]); zh = math.sin(b)*yh0 - math.cos(b)*np.array([0,0,1.0]); xh = np.cross(yh, zh)
q = R_quat(np.column_stack([xh, yh, zh]))
tcp = np.array([-0.179, 0.13, 0.92])
def go(p, sec=3, qq=None, **kw):
    qq = q if qq is None else qq
    code = r.move_pose(p, qq, seconds=sec, at_tcp=True, **kw)
    if code is None: sys.exit("move failed")
    return code
p0, q0 = r.fk()
tcp0 = p0 + 0.1034*quat_R(*q0)[:,2]
pass
r.report()
go(tcp + [0,0,0.10], 4)
r.report()
go(tcp, 3)
p,_ = r.report()
print("tcp now", (p + 0.1034*zh).round(4))
f = r.close()
if not (0.015 < f[0] < 0.032):
    print("grasp looks wrong", f); sys.exit(1)
go(tcp + [0,0,0.08], 3)
print("fingers after lift", r.fingers())
r.report()
