import numpy as np, math
from arm import Arm, Q_DOWN_FX, tilted, TCP, quat_R, line_ik
a = Arm()
q = tilted(Q_DOWN_FX, [1,0,0], -np.radians(35))
R = quat_R(q)
def tcp():
    p, _ = a.hand_pose(); return p + TCP*R[:,2]
print("fingers", a.fingers(), "tcp", np.round(tcp(),3))
# approach above then in front of the drawer
for pos, sec in [((-0.16, -0.12, 1.10), 5), ((-0.16, -0.15, 0.97), 4)]:
    _, code, err = a.move_tcp(pos, q, seconds=sec)
    print("to", pos, "code", code, "err", round(err,4), "tcp", np.round(tcp(),3), "F", np.round(a.force(),2))
# push in small steps, watch progress
for y in [-0.19, -0.205, -0.215, -0.225]:
    wps = line_ik(a, tcp(), (-0.16, y, 0.97), q, steps=2, max_jump=0.5)
    code, err = a.move_joints(wps, 3)
    t = tcp(); F = a.force()
    print("push to", y, "code", code, "lag", round(err,4), "tcp", np.round(t,3), "F", np.round(F,2))
    if t[1] > y + 0.004:  # not following -> at the stop
        print("stopped short: drawer at its limit"); break
# retract
_, code, err = a.move_tcp((-0.16, -0.12, 1.10), q, seconds=4)
_, code, err = a.move_tcp((-0.10, 0.10, 1.20), q, seconds=5)
print("parked tcp", np.round(tcp(),3), "code", code, "err", round(err,4))
a.close()
