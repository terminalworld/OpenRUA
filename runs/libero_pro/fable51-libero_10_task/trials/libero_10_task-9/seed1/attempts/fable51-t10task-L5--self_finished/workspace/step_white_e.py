"""Mug held; hand needs a -118 deg roll about its z which j7 cannot do
directly -> wrap j7 by +242 deg (slow). Then insert lying into the cavity."""
import sys
import numpy as np
from rob import *
from pathcheck import check

R_F = np.array(hand_R([-0.163, 0.898, -0.409], [-0.984, -0.179, 0]))
xt, Y_IN, Z_IN = -0.0723, 0.30, 1.0
r = Rob("white_e")
chk = lambda a, b: check(r, a, b)

def go(p, R, secs=4.0, iters=2):
    q = r.move_tcp_cl(np.asarray(p), R, secs=secs, check=chk, iters=iters)
    assert q is not None, f"move failed {p}"
    return q

q, _ = r.joints()
tcp, R = r.fk()
ang = np.degrees(np.arccos(np.clip((np.trace(R_F @ R.T) - 1) / 2, -1, 1)))
print("start tcp", np.round(tcp, 4), "rot err to R_F", round(ang, 1), "fingers", r.finger_gap())
if ang > 20:
    q2 = q.copy(); q2[6] = q[6] + (-2.06 + 2 * np.pi)
    assert -2.9 < q2[6] < 2.9
    print("wrapping j7 to", round(q2[6], 3))
    r.movej([q2], [20.0])
    tcp, R = r.fk()
    ang = np.degrees(np.arccos(np.clip((np.trace(R_F @ R.T) - 1) / 2, -1, 1)))
    print("after wrap tcp", np.round(tcp, 4), "rot err", round(ang, 1), "fingers", r.finger_gap())
if "--wrap-only" in sys.argv:
    rclpy.shutdown(); sys.exit()

go([xt, 0.10, Z_IN], R_F, secs=6.0)
print("fingers:", r.finger_gap())
if "--pre-only" in sys.argv:
    rclpy.shutdown(); sys.exit()
for y in [0.16, 0.22, Y_IN]:
    go([xt, y, Z_IN], R_F, secs=3.0)
go([xt, Y_IN, 0.985], R_F, secs=2.0)
r.gripper(0.04)
go([xt, 0.10, Z_IN], R_F, secs=4.0, iters=0)
go([xt, 0.10, 1.15], R_F, secs=3.0, iters=0)
print("DONE_E")
rclpy.shutdown()
