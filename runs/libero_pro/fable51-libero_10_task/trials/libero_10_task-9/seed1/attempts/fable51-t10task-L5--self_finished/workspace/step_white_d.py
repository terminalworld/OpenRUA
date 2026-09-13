"""Mug held lying (rim pinch). Re-orient so mug axis -> +y (bottom forward,
level), carry to the microwave opening, insert lying on its side, release."""
import sys
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from rob import *
from pathcheck import check, fk_links

f = np.load("mugfit2.npy"); c_mug, a_mug = f[:3], f[3:6]      # axis rim->bottom
r = Rob("white_d")
tcp0, R0 = r.fk()
e1 = R0[:, 1]                                             # finger axis (world)
print("tcp0", np.round(tcp0, 4), "mug axis", np.round(a_mug, 3), "e1", np.round(e1, 3))

# rotation Q: mug axis -> +y, finger axis -> -x (approximately)
e1p = e1 - (e1 @ a_mug) * a_mug; e1p /= np.linalg.norm(e1p)
Rs = np.column_stack([a_mug, e1p, np.cross(a_mug, e1p)])
Rt = np.column_stack([[0, 1, 0], [-1, 0, 0], [0, 0, 1]])
Q = Rt @ Rs.T
R_F = Q @ R0
print("final approach", np.round(R_F[:, 2], 3), "final finger axis", np.round(R_F[:, 1], 3))
# mug centre offset from TCP after rotation
TCP_FIT = np.array([0.0394, 0.0223, 1.1496])   # TCP when mugfit2 was measured
off = Q @ (c_mug - TCP_FIT)
print("mug centre rel tcp after rot", np.round(off, 4))

chk = lambda a, b: check(r, a, b)
X_MUG = -0.04
xt = X_MUG - off[0]
Y_IN, Z_IN = 0.30, 1.0
print("insert TCP x", round(xt, 4))
if "--dry" in sys.argv:
    rclpy.shutdown(); sys.exit()

def go(p, R, secs=4.0, iters=2):
    q = r.move_tcp_cl(np.asarray(p), R, secs=secs, check=chk, iters=iters)
    assert q is not None, f"move failed {p}"
    return q

# 1. up, 2. over to a free spot
if tcp0[2] < 1.2:
    go(tcp0 + [0, 0, 0.10], R0)
p_rot = np.array([0.0, -0.10, 1.25])
go(p_rot, R0, secs=5.0)
# 3. re-orient in steps (slerp)
rot0, rot1 = Rot.from_matrix(R0), Rot.from_matrix(R_F)
dvec = (rot0.inv() * rot1).as_rotvec()
print("total rotation deg", np.degrees(np.linalg.norm(dvec)))
for s in [0.33, 0.66, 1.0]:
    Rk = (rot0 * Rot.from_rotvec(s * dvec)).as_matrix()
    go(p_rot, Rk, secs=5.0)
print("fingers:", r.finger_gap())
# 4. pre-insert, 5. insert in steps
go([xt, 0.10, Z_IN], R_F, secs=6.0)
for y in [0.16, 0.22, Y_IN]:
    go([xt, y, Z_IN], R_F, secs=3.0)
go([xt, Y_IN, 0.985], R_F, secs=2.0)
r.gripper(0.04)
go([xt, 0.10, Z_IN], R_F, secs=4.0, iters=0)
go([xt, 0.10, 1.15], R_F, secs=3.0, iters=0)
print("DONE_D")
rclpy.shutdown()
