"""White mug lying on the table, rim fit in rimfit.npy (centre, normal, r).
Pinch the rim wall at its +x side point, fingers along the horizontal
tangent, approach along the mug axis into the opening. Lift and stop."""
import sys
import numpy as np
from rob import *
from pathcheck import check, fk_links

f = np.load("rimfit.npy"); cen, n, rr = f[:3], f[3:6], f[6]
n /= np.linalg.norm(n)
e1 = np.cross(n, [0, 0, 1]); e1 /= np.linalg.norm(e1)
if e1[0] < 0: e1 = -e1
e2 = np.cross(n, e1)                       # down-ish, in the rim plane
TH = np.radians(float(sys.argv[sys.argv.index("--th") + 1]) if "--th" in sys.argv else 30.0)
APP = -n * np.cos(TH) + e2 * np.sin(TH)     # approach tilted TH from mug axis, about e1
R_S = hand_R(APP, e1)
pinch = cen + (rr - 0.002) * e1
grasp = pinch + 0.02 * APP
pregrasp = grasp - 0.06 * APP
lift = grasp + np.array([0, 0, 0.20])
print("rim centre", np.round(cen, 4), "normal", np.round(n, 3), "e1", np.round(e1, 3))
print("pregrasp", np.round(pregrasp, 4), "grasp", np.round(grasp, 4))
hand = grasp - TCP_OFF * APP
print("hand origin at grasp", np.round(hand, 4), " palm low z ~", round(hand[2] - 0.058 * abs(APP[2]) - 0.03, 3))

r = Rob("white_c")
q0, _ = r.joints()
for name, p in [("pregrasp", pregrasp), ("grasp", grasp)]:
    q, d = r.ik_near(p, R_S, seed=q0)
    print(name, "IK", None if q is None else np.round(q, 3).tolist(), d)
    if q is not None:
        L = fk_links(r, q)
        print("   " + " ".join(f"{k[-5:]}={np.round(v,3).tolist()}" for k, v in L.items()))
if "--dry" in sys.argv:
    rclpy.shutdown(); sys.exit()

r.gripper(0.04)
chk = lambda a, b: check(r, a, b)
q = r.move_tcp_cl(pregrasp, R_S, secs=8.0, check=chk)
assert q is not None
q = r.move_tcp_cl(grasp, R_S, secs=3.0, check=chk)
assert q is not None
gap = r.gripper(0.0)
print("GAP after close:", gap)
q = r.move_tcp_cl(lift, R_S, secs=4.0, check=chk, iters=1)
print("fingers after lift:", r.finger_gap())
print("DONE_C")
rclpy.shutdown()
