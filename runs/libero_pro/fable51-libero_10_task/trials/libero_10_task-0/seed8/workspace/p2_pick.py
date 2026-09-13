"""Pick one object and drop it in the basket.  Usage: p2_pick.py cc|tomato"""
import sys
import numpy as np
from rb import RB, q_down_yaw

OBJ = {
    # name: (center xy, grasp tcp z, yaw deg, pregrasp z)
    # cc: box spans x[0.068,0.148]; grasp toward +x end and yaw -10 so the
    # palm's +y end stays clear of the milk carton at (0.06,-0.11)
    "cc":     ((0.125, -0.209), 0.440, -10.0, 0.60),
    "tomato": ((-0.127, 0.046), 0.478, 0.0, 0.65),
}
BASKET = (-0.01, 0.25)
DROP_Z = 0.75

(cx, cy), gz, yaw, pz = OBJ[sys.argv[1]]
r = RB("p2")
q = q_down_yaw(yaw)

f = r.fingers()
if f[0] < 0.035:
    r.gripper(0.04)

r.log("== pregrasp")
r.goto([cx, cy, pz], q, seconds=5.0)
from scipy.spatial.transform import Rotation as Rot
_, hq, _ = r.hand_pose()
fy = Rot.from_quat(hq).as_matrix()[:, 1]
fang = np.degrees(np.arctan2(fy[1], fy[0])) % 180
r.log(f"finger axis angle from +x: {fang:.1f} deg (want {(90 + yaw) % 180:.1f})")
if abs(((fang - (90 + yaw)) + 90) % 180 - 90) > 5:
    r.log("!! finger axis wrong; aborting")
    r.close(); sys.exit(3)
r.log("== descend")
code, err, tcp = r.goto([cx, cy, gz], q, seconds=3.0)
if np.linalg.norm(tcp[:2] - [cx, cy]) > 0.008 or abs(tcp[2] - gz) > 0.008:
    r.log("!! descend inaccurate, retrying")
    r.goto([cx, cy, gz], q, seconds=3.0)
r.log("== close")
f = r.gripper(0.0)
gap = f[0] - f[1]
r.log(f"finger gap = {gap:.4f} m")
if gap < 0.01:
    r.log("!! closed on air; aborting")
    r.gripper(0.04)
    r.goto([cx, cy, pz], q, seconds=3.0)
    r.close()
    sys.exit(2)
r.log("== lift")
r.goto([cx, cy, DROP_Z], q, seconds=3.0)
r.snap("agentview", f"/workspace/{sys.argv[1]}_lifted.png")
f = r.fingers()
r.log(f"gap after lift = {f[0]-f[1]:.4f}")
r.log("== to basket")
r.goto([BASKET[0], BASKET[1], DROP_Z], q, seconds=5.0)
f = r.fingers()
r.log(f"gap over basket = {f[0]-f[1]:.4f}")
r.log("== release")
r.gripper(0.04)
r.spin(0.5)
r.log("== retreat up")
r.goto([BASKET[0], BASKET[1], 0.85], q, seconds=3.0)
r.snap("agentview", f"/workspace/{sys.argv[1]}_dropped.png")
r.close()
