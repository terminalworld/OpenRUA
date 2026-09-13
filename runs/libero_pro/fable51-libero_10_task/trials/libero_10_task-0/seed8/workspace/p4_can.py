"""Grasp the lying tomato can perpendicular to its axis, drop in basket.
Usage: p4_can.py <cx> <cy> <axis_deg> <ztop>"""
import sys
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from rb import RB, q_down_yaw

cx, cy, axis, ztop = map(float, sys.argv[1:5])
yaw = axis            # finger axis at 90+yaw = axis+90 -> perpendicular to can
gz = ztop - 0.039     # pads around the can's centre height, palm clear of top
pz = ztop + 0.08
DROP = (0.0, 0.26, 0.72)

r = RB("p4")
q = q_down_yaw(yaw)
if r.fingers()[0] < 0.035:
    r.gripper(0.04)

r.log("== pregrasp")
r.goto([cx, cy, pz], q, seconds=5.0)
_, hq, _ = r.hand_pose()
fy = Rot.from_quat(hq).as_matrix()[:, 1]
fang = np.degrees(np.arctan2(fy[1], fy[0])) % 180
r.log(f"finger axis {fang:.1f} deg, want {(90 + yaw) % 180:.1f}")
if abs(((fang - (90 + yaw)) + 90) % 180 - 90) > 5:
    r.log("!! finger axis wrong"); r.close(); sys.exit(3)

r.log("== straight descent")
code, err, tcp = r.line([cx, cy, pz], [cx, cy, gz], q, step=0.02)
if np.linalg.norm(tcp[:2] - [cx, cy]) > 0.006 or abs(tcp[2] - gz) > 0.006:
    r.log("!! descent off; backing out")
    r.line([cx, cy, tcp[2]], [cx, cy, pz], q)
    r.close(); sys.exit(2)

r.log("== close")
f = r.gripper(0.0)
gap = f[0] - f[1]
r.log(f"gap = {gap:.4f} (fingers {f[0]:.4f} {f[1]:.4f}); expect ~0.065")
if not (0.055 < gap < 0.075):
    r.log("!! bad grasp; releasing and backing out")
    r.gripper(0.04)
    r.line([cx, cy, gz], [cx, cy, pz], q)
    r.close(); sys.exit(2)

r.log("== lift straight")
r.line([cx, cy, gz], [cx, cy, 0.72], q, step=0.04)
f = r.fingers(); r.log(f"gap after lift {f[0]-f[1]:.4f}")
r.snap("agentview", "/workspace/can_lifted.png")
if f[0] - f[1] < 0.05:
    r.log("!! lost the can during lift"); r.close(); sys.exit(2)

r.log("== to basket")
r.goto(DROP, q, seconds=5.0)
f = r.fingers(); r.log(f"gap over basket {f[0]-f[1]:.4f}")
if f[0] - f[1] < 0.05:
    r.log("!! lost the can in transit"); r.close(); sys.exit(2)
r.log("== release")
r.gripper(0.04)
r.spin(0.5)
r.goto([DROP[0], DROP[1], 0.85], q, seconds=3.0)
r.snap("agentview", "/workspace/can_dropped.png")
r.close()
