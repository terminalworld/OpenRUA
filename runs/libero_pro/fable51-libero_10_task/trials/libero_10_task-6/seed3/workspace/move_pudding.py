#!/usr/bin/env python3
"""Pick the chocolate pudding (lying flat, long axis along x) across y, yaw 90 deg,
place it flat to the +y side of the plate with its long axis along y.

python3 move_pudding.py dry   -> IK feasibility only
python3 move_pudding.py go    -> execute
"""
import sys, subprocess
import numpy as np, rclpy
from arm import Arm, DOWN_Y, down_closing_along

PUD = np.array([0.205, 0.1575])          # grasp point (1.5 cm toward +x of the box centre)
TABLE = 0.425
GRASP_Z = TABLE + 0.012                  # fingertips 1.2 cm above the table (box is 4 cm tall)
CARRY_Z = 0.60
TARGET = np.array([0.123, 0.140])        # box centre: 2 cm beyond the plate edge (y=0.079)
PLACE_Z = GRASP_Z + 0.006

a = Arm()
Qg = DOWN_Y
cands = [down_closing_along(0), down_closing_along(180)]
poses = [([*PUD, CARRY_Z], Qg), ([*PUD, GRASP_Z], Qg)]
print("IK check from current q", np.round(a.arm_q(), 3).tolist())
for p, Q in poses:
    q, j = a.ik_near(p, Q, max_jump=9); print("  grasp pose", np.round(p, 3), "jump %.2f" % j)
best = None
for i, Qp in enumerate(cands):
    try:
        q1, j1 = a.ik_near([*PUD, CARRY_Z], Qp, max_jump=9)
        q2, j2 = a.ik_near([*TARGET, CARRY_Z], Qp, max_jump=9)
        q3, j3 = a.ik_near([*TARGET, PLACE_Z], Qp, max_jump=9)
        print("  place yaw cand %d: jumps from park %.2f %.2f %.2f" % (i, j1, j2, j3))
        if best is None or j1 < best[1]:
            best = (Qp, j1)
    except RuntimeError as e:
        print("  place yaw cand %d: %s" % (i, e))
Qp = best[0]
if sys.argv[1:2] != ["go"]:
    rclpy.shutdown(); sys.exit()

a.gripper(0.04)
print("== hover over pudding"); a.goto([*PUD, CARRY_Z], Qg, seconds=5.0, max_jump=3.0)
print("== descend"); a.goto([*PUD, GRASP_Z], Qg, seconds=3.0)
f = a.gripper(0.0); gap = abs(f[0]) + abs(f[1])
print("gap %.4f (box is ~0.053 across y)" % gap)
if not 0.040 < gap < 0.065:
    raise RuntimeError("grasp looks wrong")
print("== lift"); a.goto([*PUD, CARRY_Z], Qg, seconds=3.0)
f = a.fingers(); print("fingers after lift", f)
if abs(f[0]) + abs(f[1]) < 0.040:
    raise RuntimeError("lost the box")
print("== yaw 90"); a.goto([*PUD, CARRY_Z], Qp, seconds=3.0, max_jump=2.5)
print("== carry"); a.goto([*TARGET, CARRY_Z], Qp, seconds=3.5)
print("== lower"); a.goto([*TARGET, PLACE_Z], Qp, seconds=3.0)
print("== release"); a.gripper(0.04)
print("== up"); a.goto([*TARGET, CARRY_Z], Qp, seconds=3.0)
print("== park"); a.goto([-0.15, -0.30, 0.75], DOWN_Y, seconds=5.0, max_jump=3.0)
subprocess.run([sys.executable, "/workspace/locate.py", "agentview"], check=True, stdout=subprocess.DEVNULL)
rclpy.shutdown()
