#!/usr/bin/env python3
"""Descend onto the knob fin, grasp, rotate joint7 by DELTA, release, lift."""
import sys, subprocess
from rob import *

KNOB = np.array([-0.205, 0.194])
DELTA = float(sys.argv[1]) if len(sys.argv) > 1 else -math.pi / 2
r = Robot("step2")
print("wrench before", r.wrench()[0].round(2), flush=True)
q = r.move_tcp((KNOB[0], KNOB[1], 0.945), quat_down(0), seconds=3)
print("wrench at grasp height", r.wrench()[0].round(2), flush=True)
f = r.gripper(GRIP["closed_m"])
subprocess.run(["python3", "tools/perception/cam_snap.py", "agentview", "agent_knob_grasp.png"])
q = r.arm_q()
q7 = q[6] + DELTA
if not (LIMITS[6][0] < q7 < LIMITS[6][1]):
    raise SystemExit(f"joint7 target {q7} out of limits")
q2 = list(q); q2[6] = q7
r.move_q(q2, seconds=3)
print("wrench after rotate", r.wrench()[0].round(2), flush=True)
subprocess.run(["python3", "tools/perception/cam_snap.py", "agentview", "agent_knob_rotated.png"])
r.gripper(GRIP["open_m"])
r.move_tcp((KNOB[0], KNOB[1], 1.05), quat_down(0), seconds=2)
subprocess.run(["python3", "tools/perception/cam_snap.py", "agentview", "agent_knob_after.png"])
subprocess.run(["python3", "tools/perception/cam_snap.py", "sideview", "side_knob_after.png"])
