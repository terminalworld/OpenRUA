#!/usr/bin/env python3
"""Pick the moka pot: hover, descend to the waist, close, lift."""
import sys, subprocess
from rob import *

POT = np.array([-0.042, -0.237])
GRASP_Z = 0.965
YAW = math.pi / 2  # fingers along world x
r = Robot("step5")
r.gripper(GRIP["open_m"])
r.move_tcp((POT[0], POT[1], 1.12), quat_down(YAW))
subprocess.run(["python3", "tools/perception/cam_snap.py", "robot0_eye_in_hand", "eih_pot_hover.png"])
subprocess.run(["python3", "tools/perception/cam_snap.py", "agentview", "agent_pot_hover.png"])
if "--hover-only" in sys.argv:
    raise SystemExit(0)
r.move_tcp((POT[0], POT[1], 1.02), quat_down(YAW))
print("wrench pre-grasp", r.wrench()[0].round(2), flush=True)
r.move_tcp((POT[0], POT[1], GRASP_Z), quat_down(YAW))
print("wrench at grasp z", r.wrench()[0].round(2), flush=True)
subprocess.run(["python3", "tools/perception/cam_snap.py", "agentview", "agent_pot_low.png"])
f = r.gripper(GRIP["closed_m"])
print("grasp gap (per finger)", f, flush=True)
r.move_tcp((POT[0], POT[1], 1.15), quat_down(YAW))
print("wrench lifted", r.wrench()[0].round(2), "fingers", r.fingers(), flush=True)
subprocess.run(["python3", "tools/perception/cam_snap.py", "agentview", "agent_pot_lifted.png"])
