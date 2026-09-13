#!/usr/bin/env python3
"""Carry the moka pot over the burner, lower it, release, retreat, snapshot."""
import sys, subprocess
from rob import *

STOVE = np.array([-0.057, 0.195])
YAW = math.pi / 2
BASE_BELOW_TCP = 0.136  # pot base sits this far below the TCP with the current grasp
BURNER_Z = 0.93
r = Robot("step6")
print("fingers", r.fingers(), "wrench", r.wrench()[0].round(2), flush=True)
# waypoint half way keeps the path away from the pan and the motion smooth
r.move_tcp((-0.06, -0.02, 1.15), quat_down(YAW))
print("fingers", r.fingers(), flush=True)
r.move_tcp((STOVE[0], STOVE[1], 1.15), quat_down(YAW))
print("fingers", r.fingers(), flush=True)
subprocess.run(["python3", "tools/perception/cam_snap.py", "robot0_eye_in_hand", "eih_stove_hover.png"])
if "--hover-only" in sys.argv:
    raise SystemExit(0)
r.move_tcp((STOVE[0], STOVE[1], BURNER_Z + BASE_BELOW_TCP + 0.01), quat_down(YAW))
print("wrench before release", r.wrench()[0].round(2), "fingers", r.fingers(), flush=True)
r.gripper(GRIP["open_m"])
r.move_tcp((STOVE[0], STOVE[1], 1.15), quat_down(YAW))
subprocess.run(["python3", "tools/perception/cam_snap.py", "agentview", "agent_placed.png"])
HOME = [0.0, -0.161037389, 0.0, -2.44459747, 0.0, 2.2267522, 0.7853981633974483]
r.move_q(HOME)
subprocess.run(["python3", "tools/perception/cam_snap.py", "agentview", "agent_final.png"])
subprocess.run(["python3", "tools/perception/cam_snap.py", "birdview", "bird_final.png"])
subprocess.run(["python3", "tools/perception/cam_snap.py", "sideview", "side_final.png"])
