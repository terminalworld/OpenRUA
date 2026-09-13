#!/usr/bin/env python3
"""Approach the stove knob: open gripper, hover above the fin, descend to grasp height."""
from rob import *
import subprocess

KNOB = np.array([-0.205, 0.194])
r = Robot("step1")
r.gripper(GRIP["open_m"])
q = r.move_tcp((KNOB[0], KNOB[1], 1.02), quat_down(0), seconds=4)
print("hover q", np.round(q, 3), flush=True)
subprocess.run(["python3", "tools/perception/cam_snap.py", "robot0_eye_in_hand", "eih_knob_hover.png"])
subprocess.run(["python3", "tools/perception/cam_snap.py", "agentview", "agent_knob_hover.png"])
print("fingers", r.fingers(), flush=True)
