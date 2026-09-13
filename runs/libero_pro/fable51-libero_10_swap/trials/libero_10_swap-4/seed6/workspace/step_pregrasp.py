#!/usr/bin/env python3
"""Move TCP to a world position with a down-pointing hand. Usage:
python3 step_pregrasp.py x y z [yaw_deg] [seconds]"""
import sys
import numpy as np
from arm import Arm, down_quat

x, y, z = map(float, sys.argv[1:4])
yaw = np.deg2rad(float(sys.argv[4])) if len(sys.argv) > 4 else 0.0
secs = float(sys.argv[5]) if len(sys.argv) > 5 else 4.0
a = Arm("pregrasp")
print("before TCP world", a.tcp_world()[0].round(4), flush=True)
q = a.move_tcp_world([x, y, z], down_quat(yaw), seconds=secs)
print("q now", a.arm_q().round(4))
