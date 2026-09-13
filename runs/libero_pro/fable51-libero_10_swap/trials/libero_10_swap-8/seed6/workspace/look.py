#!/usr/bin/env python3
"""Move the hand to (x,y,z) looking straight down, then capture the eye-in-hand cloud."""
import sys, subprocess
import numpy as np
from geom import R_PLACE
from rob import Robot, log
p = np.array(list(map(float, sys.argv[1:4])))
r = Robot("look")
q = r.ik(p, R_PLACE)
assert q is not None, "no IK"
ok = r.move_path([q], 6.0)
log(f"look ok={ok} hand {np.round(r.fk()[0],4)}")
subprocess.run(["python3", "cloud.py", "robot0_eye_in_hand"], check=True)
