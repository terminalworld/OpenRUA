#!/usr/bin/env python3
"""Move the hand by a world-frame delta keeping its current orientation (from FK).
Usage: python3 -u nudge.py dx dy dz [secs]
"""
import sys
import numpy as np
from rob import Robot, log

d = np.array([float(a) for a in sys.argv[1:4]])
secs = float(sys.argv[4]) if len(sys.argv) > 4 else 3.0
r = Robot("nudge")
p, q = r.fk()
x, y, z, w = q
R = np.array([
    [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
    [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
    [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
])
log("hand", np.round(p, 4), "fingers", r.fingers())
ok = r.move_line(p + d, R, secs)
log("ok" if ok else "STALLED", "fingers", r.fingers())
