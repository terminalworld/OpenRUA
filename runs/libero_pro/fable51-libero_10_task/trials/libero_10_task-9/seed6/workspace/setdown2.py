"""Carry held upright mug (pinch on bar) to pinch xy and set down: setdown2.py px py [transit_z] [bottom_offset]"""
import sys, numpy as np
from rob import *; from plan import Planner
px, py = float(sys.argv[1]), float(sys.argv[2]); tz = float(sys.argv[3]) if len(sys.argv) > 3 else 1.04
TABLE = 0.9017; PINCH_ABOVE_BOTTOM = 0.056
r = Planner("setdown2")
tcp, R = r.tcp(); a = R[:,2]
wps = [np.array([tcp[0], tcp[1], tz]), np.array([px, py, tz]), np.array([px, py, TABLE + PINCH_ABOVE_BOTTOM + 0.004])]
for w in wps:
    ok = r.move_line_tcp([w], R, avoid=False); print("to", w.round(3), "ok", ok, "fingers", np.round(r.fingers(),4))
    if not ok: sys.exit("move failed")
r.gripper(0.04)
tcp, R = r.tcp(); ok = r.move_line_tcp([tcp - 0.06*a], R, avoid=False); print("retreat ok", ok)
tcp, R = r.tcp(); ok = r.move_line_tcp([tcp + [0,0,0.06]], R, avoid=False); print("up ok", ok, r.tcp()[0].round(3))
