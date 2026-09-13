#!/usr/bin/env python3
"""Step 1: grasp the stove knob fin and rotate it."""
import sys
import math
import numpy as np
from rob import *

KNOB = np.array([-0.200, 0.198])
TCP = M["hand"]["tcp_offset_m"]
direction = float(sys.argv[1]) if len(sys.argv) > 1 else 1.0
angle = float(sys.argv[2]) if len(sys.argv) > 2 else math.pi / 2

r = Robot("knob")
print("start q", np.round(r.arm_q(), 3), flush=True)
r.gripper(0.04)

q_top = topdown_quat(0.0)  # fingers close along world Y (fin runs along X)
hover = [KNOB[0], KNOB[1], 0.935 + TCP + 0.12]
print("hover", hover, flush=True)
q1 = r.move_pose(hover, q_top, 4.0)
if q1 is None:
    raise SystemExit("IK hover failed")

grasp = [KNOB[0], KNOB[1], 0.935 + TCP]
q2 = r.move_pose(grasp, q_top, 2.5, seed=q1)
if q2 is None:
    raise SystemExit("IK grasp failed")
p, qu = r.fk()
print("hand at", p.round(4), "fingertip z", round(p[2] - TCP, 4), flush=True)
print("wrench", r.wrench(), flush=True)

f = r.gripper(0.0)
print("fingers after close", f, flush=True)

# rotate about the vertical hand axis using joint7 only
q = r.arm_q()
q_rot = list(q)
q_rot[6] = q[6] + direction * angle
lim = FJT["limits_rad"][6]
if not (lim[0] < q_rot[6] < lim[1]):
    raise SystemExit(f"joint7 target {q_rot[6]} out of limits {lim}")
print("rotating joint7 from", round(q[6], 3), "to", round(q_rot[6], 3), flush=True)
r.move_q(q_rot, 3.0)
print("q after rot", np.round(r.arm_q(), 3), flush=True)
print("wrench", r.wrench(), flush=True)

r.gripper(0.04)
q = r.arm_q()
_, qu_now = r.fk(q)
r.move_pose(hover, qu_now, 2.5, seed=q)
print("done", flush=True)
r.close()
