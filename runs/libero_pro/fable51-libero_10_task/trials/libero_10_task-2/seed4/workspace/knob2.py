#!/usr/bin/env python3
"""Continue: gripper already closed on the fin; rotate joint7 by given delta, release, lift."""
import sys
import numpy as np
from rob import *

delta = float(sys.argv[1])
TCP = M["hand"]["tcp_offset_m"]
r = Robot("knob2")
q = r.arm_q()
print("q", np.round(q, 3), "fingers", r.fingers(), flush=True)
q_rot = list(q); q_rot[6] = q[6] + delta
lim = FJT["limits_rad"][6]
assert lim[0] < q_rot[6] < lim[1], q_rot[6]
print("rotating joint7", round(q[6], 3), "->", round(q_rot[6], 3), flush=True)
r.move_q(q_rot, 3.0)
print("q after", np.round(r.arm_q(), 3), "fingers", r.fingers(), flush=True)
print("wrench", r.wrench(), flush=True)
if "--hold" not in sys.argv:
    r.gripper(0.04)
    q = r.arm_q()
    p, qu = r.fk(q)
    r.move_pose([p[0], p[1], p[2] + 0.12], qu, 2.5, seed=q)
print("done", flush=True)
r.close()
