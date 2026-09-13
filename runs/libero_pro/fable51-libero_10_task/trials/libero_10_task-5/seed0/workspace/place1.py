#!/usr/bin/env python3
"""Step 2: carry the cup over slot B, lower until contact, release, retreat."""
import sys
import numpy as np
from kin import Robot

QD = np.array([0.7071, 0.7071, 0.0, 0.0])
CUP_OFF = np.array([0.0, 0.069])          # cup body centre minus TCP (xy, world)
TX, TY = float(sys.argv[1]), float(sys.argv[2])   # desired cup-centre xy
Z_HI, Z_START, Z_MIN = 1.20, 1.16, 1.07
tcp_xy = np.array([TX, TY]) - CUP_OFF

r = Robot("place1")
r.report("start")
base_w = r.wrench()
print("baseline wrench", base_w.round(2))

q = r.arm_q()
s = r.ik((tcp_xy[0], tcp_xy[1], Z_HI), QD, seed=q); assert s
r.move(s, 6.0)
r.report("over slot")

seed = s
z = Z_START
while z >= Z_MIN - 1e-6:
    s = r.ik((tcp_xy[0], tcp_xy[1], z), QD, seed=seed); assert s, f"IK fail z={z}"
    seed = s
    r.move(s, 1.0, retries=1)
    w = r.wrench()
    dz = w[2] - base_w[2]
    p = r.report(f"z={z:.3f}")
    print(f"   Fz={w[2]:.2f} dFz={dz:.2f} torque={w[3:].round(2)}")
    if abs(dz) > 1.5 or abs(w[3]) > 0.3 or abs(w[4]) > 0.3:
        print("   contact detected")
        break
    z -= 0.01
