#!/usr/bin/env python3
"""Pick the cream cheese box (fingers along Y) and drop it into the basket."""
import numpy as np
from ctl import Ctl, Q_HAND_Y

X, Y = -0.169, 0.033
Z_GRASP, Z_PRE, Z_HIGH = 0.445, 0.60, 0.76
BASKET = (0.0, 0.25)
Z_RELEASE = 0.65
OPEN = 0.04
q = Q_HAND_Y

c = Ctl()
print("start tcp", np.round(c.tcp_pose()[0], 3), "fingers", np.round(c.finger_gap(), 4))
print("open"); c.gripper(OPEN)
print("go high above box"); c.move_tcp((X, Y, Z_HIGH), q, 4.0)
print("descend to pre-grasp"); c.move_tcp_line((X, Y, Z_PRE), q, n=3, seconds=3.0)
print("descend to grasp"); c.move_tcp_line((X, Y, Z_GRASP), q, n=4, seconds=4.0)
p, o = c.tcp_pose(); print("at grasp: tcp", np.round(p, 4), "q", np.round(o, 3))
print("close"); f1, f2 = c.gripper(0.0)
gap = abs(f1) + abs(f2)
print(f"gap after close = {gap:.4f} m ({'HOLDING' if 0.008 < gap else 'EMPTY?'})")
print("lift"); c.move_tcp_line((X, Y, Z_PRE), q, n=3, seconds=3.0)
print("fingers after lift", np.round(c.finger_gap(), 4))
print("to high"); c.move_tcp((X, Y, Z_HIGH), q, 2.5)
print("over basket"); c.move_tcp((BASKET[0], BASKET[1], Z_HIGH), q, 4.0)
print("fingers over basket", np.round(c.finger_gap(), 4))
print("lower"); c.move_tcp_line((BASKET[0], BASKET[1], Z_RELEASE), q, n=2, seconds=2.0)
print("release"); c.gripper(OPEN)
print("retreat"); c.move_tcp((BASKET[0], BASKET[1], Z_HIGH), q, 2.0)
print("park away from basket"); c.move_tcp((-0.15, -0.05, 0.76), q, 3.0)
print("DONE tcp", np.round(c.tcp_pose()[0], 3))
