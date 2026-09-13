import numpy as np
from lib import Robot
r = Robot("stepk")
quat = (0.8315, 0.5556, 0, 0)
def go(pos, sec):
    for attempt in range(3):
        q = r.ik_world(pos, quat)
        code, err = r.move_q(q, sec)
        if err < 0.01: break
        print("  retrying (lag)")
    p, _, _ = r.fk_world(); print("  at", np.round(p,4), "gap", round(r.finger_gap(),4), "wrench", np.round(r.wrench()[0],2))
go([-0.0294, -0.237, 1.32], 2.0)
go([-0.058, 0.20, 1.32], 4.0)
go([-0.058, 0.20, 1.20], 2.0)
go([-0.058, 0.20, 1.165], 1.5)
