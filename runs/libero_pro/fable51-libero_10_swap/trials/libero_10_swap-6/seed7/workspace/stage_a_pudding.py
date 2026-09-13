#!/usr/bin/env python3
"""Stage A: pick the chocolate pudding, place it to the right (+y) of the plate."""
from robot import *

PUD = np.array([-0.187, 0.013])        # pudding centre (world xy), from birdview
PLATE = np.array([0.141, 0.000])
TABLE = 0.425
DEST = np.array([0.14, 0.17])          # right of the plate (agentview right = +y)
Q = down_quat(0)                       # fingers along world y (box's 4.4 cm side)
READY = np.array([0, -0.785, 0, -2.356, 0, 1.571, 0.785])

r = Robot()
print("== open gripper"); r.open()
print("== ready pose"); r.move_joints(READY)
print("== above pudding"); assert r.move_tcp([*PUD, 0.60], Q)
print("== descend");       assert r.move_tcp_line([*PUD, 0.60], [*PUD, 0.438], Q, steps=4)
p, _ = r.tcp(); print("   tcp now", np.round(p, 4))
print("== close");         gap = r.close()
print("   finger gap after close:", round(gap, 4), "(>0 means something is held)")
print("== lift");          assert r.move_tcp_line([*PUD, 0.438], [*PUD, 0.60], Q, steps=3)
print("   gap while lifted:", round(r.finger_gap(), 4))
print("== over destination"); assert r.move_tcp([*DEST, 0.60], Q)
print("== lower");         assert r.move_tcp_line([*DEST, 0.60], [*DEST, 0.447], Q, steps=4)
print("== release");       r.open()
print("== retreat up");    assert r.move_tcp_line([*DEST, 0.447], [*DEST, 0.62], Q, steps=3)
print("== back to ready"); r.move_joints(READY)
print("STAGE A DONE")
