#!/usr/bin/env python3
"""Pick the tomato sauce can and place it in the basket."""
from arm import *

CAN = np.array([-0.095, 0.027])
BASKET = np.array([0.005, 0.264])
a = Arm()
print("start tcp", a.hand_pose()[1].round(3), "fingers", a.fingers())

print("open gripper"); a.gripper(GRIP["open_m"])
print("pregrasp"); a.move_tcp([*CAN, 0.62], seconds=4)
print("descend"); a.move_tcp([*CAN, 0.47], seconds=3)
print("close"); f = a.gripper(GRIP["closed_m"])
print("lift"); a.move_tcp([*CAN, 0.72], seconds=3)
print("fingers after lift", a.fingers())
print("over basket"); a.move_tcp([*BASKET, 0.76], seconds=4)
print("release"); a.gripper(GRIP["open_m"])
print("retreat"); a.move_tcp([*BASKET, 0.85], seconds=3)
print("end tcp", a.hand_pose()[1].round(3), "fingers", a.fingers())
a.close()
print("DONE")
