#!/usr/bin/env python3
"""Pick the cream cheese box and place it in the basket."""
from arm import *

CC = np.array([0.103, -0.187])
BASKET = np.array([-0.01, 0.30])   # slightly off the can's drop spot
a = Arm()
print("start tcp", a.hand_pose()[1].round(3), "fingers", a.fingers())
print("open gripper"); a.gripper(GRIP["open_m"])
print("pregrasp"); a.move_tcp([*CC, 0.60], seconds=4)
print("descend"); a.move_tcp([*CC, 0.438], seconds=3)
print("close"); a.gripper(GRIP["closed_m"])
print("lift"); a.move_tcp([*CC, 0.70], seconds=3)
print("fingers after lift", a.fingers())
print("over basket"); a.move_tcp([*BASKET, 0.76], seconds=4)
print("fingers over basket", a.fingers())
print("release"); a.gripper(GRIP["open_m"])
print("retreat"); a.move_tcp([*BASKET, 0.85], seconds=3)
print("end tcp", a.hand_pose()[1].round(3), "fingers", a.fingers())
a.close()
print("DONE")
