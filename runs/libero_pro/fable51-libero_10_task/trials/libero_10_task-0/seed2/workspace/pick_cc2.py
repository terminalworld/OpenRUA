#!/usr/bin/env python3
from arm import *
CC = np.array([0.1096, -0.192]); Q = yaw_quat(np.radians(3.8))
BASKET = np.array([-0.01, 0.30])
a = Arm()
print("open"); a.gripper(GRIP["open_m"])
print("pregrasp"); a.move_tcp([*CC, 0.58], Q, seconds=4)
print("descend"); a.move_tcp([*CC, 0.435], Q, seconds=3)
print("close"); a.gripper(GRIP["closed_m"])
print("lift"); a.move_tcp([*CC, 0.70], Q, seconds=3)
f = a.fingers(); print("fingers after lift", f)
if f[0] < 0.01:
    print("GRASP FAILED - stopping before basket"); a.close(); raise SystemExit(1)
print("over basket"); a.move_tcp([*BASKET, 0.76], seconds=4)
print("fingers over basket", a.fingers())
print("release"); a.gripper(GRIP["open_m"])
print("retreat"); a.move_tcp([*BASKET, 0.85], seconds=3)
print("end tcp", a.hand_pose()[1].round(3), "fingers", a.fingers())
a.close(); print("DONE")
