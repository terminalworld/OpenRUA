#!/usr/bin/env python3
import sys
sys.path.insert(0, "/workspace")
from arm import Arm
import rclpy

BUT = (0.067, 0.048)        # butter center (0.067,0.038) +1cm y for milk clearance
GRASP_Z = 0.44              # fingertips ~0.431 (table 0.425), hand bottom 0.485 > top 0.449
PRE_Z = 0.60
CARRY_Z = 0.76
DROP = (0.01, 0.295)        # inside basket, +y side (can is at y~0.23)
DROP_Z = 0.70

a = Arm()
print("open gripper"); a.gripper(True)
print("pre-grasp"); a.move_tcp(BUT[0], BUT[1], PRE_Z, 0, 4)
print("descend"); a.move_tcp(BUT[0], BUT[1], GRASP_Z, 0, 2.5)
js = a.joints()
print("close"); js = a.gripper(False)
gap = js["panda_finger_joint1"]
if gap < 0.008:
    print("GRASP FAILED: fingers closed to", gap); rclpy.shutdown(); sys.exit(1)
print("lift"); a.move_tcp(BUT[0], BUT[1], CARRY_Z, 0, 3)
js = a.joints(); print("  fingers after lift", js["panda_finger_joint1"])
print("to basket"); a.move_tcp(DROP[0], DROP[1], CARRY_Z, 0, 4)
print("lower"); a.move_tcp(DROP[0], DROP[1], DROP_Z, 0, 2)
print("release"); a.gripper(True)
print("retreat"); a.move_tcp(DROP[0], DROP[1], CARRY_Z, 0, 2)
print("DONE")
rclpy.shutdown()
