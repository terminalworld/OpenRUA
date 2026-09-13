#!/usr/bin/env python3
import sys
sys.path.insert(0, "/workspace")
from arm import Arm
import rclpy

CAN = (-0.122, -0.163)      # world xy of alphabet soup can (birdview)
GRASP_Z = 0.465             # TCP height: hand bottom 0.51 > can top 0.50
PRE_Z = 0.62
CARRY_Z = 0.76
DROP = (-0.015, 0.232)      # inside basket, -y side
DROP_Z = 0.70               # hand bottom 0.745, can bottom ~0.66 > rim 0.628

a = Arm()
print("open gripper"); a.gripper(True)
print("pre-grasp"); a.move_tcp(CAN[0], CAN[1], PRE_Z, 0, 4)
print("descend"); a.move_tcp(CAN[0], CAN[1], GRASP_Z, 0, 2.5)
print("close"); js = a.gripper(False)
gap = js["panda_finger_joint1"]
if gap < 0.01:
    print("GRASP FAILED: fingers closed to", gap); rclpy.shutdown(); sys.exit(1)
print("lift"); a.move_tcp(CAN[0], CAN[1], CARRY_Z, 0, 2.5)
js = a.joints(); print("  fingers after lift", js["panda_finger_joint1"])
print("to basket"); a.move_tcp(DROP[0], DROP[1], CARRY_Z, 0, 4)
print("lower"); a.move_tcp(DROP[0], DROP[1], DROP_Z, 0, 2)
print("release"); a.gripper(True)
print("retreat"); a.move_tcp(DROP[0], DROP[1], CARRY_Z, 0, 2)
print("DONE")
rclpy.shutdown()
