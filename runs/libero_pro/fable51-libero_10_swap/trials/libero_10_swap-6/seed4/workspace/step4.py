from arm import *
a = Arm()
a.gripper(0.0)
import time
for i in range(3):
    a.move_joints(a.arm_q(), 0.5)   # tick the sim so finger readings settle
    print("fingers", a.fingers())
