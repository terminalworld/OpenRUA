from lib import *
r = Robot("warm")
print("gap before", r.finger_gap())
r.gripper(0.04)
r.report("after open")
