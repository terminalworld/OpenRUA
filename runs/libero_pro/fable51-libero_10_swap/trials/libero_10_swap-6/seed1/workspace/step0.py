from arm import *
a = Arm()
print("fingers before:", a.fingers())
a.gripper(0.04)
print(a.fk())
