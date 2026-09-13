from rob import *
r=Robot()
print("gap before", round(r.finger_gap(),4))
gap=r.gripper(GRIP["closed_m"])
print("gap after close", round(gap,4))
