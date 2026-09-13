from arm import *
a = Arm()
a.gripper(0.04)
move_tcp(a, [0.135, -0.0195, 0.60], 0.0, 1.5)
move_tcp(a, [0.135, -0.0195, 0.75], 0.0, 2.0)
print("fingers", a.fingers())
