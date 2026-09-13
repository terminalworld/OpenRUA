from arm import *
a = Arm()
move_tcp(a, [0.135, 0.155, 0.456], 90.0, 2.0)
a.gripper(0.04)
move_tcp(a, [0.135, 0.155, 0.60], 90.0, 2.0)
move_tcp(a, [-0.10, -0.30, 0.75], 0.0, 3.0)
print("fingers", a.fingers())
