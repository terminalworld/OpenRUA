from arm import *
a = Arm()
move_tcp(a, [-0.200, 0.013, 0.72], 0.0, 2.0)
move_tcp(a, [0.135, 0.155, 0.72], 90.0, 4.0)
print("fingers", a.fingers())
move_tcp(a, [0.135, 0.155, 0.56], 90.0, 2.5)
print("fingers", a.fingers())
