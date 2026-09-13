from arm import *
a = Arm()
move_tcp(a, [-0.094, -0.195, 0.72], 0.0, 3.0)
print("fingers", a.fingers())
