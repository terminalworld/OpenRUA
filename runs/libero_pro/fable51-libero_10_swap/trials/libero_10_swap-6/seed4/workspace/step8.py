from arm import *
a = Arm()
print("fingers", a.fingers())
move_tcp(a, [-0.200, 0.013, 0.60], 0.0, 3.0)
move_tcp(a, [-0.200, 0.013, 0.50], 0.0, 2.0)
