from arm import *
a = Arm()
move_tcp(a, [0.135, -0.0195, 0.72], 0.0, 4.0)
print("fingers", a.fingers())
move_tcp(a, [0.135, -0.0195, 0.60], 0.0, 2.5)
move_tcp(a, [0.135, -0.0195, 0.556], 0.0, 2.0)
print("fingers", a.fingers())
