from arm import *
a = Arm()
ok1 = move_tcp(a, [-0.094, -0.195, 0.575], 0.0, 2.0)
ok2 = move_tcp(a, [-0.094, -0.195, 0.529], 0.0, 2.0)
print("descent ok", ok1, ok2)
