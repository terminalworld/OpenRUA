from lib import *
r = Robot("liftA")
kx, ky = -0.2052, -0.1946
Q = down_quat(90)
r.move_tcp((kx, ky, 1.10), Q, 3)
print("gap after lift", r.finger_gap())
