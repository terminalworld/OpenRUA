from lib import *
r = Robot("park")
r.move_tcp((-0.15, 0.0, 1.30), down_quat(90), 5)
print("gap", r.finger_gap())
