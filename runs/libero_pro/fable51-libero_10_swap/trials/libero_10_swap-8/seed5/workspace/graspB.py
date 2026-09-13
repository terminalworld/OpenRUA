from lib import *
r = Robot("graspB")
kx, ky = -0.0671, 0.2331
Q = down_quat(90)
if r.finger_gap() < 0.078: r.gripper(0.04)
print("-- above knob"); r.move_tcp((kx, ky, 1.12), Q, 3)
print("-- descend"); r.move_tcp((kx, ky, 1.039), Q, 3)
print("-- close"); gap = r.gripper(0.0)
print("-- lift"); r.move_tcp((kx, ky, 1.25), Q, 3)
print("gap after lift", r.finger_gap())
