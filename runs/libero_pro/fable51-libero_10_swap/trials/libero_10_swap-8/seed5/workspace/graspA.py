from lib import *
r = Robot("graspA")
kx, ky = -0.2052, -0.1946
Q = down_quat(90)
print("gap before", r.finger_gap())
if r.finger_gap() < 0.078: r.gripper(0.04)
print("-- above knob")
r.move_tcp((kx, ky, 1.12), Q, 3)
print("-- descend to knob")
r.move_tcp((kx, ky, 1.039), Q, 3)
print("-- close")
gap = r.gripper(0.0)
print("gap after close", gap)
q = r.arm_q(); print("q", np.round(q,3))
