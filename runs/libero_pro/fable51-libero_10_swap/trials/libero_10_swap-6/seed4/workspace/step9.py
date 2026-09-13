from arm import *
a = Arm()
move_tcp(a, [-0.200, 0.013, 0.448], 0.0, 2.0)
a.gripper(0.0)
for i in range(2):
    a.move_joints(a.arm_q(), 0.5); print("fingers", a.fingers())
move_tcp(a, [-0.200, 0.013, 0.60], 0.0, 2.0)
print("fingers after lift", a.fingers())
