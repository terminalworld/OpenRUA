from arm import *
a = Arm()
home = [0.0, -0.161, 0.0, -2.445, 0.0, 2.227, 0.785]
for i in range(4):
    code, err = a.move_joints(home, 6.0)
    if err < 0.01: break
print("hand:", np.round(a.fk()[0],4), np.round(a.arm_q(),3))
