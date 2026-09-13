from arm import *
a = Arm()
q = a.ik((0.14, -0.04, 0.72), Q_DOWN_X, tcp=True)
for i in range(3):
    code, err = a.move_joints(q, 3.5)
    if err < 0.01: break
print("hand:", np.round(a.fk()[0],4), np.round(a.arm_q(),3))
