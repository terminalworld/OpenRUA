from arm import *
a = Arm()
q = a.ik((-0.30, -0.25, 0.95), Q_DOWN)
for i in range(3):
    code, err = a.move_joints(q, 3.5)
    if err < 0.01: break
print("hand:", np.round(a.fk()[0],4))
