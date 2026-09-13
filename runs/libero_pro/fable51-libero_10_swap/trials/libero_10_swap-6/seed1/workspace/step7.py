from arm import *
a = Arm()
pre = (-0.215, 0.022, 0.60)
q = a.ik(pre, Q_DOWN, tcp=True)
for i in range(3):
    code, err = a.move_joints(q, 3.5)
    if err < 0.01: break
print("hand:", np.round(a.fk()[0],4))
