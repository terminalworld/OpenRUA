from arm import *
a = Arm()
c = np.array([-0.110, -0.176]); r = 0.045
pre = (c[0], c[1] - r, 0.62)
q = a.ik(pre, Q_DOWN, tcp=True)
for i in range(3):
    code, err = a.move_joints(q, 3.0)
    if err < 0.01: break
print("fk hand:", np.round(a.fk()[0],4), "q:", np.round(a.arm_q(),3))
