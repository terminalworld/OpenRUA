from arm import *
a = Arm()
g = (0.186, -0.055)
q = a.ik((g[0], g[1], 0.66), Q_DOWN_X, tcp=True)
for i in range(3):
    code, err = a.move_joints(q, 2.5)
    if err < 0.01: break
print("hand:", np.round(a.fk()[0],4), "fingers:", np.round(a.fingers(),4))
