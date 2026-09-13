from arm import *
a = Arm()
g = (-0.1066, -0.2214)
# descend in two steps for accuracy
for z in (0.57, 0.524):
    q = a.ik((g[0], g[1], z), Q_DOWN, tcp=True)
    for i in range(3):
        code, err = a.move_joints(q, 2.0)
        if err < 0.01: break
    print("hand:", np.round(a.fk()[0],4))
f = a.gripper(0.0)
