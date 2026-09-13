from arm import *
a = Arm()
g = (0.186, -0.055)
for z, secs in ((0.60, 3.0), (0.532, 2.0)):
    q = a.ik((g[0], g[1], z), Q_DOWN_X, tcp=True)
    if q is None: raise SystemExit("ik fail")
    for i in range(3):
        code, err = a.move_joints(q, secs)
        if err < 0.01: break
    print("hand:", np.round(a.fk()[0],4))
a.gripper(0.0)
