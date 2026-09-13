from arm import *
a = Arm()
plate = np.array([0.147, -0.019])
tcp = (plate[0] + 0.045, plate[1])
for z, secs in ((0.66, 2.5), (0.545, 2.0)):
    q = a.ik((tcp[0], tcp[1], z), Q_DOWN_X, tcp=True)
    if q is None: raise SystemExit("ik fail")
    for i in range(3):
        code, err = a.move_joints(q, secs)
        if err < 0.01: break
    print("hand:", np.round(a.fk()[0],4), "fingers:", np.round(a.fingers(),4))
a.gripper(0.04)
q = a.ik((tcp[0], tcp[1], 0.75), Q_DOWN_X, tcp=True)
for i in range(3):
    code, err = a.move_joints(q, 2.5)
    if err < 0.01: break
print("hand:", np.round(a.fk()[0],4))
