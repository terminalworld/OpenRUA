from arm import *
a = Arm()
plate = np.array([0.147, -0.019])
tcp_xy = (plate[0], plate[1] - 0.045)
for z, secs in ((0.68, 3.5), (0.575, 2.0)):
    q = a.ik((tcp_xy[0], tcp_xy[1], z), Q_DOWN, tcp=True)
    if q is None: raise SystemExit("ik fail")
    for i in range(3):
        code, err = a.move_joints(q, secs)
        if err < 0.01: break
    print("hand:", np.round(a.fk()[0],4), "fingers:", np.round(a.fingers(),4))
