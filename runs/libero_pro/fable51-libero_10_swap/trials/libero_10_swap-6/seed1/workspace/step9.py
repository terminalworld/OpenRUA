from arm import *
a = Arm()
g = (-0.210, 0.025)
tgt = (0.147, 0.116)
q = a.ik((g[0], g[1], 0.68), Q_DOWN, tcp=True)
for i in range(3):
    code, err = a.move_joints(q, 2.5)
    if err < 0.01: break
print("hand:", np.round(a.fk()[0],4), "fingers:", np.round(a.fingers(),4))
for z, secs in ((0.68, 3.5), (0.455, 2.0)):
    q = a.ik((tgt[0], tgt[1], z), Q_DOWN, tcp=True)
    if q is None: raise SystemExit("ik fail")
    for i in range(3):
        code, err = a.move_joints(q, secs)
        if err < 0.01: break
    print("hand:", np.round(a.fk()[0],4), "fingers:", np.round(a.fingers(),4))
