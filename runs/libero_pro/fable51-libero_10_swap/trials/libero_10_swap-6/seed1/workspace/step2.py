from arm import *
a = Arm()
c = np.array([-0.110, -0.176]); r = 0.045
pre = (c[0], c[1] - r, 0.62)
a.move_to(pre, Q_DOWN, seconds=3.0, tcp=True)
print("fk hand:", np.round(a.fk()[0],4))
