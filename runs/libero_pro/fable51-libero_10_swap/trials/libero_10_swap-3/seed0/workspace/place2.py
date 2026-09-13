import sys
from robot import *
r = Robot("place2")
Q = (0.7071, 0.7071, 0, 0)
f0 = r.wrench()[0]
def rep(tag, code, err, stop=True):
    t = r.tcp()[0]; f = r.fingers(); w = r.wrench()[0]
    print(tag, "code", code, "err", round(err,4), "tcp", np.round(t,4), "fingers", np.round(f,4), "wrench", np.round(w,2), flush=True)
    if stop and (np.abs(w - f0).max() > 3.0 or abs(f[0]) < 0.0005):
        print("ABORT: contact or lost grasp", flush=True); sys.exit(1)
X = -0.156
rep("transport", *r.move_tcp_line([X, -0.05, 1.08], Q, 3.0))
rep("approach", *r.move_tcp_line([X, -0.138, 1.035], Q, 2.5))
rep("descend a", *r.move_tcp_line([X, -0.138, 1.015], Q, 1.5))
rep("descend b", *r.move_tcp_line([X, -0.138, 0.997], Q, 1.5))
rep("slide", *r.move_tcp_line([X, -0.155, 0.997], Q, 1.5))
