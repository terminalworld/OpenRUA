from robot import *
r = Robot("lift2")
Q = (0.7071, 0.7071, 0, 0)
t,_ = r.tcp()
code, err = r.move_tcp_line([t[0], t[1], 1.08], Q, 2.5)
print("lift code", code, "err", round(err,4), "tcp", np.round(r.tcp()[0],4), "fingers", np.round(r.fingers(),4), "wrench", np.round(r.wrench()[0],3))
code, err = r.move_tcp_line([-0.16, -0.05, 1.08], Q, 2.5)
print("shift code", code, "err", round(err,4), "tcp", np.round(r.tcp()[0],4), "fingers", np.round(r.fingers(),4), "wrench", np.round(r.wrench()[0],3))
