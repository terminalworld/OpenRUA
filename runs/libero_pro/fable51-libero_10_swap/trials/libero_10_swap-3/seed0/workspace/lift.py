from robot import *
r = Robot("lift")
Q = (0.7071, 0.7071, 0, 0)
t,_ = r.tcp()
code, err = r.move_tcp_line([t[0], t[1], 1.06], Q, 3.0)
print("lift code", code, "err", round(err,4), "tcp", np.round(r.tcp()[0],4))
print("fingers", r.fingers(), "wrench", np.round(r.wrench()[0],3))
