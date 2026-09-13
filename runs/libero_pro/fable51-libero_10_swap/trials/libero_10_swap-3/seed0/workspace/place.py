from robot import *
r = Robot("place")
Q = (0.7071, 0.7071, 0, 0)
tx, ty = -0.1467, -0.158
code, err = r.move_tcp_line([tx, ty, 1.06], Q, 4.0)
print("transport code", code, "err", round(err,4), "tcp", np.round(r.tcp()[0],4), "fingers", np.round(r.fingers(),4))
code, err = r.move_tcp_line([tx, ty, 1.0], Q, 2.5)
print("descend code", code, "err", round(err,4), "tcp", np.round(r.tcp()[0],4), np.round(r.tcp()[1],3), "fingers", np.round(r.fingers(),4))
print("wrench", np.round(r.wrench()[0],3))
