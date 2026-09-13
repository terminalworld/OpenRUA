from robot import *
r = Robot("step2")
Q = (0.7071, 0.7071, 0, 0)
gx, gy = -0.192, 0.041
code, err = r.move_tcp_line([gx, gy, 1.05], Q, 2.0)
print("shift code", code, "err", round(err,4), "tcp", np.round(r.tcp()[0],4))
code, err = r.move_tcp_line([gx, gy, 0.938], Q, 4.0)
print("descend code", code, "err", round(err,4), "tcp", np.round(r.tcp()[0],4), np.round(r.tcp()[1],3))
print("wrench", np.round(r.wrench()[0],3))
