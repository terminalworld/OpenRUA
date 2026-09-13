from robot import *
r = Robot("backoff"); Q = np.load("Qpush.npy")
code, err = r.move_tcp_line([-0.19, -0.20, 0.972], Q, 1.5)
print("backoff", code, round(err,4), "tcp", np.round(r.tcp()[0],4), "wrench", np.round(r.wrench()[0],2))
