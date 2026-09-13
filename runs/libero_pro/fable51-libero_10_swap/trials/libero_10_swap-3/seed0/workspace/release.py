from robot import *
r = Robot("release")
Q = (0.7071, 0.7071, 0, 0)
print("open", r.gripper(0.04), flush=True)
def rep(tag, code, err):
    print(tag, "code", code, "err", round(err,4), "tcp", np.round(r.tcp()[0],4), "fingers", np.round(r.fingers(),4), "wrench", np.round(r.wrench()[0],2), flush=True)
rep("lift", *r.move_tcp_line([-0.156, -0.155, 1.08], Q, 2.0))
rep("back", *r.move_tcp_line([-0.156, -0.05, 1.10], Q, 2.0))
