from robot import *
r = Robot("setdown")
Q = (0.7071, 0.7071, 0, 0)
def rep(tag, code, err):
    print(tag, "code", code, "err", round(err,4), "tcp", np.round(r.tcp()[0],4), "fingers", np.round(r.fingers(),4), "wrench", np.round(r.wrench()[0],2), flush=True)
rep("move", *r.move_tcp_line([-0.189, 0.039, 1.08], Q, 3.0))
rep("descend", *r.move_tcp_line([-0.189, 0.039, 0.985], Q, 2.0))
rep("descend2", *r.move_tcp_line([-0.189, 0.039, 0.938], Q, 2.0))
print("open", r.gripper(0.04), flush=True)
rep("lift", *r.move_tcp_line([-0.189, 0.039, 1.0], Q, 2.0))
