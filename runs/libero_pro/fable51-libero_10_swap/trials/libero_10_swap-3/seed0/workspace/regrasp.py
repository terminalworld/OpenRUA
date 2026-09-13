from robot import *
r = Robot("regrasp")
Q = (0.7071, 0.7071, 0, 0)
def rep(tag, code, err):
    print(tag, "code", code, "err", round(err,4), "tcp", np.round(r.tcp()[0],4), "fingers", np.round(r.fingers(),4), "wrench", np.round(r.wrench()[0],2), flush=True)
X, Y = -0.150-0.050, 0.031
rep("hover", *r.move_tcp_line([X, Y, 1.0], Q, 2.0))
rep("descend", *r.move_tcp_line([X, Y, 0.947], Q, 2.0))
print("close", r.gripper(0.0), flush=True)
rep("lift", *r.move_tcp_line([X, Y, 1.08], Q, 2.5))
