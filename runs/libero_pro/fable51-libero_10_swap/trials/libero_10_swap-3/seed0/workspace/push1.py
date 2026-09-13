from robot import *
r = Robot("push1")
Qd = (0.7071,0.7071,0,0); Qp = (0.5,-0.5,0.5,0.5)
def rep(tag, code, err):
    print(tag, "code", code, "err", round(err,4), "tcp", np.round(r.tcp()[0],4), "q", np.round(r.arm_q(),3), "wrench", np.round(r.wrench()[0],2), flush=True)
print("close", r.gripper(0.0), flush=True)
rep("up", *r.move_tcp_line([-0.19,0.05,1.25], Qd, 3.0))
qb = list(np.load("qb.npy"))
for i in range(4):
    code, err = r.move_joints([qb], 12.0 if i==0 else 4.0)
    rep(f"reconf{i}", code, err)
    if err < 0.01: break
rep("down", *r.move_tcp_line([-0.19,0.05,0.965], Qp, 3.0))
rep("fwd", *r.move_tcp_line([-0.19,-0.03,0.965], Qp, 2.0))
