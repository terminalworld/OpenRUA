from robot import *
r = Robot("retreat"); Q = np.load("Qpush.npy"); qa = list(np.load("qa.npy"))
code, err = r.move_tcp_line([-0.19, 0.0, 0.972], Q, 3.0); print("back", code, round(err,4), np.round(r.tcp()[0],3), flush=True)
code, err = r.move_tcp_line([-0.19, 0.0, 1.10], Q, 2.0, avoid=True); print("up", code, round(err,4), np.round(r.tcp()[0],3), flush=True)
for i in range(4):
    code, err = r.move_joints([qa], 12.0 if i==0 else 4.0)
    print("home", i, code, round(err,4), np.round(r.arm_q(),3), "tcp", np.round(r.tcp()[0],3), "wrench", np.round(r.wrench()[0],2), flush=True)
    if err < 0.01: break
