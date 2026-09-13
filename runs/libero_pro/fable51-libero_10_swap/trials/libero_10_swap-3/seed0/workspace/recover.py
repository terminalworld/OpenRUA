from robot import *
r = Robot("recover")
qa = list(np.load("qa.npy"))
for i in range(3):
    code, err = r.move_joints([qa], 8.0 if i==0 else 4.0)
    print("back", i, code, round(err,4), np.round(r.arm_q(),3), "tcp", np.round(r.tcp()[0],3), "wrench", np.round(r.wrench()[0],2), flush=True)
    if err < 0.01: break
