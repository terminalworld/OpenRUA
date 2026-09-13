import numpy as np, rob, sys
r = rob.Robot()
q = r.arm_q(); print("start", np.round(q,3), flush=True)
home = [0, -0.13, 0, -2.48, 0, 2.34, 0.79]
s1 = list(q); s1[1] = 0.3; s1[3] = -2.9; s1[5] = 1.2
s2 = list(s1); s2[0] = 0.0; s2[2] = 0.0; s2[4] = 0.0
for tag, qq in [("s1", s1), ("s2", s2), ("home", home)]:
    print("==", tag, flush=True)
    r.move_q(qq, 6.0)
    err = rob.settle(r, qq, seconds=3.0)
    print("  settled err", round(err,4), "tcp", rob.tcp_now(r).round(3), flush=True)
r.snap("birdview", "bv3.png"); r.snap("agentview", "av3.png")
print("DONE", flush=True)
