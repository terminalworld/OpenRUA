import numpy as np, rob, sys
r = rob.Robot("s11")
al = np.deg2rad(45)
Q45 = rob.frame_quat([0, np.sin(al), -np.cos(al)], [-1, 0, 0])
Qd = rob.frame_quat([0, 0, -1], [0, 1, 0])
print("== up/away", flush=True)
rob.go_tcp(r, [-0.10, 0.08, 1.22], Q45, steps=3, seconds=5.0, max_jump=0.9)
rob.go_tcp(r, [-0.28, -0.06, 1.25], Q45, steps=4, seconds=6.0, max_jump=0.9)
print("== hand down", flush=True)
rob.go_tcp(r, [-0.28, -0.06, 1.25], Qd, steps=6, seconds=8.0, max_jump=0.9)
print("q", np.round(r.arm_q(), 3), "wrench", np.round(r.wrench(), 2), flush=True)
for cam, out in [("birdview", "bv_final.png"), ("agentview", "av_final.png"), ("frontview", "fv_final.png")]:
    for _ in range(3):
        try:
            r.snap(cam, out); print("saved", out, flush=True); break
        except Exception as e:
            print("snap retry", cam, e, flush=True)
print("DONE", flush=True)
