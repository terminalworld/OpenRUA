import numpy as np, rob, sys
r = rob.Robot()
p, q0 = r.fk(); print("start tcp", rob.tcp_now(r).round(4), "fingers", r.fingers(), flush=True)
Qg = q0
Qf = rob.frame_quat([0, 0, -1], [0, -1, 0])   # hand down, x_h=+x, fingers along y
print("== lift to 1.10", flush=True)
if rob.go_tcp(r, [-0.24, 0.168, 1.10], Qg, steps=3, seconds=4.0, max_jump=0.8) is None: sys.exit("fail")
print("fingers", r.fingers(), flush=True)
r.snap("agentview", "av7.png")
print("== yaw to x", flush=True)
if rob.go_tcp(r, [-0.24, 0.168, 1.10], Qf, steps=8, seconds=14.0, max_jump=0.9) is None: sys.exit("fail")
print("fingers", r.fingers(), "q", np.round(r.arm_q(),2), flush=True)
r.snap("agentview", "av8.png"); r.snap("birdview", "bv8.png")
print("== translate over drawer", flush=True)
if rob.go_tcp(r, [0.065, 0.105, 1.10], Qf, steps=6, seconds=8.0, max_jump=0.8) is None: sys.exit("fail")
print("fingers", r.fingers(), "q", np.round(r.arm_q(),2), flush=True)
r.snap("agentview", "av9.png"); r.snap("frontview", "fv9.png")
print("DONE", flush=True)
