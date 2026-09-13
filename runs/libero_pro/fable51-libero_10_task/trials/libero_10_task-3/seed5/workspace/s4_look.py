import numpy as np, rob, sys
r = rob.Robot()
Q = rob.frame_quat([0, 0, -1], [0, -1, 0])
q = r.move_pose(rob.hand_from_tcp([-0.30, -0.02, 1.15], Q), Q, seconds=5.0, max_jump=1.5)
rob.settle(r, q); print("tcp", rob.tcp_now(r).round(3), "q", np.round(r.arm_q(),2), flush=True)
r.snap("birdview", "bv5.png")
print("DONE", flush=True)
