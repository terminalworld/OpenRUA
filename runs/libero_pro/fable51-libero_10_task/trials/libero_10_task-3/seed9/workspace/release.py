import numpy as np
from rob import *
from goto import goto
r = Robot("rel")
r.gripper(0.04)
r.spin(0.5)
t0, R0 = r.tcp(); print("tcp", np.round(t0,3), "force", np.round(r.force(),2))
goto(r, t0 + [0,0,0.10], R0, seconds=3, via_n=1, resend=1)
r.snap("agentview", "/workspace/snaps/av_after.png"); r.snap("frontview", "/workspace/snaps/fv_after.png"); r.snap("birdview", "/workspace/snaps/bv_after.png")
print("force", np.round(r.force(),2))
