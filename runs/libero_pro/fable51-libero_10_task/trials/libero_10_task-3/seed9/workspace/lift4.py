import numpy as np
from rob import *
from goto import goto
r = Robot("l4")
t0, R0 = r.tcp()
goto(r, t0 + np.array([0,0,0.13]), R0, seconds=4, via_n=2, resend=1)
print("gap", round(r.finger_gap(),4), "force", np.round(r.force(),2))
r.snap("sideview"); r.snap("agentview")
