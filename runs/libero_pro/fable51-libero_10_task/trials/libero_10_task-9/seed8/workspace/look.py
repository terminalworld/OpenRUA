import sys
from common import *
c = Ctl("look"); r, sc = c.r, c.sc
scene_no_white(sc)
x, y, z = map(float, sys.argv[1:4])
q = c.ik_valid(np.array([x, y, z]), R_DOWN, seed=r.arm_q(), tries=8)
assert q is not None and c.goto_q(q, t=5.0), "look move failed"
r.report("look")
r.snap("robot0_eye_in_hand")
