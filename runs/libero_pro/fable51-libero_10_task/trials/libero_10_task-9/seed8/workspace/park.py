import sys
from common import *
c = Ctl("park"); r, sc = c.r, c.sc
scene_no_white(sc)
Rf = hand_R(np.pi / 2)
p, _ = r.fk()
P = np.array([0.05, -0.20, 1.35])
q = c.ik_valid(P, Rf, seed=r.arm_q(), tries=8)
assert q is not None and c.goto_q(q, t=5.0), "park failed"
r.report("parked")
for cam in sys.argv[1:] or ("birdview", "sideview", "agentview"):
    r.snap(cam)
