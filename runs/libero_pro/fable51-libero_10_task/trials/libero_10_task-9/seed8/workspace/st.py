from lib import *
r = Robot("st")
q = r.arm_q(); print("q now", np.round(q,3))
tgt = np.array([0.035,0.915,-0.235,-1.539,1.606,1.358,1.479]); print("diff", np.round(np.array(q)-tgt,3))
r.report("now")
for c in ["agentview","frontview"]: r.snap(c)
