import sys, numpy as np
sys.argv=[sys.argv[0]]
import ctl
c=ctl.Ctl(); c.spin(1)
q=c.solve_ik((-0.080,-0.161,0.455),0)
cur=np.array(c.arm_q())
print("cur ",np.round(cur,3).tolist()); print("goal",np.round(q,3).tolist()); print("diff",np.round(np.array(q)-cur,3).tolist())
