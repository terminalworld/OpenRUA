import sys, numpy as np
sys.argv=[sys.argv[0],""]
from ctl import Ctl, TCP
c=Ctl()
print("joints now", np.array(c.arm_q()).round(3))
q=c.solve_ik((-0.21,-0.126,0.47+TCP),(1,0,0,0)); print("ik sol", np.array(q).round(3) if q else None)
