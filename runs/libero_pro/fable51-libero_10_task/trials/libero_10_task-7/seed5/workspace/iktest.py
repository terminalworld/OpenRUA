import sys, numpy as np
sys.argv=[sys.argv[0],""]
from ctl import Ctl, quat_R, TCP
c=Ctl()
hand,tcp,R=c.pose()
tests=[("current hand",hand,(1,0,-0.028,0)),
       ("current hand pure down",hand,(1,0,0,0)),
       ("ketchup hi",np.array([-0.21,-0.126,0.62+TCP]),(1,0,0,0)),
       ("ketchup hi yaw45",np.array([-0.21,-0.126,0.62+TCP]),(0.924,0.383,0,0)),
       ("ketchup hi yaw-45",np.array([-0.21,-0.126,0.62+TCP]),(0.924,-0.383,0,0)),
       ("center hi",np.array([-0.1,0.0,0.62+TCP]),(1,0,0,0)),
]
for name,p,q in tests:
    print(name, p.round(3)); c.solve_ik(p,q)
