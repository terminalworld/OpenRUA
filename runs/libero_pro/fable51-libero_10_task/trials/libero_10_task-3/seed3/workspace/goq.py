import sys, numpy as np
from panda import *
np.set_printoptions(precision=4, suppress=True)
r = Robot()
qt = np.array([float(x) for x in sys.argv[1].split(',')])
T = float(sys.argv[2]) if len(sys.argv) > 2 else 4.0
print(r.move_q(qt, T))
q = r.q(); print('q', q.round(3), 'err', (q-qt).round(3)); print('tcp', fk_tcp(q)[:3,3].round(4))
