import sys
from moka_plan import *
m = Mover("retreat")
p0,q0 = m.fk_world(); print("hand now", np.round(p0,4))
back = float(sys.argv[1]); up = float(sys.argv[2])
t = p0 - back*np.array(DP); t[2] += up
r = move_interp(m, t, Q_PLACE, n=3, seconds=3.0)
p1,q1 = m.fk_world(); print("hand after", np.round(p1,4), "fingers", np.round(m.fingers(),4), "wrench", np.round(wrench(m),2))
