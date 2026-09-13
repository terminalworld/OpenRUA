import sys
from moka_plan import *
m = Mover("probe")
p0,q0 = m.fk_world(); print("hand now", np.round(p0,4)); w0 = wrench(m); print("wrench before", np.round(w0,2))
tx,ty,tz = [float(a) for a in sys.argv[1:4]]
t = np.array([tx,ty,tz]); move_interp(m, t, Q_GRASP, n=3, seconds=3.0)
p1,q1 = m.fk_world(); print("hand after", np.round(p1,4), "q", np.round(m.arm_q(),3), "fingers", np.round(m.fingers(),4))
w1 = wrench(m); print("wrench after", np.round(w1,2), "delta", np.round(np.array(w1)-np.array(w0),2))
