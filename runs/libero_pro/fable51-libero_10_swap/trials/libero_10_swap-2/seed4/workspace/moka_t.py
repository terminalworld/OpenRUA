import sys
from moka_plan import *
m = Mover("moka_t")
POT_OFF2 = 0.109
dry = "dry" in sys.argv
p0,q0 = m.fk_world(); print("hand now", np.round(p0,4)); w0 = wrench(m)
z = float(sys.argv[1])
goal = np.array([B[0] - POT_OFF2*DP[0], B[1] - POT_OFF2*DP[1], z])
print("goal", np.round(goal,4))
r = move_interp(m, goal, Q_PLACE, n=8, seconds=6.0, dry=dry)
p1,q1 = m.fk_world(); print("hand after", np.round(p1,4), "q", np.round(m.arm_q(),3), "fingers", np.round(m.fingers(),4))
w1 = wrench(m); print("wrench after", np.round(w1,2), "delta", np.round(np.array(w1)-np.array(w0),2))
