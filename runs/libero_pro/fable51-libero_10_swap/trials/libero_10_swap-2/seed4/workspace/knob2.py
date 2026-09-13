from rk import *
import sys
m = Mover("knob2")
delta = float(sys.argv[1])
q = m.arm_q(); q2 = list(q); q2[6] = q[6] + delta
print("turn j7", round(q[6],3), "->", round(q2[6],3)); m.goto_q(q2, 3.0)
print("q after", np.round(m.arm_q(),3), "fingers", np.round(m.fingers(),4))
