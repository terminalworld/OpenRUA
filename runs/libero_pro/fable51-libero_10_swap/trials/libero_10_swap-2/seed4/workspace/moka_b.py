import sys
from moka_plan import *
m = Mover("moka_b")
dist = float(sys.argv[1])
p0,_ = m.fk_world()
target = p0 + dist*D; target[2] = GRASP_Z
print("advance", dist, "->", np.round(target,4)); print("wrench before", wrench(m))
r = move_interp(m, target, Q_GRASP, n=3, seconds=2.5)
print("wrench after", wrench(m))
