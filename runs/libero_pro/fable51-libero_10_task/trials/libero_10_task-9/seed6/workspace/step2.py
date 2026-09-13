"""Step 2: straight-line approach to the handle bar and close the gripper."""
import numpy as np, sys
from plan import *
r = Planner("step2")
p0, R = r.tcp(); approach = R[:,2]
bar = np.array([-0.080, -0.207, 0.947])
target = bar + float(sys.argv[1] if len(sys.argv)>1 else 0.0)*approach   # optional extra depth along approach
print("tcp", np.round(p0,3), "-> target", np.round(target,3))
ok = r.move_line_tcp([target], R, avoid=True)
if not ok:
    print("retrying without collision checking"); ok = r.move_line_tcp([target], R, avoid=False)
print("approach ok", ok)
