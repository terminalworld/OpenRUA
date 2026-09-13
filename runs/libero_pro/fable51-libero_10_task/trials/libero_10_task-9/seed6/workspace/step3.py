"""Step 3: lift straight up with the mug."""
import numpy as np, sys
from plan import *
r = Planner("step3")
p0, R = r.tcp(); dz = float(sys.argv[1]) if len(sys.argv)>1 else 0.12
target = p0 + np.array([0,0,dz])
ok = r.move_line_tcp([target], R, avoid=True)
if not ok: print("retry w/o collision check"); ok = r.move_line_tcp([target], R, avoid=False)
print("lift ok", ok, "fingers", r.fingers())
