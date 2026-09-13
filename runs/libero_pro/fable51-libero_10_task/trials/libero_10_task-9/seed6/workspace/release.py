import sys, numpy as np
from rob import *; from plan import Planner
r = Planner("release"); r.gripper(0.04)
tcp, R = r.tcp(); a = R[:,2]
ok = r.move_line_tcp([tcp - 0.06*a], R, avoid=False); print("retreat ok", ok)
tcp, R = r.tcp(); ok = r.move_line_tcp([tcp + [0,0,0.08]], R, avoid=False); print("up ok", ok)
