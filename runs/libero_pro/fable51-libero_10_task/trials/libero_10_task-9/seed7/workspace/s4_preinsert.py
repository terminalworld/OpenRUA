import sys
from cart import *
r = Robot()
Rf = np.array([[-1,0,0],[0,0,1],[0,1,0]], float)
tcp0 = current_tcp(r, Rf); print("tcp now", np.round(tcp0,3))
goal = np.array([float(v) for v in sys.argv[1:4]])
via = np.array([goal[0], min(goal[1], -0.03), 1.25]) if len(sys.argv) > 4 else None
if via is not None:
    qs = cart_path(r, tcp0, Rf, via, Rf, 3); run(qs, r, 4.0)
    tcp0 = current_tcp(r, Rf)
qs = cart_path(r, tcp0, Rf, goal, Rf, 4); run(qs, r, 4.0)
pos, qt = r.fk_world(); print("tcp", np.round(current_tcp(r, Rf),4), "quat", np.round(qt,3), "fingers", r.fingers())
