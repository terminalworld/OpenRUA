import sys, numpy as np
from rlib import Robot
from cart import cart_path, current_tcp, run
r = Robot()
dz = float(sys.argv[1])
pos, quat = r.fk_world()
from scipy.spatial.transform import Rotation
R0 = Rotation.from_quat(quat).as_matrix()
tcp0 = current_tcp(r, R0)
tcp1 = tcp0 + np.array([0, 0, dz])
qs = cart_path(r, tcp0, R0, tcp1, R0, 4)
if qs is None: raise SystemExit("ik fail")
q0 = np.array(r.joints()); dq = np.abs(np.array(qs[-1]) - q0)
secs = max(5, 6*dq[6]+2, dq.max()*4)
ok = run(qs, r, secs)
print("ok", ok, "tcp", np.round(current_tcp(r, R0), 3), "q", np.round(r.joints(), 2))
