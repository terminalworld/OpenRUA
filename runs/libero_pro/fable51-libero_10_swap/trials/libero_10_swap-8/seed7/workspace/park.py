import sys
from rob import *
r = Robot()
tcp = tuple(map(float, sys.argv[1:4])) if len(sys.argv) >= 4 else (0.0, -0.30, 1.25)
q = r.ik_tcp(tcp, grasp_R(-math.pi/2, 0), seed=r.arm_q())
if q is None: raise SystemExit("IK failed")
code, err = r.move_q(q, 4.0)
if err > 0.02: code, err = r.move_q(q, 4.0)
print("fk", np.round(r.fk_hand()[2], 4), "fingers", np.round(r.fingers(), 4))
