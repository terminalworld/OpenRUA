"""Checked park: python3 park2.py [x y z]  (default -0.25 -0.30 1.25), home orientation; refuses branch jumps."""
import sys
from rob import *
HOME=[0.0,-0.161037389,0.0,-2.44459747,0.0,2.2267522,0.7853981633974483]
tcp = tuple(map(float, sys.argv[1:4])) if len(sys.argv) >= 4 else (-0.25, -0.30, 1.25)
r = Robot(); q0 = r.arm_q()
q = r.ik_tcp(tcp, grasp_R(-math.pi/2, 0), seed=q0)
if q is None or np.max(np.abs(np.array(q)-np.array(q0))) > 1.8:
    q = r.ik_tcp(tcp, grasp_R(-math.pi/2, 0), seed=HOME)
    if q is None or np.max(np.abs(np.array(q)-np.array(q0))) > 1.8:
        print("no continuous IK; going HOME instead", flush=True); q = HOME
print("target q", np.round(q,2), "jump", np.round(np.max(np.abs(np.array(q)-np.array(q0))),2), flush=True)
code, err = r.move_q(q, 4.0)
if err > 0.02: code, err = r.move_q(q, 4.0)
print("fk", np.round(r.fk_hand()[2],4), flush=True)
