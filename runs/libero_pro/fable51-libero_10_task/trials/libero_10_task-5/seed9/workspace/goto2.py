"""python3 goto2.py x y z QNAME [seconds] -- IK+trajectory with named orientation from lib"""
import sys, numpy as np
sys.path.insert(0, "/workspace")
import lib
from lib import Robot
pos = [float(x) for x in sys.argv[1:4]]; quat = getattr(lib, sys.argv[4]); secs = float(sys.argv[5]) if len(sys.argv) > 5 else 3.0
r = Robot()
q = r.solve_ik(pos, quat, tries=10)
if q is None:
    print("IK FAILED"); r.close(); sys.exit(1)
print("target joints", np.round(q, 4))
for attempt in range(4):
    code, err = r.move_joints(q, secs)
    print(f"attempt {attempt}: code {code} max joint err {err:.4f}")
    if err < 0.02: break
    secs = max(secs, 3.0)
print("link8 pose", np.round(r.fk_pose(), 4), "fingers", np.round(r.fingers(), 4), "wrench", np.round(r.wrench(), 2))
r.close()
