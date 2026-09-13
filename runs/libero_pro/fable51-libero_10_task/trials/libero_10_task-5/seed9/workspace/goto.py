"""python3 goto.py x y z [qx qy qz qw] [seconds]  -- IK (link8, world) + trajectory, retry until converged"""
import sys, numpy as np
sys.path.insert(0, "/workspace")
from lib import Robot, Q_FINGERS_Y
a = [float(x) for x in sys.argv[1:]]
pos = a[:3]; quat = a[3:7] if len(a) >= 7 else Q_FINGERS_Y; secs = a[7] if len(a) >= 8 else (a[3] if len(a) == 4 else 3.0)
r = Robot()
q = r.solve_ik(pos, quat, tries=8)
if q is None:
    print("IK FAILED"); r.close(); sys.exit(1)
print("target joints", np.round(q, 4))
for attempt in range(4):
    code, err = r.move_joints(q, secs)
    print(f"attempt {attempt}: code {code} max joint err {err:.4f}")
    if err < 0.02: break
    secs = max(secs, 3.0)
print("link8 pose", np.round(r.fk_pose(), 4), "fingers", np.round(r.fingers(), 4))
r.close()
