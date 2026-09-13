import sys, numpy as np
sys.path.insert(0, "/workspace")
from lib import Robot, Q_FINGERS_Y
r = Robot()
print("start joints", np.round(r.joints(), 3), "fingers", r.fingers())
print("link8 pose now", np.round(r.fk_pose(), 4))
target = [-0.114, 0.057, 1.14]
q = r.solve_ik(target, Q_FINGERS_Y, tries=5)
print("IK pregrasp:", None if q is None else np.round(q, 4))
if q is not None:
    print("FK of solution:", np.round(r.fk_pose(q), 4))
    code, err = r.move_joints(q, 4.0)
    print("moved code", code, "max joint err", round(err, 4))
    print("link8 pose after", np.round(r.fk_pose(), 4))
r.close()
