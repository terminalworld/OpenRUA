import sys, numpy as np
sys.path.insert(0, "/workspace")
from lib import Robot, Q_FINGERS_Y
r = Robot()
x, y = -0.445, -0.0915
w0 = r.wrench(); print("wrench start", np.round(w0, 2), "fingers", np.round(r.fingers(), 4))
for z in [float(v) for v in sys.argv[1:]]:
    q = r.solve_ik([x, y, z], Q_FINGERS_Y, tries=8)
    if q is None: print("IK fail at", z); break
    for a in range(3):
        code, err = r.move_joints(q, 2.0)
        if err < 0.02: break
    r.spin(0.3)
    w = r.wrench()
    print(f"z={z:.3f} code={code} err={err:.4f} pose={np.round(r.fk_pose()[:3],4)} fingers={np.round(r.fingers(),4)} wrench={np.round(w,2)} dF={np.round(w[:3]-w0[:3],2)}")
r.close()
