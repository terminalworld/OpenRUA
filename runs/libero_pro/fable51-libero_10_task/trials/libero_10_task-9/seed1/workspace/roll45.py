import numpy as np, rclpy
from scipy.spatial.transform import Rotation as Rot
from rob import Robot
r = Robot("roll45")
q0 = r.arm_q(); pos, quat = r.tcp(q0)
print("tcp now", pos.round(4), quat.round(4))
Rcur = Rot.from_quat(quat).as_matrix()
Rnew = Rot.from_euler('y', -45, degrees=True).as_matrix() @ Rcur
# check feature mapping: center offset (0.036,·,-0.017) in world at current pose -> hand frame -> new world
c_w = np.array([0.036, 0.0, -0.017]); c_h = Rcur.T @ c_w; print("center offset after roll:", (Rnew @ c_h).round(4))
h_w = np.array([np.cos(np.deg2rad(116)), 0, np.sin(np.deg2rad(116))]); print("handle dir after roll:", (Rnew @ Rcur.T @ h_w).round(3))
print("new y_h (finger axis):", Rnew[:, 1].round(3), " z_h:", Rnew[:, 2].round(3))
qn = Rot.from_matrix(Rnew).as_quat(); print("new quat", qn.round(4))
np.save("snaps/quat_ins.npy", qn)
q = r.ik(pos, qn, seed=q0)
print("ik", None if q is None else np.round(q, 3))
if q is not None:
    r.move_q_corrected(q, seconds=6.0)
    p2, q2 = r.tcp(); print("tcp after", p2.round(4), q2.round(4)); print("fingers", r.fingers())
