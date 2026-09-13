import numpy as np, cv2
from rob import *
r = Robot("peek")
q0 = r.arm_q()
d = np.array([0, np.cos(np.radians(30)), -np.sin(np.radians(30))])  # optical axis
Y = np.array([1.0, 0, 0]); X = np.cross(Y, d); R = np.stack([X, Y, d], 1)
cam_target = np.array([0.0, 0.0, 1.12])
hand = cam_target - 0.05 * X
q = r.ik_hand(hand, R, seed=q0)
print("ik", None if q is None else np.round(q, 3))
if q is None: raise SystemExit
r.move_q(q, 4.0)
pos, Rh = r.fk_hand(); print("hand", np.round(pos, 3)); print(np.round(Rh, 2))
img, P = r.cloud("robot0_eye_in_hand")
np.save("snaps/peek_xyz.npy", P)
Z = P[..., 2]; Xw = P[..., 0]; Yw = P[..., 1]
m = (np.abs(Z - 0.925) < 0.01) & (Xw > -0.1) & (Xw < 0.1) & (Yw > 0.05)
print("drawer floor pts:", m.sum(), "y range", Yw[m].min() if m.any() else None, Yw[m].max() if m.any() else None)
for ylo in np.arange(0.10, 0.45, 0.02):
    mm = (Xw > -0.1) & (Xw < 0.1) & (Yw >= ylo) & (Yw < ylo + 0.02) & (Z > 0.90) & (Z < 1.0)
    if mm.any():
        print(f"  y {ylo:.2f}: n={mm.sum():5d} z[{Z[mm].min():.3f},{Z[mm].max():.3f}]  hist", np.histogram(Z[mm], bins=[0.9,0.93,0.95,0.97,0.99,1.0])[0])
