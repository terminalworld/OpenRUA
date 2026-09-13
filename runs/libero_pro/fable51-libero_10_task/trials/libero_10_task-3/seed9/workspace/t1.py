import numpy as np
from rob import *
r = Robot("t1")
q = r.arm_q(); print("q", np.round(q,3))
pos, R = r.fk_hand(); print("hand world", np.round(pos,4)); print(np.round(R,3))
tcp, _ = r.tcp(); print("tcp world", np.round(tcp,4))
T, Rc = r.cam_tf("robot0_eye_in_hand"); print("eih cam", np.round(T,4)); print(np.round(Rc,3))
# camera pose relative to hand
Rh_c = R.T @ Rc; th_c = R.T @ (T - pos)
print("hand->cam t", np.round(th_c,4)); print(np.round(Rh_c,3))
print("gap", r.finger_gap(), "force", r.force())
