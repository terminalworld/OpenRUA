import numpy as np
from robot import Robot, quat_R
r = Robot("check")
js = r.joints(); print({k: round(v,3) for k,v in js.items()})
hp,hq = r.fk_pose(); R=quat_R(*hq); print("hand", hp.round(4), "tcp", r.tcp_from_hand(hp,hq).round(4), "handX", R[:,0].round(3), "yaw_deg", round(np.degrees(np.arctan2(R[1,0],R[0,0])),1))
