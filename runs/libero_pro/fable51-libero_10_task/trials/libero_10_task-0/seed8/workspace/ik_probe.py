import numpy as np
from rb import RB, q_down_yaw
r = RB("probe")
p, q, tcp = r.hand_pose()
r.log("hand", np.round(p,3), np.round(q,3), "tcp", np.round(tcp,3))
tests = [("current", tcp, q),
         ("cc pregrasp", [0.108,-0.211,0.55], q_down_yaw(0)),
         ("tomato pregrasp", [-0.12,0.047,0.60], q_down_yaw(0)),
         ("basket", [0.0,0.264,0.72], q_down_yaw(0)),
         ("park1", [-0.25,-0.45,0.75], q_down_yaw(0)),
         ("park2", [-0.1,-0.4,0.70], q_down_yaw(0)),
         ("park3", [0.0,-0.35,0.65], q_down_yaw(0)),
         ("home-ish", [-0.05,0.0,0.68], q_down_yaw(0)),
]
for name, xyz, qq in tests:
    try:
        sol = r.ik_world(xyz, qq)
        r.log(name, "OK", np.round(sol,3))
    except RuntimeError as e:
        r.log(name, e)
r.close()
