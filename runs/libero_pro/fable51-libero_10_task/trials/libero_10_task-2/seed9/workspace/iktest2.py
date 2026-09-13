import numpy as np, rclpy, sys
from arm import Arm, JOINTS, TCP_OFF, quat_to_R
arm = Arm()
cands = [
 ("q=(1,0,0,0) z1.02", (-0.199,0.189,1.02), (1,0,0,0)),
 ("q cur-ish", (-0.199,0.189,1.02), (0.9996,0,-0.0284,0)),
 ("q yaw 45 (0.924,-0.383,0,0)", (-0.199,0.189,1.02), (0.9239,-0.3827,0,0)),
 ("q yaw -45", (-0.199,0.189,1.02), (0.9239,0.3827,0,0)),
 ("q=(1,0,0,0) z1.10", (-0.199,0.189,1.10), (1,0,0,0)),
 ("q=(1,0,0,0) y=0.10", (-0.199,0.10,1.02), (1,0,0,0)),
 ("q=(1,0,0,0) x=-0.1", (-0.1,0.189,1.02), (1,0,0,0)),
]
for label, tcp, q in cands:
    q = np.array(q)/np.linalg.norm(q)
    hand = np.asarray(tcp) - TCP_OFF * quat_to_R(*q)[:, 2]
    try:
        sol = arm.solve_ik(hand, q)
        print(label, "OK", [round(v,3) for v in sol])
    except SystemExit as e:
        print(label, "->", e)
rclpy.shutdown()
