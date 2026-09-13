"""Tilt the hand about the world x axis (through the TCP) by PHI degrees in steps, keeping TCP position.
Usage: s12_tilt.py <phi_deg_total> [steps]"""
import sys, math, numpy as np
sys.path.insert(0, "/workspace")
from rob import Robot, topdown_quat, qmul

phi = math.radians(float(sys.argv[1])); steps = int(sys.argv[2]) if len(sys.argv) > 2 else 3
r = Robot("s12")
Q0 = topdown_quat(math.pi / 2)
tcp0, q_now = r.tcp_pose_world()
print("start tcp", np.round(tcp0, 4), "quat", np.round(q_now, 3), "fingers", r.fingers())
# current tilt: recover from q_now relative to Q0 -> assume the hand is Rx(phi0)*Q0
# (just compose on top of what was requested before; caller keeps track of cumulative phi)
phi0 = 0.0
if len(sys.argv) > 3:
    phi0 = math.radians(float(sys.argv[3]))
for k in range(1, steps + 1):
    a = phi0 + phi * k / steps
    qx = (math.sin(a / 2), 0, 0, math.cos(a / 2))
    Q = qmul(qx, Q0)
    for t in range(3):
        j = r.ik_world(tcp0, Q, seed=r.joints())
        if j is None:
            print("IK fail at", math.degrees(a)); sys.exit(1)
        ptcp, pq = r.tcp_pose_world(j)
        dq = min(np.linalg.norm(np.array(pq) - np.array(Q)), np.linalg.norm(np.array(pq) + np.array(Q)))
        if np.linalg.norm(ptcp - tcp0) > 0.01 or dq > 0.05:
            print("FK mismatch", np.linalg.norm(ptcp - tcp0), dq); sys.exit(1)
        code, err = r.move_joints(j, 2.5)
        if err < 0.02:
            break
    tcp, qq = r.tcp_pose_world()
    print(f"tilt {math.degrees(a):.1f} deg: tcp {np.round(tcp,4)} quat {np.round(qq,3)} fingers {np.round(r.fingers(),4)}")
