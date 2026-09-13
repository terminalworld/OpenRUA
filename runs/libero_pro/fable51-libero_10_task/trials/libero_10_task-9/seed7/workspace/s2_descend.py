import sys
from rlib import *
r = Robot()
R = np.array([[-1,0,0],[0,1,0],[0,0,-1]], float); quat = quat_from_R(R)
c = np.array([-0.1174, -0.2675])
z = float(sys.argv[1])
tcp = np.array([c[0], c[1]-0.044, z])
hand = hand_pose_from_tcp(tcp, R)
q = r.ik_world(hand, quat)
print("IK:", None if q is None else np.round(q,3))
if q is not None:
    for i in range(3):
        code, err = r.move_joints([q], 3.0)
        if err < 0.01: break
    pos, qt = r.fk_world()
    print("hand", np.round(pos,4), "tcp", np.round(pos + 0.1034*R[:,2],4), "quat", np.round(qt,3))
