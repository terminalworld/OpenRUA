from rlib import *
r = Robot()
R = np.array([[-1,0,0],[0,1,0],[0,0,-1]], float)
quat = quat_from_R(R); print("quat", quat)
mug = np.array([-0.118, -0.268]); rim_z = 1.01
tcp = np.array([mug[0], mug[1]-0.044, 1.12])
hand = hand_pose_from_tcp(tcp, R)
q = r.ik_world(hand, quat)
print("IK:", None if q is None else np.round(q,3))
if q is not None:
    r.move_joints([q], 4.0)
    print("hand now", np.round(r.fk_world()[0],3), "target", np.round(hand,3))
