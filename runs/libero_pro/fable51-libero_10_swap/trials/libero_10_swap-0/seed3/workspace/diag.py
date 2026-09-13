from rob import *
r = Robot()
q0 = r.arm_q(); log("current q", np.round(q0,3))
_,_,tcp = r.hand_pose(); log("tcp", tcp.round(4))
seed=q0
for z in (0.50, 0.485, 0.47, 0.455):
    q = r.solve_ik(-0.244,-0.173,z, topdown_quat(0.0), seed=seed)
    log(z, None if q is None else np.round(q,3), "jump", None if q is None else np.round(max(abs(a-b) for a,b in zip(q,seed)),3))
    if q: seed=q
# also try alternative yaws for the grasp height, seeded from current
for yaw in (0.4, -0.4, 0.785, -0.785, 1.571):
    q = r.solve_ik(-0.244,-0.173,0.455, topdown_quat(yaw), seed=q0)
    log("yaw",yaw, None if q is None else np.round(q,3), "jump", None if q is None else np.round(max(abs(a-b) for a,b in zip(q,q0)),3))
