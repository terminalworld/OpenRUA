from lib import *
r = Robot("t1")
print("q", np.round(r.arm_q(),3))
pos, quat = r.fk_world()
print("hand world", np.round(pos,4), np.round(quat,4))
print("tcp world", np.round(r.tcp_world()[0],4))
print("finger gap", r.finger_gap())
for yaw in (0, 90):
    q = down_quat(yaw); print("down_quat", yaw, np.round(q,4))
# IK feasibility probes
for name, p in [("aboveA", (-0.206,-0.195,1.20)), ("graspA", (-0.206,-0.195,1.00)),
                ("stoveC", (0.188,0.03,1.00)), ("stoveNear", (0.14,0.03,1.00)), ("stoveFar", (0.24,0.03,1.00)),
                ("stoveC_hi", (0.188,0.03,1.10)), ("aboveB", (-0.068,0.233,1.20)), ("graspB", (-0.068,0.233,1.00))]:
    for yaw in (90,):
        q = r.ik_world(p, down_quat(yaw))
        print(name, yaw, None if q is None else np.round(q,3))
