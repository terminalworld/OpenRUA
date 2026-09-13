from robot import *
r = Robot("probe3")
tcp, quat = r.tcp(); print("tcp now", np.round(tcp,4), np.round(quat,4))
for quat in [(0.7071,-0.7071,0,0),(0.7071,0.7071,0,0)]:
    q = r.ik_tcp([-0.189, 0.022, 1.05], quat)
    print("IK hover", quat, None if q is None else np.round(q,3))
    if q is not None:
        t, qq = r.tcp(q); print("   FK check tcp", np.round(t,4), np.round(qq,4))
        q2 = r.ik_tcp([-0.189, 0.022, 0.938], quat, q); print("   IK grasp depth", None if q2 is None else np.round(q2,3))
        q3 = r.ik_tcp([-0.16, -0.16, 0.955], quat, q); print("   IK place", None if q3 is None else np.round(q3,3))
