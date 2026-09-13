from robot import *
r = Robot("step1")
Q = (0.7071, 0.7071, 0, 0)
q = r.ik_tcp([-0.189, 0.022, 1.05], Q)
print("target q", np.round(q,3))
code, err = r.move_joints([q], 4.0)
print("code", code, "max joint err", round(err,4))
tcp, quat = r.tcp(); print("tcp now", np.round(tcp,4), np.round(quat,4))
