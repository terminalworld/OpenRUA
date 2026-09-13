from robot import *
r = Robot("iktest")
Qp = (0.5,-0.5,0.5,0.5)
print(np.round(quat_to_R(*Qp),3))
seed = r.arm_q()
for p in [(-0.19,-0.02,0.955),(-0.19,-0.06,0.955),(-0.19,-0.14,0.955),(-0.19,-0.22,0.955),(-0.19,-0.02,1.05)]:
    q = r.ik_tcp(p, Qp, seed)
    print(p, None if q is None else np.round(q,3))
    if q is not None:
        t,qq = r.tcp(q); print("   fk tcp", np.round(t,4), np.round(qq,3))
