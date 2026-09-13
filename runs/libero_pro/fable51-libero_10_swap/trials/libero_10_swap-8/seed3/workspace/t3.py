from rob import *
r = Robot()
q = r.arm_q()
# pre-grasp test pose for pot A: side approach from -x, pitch 12
R = side_grasp_R(0, 12)
tcp = np.array([-0.199-0.008-0.10, -0.200, 0.894+0.04+0.05])
sol = r.ik_tcp(tcp, R, seed=q)
print("sol", None if sol is None else np.round(sol,3))
if sol is not None:
    t, R2 = r.tcp(sol); print("fk tcp of sol", t.round(4), "target", tcp.round(4)); print(R2.round(3)); print(R.round(3))
