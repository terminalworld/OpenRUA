from rob import *
r = Robot()
R = xbar_R(0.52)
print("xbar_R(30deg)=\n", np.round(R,3))
seed = r.arm_q()
for name, tcp, t in [("A bar", (0.06,0.002,1.057),0.52), ("A bar pre",(0.06,0.002,1.15),0.52),
                     ("place .18 z1.09",(0.18,0.0,1.09),0.52), ("place .18 z1.06",(0.18,0.0,1.06),0.52), ("place .18 z1.20",(0.18,0.0,1.20),0.52),
                     ("place .19 z1.09",(0.19,0.0,1.09),0.52), ("place .18 y.097 z1.09",(0.18,0.097,1.09),0.52), ("place .18 y.097 z1.20",(0.18,0.097,1.20),0.52),
                     ("place .18 z1.09 t40",(0.18,0.0,1.09),0.70), ("place .18 z1.20 t40",(0.18,0.0,1.20),0.70),
                     ("B bar pre t0",(-0.076,0.164,1.15),0.0)]:
    q = r.ik_tcp(tcp, xbar_R(t), seed=seed)
    if q is not None:
        pos, quat, tc = r.fk_hand(q); Rh = R_from_q(*quat)
        print(f"{name}: q={np.round(q,3)} tcp_err={np.linalg.norm(tc-np.array(tcp))*1000:.1f}mm hz={np.round(Rh[:,2],2)} hy={np.round(Rh[:,1],2)}")
    else: print(name, None)
