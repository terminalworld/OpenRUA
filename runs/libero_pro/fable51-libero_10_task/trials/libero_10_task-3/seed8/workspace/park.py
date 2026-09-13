from robot import *
from robot import state_valid
r=Robot()
tcp=np.array([-0.15,-0.32,1.25])
q=best_ik(r,tcp,DOWN,n_rand=6)
print(np.round(q,3), state_valid(r,q))
qc=r.arm_q()
# check link clearance along interpolation
for t in np.linspace(0,1,11):
    qq=qc+(q-qc)*t; lc=link_clearance(r,qq); print(f'{t:.1f}',np.round(r.tcp(qq)[0],3),min(c for _,c,_ in lc), [n for n,c,i in lc if i])
