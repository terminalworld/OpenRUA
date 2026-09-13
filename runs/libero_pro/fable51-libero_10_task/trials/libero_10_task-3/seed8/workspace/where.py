from robot import *
from robot import state_valid
np.set_printoptions(suppress=True,precision=3)
r=Robot(); q=r.arm_q(); print('q',q); print('valid',state_valid(r,q))
for n,(p,R) in fk_links(r,q).items(): print(n,p)
p,R=r.tcp(q); print('tcp',p); print('R\n',R)
print('fingers',r.fingers())
