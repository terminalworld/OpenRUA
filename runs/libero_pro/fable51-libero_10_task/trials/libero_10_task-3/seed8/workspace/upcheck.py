from robot import *
from plan3 import check_path
np.set_printoptions(suppress=True,precision=3)
r=Robot(); q0=r.arm_q(); p,R=r.tcp(q0)
q1=ik_near(r,np.array([-0.12,0.12,1.42]),R,q0,max_dq=0.8,tries=10)
check_path(r,[q0,q1],n_sub=10,label='up-away')
r.move_q(q1, max(np.abs(q1-q0).max()/0.1,0.6))
q=r.arm_q(); print('tcp',r.tcp(q)[0],'w',np.round(r.wrench(),1))
np.save('q_view.npy',q)
