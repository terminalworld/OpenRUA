from robot import *
np.set_printoptions(suppress=True,precision=3)
r=Robot(); q=r.arm_q(); print('q',q,'tcp',r.tcp()[0],'w',np.round(r.wrench(),1))
q2=q.copy(); q2[1]-=0.08; q2[3]-=0.05
r.move_q(q2,2.0); print('q',r.arm_q(),'tcp',r.tcp()[0],'w',np.round(r.wrench(),1))
