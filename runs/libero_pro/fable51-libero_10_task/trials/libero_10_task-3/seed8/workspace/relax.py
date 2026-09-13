from robot import *
np.set_printoptions(suppress=True,precision=3)
r=Robot(); q=r.arm_q(); print('q',q,'w',np.round(r.wrench(),1))
r.move_q(q,1.0); print('after hold q',r.arm_q(),'w',np.round(r.wrench(),1),'tcp',r.tcp()[0])
q=r.arm_q(); p,R=r.tcp(q)
for i in range(5):
    sol=ik_near(r,p+[0,0,0.01*(i+1)],R,q,max_dq=0.3,tries=6,avoid_collisions=False)
    if sol is None: print('no ik'); break
    r.move_q(sol,1.0); q=r.arm_q(); print(' tcp',r.tcp()[0],'w',np.round(r.wrench(),1),'dq',np.round(q-sol,3))
