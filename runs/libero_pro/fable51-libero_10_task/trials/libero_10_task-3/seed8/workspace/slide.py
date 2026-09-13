from robot import *
np.set_printoptions(suppress=True,precision=3)
r=Robot()
for d in ([0.02,0,0],[0.02,0,0],[0.02,0,0],[0,0,0.02],[0,0,0.03],[0,0,0.05]):
    q=r.arm_q(); r.move_q(q,0.6)
    p,R=r.tcp(q)
    sol=ik_near(r,p+np.array(d),R,q,max_dq=0.3,tries=6,avoid_collisions=False)
    if sol is None: print(d,'no ik'); break
    r.move_q(sol,1.5); q2=r.arm_q(); print(d,'-> tcp',r.tcp(q2)[0],'w',np.round(r.wrench(),1),'moved',np.round(r.tcp(q2)[0]-p,3))
