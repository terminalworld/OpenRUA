from robot import *
np.set_printoptions(suppress=True,precision=3)
r=Robot(); q=r.arm_q(); p,R=r.tcp(q); print('tcp',p,'R z',R[:,2])
for dz in (0.04,0.08,0.12):
    sol=ik_near(r,p+[0,0,dz],R,q,max_dq=0.6,tries=10,avoid_collisions=True)
    print(dz,sol)
    if sol is not None:
        r.move_q(sol,3.0); q=sol
print('tcp',r.tcp()[0],'wrench',np.round(r.wrench(),2))
