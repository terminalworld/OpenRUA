from robot import *
np.set_printoptions(suppress=True,precision=3)
r=Robot()
ax=np.array([-0.947,-0.322,0.0]); closing=np.cross([0,0,-1],ax)
R_G=rot_from_axes([0,0,-1],closing)
C=np.array([-0.2186,-0.0153,0.92])
rng=np.random.default_rng(0)
sols=[]
for i in range(25):
    seed=np.array([0,0.3,0,-2.3,0,2.6,0.46])+rng.normal(0,0.6,7)
    q=r.ik_tcp(C,R_G,seed=seed,avoid_collisions=(i%2==0))
    if q is not None: sols.append((abs(q[0])+abs(q[2])+abs(q[4]),i%2==0,q))
sols.sort(key=lambda s:s[0])
for s in sols[:10]: print(round(s[0],2),s[1],s[2])
