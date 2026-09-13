from robot import *
from robot import state_valid
r=Robot()
ax=np.array([-0.947,-0.322,0.0]); closing=np.cross([0,0,-1],ax)
R_G=rot_from_axes([0,0,-1],closing)
C=np.array([-0.2186,-0.0153,0.92]); PRE=C+[0,0,0.13]
for seed in [np.array([0,0.3,0,-2.3,0,2.6,0.46]),np.array([0,0.0,0,-2.4,0,2.4,0.46]),np.array([0.1,0.5,-0.1,-2.0,0,2.5,0.5]),np.array([0,0.3,0,-2.3,0,2.6,-2.68])]:
    for p in (PRE,C):
        q=r.ik_tcp(p,R_G,seed=seed,avoid_collisions=True)
        print(np.round(p,3),None if q is None else np.round(q,3))
