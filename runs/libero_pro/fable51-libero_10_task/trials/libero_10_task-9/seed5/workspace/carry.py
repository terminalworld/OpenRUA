import numpy as np, sys, json
sys.path.insert(0,'.')
from rob import *
r=Robot('carry'); prev=np.array(r.arm_q())
d2=np.array([-0.5,0.7071,-0.5]); x2=np.array([-0.5,-0.7071,-0.5])
d2/=np.linalg.norm(d2); x2/=np.linalg.norm(x2)
qh2=hand_quat(d2,x2)
print('y_hand',np.round(np.cross(d2,x2),3))
W=[(-0.10,-0.20,1.15),(-0.043,0.05,1.15),(-0.043,0.10,1.023),(-0.043,0.20,1.023),(-0.043,0.30,1.023)]
sols=[]
for w in W:
    s=best_ik(r,np.array(w),qh2,seed=prev,prefer=prev,avoid=lambda L: not clear_of_door(L))
    print(w, None if s is None else np.round(s,2))
    if s is not None: prev=np.array(s)
    sols.append(s)
json.dump([None if s is None else list(map(float,s)) for s in sols],open('carry_sols.json','w'))
if len(sys.argv)>1 and sys.argv[1]=='go':
    n=int(sys.argv[2]) if len(sys.argv)>2 else len(W)
    for i,s in enumerate(sols[:n]):
        r.move_joints(s,4.0)
        print('W',i,'tcp',np.round(r.tcp()[0],4),'fingers',np.round(r.fingers(),4),'wrench',np.round(r.wrench()[:3],2))
