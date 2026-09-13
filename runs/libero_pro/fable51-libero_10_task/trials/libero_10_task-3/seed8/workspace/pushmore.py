from robot import *
np.set_printoptions(suppress=True,precision=3)
r=Robot(); plan=np.load('plan3.npy',allow_pickle=True).item()
seq=plan['push0B']+plan['push1B']
# go back down to end of push1B via push0B end, then push1B
for q in [plan['push0B'][-1]]+plan['push1B']:
    if r.move_q(q,max(np.abs(q-r.arm_q()).max()/0.1,0.6))!=0: raise SystemExit('abort')
q=r.arm_q(); p,R=r.tcp(q); print('at',p,'w',np.round(r.wrench(),1))
for dy in [0.005,0.005,0.005]:
    tgt=p+np.array([0,dy,0]); sol=ik_near(r,tgt,R,q,max_dq=0.3,tries=8,avoid_collisions=False)
    if sol is None: print('no ik'); break
    code=r.move_q(sol,1.0); q=r.arm_q(); p2=r.tcp(q)[0]; w=r.wrench()
    print('target y',round(tgt[1],4),'reached',p2,'w',np.round(w,1),'code',code)
    p=p2
    if np.linalg.norm(w[:3])>15: print('force high, stop'); break
r.move_q(r.arm_q(),0.6)
