from robot import *
r=Robot()
plan=np.load('plan3.npy',allow_pickle=True).item()
q0=np.array([0,-0.161,0,-2.445,0,2.227,0.785]); qc=r.arm_q()
print('current state', state_valid(r,qc))
print('start->above', path_valid(r,[q0,plan['descend'][0]])[:5])
order=['descend','approach','grasp','lift','reorient','preplace','place','retreat','prepush','push0','push1','pushup','prepushB','push0B','push1B','pushupB']
prev=plan['descend'][0]
for k in order:
    qs=[prev]+list(plan[k])
    bad=path_valid(r,qs,finger=0.02 if k in('lift','reorient','preplace','place') else 0.04)
    print(k,'bad' if bad else 'ok',bad[:3])
    prev=qs[-1]
