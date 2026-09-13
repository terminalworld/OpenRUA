from rb import *
r=Robot('insert2')
zh=np.array([0,0,-1.0]); yh=np.array([0,1.0,0])
R=R_from(zh,yh)
def flange(t): return t-0.1034*zh
w0=r.wrench()
def go(t,sec,seed=None,tag='',fmax=6):
    q=r.ik(flange(t),R,seed=seed)
    if q is None: raise SystemExit('IK fail '+str(t))
    code,err=r.move(q,sec)
    if err>0.01: code,err=r.move(q,3)
    p=r.fk()[0]; w=r.wrench()
    print(tag,'tcp',(p+0.1034*zh).round(3),'tgt',t.round(3),code,err.round(4),'dW',(w-w0).round(1),'fing',np.round(r.fingers(),4))
    if np.abs(w-w0)[:3].max()>fmax: raise SystemExit('force!')
    return q
q=r.arm_q()
p,_=r.fk(); tcp=p+0.1034*zh
x=-0.05
for y in [0.13,0.17,0.20,0.22,0.235,0.2495]:
    q=go(np.array([x,y,1.0685]),3,q,tag=f'in y={y}')
    if r.fingers()[0]<0.001: raise SystemExit('slipped!')
r.snap('agentview','agent_in2.png'); r.snap('frontview','front_in2.png')
