from rb import *
r=Robot('insert')
beta=np.radians(20)
zh=np.array([0,np.sin(beta),-np.cos(beta)]); yh=np.array([0,np.cos(beta),np.sin(beta)])
R=R_from(zh,yh)
def flange(t): return t-0.1034*zh
w0=r.wrench()
def go(t,sec,seed=None,tag=''):
    q=r.ik(flange(t),R,seed=seed)
    if q is None: raise SystemExit('IK fail '+str(t))
    code,err=r.move(q,sec)
    if err>0.01: code,err=r.move(q,3)
    p=r.fk()[0]; w=r.wrench()
    print(tag,'tcp',(p+0.1034*zh).round(3),'tgt',t.round(3),code,err.round(4),'dW',(w-w0).round(1),'fing',np.round(r.fingers(),4))
    return q, np.abs(w-w0)[:3].max()
p,_=r.fk(); tcp=p+0.1034*zh
q,_=go(np.array([-0.05,0.17-0.0435,tcp[2]]),6,tag='out')
for z in [1.15,1.10,1.058]:
    q,f=go(np.array([-0.05,0.17-0.0435,z]),3,q,tag='down')
    if f>8: raise SystemExit('force!')
for y in [0.22,0.26,0.30]:
    q,f=go(np.array([-0.05,y-0.0435,1.058]),3,q,tag='in')
    if f>8: raise SystemExit('force!')
r.snap('agentview','agent_in.png'); r.snap('frontview','front_in.png')
