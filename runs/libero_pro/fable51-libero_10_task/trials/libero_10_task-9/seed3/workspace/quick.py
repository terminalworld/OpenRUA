from rb import *
import time
r=Robot('quick')
zh=np.array([0,0,-1.0]); yh=np.array([0,1.0,0])
R=R_from(zh,yh)
def flange(t): return t-0.1034*zh
x_mug,y_mug,rim=-0.052,0.122,1.013
tcp=np.array([x_mug,y_mug-0.0435,rim-0.014])
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
q=go(tcp+np.array([0,0,0.10]),5,tag='pre')
q=go(tcp+np.array([0,0,0.04]),2,q,tag='mid')
q=go(tcp,2,q,tag='grasp')
# precompute trajectory
qa=r.ik(flange(np.array([x_mug,y_mug-0.0435,1.067])),R,seed=q)
qb=r.ik(flange(np.array([-0.05,0.16,1.067])),R,seed=qa)
qc=r.ik(flange(np.array([-0.05,0.2495,1.067])),R,seed=qb)
print('traj ok', qa is not None, qb is not None, qc is not None)
t0=time.time()
print('grip',r.grip(0.0), time.time()-t0)
code,err=r.move(qc,4.5,extra_points=[(qa,1.5),(qb,3.0)])
print('insert',code,err.round(4),'t',round(time.time()-t0,1),'fing',np.round(r.fingers(),4),'tcp',(r.fk()[0]+0.1034*zh).round(3))
print('open',r.grip(0.04), round(time.time()-t0,1))
print('wrench',(r.wrench()-w0).round(1))
r.snap('agentview','agent_q.png'); r.snap('frontview','front_q.png')
