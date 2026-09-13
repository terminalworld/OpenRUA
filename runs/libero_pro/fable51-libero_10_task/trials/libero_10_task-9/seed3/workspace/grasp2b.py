from rb import *
r=Robot('grasp2b')
zh=np.array([0,0,-1.0]); yh=np.array([0,1.0,0])
R=R_from(zh,yh)
def flange(t): return t-0.1034*zh
x_mug,y_mug,rim=-0.0435,0.1425,1.013
tcp=np.array([x_mug,y_mug-0.0435,rim-0.012])
w0=r.wrench()
def go(t,sec,seed=None,tag='',fmax=8):
    q=r.ik(flange(t),R,seed=seed)
    if q is None: raise SystemExit('IK fail '+str(t))
    code,err=r.move(q,sec)
    if err>0.01: code,err=r.move(q,3)
    p=r.fk()[0]; w=r.wrench()
    print(tag,'tcp',(p+0.1034*zh).round(3),'tgt',t.round(3),code,err.round(4),'dW',(w-w0).round(1),'fing',np.round(r.fingers(),4))
    if np.abs(w-w0)[:3].max()>fmax: raise SystemExit('force!')
    return q
q=r.arm_q()
q=go(tcp+np.array([0,0,0.05]),3,q,tag='mid')
q=go(tcp,3,q,tag='grasp')
print('grip',r.grip(0.0)); r.spin(0.3); print('fingers',np.round(r.fingers(),4))
q=go(tcp+np.array([0,0,0.0675]),3,q,tag='lift')   # rim -> 1.0805
print('fingers',np.round(r.fingers(),4))
r.snap('agentview','agent_g2.png')
