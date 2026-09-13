from rb import *
r=Robot('setdown2')
zh=np.array([0,0,-1.0]); yh=np.array([0,1.0,0])
R=R_from(zh,yh)
def flange(t): return t-0.1034*zh
w0=r.wrench(); q=r.arm_q()
for z in [1.08,1.05,1.03,1.01,0.995,0.98]:
    t=np.array([-0.05,0.17,z]); q=r.ik(flange(t),R,seed=q); code,err=r.move(q,2)
    w=r.wrench(); print('z',z,code,err.round(4),'dW',(w-w0).round(1),'fing',np.round(r.fingers(),4))
    if np.abs(w-w0)[:3].max()>4: break
print('open',r.grip(0.04)); r.spin(0.5); print('fingers',np.round(r.fingers(),4))
r.snap('agentview','agent_sd2.png')
