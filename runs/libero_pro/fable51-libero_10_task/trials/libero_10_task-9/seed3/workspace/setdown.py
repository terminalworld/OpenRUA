from rb import *
r=Robot('setdown')
zh=np.array([0,0,-1.0]); yh=np.array([0,1.0,0])
R=R_from(zh,yh)
def flange(t): return t-0.1034*zh
w0=r.wrench(); q=r.arm_q()
for z in [1.04,1.02,1.008,1.0]:
    t=np.array([-0.05,0.17,z]); q=r.ik(flange(t),R,seed=q); code,err=r.move(q,2)
    w=r.wrench(); print('z',z,code,err.round(4),'dW',(w-w0).round(1),'fing',np.round(r.fingers(),4))
print('open',r.grip(0.04))
t=np.array([-0.05,0.17,1.12]); q=r.ik(flange(t),R,seed=q); print(r.move(q,3))
r.snap('agentview','agent_sd.png')
