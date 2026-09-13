from rb import *
r=Robot('place')
beta=np.radians(20)
zh=np.array([0,np.sin(beta),-np.cos(beta)]); yh=np.array([0,np.cos(beta),np.sin(beta)])
R=R_from(zh,yh)
def flange(t): return t-0.1034*zh
p,_=r.fk(); tcp=p+0.1034*zh; print('tcp now',tcp.round(3))
w0=r.wrench()
q=r.arm_q()
for z in [1.15,1.10,1.075,1.06]:
    t=np.array([-0.055,0.30-0.0435,z]); q=r.ik(flange(t),R,seed=q)
    code,err=r.move(q,3); 
    if err>0.01: code,err=r.move(q,2)
    w=r.wrench(); print('z',z,code,err.round(4),r.fk()[0].round(3),'dW',(w-w0).round(2),'fingers',np.round(r.fingers(),4))
r.snap('agentview','agent_place.png'); r.snap('frontview','front_place.png')
