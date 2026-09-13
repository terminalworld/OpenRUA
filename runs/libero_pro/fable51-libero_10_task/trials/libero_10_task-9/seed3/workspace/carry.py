from rb import *
r=Robot('carry')
beta=np.radians(20)
zh=np.array([0,np.sin(beta),-np.cos(beta)]); yh=np.array([0,np.cos(beta),np.sin(beta)])
R=R_from(zh,yh)
def flange(t): return t-0.1034*zh
print('fingers',r.fingers())
# current TCP
p,_=r.fk(); tcp=p+0.1034*zh; print('tcp now',tcp.round(3))
rim=tcp[2]+0.012; print('mug rim',rim.round(3),'bottom',(rim-0.11).round(3))
# lift so mug bottom >= 1.13: rim >= 1.24 -> tcp z = 1.228
q0=r.arm_q()
t1=np.array([tcp[0],tcp[1],1.228]); q1=r.ik(flange(t1),R,seed=q0)
# move over to cavity at same height: mug center target (-0.055,0.30) -> tcp y = 0.30-0.0435
t2=np.array([-0.055,0.30-0.0435,1.228]); q2=r.ik(flange(t2),R,seed=q1)
print('q1',q1.round(3)); print('q2',q2.round(3))
code,err=r.move(q2,8,extra_points=[(q1,3.0)]); print('carry',code,err.round(4),r.fk()[0].round(3))
if err>0.01: code,err=r.move(q2,4); print('carry retry',code,err.round(4),r.fk()[0].round(3))
print('fingers',r.fingers(),'wrench',r.wrench().round(2))
r.snap('agentview','agent_carry.png'); r.snap('frontview','front_carry.png')
