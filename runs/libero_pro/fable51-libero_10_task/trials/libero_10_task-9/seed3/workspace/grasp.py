from rb import *
import sys
r=Robot('grasp')
beta=np.radians(20)
zh=np.array([0,np.sin(beta),-np.cos(beta)]); yh=np.array([0,np.cos(beta),np.sin(beta)])
R=R_from(zh,yh)
x_mug,y_mug,rim=-0.08,-0.287,1.013
tcp=np.array([x_mug, y_mug-0.0435, rim-0.012])
def flange(t): return t-0.1034*zh
print('fingers',r.fingers())
# pre-grasp
p1=flange(tcp+np.array([0,0,0.12]))
q1=r.ik(p1,R); print('q1',None if q1 is None else q1.round(3))
code,err=r.move(q1,5); print('pre code',code,err.round(4), r.fk()[0].round(3), 'target',p1.round(3))
if err>0.02: code,err=r.move(q1,5); print('pre retry',code,err.round(4))
# descend in two steps
p2=flange(tcp+np.array([0,0,0.05])); q2=r.ik(p2,R,seed=q1)
p3=flange(tcp); q3=r.ik(p3,R,seed=q2)
print('q3',q3.round(3))
code,err=r.move(q3,4,extra_points=[(q2,2.0)]); print('descend code',code,err.round(4),r.fk()[0].round(3),'target',p3.round(3))
if err>0.01: code,err=r.move(q3,3); print('descend retry',code,err.round(4),r.fk()[0].round(3))
print('wrench before',r.wrench().round(2))
r.snap('robot0_eye_in_hand','eih_pregrasp.png'); r.snap('agentview','agent_pregrasp.png')
