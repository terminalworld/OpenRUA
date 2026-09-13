from robot import *
from plan3 import line, check_path, obstacles
import plan4
np.set_printoptions(suppress=True,precision=3)
r=Robot(); q0=r.arm_q(); p0,R0=r.tcp(q0)
p4=np.load('plan4.npy',allow_pickle=True).item()
qpre=p4['pre4'][0] if isinstance(p4['pre4'],list) else p4['pre4']
print('pre4 q',qpre, 'tcp',r.tcp(qpre)[0])
R_G=plan4.R_G
UP=np.array([p0[0],p0[1],1.40])
q1=ik_near(r,UP,R0,q0,max_dq=0.6,tries=10)
print('q1',q1)
W2=np.array([-0.20,-0.08,1.40])
q2=ik_near(r,W2,R_G,qpre,max_dq=1.5,tries=12)
print('q2',q2, r.tcp(q2)[0])
PRE=r.tcp(qpre)[0]
seg=line(r,W2,PRE,R_G,q2,5)
print('seg end',seg[-1])
check_path(r,[q0,q1],n_sub=10,label='up')
check_path(r,[q1,q2],n_sub=30,label='over+rotate')
check_path(r,[q2]+seg+[qpre],n_sub=6,label='down to pre4')
plan={'up5':[q1],'over5':[q2],'down5':seg+[qpre]}
np.save('plan5.npy',plan,allow_pickle=True)
