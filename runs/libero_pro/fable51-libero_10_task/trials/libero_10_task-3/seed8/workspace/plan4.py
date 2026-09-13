from robot import *
from robot import state_valid, path_valid
from plan3 import obstacles, check_path, line
np.set_printoptions(suppress=True,precision=3)
r=Robot()
ax=np.array([-0.947,-0.322,0.0]); ax/=np.linalg.norm(ax)   # bottle axis toward cork
C=np.array([-0.2186,-0.0153,0.92])                          # bottle mass-center (5.5cm from bottom)
closing=-np.cross([0,0,-1],ax)                             # x_h = -ax (toward bottle bottom); keeps j7 away from limit
R_G=rot_from_axes([0,0,-1],closing); print('x_h',np.round(R_G[:,0],3))
R_D=rot_from_axes([0,0,-1],[0,-1,0]); print('drop x_h',np.round(R_D[:,0],3))   # x_h=+x (bottom) -> cork toward -x
PRE=C+np.array([0,0,0.13]); LIFT=C+np.array([0,0,0.23])
DROP=np.array([0.03,0.15,1.11]); PREDROP=np.array([0.03,0.15,1.20]); MID=np.array([-0.10,0.05,1.22])
plan={}
qc=r.arm_q()
qpre=best_ik(r,PRE,R_G,extra_seeds=[np.array([0.1,0.445,-0.1,-2.53,0.24,2.97,0.245])],n_rand=8); print('pre',np.round(qpre,3))
plan['pre4']=[qpre]
g=line(r,PRE,C,R_G,qpre,4); plan['grasp4']=g; print('grasp',np.round(g[-1],3))
l=line(r,C,LIFT,R_G,g[-1],4); plan['lift4']=l
qmid=ik_near(r,MID,R_D,l[-1],max_dq=1.2,tries=12); print('mid',np.round(qmid,3))
plan['mid4']=[qmid]
pd=line(r,MID,PREDROP,R_D,qmid,4); plan['predrop4']=pd
dr=line(r,PREDROP,DROP,R_D,pd[-1],3); plan['drop4']=dr
up=line(r,DROP,PREDROP,R_D,dr[-1],2); plan['up4']=up
np.save('plan4.npy',plan,allow_pickle=True)
check_path(r,[qc,qpre],n_sub=12,label='park->pre')
check_path(r,[qpre]+g,label='pre->grasp')
check_path(r,g[-1:]+l,held=True,label='lift')
check_path(r,l[-1:]+[qmid]+pd+dr,held=True,n_sub=12,label='lift->drop')
for k,v in [('park->pre',[qc,qpre]),('pre->grasp',[qpre]+g),('lift',g[-1:]+l),('lift->drop',l[-1:]+[qmid]+pd+dr)]:
    print(k,'selfcoll',path_valid(r,v,finger=0.02)[:3])
for q in [qpre,g[-1],l[-1],qmid,pd[-1],dr[-1]]:
    print(np.round(q,3),np.round(r.tcp(q)[0],3))
