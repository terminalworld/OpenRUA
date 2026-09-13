from robot import *
from plan import R_GRASP,R_PLACE,R_PUSH,A,GRASP,seg_check
Q_GRASP=np.array([1.226,0.871,-0.837,-2.486,-2.836,2.182,-1.737])
POSES=[('above',[-0.20,0.053,1.15],R_GRASP,'grasp'),
       ('pre_grasp',GRASP-0.11*A,R_GRASP,'grasp'),
       ('grasp',GRASP,R_GRASP,None),
       ('lift',[-0.148,0.053,1.15],R_GRASP,'grasp'),
       ('reorient',[-0.10,0.08,1.20],R_PLACE,'ref'),
       ('pre_place',[0.03,0.13,1.15],R_PLACE,'prev'),
       ('place',[0.03,0.13,1.02],R_PLACE,'prev'),
       ('retreat',[0.03,0.09,1.25],R_PLACE,'prev'),
       ('pre_push',[0.07,0.05,1.10],R_PUSH,'ref'),
       ('push0',[0.07,0.05,0.965],R_PUSH,'prev'),
       ('push1',[0.07,0.212,0.965],R_PUSH,'prev')]
def build(r):
    qs={'grasp':Q_GRASP}; prev=None
    for name,p,R,mode in POSES:
        if mode is None: prev=Q_GRASP; continue
        ref={'grasp':Q_GRASP,'prev':prev,'ref':REF}[mode]
        seeds=[ref] if mode!='ref' else [prev]
        sol=best_ik(r,np.array(p),R,extra_seeds=seeds,n_rand=8,ref=ref)
        qs[name]=sol; prev=sol
        print(name,None if sol is None else np.round(sol,3), 'tcp',np.round(r.tcp(sol)[0],3) if sol is not None else '')
    return qs
if __name__=='__main__':
    r=Robot()
    q0=r.arm_q(); print('current',np.round(q0,3))
    qs=build(r); qs['current']=q0
    names=['current']+[n for n,_,_,_ in POSES]
    for a,b in zip(names[:-1],names[1:]):
        if qs[a] is None or qs[b] is None: continue
        # tcp path
        tcps=[r.tcp(qs[a]+(qs[b]-qs[a])*t)[0] for t in np.linspace(0,1,9)]
        tcps=np.array(tcps)
        print(a,'->',b,'max dq',np.round(np.abs(qs[b]-qs[a]).max(),2),'worst',seg_check(r,qs[a],qs[b]),
              'tcp z min',np.round(tcps[:,2].min(),3),'tcp y max',np.round(tcps[:,1].max(),3))
    np.save('plan_q.npy',qs,allow_pickle=True)
