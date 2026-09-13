from robot import *
c20,s20=np.cos(np.radians(20)),np.sin(np.radians(20))
c45=np.cos(np.radians(45))
c40,s40=np.cos(np.radians(40)),np.sin(np.radians(40))
R_GRASP=rot_from_axes([c20,0,-s20],[0,-1,0])
R_PLACE=rot_from_axes([0,c45,-c45],[0,c45,c45])
R_PUSH=rot_from_axes([0,s40,-c40],[1,0,0])
A=np.array([c20,0,-s20])
GRASP=np.array([-0.148,0.053,0.95])
POSES=[('pre_grasp',GRASP-0.11*A,R_GRASP),
       ('grasp',GRASP,R_GRASP),
       ('lift',[-0.148,0.053,1.15],R_GRASP),
       ('reorient',[-0.10,0.08,1.20],R_PLACE),
       ('pre_place',[0.03,0.13,1.15],R_PLACE),
       ('place',[0.03,0.13,1.02],R_PLACE),
       ('retreat',[0.03,0.09,1.25],R_PLACE),
       ('pre_push',[0.07,0.05,1.10],R_PUSH),
       ('push0',[0.07,0.05,0.965],R_PUSH),
       ('push1',[0.07,0.212,0.965],R_PUSH)]
def chain(r,start_q):
    qs={}; prev=start_q
    for name,p,R in POSES:
        sol=best_ik(r,np.array(p),R,extra_seeds=[prev],n_rand=8)
        qs[name]=sol
        print(name,None if sol is None else np.round(sol,3))
        if sol is not None: prev=sol
    return qs
def seg_check(r,q0,q1,n=8,finger=0.04):
    worst=(9,None)
    for t in np.linspace(0,1,n+1):
        q=q0+(q1-q0)*t
        cl=link_clearance(r,q,finger=finger)
        for name,c,inside in cl:
            if c<worst[0]: worst=(c,f'{name}@t={t:.2f}')
            if inside: print('   INSIDE CABINET',name,'t',round(t,2))
    return worst
if __name__=='__main__':
    r=Robot()
    q0=r.arm_q()
    print('current',np.round(q0,3))
    qs=chain(r,q0)
    names=['current']+[n for n,_,_ in POSES]
    allq={'current':q0,**qs}
    for a,b in zip(names[:-1],names[1:]):
        if allq[a] is None or allq[b] is None: continue
        print(a,'->',b,'max dq',np.round(np.abs(allq[b]-allq[a]).max(),2),'worst clearance',seg_check(r,allq[a],allq[b]))
    np.save('plan_q.npy',allq,allow_pickle=True)
