from robot import *
TILT=10
R_G=tilt_R(TILT)
A=R_G[:,2]
c45=np.cos(np.radians(45)); c40,s40=np.cos(np.radians(40)),np.sin(np.radians(40))
c35,s35=np.cos(np.radians(35)),np.sin(np.radians(35))
R_PLACE=rot_from_axes([0,s35,-c35],[0,c35,s35])
R_PUSH=rot_from_axes([0,s40,-c40],[1,0,0])
c50,s50=np.cos(np.radians(50)),np.sin(np.radians(50))
R_PUSH2=rot_from_axes([0,s50,-c50],[0,c50,s50])
GRASP=np.array([-0.148,0.053,0.955])
PRE=GRASP-0.11*A
ABOVE=np.array([-0.22,0.053,1.10])
LIFT=np.array([-0.148,0.053,1.10])
REOR=np.array([-0.10,0.08,1.20]); PREPL=np.array([0.03,0.13,1.15]); PLACE=np.array([0.03,0.13,1.06]); RETR=np.array([0.03,0.10,1.25])
PREPUSH=np.array([0.115,0.05,1.10]); PUSH0=np.array([0.115,0.05,0.965]); PUSH1=np.array([0.115,0.165,0.965]); PUSHUP=np.array([0.115,0.13,1.15])
c15,s15=np.cos(np.radians(15)),np.sin(np.radians(15))
R_PUSHB=rot_from_axes([0,c15,-s15],[1,0,0])
PREPUSHB=np.array([0.10,0.16,1.10]); PUSH0B=np.array([0.10,0.16,0.965]); PUSH1B=np.array([0.10,0.202,0.965]); PUSHUPB=np.array([0.10,0.16,1.15])

def line(r,p0,p1,R,prev,n,max_dq=0.5):
    out=[]
    for t in np.linspace(0,1,n+1)[1:]:
        p=p0+(p1-p0)*t
        sol=ik_near(r,p,R,prev,max_dq=max_dq,tries=8)
        if sol is None: print('  line fail at',np.round(p,3)); return None
        out.append(sol); prev=sol
    return out

def bottle_pts(tcp,R):
    xh=R[:,0]; pts=[]
    for s in np.linspace(-0.055,0.103,6):
        for ang in np.linspace(0,2*np.pi,6,endpoint=False):
            pts.append(tcp+s*xh+0.02*(np.cos(ang)*R[:,1]+np.sin(ang)*R[:,2]))
    return np.array(pts)

def obstacles(pts):
    """return list of obstacle names hit by any point"""
    hits=[]
    x,y,z=pts[:,0],pts[:,1],pts[:,2]
    if (z<0.905).any(): hits.append('table')
    if ((x>-0.125)&(x<0.135)&(y>0.065)&(y<0.215)&(z<0.985)&~((x>-0.09)&(x<0.10)&(y>0.095)&(y<0.21))).any(): hits.append('drawer_walls')
    if ((x>-0.12)&(x<0.13)&(y>0.21)&(z<1.135)).any(): hits.append('cabinet')
    if ((x>-0.055)&(x<0.045)&(y>0.175)&(y<0.215)&(z>0.995)&(z<1.105)).any(): hits.append('handles')
    if ((x>-0.045)&(x<0.045)&(y>0.035)&(y<0.062)&(z<0.96)).any(): hits.append('low_handle')
    if (((x-0.013)**2+(y+0.03)**2<0.068**2)&(z<1.026)).any(): hits.append('bowl')
    if ((x>-0.26)&(x<0.08)&(y>-0.52)&(y<-0.18)&(z<1.25)).any(): hits.append('stand')
    return hits

def check_path(r,qs,held=False,n_sub=4,label=''):
    worst=9; bad=set()
    for a,b in zip(qs[:-1],qs[1:]):
        for t in np.linspace(0,1,n_sub,endpoint=False):
            q=a+(b-a)*t
            fk=fk_links(r,q)
            p,R=fk['panda_hand']
            hp=hand_points(p,R,0.02 if held else 0.04)
            for h in obstacles(hp): bad.add('hand:'+h)
            tcp=p+TCP_OFF*R[:,2]
            if held:
                for h in obstacles(bottle_pts(tcp,R)): bad.add('bottle:'+h)
            for name,c,inside in link_clearance(r,q):
                worst=min(worst,c)
                if inside: bad.add(name+':cabinet')
    print(f'{label}: worst link clearance {worst:.3f} hits {sorted(bad)}')
    return bad

if __name__=='__main__':
    r=Robot(); q0=r.arm_q()
    plan={}
    qg=best_ik(r,GRASP,R_G,extra_seeds=[np.array([1.226,0.871,-0.837,-2.486,-2.836,2.182,-1.737])],n_rand=6,ref=np.array([1.226,0.871,-0.837,-2.486,-2.836,2.182,-1.737]))
    print('grasp',np.round(qg,3),link_clearance(r,qg))
    plan['grasp']=[qg]
    # backwards: grasp -> pre -> above
    seg=line(r,GRASP,PRE,R_G,qg,4); plan['approach']=seg[::-1]  # pre..grasp order later
    seg2=line(r,PRE,ABOVE,R_G,seg[-1],6); plan['descend']=seg2[::-1]
    print('above',np.round(seg2[-1],3))
    lift=line(r,GRASP,LIFT,R_G,qg,6); plan['lift']=lift
    print('lift',np.round(lift[-1],3))
    qre=best_ik(r,REOR,R_PLACE,n_rand=8); print('reorient',np.round(qre,3))
    plan['reorient']=[qre]
    pp=line(r,REOR,PREPL,R_PLACE,qre,3); plan['preplace']=pp
    pl=line(r,PREPL,PLACE,R_PLACE,pp[-1],4); plan['place']=pl
    rt=line(r,PLACE,RETR,R_PLACE,pl[-1],3); plan['retreat']=rt
    qpp=best_ik(r,PREPUSH,R_PUSH2,extra_seeds=[rt[-1]],n_rand=8); print('prepush',np.round(qpp,3)); plan['prepush']=[qpp]
    p0=line(r,PREPUSH,PUSH0,R_PUSH2,qpp,3); plan['push0']=p0
    p1=line(r,PUSH0,PUSH1,R_PUSH2,p0[-1],8); plan['push1']=p1
    pu=line(r,PUSH1,PUSHUP,R_PUSH2,p1[-1],3); plan['pushup']=pu
    qpb=best_ik(r,PREPUSHB,R_PUSHB,extra_seeds=[pu[-1]],n_rand=8); print('prepushB',np.round(qpb,3)); plan['prepushB']=[qpb]
    p0b=line(r,PREPUSHB,PUSH0B,R_PUSHB,qpb,3); plan['push0B']=p0b
    p1b=line(r,PUSH0B,PUSH1B,R_PUSHB,p0b[-1],6); plan['push1B']=p1b
    pub=line(r,PUSH1B,PUSHUPB,R_PUSHB,p1b[-1],3); plan['pushupB']=pub
    np.save('plan3.npy',plan,allow_pickle=True)
    # checks
    check_path(r,[q0]+plan['descend'][:1],label='current->above')
    check_path(r,plan['descend']+plan['approach'],label='above->pre->grasp')
    check_path(r,plan['lift'],held=True,label='lift')
    check_path(r,[plan['lift'][-1],qre],held=True,n_sub=12,label='lift->reorient (family switch)')
    check_path(r,[qre]+pp+pl,held=True,label='reorient->place')
    check_path(r,[pl[-1]]+rt+[qpp],label='retreat->prepush')
    check_path(r,[qpp]+p0,label='prepush->push0')
    check_path(r,p0[-1:]+p1+pu,label='pushA')
    check_path(r,[pu[-1],qpb],label='pushupA->prepushB')
    check_path(r,[qpb]+p0b,label='prepushB->push0B')
    check_path(r,p0b[-1:]+p1b+pub,label='pushB')
