from robot import *
from plan3 import *
r=Robot()
plan=np.load('plan3.npy',allow_pickle=True).item()
def detail(qs,label,held=False,skip=()):
    print('==',label)
    for i,(a,b) in enumerate(zip(qs[:-1],qs[1:])):
        for t in np.linspace(0,1,4,endpoint=False):
            q=a+(b-a)*t
            p,R=r.fk(q)
            hp=hand_points(p,R,0.02 if held else 0.04)
            tcp=p+TCP_OFF*R[:,2]
            for h in obstacles(hp):
                if h in skip: continue
                m=[pt for pt in hp if h in obstacles(pt[None,:])]
                print(f'  seg{i} t={t:.2f} tcp {np.round(tcp,3)} hand:{h} pts {np.round(m[0],3)} .. n={len(m)}')
            if held:
                bp=bottle_pts(tcp,R)
                for h in obstacles(bp):
                    m=[pt for pt in bp if h in obstacles(pt[None,:])]
                    print(f'  seg{i} t={t:.2f} tcp {np.round(tcp,3)} bottle:{h} pts {np.round(m[0],3)} n={len(m)}')
pl=plan['place']; pp=plan['preplace']; rt=plan['retreat']
detail([plan['reorient'][0]]+pp+pl,'reorient->place',held=True)
detail([pl[-1]]+rt+plan['prepush'],'retreat->prepush')
detail(plan['push0'][-1:]+plan['push1']+plan['pushup'],'push',skip=('drawer_walls',))
