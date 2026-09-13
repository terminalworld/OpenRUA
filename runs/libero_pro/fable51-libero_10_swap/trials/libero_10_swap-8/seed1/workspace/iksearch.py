from rob import *
import itertools
def search(r,pos,qt,az=None,m=0.1):
    pos=np.array(pos,float)
    if az is None: az=math.atan2(pos[1],pos[0]+0.66)
    sols=[]
    for a,b,c in itertools.product((0.2,0.5,0.8,1.1,1.4),(-1.0,-1.6,-2.2,-2.8),(1.2,1.8,2.4,3.0)):
        seed=np.array([az,a,0,b,0,c,0.785+az])
        s=r.ik_world(pos,qt,seed=seed,tcp=True)
        if s is not None and margin_ok(s,m): sols.append(s)
    if not sols: return None
    sols=np.array(sols)
    score=np.abs(sols[:,2])+np.abs(sols[:,4])+np.abs(sols[:,0]-az)
    i=np.argmin(score); return sols[i], len(sols)
