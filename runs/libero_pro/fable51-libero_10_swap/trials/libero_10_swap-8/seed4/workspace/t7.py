from pp import *
pp=PP(); r=pp.r
def seed_for(p):
    j1=np.arctan2(p[1], p[0]+0.66)
    return np.array([j1, 0.6, 0.0, -1.9, 0.0, 2.8, 0.785+j1])
tests={'B_high':(-0.136,0.254,1.092),'B_pre':(-0.136,0.254,0.972),'B_grasp':(-0.042,0.254,0.938),
'B_lift':(-0.042,0.254,1.06),'B_place_above':(0.23,0.08,1.06),'B_place':(0.23,0.08,0.973),'B_retreat':(0.136,0.08,1.007),'B_retreat_up':(0.136,0.08,1.107),
'A_high':(-0.289,-0.2005,1.092),'A_pre':(-0.289,-0.2005,0.972),'A_grasp':(-0.195,-0.2005,0.938),'A_lift':(-0.195,-0.2005,1.06),
'A_place_above':(0.17,-0.02,1.06),'A_place':(0.17,-0.02,0.973),'A_retreat':(0.076,-0.02,1.007)}
prev=None
for k,p in tests.items():
    p=np.array(p)
    best=None
    for sd in ([prev] if prev is not None else [])+[seed_for(p)]:
        s=r.ik_tcp(p,R_G,seed=sd,timeout=3)
        if s is None: continue
        d=np.abs(s-(prev if prev is not None else seed_for(p))).max()
        if best is None or d<best[1]: best=(s,d)
    if best is None: print(k,'FAIL'); continue
    s,d=best; prev=s
    print(f'{k:15s} jump={d:.2f}', s.round(2))
