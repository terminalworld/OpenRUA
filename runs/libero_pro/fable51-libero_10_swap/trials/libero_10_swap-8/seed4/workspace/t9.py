from pp import *
pp=PP(); r=pp.r
LIM=np.array(FJT['limits_rad'])
tests={'B_high':(-0.136,0.254,1.092),'B_pre':(-0.136,0.254,0.972),'B_grasp':(-0.042,0.254,0.938),
'B_lift':(-0.042,0.254,1.06),'B_place_above':(0.23,0.08,1.06),'B_place':(0.23,0.08,0.973),'B_retreat':(0.136,0.08,1.007),'B_retreat_up':(0.136,0.08,1.107),
'A_high':(-0.289,-0.2005,1.092),'A_pre':(-0.289,-0.2005,0.972),'A_grasp':(-0.195,-0.2005,0.938),'A_lift':(-0.195,-0.2005,1.06),
'A_place_above':(0.17,-0.02,1.06),'A_place':(0.17,-0.02,0.973),'A_retreat':(0.076,-0.02,1.007)}
for seedname,base in {'wristflip':np.array([0.4,0.8,-0.7,-2.5,2.8,1.7,-1.9])}.items():
    print('=== seed',seedname)
    prev=None
    for k,p in tests.items():
        p=np.array(p)
        cands=[]
        for sd in ([prev] if prev is not None else [])+[base]:
            s=r.ik_tcp(p,R_G,seed=sd,timeout=3)
            if s is not None: cands.append(s)
        if not cands: print(f'{k:15s} FAIL'); continue
        ref=prev if prev is not None else base
        s=min(cands,key=lambda s:np.abs(s-ref).max()); d=np.abs(s-ref).max()
        margin=np.minimum(s-LIM[:,0],LIM[:,1]-s).min()
        prev=s
        print(f'{k:15s} jump={d:.2f} margin={margin:.2f}', s.round(2))
