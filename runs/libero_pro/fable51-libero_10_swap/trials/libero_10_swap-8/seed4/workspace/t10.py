from pp import *
import pp as P
pp_=PP(); r=pp_.r
LIM=np.array(FJT['limits_rad'])
tests={'B_high':(-0.136,0.254,1.092),'B_pre':(-0.136,0.254,0.972),'B_grasp':(-0.042,0.254,0.938),
'B_lift':(-0.042,0.254,1.06),'B_place_above':(0.23,0.08,1.06),'B_place':(0.23,0.08,0.973),'B_retreat':(0.136,0.08,1.007),'B_retreat_up':(0.136,0.08,1.107),
'A_high':(-0.289,-0.2005,1.092),'A_pre':(-0.289,-0.2005,0.972),'A_grasp':(-0.195,-0.2005,0.938),'A_lift':(-0.195,-0.2005,1.06),
'A_place_above':(0.17,-0.02,1.06),'A_place':(0.17,-0.02,0.973),'A_retreat':(0.076,-0.02,1.007)}
def run(R, seeds):
    prev=None; ok=0
    for k,p in tests.items():
        p=np.array(p); cands=[]
        for sd in ([prev] if prev is not None else [])+seeds:
            s=r.ik_tcp(p,R,seed=sd,timeout=2)
            if s is not None: cands.append(s)
        if not cands: print(f'{k:15s} FAIL'); continue
        ref=prev if prev is not None else seeds[0]
        s=min(cands,key=lambda s:np.abs(s-ref).max()); d=np.abs(s-ref).max()
        margin=np.minimum(s-LIM[:,0],LIM[:,1]-s).min(); prev=s; ok+=1
        print(f'{k:15s} jump={d:.2f} margin={margin:.2f}', s.round(2))
    return ok
R2=R_tilt(np.radians(20)) @ rot_z(np.pi)
print('R2 z',R2[:,2].round(2),'y',R2[:,1].round(2),'x',R2[:,0].round(2))
q0=r.arm_q()
seeds=[q0, np.array([0,0.5,0,-2.0,0,2.5,0.785]), np.array([0,0.5,0,-2.0,0,0.5,0.785]), np.array([0,0.5,0,-2.0,3.0,1.5,-2.3])]
print('=== roll180 tilt20'); run(R2, seeds)
