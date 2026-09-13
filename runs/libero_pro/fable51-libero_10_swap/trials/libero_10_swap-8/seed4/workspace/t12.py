from pp import *
import time
pp_=PP(); r=pp_.r
LIM=np.array(FJT['limits_rad'])
ZH20=R_G[:,2]
def seq(cx,cy,tx,ty,zb):
    g=np.array([cx,cy,GRASP_Z]); pre=g-0.10*ZH20; high=pre+[0,0,0.12]
    lift=g+[0,0,0.12]; above=np.array([tx,ty,1.06]); down=np.array([tx,ty,zb+GRASP_Z-TABLE_Z]); back=down-0.10*ZH20; up=back+[0,0,0.10]
    return {'high':high,'pre':pre,'grasp':g,'lift':lift,'above':above,'down':down,'back':back,'up':up}
for name,R in [('R_G',R_G),('R2',R_G@rot_z(np.pi))]:
  for pot,(cx,cy,tx,ty,zb) in {'B':(-0.042,0.254,0.20,-0.02,0.935),'A':(-0.195,-0.2005,0.14,0.085,0.945)}.items():
    print(f'=== {name} pot {pot}')
    prev=None; t0=time.time()
    for k,p in seq(cx,cy,tx,ty,zb).items():
        s=r.ik_best(p,R,prev=prev)
        if s is None: print(f'  {k:6s} FAIL'); continue
        margin=np.minimum(s-LIM[:,0],LIM[:,1]-s).min(); d=0 if prev is None else np.abs(s-prev).max(); prev=s
        print(f'  {k:6s} jump={d:.2f} margin={margin:.2f}', s.round(2))
    print('  time',round(time.time()-t0,1))
