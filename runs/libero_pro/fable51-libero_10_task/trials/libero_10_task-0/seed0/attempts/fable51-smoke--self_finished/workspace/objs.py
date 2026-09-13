import numpy as np
Pw=np.load('agentview_xyz.npy')
def stats(name,xr,yr,zmin=0.432):
    m=(Pw[...,0]>xr[0])&(Pw[...,0]<xr[1])&(Pw[...,1]>yr[0])&(Pw[...,1]<yr[1])&(Pw[...,2]>zmin)&(Pw[...,2]<0.9)
    p=Pw[m]
    if len(p)==0: print(name,'none'); return
    print(f"{name}: n={len(p)} x[{p[:,0].min():.3f},{p[:,0].max():.3f}] y[{p[:,1].min():.3f},{p[:,1].max():.3f}] z[{p[:,2].min():.3f},{p[:,2].max():.3f}] centroid=({p[:,0].mean():.3f},{p[:,1].mean():.3f}) top-centroid=({p[p[:,2]>p[:,2].max()-0.01][:,0].mean():.3f},{p[p[:,2]>p[:,2].max()-0.01][:,1].mean():.3f})")
stats('cream_cheese',(0.03,0.18),(-0.25,-0.15))
stats('tomato_sauce',(-0.16,-0.05),(-0.02,0.09))
stats('blue_can',(-0.16,-0.05),(-0.21,-0.11))
stats('milk',(0.03,0.15),(-0.14,-0.05))
stats('butter',(0.0,0.1),(0.04,0.12))
stats('OJ',(-0.02,0.06),(-0.26,-0.18))
stats('basket',(-0.12,0.12),(0.15,0.38),0.5)
stats('ketchup',(-0.3,-0.18),(-0.2,-0.09))
print('--- refined')
stats('cream_cheese',(0.065,0.18),(-0.25,-0.15))
Pw2=Pw.copy()
m=(Pw[...,0]>-0.16)&(Pw[...,0]<-0.055)&(Pw[...,1]>-0.0)&(Pw[...,1]<0.1)&(Pw[...,2]>0.432)&(Pw[...,2]<0.53)
p=Pw[m]; print(f"tomato_sauce (z<0.53): n={len(p)} x[{p[:,0].min():.3f},{p[:,0].max():.3f}] y[{p[:,1].min():.3f},{p[:,1].max():.3f}] z[{p[:,2].min():.3f},{p[:,2].max():.3f}] centroid=({p[:,0].mean():.3f},{p[:,1].mean():.3f})")
top=p[p[:,2]>0.49]; print(' top: n',len(top),'x[%.3f,%.3f] y[%.3f,%.3f] c=(%.3f,%.3f)'%(top[:,0].min(),top[:,0].max(),top[:,1].min(),top[:,1].max(),top[:,0].mean(),top[:,1].mean()))
# cream cheese top surface
m=(Pw[...,0]>0.065)&(Pw[...,0]<0.18)&(Pw[...,1]>-0.25)&(Pw[...,1]<-0.15)&(Pw[...,2]>0.445)&(Pw[...,2]<0.5)
p=Pw[m]; print(f"cc top: n={len(p)} x[{p[:,0].min():.3f},{p[:,0].max():.3f}] y[{p[:,1].min():.3f},{p[:,1].max():.3f}] z[{p[:,2].min():.3f},{p[:,2].max():.3f}] centroid=({p[:,0].mean():.3f},{p[:,1].mean():.3f})")
