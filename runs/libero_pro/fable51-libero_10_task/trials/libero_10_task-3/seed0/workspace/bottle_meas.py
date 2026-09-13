import numpy as np
Wd=np.load("/workspace/agent_world.npy"); wx,wy,wz=Wd[...,0],Wd[...,1],Wd[...,2]
# bottle pixels in agentview: roughly u 335-365, v 160-245. Restrict to region near bottle x,y
m=np.zeros(wz.shape,bool); m[150:250,325:375]=True
m&=(wx>-0.20)&(wx<-0.07)&(np.abs(wy-0.048)<0.06)&(wz>0.90)
print("n",m.sum(), "x",wx[m].min().round(3),wx[m].max().round(3),"y",wy[m].min().round(3),wy[m].max().round(3),"z",wz[m].min().round(3),wz[m].max().round(3))
for lo in np.arange(0.90,1.14,0.02):
    mm=m&(wz>=lo)&(wz<lo+0.02)
    if mm.sum(): print(f"z[{lo:.2f},{lo+0.02:.2f}] n={mm.sum()} y[{wy[mm].min():.3f},{wy[mm].max():.3f}] width={wy[mm].max()-wy[mm].min():.3f} xfront={wx[mm].min():.3f}")
