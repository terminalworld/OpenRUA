import numpy as np, sys
from cloudgen import cloud
fx=float(sys.argv[1])
Pw,d=cloud('snaps/agent_depth.npy',fx,fx,320,240,[0.4586,0.0,1.6104],[0.638,0.638,-0.3048,-0.3048])
X,Y,Z=Pw[...,0],Pw[...,1],Pw[...,2]
np.set_printoptions(precision=3,suppress=True,linewidth=200)
print('table sample', Pw[420,320], Pw[300,100], Pw[300,600])
# caddy: points with Z>0.93 and X in -0.5..-0.3
m=(Z>0.93)&(Z<1.1)&(X>-0.5)&(X<-0.3)
print('caddy pts',m.sum(),'X',X[m].min(),X[m].max(),'Y',Y[m].min(),Y[m].max(),'Z',Z[m].max())
# wall top segments: Z>1.03
for lo,hi in [(1.03,1.1),(0.94,1.0)]:
    mm=m&(Z>lo)&(Z<hi)
    print('walls Z in',lo,hi,'n',mm.sum(),'X',X[mm].min().round(3),X[mm].max().round(3),'Y',Y[mm].min().round(3),Y[mm].max().round(3))
# Y histogram of tall wall points
mm=m&(Z>1.03)
h,e=np.histogram(Y[mm],bins=np.arange(-0.42,0.06,0.02))
for a,b in zip(e,h): print(round(a,2),b)
# book
bm=(Z>0.95)&(X>-0.2)&(X<0.0)&(Y>-0.1)&(Y<0.15)
print('book X',X[bm].min(),X[bm].max(),'Y',Y[bm].min(),Y[bm].max(),'Zmax',Z[bm].max())
