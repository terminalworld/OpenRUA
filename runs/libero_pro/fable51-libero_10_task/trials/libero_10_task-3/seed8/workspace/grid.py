import numpy as np, sys
from dh import fk_hand
tcp=np.array([-0.148,0.053,0.95]); deg=float(sys.argv[1]) if len(sys.argv)>1 else 20
a=np.array([np.cos(np.radians(deg)),0,-np.sin(np.radians(deg))])
target=tcp-0.1034*a
j1=np.arctan2(0.053,0.512)
res=[]
for j2 in np.arange(-1.7,1.71,0.1):
  for j4 in np.arange(-3.0,-0.1,0.1):
    for j6 in np.arange(0,3.71,0.1):
      p,R=fk_hand([j1,j2,0,j4,0,j6,0.785])
      err=np.linalg.norm(p-target); ang=np.degrees(np.arccos(np.clip(R[:,2]@a,-1,1)))
      res.append((err,ang,j2,j4,j6))
res=np.array(res)
m=res[:,0]<0.03
print('within 3cm:',m.sum())
for r in res[m][np.argsort(res[m][:,1])][:15]: print(np.round(r,2))
