import numpy as np
from cloud import world_points
x,y,z,_=world_points('birdview')
box=(x>-0.143)&(x<0.044)&(y>0.111)&(y<0.297)
top=box&(z>1.045)
print(np.histogram(z[top],bins=[1.045,1.06,1.07,1.08,1.09,1.1,1.11,1.13])[0])
hi=box&(z>1.10); print('z>1.10 pts', hi.sum(), 'xy', np.round([x[hi].mean(),y[hi].mean()],3), 'x', np.round([x[hi].min(),x[hi].max()],3),'y', np.round([y[hi].min(),y[hi].max()],3))
lid=box&(z>1.06)&(z<1.10); print('lid pts', lid.sum(), 'xy center', np.round([x[lid].mean(),y[lid].mean()],4), 'zmax', round(z[lid].max(),4))
# pot body ring at z 1.0-1.04 (upper chamber) center & radius
ring=box&(z>1.0)&(z<1.04)&(np.hypot(x+0.055,y-0.2)<0.06)
print('upper chamber pts', ring.sum(), 'center', np.round([x[ring].mean(),y[ring].mean()],4), 'radius max', round(np.hypot(x[ring]-x[ring].mean(),y[ring]-y[ring].mean()).max(),4))
