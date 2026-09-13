import numpy as np
xyz=np.load('bird_xyz2.npy').reshape(-1,3); x,y,z=xyz.T
m=(x>-0.30)&(x<-0.12)&(y>-0.08)&(y<0.06)&(z>0.925)&(z<0.96)
pts=xyz[m]; print('n',len(pts))
c=pts[:,:2].mean(0); U,S,Vt=np.linalg.svd(pts[:,:2]-c,full_matrices=False); ax=Vt[0]
if ax[0]>0: ax=-ax
proj=(pts[:,:2]-c)@ax; perp=(pts[:,:2]-c)@Vt[1]
print('body center',np.round(c,4),'axis (toward cork)',np.round(ax,3),'angle deg',np.degrees(np.arctan2(ax[1],ax[0])))
print('along',proj.min(),proj.max(),'perp',perp.min(),perp.max(),'zmax',pts[:,2].max())
for s in np.arange(proj.min(),proj.max(),0.01):
    sel=np.abs(proj-s)<0.005
    if sel.any(): print(f' s={s:.3f} zmax={pts[sel,2].max():.3f} perp {perp[sel].min():.3f}..{perp[sel].max():.3f} n={sel.sum()}')
