import numpy as np
d=np.load('birdview_depth.npy'); T=np.load('birdview_T.npy'); fx,fy,cx,cy=np.load('birdview_K.npy')
H,W=d.shape; v,u=np.mgrid[0:H,0:W]
P=np.stack([(u-cx)*d/fx,(v-cy)*d/fy,d,np.ones_like(d)],-1)@T.T
xyz=P[...,:3]; x,y,z=xyz[...,0],xyz[...,1],xyz[...,2]
m=(x>-0.4)&(x<-0.0)&(y>-0.1)&(y<0.25)&(z>0.905)&(z<0.99)&~((x>-0.13)&(y>0.06))
pts=xyz[m]; print('n',len(pts))
print('x',pts[:,0].min(),pts[:,0].max(),'y',pts[:,1].min(),pts[:,1].max(),'zmax',pts[:,2].max())
c=pts[:,:2].mean(0); U,S,Vt=np.linalg.svd(pts[:,:2]-c,full_matrices=False); ax=Vt[0]
proj=(pts[:,:2]-c)@ax; print('center',np.round(c,3),'axis',np.round(ax,3),'along',proj.min(),proj.max())
for s in np.arange(proj.min(),proj.max(),0.01):
    sel=np.abs(proj-s)<0.005
    if sel.any():
        perp=(pts[sel,:2]-c)@Vt[1]
        print(f' s={s:.3f} zmax={pts[sel,2].max():.3f} perp {perp.min():.3f}..{perp.max():.3f} n={sel.sum()}')
np.save('bottle_pts.npy',pts)
