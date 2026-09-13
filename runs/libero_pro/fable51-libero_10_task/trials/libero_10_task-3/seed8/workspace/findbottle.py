import numpy as np
d=np.load('birdview_depth.npy'); T=np.load('birdview_T.npy'); fx,fy,cx,cy=np.load('birdview_K.npy')
H,W=d.shape; v,u=np.mgrid[0:H,0:W]
P=np.stack([(u-cx)*d/fx,(v-cy)*d/fy,d,np.ones_like(d)],-1)@T.T
xyz=P[...,:3]
x,y,z=xyz[...,0],xyz[...,1],xyz[...,2]
# region left of drawer: x -0.35..-0.02, y -0.05..0.2, z between table+0.01 and 0.99 (bottle lying)
m=(x>-0.35)&(x<-0.0)&(y>-0.05)&(y<0.2)&(z>0.91)&(z<0.99)
pts=xyz[m]; print('n',len(pts))
print('x range',pts[:,0].min(),pts[:,0].max(),'y range',pts[:,1].min(),pts[:,1].max(),'zmax',pts[:,2].max())
# PCA
c=pts[:,:2].mean(0); U,S,Vt=np.linalg.svd(pts[:,:2]-c,full_matrices=False)
print('center',c,'axis',Vt[0],'extent along axis',(pts[:,:2]-c)@Vt[0]).min() if False else None
proj=(pts[:,:2]-c)@Vt[0]; print('center',np.round(c,3),'axis',np.round(Vt[0],3),'along',proj.min(),proj.max())
# top ridge: highest points
top=pts[pts[:,2]>pts[:,2].max()-0.01]; print('ridge pts',len(top),'ridge center',np.round(top.mean(0),3))
# body vs neck: heights along axis
for s in np.linspace(proj.min(),proj.max(),9):
    sel=np.abs(proj-s)<0.01
    if sel.any(): print(f' s={s:.3f} zmax={pts[sel,2].max():.3f} n={sel.sum()}')
