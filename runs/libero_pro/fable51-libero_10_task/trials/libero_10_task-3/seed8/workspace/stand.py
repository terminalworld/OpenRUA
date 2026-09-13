import numpy as np
d=np.load('birdview_depth.npy'); T=np.load('birdview_T.npy'); fx,fy,cx,cy=np.load('birdview_K.npy')
v,u=np.mgrid[0:d.shape[0],0:d.shape[1]]
z=d; X=(u-cx)*z/fx; Y=(v-cy)*z/fy
P=np.stack([X,Y,z,np.ones_like(z)],-1)@T.T
np.save('bird_xyz3.npy',P[...,:3])
x,y,zz=P[...,0],P[...,1],P[...,2]
res=0.02
for yy in np.arange(-0.60,-0.15,res):
    row=''
    for xx in np.arange(-0.40,0.06,res):
        m=(x>=xx)&(x<xx+res)&(y>=yy)&(y<yy+res)
        row+=f'{int(round((zz[m].max()-0.9)*100)):3d}' if m.any() else '  .'
    print(f'{yy:6.2f}',row)
print('x cols from -0.40 step 0.02')
