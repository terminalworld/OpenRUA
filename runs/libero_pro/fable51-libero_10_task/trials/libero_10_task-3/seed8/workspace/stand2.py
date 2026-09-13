import numpy as np
P=np.load('bird_xyz3.npy'); x,y,z=P[...,0],P[...,1],P[...,2]
m=(x>-0.35)&(x<0.1)&(y>-0.6)&(y<-0.15)&(z>0.95)&(z<1.3)
print('z hist',np.histogram(z[m],bins=np.arange(0.95,1.31,0.02)))
res=0.02
for yy in np.arange(-0.56,-0.15,res):
    row=''
    for xx in np.arange(-0.34,0.10,res):
        mm=m&(x>=xx)&(x<xx+res)&(y>=yy)&(y<yy+res)
        row+=f'{int(round((z[mm].max()-0.9)*100)):3d}' if mm.any() else '  .'
    print(f'{yy:6.2f}',row)
print('x cols from -0.34 step 0.02')
