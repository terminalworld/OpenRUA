import numpy as np,sys
d=np.load('birdview_depth.npy'); T=np.load('birdview_T.npy'); fx,fy,cx,cy=np.load('birdview_K.npy')
H,W=d.shape; v,u=np.mgrid[0:H,0:W]
P=np.stack([(u-cx)*d/fx,(v-cy)*d/fy,d,np.ones_like(d)],-1)@T.T
xyz=P[...,:3].reshape(-1,3); np.save('bird_xyz2.npy',P[...,:3])
x0,x1,y0,y1=[float(a) for a in sys.argv[1:5]]; res=float(sys.argv[5]) if len(sys.argv)>5 else 0.01
xs=np.arange(x0,x1,res); ys=np.arange(y0,y1,res)
print('      '+' '.join(f'{y*100:4.0f}' for y in ys))
for xx in xs:
    row=[]
    for yy in ys:
        m=(np.abs(xyz[:,0]-xx)<res/2)&(np.abs(xyz[:,1]-yy)<res/2)
        row.append(f'{(xyz[m,2].max()-0.9)*100:4.0f}' if m.any() else '   .')
    print(f'{xx*100:5.0f} '+' '.join(row))
