import numpy as np, cv2, sys
def quat_R(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
CAMS={'birdview':((-0.2,0,3.0),(0.7071,0.7071,0,0),579.4112549695428),
      'agentview':((0.6066,0,0.96),(0.6182,0.6182,-0.3432,-0.3432),579.4112549695428)}
def cloud(cam):
    d=np.load(f'{cam}_depth.npy'); t,q,f=CAMS[cam]
    h,w=d.shape; v,u=np.mgrid[0:h,0:w]
    X=(u-320)*d/f; Y=(v-240)*d/f
    P=np.stack([X,Y,d],-1)@quat_R(*q).T+np.array(t)
    return P
if __name__=='__main__':
    P=cloud('birdview')
    z=P[...,2]
    print('table z candidates (mode):', np.round(np.median(z[200:330,230:420]),3))
    # find objects: z above table
    tz=np.median(z[200:330,230:420])
    mask=(z>tz+0.01)&(z<tz+0.4)
    img=cv2.imread('birdview.png')
    n,lab,stats,cent=cv2.connectedComponentsWithStats(mask.astype(np.uint8))
    for i in range(1,n):
        if stats[i,4]<10: continue
        m=lab==i
        pts=P[m]
        print(f'comp {i}: px centroid ({cent[i][0]:.0f},{cent[i][1]:.0f}) area {stats[i,4]} world xy min/max x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] ztop {pts[:,2].max():.3f} zmed {np.median(pts[:,2]):.3f}')
