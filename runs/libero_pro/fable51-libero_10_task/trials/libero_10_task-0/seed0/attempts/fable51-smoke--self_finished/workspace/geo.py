import numpy as np, cv2
CAMS={'agentview':((0.6066,0.0,0.9600),(0.6182,0.6182,-0.3432,-0.3432),579.4112549695428),
      'birdview':((-0.2,0.0,3.0),(0.7071,0.7071,0.0,0.0),None)}
def quatR(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
def cloud(cam,f):
    d=np.load(f'{cam}_depth.npy').astype(np.float64)
    T,q,_=CAMS[cam]; R=quatR(*q)
    H,W=d.shape; cx,cy=W/2,H/2
    u,v=np.meshgrid(np.arange(W),np.arange(H))
    P=np.stack([(u-cx)*d/f,(v-cy)*d/f,d],-1)
    return P@R.T+np.array(T)
