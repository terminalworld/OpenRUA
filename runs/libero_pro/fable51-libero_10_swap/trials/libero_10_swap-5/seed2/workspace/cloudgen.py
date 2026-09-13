import numpy as np
def quat_R(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
def cloud(npy, fx, fy, cx, cy, t, q):
    d=np.load(npy); H,W=d.shape
    vv,uu=np.mgrid[0:H,0:W]
    P=np.stack([(uu-cx)*d/fx,(vv-cy)*d/fy,d],-1)
    R=quat_R(*q)
    return P@R.T+np.array(t), d
