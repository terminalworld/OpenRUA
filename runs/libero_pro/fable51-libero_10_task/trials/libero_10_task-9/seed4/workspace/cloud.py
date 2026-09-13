import numpy as np, cv2, sys
fx=579.4112549695428; cx=320; cy=240
CAMS = {
 "agentview": ((0.6586,0.0,1.6104),(0.6380,0.6380,-0.3048,-0.3048)),
 "frontview": ((1.0,0.0,1.48),(0.5608,0.5608,-0.4306,-0.4306)),
 "birdview": ((-0.2,0.0,3.0),(0.7071,0.7071,0.0,0.0)),
 "sideview": ((-0.0565,1.2761,1.4880),(0.0099,0.8064,-0.5912,-0.0069)),
}
def R_of(q):
    x,y,z,w=q
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
def cloud(cam):
    d=np.load(f"snaps/{cam}_depth.npy")
    t,q=CAMS[cam]; R=R_of(q)
    v,u=np.mgrid[0:d.shape[0],0:d.shape[1]]
    X=(u-cx)*d/fx; Y=(v-cy)*d/fy if False else (v-cy)*d/fx
    P=np.stack([X,Y,d],-1).reshape(-1,3)@R.T+np.array(t)
    return P.reshape(d.shape+(3,))
if __name__=="__main__":
    cam=sys.argv[1]
    P=cloud(cam)
    np.save(f"snaps/{cam}_world.npy",P)
    # sanity: table height
    print("table z candidates:", np.percentile(P[...,2][np.isfinite(P[...,2])], [5,25,50]))
