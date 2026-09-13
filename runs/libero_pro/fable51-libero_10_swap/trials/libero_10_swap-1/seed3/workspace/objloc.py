import numpy as np, sys
d=np.load('eih_depth.npy')
fx=fy=312.77408948188935; cx=320; cy=240
# world<-cam for robot0_eye_in_hand_optical_frame (from TF dump)
t=np.array([-0.0030,0.0,0.7753]); x,y,z,w=0.7070,-0.7066,-0.0201,0.0201
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
print("R=\n",R.round(3))
vs,us=np.mgrid[0:480,0:640]
P=np.stack([(us-cx)*d/fx,(vs-cy)*d/fy,d],-1)
W=P@R.T+t
def report(name,r0,r1,c0,c1):
    sub=W[r0:r1,c0:c1]; zz=sub[...,2]
    m=(zz>0.435)&(zz<0.48)
    pts=sub[m]
    c=pts.mean(0)
    xy=pts[:,:2]-c[:2]
    ev,evec=np.linalg.eigh(xy.T@xy/len(xy))
    ax=evec[:,1]; yaw=np.arctan2(ax[1],ax[0])
    ext_long=xy@ax; ext_short=xy@evec[:,0]
    print(f"{name}: n={m.sum()} center={c.round(4)} top_z={zz[m].max():.4f} long_axis={ax.round(3)} yaw={np.degrees(yaw):.1f}deg len={ext_long.max()-ext_long.min():.3f} wid={ext_short.max()-ext_short.min():.3f}")
report('butter',270,360,235,300)
report('creamcheese',175,280,515,600)
