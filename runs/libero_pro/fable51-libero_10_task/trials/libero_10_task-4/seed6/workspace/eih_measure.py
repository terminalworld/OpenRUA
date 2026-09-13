"""Measure objects from an eye-in-hand color+depth pair using the TF dump: prints XY extents per Z band."""
import re, sys, numpy as np, cv2
tfpath, dpath, ipath = sys.argv[1:4]
txt=open(tfpath).read()
for blk in txt.split('- header:')[1:]:
    child=re.search(r'child_frame_id: (\S+)',blk).group(1)
    if child!='robot0_eye_in_hand_optical_frame': continue
    nums=[float(x) for x in re.findall(r'[xyzw]: (-?[\d.e-]+)',blk)]
t=np.array(nums[:3]); x,y,z,w=nums[3:]
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
print("cam", t.round(4))
d=np.load(dpath); img=cv2.imread(ipath)
fx=312.77408948188935; cx,cy=320,240
vs,us=np.mgrid[0:480,0:640]
P=(np.stack([(us-cx)*d/fx,(vs-cy)*d/fx,d],-1).reshape(-1,3)@R.T+t).reshape(480,640,3)
Z=P[...,2]
bands=[tuple(map(float,b.split(':'))) for b in sys.argv[4:]]
for lo,hi in bands:
    m=(Z>lo)&(Z<hi); pts=P[m]
    if len(pts)==0: print(lo,hi,"none"); continue
    vs2,us2=np.nonzero(m)
    print(f"Z {lo}-{hi}: n={len(pts)} X[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] Y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] u[{us2.min()},{us2.max()}] v[{vs2.min()},{vs2.max()}]")
# plate (white, low)
pm=(Z>0.43)&(Z<0.47)&(img.min(-1)>150); pts=P[pm]
if len(pts): print(f"plate-white: n={len(pts)} X[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] Y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}]")
np.save('/workspace/last_P.npy',P)
