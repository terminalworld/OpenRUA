import numpy as np
from ctl import Robot
r = Robot("s11b")
c,d,P,T = r.snap("robot0_eye_in_hand","/workspace/s11b_eih.png")
np.save("/workspace/s11b_eih_P.npy", P)
pts = P.reshape(-1,3); pts = pts[np.isfinite(pts).all(1)]
# rim: z in 0.975..0.995, exclude fingers (fingers are near y 0.06-0.15 and z>0.97 too...)
rim = pts[(pts[:,2]>0.975)&(pts[:,2]<0.996)]
print("rim-ish pts", len(rim))
# points near x=-0.548 +-0.01
sl = rim[np.abs(rim[:,0]+0.548)<0.01]
if len(sl): 
    ys = np.sort(sl[:,1]); print("y range at x=-0.548:", ys.min().round(4), ys.max().round(4))
    h,_ = np.histogram(sl[:,1], bins=np.arange(-0.05,0.17,0.005)); print(list(zip(np.arange(-0.05,0.17,0.005).round(3), h)))
# fingers: dark gray, z in 0.96..1.0 near y ~0.06..0.15
fing = pts[(pts[:,2]>0.96)&(pts[:,2]<1.0)&(np.abs(pts[:,0]+0.548)<0.02)]
print("mug circle fit from rim pts:")
mask = (rim[:,2]>0.985)
xy = rim[mask][:,:2]
A = np.c_[2*xy, np.ones(len(xy))]; b = (xy**2).sum(1)
cx,cy,c0 = np.linalg.lstsq(A,b,rcond=None)[0]; rad = np.sqrt(c0+cx**2+cy**2)
print("center", round(cx,4), round(cy,4), "r", round(rad,4))
