import numpy as np
from ctl import *
r=Robot("tilt")
p,qq=r.hand_pose(); print("hand",p.round(4))
for cam in ["frontview","sideview","agentview"]:
    c,d,P,T=r.snap(cam,f"/workspace/tilt_{cam}.png")
    Q=P.reshape(-1,3); Q=Q[np.isfinite(Q).all(1)]
    # mug region: within 12 cm of expected mug center (-0.444,-0.090), z 1.05..1.25
    sel=Q[(np.hypot(Q[:,0]+0.444,Q[:,1]+0.090)<0.12)&(Q[:,2]>1.07)&(Q[:,2]<1.26)]
    print(cam,"n",len(sel))
    if len(sel)==0: continue
    print("  x %.3f..%.3f y %.3f..%.3f z %.3f..%.3f"%(sel[:,0].min(),sel[:,0].max(),sel[:,1].min(),sel[:,1].max(),sel[:,2].min(),sel[:,2].max()))
    # lowest points location
    low=sel[sel[:,2]<sel[:,2].min()+0.01]; print("  lowest pts mean xy",low[:,:2].mean(0).round(3),"n",len(low))
    # per z slice: x/y extents
    for lo in np.arange(1.07,1.26,0.02):
        s=sel[(sel[:,2]>=lo)&(sel[:,2]<lo+0.02)]
        if len(s)>3: print("  z %.2f n=%4d x %.3f..%.3f  y %.3f..%.3f"%(lo,len(s),s[:,0].min(),s[:,0].max(),s[:,1].min(),s[:,1].max()))
