import numpy as np
from ctl import *
r=Robot("g4")
xh,yh=-0.0740,0.0448
q=topdown_quat(np.pi/2)
r.move_pose([xh,yh,1.20],q,seconds=3)
print("fingers",r.finger())
c,d,P,T=r.snap("frontview","/workspace/g4_front.png")
Q=P.reshape(-1,3); Q=Q[np.isfinite(Q).all(1)]
# points near cup xy above table: find cup body
sel=Q[(np.hypot(Q[:,0]+0.1207,Q[:,1]-0.0448)<0.08)&(Q[:,2]>0.885)&(Q[:,2]<1.2)]
print("pts near cup column: n=%d z range %.3f-%.3f"%(len(sel),sel[:,2].min(),sel[:,2].max()))
low=Q[(np.hypot(Q[:,0]+0.1207,Q[:,1]-0.0448)<0.06)&(Q[:,2]>0.885)&(Q[:,2]<0.95)]
print("pts still low (0.885-0.95):",len(low))
