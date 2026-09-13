import numpy as np
P=np.load("eih1_P.npy").reshape(-1,3); P=P[np.isfinite(P).all(1)]
# middle column y in [-0.19,-0.09]; profile along x at 2mm: max z and floor presence
sel=P[(P[:,1]>-0.19)&(P[:,1]<-0.09)&(P[:,0]>-0.50)&(P[:,0]<-0.30)]
xs=np.arange(-0.50,-0.30,0.002)
for x in xs:
    s=sel[(sel[:,0]>=x)&(sel[:,0]<x+0.002)]
    if len(s)==0: print("%.3f  -"%x); continue
    z=s[:,2]
    print("%.3f n=%4d zmax=%.3f zmin=%.3f pct90=%.3f  floor(<0.91)=%d"%(x,len(s),z.max(),z.min(),np.percentile(z,90),(z<0.91).sum()))
