import numpy as np, sys
from scipy.optimize import least_squares
d=np.load(sys.argv[1] if len(sys.argv)>1 else 'eih_depth.npy'); f=312.774
tab=np.median(d[50:150,250:400])
can=(d>0.108)&(d<tab-0.03); can[400:,:]=False; can[:,:150]=False; can[:,500:]=False
rows=[];lefts=[];rights=[]
for r in range(200,365):
    cs=np.where(can[r])[0]
    if len(cs)>20 and cs.max()-cs.min()<400:
        rows.append(r); lefts.append(cs.min()); rights.append(cs.max()+1)
rows=np.array(rows); lefts=np.array(lefts); rights=np.array(rights)
c0=(lefts+rights)/2
def resid(p):
    u0,v0,R=p
    half=np.sqrt(np.clip(R**2-(rows-v0)**2,0,None))
    return np.concatenate([lefts-(u0-half), rights-(u0+half)])
p=least_squares(resid,[np.median(c0),330,95]).x
u0,v0,R=p
Z=np.median(d[can])
print(f'circle center px ({u0:.1f},{v0:.1f}) R={R:.1f}px -> diam {2*R*Z/f:.4f} m, Z={Z:.4f}')
print('rows used',rows.min(),rows.max(),'median col center',np.median(c0))
