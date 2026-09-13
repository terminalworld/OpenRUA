import numpy as np, sys
from cloudgen import cloud
fx=float(sys.argv[1]); cx=float(sys.argv[2]); cy=float(sys.argv[3])
Pw,d=cloud('snaps/eih_depth.npy',fx,fx,cx,cy,[-0.243,0.0,1.2673],[0.707,-0.7066,-0.0201,0.0201])
X,Y,Z=Pw[...,0],Pw[...,1],Pw[...,2]
np.set_printoptions(precision=3,suppress=True,linewidth=250)
print('corners', Pw[0,0],Pw[0,639],Pw[479,0],Pw[479,639])
print('table sample', Pw[200,320])
m=(Z>1.03)&(Z<1.1)&(X>-0.5)&(X<-0.3)
print('tall walls X',X[m].min(),X[m].max(),'Y',Y[m].min(),Y[m].max())
h,e=np.histogram(Y[m],bins=np.arange(-0.42,0.12,0.02))
for a,b in zip(e,h): print(round(a,2),b)
# for Y>-0.07 region, X histogram of tall walls
mm=m&(Y>-0.06)&(Y<0.06)
h,e=np.histogram(X[mm],bins=np.arange(-0.48,-0.30,0.01))
print('right column tall walls X hist'); 
for a,b in zip(e,h): print(round(a,2),b)
mm=(Z>0.93)&(Z<1.03)&(X>-0.5)&(X<-0.3)&(Y>-0.06)&(Y<0.06)
h,e=np.histogram(X[mm],bins=np.arange(-0.48,-0.30,0.01))
print('right column low walls X hist'); 
for a,b in zip(e,h): print(round(a,2),b)
# ascii map of Z on grid X -0.48..-0.30 (rows), Y -0.40..0.10 (cols), 1cm
mask=(X>-0.5)&(X<-0.28)&(Y>-0.42)&(Y<0.12)
xi=((X-(-0.48))/0.01).astype(int); yi=((Y-(-0.40))/0.01).astype(int)
grid=np.full((19,53),np.nan)
for a,b,z in zip(xi[mask],yi[mask],Z[mask]):
    if 0<=a<19 and 0<=b<53: grid[a,b]=np.nanmax([grid[a,b],z])
for r in range(19):
    print(f'{-0.48+r*0.01:+.2f} '+''.join('?' if np.isnan(z) else ('T' if z>1.03 else ('t' if z>0.98 else ('l' if z>0.93 else ('f' if z>0.89 else '.')))) for z in grid[r]))
