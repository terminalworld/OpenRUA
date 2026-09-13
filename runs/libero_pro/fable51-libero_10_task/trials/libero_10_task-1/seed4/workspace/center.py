import numpy as np, cv2
d=np.load('eih_depth.npy'); f=579.4112549695428
tab = np.median(d[50:150,250:400]); print('table depth', tab)
roi = np.zeros_like(d,bool); roi[300:400,:]=True
fing = (d<0.11)&roi
ys,xs=np.where(fing)
left = xs<320; right=~left
print('left finger cols', xs[left].min(), xs[left].max(), 'rows', ys[left].min(), ys[left].max(), 'depth', np.median(d[ys[left],xs[left]]))
print('right finger cols', xs[right].min(), xs[right].max(), 'rows', ys[right].min(), ys[right].max(), 'depth', np.median(d[ys[right],xs[right]]))
mid_col = (xs[left].max()+xs[right].min())/2
print('finger inner faces:', xs[left].max(), xs[right].min(), 'mid col', mid_col, 'inner gap px', xs[right].min()-xs[left].max())
can = (d>0.11)&(d<tab-0.03); can[400:,:]=False
n,lab,st,cen=cv2.connectedComponentsWithStats(can.astype(np.uint8))
i=1+np.argmax(st[1:,4]); can=(lab==i)
cy,cx=np.where(can); Z=np.median(d[cy,cx])
print('can pixels',len(cy),'depth med',Z, 'rows',cy.min(),cy.max(),'cols',cx.min(),cx.max())
print('can centroid col,row', cx.mean(), cy.mean(), ' bbox center', (cx.min()+cx.max())/2, (cy.min()+cy.max())/2)
print('can diameter est (m)', (cx.max()-cx.min())*Z/f, (cy.max()-cy.min())*Z/f)
fd=np.median(d[ys,xs]); print('finger gap est (m) at finger depth', (xs[right].min()-xs[left].max())*fd/f)
vis=cv2.imread('eih3.png'); vis[can]=(0,255,0); vis[fing]=(255,0,0); cv2.imwrite('center_vis.png',vis)
