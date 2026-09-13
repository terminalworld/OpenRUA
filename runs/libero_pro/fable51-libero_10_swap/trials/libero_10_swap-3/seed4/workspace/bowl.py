import numpy as np, cv2
wx=np.load('bird_wx.npy'); wy=np.load('bird_wy.npy'); wz=np.load('bird_wz.npy')
m=((wz>0.905)&(wz<1.0)).astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(m)
for i in range(1,n):
    mk=lab==i
    if stats[i,4]<50: continue
    print(f"comp {i}: n={stats[i,4]} px bbox u{stats[i,0]}-{stats[i,0]+stats[i,2]} v{stats[i,1]}-{stats[i,1]+stats[i,3]} x[{wx[mk].min():.3f},{wx[mk].max():.3f}] y[{wy[mk].min():.3f},{wy[mk].max():.3f}] z[{wz[mk].min():.3f},{wz[mk].max():.3f}] cen x={wx[mk].mean():.3f} y={wy[mk].mean():.3f}")
