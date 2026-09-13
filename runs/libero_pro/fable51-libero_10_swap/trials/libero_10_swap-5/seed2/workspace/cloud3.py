import numpy as np
d=np.load('snaps/bird_depth.npy')
fx=fy=579.4112549695428; cx=320; cy=240
H,W=d.shape
vv,uu=np.mgrid[0:H,0:W]
X=(vv-cy)*d/fy - 0.2; Y=(uu-cx)*d/fx; Z=3.0-d
np.set_printoptions(precision=3, suppress=True, linewidth=200)
print('back wall row v=162..164 Z:', Z[162:165,215:300].max(0)[::5])
print('front wall rows v=199..203 Z:', Z[199:204,215:300].max(0)[::5])
print('divider row v=183 Z:', Z[182:185,260:300].max(0)[::4])
print('left wall Z:', Z[165:200,211:216].max(1)[::5])
print('mid divider u=257-259 Z:', Z[165:200,256:261].max(1)[::5])
print('X of back wall', X[162,250], 'X of front wall', X[201,250], 'X divider', X[183,280])
print('Y left wall', Y[180,213], 'Y mid divider', Y[180,258])
print('floor Z in back-right', Z[170:180,265:295].mean(), 'front-right', Z[187:197,265:295].mean(), 'left', Z[170:195,220:250].mean())
print('mug region: Z max', Z[225:260,340:385].max(), 'at', np.unravel_index(Z[225:260,340:385].argmax(), (35,45)))
