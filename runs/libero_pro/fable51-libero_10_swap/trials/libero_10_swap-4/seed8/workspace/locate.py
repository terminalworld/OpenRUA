import numpy as np, cv2, sys
d=np.load('bird_depth.npy')
f=579.4112549695428; cx,cy=320.0,240.0
camx,camy,camz=-0.2,0.0,3.0
def px2w(u,v,z):
    # birdview: u -> world y, v -> world x (px2world showed v=200 -> x=-0.329, v=284 -> x=-0.009)
    depth=z
    X=(u-cx)*depth/f; Y=(v-cy)*depth/f
    return (camx+Y, camy+X, camz-depth)
def ring(name,u0,v0,r=16,lo=2.40,hi=2.50):
    win=d[v0-r:v0+r,u0-r:u0+r]
    m=(win>lo)&(win<hi)
    vs,us=np.nonzero(m)
    us=us+u0-r; vs=vs+v0-r
    # circle fit (algebraic)
    A=np.c_[2*us,2*vs,np.ones(len(us))]; b=us**2+vs**2
    sol=np.linalg.lstsq(A,b,rcond=None)[0]
    ucen,vcen=sol[0],sol[1]; rad=np.sqrt(sol[2]+ucen**2+vcen**2)
    zr=win[m].mean()
    w=px2w(ucen,vcen,zr)
    print(f"{name}: n={len(us)} center px=({ucen:.1f},{vcen:.1f}) r_px={rad:.1f} -> world x={w[0]:.4f} y={w[1]:.4f} rim_z={w[2]:.4f} rim_diam={2*rad*zr/f:.3f}")
ring('white',286,261)
ring('yellow',329,237,r=14)
ring('red',352,271)
# plates: centroid of pixels within plate depth band
for name,u0,v0 in [('leftplate',253,284),('rightplate',386,284)]:
    win=d[v0-25:v0+25,u0-25:u0+25]; m=(win>2.545)&(win<2.565)
    vs,us=np.nonzero(m); u=us.mean()+u0-25; v=vs.mean()+v0-25; z=win[m].mean()
    w=px2w(u,v,z); print(f"{name}: px=({u:.1f},{v:.1f}) n={len(us)} world x={w[0]:.4f} y={w[1]:.4f} top_z={w[2]:.4f}")
