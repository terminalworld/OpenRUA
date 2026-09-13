import numpy as np
# cup: truncated cone, bottom r=0.037, top r=0.052, h=0.16 (axis frame, bottom at origin)
rb, rt, H = 0.036, 0.050, 0.11
# slot geometry (xz plane, y ignored): back wall inner face x=-0.464 (z<=1.056), floor z=0.904,
# divider face x=-0.405 (z<=0.988), divider top z=0.988 for x in [-0.405,-0.397], front compartment beyond.
# Also back wall top z=1.056 for x in [-0.475,-0.464].
XL, ZLT = -0.464, 1.056
XR, ZRT = -0.405, 0.988
ZF = 0.904
# sample cup surface points in its own frame (x along tilt direction, z up)
hs = np.linspace(0, H, 33); phis = np.linspace(0, 2*np.pi, 72, endpoint=False)
pts = []
for h in hs:
    r = rb + (rt-rb)*h/H
    for p in phis:
        pts.append([r*np.cos(p), r*np.sin(p), h])
# bottom disk
for rr in np.linspace(0, rb, 6):
    for p in phis:
        pts.append([rr*np.cos(p), rr*np.sin(p), 0.0])
P = np.array(pts)
def free(x, z):
    # inside solid? back wall: x<=XL and z<=ZLT ; below floor ; divider region x>=XR and z<=ZRT (treat everything right of XR below ZRT as solid)
    solid = ((x <= XL) & (z <= ZLT)) | (z <= ZF) | ((x >= XR) & (z <= ZRT))
    return ~solid
best = {}
for th_deg in range(-50, 51, 5):
    th = np.deg2rad(th_deg)  # positive: top leans toward +x
    R = np.array([[np.cos(th), np.sin(th)], [-np.sin(th), np.cos(th)]])  # rotate about y in xz
    xz = P[:, [0, 2]] @ R.T
    lo = None
    for xc in np.arange(-0.47, -0.36, 0.001):
        # find lowest zc such that all points free
        for zc in np.arange(0.90, 1.20, 0.001):
            x = xz[:, 0] + xc; z = xz[:, 1] + zc
            if free(x, z).all():
                if lo is None or zc < lo[1]:
                    lo = (xc, zc)
                break
    xc, zc = lo
    # cup mid-height center in world
    cx = xc + (H/2)*np.sin(th); cz = zc + (H/2)*np.cos(th)
    print(f"tilt {th_deg:+3d}: bottom-center=({xc:.3f},{zc:.3f})  mid-center=({cx:.3f},{cz:.3f})")
