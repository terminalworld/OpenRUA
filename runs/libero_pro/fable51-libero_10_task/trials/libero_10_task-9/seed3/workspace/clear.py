"""Hand vs microwave-front clearance model (world frame).

Hand model (relative to TCP, hand axes x_h,y_h,z_h): two fingers (each 0.01 along y_h,
0.02 along x_h, 0.045 along -z_h from the tip), body box 0.20 (y_h) x 0.065 (x_h) x 0.058 (-z_h)
sitting on top of the fingers. Finger gap g (per finger position) is given.
Microwave: front face y=0.271. Top frame slab: y>=0.271, z in [1.088,1.107] and above (top surface 1.107,
anything at y>=0.271 must have z>=1.107 or be inside the cavity: x in [-0.16,0.05], z in [0.945,1.088]).
Returns min clearance (negative = penetration) over sampled points.
"""
import numpy as np

YF = 0.271
ZTOP = 1.107
ZCEIL = 1.088
ZFLOOR = 0.945
XL, XR = -0.16, 0.05


def hand_points(tcp, R, g=0.004, n=6):
    xh, yh, zh = R[:, 0], R[:, 1], R[:, 2]
    pts = []
    # fingers
    for s in (+1, -1):
        for a in np.linspace(-0.01, 0.01, 3):          # along x_h
            for b in np.linspace(g, g + 0.01, 3):      # along y_h (from gap face outward)
                for c in np.linspace(0, 0.045, n):     # along -z_h from tip
                    pts.append(tcp + a * xh + s * b * yh - c * zh)
    # body
    for a in np.linspace(-0.0325, 0.0325, 3):
        for b in np.linspace(-0.105, 0.105, 9):
            for c in np.linspace(0.045, 0.045 + 0.058, 4):
                pts.append(tcp + a * xh + b * yh - c * zh)
    return np.array(pts)


def clearance(pts):
    """Signed clearance of points from microwave solid (front region only)."""
    y, z, x = pts[:, 1], pts[:, 2], pts[:, 0]
    d = np.full(len(pts), 1.0)
    inside_y = y >= YF
    # above top: fine (clearance = z - ZTOP)
    above = z >= ZTOP
    d[inside_y & above] = np.minimum(d[inside_y & above], (z - ZTOP)[inside_y & above])
    # in cavity: clearance to ceiling/floor/side walls
    cav = inside_y & (z < ZCEIL) & (z > ZFLOOR) & (x > XL) & (x < XR)
    d[cav] = np.minimum.reduce([d[cav], (ZCEIL - z)[cav], (z - ZFLOOR)[cav], (x - XL)[cav], (XR - x)[cav]])
    # in frame slab or walls: penetration
    bad = inside_y & ~above & ~cav
    pen = np.minimum(np.abs(z - ZCEIL), np.abs(z - ZTOP))
    d[bad] = -pen[bad]
    # points in front of the face: clearance = YF - y (only matters if within frame z band)
    front = ~inside_y
    band = (z > ZCEIL) & (z < ZTOP)
    d[front & band] = np.minimum(d[front & band], (YF - y)[front & band])
    return d.min()


def hand_clearance(tcp, R, g=0.004):
    return clearance(hand_points(np.asarray(tcp, float), R, g))
