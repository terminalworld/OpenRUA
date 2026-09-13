"""Key-point clearance check for hand (+held bottle) against scene boxes."""
import numpy as np, math
from panda import R_from_axes, make_T
BOXES = {
 'panel':  ([-0.11, 0.070, 0.90], [0.12, 0.085, 1.00]),
 'handle': ([-0.04, 0.039, 0.94], [0.05, 0.053, 0.96]),
 'wall-x': ([-0.11, 0.085, 0.90], [-0.09, 0.30, 1.00]),
 'wall+x': ([0.11, 0.085, 0.90], [0.12, 0.30, 1.00]),
 'cabinet':([-0.116, 0.228, 0.90], [0.136, 0.417, 1.127]),
 'tophandle':([-0.045, 0.188, 1.015], [0.055, 0.23, 1.105]),
 'floor':  ([-0.09, 0.085, 0.80], [0.11, 0.30, 0.923]),
 'table':  ([-1, -1, 0.0], [1, 1, 0.90]),
}
def hand_pts(T, open_=True):
    """points in hand frame (TCP origin, z=approach, y=finger axis)."""
    pts = []
    for x in (-0.03, 0.03):
        for y in np.linspace(-0.10, 0.10, 11):
            for z in (-0.1034, -0.07, -0.0374):   # palm from flange side to bottom face
                pts.append([x, y, z])
    fy = 0.05 if open_ else 0.017
    for x in (-0.01, 0.01):
        for y in (-fy, fy):
            for z in np.linspace(-0.0374, 0.0, 5):  # fingers from palm to pad center (pad center = TCP)
                pts.append([x, y, z])
                pts.append([x, y, z + 0.0115])      # pad tips a bit beyond TCP
    P = np.array(pts)
    return (T[:3, :3] @ P.T).T + T[:3, 3]
def bottle_pts(T, axis_dir_hand):
    """bottle held at neck (TCP), 2 cm from tip; body toward axis_dir_hand (unit, hand frame)."""
    d = np.array(axis_dir_hand, float)
    pts = []
    # neck: from -0.02 to +0.04 along d, r=0.007; body from 0.04 to 0.14, r=0.0275
    for s, r in [(-0.02, 0.007), (0.0, 0.007), (0.04, 0.007), (0.045, 0.0275), (0.09, 0.0275), (0.14, 0.0275)]:
        for ang in np.linspace(0, 2*math.pi, 12, endpoint=False):
            # perpendicular basis in hand frame
            u = np.cross(d, [0, 0, 1.0]);
            if np.linalg.norm(u) < 1e-6: u = np.cross(d, [0, 1.0, 0])
            u /= np.linalg.norm(u); v = np.cross(d, u)
            pts.append(s * d + r * (math.cos(ang) * u + math.sin(ang) * v))
    P = np.array(pts)
    return (T[:3, :3] @ P.T).T + T[:3, 3]
def dist(P, lo, hi):
    lo, hi = np.array(lo), np.array(hi)
    d = np.maximum(np.maximum(lo - P, 0), P - hi)
    return np.linalg.norm(d, axis=-1).min()
def report(T, open_=True, bottle_dir=None, skip=()):
    H = hand_pts(T, open_)
    out = {}
    for n, (lo, hi) in BOXES.items():
        if n in skip: continue
        out[n] = dist(H, lo, hi)
        if bottle_dir is not None:
            out[n + '/bottle'] = dist(bottle_pts(T, bottle_dir), lo, hi)
    return out
def place_pose(beta, gamma, p):
    """finger axis f rolled by beta about x (+y finger up), approach down/+y; then pitch gamma about world y."""
    f = np.array([0, math.cos(beta), math.sin(beta)])
    a = np.array([0, math.sin(beta), -math.cos(beta)])
    R0 = R_from_axes(a, y=f)
    c, s = math.cos(gamma), math.sin(gamma)
    Ry = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    return make_T(p, Ry @ R0)
if __name__ == '__main__':
    import sys
    beta, gamma, z = [float(v) for v in sys.argv[1:4]]
    T = place_pose(math.radians(beta), math.radians(gamma), [-0.05, 0.135, z])
    print('hand x axis (bottle axis = -x):', T[:3, 0].round(3), 'z axis', T[:3, 2].round(3))
    bd = [-1, 0, 0]  # body toward hand -x  (world +x)
    B = bottle_pts(T, bd)
    print('bottle world x %.3f..%.3f y %.3f..%.3f z %.3f..%.3f' % (B[:,0].min(), B[:,0].max(), B[:,1].min(), B[:,1].max(), B[:,2].min(), B[:,2].max()))
    for k, v in report(T, True, bd).items():
        print(f'  {k:18s} {v:+.3f}')
