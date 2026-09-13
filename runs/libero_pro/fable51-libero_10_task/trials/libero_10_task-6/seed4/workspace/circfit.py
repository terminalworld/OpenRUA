import numpy as np
from scipy.optimize import least_squares
def fit(pts, r=None):
    x,y = pts[:,0], pts[:,1]
    if r is None:
        f = lambda p: np.hypot(x-p[0], y-p[1]) - p[2]; p0=[x.mean(), y.mean(), 0.04]
    else:
        f = lambda p: np.hypot(x-p[0], y-p[1]) - r; p0=[x.mean(), y.mean()]
    return least_squares(f, p0, loss="soft_l1", f_scale=0.005).x
