import numpy as np
import cloud

def knob_and_flats(tcp, guess, P=None):
    """pot centre from knob-top centroid; flat-normal angle from lid octagon"""
    if P is None:
        P, _, _ = cloud.grab("robot0_eye_in_hand")
    m = (np.abs(P[..., 0] - guess[0]) < 0.08) & (np.abs(P[..., 1] - guess[1]) < 0.08) \
        & (P[..., 2] < tcp[2] - 0.008) & np.isfinite(P).all(-1)
    pts = P[m]
    ztop = np.percentile(pts[:, 2], 99.5)
    knob = pts[pts[:, 2] > ztop - 0.006]
    kc = knob[:, :2].mean(0)
    # knob is a disc: refine with the extents midpoint (robust to partial occlusion? use max-extent)
    lid = pts[(pts[:, 2] > ztop - 0.032) & (pts[:, 2] < ztop - 0.012)]  # lid top, below knob
    q = lid[:, :2] - kc
    def ext(a):
        u = np.array([np.cos(np.deg2rad(a)), np.sin(np.deg2rad(a))]); pr = q @ u
        return pr.min(), pr.max()
    angs = np.arange(0, 45, 0.5)
    e = np.array([np.subtract(*ext(a)[::-1]) for a in angs])
    amin = angs[np.argmin(e)]
    # extents along x (vertex axis) for centre check: midpoint along 0 deg
    lo, hi = ext(0.0)
    return dict(knob_centre=kc, z_top=ztop, flat_normal_deg=float(amin), ftf=float(e.min()),
                vtv=float(e.max()), x_ext=(lo, hi), x_mid=kc[0] + (lo + hi) / 2, n_knob=len(knob))
