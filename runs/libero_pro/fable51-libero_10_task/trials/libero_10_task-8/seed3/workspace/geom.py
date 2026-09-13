#!/usr/bin/env python3
"""Rough clearance check of a pitched side-grasp on pot A against boxes.
Hand model in the hand frame (origin = TCP, axis a = approach, c = closing):
 - fingers: s in [0,0.045], c in +-[0.0387, 0.0587], hx +-0.01
 - hand body: s in [0.045,0.1034], c +-0.1, hx +-0.035
 - wrist: s in [0.1034, 0.30], radius 0.05
Points sampled on each obstacle box; report min signed distance per part."""
import numpy as np, sys

OBST = {
    "potA_body_low": ([0.013, -0.002, 0.90], [0.088, 0.072, 0.945]),
    "potA_waist": ([0.018, 0.003, 0.945], [0.084, 0.067, 0.98]),
    "potA_upper": ([0.015, 0.0, 0.98], [0.087, 0.071, 1.03]),
    "potA_lid": ([0.012, -0.003, 1.03], [0.088, 0.073, 1.046]),
    "potA_knob": ([0.043, 0.028, 1.046], [0.058, 0.043, 1.062]),
    "potA_spout": ([0.038, 0.067, 0.985], [0.062, 0.095, 1.04]),
    "potA_handle": ([0.038, -0.05, 0.945], [0.062, 0.0, 1.04]),
    "potB_body": ([-0.09, 0.19, 0.90], [-0.03, 0.27, 1.062]),
    "potB_handle": ([-0.075, 0.14, 0.945], [-0.045, 0.19, 1.04]),
    "table": ([-0.5, -0.5, 0.80], [0.6, 0.6, 0.90]),
}


def frame(phi_deg, elev_deg):
    ph, el = np.radians(phi_deg), np.radians(elev_deg)
    src = np.array([np.cos(ph) * np.cos(el), np.sin(ph) * np.cos(el), np.sin(el)])  # -approach
    a = -src
    c = np.array([np.sin(ph), -np.cos(ph), 0.0])
    hx = np.cross(c, a)
    return a, c, hx


def box_pts(lo, hi, n=8):
    g = [np.linspace(lo[i], hi[i], n) for i in range(3)]
    return np.array(np.meshgrid(*g, indexing="ij")).reshape(3, -1).T


def check(tcp, phi, elev, verbose=True):
    a, c, hx = frame(phi, elev)
    worst = {}
    for name, (lo, hi) in OBST.items():
        P = box_pts(lo, hi) - np.array(tcp)
        s = -(P @ a); pc = P @ c; ph = P @ hx
        # fingers (grasp target is potA waist so skip finger/potA_waist check)
        d_f = np.maximum.reduce([-s, s - 0.045, 0.0387 - np.abs(pc), np.abs(pc) - 0.0587, np.abs(ph) - 0.01])
        d_h = np.maximum.reduce([0.045 - s, s - 0.1034, np.abs(pc) - 0.1, np.abs(ph) - 0.035])
        d_w = np.maximum.reduce([0.1034 - s, s - 0.30, np.hypot(pc, ph) - 0.05])
        worst[name] = (d_f.min(), d_h.min(), d_w.min())
    if verbose:
        print(f"phi={phi} elev={elev} tcp={tcp}")
        for k, v in worst.items():
            flag = " <-- COLLISION" if min(v) < 0 else ""
            print(f"  {k:15s} finger={v[0]:+.3f} hand={v[1]:+.3f} wrist={v[2]:+.3f}{flag}")
    return worst


if __name__ == "__main__":
    tcp = [float(v) for v in sys.argv[1:4]]
    check(tcp, float(sys.argv[4]), float(sys.argv[5]))
