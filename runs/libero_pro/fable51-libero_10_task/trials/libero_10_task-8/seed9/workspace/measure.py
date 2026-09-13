import numpy as np, rob, cloud, knob, precise

def measure(r, P=None):
    t, quat = r.tcp(); R = rob.quat_to_R(quat)
    if P is None:
        P, _, _ = cloud.grab("robot0_eye_in_hand")
    k = knob.knob_and_flats(t, t[:2], P)
    yaw = precise.yaw_of(quat)
    mis = ((k["flat_normal_deg"] - yaw + 22.5) % 45) - 22.5
    rel = P - t; hx = rel @ R[:, 0]; hy = rel @ R[:, 1]; hz = rel @ R[:, 2]
    fm = (np.abs(hx) < 0.017) & (hz > -0.06) & (hz < 0.015) & np.isfinite(P).all(-1) & (np.abs(hy) > 0.02)
    fl = -np.percentile(np.abs(hy[fm & (hy < 0)]), 1); fr = np.percentile(np.abs(hy[fm & (hy > 0)]), 1)
    lid = P[(np.abs(P[..., 0] - t[0]) < 0.08) & (np.abs(P[..., 1] - t[1]) < 0.08) & (P[..., 2] > k["z_top"] - 0.032) & (P[..., 2] < k["z_top"] - 0.012) & np.isfinite(P).all(-1)]
    pr = (lid[:, :2] - t[:2]) @ R[:2, 1]   # lid extent along closing axis, rel TCP
    px = (lid[:, :2] - t[:2]) @ R[:2, 0]
    off = k["knob_centre"] - t[:2]
    print(f"tcp {t.round(4)} yaw {yaw:.2f}  knob-TCP {(off*1000).round(1)} mm (hand x {1000*off@R[:2,0]:.1f}, hand y {1000*off@R[:2,1]:.1f})")
    print(f"knob top z {k['z_top']:.4f}  flat normal {k['flat_normal_deg']} -> misalign {mis:.2f} deg  ftf {k['ftf']*1000:.1f} vtv {k['vtv']*1000:.1f} mm")
    print(f"fingers hy {fl*1000:.1f}..{fr*1000:.1f}  lid along hy {pr.min()*1000:.1f}..{pr.max()*1000:.1f}  clearance {1000*(pr.min()-fl):.1f}/{1000*(fr-pr.max()):.1f} mm   lid along hx {px.min()*1000:.1f}..{px.max()*1000:.1f}")
    return dict(tcp=t, yaw=yaw, off=off, mis=mis, k=k, fl=fl, fr=fr, pr=(pr.min(), pr.max()))
