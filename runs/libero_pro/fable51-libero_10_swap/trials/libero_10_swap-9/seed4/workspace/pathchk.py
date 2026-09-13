import numpy as np
from rob import Robot
from scipy.spatial.transform import Rotation as Rot

MW = dict(x=(-0.27, 0.10), y=(-0.325, -0.12), z=(0.85, 1.115))  # microwave body bbox (world)


def in_box(p, b):
    return b["x"][0] < p[0] < b["x"][1] and b["y"][0] < p[1] < b["y"][1] and b["z"][0] < p[2] < b["z"][1]


def check_path(r, q_from, q_to, n=15, mug=True, verbose=False):
    """Sample joint interpolation; report min clearance-ish info. mug: mug hangs at TCP + (0, +0.075, -0.055..+0.045)."""
    bad = []
    for i in range(n + 1):
        q = q_from + (q_to - q_from) * i / n
        pos, quat = r.fk_pose(q, "panda_hand")
        R = Rot.from_quat(quat).as_matrix()
        tcp = pos + 0.1034 * R[:, 2]
        pts = {"hand": pos, "tcp": tcp}
        pts["l7"] = r.fk_pose(q, "panda_link7")[0]
        pts["l6"] = r.fk_pose(q, "panda_link6")[0]
        if mug:
            c = tcp + np.array([0.002, 0.075, 0])
            pts["mug_bot"] = c + np.array([0, 0, -0.055])
            pts["mug_top"] = c + np.array([0, 0, 0.045])
            pts["mug_edge_y"] = c + np.array([0, 0.048, -0.055])
            pts["mug_edge_ym"] = c + np.array([0, -0.048, -0.055])
        hits = [k for k, p in pts.items() if in_box(p, MW)]
        if verbose or hits:
            print(i, "tcp", np.round(tcp, 3), "l7", np.round(pts["l7"], 3), "HITS" if hits else "", hits)
        bad += hits
    return bad
