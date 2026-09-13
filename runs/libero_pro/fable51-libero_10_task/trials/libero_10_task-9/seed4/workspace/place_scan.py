import numpy as np, itertools
from grasp_eval import *
np.set_printoptions(precision=3, suppress=True)
MP = mug_points(step=0.004)
def evaluate(x_m, y_m, phi, theta, d, alpha=np.pi, drop=0.01):
    Rm = rot_axis([1, 0, 0], phi)
    # place mug so its lowest point is `drop` above the cavity floor
    mp0 = MP @ Rm.T
    zmin = mp0[:, 2].min()
    base = np.array([x_m, y_m, CAV_Z[0] + drop - zmin])
    mp = mp0 + base
    hw, Rh, tcp = grasp_hand_pose(base, alpha, theta, d, mug_R=Rm)
    hp = transform(hand_points((0.0025, 0.0025)), Rh, hw)
    cm, pm = clearance(mp, include_door=False)
    ch, ph = clearance(hp, include_door=False)
    com = base + Rm @ np.array([0, 0, 0.056])
    return cm, ch, com, base, hw, Rh
if __name__ == "__main__":
    res = []
    for y_m, phi_d, th_d, d in itertools.product(np.arange(0.25, 0.36, 0.01), [0, 10, 15, 20, 25, 30], [0, 10, 20], [0.010, 0.015]):
        for sgn in (1, -1):
            cm, ch, com, base, hw, Rh = evaluate(-0.05, y_m, sgn * np.radians(phi_d), sgn * np.radians(th_d), d)
            res.append((min(cm, ch), cm, ch, y_m, sgn * phi_d, sgn * th_d, d, com[1], base[2]))
    res.sort(key=lambda r: -r[0])
    for r in res[:25]:
        print("clr %.3f (mug %.3f hand %.3f) y_m %.2f phi %+d theta %+d d %.3f  com_y %.3f base_z %.3f" % r)
