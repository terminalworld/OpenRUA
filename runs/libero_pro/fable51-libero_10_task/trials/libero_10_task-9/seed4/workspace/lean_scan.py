import numpy as np, itertools
from grasp_eval import *
np.set_printoptions(precision=3, suppress=True)
MP = mug_points(step=0.004)
def lean_pose(beta, y_p, z_p, x_m=-0.05):
    """Mug leaning toward -y by beta (top toward robot); its base's -y edge at (y_p, z_p)."""
    Rm = rot_axis([1, 0, 0], beta)
    edge_local = np.array([0, -MUG_R, 0.0])          # base -y edge in mug frame
    base = np.array([x_m, y_p, z_p]) - Rm @ edge_local
    return Rm, base
def evaluate(beta, y_p, z_p, theta, d, alpha=np.pi, x_m=-0.05):
    Rm, base = lean_pose(beta, y_p, z_p, x_m)
    mp = MP @ Rm.T + base
    hw, Rh, tcp = grasp_hand_pose(base, alpha, theta, d, mug_R=Rm)
    hp = transform(hand_points((0.0025, 0.0025)), Rh, hw)
    cm, pm = clearance(mp, include_door=False)
    ch, ph = clearance(hp, include_door=False)
    com = base + Rm @ np.array([0, 0, 0.056])
    return cm, ch, pm, ph, com, base, hw, Rh
if __name__ == "__main__":
    res = []
    for beta_d, y_p, z_p, th_d, d in itertools.product([20, 25, 30, 35], np.arange(0.23, 0.31, 0.01), [0.957, 0.945], [0, 10, 20], [0.010, 0.015]):
        cm, ch, pm, ph, com, base, hw, Rh = evaluate(np.radians(beta_d), y_p, z_p, np.radians(th_d), d)
        res.append((min(cm, ch), cm, ch, beta_d, y_p, z_p, th_d, d, com[1] - y_p, pm, ph))
    res.sort(key=lambda r: -r[0])
    for r in res[:30]:
        print("clr %.3f (mug %.3f hand %.3f) beta %d y_p %.2f z_p %.3f theta %d d %.3f  com-yp %+.3f mugpt %s handpt %s" % r)
