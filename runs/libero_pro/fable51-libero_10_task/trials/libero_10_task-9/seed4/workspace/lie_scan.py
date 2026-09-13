import numpy as np, itertools
from grasp_eval import *
np.set_printoptions(precision=3, suppress=True)
MP = mug_points(step=0.004)
def lie_eval(y_m, x_m, psi, beta, theta, d, drop=0.008, alpha=np.pi):
    # yaw psi about z (handle direction), then roll beta about world x (top toward -y)
    Rm = rot_axis([1, 0, 0], beta) @ rot_axis([0, 0, 1], psi)
    mp0 = MP @ Rm.T
    # centre of the mug body (mid-height) placed at (x_m, y_m); lowest point at floor+drop
    c0 = Rm @ np.array([0, 0, MUG_H / 2])
    base = np.array([x_m - c0[0], y_m - c0[1], 0.0])
    base[2] = CAV_Z[0] + drop - (mp0[:, 2].min())
    mp = mp0 + base
    hw, Rh, tcp = grasp_hand_pose(base, alpha, theta, d, mug_R=Rm)
    hp = transform(hand_points((0.0025, 0.0025)), Rh, hw)
    cm, pm = clearance(mp, include_door=False)
    ch, ph = clearance(hp, include_door=False)
    return cm, ch, pm, ph, hw, Rh, tcp, mp
if __name__ == "__main__":
    for psi_d in (-90, 90):
        for y_m in np.arange(0.30, 0.37, 0.01):
            for beta_d in (90, 80, 70):
                cm, ch, pm, ph, hw, Rh, tcp, mp = lie_eval(y_m, -0.05, np.radians(psi_d), np.radians(beta_d), 0.0, 0.015)
                print(f"psi {psi_d} y_m {y_m:.2f} beta {beta_d}: mug clr {cm:+.3f} at {pm}  hand clr {ch:+.3f} at {ph}  mug y-span [{mp[:,1].min():.3f},{mp[:,1].max():.3f}] z-span [{mp[:,2].min():.3f},{mp[:,2].max():.3f}] tcp {tcp}")
