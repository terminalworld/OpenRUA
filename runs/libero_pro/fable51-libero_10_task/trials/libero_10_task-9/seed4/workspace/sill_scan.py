"""Scan: mug held upright, hovering h above the SILL (y_m ~0.26), hand tilted
back by theta about the closing axis (world x), rim pinch at the -x point."""
import numpy as np
from geom import *
from rlib import R_from_axes, TCP as TCPOFF
np.set_printoptions(precision=3, suppress=True)
SILL_Z = OPEN_Z[0]
mugpts = mug_points(0.004)
handpts = hand_points((0.0025, 0.0025), 0.004)

def sill_eval(x_m, y_m, h, theta_d, d):
    th = np.radians(theta_d)
    base = np.array([x_m, y_m, SILL_Z + h])
    rim_z = base[2] + MUG_H
    tcp = np.array([x_m - 0.0435, y_m, rim_z - d])
    Rh = R_from_axes([0, np.sin(th), -np.cos(th)], [-1, 0, 0])
    hand = tcp - TCPOFF * Rh[:, 2]
    cm, pm = clearance(transform(mugpts, np.eye(3), base))
    ch, ph = clearance(transform(handpts, Rh, hand))
    return cm, pm, ch, ph, hand, Rh, tcp

if __name__ == "__main__":
    for x_m in (-0.03,):
        for y_m in (0.255, 0.26, 0.265):
            for h in (0.003, 0.006):
                for th in (45, 50, 55, 60):
                    for d in (0.008, 0.010):
                        cm, pm, ch, ph, hand, Rh, tcp = sill_eval(x_m, y_m, h, th, d)
                        flag = "OK " if min(cm, ch) > 0.004 else "   "
                        print(f"{flag}y_m {y_m} h {h} th {th} d {d}: mug {cm:+.3f} at {pm} hand {ch:+.3f} at {ph}  hand {hand} tcp {tcp}")
