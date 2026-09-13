"""Carry the held pot to a target and set it down.

Usage: python3 place.py <pot_target_x> <pot_target_y> <carry_z> <place_tcp_z> [yaw_deg] [fwd_off] [side_off]
pot_target: where the pot centre should end up (world xy).
carry_z:    TCP height during transport.  place_tcp_z: TCP height at release.
fwd_off:    pot centre offset ahead of TCP along approach dir (measured, m).
side_off:   pot centre offset along hand +y from the TCP (m).
"""
import json
import sys

from rob import *

PITCH = 10.0




def main():
    tx, ty, carry_z, place_z = map(float, sys.argv[1:5])
    yaw = float(sys.argv[5]) if len(sys.argv) > 5 else 0.0
    fwd = float(sys.argv[6]) if len(sys.argv) > 6 else 0.02
    side = float(sys.argv[7]) if len(sys.argv) > 7 else 0.0
    r = Robot()
    R = side_grasp_R(yaw, PITCH)
    ap = R[:, 2].copy(); ap[2] = 0; ap /= np.linalg.norm(ap)
    sd = R[:, 1]
    tcp_xy = np.array([tx, ty, 0.0]) - fwd * ap - side * sd
    t0, _ = r.tcp()
    print("start TCP", t0.round(3), "gap", round(r.finger_gap(), 4))
    A = np.array([t0[0], t0[1], carry_z])
    B = np.array([tcp_xy[0], tcp_xy[1], carry_z])
    C = np.array([tcp_xy[0], tcp_xy[1], place_z])
    print("A", A.round(3), "B", B.round(3), "C", C.round(3))
    seed = r.arm_q()
    # sanity: IK reachable at C with good posture before moving
    sC = r.best_ik_tcp(C, R, [seed, [-0.2, 0.9, 0.0, -1.9, 2.3, 1.9, 0.6], [0, 0.6, 0, -2.0, 0, 2.6, 0.8]])
    if sC is None:
        raise SystemExit("place pose unreachable; nothing moved")
    print("-> up"); r.move_converged(cart_line(r, R, t0, A, 0.03, seed), 2.5)
    print("-> over target"); r.move_converged(cart_line(r, R, A, B, 0.04, r.arm_q()), 5.0)
    print("-> down"); r.move_converged(cart_line(r, R, B, C, 0.03, r.arm_q()), 4.0)
    t, _ = r.tcp(); print("TCP at release", t.round(4), "gap", round(r.finger_gap(), 4))
    json.dump({"yaw": yaw, "C": C.tolist(), "target": [tx, ty]}, open("place_state.json", "w"))


if __name__ == "__main__":
    main()
