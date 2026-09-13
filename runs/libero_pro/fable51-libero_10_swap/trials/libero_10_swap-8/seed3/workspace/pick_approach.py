"""Approach a pot from -x (or given yaw) with a pitched side grasp, stop with the
pads around the lower chamber, gripper still open. Saves state to a json.

Usage: python3 pick_approach.py <pot_x> <pot_y> [yaw_deg] [pitch_deg] [d] [grasp_z]
"""
import json
import sys

from rob import *

TABLE = 0.894
PITCH = 10.0
D = 0.010          # pads stop this far short of the pot centre (along approach)
GRASP_Z = 0.040    # TCP height above table at grasp
BACK = 0.08        # stand-off behind the pot before sliding in




def main():
    px, py = float(sys.argv[1]), float(sys.argv[2])
    yaw = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
    pitch = float(sys.argv[4]) if len(sys.argv) > 4 else PITCH
    d = float(sys.argv[5]) if len(sys.argv) > 5 else D
    grasp_z = float(sys.argv[6]) if len(sys.argv) > 6 else GRASP_Z
    r = Robot()
    R = side_grasp_R(yaw, pitch)
    ap = np.array([math.cos(math.radians(yaw)), math.sin(math.radians(yaw)), 0.0])  # approach dir
    pot = np.array([px, py, 0.0])
    P3 = pot - d * ap + np.array([0, 0, TABLE + grasp_z])
    P2 = P3 - BACK * ap
    P1 = P2 + np.array([0, 0, 0.11])
    P0 = P1 + np.array([0, 0, 0.10])
    print("P0", P0.round(3), "P1", P1.round(3), "P2", P2.round(3), "P3", P3.round(3))

    q0 = r.arm_q()
    seeds = [q0, [0, -0.785, 0, -2.356, 0, 1.571, 0.785], [-0.4, 0.3, 0, -2.2, 0, 2.5, 0.4],
             [-0.9, 0.65, 0.0, -2.65, 2.3, 1.65, 0.65]]
    s0 = r.best_ik_tcp(P0, R, seeds)
    if s0 is None:
        raise SystemExit("no IK for P0")
    print("-> P0"); r.move_converged([s0], 4.0)
    s1 = r.ik_tcp(P1, R, seed=s0); print("-> P1"); r.move_converged([s1], 2.5)
    print("-> P2"); r.move_converged(cart_line(r, R, P1, P2, 0.03, s1), 3.0)
    print("-> P3 (slide in)"); r.move_converged(cart_line(r, R, P2, P3, 0.02, r.arm_q()), 5.0)
    t, _ = r.tcp()
    print("TCP now", t.round(4), "target", P3.round(4))
    json.dump({"pot": [px, py], "yaw": yaw, "pitch": pitch, "d": d, "grasp_z": grasp_z, "P3": P3.tolist(), "P1": P1.tolist(), "q": r.arm_q()},
              open("approach_state.json", "w"))


if __name__ == "__main__":
    main()
