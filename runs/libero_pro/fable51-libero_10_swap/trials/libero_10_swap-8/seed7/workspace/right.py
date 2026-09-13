"""Right pot B (lying on the stove, base toward robot, handle arch on top).
Usage: python3 -u right.py dry | pick | place
  pick : grasp the handle arch top with a pitched hand and lift (pot swings to hang)
  place: touch down, roll the pot upright about its base edge along an arc, release
Pot frame: a = axis from base, h = toward handle. Grip point G=(11.4, 9.4) cm, CoM~(6.0,0.5).
"""
import sys
from rob import *

Y = 0.096                     # pot axis y (lying) and final pot y
GX, GZ = 0.235, 1.055         # TCP on the arch top (arch top z~1.064)
PRE_Z, LIFT_Z = 1.14, 1.20
T0 = 1.0                      # initial hand pitch toward +x (rad)
# hang geometry (see notes): CoM under G -> pot rotated psi=31.2deg from lying
PSI = math.radians(31.2)
E_OFF = (-0.0307, -0.1693)    # base edge E relative to G while hanging
R_ARC = 0.170                 # |G-E| = 0.172, run 2 mm inside (gentle press)
A0, A1 = math.radians(79.7), math.radians(138.5)   # arc angles of G about E
POT_X = 0.175                 # desired final pot centre x
E_X = POT_X + 0.035
G0 = (E_X - E_OFF[0], Y, 0.93 - E_OFF[1])          # touchdown TCP

def arc_points(n=8):
    pts = []
    for k in range(n + 1):
        th = A0 + (A1 - A0) * k / n
        tcp = (E_X + R_ARC * math.cos(th), Y, 0.93 + R_ARC * math.sin(th))
        tilt = T0 - (th - A0)
        pts.append((tcp, tilt))
    return pts

def solve(r, tcp, tilt, seed):
    q = r.ik_tcp(tcp, xbar_R(tilt), seed=seed)
    if q is None:
        raise SystemExit(f"IK failed for {np.round(tcp,3)} tilt {tilt:.2f}")
    return q

def go(r, tcp, tilt, seconds, seed=None):
    q = solve(r, tcp, tilt, seed if seed is not None else r.arm_q())
    print(f"-> tcp {np.round(tcp,3)} tilt {tilt:.2f} q {np.round(q,3)}", flush=True)
    code, err = r.move_q(q, seconds)
    if err > 0.02:
        print("  retry move", flush=True); code, err = r.move_q(q, seconds)
    pos, quat, t = r.fk_hand()
    print(f"  fk tcp now {np.round(t,4)} (err {np.linalg.norm(t-np.array(tcp))*1000:.1f} mm)", flush=True)
    return q

def main():
    mode = sys.argv[1]
    r = Robot()
    print("fingers", r.fingers(), "q", np.round(r.arm_q(), 3), flush=True)
    print("touchdown G0", np.round(G0, 4), flush=True)
    if mode == "dry":
        seed = r.arm_q()
        for name, tcp, t in [("pre", (GX, Y, PRE_Z), T0), ("grasp", (GX, Y, GZ), T0), ("lift", (GX, Y, LIFT_Z), T0),
                             ("above G0", (G0[0], Y, LIFT_Z), T0), ("G0", G0, T0)]:
            seed = solve(r, tcp, t, seed); print(f"{name}: {np.round(tcp,3)} t{t:.2f} -> {np.round(seed,2)}")
        prev = seed
        for tcp, t in arc_points():
            q = solve(r, tcp, t, prev)
            print(f"arc {np.round(tcp,3)} tilt {t:.2f} -> {np.round(q,2)} jump {np.max(np.abs(np.array(q)-np.array(prev))):.2f}")
            prev = q
        return
    if mode == "pick":
        r.gripper(GRIP["open_m"])
        go(r, (GX, Y, PRE_Z), T0, 4.0)
        go(r, (GX, Y, GZ), T0, 2.5)
        f = r.gripper(GRIP["closed_m"])
        gap = abs(f[0]) + abs(f[1])
        print(f"  finger gap after close: {gap*1000:.1f} mm", flush=True)
        if gap < 0.004:
            print("GRASP FAILED", flush=True)
            r.gripper(GRIP["open_m"]); go(r, (GX, Y, PRE_Z), T0, 2.5); return
        go(r, (GX, Y, LIFT_Z), T0, 3.0)
        print("fingers after lift", r.fingers(), flush=True)
    elif mode == "place":
        go(r, (G0[0], Y, LIFT_Z), T0, 3.0)
        go(r, (G0[0], Y, G0[2] + 0.01), T0, 3.0)
        go(r, G0, T0, 2.0)
        prev = r.arm_q()
        for tcp, t in arc_points():
            prev = go(r, tcp, t, 1.5, seed=prev)
        print("fingers at end of arc", r.fingers(), flush=True)
        r.gripper(GRIP["open_m"])
        tcp, t = arc_points()[-1]
        go(r, (tcp[0], Y, LIFT_Z), 0.0, 3.0)
    print("done", flush=True)

main()
