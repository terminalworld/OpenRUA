"""Usage: python3 exec.py <pick|place> <A|B>   (logs to stdout; run with -u)"""
import sys
from rob import *

POTS = {"A": (-0.204, -0.202), "B": (-0.072, 0.225)}
HANDLE_DY = -0.0605         # handle bar centre relative to pot centre (handles point -y)
GRASP_Z = 1.025             # TCP height on the handle bar (~12.6 cm above table)
PRE_Z = 1.15
LIFT_Z = 1.20
PLACE = {"A": (0.18, 0.00), "B": (0.18, 0.097)}   # TCP lands at (x+CARRY_DX, y); pot body settles at or just beyond the TCP
PLACE_Z = 1.075             # contact happens ~1.085 with the hanging pot; press gently
CARRY_DX = -0.06            # after -90deg yaw the handle points -x: TCP sits 6 cm toward robot

def goto(r, tcp, yaw, seconds, seed=None, alt_yaws=()):
    for y in (yaw,) + tuple(alt_yaws):
        q = r.ik_tcp(tcp, grasp_R(y, 0), seed=seed)
        if q is not None: break
    if q is None:
        raise SystemExit(f"IK failed for {tcp}")
    print(f"-> tcp {np.round(tcp,3)} yaw {y:.2f} q {np.round(q,3)}", flush=True)
    code, err = r.move_q(q, seconds)
    if err > 0.02:
        print("  retry move", flush=True); code, err = r.move_q(q, seconds)
    pos, quat, t = r.fk_hand()
    print(f"  fk tcp now {np.round(t,4)}  (err {np.round(np.linalg.norm(t-np.array(tcp))*1000,1)} mm)", flush=True)
    return q

def main():
    mode, pot = sys.argv[1], sys.argv[2]
    r = Robot()
    print("fingers", r.fingers(), "q", np.round(r.arm_q(),3), flush=True)
    if mode == "pick":
        px, py = POTS[pot]; hx, hy = px, py + HANDLE_DY
        r.gripper(GRIP["open_m"])
        goto(r, (hx, hy, PRE_Z), 0.0, 4.0, alt_yaws=(math.pi,))
        goto(r, (hx, hy, GRASP_Z), 0.0, 2.5, alt_yaws=(math.pi,))
        f = r.gripper(GRIP["closed_m"])
        gap = abs(f[0]) + abs(f[1])
        print(f"  finger gap after close: {gap*1000:.1f} mm", flush=True)
        if gap < 0.004:
            print("GRASP FAILED (closed on air)", flush=True)
            r.gripper(GRIP["open_m"]); goto(r, (hx, hy, PRE_Z), 0.0, 2.5, alt_yaws=(math.pi,)); return
        goto(r, (hx, hy, LIFT_Z), 0.0, 3.0, alt_yaws=(math.pi,))
        print("fingers after lift", r.fingers(), flush=True)
    elif mode == "place":
        tx, ty = PLACE[pot]; cx, cy = tx + CARRY_DX, ty
        goto(r, (cx, cy, LIFT_Z), -math.pi/2, 5.0, alt_yaws=(math.pi/2,))
        print("fingers at carry", r.fingers(), flush=True)
        goto(r, (cx, cy, PLACE_Z), -math.pi/2, 3.0, alt_yaws=(math.pi/2,))
        r.gripper(GRIP["open_m"])
        goto(r, (cx, cy, LIFT_Z), -math.pi/2, 3.0, alt_yaws=(math.pi/2,))
    print("done", flush=True)

main()
