"""Phase A: re-lay pot B (lying along y, base -y) along x with base toward -x.
Usage: python3 -u relay.py dry|run"""
import sys
from rob import *
from flip import R_of
LIM = np.array([[-2.9,2.9],[-1.76,1.76],[-2.9,2.9],[-3.07,-0.07],[-2.9,2.9],[-0.02,3.75],[-2.9,2.9]])
def margin(q): q=np.array(q); return float(np.min(np.minimum(q-LIM[:,0], LIM[:,1]-q)))
ARCH = (-0.076, 0.21)     # arch-top ridge of lying pot
GZ, LIFT = 1.032, 1.20
NEWX = 0.0                 # TCP x when setting pot down along x (pot body x in [-0.10, 0.06])
STEPS = [("pre", (ARCH[0], ARCH[1], LIFT), 0.0, 4.0, None),
         ("grasp", (ARCH[0], ARCH[1], GZ), 0.0, 3.0, "close"),
         ("lift", (ARCH[0], ARCH[1], LIFT), 0.0, 3.0, None),
         ("yaw45", (ARCH[0], ARCH[1], LIFT), -math.pi/4, 2.5, None),
         ("yaw90", (ARCH[0], ARCH[1], LIFT), -math.pi/2, 2.5, None),
         ("shift", (NEWX, ARCH[1], LIFT), -math.pi/2, 3.0, None),
         ("down", (NEWX, ARCH[1], GZ+0.03), -math.pi/2, 3.0, None),
         ("set", (NEWX, ARCH[1], GZ+0.002), -math.pi/2, 2.0, "open"),
         ("up", (NEWX, ARCH[1], LIFT), -math.pi/2, 3.0, None)]
def main():
    r = Robot(); mode = sys.argv[1]
    print("fingers", r.fingers(), "q", np.round(r.arm_q(),3), flush=True)
    prev = r.arm_q()
    plan = []
    for name, tcp, yaw, sec, act in STEPS:
        q = r.ik_tcp(tcp, grasp_R(yaw, 0), seed=prev)
        if q is None: raise SystemExit(f"IK fail {name}")
        jump = float(np.max(np.abs(np.array(q)-np.array(prev))))
        print(f"{name:6s} {np.round(tcp,3)} yaw {yaw:.2f} -> {np.round(q,2)} margin {margin(q):.2f} jump {jump:.2f}", flush=True)
        if jump > 1.6 and name != "pre": raise SystemExit("branch jump")
        plan.append((name, q, sec, act)); prev = q
    if mode != "run": return
    r.gripper(GRIP["open_m"])
    for name, q, sec, act in plan:
        code, err = r.move_q(q, sec)
        if err > 0.02: print("  retry", flush=True); code, err = r.move_q(q, sec)
        pos, quat, t = r.fk_hand(); print(f"  {name}: tcp {np.round(t,4)}", flush=True)
        if act == "close":
            f = r.gripper(GRIP["closed_m"]); gap = abs(f[0])+abs(f[1]); print(f"  gap {gap*1000:.1f} mm", flush=True)
            if gap < 0.004: print("GRASP FAILED"); r.gripper(GRIP["open_m"]); r.move_q(plan[0][1], 3.0); return
        elif act == "open":
            print("  fingers", r.gripper(GRIP["open_m"]), flush=True)
    print("done", flush=True)
main()
