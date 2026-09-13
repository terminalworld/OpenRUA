"""Carry upright pot B from the table to the stove with a pitched waist grasp.
Usage: python3 -u carry.py dry|run [yaw_final_deg]"""
import sys
from rob import *
LIM = np.array([[-2.9,2.9],[-1.76,1.76],[-2.9,2.9],[-3.07,-0.07],[-2.9,2.9],[-0.02,3.75],[-2.9,2.9]])
def margin(q): q=np.array(q); return float(np.min(np.minimum(q-LIM[:,0], LIM[:,1]-q)))
B = np.array([-0.135, -0.076]); HANDLE0 = math.radians(-145.0)   # pot B axis (table) and handle direction
PHI0 = math.radians(-95.0)                                       # pitch direction at pick (palm on opposite side)
TARGET = np.array([0.13, 0.10])                                   # pot B centre on the stove
PHI1 = math.radians(float(sys.argv[2])) if len(sys.argv) > 2 else math.radians(-45.0)   # pitch direction at the stove
T = 1.3                                                           # hand pitch (rad) about the tangential closing axis
WAIST_T = 0.899 + 0.0675; WAIST_S = 0.93 + 0.0675 + 0.003
LIFT = 1.20
A_BAR = (np.array([0.150, -0.0045, 0.996]), np.array([0.150, -0.0045, 1.056]))   # pot A handle bar segment (approx)
def R_pitched(phi, sgn=1):
    u = np.array([math.cos(phi), math.sin(phi), 0.0]); tan = np.array([-u[1], u[0], 0.0])*sgn
    hz = math.sin(T)*u - math.cos(T)*np.array([0,0,1.0]); hy = tan; hx = np.cross(hy, hz)
    return np.column_stack([hx, hy, hz])
def body_clearance(tcp, R):
    """min distance from hand-body box (palm..wrist) to pot A bar segment"""
    hx, hy, hz = R[:,0], R[:,1], R[:,2]; best = 9
    for d in np.linspace(0.058, 0.103, 4):
        for a in (-0.035, 0.035):
            for b in np.linspace(-0.10, 0.10, 9):
                p = np.array(tcp) - d*hz + a*hx + b*hy
                s0, s1 = A_BAR; v = s1-s0; t = np.clip(np.dot(p-s0, v)/np.dot(v,v), 0, 1)
                best = min(best, np.linalg.norm(p-(s0+t*v)))
    return best
def main():
    r = Robot(); mode = sys.argv[1]
    q0 = r.arm_q(); print("fingers", r.fingers(), "q", np.round(q0,3), flush=True)
    plan = None
    for sgn in (1, -1):
        R0 = R_pitched(PHI0, sgn); R1 = R_pitched(PHI1, sgn)
        g = np.array([B[0], B[1], WAIST_T]); p0 = np.array([math.cos(PHI0), math.sin(PHI0), 0.0])
        pre = g - 0.08*p0 + np.array([0,0,0.01])
        steps = [("high", (pre[0], pre[1], 1.15), PHI0, 4.0, None), ("pre", pre, PHI0, 3.0, None),
                 ("slide", g + np.array([0,0,0.01]), PHI0, 3.0, None), ("grasp", g, PHI0, 2.0, "close"),
                 ("lift", (g[0], g[1], LIFT), PHI0, 3.0, None)]
        n = 5
        for k in range(1, n+1):
            steps.append((f"yaw{k}", (g[0], g[1], LIFT), PHI0 + (PHI1-PHI0)*k/n, 2.5, None))
        s = np.array([TARGET[0], TARGET[1], WAIST_S])
        for k, (wx_, wy_) in enumerate([(-0.05, -0.02), (0.05, 0.04)]):
            steps.append((f"way{k+1}", (wx_, wy_, 1.22), PHI1, 3.0, None))
        steps += [("over", (s[0], s[1], LIFT), PHI1, 3.0, None), ("down", (s[0], s[1], WAIST_S+0.03), PHI1, 3.0, None),
                  ("set", s, PHI1, 2.5, "open")]
        p1 = np.array([math.cos(PHI1), math.sin(PHI1), 0.0]); back = s - 0.08*p1 + np.array([0,0,0.02])
        print(f"  handle at stove -> {math.degrees(HANDLE0 + PHI1 - PHI0):.0f} deg; bar at {np.round(TARGET + 0.075*np.array([math.cos(HANDLE0+PHI1-PHI0), math.sin(HANDLE0+PHI1-PHI0)]),3)}")
        steps += [("back", back, PHI1, 2.5, None), ("up", (back[0], back[1], 1.25), PHI1, 3.0, None)]
        prev = q0; out = []; ok = True
        for name, tcp, phi, sec, act in steps:
            R = R_pitched(phi, sgn)
            q = r.ik_tcp(tcp, R, seed=prev)
            if q is None: print(f"  sgn {sgn}: IK fail {name}"); ok = False; break
            jump = float(np.max(np.abs(np.array(q)-np.array(prev))))
            if (jump > 1.6 and name != "pre") or margin(q) < 0.12:
                print(f"  sgn {sgn}: bad {name} jump {jump:.2f} margin {margin(q):.2f}"); ok = False; break
            out.append((name, np.array(tcp), R, q, sec, act, jump)); prev = q
        if ok:
            print(f"PLAN sgn {sgn}; final hz {np.round(R1[:,2],2)} hy {np.round(R1[:,1],2)}; hand-body clearance to A bar at set: {body_clearance(s, R1)*100:.1f} cm, at down: {body_clearance((s[0],s[1],WAIST_S+0.03), R1)*100:.1f} cm")
            for name, tcp, R, q, sec, act, jump in out:
                print(f"  {name:6s} {np.round(tcp,3)} -> {np.round(q,2)} m {margin(q):.2f} jump {jump:.2f}")
            plan = out; break
    if plan is None: raise SystemExit("no plan")
    if mode != "run": return
    r.gripper(GRIP["open_m"])
    for name, tcp, R, q, sec, act, jump in plan:
        sec = max(sec, 3.0*jump)
        for attempt in range(3):
            code, err = r.move_q(q, sec)
            if err < 0.02: break
            print("  retry", flush=True)
        pos, quat, tt = r.fk_hand(); print(f"  {name}: tcp {np.round(tt,4)} err {err:.3f}", flush=True)
        if err > 0.05: print("STOP: not converged"); return
        if act == "close":
            f = r.gripper(GRIP["closed_m"]); gap = abs(f[0])+abs(f[1]); print(f"  gap {gap*1000:.1f} mm", flush=True)
            if gap < 0.04 or gap > 0.078: print("GRASP BAD"); r.gripper(GRIP["open_m"]); r.move_q(plan[0][3], 3.0); return
        elif act == "open":
            print("  fingers", r.gripper(GRIP["open_m"]), flush=True)
    print("done", flush=True)
main()
