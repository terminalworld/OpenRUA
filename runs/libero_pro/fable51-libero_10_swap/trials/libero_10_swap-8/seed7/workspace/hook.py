"""Phase B: hook-lift pot B (lying along x, base face x=-0.10, axis y=0.207, handle up) upright.
Hand points -y (from +y side), fingers stacked vertically (open): lower finger slides under arch, rides arc about base rim P.
Usage: python3 -u hook.py dry|run"""
import sys
from rob import *
LIM = np.array([[-2.9,2.9],[-1.76,1.76],[-2.9,2.9],[-3.07,-0.07],[-2.9,2.9],[-0.02,3.75],[-2.9,2.9]])
def margin(q): q=np.array(q); return float(np.min(np.minimum(q-LIM[:,0], LIM[:,1]-q)))
R_HOOK = np.array([[1.0,0,0],[0,0,1.0],[0,-1.0,0]])   # hz=-y, closing axis=z, hx=+x (wrist on +y side)
PX, PY, PZ = -0.10, 0.207, 0.899      # base bottom rim (pivot)
R_ARC = 0.1508                        # |arch underside point - P|
F_OFF = 0.04 - 0.004                  # TCP is 4cm above lower finger top; run finger top 4mm inside arc
PHI0, PHI1, N = math.radians(50), math.radians(143), 16
Y_PRE = 0.29
def tcp_at(phi):
    return (PX + R_ARC*math.cos(phi), PY, PZ + R_ARC*math.sin(phi) + F_OFF)
def main():
    r = Robot(); mode = sys.argv[1]
    print("fingers", r.fingers(), "q", np.round(r.arm_q(),3), flush=True)
    t0 = tcp_at(PHI0)
    steps = [("above", (t0[0], Y_PRE, 1.25), 4.0), ("pre", (t0[0], Y_PRE, t0[2]), 3.0), ("insert", t0, 3.0)]
    for k in range(1, N+1):
        steps.append((f"arc{k}", tcp_at(PHI0 + (PHI1-PHI0)*k/N), 1.5))
    t1 = tcp_at(PHI1)
    steps += [("retreat", (t1[0], Y_PRE, t1[2]), 3.0), ("up", (t1[0], Y_PRE, 1.25), 3.0)]
    prev = r.arm_q(); plan = []
    for name, tcp, sec in steps:
        q = r.ik_tcp(tcp, R_HOOK, seed=prev)
        if q is None: raise SystemExit(f"IK fail {name} {tcp}")
        jump = float(np.max(np.abs(np.array(q)-np.array(prev))))
        print(f"{name:8s} {np.round(tcp,4)} -> {np.round(q,2)} margin {margin(q):.2f} jump {jump:.2f}", flush=True)
        if jump > 1.2 and name != "above": raise SystemExit("branch jump")
        plan.append((name, q, sec)); prev = q
    if mode != "run": return
    r.gripper(GRIP["open_m"])
    for name, q, sec in plan:
        sec = max(sec, 3.0*float(np.max(np.abs(np.array(q)-np.array(r.arm_q())))))
        code, err = r.move_q(q, sec)
        for _ in range(2):
            if err > 0.02: print("  retry", flush=True); code, err = r.move_q(q, max(sec, 3.0))
        pos, quat, t = r.fk_hand(); print(f"  {name}: tcp {np.round(t,4)} err {err:.4f}", flush=True)
        if err > 0.05: print("  LARGE ERROR - stopping", flush=True); return
    print("done", flush=True)
main()
