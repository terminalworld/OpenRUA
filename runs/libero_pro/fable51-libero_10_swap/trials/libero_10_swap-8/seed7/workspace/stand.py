"""Stand pot B (lying on table, handle horizontal toward plate) upright on the table.
Waist grasp from above -> lift -> yaw pot 180deg -> pitch 90deg about closing axis (base down)
-> set down at SPOT -> release -> retreat.   Usage: python3 -u stand.py dry|run"""
import sys
from rob import *
LIM = np.array([[-2.9,2.9],[-1.76,1.76],[-2.9,2.9],[-3.07,-0.07],[-2.9,2.9],[-0.02,3.75],[-2.9,2.9]])
def margin(q): q=np.array(q); return float(np.min(np.minimum(q-LIM[:,0], LIM[:,1]-q)))
C = np.array([0.088, 0.2038]); AX = np.array([0.589, 0.808])      # pot axis (+AX -> base end)
G = C + 0.0*AX                                                      # finger centre on the waist
GZ = 0.941; LIFT = 1.20; TILT_Z = 1.15
SPOT = tuple(map(float, sys.argv[2:4])) if len(sys.argv) >= 4 else (-0.05, 0.25)
DYAWS = [math.radians(d) for d in ([float(sys.argv[4])] if len(sys.argv) >= 5 else (-100, -90, -110, -80))]                                                # where the pot will stand
SET_Z = 0.899 + 0.073 + 0.005                                       # TCP (7.3 cm above base) + 5 mm drop
def hand_R(yaw, t):
    c, s = math.cos(t), math.sin(t)
    return grasp_R(yaw, 0) @ np.array([[c,0,s],[0,1,0],[-s,0,c]])   # rotate about the hand's own y (closing axis)
def yaw_perp(sign):   # hand yaw so that closing axis hy = Rz(yaw)*x is perpendicular to AX
    hy = sign*np.array([-AX[1], AX[0]]); return math.atan2(hy[1], hy[0])
def main():
    r = Robot(); mode = sys.argv[1]
    q0 = r.arm_q(); print("fingers", r.fingers(), "q", np.round(q0,3), flush=True)
    # pick the grasp yaw whose IK leaves room for a +/-180deg yaw of j7 later
    cands = []
    for sgn in (1, -1):
        psi = yaw_perp(sgn)
        q = r.ik_tcp((G[0], G[1], LIFT), grasp_R(psi, 0), seed=q0)
        if q is not None: cands.append((psi, q)); print(f"cand yaw {psi:+.3f}: q {np.round(q,2)} margin {margin(q):.2f} j7 {q[6]:+.2f}")
    if not cands: raise SystemExit("no IK above pot")
    # for each candidate, decide the direction of the 180 yaw (toward j7 room) and test the pitch
    plan = None
    for psi, qa in cands:
        for dyaw in DYAWS:
            for tilt in (math.pi/2, -math.pi/2):
                R = hand_R(psi + dyaw, tilt)
                # pot base direction in world after yaw: Rz(dyaw)*AX ; it must map to -z under the tilt.
                # base dir in hand frame at grasp = grasp_R(psi,0)^T [AX,0]; after the motion, world = R @ that
                b_hand = grasp_R(psi, 0).T @ np.array([AX[0], AX[1], 0.0])
                b_world = R @ b_hand
                if b_world[2] > -0.95: continue
                steps = [("pre", (G[0], G[1], LIFT), psi, 0.0, 4.0, None),
                         ("grasp", (G[0], G[1], GZ), psi, 0.0, 3.0, "close"),
                         ("lift", (G[0], G[1], LIFT), psi, 0.0, 3.0, None)]
                for k in (1, 2, 3):
                    steps.append((f"yaw{k}", (G[0], G[1], LIFT), psi + dyaw*k/3, 0.0, 2.5, None))
                steps.append(("move", (SPOT[0], SPOT[1], TILT_Z), psi + dyaw, 0.0, 4.0, None))
                for k in (1, 2, 3):
                    steps.append((f"tilt{k}", (SPOT[0], SPOT[1], TILT_Z), psi + dyaw, tilt*k/3, 2.5, None))
                steps += [("move2", (SPOT[0], SPOT[1], 1.10), psi + dyaw, tilt, 3.0, None),
                          ("down", (SPOT[0], SPOT[1], SET_Z + 0.03), psi + dyaw, tilt, 3.0, None),
                          ("set", (SPOT[0], SPOT[1], SET_Z), psi + dyaw, tilt, 2.0, "open")]
                # retreat along -hz then up
                back = -R[:, 2]*0.09
                steps += [("back", (SPOT[0]+back[0], SPOT[1]+back[1], SET_Z), psi + dyaw, tilt, 2.5, None),
                          ("up", (SPOT[0]+back[0], SPOT[1]+back[1], 1.20), psi + dyaw, tilt, 3.0, None)]
                prev = q0; ok = True; out = []
                for name, tcp, yaw, t, sec, act in steps:
                    q = r.ik_tcp(tcp, hand_R(yaw, t), seed=prev)
                    if q is None: print(f"  [psi {psi:+.2f} dyaw {dyaw:+.2f} tilt {tilt:+.2f}] IK fail at {name}"); ok = False; break
                    jump = float(np.max(np.abs(np.array(q)-np.array(prev))))
                    if (jump > 1.6 and name != "pre") or margin(q) < 0.15:
                        print(f"  [psi {psi:+.2f} dyaw {dyaw:+.2f} tilt {tilt:+.2f}] bad {name}: jump {jump:.2f} margin {margin(q):.2f}"); ok = False; break
                    out.append((name, tcp, yaw, t, q, sec, act, jump)); prev = q
                if ok:
                    print(f"PLAN psi {psi:+.3f} dyaw {dyaw:+.2f} tilt {tilt:+.2f}: hand final hz {np.round(R[:,2],2)} hy {np.round(R[:,1],2)}")
                    for name, tcp, yaw, t, q, sec, act, jump in out:
                        print(f"  {name:6s} {np.round(tcp,3)} yaw {yaw:+.2f} tilt {t:+.2f} -> {np.round(q,2)} m {margin(q):.2f} jump {jump:.2f}")
                    if plan is None: plan = out
                    break
            if plan is not None: break
        if plan is not None: break
    if plan is None: raise SystemExit("no feasible plan")
    if mode != "run": return
    r.gripper(GRIP["open_m"])
    for name, tcp, yaw, t, q, sec, act, jump in plan:
        sec = max(sec, 3.0*jump)
        for attempt in range(3):
            code, err = r.move_q(q, sec)
            if err < 0.02: break
            print("  retry", flush=True)
        pos, quat, tt = r.fk_hand(); print(f"  {name}: tcp {np.round(tt,4)} err {err:.3f}", flush=True)
        if err > 0.05: print("STOP: not converged"); return
        if act == "close":
            f = r.gripper(GRIP["closed_m"]); gap = abs(f[0])+abs(f[1]); print(f"  gap {gap*1000:.1f} mm", flush=True)
            if gap < 0.04 or gap > 0.078: print("GRASP BAD"); r.gripper(GRIP["open_m"]); r.move_q(plan[0][4], 3.0); return
        elif act == "open":
            print("  fingers", r.gripper(GRIP["open_m"]), flush=True)
    print("done", flush=True)
main()
