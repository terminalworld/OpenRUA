"""Carry pot B (held rigidly, horizontal, base toward -x) to a free table spot and lay it down
with the lid toward -x. Usage: python3 -u flip.py dry|run"""
import sys
from rob import *
def R_of(yaw, tilt):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c,-s,0],[s,c,0],[0,0,1]]) @ xbar_R(tilt)
SPOT = (0.05, 0.22)
SGN = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0   # yaw direction
Y8 = [SGN*math.pi*k/8 for k in range(1, 5)]
STEPS = [((0.05, 0.10, 1.25), 0.0, 0.6),        # lift clear of knob object
         (SPOT + (1.25,), 0.0, 0.6)]            # carry to free spot
STEPS += [(SPOT + (1.25,), y, 0.6) for y in Y8]                                 # yaw 90 (pot base -> -/+y)
STEPS += [(SPOT + (1.25,), Y8[-1], 0.3), (SPOT + (1.25,), Y8[-1], 0.0)]         # un-pitch (base end rises)
STEPS += [(SPOT + (1.25,), SGN*math.pi*k/8, 0.0) for k in range(5, 9)]          # yaw to 180 (base -> +x)
STEPS += [(SPOT + (1.05,), SGN*math.pi, 0.0), (SPOT + (0.97,), SGN*math.pi, 0.0)]  # lower until lid rim touches
if __name__ == '__main__':
    r = Robot(); mode = sys.argv[1]
    print("fingers", r.fingers(), flush=True)
    seed = r.arm_q()
    for tcp, yaw, tilt in STEPS:
        q = r.ik_tcp(tcp, R_of(yaw, tilt), seed=seed)
        if q is None: raise SystemExit(f"IK failed {tcp} yaw {yaw:.2f} tilt {tilt:.2f}")
        print(f"-> {np.round(tcp,3)} yaw {yaw:.2f} tilt {tilt:.2f} q {np.round(q,2)} jump {np.max(np.abs(np.array(q)-np.array(seed))):.2f}", flush=True)
        if mode == "run":
            code, err = r.move_q(q, 3.0)
            if err > 0.02 and tcp[2] > 1.0:
                print("  retry", flush=True); code, err = r.move_q(q, 3.0)
            pos, quat, t = r.fk_hand(); print(f"  fk tcp {np.round(t,4)} fingers {np.round(r.fingers(),4)}", flush=True)
            seed = r.arm_q()
        else:
            seed = q
    if mode == "run":
        r.gripper(GRIP["open_m"])
        pos, quat, t = r.fk_hand()
        q = r.ik_tcp((t[0], t[1], 1.25), R_of(SGN*math.pi, 0.0), seed=r.arm_q()); r.move_q(q, 3.0)
    print("done", flush=True)
