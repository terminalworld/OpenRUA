"""Lay pot B (held rigidly, horizontal, base toward -x) down along y on free table: base -> -y, knob -> +y.
Usage: python3 -u flip2.py dry|run"""
import sys
from rob import *
from flip import R_of
SPOT = (-0.07, 0.25)
Y4 = [math.pi/2 * k / 4 for k in range(1, 5)]
STEPS = [((0.05, 0.10, 1.30), 0.0, 0.6), (SPOT + (1.30,), 0.0, 0.6)]
STEPS += [(SPOT + (1.30,), y, 0.6) for y in Y4]
STEPS += [(SPOT + (1.10,), math.pi/2, 0.6), (SPOT + (1.03,), math.pi/2, 0.6), (SPOT + (1.022,), math.pi/2, 0.6)]
r = Robot(); mode = sys.argv[1]
print("fingers", r.fingers(), flush=True)
seed = r.arm_q()
for tcp, yaw, tilt in STEPS:
    q = r.ik_tcp(tcp, R_of(yaw, tilt), seed=seed)
    if q is None: raise SystemExit(f"IK failed {tcp} yaw {yaw:.2f} tilt {tilt:.2f}")
    print(f"-> {np.round(tcp,3)} yaw {yaw:.2f} tilt {tilt:.2f} q {np.round(q,2)} jump {np.max(np.abs(np.array(q)-np.array(seed))):.2f}", flush=True)
    if mode == "run":
        code, err = r.move_q(q, 3.0)
        if err > 0.02 and tcp[2] > 1.05:
            print("  retry", flush=True); code, err = r.move_q(q, 3.0)
        pos, quat, t = r.fk_hand(); print(f"  fk tcp {np.round(t,4)} fingers {np.round(r.fingers(),4)}", flush=True)
        seed = r.arm_q()
    else:
        seed = q
if mode == "run":
    r.gripper(GRIP["open_m"])
    pos, quat, t = r.fk_hand()
    q = r.ik_tcp((t[0], t[1], 1.30), R_of(math.pi/2, 0.6), seed=r.arm_q()); r.move_q(q, 3.0)
    print("fk", np.round(r.fk_hand()[2],4))
# knob grasp reachability preview
for t in (1.0, 0.8):
    q = r.ik_tcp((SPOT[0], SPOT[1] + 0.043, 0.934), R_of(-math.pi/2, t), seed=r.arm_q())
    print(f"knob grasp preview tilt {t}: {'NO' if q is None else np.round(q,2)}")
print("done", flush=True)
