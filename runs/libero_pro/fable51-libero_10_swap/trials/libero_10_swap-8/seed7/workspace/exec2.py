"""Handle-bar pick/place with the bar along x (fingers close along y), approach pitched 30 deg.
Usage: python3 exec2.py pickx <bar_x> <bar_y> <bar_z>
       python3 exec2.py placex <x> <y> <z_release>
"""
import sys
from rob import *
TILT = 0.52
LIFT_Z = 1.20

def goto(r, tcp, seconds, seed=None):
    q = r.ik_tcp(tcp, xbar_R(TILT), seed=seed)
    if q is None: raise SystemExit(f"IK failed for {tcp}")
    print(f"-> tcp {np.round(tcp,3)} q {np.round(q,3)}", flush=True)
    code, err = r.move_q(q, seconds)
    if err > 0.02:
        print("  retry move", flush=True); code, err = r.move_q(q, seconds)
    pos, quat, t = r.fk_hand()
    print(f"  fk tcp now {np.round(t,4)}  (err {np.linalg.norm(t-np.array(tcp))*1000:.1f} mm)", flush=True)
    return q

r = Robot()
mode = sys.argv[1]; x, y, z = map(float, sys.argv[2:5])
print("fingers", r.fingers(), "q", np.round(r.arm_q(),3), flush=True)
if mode == "pickx":
    r.gripper(GRIP["open_m"])
    goto(r, (x, y, z + 0.10), 4.0)
    goto(r, (x, y, z), 2.5)
    f = r.gripper(GRIP["closed_m"]); gap = abs(f[0]) + abs(f[1])
    print(f"  finger gap after close: {gap*1000:.1f} mm", flush=True)
    if gap < 0.004:
        print("GRASP FAILED (closed on air)", flush=True)
        r.gripper(GRIP["open_m"]); goto(r, (x, y, z + 0.10), 2.5); sys.exit(1)
    goto(r, (x, y, LIFT_Z), 3.0)
    print("fingers after lift", r.fingers(), flush=True)
elif mode == "placex":
    goto(r, (x, y, LIFT_Z), 5.0)
    print("fingers at carry", r.fingers(), flush=True)
    goto(r, (x, y, z + 0.04), 2.5)
    goto(r, (x, y, z), 2.0)
    r.gripper(GRIP["open_m"])
    goto(r, (x, y, LIFT_Z), 3.0)
print("done", flush=True)
