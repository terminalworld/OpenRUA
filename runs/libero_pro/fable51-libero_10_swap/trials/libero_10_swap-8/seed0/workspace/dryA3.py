"""Pot A (lying on the table, axis along +x, lid at +x): fork/pinch the boiler flanks, lift, place upright.
usage: dryA3.py gam sgn pitch sx sy phi stagex stagey [--live] [--wx=0.215] [--lift=1.16]"""
import math, sys, numpy as np, recover
from recover import Recover
from task import TCP_OFF
recover.GRASP_UP = 0.01
recover.Z_BOTTOM_TO_WAIST = 0.045          # bottom face x~0.170 -> grasp x 0.215
opt = lambda k, d: float(next((x[len(k) + 3:] for x in sys.argv if x.startswith("--" + k + "=")), d))
wx = opt("wx", 0.215); zlift = opt("lift", 1.16)
waist = [wx, 0.173, 0.939]; a = [1.0, 0.0]
gam, sgn, pitch = float(sys.argv[1]), int(sys.argv[2]), float(sys.argv[3])
spot = tuple(map(float, sys.argv[4:6])); phi = float(sys.argv[6]); stage = tuple(map(float, sys.argv[7:9]))
live = "--live" in sys.argv
t = Recover(not live); t.stage = stage; t.handle_world = np.array([0.0, -1.0, 0.0])
t.grasp_R(waist, a, math.radians(gam), sgn, math.radians(pitch), close=False)
if live:
    R = t.hand_R(); tip = t.r.fk()[0] + R[:, 2] * TCP_OFF
    want = np.asarray(waist) - recover.GRASP_UP * R[:, 2]
    d = want - tip
    short = d @ R[:, 2]; lat = np.linalg.norm(d - short * R[:, 2])
    print(f"   descend shortfall along z: {short*1000:.1f} mm, lateral {lat*1000:.1f} mm", flush=True)
    if short > 0.005 or lat > 0.006:
        print("!! descent deflected, backing off"); t.go_R([tip - 0.10 * R[:, 2]], [R], 3.0, "abort: retreat"); sys.exit(1)
    f = t.gripper(0.0); w = f[0] - f[1]
    print("   grasp width %.2f cm" % (w * 100), flush=True)
    if not (0.060 < w < 0.078):
        print("!! unexpected grasp width, releasing"); t.open_full()
        t.go_R([tip - 0.10 * R[:, 2]], [R], 3.0, "abort: retreat"); sys.exit(1)
else:
    t.gripper(0.0)
t.lift(zlift)
if live:
    f = t.r.fingers(); print("   width after lift %.2f cm" % ((f[0] - f[1]) * 100), flush=True)
    if f[0] - f[1] < 0.055:
        print("!! pot lost during lift"); t.open_full(); sys.exit(1)
t.place_auto([a[0], a[1], 0.0], spot, math.radians(phi))
print("DRY OK" if not live else "LIVE DONE")
if live: print("fingers:", t.r.fingers())
