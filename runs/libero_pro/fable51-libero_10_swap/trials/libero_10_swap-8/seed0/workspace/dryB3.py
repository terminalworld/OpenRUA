import math, sys, numpy as np, recover
from recover import Recover
from task import TCP_OFF
recover.GRASP_UP = 0.01
recover.Z_BOTTOM_TO_WAIST = 0.057
waist = [0.237, 0.012, 0.968]; a = [0.065, 0.998]
gam, sgn, pitch = float(sys.argv[1]), int(sys.argv[2]), float(sys.argv[3])
spot = tuple(map(float, sys.argv[4:6])); phi = float(sys.argv[6]); stage = tuple(map(float, sys.argv[7:9]))
live = "--live" in sys.argv
t = Recover(not live); t.stage = stage; t.handle_world = np.array([0.998, -0.065, 0.0])
t.grasp_R(waist, a, math.radians(gam), sgn, math.radians(pitch), close=False)
if live:
    R = t.hand_R(); tip = t.r.fk()[0] + R[:, 2] * TCP_OFF
    want = np.asarray(waist) - recover.GRASP_UP * R[:, 2]
    short = (want - tip) @ R[:, 2]
    print(f"   descend shortfall along z: {short*1000:.1f} mm", flush=True)
    if short > 0.005:
        print("!! descent blocked, backing off"); t.go_R([tip - 0.10 * R[:, 2]], [R], 3.0, "abort: retreat"); sys.exit(1)
    f = t.gripper(0.0); w = f[0] - f[1]
    print("   grasp width %.2f cm" % (w * 100), flush=True)
    if not (0.052 < w < 0.070):
        print("!! unexpected grasp width, releasing"); t.open_full()
        t.go_R([tip - 0.10 * R[:, 2]], [R], 3.0, "abort: retreat"); sys.exit(1)
else:
    t.gripper(0.0)
t.lift(1.10)
t.place_auto([a[0], a[1], 0.0], spot, math.radians(phi))
print("DRY OK" if not live else "LIVE DONE")
if live: print("fingers:", t.r.fingers())
