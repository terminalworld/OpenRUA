#!/usr/bin/env python3
"""pick_place.py <name> <x> <y>   -- pick the can near world (x,y) and drop it in the basket."""
import sys
import cv2
from arm import *

name, gx, gy = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
DROP_XY = (float(sys.argv[4]), float(sys.argv[5])) if len(sys.argv) > 5 else None  # explicit drop point
TABLE = 0.425
Z_TRAVEL = 0.83      # hand z for carrying: TCP 0.727, can bottom ~0.67 > rim 0.575
Z_LOOK = 0.78        # hand z for the eye-in-hand refinement look
Z_DROP = 0.70        # fingertips (hand z - 0.1034) stay above rim z 0.579 even near the rim
RIM_MIN_H = 0.12     # basket rim height above table (measured 0.15)
Q = DOWN_X           # fingers along world x: hand is narrow in y (basket side) -> no rim collision


def basket_center(a):
    """Interior-floor centroid of the basket from the birdview (basket = tallest thing in y>0.25)."""
    W, C, D = a.cloud_world("birdview")
    ok = np.isfinite(D) & (D > 0.05)
    h = W[..., 2] - TABLE
    r = ok & (h > RIM_MIN_H) & (W[..., 1] > 0.25)
    rc = W[r][:, :2].mean(0)
    near = np.hypot(W[..., 0] - rc[0], W[..., 1] - rc[1]) < 0.10
    m = ok & (h > 0.01) & (h < 0.05) & near
    fc = W[m][:, :2].mean(0)
    print(f"   basket rim centroid {rc.round(4)} floor centroid {fc.round(4)} (n={m.sum()})", flush=True)
    return fc, W, C, D


def height_grid(W, D):
    xs = np.arange(-0.16, 0.13, 0.01); ys = np.arange(0.16, 0.50, 0.01)
    print('      ' + ' '.join(f'{x*100:4.0f}' for x in xs))
    for yy in ys:
        row = []
        for xx in xs:
            m = (np.abs(W[..., 0] - xx) < 0.005) & (np.abs(W[..., 1] - yy) < 0.005) & np.isfinite(D)
            row.append(f'{(W[..., 2][m].max()-TABLE)*100:4.0f}' if m.any() else '   .')
        print(f'y={yy*100:3.0f} ' + ' '.join(row), flush=True)


def can_check(loc):
    """Sanity: a can top sits 0.49-0.54, and is 6-8 cm across; the basket rim is at 0.575."""
    x, y, ztop = loc
    return 0.49 < ztop < 0.545


a = Arm()
print(f"== {name}: start q={np.round(a.arm_q(),3)} fingers={a.fingers()}", flush=True)
BASKET, _, _, _ = basket_center(a)
if DROP_XY is not None:
    BASKET = np.array(DROP_XY); print(f"   using explicit drop point {BASKET}", flush=True)

print("-- open gripper", flush=True)
a.gripper(GRIP["open_m"])

print("-- move above guess", flush=True)
assert a.move_to((gx, gy, Z_LOOK), Q, seconds=3.0)

print("-- refine with eye-in-hand", flush=True)
loc = a.locate("robot0_eye_in_hand", (gx, gy), radius=0.045, zmax=0.55)
if loc is None or not can_check(loc):
    print("!! refinement did not see a can-like blob; falling back to birdview", flush=True)
    loc = a.locate("birdview", (gx, gy), radius=0.045, zmax=0.55)
assert loc is not None and can_check(loc), f"no can near guess: {loc}"
x, y, ztop = loc
d = np.hypot(x - gx, y - gy)
print(f"   correction {d*1000:.1f} mm", flush=True)
if d > 0.004:
    assert a.move_to((x, y, Z_LOOK), Q, seconds=2.0)
    loc2 = a.locate("robot0_eye_in_hand", (x, y), radius=0.045, zmax=0.55)
    if loc2 is not None and can_check(loc2):
        x, y, ztop = loc2

z_grasp = ztop - 0.040 + TCP          # TCP 4 cm below the can top (cans are 9.1 cm tall)
z_grasp = max(z_grasp, TABLE + 0.015 + TCP)  # never below 1.5 cm above the table
print(f"-- descend to grasp: xy=({x:.4f},{y:.4f}) hand z={z_grasp:.4f}", flush=True)
assert a.move_to((x, y, z_grasp + 0.08), Q, seconds=2.0)
assert a.move_to((x, y, z_grasp), Q, seconds=2.0)

print("-- close gripper", flush=True)
f = a.gripper(GRIP["closed_m"])
gap = abs(f[0]) + abs(f[1])
print(f"   finger gap after close = {gap*1000:.1f} mm", flush=True)
if not 0.045 < gap < 0.08:
    print("!! grasp FAILED (gap not can-sized)", flush=True)
    a.gripper(GRIP["open_m"])
    a.move_to((x, y, Z_TRAVEL), Q, seconds=2.0)
    sys.exit(2)

print("-- lift", flush=True)
assert a.move_to((x, y, Z_TRAVEL), Q, seconds=2.5)
f = a.fingers(); print(f"   fingers after lift {f}", flush=True)

print("-- carry to basket", flush=True)
assert a.move_to((BASKET[0], BASKET[1], Z_TRAVEL), Q, seconds=3.5)
assert a.move_to((BASKET[0], BASKET[1], Z_DROP), Q, seconds=2.0)

print("-- release", flush=True)
a.gripper(GRIP["open_m"])

print("-- retreat", flush=True)
assert a.move_to((BASKET[0], BASKET[1], Z_TRAVEL), Q, seconds=2.0)
assert a.move_to((-0.15, 0.05, Z_TRAVEL), Q, seconds=2.5)

print("-- verify: birdview", flush=True)
BASKET2, W, C, D = basket_center(a)
height_grid(W, D)
cv2.imwrite(f"after_{name}.png", C)
print(f"== {name}: DONE (basket moved {np.hypot(*(BASKET2-BASKET))*1000:.0f} mm during place)", flush=True)
