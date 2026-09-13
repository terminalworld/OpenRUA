"""Rim-side pinch of the lying mug, swing it upright (hanging), carry into the microwave, push in."""
import sys, numpy as np
from ctl import Robot, quat_from_axes, log
from geom import a_of, n_of, hand_points, collides

def Q(th): return quat_from_axes(a_of(th), [1.0, 0, 0])
def Qphi(phi):  # approach rotated about x: phi=0 -> a=-y (from +y side), 90 -> down, 135 -> a_of(45)
    t = np.radians(phi); return quat_from_axes([0, -np.cos(t), -np.sin(t)], [1.0, 0, 0])

XC_LYING, YR, ZA = -0.059, 0.053, 0.953       # lying mug: axis x, rim edge y, axis height at rim
TIPX = XC_LYING - 0.0475                      # pinch at the -x rim wall
G = np.array([TIPX, YR - 0.025, ZA])          # grasp tip
XC = -0.09                                    # desired mug centre x in the microwave
TX = XC - 0.0475                              # tip x while hanging

def hang_mug_points(tip):
    c = np.array([tip[0] + 0.0475, tip[1], tip[2] - 0.0275])
    pts = []
    for h, rad in ((-0.0525, 0.0375), (0.0525, 0.05), (0.0, 0.043)):
        for ang in np.linspace(0, 2*np.pi, 24, endpoint=False):
            pts.append(c + [rad*np.cos(ang), rad*np.sin(ang), h])
    for dy in (-0.06, -0.075, -0.09):          # handle toward -y
        for dz in (-0.03, 0.0, 0.03):
            pts.append(c + [0, dy, dz])
    return np.array(pts)

r = Robot("rg")
DRY = "dry" in sys.argv

def plan(steps, seed, quatf=Q, with_mug=False, margin=0.004):
    out = []
    for p, th in steps:
        p = np.array(p, float)
        if quatf is Q:
            if collides(hand_points(p, th), margin): log("  HAND COLLISION", p, th)
        if with_mug and collides(hang_mug_points(p), margin): log("  MUG COLLISION", p, th)
        s = r.ik(p, quatf(th), seed=seed, at_tcp=True)
        if s is None: sys.exit(f"IK fail {p} {th}")
        jump = float(np.abs(np.array(s) - np.array(seed)).max())
        log(f"  {np.round(p,3)} th={th} jump={jump:.2f}")
        out.append(s); seed = s
    return out, seed

seed = r.joints()
# 1 approach from +y at axis height, horizontal hand (phi=0)
pre, seed = plan([((TIPX, 0.12, 1.15), 0), ((TIPX, 0.12, ZA), 0), ((TIPX, 0.07, ZA), 0), (G, 0)], seed, quatf=Qphi)
# 2 lift, then swing the hand about x through phi 45,90,135 with tip fixed
LZ = 1.10
lift, seed = plan([((TIPX, G[1], LZ), 0)], seed, quatf=Qphi)
swing, seed = plan([((TIPX, G[1], LZ), 45), ((TIPX, G[1], LZ), 90), ((TIPX, G[1], LZ), 135)], seed, quatf=Qphi)
# 3 carry (hanging mug, upright)
carry, seed = plan([((TX, G[1], 1.25), 45), ((TX, -0.15, 1.25), 55), ((TX, -0.35, 1.25), 65), ((TX, -0.45, 1.20), 65),
                    ((TX, -0.45, 1.06), 55)], seed, with_mug=True)
ins, seed = plan([((TX, -0.42, 1.045), 45), ((TX, -0.38, 1.04), 35), ((TX, -0.326, 1.035), 30), ((TX, -0.326, 1.022), 30)], seed, with_mug=True)
ret, seed = plan([((TX, -0.326, 1.062), 30), ((TX, -0.38, 1.065), 40), ((TX, -0.43, 1.07), 55)], seed)
push, seed = plan([((XC, -0.43, 1.0), 75), ((XC, -0.40, 1.0), 55), ((XC, -0.37, 1.0), 40), ((XC, -0.345, 1.0), 30), ((XC, -0.322, 1.0), 25)], seed)
out, seed = plan([((XC, -0.37, 1.0), 40), ((XC, -0.40, 1.03), 50), ((XC, -0.44, 1.12), 60)], seed)
if DRY: sys.exit(0)

def go(wps, times, hold=4.0):
    code = r.move_joints(wps, times, hold=hold)
    log("   hand", np.round(r.hand_pose()[0], 3), "fingers", np.round(r.fingers(), 4))
    return code

if "p2" not in sys.argv:
  r.gripper(0.04)
  go(pre, [5.0, 9.0, 12.0, 15.0])
  f = r.gripper(0.0)
  if not (0.0008 < f[0] < 0.012): sys.exit(f"bad rim grasp {f}")
  go(lift, [4.0])
  go(swing, [4.0, 8.0, 12.0])
  log("swing done; fingers", r.fingers())
if "p1" in sys.argv: sys.exit(0)
go(carry, [4.0, 9.0, 14.0, 17.0, 21.0])
go(ins, [3.0, 6.0, 9.0, 11.0], hold=3.0)
log("fingers before release", r.fingers())
r.gripper(0.04)
go(ret, [3.0, 6.0, 9.0])
r.gripper(0.0)
go(push, [4.0, 7.0, 10.0, 13.0, 16.0])
go(out, [3.0, 6.0, 9.0])
