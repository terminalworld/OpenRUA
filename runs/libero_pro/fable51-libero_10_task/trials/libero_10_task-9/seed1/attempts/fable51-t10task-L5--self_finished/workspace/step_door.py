"""Close the microwave door by pushing its outer face with closed fingers
along an arc about the hinge."""
import sys
import numpy as np
from rob import *
import pathcheck
from pathcheck import fk_links, RAD

HINGE = np.array([-0.19, 0.265])
TH0 = np.radians(-122.5)          # current door angle (tip at (-0.328,0.048))
RHO = 0.18                        # contact radius along the door
OFF = 0.0245                      # half door thickness + finger half width
Z = 1.085                         # fingertips on the top 2 cm of the door; palm clears door top
LEAN = np.radians(30)
U = np.array([-0.875, -0.485, 0.0])   # fixed lean direction (palm displaced this way)
FING = np.array([0.485, -0.875, 0.0]) # fixed finger axis, perpendicular to U
R_FIX = np.array(hand_R(-np.array([0, 0, 1.0]) * np.cos(LEAN) - U * np.sin(LEAN), FING))

def door_pose(th, sgn=1.0):
    d = np.array([np.cos(th), np.sin(th), 0.0])
    n_out = np.array([np.sin(th), -np.cos(th), 0.0])
    tcp = np.array([HINGE[0], HINGE[1], Z]) + RHO * d + OFF * n_out
    return tcp, R_FIX

def check_no_door(r, q0, q1, n=6):
    bad = False
    for i in range(n + 1):
        q = q0 + (q1 - q0) * i / n
        L = fk_links(r, q)
        for k, p in L.items():
            h = [x for x in pathcheck.hazards(p, RAD[k]) if x != "DOOR"]
            if h:
                print(f"  hazard {k} {h} at {np.round(p,3).tolist()}"); bad = True
    return not bad

r = Rob("door")
chk = lambda a, b: check_no_door(r, a, b)
if "--dry" in sys.argv:
    q, _ = r.joints()
    SG = float(sys.argv[sys.argv.index("--sgn")+1]) if "--sgn" in sys.argv else 1.0
    for th in np.radians(np.arange(-122.5, 1, 15)):
        tcp, R = door_pose(th, SG)
        qs, d = r.ik_near(tcp, R, seed=q, tries=6)
        print(f"th {np.degrees(th):6.1f} tcp {np.round(tcp,3).tolist()} ik {'ok' if qs is not None else 'FAIL'} dist {None if d is None else round(d,2)} j7 {None if qs is None else round(qs[6],2)}")
        if qs is not None: q = qs
    rclpy.shutdown(); sys.exit()

r.gripper(0.0)
tcp0, R0 = door_pose(TH0)
hi = tcp0 + [0, 0, 0.25]
assert r.move_tcp_cl(hi, R0, secs=8.0, check=chk, iters=1) is not None
assert r.move_tcp_cl(tcp0, R0, secs=4.0, check=chk, iters=1) is not None
angles = list(np.arange(np.degrees(TH0) + 10, 0, 10)) + [0.0, 2.0]
for a in angles:
    tcp, R = door_pose(np.radians(a))
    q = r.move_tcp_cl(tcp, R, secs=3.0, check=chk, iters=1)
    assert q is not None, f"failed at door angle {a}"
    print(f"--- door angle {a:.1f} done")
# retreat: back off -y a bit then up
tcp, R = door_pose(0.0)
r.move_tcp_cl(tcp + [0, -0.05, 0], R, secs=2.0, check=chk, iters=0)
r.move_tcp_cl(tcp + [0, -0.05, 0.25], R, secs=4.0, check=chk, iters=0)
print("DONE_DOOR")
rclpy.shutdown()
