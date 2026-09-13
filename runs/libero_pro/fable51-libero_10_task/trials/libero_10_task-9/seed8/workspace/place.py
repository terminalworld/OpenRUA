"""Set the (horizontally hanging) mug down inside the cavity: insert high, lower until the rim touches,
then advance+descend so the mug rights itself onto the cavity floor."""
import sys
from common import *
c = Ctl("place"); r, sc = c.r, c.sc
dry = "--go" not in sys.argv
scene_no_white(sc)
Rf = hand_R(np.pi / 2)
# hanging mug, measured in world relative to hand (hand at y=0.03,z=1.10): x[-0.096,-0.003] y[0.08,0.21] z[0.98,1.05]
p_now, _ = r.fk(); r.report("start")
lo_w = np.array([-0.096, 0.08, 0.98]) - p_now; hi_w = np.array([-0.003, 0.21, 1.05]) - p_now
corners = np.array([[lo_w[0] if sx < 0 else hi_w[0], lo_w[1] if sy < 0 else hi_w[1], lo_w[2] if sz < 0 else hi_w[2]]
                    for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
ch = corners @ Rf
sc.attach("carried_mug", None, None, detach=True)
sc.attach("carried_mug", ch.min(0) - 0.005, ch.max(0) + 0.005)
print("hanging mug box (hand)", np.round(ch.min(0), 3), np.round(ch.max(0), 3), flush=True)
X = -0.047
P1 = np.array([X, 0.19, 1.10])    # rim end inside the opening
P2 = np.array([X, 0.19, 1.062])   # rim about to touch the floor
P3 = np.array([X, 0.215, 1.043])  # bar 5.5 cm above the floor -> mug upright, centre y~0.37
q = r.arm_q()
for name, P, ign in [("P1", P1, ()), ("P2", P2, ()), ("P3", P3, ("carried_mug",))]:
    qq = c.ik_valid(P, Rf, seed=q, tries=8, ignore=ign)
    print(f"  {name}: {None if qq is None else np.round(qq, 2)}", flush=True)
    if qq is None: sys.exit(name + " invalid")
    q = qq
if dry: sys.exit()
assert c.line(P1, Rf, n=5, t=5.0), "insert failed"; r.report("P1")
r.snap("robot0_eye_in_hand", "/workspace/eih_P1.png")
assert c.line(P2, Rf, n=3, t=3.0), "lower failed"; r.report("P2")
r.snap("robot0_eye_in_hand", "/workspace/eih_P2.png")
try:
    c.line(P3, Rf, n=4, t=4.0, ignore=("carried_mug",))
except RuntimeError as e:
    print("P3:", e, flush=True)
r.report("P3")
r.snap("robot0_eye_in_hand", "/workspace/eih_P3.png")
print("gap", r.finger_gap())
