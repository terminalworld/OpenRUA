import sys
from ctl import *
c = Ctl("carry"); r, sc = c.r, c.sc
dry = "--go" not in sys.argv
sc.publish(world_objects(with_white_mug=False) + yellow_objects(dx=0.126, dy=-0.013), remove=["white_mug", "white_handle"])
PH = np.radians(30)
def hand_R(psi):
    d = np.array([np.cos(psi), np.sin(psi), 0])
    z = np.cos(PH) * d + np.array([0, 0, -np.sin(PH)])
    y = np.array([-np.sin(psi), np.cos(psi), 0]); x = np.cross(y, z)
    return R_from_axes(x, y, z)
# mug box relative to the hand (computed from the grasp geometry, world -> hand at psi=-90deg)
R0 = hand_R(-np.pi / 2)
mug_c_rel = np.array([-0.0015, -0.1575, -0.0435])          # mug centre minus hand origin, world, at grasp
half = np.array([0.047, 0.062, 0.0575])                      # incl. handle margin on +y side (asymmetric ok)
corners = np.array([mug_c_rel + np.array([sx, sy, sz]) * half for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
ch = corners @ R0                                            # to hand frame: R0^T * v  == v @ R0
lo, hi = ch.min(0) - 0.005, ch.max(0) + 0.005
print("attached mug box (hand frame) lo", np.round(lo, 3), "hi", np.round(hi, 3), flush=True)
sc.attach("carried_mug", lo, hi)
p0, _ = r.fk(); r.report("start")
ORIG = np.array([-0.097, -0.096, 1.15])
# 1. yaw arc
psis = np.radians(np.arange(-90, -271, -30))
qs, q = [], r.arm_q()
for ps in psis:
    qq = c.ik_valid(ORIG, hand_R(ps), seed=q, tries=8)
    print(f"  psi {np.degrees(ps):.0f}: {None if qq is None else np.round(qq,2)}", flush=True)
    if qq is None: sys.exit("arc IK failed")
    if np.abs(np.array(qq) - np.array(q)).max() > 1.2: sys.exit("arc joint jump")
    qs.append(qq); q = qq
# 2. transit, 3. descend, 4. insert
Rf = hand_R(np.pi / 2)
P_tr = np.array([-0.047, 0.03, 1.15]); P_dn = np.array([-0.047, 0.03, 1.053]); P_in = np.array([-0.047, 0.1875, 1.053])
for name, P in [("transit", P_tr), ("descend", P_dn), ("insert", P_in)]:
    qq = c.ik_valid(P, Rf, seed=q, tries=8)
    print(f"  {name}: {None if qq is None else np.round(qq,2)}", flush=True)
    if qq is None: sys.exit(name + " IK failed")
    q = qq
if dry: sys.exit()
for i, qq in enumerate(qs[1:]):
    assert c.goto_q(qq, t=3.0), f"arc step {i} path invalid"
r.report("rotated")
assert c.line(P_tr, Rf, n=4, t=4.0), "transit failed"; r.report("transit")
assert c.line(P_dn, Rf, n=4, t=3.0), "descend failed"; r.report("descended")
r.snap("frontview")
assert c.line(P_in, Rf, n=6, t=5.0), "insert failed"; r.report("inserted")
r.snap("frontview"); r.snap("agentview")
print("gap", r.finger_gap())
