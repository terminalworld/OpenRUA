import sys
from ctl import *
c = Ctl("grasp"); r, sc = c.r, c.sc
dry = "--go" not in sys.argv
sc.publish(world_objects() + yellow_objects(dx=0.126, dy=-0.013))
r.report("start")
BAR = np.array([-0.097, -0.1755, 0.955])
PH = np.radians(30)
def hand_R(psi):
    d = np.array([np.cos(psi), np.sin(psi), 0])
    z = np.cos(PH) * d + np.array([0, 0, -np.sin(PH)])
    y = np.array([-np.sin(psi), np.cos(psi), 0]); x = np.cross(y, z)
    return R_from_axes(x, y, z)
R = hand_R(-np.pi / 2); z = R[:, 2]
tip = BAR + 0.012 * z
o_lo = tip - 0.1034 * z
o_hi = o_lo + np.array([0, 0.04, 0.10])
seed = [0.0, 0.6, 0.0, -1.9, 0.0, 2.5, 0.785]
q_hi = c.ik_valid(o_hi, R, seed=seed, tries=8)
q_lo = c.ik_valid(o_lo, R, seed=q_hi, tries=8, ignore=("white_handle",))
print("o_lo", np.round(o_lo, 3), "q_hi", np.round(q_hi, 3), "q_lo", np.round(q_lo, 3), flush=True)
if dry: sys.exit()
if r.finger_gap() < 0.07: r.gripper(0.08)
p_now, R_now = r.fk()
if p_now[2] < 1.15:
    assert c.line(p_now + [0, 0, 1.20 - p_now[2]], R_now, n=3, t=3.0), "lift failed"
assert c.goto_q(q_hi, t=6.0), "goto pre-grasp failed"
r.report("pre-grasp")
assert c.line(o_lo, hand_R(-np.pi / 2), n=5, t=4.0, ignore=("white_handle",)), "approach failed"
r.report("grasp pose")
gap = r.gripper(0.0)
print("GAP after close:", gap, flush=True)
r.snap("agentview"); r.snap("frontview")
