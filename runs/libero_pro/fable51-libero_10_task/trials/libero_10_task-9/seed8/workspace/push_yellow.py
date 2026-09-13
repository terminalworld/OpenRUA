import sys
from ctl import *
c = Ctl("push_yellow"); r, sc = c.r, c.sc
dry = "--go" not in sys.argv
sc.publish(world_objects() + yellow_objects())
r.report("start")
al = np.radians(20)
z = np.array([np.sin(al), 0, -np.cos(al)]); y = np.array([0, -1., 0]); x = np.cross(y, z)
R = R_from_axes(x, y, z)
def origin(tip): return np.asarray(tip) - 0.1034 * z
tip_hi  = np.array([-0.05, -0.002, 1.05])
tip_lo  = np.array([-0.05, -0.002, 0.94])
tip_end = np.array([ 0.10, -0.002, 0.94])
q0 = [0.0,0.6,0.0,-1.9,0.0,2.5,0.785]
for name, tip in [("hi", tip_hi), ("lo", tip_lo), ("end", tip_end)]:
    q = c.ik_valid(origin(tip), R, seed=q0, ignore=("yellow",) if name == "end" else ())
    print(name, np.round(origin(tip), 3), None if q is None else np.round(q, 3), flush=True)
    if q is not None: q0 = q
if dry: sys.exit()
if r.finger_gap() > 0.01:
    r.gripper(0.0)
p_now, R_now = r.fk()
at_hi = np.linalg.norm(p_now - origin(tip_hi)) < 0.01
if not at_hi:
    if p_now[2] < 1.2:
        assert c.line(p_now + [0, 0, 1.25 - p_now[2]], R_now, n=3, t=3.0), 'lift failed'
        r.report('lifted')
    qhi = c.ik_valid(origin(tip_hi), R, seed=[0.0,0.6,0.0,-1.9,0.0,2.5,0.785])
    assert c.goto_q(qhi, t=6.0), "goto hi failed"
    r.report("hi")
assert c.line(origin(tip_lo), R, n=4, t=3.0), "descend failed"
r.report("lo")
# push: tolerate contacts with the yellow boxes (we are pushing it)
assert c.line(origin(tip_end), R, n=6, t=6.0, ignore=("yellow",)), "push failed"
r.report("end")
assert c.line(origin(tip_end + [0, 0, 0.12]), R, n=4, t=3.0, ignore=("yellow",)), "retract failed"
r.report("up")
r.snap("agentview"); r.snap("birdview")
