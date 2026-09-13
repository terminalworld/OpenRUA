from ctl import *
c = Ctl("lift"); r, sc = c.r, c.sc
sc.publish(world_objects(with_white_mug=False) + yellow_objects(dx=0.126, dy=-0.013), remove=["white_mug", "white_handle"])
p, R = r.fk(); r.report("before lift")
assert c.line(p + [0, 0, 0.15], R, n=4, t=4.0), "lift failed"
r.report("lifted")
print("gap", r.finger_gap())
r.snap("frontview"); r.snap("agentview")
