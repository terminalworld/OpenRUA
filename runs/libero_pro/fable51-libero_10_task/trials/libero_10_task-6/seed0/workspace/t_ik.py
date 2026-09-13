from robot import *
r = Robot()
pos, quat = r.fk(); print("hand world", np.round(pos,4), np.round(quat,4))
q = r.ik(pos, quat, at_tcp=False); print("IK roundtrip", None if q is None else np.round(q,3), "current", np.round(r.q(),3))
for z in [0.75, 0.70, 0.66]:
    q = r.ik([-0.196, 0.056, z], down_quat(0))
    print("IK pregrasp z",z, None if q is None else np.round(q,3))
    if q is not None: print("  tcp check", np.round(r.tcp(q)[0],4))
q = r.ik([-0.057, 0.115, 0.60], down_quat(0)); print("IK above pudding", None if q is None else np.round(q,3))
q = r.ik([0.126, 0.056, 0.70], down_quat(0)); print("IK above plate", None if q is None else np.round(q,3))
q = r.ik([0.126, 0.17, 0.55], down_quat(0)); print("IK right of plate", None if q is None else np.round(q,3))
