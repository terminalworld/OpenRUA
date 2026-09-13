from ctl import *
c = Ctl("t9"); r, sc = c.r, c.sc
sc.publish(world_objects() + yellow_objects(dx=0.126, dy=-0.013))
bar = np.array([-0.097, -0.1755, 0.955])
for deg in [30, 40, 50, 60]:
    ph = np.radians(deg)
    z = np.array([0, -np.cos(ph), -np.sin(ph)])
    for sgn in (+1, -1):
        y = np.array([sgn * 1., 0, 0]); x = np.cross(y, z); R = R_from_axes(x, y, z)
        tip = bar + 0.012 * z
        o_lo = tip - 0.1034 * z
        o_hi = o_lo + np.array([0, 0.04, 0.10])
        seed = [0.0, 0.6, 0.0, -1.9, 0.0, 2.5, 0.785]
        q_hi = c.ik_valid(o_hi, R, seed=seed, tries=8)
        q_lo = c.ik_valid(o_lo, R, seed=q_hi or seed, tries=8, ignore=("white_handle",))
        print(f"pitch {deg} sgn {sgn}: o_lo={np.round(o_lo,3)} hi={None if q_hi is None else np.round(q_hi,2)} lo={None if q_lo is None else np.round(q_lo,2)}", flush=True)
