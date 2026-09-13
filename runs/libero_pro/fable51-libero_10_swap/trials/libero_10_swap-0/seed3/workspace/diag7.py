from rob import *
r = Robot()
CX, CY, ang = 0.268, 0.353, math.radians(31.0)
c = np.array([math.cos(ang), math.sin(ang), 0.0]); f = np.array([-math.sin(ang), math.cos(ang), 0.0])
seed = [0.369, 1.114, 0.087, -1.314, -0.009, 2.945, 0.64]
for th in (30, 40, 50, 60):
    tt = math.radians(th); a = math.sin(tt) * c + np.array([0, 0, -math.cos(tt)])
    for sgn in (1, -1):
        q = quat_from_axes(a, sgn * f)
        for z in (0.45, 0.455):
            g = r.solve_ik(CX, CY, z, q, seed=seed)
            log(f"tilt {th} sgn {sgn} z {z}: {None if g is None else np.round(g,3)}")
