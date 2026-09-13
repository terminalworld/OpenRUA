from rob import *
r = Robot()
CX, CY, ang = 0.219, 0.337, math.radians(30.6)
c = np.array([math.cos(ang), math.sin(ang), 0.0])
f = np.array([-math.sin(ang), math.cos(ang), 0.0])
for th in (0, 15, 25, 35, 45, 55):
    t = math.radians(th)
    a = math.sin(t) * c + np.array([0, 0, -math.cos(t)])
    q = quat_from_axes(a, f)
    for z in (0.60, 0.445):
        sol = r.solve_ik(CX, CY, z, q)
        log(f"tilt {th} z {z}:", None if sol is None else np.round(sol, 3))
    # also flipped finger axis
    q2 = quat_from_axes(a, -f)
    sol = r.solve_ik(CX, CY, 0.445, q2); log(f"tilt {th} z 0.445 flipped:", None if sol is None else np.round(sol, 3))
