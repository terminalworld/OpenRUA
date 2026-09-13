from rob import *
r = Robot()
CX, CY, ang = 0.228, 0.346, math.radians(31.0)
c = np.array([math.cos(ang), math.sin(ang), 0.0]); f = np.array([-math.sin(ang), math.cos(ang), 0.0])
lim = np.array(FJT["limits_rad"])
cur = r.arm_q()
seeds = {"cur": cur, "stretch": [0.4, 1.0, 0.0, -1.2, 0.0, 2.2, 0.8], "stretch2": [0.4, 1.2, 0.0, -1.0, 0.0, 2.6, 0.8]}
best=[]
for th in (30, 40, 50, 60, 70):
    t = math.radians(th); a = math.sin(t) * c + np.array([0, 0, -math.cos(t)])
    for sgn in (1, -1):
        q = quat_from_axes(a, sgn * f)
        for sn, sd in seeds.items():
            sol = r.solve_ik(CX, CY, 0.445, q, seed=sd)
            if sol is None: continue
            sol = np.array(sol); margin = np.min(np.minimum(sol - lim[:,0], lim[:,1] - sol))
            # pre-grasp 12cm back along approach, seeded from sol
            pre = np.array([CX, CY, 0.445]) - 0.12 * a
            sol2 = r.solve_ik(*pre, q, seed=sol)
            jump = None if sol2 is None else np.max(np.abs(np.array(sol2) - sol))
            log(f"tilt {th} sgn {sgn} seed {sn}: margin {margin:.2f} pre_ok {sol2 is not None} jump {jump} sol {sol.round(2)}")
            if sol2 is not None: best.append((margin, th, sgn, sn, sol.round(3).tolist(), np.round(sol2,3).tolist()))
best.sort(reverse=True)
log("BEST", best[:3])
