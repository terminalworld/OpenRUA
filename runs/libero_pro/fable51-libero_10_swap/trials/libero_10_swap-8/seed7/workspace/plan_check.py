from rob import *
r = Robot()
home = r.arm_q()
POTS = {"A": (-0.204, -0.202), "B": (-0.072, 0.225)}
HANDLE_DY = -0.061          # handle bar centre relative to pot centre (points -y)
GRASP_Z = 1.025             # TCP height for handle bar (~12.6 cm above table)
LIFT_Z = 1.20
PLACE = {"A": (0.18, 0.00), "B": (0.18, 0.094)}
PLACE_Z = 1.060
for pot in ["A", "B"]:
    px, py = POTS[pot]
    seed = home
    print("== pot", pot)
    for name, tcp, yaw in [("pre", (px, py+HANDLE_DY, 1.15), 0), ("grasp", (px, py+HANDLE_DY, GRASP_Z), 0), ("lift", (px, py+HANDLE_DY, LIFT_Z), 0)]:
        for y in (yaw, yaw+math.pi):
            q = r.ik_tcp(tcp, grasp_R(y, 0), seed=seed)
            if q is not None: break
        print(f"  {name} yaw={y:.2f}: {None if q is None else np.round(q,3)}")
        if q is not None: seed = q
    tx, ty = PLACE[pot]
    for name, tcp in [("carry", (tx-0.06, ty, LIFT_Z)), ("place", (tx-0.06, ty, PLACE_Z)), ("retreat", (tx-0.06, ty, LIFT_Z))]:
        for y in (-math.pi/2, math.pi/2):
            q = r.ik_tcp(tcp, grasp_R(y, 0), seed=seed)
            if q is not None: break
        print(f"  {name} yaw={y:.2f}: {None if q is None else np.round(q,3)}")
        if q is not None:
            seed = q
            print("     fk check tcp:", np.round(r.fk_hand(q)[2],4), "hy:", np.round(R_from_q(*r.fk_hand(q)[1])[:,1],2))
