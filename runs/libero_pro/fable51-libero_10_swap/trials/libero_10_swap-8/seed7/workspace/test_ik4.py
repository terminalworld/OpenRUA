from rob import *
r = Robot()
print("fingers", r.fingers())
seed = r.arm_q()
for yaw in (0.0, -math.pi/2):
    for z in (1.20, 1.12):
        row = []
        for x in (0.08, 0.10, 0.12, 0.13, 0.14):
            q = r.ik_tcp((x, 0.097, z), grasp_R(yaw, 0), seed=seed)
            row.append("ok " if q is not None else "-- ")
        print(f"yaw {yaw:5.2f} z {z}: x=.08,.10,.12,.13,.14 -> {''.join(row)}")
for t in (0.35, 0.52):
    row=[]
    for x in (0.10, 0.13, 0.15, 0.18):
        q = r.ik_tcp((x, 0.097, 1.20), xbar_R(t), seed=seed); row.append("ok " if q is not None else "-- ")
    print(f"xbar tilt {t}: x=.10,.13,.15,.18 -> {''.join(row)}")
