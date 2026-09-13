from rob import *
r = Robot()
cur = r.arm_positions()
q = (0.9996, 0.0, -0.0284, 0.0)
R = quat_to_R(*q)
hand_tf = np.array([-0.053, 0.0, 0.7776])
for label, hand in [("world-coords", hand_tf), ("base-coords", hand_tf - BASE_W)]:
    tcp = hand + TCP * R[:, 2]
    # solve_ik subtracts BASE_W itself, so add it back to test raw frames
    try:
        sol = r.solve_ik(tcp + BASE_W, q, cur)
        print(label, "OK sol", np.round(sol, 3).tolist(), "maxdiff", max(abs(a-b) for a,b in zip(sol,cur)))
    except Exception as e:
        print(label, "FAIL", e)
r.close()
