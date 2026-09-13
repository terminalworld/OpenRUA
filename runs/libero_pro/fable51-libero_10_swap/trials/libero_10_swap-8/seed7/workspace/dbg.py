from rob import *
for yaw in (0, -math.pi/2):
    R = grasp_R(yaw, 0); q = q_from_R(R)
    print("yaw", yaw, "\nR=\n", np.round(R,3), "\nq=", np.round(q,4), "\nR_from_q=\n", np.round(R_from_q(*q),3), "det", np.linalg.det(R))
