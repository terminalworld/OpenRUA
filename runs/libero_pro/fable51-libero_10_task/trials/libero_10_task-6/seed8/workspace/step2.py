from robot import *
r = Robot()
MUG = np.array([-0.1821, 0.0005]); RIM_Z = 0.5735; R_MUG = 0.0415
q = down_quat(90)  # fingers close along world x
grasp_xy = MUG + [R_MUG, 0]
print("pre-grasp"); r.move_tcp([*grasp_xy, 0.66], q, 4.0)
r.gripper(True)
print("descend");  r.move_tcp([*grasp_xy, 0.548], q, 3.0)
print("tcp", r.tcp_pose()[0].round(4), "fingers", r.fingers())
