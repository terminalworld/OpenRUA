import numpy as np, rclpy, subprocess
from ctl import *
c = Ctl()
BOTTLE = np.array([-0.1665, 0.0335])
GZ = 0.955
phi = np.radians(10)
R_g = R_from_axes(zaxis=[np.cos(phi), 0, -np.sin(phi)], yaxis=[0, -1, 0])   # approach +x, camera on top
print("grasp R: Z", R_g.as_matrix()[:,2].round(3), "Y", R_g.as_matrix()[:,1].round(3), "X", R_g.as_matrix()[:,0].round(3))
print("fingers:", c.fingers())
print("1) transit above/behind bottle")
move_checked(c, [-0.30, BOTTLE[1], 1.12], R_g, secs=5.0)
print("2) pre-grasp")
move_checked(c, [-0.27, BOTTLE[1], GZ], R_g, secs=4.0)
subprocess.run(["python3", "tools/perception/cam_snap.py", "robot0_eye_in_hand", "pregrasp.png"], timeout=60)
print("3) approach")
move_checked(c, [BOTTLE[0] + 0.008, BOTTLE[1], GZ], R_g, secs=3.0)
np.save("q_grasp.npy", np.array(c.arm_q()))
rclpy.shutdown()
