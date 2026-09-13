import numpy as np, rclpy
from ctl import *
c = Ctl()
th = np.radians(17)
Zh = [0, np.cos(th), -np.sin(th)]          # look toward +y, 17 deg down
R = R_from_axes(zaxis=Zh, yaxis=[0, -np.sin(th), -np.cos(th)])  # X_h = +x world
print("target R X:", R.as_matrix()[:,0].round(3))
c.move([-0.08, -0.12, 1.08], R, secs=4.0)
print(fmt(*c.hand_pose()))
rclpy.shutdown()
