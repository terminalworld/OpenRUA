import sys, numpy as np, rclpy
sys.path.insert(0,'/workspace')
from ctl import Ctl
from scipy.spatial.transform import Rotation as Rot
c=Ctl()
q0=c.arm_q()
seed=q0
for name,(x,y,z,yaw) in {'lift_hi':(-0.087,0.046,1.30,0),'mid':(-0.28,-0.05,1.30,45),'preplace_hi':(-0.4695,-0.1425,1.30,90),'place':(-0.4695,-0.1425,1.075,90)}.items():
    R = Rot.from_euler("z", yaw, degrees=True) * Rot.from_euler("x", 180, degrees=True)
    hand = np.array([x,y,z]) - 0.1034*R.as_matrix()[:,2]
    try:
        sol=c.solve_ik(hand, R.as_quat(), seed=seed); print(name,'OK',np.round(sol,3)); seed=sol
    except SystemExit as e: print(name,e)
rclpy.shutdown()
