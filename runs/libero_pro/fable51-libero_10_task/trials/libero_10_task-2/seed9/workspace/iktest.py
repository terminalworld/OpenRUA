import numpy as np, rclpy
from arm import Arm, BASE_IN_WORLD, JOINTS
arm = Arm()
js = arm.joints(); cur = [js[j] for j in JOINTS]
q = (1,0,0,0)
for label, pos in [("world-frame pose", np.array([-0.203,0,1.27])), ("base-frame pose (+BASE so solve_ik subtracts to base)", np.array([-0.203,0,1.27]) + BASE_IN_WORLD)]:
    try:
        sol = arm.solve_ik(pos, q)
        print(label, "-> max dev from current:", round(max(abs(a-b) for a,b in zip(sol,cur)),3), [round(v,3) for v in sol])
    except SystemExit as e:
        print(label, "->", e)
rclpy.shutdown()
