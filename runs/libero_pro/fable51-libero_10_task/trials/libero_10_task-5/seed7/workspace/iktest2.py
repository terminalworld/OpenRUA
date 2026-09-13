import sys, numpy as np, rclpy
sys.path.insert(0,'/workspace')
from ctl import Ctl, BASE_IN_WORLD
c=Ctl()
q0=c.arm_q(); print('q0',np.round(q0,3))
# current hand pose from tf2_echo: base (0.457,0,0.358) quat (1,0,-0.028,0)
for pos,quat in [((0.457,0,0.358),(1,0,-0.028,0)), ((0.457,0,0.358),(1,0,0,0)), ((0.5,0,0.3),(1,0,0,0)), ((0.66,-0.085,0.25),(1,0,0,0)), ((0.66,-0.085,0.15),(1,0,0,0)), ((0.6,0,0.15),(1,0,0,0))]:
    try:
        sol=c.solve_ik(np.array(pos)+BASE_IN_WORLD, quat, seed=q0); print(pos,quat,'OK',np.round(sol,3))
    except SystemExit as e: print(pos,quat,e)
rclpy.shutdown()
