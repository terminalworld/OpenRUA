from ctl import *
import ctl
c = Ctl()
t,q = c.tcp_world()
ctl.BASE_W = np.zeros(3)
sol,code=c.solve_ik(t, q, at_tcp=True); print("IK (world) back to current:", code, None if sol is None else np.round(np.array(sol)-np.array(c.arm_q()),3))
sol,code=c.solve_ik(t, DOWN, at_tcp=True); print("IK DOWN:", code, None if sol is None else np.round(sol,3))
sol,code=c.solve_ik([-0.2255,0.035,0.70], DOWN, at_tcp=True); print("IK above mug:", code, None if sol is None else np.round(sol,3))
sol,code=c.solve_ik([0.131,0.035,0.70], DOWN, at_tcp=True); print("IK above plate:", code, None if sol is None else np.round(sol,3))
sol,code=c.solve_ik([0.13,0.15,0.445], DOWN, at_tcp=True); print("IK pudding goal low:", code, None if sol is None else np.round(sol,3))
