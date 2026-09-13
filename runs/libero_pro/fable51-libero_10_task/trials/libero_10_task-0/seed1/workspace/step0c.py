from arm import *
r = Robot()
p, q, _ = r.fk_world(); print("hand world", p, q)
sol, code = r.ik_world(p, q, at_tcp=False); print("IK roundtrip code", code, "sol", np.round(sol,3) if sol else None, "cur", np.round(r.arm_q(),3))
# IK for pre-grasp above cream cheese: tcp at (0.09,-0.188, 0.55) top-down
sol, code = r.ik_world([0.09, -0.188, 0.55], topdown_quat(0.0), at_tcp=True); print("IK cheese pregrasp", code, np.round(sol,3) if sol else None)
sol, code = r.ik_world([-0.079, 0.042, 0.60], topdown_quat(0.0), at_tcp=True); print("IK can pregrasp", code, np.round(sol,3) if sol else None)
sol, code = r.ik_world([0.008, 0.253, 0.72], topdown_quat(0.0), at_tcp=True); print("IK basket", code, np.round(sol,3) if sol else None)
