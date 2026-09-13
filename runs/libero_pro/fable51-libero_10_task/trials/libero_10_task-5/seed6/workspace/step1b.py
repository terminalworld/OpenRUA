from ctl import *
r=Robot("step1b")
r.base=np.zeros(3)  # test hypothesis: MoveIt model frame == world
p,q=r.hand_pose(); print("FK hand", p, q)
t=r.tfbuf.lookup_transform("world","panda_hand",Time()).transform.translation; print("TF hand", t.x,t.y,t.z)
sol=r.ik_solve(p,q); print("IK at current pose:", None if sol is None else np.round(sol,3), "current", np.round(r.arm_q(),3))
sol=r.ik_solve([-0.48,-0.14,1.30], topdown_quat(0)); print("IK above caddy:", None if sol is None else np.round(sol,3))
