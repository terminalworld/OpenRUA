from ctl import *
r=Robot("step1")
print("base", r.base)
p,q=r.hand_pose(); print("hand", p, q)
print("topdown quat yaw0", topdown_quat(0), "yaw90", topdown_quat(np.pi/2))
# move above caddy middle column to inspect with eye-in-hand: camera ~5cm +x of hand
ok=r.move_pose([-0.48,-0.14,1.30], topdown_quat(0), 4)
color,depth,P,T=r.snap("robot0_eye_in_hand","/workspace/eih1.png")
np.save("/workspace/eih1_P.npy",P)
print("cam T", T[:3,3])
