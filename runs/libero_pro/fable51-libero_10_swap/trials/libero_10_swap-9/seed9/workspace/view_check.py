from rob import *
r = Robot('chk')
s=np.sqrt(0.5)
q_view45 = quat_from_axes([0,s,s],[1,0,0],[0,s,-s])   # hand z down-forward(+y) 45deg
q_view60 = quat_from_axes([0,np.cos(np.radians(30)),np.sin(np.radians(30))],[1,0,0],[0,np.sin(np.radians(30)),-np.cos(np.radians(30))])  # 60deg down
q_view30 = quat_from_axes([0,np.cos(np.radians(60)),np.sin(np.radians(60))],[1,0,0],[0,np.sin(np.radians(60)),-np.cos(np.radians(60))])  # 30deg down
cands = [
 ('v45 a', [-0.155,-0.50,1.05], q_view45), ('v45 b', [-0.155,-0.46,1.08], q_view45), ('v45 c', [-0.155,-0.44,1.10], q_view45),
 ('v60 a', [-0.155,-0.46,1.15], q_view60), ('v60 b', [-0.155,-0.42,1.20], q_view60),
 ('v30 a', [-0.155,-0.50,1.02], q_view30), ('v30 b', [-0.155,-0.46,1.03], q_view30),
]
seed = r.arm_q()
for name,p,q in cands:
    sol = r.ik(p,q,seed=seed)
    print(name, p, None if sol is None else np.round(sol,3), flush=True)
