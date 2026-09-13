import rb, numpy as np
R=rb.Robot()
print("tcp",np.round(R.tcp()[0],4))
rb.apply_scene(R,[rb._box("door",(-0.175,-0.375,1.005),(0.25,0.03,0.21))]); print("closed-door box added")
hinge=np.array([-0.30,-0.36]); r=0.16; z=1.02; th=np.deg2rad(45)
n=np.array([0,1.0,0]); rad=np.array([1.0,0,0]); qt=rb.frame_quat(np.sin(th)*n-np.cos(th)*np.array([0,0,1.0]),rad)
p=np.array(R.tcp()[0])
back=[(p+np.array([0,-d,0]),qt) for d in (0.01,0.02,0.03,0.04)]
qb=rb.pose_path(R,back,validate=False); assert qb, "backoff failed"
R.move_path(qb,0.5); print("backed off tcp",np.round(R.tcp()[0],4))
# plan to a high pose above/behind the table edge, pointing down
qgoal=None
for k in range(20):
    try: q=R.ik(np.array([-0.15,-0.25,1.40]),rb.frame_quat([0,0,-1],[0,1,0]),attempts=3)
    except RuntimeError: continue
    if rb.state_valid(R,q)[0]: qgoal=q; break
assert qgoal is not None
plan=rb.plan_to_joints(R,qgoal,secs=15.0,attempts=8); assert plan, "plan failed"
rb.exec_plan(R,plan,tscale=3); print("home tcp",np.round(R.tcp()[0],4))
B=R.cloud("birdview"); np.save("/workspace/bird9_pc.npy",B); R.snap("birdview","/workspace/bird9.png"); R.snap("agentview","/workspace/agent22.png")
m=np.isfinite(B[...,2])&(B[...,2]>0.95)&(B[...,2]<1.12)&(B[...,0]>-0.50)&(B[...,0]<0.10)&(B[...,1]>-0.70)&(B[...,1]<-0.30)
P=B[m]; print("pts",len(P))
for x0 in np.arange(-0.48,0.10,0.04):
    s=P[(P[:,0]>=x0)&(P[:,0]<x0+0.04)]
    if len(s): print(f"x {x0:+.2f}: y min {s[:,1].min():+.3f}  z max {s[:,2].max():.3f} n {len(s)}")
print("DONE")
