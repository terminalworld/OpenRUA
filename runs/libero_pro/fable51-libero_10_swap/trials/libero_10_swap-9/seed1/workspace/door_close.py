import rb, numpy as np
R=rb.Robot()
Q=np.load("/workspace/door_arc_q.npy"); q0=list(Q[0])
hinge=np.array([-0.30,-0.36]); r=0.16; z=1.02; th=np.deg2rad(45)
def pose(phi, off):
    f=np.deg2rad(phi); n=np.array([-np.sin(f),np.cos(f),0]); rad=np.array([np.cos(f),np.sin(f),0])
    p=np.array([*hinge,z])+r*rad-off*n; hz=np.sin(th)*n-np.cos(th)*np.array([0,0,1.0])
    return p, rb.frame_quat(hz, rad)
print("start tcp",np.round(R.tcp()[0],4),"gap",round(R.finger_gap(),4))
# 1. re-add door box, plan collision-free to a pre-start pose 7 cm behind the door's outer face
door=rb._box("door",(-0.345,-0.49,1.005),(0.25,0.03,0.21),yaw=float(np.arctan2(-0.24,-0.07)))
rb.apply_scene(R,[door]); print("door box added")
p_pre,qt=pose(-112,0.08)
qpre=R.ik(p_pre,qt,seed=q0,attempts=5); print("pre dq from q0",round(np.abs(np.array(qpre)-np.array(q0)).max(),3))
v=rb.state_valid(R,qpre); print("pre valid",v)
assert v[0]
plan=rb.plan_to_joints(R,qpre,secs=15.0,attempts=8); assert plan, "plan failed"
print("plan pts",len(plan)); rb.exec_plan(R,plan,tscale=3)
print("at pre: tcp",np.round(R.tcp()[0],4),"jerr",round(np.abs(np.array(R.arm_q())-np.array(qpre)).max(),4))
# 2. remove door box, approach the door face and push along the hinge arc
from moveit_msgs.msg import CollisionObject
co=CollisionObject(); co.id="door"; co.header.frame_id="world"; co.operation=CollisionObject.REMOVE; rb.apply_scene(R,[co])
appr=[pose(-112,o) for o in (0.06,0.04,0.02)]
qa=rb.pose_path(R,appr,seed=qpre,validate=True); assert qa, "approach path failed"
R.move_path(qa,0.6); print("at arc start: tcp",np.round(R.tcp()[0],4))
arc=[pose(phi,0.02) for phi in np.arange(-112,4,4.0)]
qs=rb.pose_path(R,arc,seed=qa[-1],validate=True); assert qs, "arc path failed"
R.move_path(qs,0.5)
qn=np.array(R.arm_q()); err=np.abs(qn-np.array(qs[-1])).max(); print("arc done jerr",round(err,4),"tcp",np.round(R.tcp()[0],4))
if err>0.01:
    R.move(qs[-1],2.0,retries=1); print("after converge tcp",np.round(R.tcp()[0],4),"jerr",round(np.abs(np.array(R.arm_q())-np.array(qs[-1])).max(),4))
# 3. retreat -y then up
p_end,qt_end=pose(0,0.02)
ret=[(p_end+np.array([0,-0.08,0]),qt_end),(p_end+np.array([0,-0.12,0.12]),qt_end)]
qr=rb.pose_path(R,ret,validate=True) or rb.pose_path(R,ret,validate=False)
if qr: R.move_path(qr,0.8)
print("retreat tcp",np.round(R.tcp()[0],4))
# 4. verify with birdview + agentview
B=R.cloud("birdview"); np.save("/workspace/bird8_pc.npy",B); R.snap("birdview","/workspace/bird8.png"); R.snap("agentview","/workspace/agent21.png")
m=np.isfinite(B[...,2])&(B[...,2]>1.04)&(B[...,2]<1.12)&(B[...,0]>-0.45)&(B[...,0]<0.10)&(B[...,1]>-0.70)&(B[...,1]<-0.30)
P=B[m]; print("front-region high pts",len(P))
for x0 in np.arange(-0.44,0.10,0.04):
    s=P[(P[:,0]>=x0)&(P[:,0]<x0+0.04)]
    if len(s): print(f"x {x0:+.2f}: y min {s[:,1].min():+.3f} max {s[:,1].max():+.3f} n {len(s)}")
print("DONE")
