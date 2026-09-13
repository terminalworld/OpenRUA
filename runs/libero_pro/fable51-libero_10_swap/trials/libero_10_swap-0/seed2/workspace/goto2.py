#!/usr/bin/env python3
"""goto2.py x y z qx qy qz qw [secs] [--steps N]  (world frame, panda_link8 pose)
Straight Cartesian path from the current link8 pose: N waypoints, IK per waypoint seeded
from the previous solution (nearest branch), sent as ONE trajectory; resent until converged."""
import sys, random, numpy as np, rclpy
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from scipy.spatial.transform import Rotation as R
ARM=[f"panda_joint{i}" for i in range(1,8)]
args=[a for a in sys.argv[1:] if not a.startswith("--")]
steps=int(sys.argv[sys.argv.index("--steps")+1]) if "--steps" in sys.argv else 1
x,y,z,qx,qy,qz,qw=map(float,args[:7]); secs=float(args[7]) if len(args)>7 else 4.0
rclpy.init(); node=rclpy.create_node("goto2")
js={}
node.create_subscription(JointState,"/joint_states",lambda m: js.__setitem__("m",m),1)
def cur():
    js.pop("m",None)
    while "m" not in js: rclpy.spin_once(node,timeout_sec=0.2)
    d=dict(zip(js["m"].name,js["m"].position)); return [d[j] for j in ARM], d
fk=node.create_client(GetPositionFK,"/compute_fk"); fk.wait_for_service()
ik=node.create_client(GetPositionIK,"/compute_ik"); ik.wait_for_service()
def do_fk(joints, link):
    r=GetPositionFK.Request(); r.fk_link_names=[link]; s=JointState(); s.name=list(ARM); s.position=list(joints)
    r.robot_state.joint_state=s
    f=fk.call_async(r); rclpy.spin_until_future_complete(node,f,timeout_sec=30)
    ps=f.result().pose_stamped[0]; p=ps.pose.position; o=ps.pose.orientation
    return np.array([p.x,p.y,p.z]), np.array([o.x,o.y,o.z,o.w])
def do_ik(pos, quat, seed, tries=12):
    best=None
    for k in range(tries):
        q=GetPositionIK.Request(); q.ik_request.group_name="panda_arm"
        pp=q.ik_request.pose_stamped.pose
        pp.position.x,pp.position.y,pp.position.z=map(float,pos)
        pp.orientation.x,pp.orientation.y,pp.orientation.z,pp.orientation.w=map(float,quat)
        s=JointState(); s.name=list(ARM)
        s.position=[v+(random.uniform(-0.1,0.1) if k else 0.0) for v in seed]
        q.ik_request.robot_state.joint_state=s; q.ik_request.timeout.sec=1
        f=ik.call_async(q); rclpy.spin_until_future_complete(node,f,timeout_sec=60)
        r=f.result()
        if r is None or r.error_code.val!=1: continue
        sol=dict(zip(r.solution.joint_state.name,r.solution.joint_state.position)); t=[sol[j] for j in ARM]
        d=sum(abs(a-b) for a,b in zip(t,seed))
        if best is None or d<best[0]: best=(d,t)
        if d<0.3: break
    return best
p0,_=cur()
start_pos,start_q=do_fk(p0,"panda_link8")
target_pos=np.array([x,y,z]); target_q=np.array([qx,qy,qz,qw])
if np.dot(start_q,target_q)<0: start_q=-start_q
key=R.from_quat([start_q,target_q])
from scipy.spatial.transform import Slerp
slerp=Slerp([0,1],key)
seed=p0; wps=[]
for i in range(1,steps+1):
    a=i/steps; pos=start_pos*(1-a)+target_pos*a; quat=slerp([a]).as_quat()[0]
    best=do_ik(pos,quat,seed)
    if best is None: print(f"IK FAILED at step {i}"); sys.exit(2)
    print(f"step {i}: dist {best[0]:.3f} joints {[round(v,3) for v in best[1]]}")
    seed=best[1]; wps.append(best[1])
target=wps[-1]
if "--dry" in sys.argv: print("dry run, no motion"); sys.exit(0)
client=ActionClient(node,FollowJointTrajectory,"/panda_arm_controller/follow_joint_trajectory"); client.wait_for_server()
for attempt in range(5):
    g=FollowJointTrajectory.Goal(); g.trajectory.joint_names=list(ARM)
    pts=[]
    if attempt==0:
        for i,w in enumerate(wps,1):
            t=secs*i/steps; pt=JointTrajectoryPoint(positions=w); pt.time_from_start=Duration(sec=int(t),nanosec=int((t%1)*1e9)); pts.append(pt)
    else:
        pt=JointTrajectoryPoint(positions=target); t=max(1.5,secs/2); pt.time_from_start=Duration(sec=int(t),nanosec=int((t%1)*1e9)); pts=[pt]
    g.trajectory.points=pts
    s=client.send_goal_async(g); rclpy.spin_until_future_complete(node,s)
    rf=s.result().get_result_async(); rclpy.spin_until_future_complete(node,rf)
    code=rf.result().result.error_code
    p,_=cur(); err=max(abs(a-b) for a,b in zip(p,target))
    print(f"attempt {attempt} code={code} max_joint_err={err:.4f}")
    if err<0.01: break
p,d=cur(); pos,ori=do_fk(p,"panda_hand")
print("hand", np.round(pos,4).tolist(), np.round(ori,3).tolist())
print("fingers", round(d["panda_finger_joint1"],4), round(d["panda_finger_joint2"],4))
