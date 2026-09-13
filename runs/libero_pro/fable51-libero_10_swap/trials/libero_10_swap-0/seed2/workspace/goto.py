#!/usr/bin/env python3
"""goto.py x y z qx qy qz qw [secs] [--joints j1,...,j7]  (world frame, link8 orientation)
IK, then FollowJointTrajectory resent until joints converge; prints FK of panda_hand."""
import sys, rclpy
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
ARM=[f"panda_joint{i}" for i in range(1,8)]
args=[a for a in sys.argv[1:] if not a.startswith("--")]
joints_arg = sys.argv[sys.argv.index("--joints")+1] if "--joints" in sys.argv else None
rclpy.init(); node=rclpy.create_node("goto")
js={}
node.create_subscription(JointState,"/joint_states",lambda m: js.__setitem__("m",m),1)
def cur():
    js.pop("m",None)
    while "m" not in js: rclpy.spin_once(node,timeout_sec=0.2)
    d=dict(zip(js["m"].name,js["m"].position)); return [d[j] for j in ARM], d
def seed_state():
    s=JointState(); p,_=cur(); s.name=list(ARM); s.position=p; return s
fk=node.create_client(GetPositionFK,"/compute_fk"); fk.wait_for_service()
def do_fk():
    r=GetPositionFK.Request(); r.fk_link_names=["panda_hand"]; r.robot_state.joint_state=seed_state()
    f=fk.call_async(r); rclpy.spin_until_future_complete(node,f,timeout_sec=30)
    ps=f.result().pose_stamped[0]; p=ps.pose.position; o=ps.pose.orientation
    return (round(p.x,4),round(p.y,4),round(p.z,4)),(round(o.x,3),round(o.y,3),round(o.z,3),round(o.w,3))
if joints_arg:
    target=[float(t) for t in joints_arg.split(",")]; secs=float(args[0]) if args else 4.0
else:
    x,y,z,qx,qy,qz,qw=map(float,args[:7]); secs=float(args[7]) if len(args)>7 else 4.0
    ik=node.create_client(GetPositionIK,"/compute_ik"); ik.wait_for_service()
    import random
    best=None; p0,_=cur()
    for k in range(12):
        q=GetPositionIK.Request(); q.ik_request.group_name="panda_arm"
        pp=q.ik_request.pose_stamped.pose
        pp.position.x,pp.position.y,pp.position.z=x,y,z
        pp.orientation.x,pp.orientation.y,pp.orientation.z,pp.orientation.w=qx,qy,qz,qw
        s=JointState(); s.name=list(ARM)
        s.position=[v+(random.uniform(-0.15,0.15) if k else 0.0) for v in p0]
        q.ik_request.robot_state.joint_state=s; q.ik_request.timeout.sec=1
        f=ik.call_async(q); rclpy.spin_until_future_complete(node,f,timeout_sec=60)
        r=f.result()
        if r is None or r.error_code.val!=1: continue
        sol=dict(zip(r.solution.joint_state.name,r.solution.joint_state.position)); t=[sol[j] for j in ARM]
        d=sum(abs(a-b) for a,b in zip(t,p0))
        if best is None or d<best[0]: best=(d,t)
        if d<0.5: break
    if best is None: print("IK FAILED"); sys.exit(2)
    print(f"IK joint-distance {best[0]:.3f}"); target=best[1]
print("target joints", [round(t,4) for t in target])
client=ActionClient(node,FollowJointTrajectory,"/panda_arm_controller/follow_joint_trajectory"); client.wait_for_server()
for attempt in range(5):
    g=FollowJointTrajectory.Goal(); g.trajectory.joint_names=list(ARM)
    pt=JointTrajectoryPoint(positions=target); pt.time_from_start=Duration(sec=int(secs),nanosec=int((secs%1)*1e9))
    g.trajectory.points=[pt]
    s=client.send_goal_async(g); rclpy.spin_until_future_complete(node,s)
    rf=s.result().get_result_async(); rclpy.spin_until_future_complete(node,rf)
    code=rf.result().result.error_code
    p,_=cur(); err=max(abs(a-b) for a,b in zip(p,target))
    print(f"attempt {attempt} code={code} max_joint_err={err:.4f}")
    if err<0.01: break
pos,ori=do_fk(); print("hand", pos, ori)
_,d=cur(); print("fingers", round(d["panda_finger_joint1"],4), round(d["panda_finger_joint2"],4))
