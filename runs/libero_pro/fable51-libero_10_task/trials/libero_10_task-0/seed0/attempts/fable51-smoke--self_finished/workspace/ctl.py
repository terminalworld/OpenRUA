#!/usr/bin/env python3
"""Controller helpers: goto (IK->FJT), grip, pose, joints. World-frame inputs.
world->panda_link0 = (-0.51, 0, 0.42), identity rotation (from /tf_static)."""
import sys, math, time
import numpy as np, rclpy
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from sensor_msgs.msg import JointState

ARM=[f'panda_joint{i}' for i in range(1,8)]
BASE=np.array([0.0,0.0,0.0])  # MoveIt model frame == world here (verified by FK)
TCP=0.1034

class Ctl:
    def __init__(s):
        rclpy.init(); s.n=rclpy.create_node('ctl')
        s.js={}
        s.n.create_subscription(JointState,'/joint_states',s._js,1)
        s.fjt=ActionClient(s.n,FollowJointTrajectory,'/panda_arm_controller/follow_joint_trajectory')
        s.grp=ActionClient(s.n,GripperCommand,'/franka_gripper/gripper_action')
        s.ikc=s.n.create_client(GetPositionIK,'/compute_ik')
        s.fkc=s.n.create_client(GetPositionFK,'/compute_fk')
        assert s.fjt.wait_for_server(10) and s.grp.wait_for_server(10)
        assert s.ikc.wait_for_service(10) and s.fkc.wait_for_service(10)
    def _js(s,m):
        s.js=dict(zip(m.name,m.position))
    def joints(s):
        s.js={}
        while not s.js: rclpy.spin_once(s.n,timeout_sec=0.2)
        return dict(s.js)
    def arm_state(s):
        j=s.joints(); st=JointState(); st.name=ARM; st.position=[j[a] for a in ARM]; return st,j
    def pose(s):
        st,j=s.arm_state()
        r=GetPositionFK.Request(); r.fk_link_names=['panda_hand']; r.robot_state.joint_state=st
        f=s.fkc.call_async(r); rclpy.spin_until_future_complete(s.n,f,timeout_sec=30)
        p=f.result().pose_stamped[0].pose
        w=np.array([p.position.x,p.position.y,p.position.z])+BASE
        q=(p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w)
        R=quatR(*q); tcp=w+TCP*R[:,2]
        return w,q,tcp,j
    def ik(s,xw,yw,zw,yaw,tcp=True):
        # hand z down; fingers along world y when yaw=0
        q=(math.cos(yaw/2),math.sin(yaw/2),0.0,0.0)
        p=np.array([xw,yw,zw])-BASE
        if tcp: p=p-TCP*quatR(*q)[:,2]
        st,_=s.arm_state()
        r=GetPositionIK.Request(); r.ik_request.group_name='panda_arm'; r.ik_request.ik_link_name='panda_hand'
        r.ik_request.pose_stamped.header.frame_id=''
        pp=r.ik_request.pose_stamped.pose
        pp.position.x,pp.position.y,pp.position.z=map(float,p)
        pp.orientation.x,pp.orientation.y,pp.orientation.z,pp.orientation.w=q
        r.ik_request.robot_state.joint_state=st
        r.ik_request.avoid_collisions=False
        f=s.ikc.call_async(r); rclpy.spin_until_future_complete(s.n,f,timeout_sec=60)
        res=f.result()
        if res is None or res.error_code.val!=1:
            raise SystemExit(f'IK failed code={None if res is None else res.error_code.val}')
        sol=dict(zip(res.solution.joint_state.name,res.solution.joint_state.position))
        return [sol[a] for a in ARM]
    def move(s,positions,secs):
        g=FollowJointTrajectory.Goal(); g.trajectory.joint_names=ARM
        pt=JointTrajectoryPoint(positions=[float(x) for x in positions])
        pt.time_from_start=Duration(sec=int(secs),nanosec=int((secs%1)*1e9))
        g.trajectory.points=[pt]
        f=s.fjt.send_goal_async(g); rclpy.spin_until_future_complete(s.n,f)
        rf=f.result().get_result_async(); rclpy.spin_until_future_complete(s.n,rf)
        code=rf.result().result.error_code
        j=s.joints(); err=max(abs(j[a]-p) for a,p in zip(ARM,positions))
        print(f'  move done code={code} max_joint_err={err:.4f}')
        return code,err
    def goto(s,xw,yw,zw,yaw,secs=3.0):
        sol=s.ik(xw,yw,zw,yaw)
        s.move(sol,secs)
        w,q,tcp,_=s.pose()
        print(f'  now hand=({w[0]:.3f},{w[1]:.3f},{w[2]:.3f}) tcp=({tcp[0]:.3f},{tcp[1]:.3f},{tcp[2]:.3f}) target_tcp=({xw:.3f},{yw:.3f},{zw:.3f})')
        return tcp
    def grip(s,width):
        g=GripperCommand.Goal(); g.command.position=float(width); g.command.max_effort=30.0
        f=s.grp.send_goal_async(g); rclpy.spin_until_future_complete(s.n,f,timeout_sec=30)
        rf=f.result().get_result_async(); rclpy.spin_until_future_complete(s.n,rf,timeout_sec=120)
        r=rf.result().result; j=s.joints()
        print(f'  grip reached={r.reached_goal} stalled={r.stalled} fingers=({j["panda_finger_joint1"]:.4f},{j["panda_finger_joint2"]:.4f})')
        return j

def quatR(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

if __name__=='__main__':
    c=Ctl(); cmd=sys.argv[1]
    if cmd=='pose':
        w,q,tcp,j=c.pose(); print('hand',w.round(4),'q',np.round(q,4),'tcp',tcp.round(4)); print('joints',{k:round(v,4) for k,v in j.items()})
    elif cmd=='goto':
        x,y,z,yaw=map(float,sys.argv[2:6]); secs=float(sys.argv[6]) if len(sys.argv)>6 else 3.0
        c.goto(x,y,z,yaw,secs)
    elif cmd=='grip':
        c.grip(float(sys.argv[2]))
    elif cmd=='ikcheck':
        x,y,z,yaw=map(float,sys.argv[2:6]); print(np.round(c.ik(x,y,z,yaw),4))
    rclpy.shutdown()
