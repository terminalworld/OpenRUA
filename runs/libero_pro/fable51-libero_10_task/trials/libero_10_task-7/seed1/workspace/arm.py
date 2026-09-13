"""Arm helper: IK (base frame), trajectory, gripper, state. Import or run as CLI.
CLI: python3 arm.py ik x y z            -> print IK joints for TCP at base-frame xyz, hand pointing down
     python3 arm.py tcp x y z [secs]    -> IK + move TCP (world frame coords!) 
     python3 arm.py grip open|close
     python3 arm.py state
"""
import sys, time, numpy as np, rclpy
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory, GripperCommand
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

ARM=[f"panda_joint{i}" for i in range(1,8)]
TCP=0.1034
# MACHINE FACT (verified): /compute_ik solves for panda_link8 in the WORLD frame (frame_id empty).
# panda_link8 quaternion for hand pointing down with fingers closing along world Y:
DOWN=(0.9239,-0.3827,0.0,0.0)
# fingers closing along world X (hand yawed 90 deg):
DOWN_X=(0.3827,-0.9239,0.0,0.0)
def yaw_quat(theta_deg):
    """link8 quaternion: hand down, closing axis rotated theta from world Y about world Z"""
    import math
    s,c=math.sin(math.radians(theta_deg)/2),math.cos(math.radians(theta_deg)/2)
    x2,y2=DOWN[0],DOWN[1]
    return (c*x2-s*y2, c*y2+s*x2, 0.0, 0.0)

class Arm:
    def __init__(self):
        if not rclpy.ok(): rclpy.init()
        self.node=rclpy.create_node("arm_helper")
        self.js={}
        self.node.create_subscription(JointState,"/joint_states",self._js,1)
        self.ik=self.node.create_client(GetPositionIK,"/compute_ik")
        self.fk=self.node.create_client(GetPositionFK,"/compute_fk")
        self.traj=ActionClient(self.node,FollowJointTrajectory,"/panda_arm_controller/follow_joint_trajectory")
        self.grip=ActionClient(self.node,GripperCommand,"/franka_gripper/gripper_action")
        self.ik.wait_for_service(); self.traj.wait_for_server(); self.grip.wait_for_server()
    def _js(self,m): self.js=dict(zip(m.name,m.position))
    def state(self):
        self.js={}
        while not self.js: rclpy.spin_once(self.node,timeout_sec=0.2)
        return dict(self.js)
    def joints(self):
        s=self.state(); return [s[a] for a in ARM]
    def fingers(self):
        s=self.state(); return s["panda_finger_joint1"], s["panda_finger_joint2"]
    def hand_pose_world(self):
        req=GetPositionFK.Request(); req.fk_link_names=["panda_hand"]
        req.robot_state.joint_state.name=ARM; req.robot_state.joint_state.position=self.joints()
        f=self.fk.call_async(req); rclpy.spin_until_future_complete(self.node,f,timeout_sec=60)
        p=f.result().pose_stamped[0].pose
        return np.array([p.position.x,p.position.y,p.position.z]), (p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w)
    def solve_ik_base(self, pos_base, quat=DOWN, seed=None, at_tcp=True):
        x,y,z=pos_base
        if at_tcp:
            R=_qR(*quat); x,y,z=np.array([x,y,z])-TCP*R[:,2]
        req=GetPositionIK.Request(); r=req.ik_request
        r.group_name="panda_arm"; r.pose_stamped.header.frame_id=""
        r.pose_stamped.pose.position.x=float(x); r.pose_stamped.pose.position.y=float(y); r.pose_stamped.pose.position.z=float(z)
        r.pose_stamped.pose.orientation.x,r.pose_stamped.pose.orientation.y,r.pose_stamped.pose.orientation.z,r.pose_stamped.pose.orientation.w=[float(q) for q in quat]
        r.robot_state.joint_state.name=ARM; r.robot_state.joint_state.position=[float(v) for v in (seed or self.joints())]
        r.avoid_collisions=False
        r.timeout=Duration(sec=2)
        f=self.ik.call_async(req); rclpy.spin_until_future_complete(self.node,f,timeout_sec=120)
        res=f.result()
        if res is None: raise RuntimeError("IK no answer")
        if res.error_code.val!=1: raise RuntimeError(f"IK failed code={res.error_code.val}")
        sol=dict(zip(res.solution.joint_state.name,res.solution.joint_state.position))
        return [sol[a] for a in ARM]
    def solve_ik_world(self, pos_world, quat=DOWN, **kw):
        return self.solve_ik_base(np.array(pos_world), quat, **kw)
    def move(self, joints, secs=3.0, wait_timeout=600):
        g=FollowJointTrajectory.Goal(); g.trajectory.joint_names=ARM
        pt=JointTrajectoryPoint(positions=[float(v) for v in joints]); pt.time_from_start=Duration(sec=int(secs),nanosec=int((secs%1)*1e9))
        g.trajectory.points=[pt]
        f=self.traj.send_goal_async(g); rclpy.spin_until_future_complete(self.node,f,timeout_sec=120)
        rf=f.result().get_result_async(); rclpy.spin_until_future_complete(self.node,rf,timeout_sec=wait_timeout)
        code=rf.result().result.error_code
        cur=self.joints(); err=max(abs(a-b) for a,b in zip(cur,joints))
        print(f"move done code={code} max_joint_err={err:.4f}", flush=True)
        return code, err
    def move_tcp_world(self, pos, quat=DOWN, secs=3.0, max_delta=1.0, tries=5):
        cur=self.joints()
        for k in range(tries):
            j=self.solve_ik_world(pos,quat,seed=cur)
            d=max(abs(a-b) for a,b in zip(j,cur))
            if d<=max_delta: break
            print(f"IK branch far from current (max delta {d:.2f}); retrying", flush=True)
        else:
            raise RuntimeError(f"no nearby IK branch (delta {d:.2f})")
        return self.move(j,secs)
    def go(self, pos, quat=DOWN, secs=3.0, retries=3):
        """move_tcp_world with re-send on tracking lag; returns final hand pose"""
        for i in range(retries):
            code,err=self.move_tcp_world(pos,quat,secs)
            if code==0 and err<0.01: break
        p,q=self.hand_pose_world(); print("hand",p.round(4),"tcp z",round(p[2]-TCP,4),np.round(q,3),flush=True)
        return p,q
    def gripper(self, width, timeout=300):
        g=GripperCommand.Goal(); g.command.position=float(width); g.command.max_effort=30.0
        f=self.grip.send_goal_async(g); rclpy.spin_until_future_complete(self.node,f,timeout_sec=120)
        rf=f.result().get_result_async(); rclpy.spin_until_future_complete(self.node,rf,timeout_sec=timeout)
        r=rf.result().result
        print(f"gripper reached={r.reached_goal} stalled={r.stalled} fingers={self.fingers()}", flush=True)
        return r

def _qR(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

if __name__=="__main__":
    a=Arm(); cmd=sys.argv[1]
    if cmd=="state":
        print(a.state()); p,q=a.hand_pose_world(); print("hand world",p.round(4),np.round(q,4))
    elif cmd=="ik":
        print(a.solve_ik_base([float(v) for v in sys.argv[2:5]]))
    elif cmd=="tcp":
        secs=float(sys.argv[5]) if len(sys.argv)>5 else 3.0
        a.move_tcp_world([float(v) for v in sys.argv[2:5]],secs=secs); p,q=a.hand_pose_world(); print("hand world",p.round(4))
    elif cmd=="grip":
        a.gripper(0.04 if sys.argv[2]=="open" else 0.0)
