"""MoveIt planning helpers: collision objects + plan_kinematic_path + execute via FJT."""
import numpy as np, rclpy
from ctl import *
from moveit_msgs.srv import GetMotionPlan, ApplyPlanningScene
from moveit_msgs.msg import CollisionObject, PlanningScene, Constraints, JointConstraint, PositionConstraint, OrientationConstraint, BoundingVolume
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose, PoseStamped
from control_msgs.action import FollowJointTrajectory

def box(name, lo, hi):
    lo=np.array(lo,float); hi=np.array(hi,float)
    co=CollisionObject(); co.header.frame_id="world"; co.id=name
    sp=SolidPrimitive(); sp.type=SolidPrimitive.BOX; sp.dimensions=list(map(float,hi-lo))
    p=Pose(); p.position.x,p.position.y,p.position.z=map(float,(lo+hi)/2); p.orientation.w=1.0
    co.primitives=[sp]; co.primitive_poses=[p]; co.operation=CollisionObject.ADD
    return co

class Planner:
    def __init__(self, r):
        self.r=r; n=r.node
        self.aps=n.create_client(ApplyPlanningScene,"/apply_planning_scene"); self.aps.wait_for_service(10)
        self.gmp=n.create_client(GetMotionPlan,"/plan_kinematic_path"); self.gmp.wait_for_service(10)
    def scene(self, objs, remove=()):
        ps=PlanningScene(); ps.is_diff=True
        for o in objs: ps.world.collision_objects.append(o)
        for name in remove:
            co=CollisionObject(); co.header.frame_id="world"; co.id=name; co.operation=CollisionObject.REMOVE
            ps.world.collision_objects.append(co)
        req=ApplyPlanningScene.Request(); req.scene=ps
        f=self.aps.call_async(req); rclpy.spin_until_future_complete(self.r.node,f,timeout_sec=30)
        return f.result().success
    def _req(self, t=5.0, attempts=10):
        req=GetMotionPlan.Request(); mr=req.motion_plan_request
        mr.group_name=M["planning"]["group"]; mr.allowed_planning_time=t; mr.num_planning_attempts=attempts
        mr.pipeline_id="ompl"; mr.planner_id="RRTConnectkConfigDefault"
        mr.max_velocity_scaling_factor=0.3; mr.max_acceleration_scaling_factor=0.3
        f1,f2=self.r.finger()  # bridge reports finger2 negative; MoveIt wants both >=0
        mr.start_state.joint_state.name=list(ARM)+["panda_finger_joint1","panda_finger_joint2"]
        mr.start_state.joint_state.position=list(map(float,self.r.arm_q()))+[abs(f1),abs(f2)]
        mr.start_state.is_diff=True
        return req
    def plan_joints(self, q, **kw):
        req=self._req(**kw); c=Constraints()
        for n_,v in zip(ARM,q):
            jc=JointConstraint(); jc.joint_name=n_; jc.position=float(v); jc.tolerance_above=0.01; jc.tolerance_below=0.01; jc.weight=1.0
            c.joint_constraints.append(jc)
        req.motion_plan_request.goal_constraints=[c]; return self._call(req)
    def plan_pose(self, pos, quat, **kw):
        req=self._req(**kw); c=Constraints()
        pc=PositionConstraint(); pc.header.frame_id="world"; pc.link_name="panda_hand"; pc.weight=1.0
        sp=SolidPrimitive(); sp.type=SolidPrimitive.SPHERE; sp.dimensions=[0.005]
        p=Pose(); p.position.x,p.position.y,p.position.z=map(float,pos); p.orientation.w=1.0
        pc.constraint_region.primitives=[sp]; pc.constraint_region.primitive_poses=[p]
        oc=OrientationConstraint(); oc.header.frame_id="world"; oc.link_name="panda_hand"; oc.weight=1.0
        oc.orientation.x,oc.orientation.y,oc.orientation.z,oc.orientation.w=map(float,quat)
        oc.absolute_x_axis_tolerance=oc.absolute_y_axis_tolerance=oc.absolute_z_axis_tolerance=0.02
        c.position_constraints=[pc]; c.orientation_constraints=[oc]
        req.motion_plan_request.goal_constraints=[c]; return self._call(req)
    def _call(self, req):
        f=self.gmp.call_async(req); rclpy.spin_until_future_complete(self.r.node,f,timeout_sec=120)
        res=f.result().motion_plan_response
        if res.error_code.val!=1: print("  plan failed code",res.error_code.val); return None
        tr=res.trajectory.joint_trajectory
        print(f"  plan ok: {len(tr.points)} pts, {tr.points[-1].time_from_start.sec + tr.points[-1].time_from_start.nanosec*1e-9:.1f}s, final {np.round(tr.points[-1].positions,2)}")
        return tr
    def execute_steps(self, tr, stride=4, seconds=1.0):
        """follow a planned path as a series of single-point goals (tracks the path, no lag pile-up)."""
        idx=[tr.joint_names.index(j) for j in ARM]
        pts=[[p.positions[i] for i in idx] for p in tr.points]
        sel=pts[stride::stride]
        if not sel or sel[-1] is not pts[-1]: sel.append(pts[-1])
        prev=np.array(self.r.arm_q())
        for k,q in enumerate(sel):
            d=np.abs(np.array(q)-prev).max(); prev=np.array(q)
            self.r.move_joints(q, seconds=max(seconds, d/0.4) if k<len(sel)-1 else 2.0, retries=0 if k<len(sel)-1 else 2)
        err=np.abs(np.array(self.r.arm_q())-np.array(pts[-1])).max(); print(f"  steps done err={err:.4f}"); return err<0.02
    def execute(self, tr, retries=2):
        idx=[tr.joint_names.index(j) for j in ARM]
        goal=FollowJointTrajectory.Goal(); goal.trajectory.joint_names=list(ARM)
        for p in tr.points:
            pt=JointTrajectoryPoint(positions=[p.positions[i] for i in idx],velocities=[p.velocities[i] for i in idx] if p.velocities else [])
            pt.time_from_start=p.time_from_start; goal.trajectory.points.append(pt)
        send=self.r.fjt.send_goal_async(goal); rclpy.spin_until_future_complete(self.r.node,send)
        res=send.result().get_result_async(); rclpy.spin_until_future_complete(self.r.node,res)
        code=res.result().result.error_code
        target=[tr.points[-1].positions[i] for i in idx]
        err=np.abs(np.array(self.r.arm_q())-np.array(target)).max(); print(f"  exec: code={code} err={err:.4f}")
        if err>0.02: self.r.move_joints(target,seconds=3,retries=retries)
        return True

def scene_objects(with_mug=True):
    objs=[box("table",[-1.4,-1.0,0.80],[0.6,1.0,0.88]),
          box("caddy",[-0.472,-0.345,0.88],[-0.318,0.06,1.06])]
    if with_mug:
        objs += [box("mug_body",[-0.575,-0.139,0.942],[-0.485,-0.066,1.014]),
                 box("mug_rim",[-0.60,-0.16,0.925],[-0.575,-0.045,1.03]),
                 box("mug_handle",[-0.572,-0.12,0.88],[-0.521,-0.085,0.945])]
    return objs

def cyl(name, cx, cy, z0, z1, rad):
    co=CollisionObject(); co.header.frame_id="world"; co.id=name
    sp=SolidPrimitive(); sp.type=SolidPrimitive.CYLINDER; sp.dimensions=[float(z1-z0), float(rad)]
    p=Pose(); p.position.x,p.position.y,p.position.z=float(cx),float(cy),float((z0+z1)/2); p.orientation.w=1.0
    co.primitives=[sp]; co.primitive_poses=[p]; co.operation=CollisionObject.ADD
    return co

MUG_C=(-0.548,0.056)
def mug_objects(cx=MUG_C[0], cy=MUG_C[1]):
    return [cyl("mug_body",cx,cy,0.88,0.995,0.052),
            box("mug_handle",[cx+0.03,cy-0.012,0.90],[cx+0.078,cy+0.012,0.97])]

def ring_objects(cx, cy, z0, z1, rad=0.05, n=12, skip_angle=None, name="mugw"):
    """hollow mug wall as n chord boxes; omit segment nearest skip_angle (rad)."""
    objs=[]
    for k in range(n):
        a=2*np.pi*k/n
        if skip_angle is not None and abs((a-skip_angle+np.pi)%(2*np.pi)-np.pi)<np.pi/n*0.99: continue
        co=CollisionObject(); co.header.frame_id="world"; co.id=f"{name}{k}"
        sp=SolidPrimitive(); sp.type=SolidPrimitive.BOX; sp.dimensions=[0.012, float(2*rad*np.tan(np.pi/n)+0.004), float(z1-z0)]
        p=Pose(); p.position.x=float(cx+rad*np.cos(a)); p.position.y=float(cy+rad*np.sin(a)); p.position.z=float((z0+z1)/2)
        qz=np.array([0,0,np.sin(a/2),np.cos(a/2)]); p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=map(float,qz)
        co.primitives=[sp]; co.primitive_poses=[p]; co.operation=CollisionObject.ADD; objs.append(co)
    objs.append(box(name+"_bottom",[cx-0.04,cy-0.04,0.88],[cx+0.04,cy+0.04,0.90]))
    return objs

def attach_mug(pl, hand_off=(0.0,0.04,0.148), rad=0.08, h=0.14, name="held_mug"):
    """Attach a cylinder (held mug, conservative) to panda_hand for planning."""
    from moveit_msgs.msg import AttachedCollisionObject
    aco=AttachedCollisionObject(); aco.link_name="panda_hand"
    co=CollisionObject(); co.header.frame_id="panda_hand"; co.id=name
    sp=SolidPrimitive(); sp.type=SolidPrimitive.CYLINDER; sp.dimensions=[float(h),float(rad)]
    p=Pose(); p.position.x,p.position.y,p.position.z=map(float,hand_off); p.orientation.w=1.0
    co.primitives=[sp]; co.primitive_poses=[p]; co.operation=CollisionObject.ADD
    aco.object=co; aco.touch_links=["panda_hand","panda_leftfinger","panda_rightfinger"]
    ps=PlanningScene(); ps.is_diff=True; ps.robot_state.is_diff=True; ps.robot_state.attached_collision_objects=[aco]
    req=ApplyPlanningScene.Request(); req.scene=ps
    f=pl.aps.call_async(req); rclpy.spin_until_future_complete(pl.r.node,f,timeout_sec=30); return f.result().success
def detach(pl, name="held_mug"):
    from moveit_msgs.msg import AttachedCollisionObject
    aco=AttachedCollisionObject(); aco.link_name="panda_hand"; aco.object.id=name; aco.object.operation=CollisionObject.REMOVE
    ps=PlanningScene(); ps.is_diff=True; ps.robot_state.is_diff=True; ps.robot_state.attached_collision_objects=[aco]
    # also drop the detached copy from the world
    co=CollisionObject(); co.id=name; co.header.frame_id="world"; co.operation=CollisionObject.REMOVE; ps.world.collision_objects=[co]
    req=ApplyPlanningScene.Request(); req.scene=ps
    f=pl.aps.call_async(req); rclpy.spin_until_future_complete(pl.r.node,f,timeout_sec=30); return f.result().success
