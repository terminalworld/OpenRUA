#!/usr/bin/env python3
"""Collision-aware planning on top of robot.Robot.

  python3 tools/planner.py scene            # publish the world model (table, microwave, door, mugs)
  python3 tools/planner.py goto x y z qx qy qz qw   # plan (OMPL) to TCP pose and execute
"""
import sys, time
import numpy as np
import rclpy
from moveit_msgs.srv import GetMotionPlan, ApplyPlanningScene, GetCartesianPath
from moveit_msgs.msg import (PlanningScene, CollisionObject, Constraints, JointConstraint,
                             RobotState, AttachedCollisionObject, PositionConstraint, OrientationConstraint)
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose, PoseStamped
from control_msgs.action import FollowJointTrajectory

sys.path.insert(0, "/workspace/tools")
from robot import Robot, JOINTS, TCP, quat_to_mat, mat_to_quat, M

TABLE_Z = 0.90


def box(id_, lo, hi, frame="world"):
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    co = CollisionObject()
    co.header.frame_id = frame
    co.id = id_
    p = SolidPrimitive(); p.type = SolidPrimitive.BOX
    p.dimensions = list(hi - lo)
    pose = Pose()
    c = (lo + hi) / 2
    pose.position.x, pose.position.y, pose.position.z = map(float, c)
    pose.orientation.w = 1.0
    co.primitives.append(p); co.primitive_poses.append(pose)
    co.operation = CollisionObject.ADD
    return co


def cylinder(id_, cx, cy, r, z0, z1, frame="world"):
    co = CollisionObject()
    co.header.frame_id = frame
    co.id = id_
    p = SolidPrimitive(); p.type = SolidPrimitive.CYLINDER
    p.dimensions = [float(z1 - z0), float(r)]
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = float(cx), float(cy), float((z0 + z1) / 2)
    pose.orientation.w = 1.0
    co.primitives.append(p); co.primitive_poses.append(pose)
    co.operation = CollisionObject.ADD
    return co


def remove(id_):
    co = CollisionObject(); co.id = id_; co.header.frame_id = "world"
    co.operation = CollisionObject.REMOVE
    return co


# ---- world model (measured from camera clouds)
MW_LO = np.array([-0.295, -0.315, TABLE_Z])
MW_HI = np.array([0.050, -0.115, 1.110])
DOOR_LO = np.array([-0.320, -0.590, TABLE_Z])
DOOR_HI = np.array([-0.292, -0.290, 1.110])
HANDLE_LO = np.array([-0.360, -0.585, TABLE_Z + 0.02])
HANDLE_HI = np.array([-0.320, -0.525, 1.090])
GRAY_MUG = (0.0, 0.345, 0.065)
YELLOW_MUG = (0.0, 0.03, 0.055)


def scene_objects(microwave="solid", yellow=True, door=True):
    objs = [box("table", [-0.55, -0.65, TABLE_Z - 0.06], [0.50, 0.65, TABLE_Z])]
    if microwave == "solid":
        objs.append(box("mw", MW_LO, MW_HI))
    elif microwave == "walls":
        # hollow: opening on the -y face. wall thickness ~2 cm
        t = 0.02
        objs.append(box("mw_top", [MW_LO[0], MW_LO[1], MW_HI[2] - t], MW_HI))
        objs.append(box("mw_bottom", MW_LO, [MW_HI[0], MW_HI[1], MW_LO[2] + t]))
        objs.append(box("mw_back", [MW_LO[0], MW_HI[1] - t, MW_LO[2]], MW_HI))          # +y wall
        objs.append(box("mw_left", MW_LO, [MW_LO[0] + t, MW_HI[1], MW_HI[2]]))            # -x wall
        objs.append(box("mw_right", [MW_HI[0] - 0.08, MW_LO[1], MW_LO[2]], MW_HI))      # +x wall incl. control panel
    if door:
        objs.append(box("door", DOOR_LO, DOOR_HI))
        objs.append(box("handle", HANDLE_LO, HANDLE_HI))
    objs.append(cylinder("gray_mug", GRAY_MUG[0], GRAY_MUG[1], GRAY_MUG[2], TABLE_Z, 1.02))
    if yellow:
        objs.append(cylinder("yellow_mug", YELLOW_MUG[0], YELLOW_MUG[1], YELLOW_MUG[2], TABLE_Z, 1.01))
    return objs


ALL_IDS = ["table", "mw", "mw_top", "mw_bottom", "mw_back", "mw_left", "mw_right", "door", "handle",
           "gray_mug", "yellow_mug"]


class Planner(Robot):
    MAX_VEL = 0.15   # rad/s average per joint over a trajectory
    def __init__(self):
        super().__init__()
        self.plan_cli = self.create_client(GetMotionPlan, "/plan_kinematic_path")
        self.scene_cli = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.cart_cli = self.create_client(GetCartesianPath, "/compute_cartesian_path")

    def apply_scene(self, objs, remove_ids=(), attached=None):
        ps = PlanningScene(); ps.is_diff = True
        for i in remove_ids:
            ps.world.collision_objects.append(remove(i))
        ps.world.collision_objects.extend(objs)
        if attached is not None:
            ps.robot_state.is_diff = True
            ps.robot_state.attached_collision_objects.extend(attached)
        req = ApplyPlanningScene.Request(); req.scene = ps
        res = self._call(self.scene_cli, req)
        return res.success

    def set_scene(self, **kw):
        return self.apply_scene(scene_objects(**kw), remove_ids=ALL_IDS)

    def attach_mug(self, r=0.055, h=0.105, z_off=0.0):
        """Attach a cylinder (the mug) to panda_hand at the fingers."""
        aco = AttachedCollisionObject()
        aco.link_name = "panda_hand"
        aco.touch_links = ["panda_hand", "panda_leftfinger", "panda_rightfinger"]
        co = CollisionObject(); co.header.frame_id = "panda_hand"; co.id = "held_mug"
        p = SolidPrimitive(); p.type = SolidPrimitive.CYLINDER; p.dimensions = [h, r]
        pose = Pose(); pose.position.z = TCP + z_off
        pose.orientation.w = 1.0
        co.primitives.append(p); co.primitive_poses.append(pose); co.operation = CollisionObject.ADD
        aco.object = co
        return self.apply_scene([], attached=[aco])

    def detach_mug(self):
        aco = AttachedCollisionObject(); aco.link_name = "panda_hand"
        aco.object.id = "held_mug"; aco.object.operation = CollisionObject.REMOVE
        return self.apply_scene([remove("held_mug")], attached=[aco])

    def _start_state(self):
        rs = RobotState()
        rs.joint_state.name = list(JOINTS)
        rs.joint_state.position = [float(v) for v in self.arm_q()]
        return rs

    def plan_joint(self, q_goal, pipeline="ompl", planner_id="RRTConnect", tries=3, time_s=5.0):
        req = GetMotionPlan.Request()
        r = req.motion_plan_request
        r.group_name = M["planning"]["group"]
        r.pipeline_id = pipeline
        r.planner_id = planner_id
        r.num_planning_attempts = 4
        r.allowed_planning_time = time_s
        r.max_velocity_scaling_factor = 0.4
        r.max_acceleration_scaling_factor = 0.4
        r.start_state = self._start_state()
        c = Constraints()
        for n, v in zip(JOINTS, q_goal):
            jc = JointConstraint(); jc.joint_name = n; jc.position = float(v)
            jc.tolerance_above = jc.tolerance_below = 0.005; jc.weight = 1.0
            c.joint_constraints.append(jc)
        r.goal_constraints.append(c)
        for _ in range(tries):
            res = self._call(self.plan_cli, req, timeout=time_s + 30)
            if res is not None and res.motion_plan_response.error_code.val == 1:
                return res.motion_plan_response.trajectory.joint_trajectory
            print("plan failed:", res and res.motion_plan_response.error_code.val, flush=True)
        return None

    def cartesian(self, waypoints_tcp, quat, step=0.01, jump=5.0, avoid=True):
        """Straight-line Cartesian path through TCP waypoints (world). Returns (traj, fraction)."""
        R = quat_to_mat(quat)
        req = GetCartesianPath.Request()
        req.header.frame_id = "world"
        req.start_state = self._start_state()
        req.group_name = M["planning"]["group"]
        req.link_name = "panda_hand"
        req.max_step = step
        req.jump_threshold = jump
        req.avoid_collisions = avoid
        req.max_velocity_scaling_factor = 0.3
        req.max_acceleration_scaling_factor = 0.3
        for w in waypoints_tcp:
            p = Pose()
            hp = np.asarray(w, float) - R @ np.array([0, 0, TCP])
            p.position.x, p.position.y, p.position.z = map(float, hp)
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
            req.waypoints.append(p)
        res = self._call(self.cart_cli, req, timeout=60)
        if res is None or res.error_code.val != 1:
            print("cartesian failed:", res and res.error_code.val, flush=True)
            return None, 0.0
        return res.solution.joint_trajectory, res.fraction

    def cartesian_poses(self, poses, step=0.005, jump=5.0, avoid=False):
        """Cartesian path through (tcp_pos, quat) keyframes; orientation is slerped between them."""
        req = GetCartesianPath.Request()
        req.header.frame_id = "world"
        req.start_state = self._start_state()
        req.group_name = M["planning"]["group"]
        req.link_name = "panda_hand"
        req.max_step = step
        req.jump_threshold = jump
        req.avoid_collisions = avoid
        req.max_velocity_scaling_factor = 0.3
        req.max_acceleration_scaling_factor = 0.3
        for pos, quat in poses:
            R = quat_to_mat(quat)
            p = Pose()
            hp = np.asarray(pos, float) - R @ np.array([0, 0, TCP])
            p.position.x, p.position.y, p.position.z = map(float, hp)
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
            req.waypoints.append(p)
        res = self._call(self.cart_cli, req, timeout=60)
        if res is None or res.error_code.val != 1:
            print("cartesian failed:", res and res.error_code.val, flush=True)
            return None, 0.0
        return res.solution.joint_trajectory, res.fraction

    def execute(self, jt, time_scale=3.0, min_time=3.0):
        """Send a planned JointTrajectory to the controller (positions only, rescaled time)."""
        names = list(jt.joint_names)
        idx = [names.index(j) for j in JOINTS]
        pts = []
        for p in jt.points:
            t = (p.time_from_start.sec + p.time_from_start.nanosec * 1e-9) * time_scale
            pts.append((t, [p.positions[i] for i in idx]))
        # the controller tracks at most ~0.3 rad/s: stretch so no joint averages > MAX_VEL
        Q = np.array([q for _, q in pts])
        travel = np.abs(np.diff(Q, axis=0)).sum(0).max() if len(Q) > 1 else 0.0
        need = max(min_time, travel / self.MAX_VEL)
        if pts[-1][0] < need and pts[-1][0] > 0:
            k = need / pts[-1][0]
            pts = [(t * k, q) for t, q in pts]
        total = pts[-1][0]
        # drop the zero-time first point if it equals the current state
        qs = [q for _, q in pts]
        # robot.move spaces points evenly; build the goal ourselves for exact timing
        from trajectory_msgs.msg import JointTrajectoryPoint
        from builtin_interfaces.msg import Duration
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        for i, (t, q) in enumerate(pts):
            if i == 0 and t == 0.0:
                continue
            pt = JointTrajectoryPoint(); pt.positions = [float(v) for v in q]
            if i == len(pts) - 1:
                pt.velocities = [0.0] * 7
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t - int(t)) * 1e9))
            goal.trajectory.points.append(pt)
        # hold point: give the lagging controller time to settle before the goal is judged
        last = goal.trajectory.points[-1]
        hold = JointTrajectoryPoint(); hold.positions = list(last.positions); hold.velocities = [0.0] * 7
        th = pts[-1][0] + 3.0
        hold.time_from_start = Duration(sec=int(th), nanosec=int((th - int(th)) * 1e9))
        goal.trajectory.points.append(hold)
        while not self.traj_cli.wait_for_server(timeout_sec=1.0):
            pass
        fut = self.traj_cli.send_goal_async(goal)
        while not fut.done():
            rclpy.spin_once(self, timeout_sec=0.1)
        gh = fut.result()
        rf = gh.get_result_async()
        while not rf.done():
            rclpy.spin_once(self, timeout_sec=0.1)
        res = rf.result().result
        err = np.abs(self.arm_q() - np.array(qs[-1])).max()
        print(f"exec done: error_code={res.error_code} ({res.error_string}) pts={len(goal.trajectory.points)} T={total:.1f}s max_joint_err={err:.4f}", flush=True)
        for _ in range(2):
            if err <= 0.02:
                break
            self.move(qs[-1], 3.0)  # converge with a resend of the final point
            err = np.abs(self.arm_q() - np.array(qs[-1])).max()
        return res.error_code, err

    def goto_q(self, q, **kw):
        jt = self.plan_joint(q, **kw)
        if jt is None:
            return False
        code, err = self.execute(jt)
        return err < 0.02

    def goto_tcp(self, pos, quat, seed=None, **kw):
        q = self.ik(pos, quat, seed=seed, attempts=10)
        if q is None:
            print("IK failed for", pos, flush=True)
            return False
        ok = self.goto_q(q, **kw)
        tp = self.tcp()[0]
        print("tcp now", np.round(tp, 4), "target", np.round(pos, 4), "ok", ok, flush=True)
        return ok

    def line(self, pos_to, quat=None, step=0.01, avoid=True, min_fraction=0.95, time_scale=1.5):
        """Straight-line TCP move from the current pose to pos_to keeping orientation."""
        p0, q0, _ = self.tcp()
        if quat is None:
            quat = q0
        jt, frac = self.cartesian([pos_to], quat, step=step, avoid=avoid)
        if jt is None or frac < min_fraction:
            print(f"line: fraction {frac:.2f} < {min_fraction}", flush=True)
            return False
        self.execute(jt, time_scale=time_scale)
        tp = self.tcp()[0]
        print("tcp now", np.round(tp, 4), "target", np.round(pos_to, 4), flush=True)
        return np.linalg.norm(tp - pos_to) < 0.01


def main():
    rclpy.init()
    p = Planner()
    cmd = sys.argv[1]
    if cmd == "scene":
        print("scene ok:", p.set_scene())
    elif cmd == "goto":
        v = [float(x) for x in sys.argv[2:9]]
        p.goto_tcp(v[:3], v[3:])
    elif cmd == "line":
        v = [float(x) for x in sys.argv[2:5]]
        p.line(np.array(v))
    p.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
