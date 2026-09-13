"""MoveIt planning helpers layered on rob.Robot: joint-goal planning and Cartesian straight lines."""
import numpy as np, rclpy
from scipy.spatial.transform import Rotation as Rot
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, RobotState
from moveit_msgs.srv import GetCartesianPath
from geometry_msgs.msg import Pose
from builtin_interfaces.msg import Duration
from rob import *

VMAX = 0.15   # rad/s the sim controller tracks reliably

def retime(traj, vmax=VMAX):
    """Rescale a JointTrajectory so no joint exceeds vmax; returns list of (positions, t)."""
    pts = [(list(p.positions), p.time_from_start.sec + p.time_from_start.nanosec*1e-9) for p in traj.points]
    out, t = [], 0.0
    prev = None
    for q, _ in pts:
        if prev is not None:
            t += max(np.abs(np.array(q)-np.array(prev)).max()/vmax, 0.05)
        out.append((q, t)); prev = q
    return out

class Planner(Robot):
    def __init__(self, name="planner"):
        super().__init__(name)
        self.mg = ActionClient(self.node, MoveGroup, "/move_action")
        self.cart = self.node.create_client(GetCartesianPath, "/compute_cartesian_path")
    def _state_from(self, q):
        rs = RobotState(); rs.joint_state.name = list(JOINTS); rs.joint_state.position = [float(v) for v in q]
        return rs
    def _start_state(self):
        rs = RobotState(); rs.joint_state.name = list(JOINTS); rs.joint_state.position = [float(v) for v in self.joints()]
        return rs
    def plan_joints(self, q_goal, attempts=3, time=10.0):
        self.mg.wait_for_server(timeout_sec=10)
        g = MoveGroup.Goal(); r = g.request
        r.group_name = M["planning"]["group"]; r.num_planning_attempts = attempts; r.allowed_planning_time = time
        r.max_velocity_scaling_factor = 0.1; r.max_acceleration_scaling_factor = 0.1
        r.start_state = self._start_state()
        c = Constraints()
        for j, v in zip(JOINTS, q_goal):
            c.joint_constraints.append(JointConstraint(joint_name=j, position=float(v), tolerance_above=0.01, tolerance_below=0.01, weight=1.0))
        r.goal_constraints = [c]
        g.planning_options.plan_only = True
        f = self.mg.send_goal_async(g); rclpy.spin_until_future_complete(self.node, f)
        rf = f.result().get_result_async(); rclpy.spin_until_future_complete(self.node, rf)
        res = rf.result().result
        if res.error_code.val != 1:
            print(f"  plan FAILED code={res.error_code.val}"); return None
        traj = res.planned_trajectory.joint_trajectory
        print(f"  plan ok: {len(traj.points)} pts")
        return traj
    def execute(self, traj):
        pts = retime(traj)
        q, t = pts[-1]
        return self.move_joints(q, t, via=pts[:-1])
    def goto_joints(self, q_goal):
        traj = self.plan_joints(q_goal)
        if traj is None: return False
        code, err = self.execute(traj)
        return err < 0.05
    def cartesian(self, hand_targets, avoid=True, step=0.01, start_q=None):
        """hand_targets: list of (hand_pos_world, R). Returns (traj, fraction)."""
        self.cart.wait_for_service(timeout_sec=10)
        req = GetCartesianPath.Request()
        req.header.frame_id = "world"; req.group_name = M["planning"]["group"]; req.link_name = "panda_hand"
        req.start_state = self._start_state() if start_q is None else self._state_from(start_q); req.max_step = step; req.jump_threshold = 0.0
        req.avoid_collisions = avoid; req.max_velocity_scaling_factor = 0.1; req.max_acceleration_scaling_factor = 0.1
        for pos, R in hand_targets:
            p = Pose(); p.position.x, p.position.y, p.position.z = map(float, pos)
            q = Rot.from_matrix(R).as_quat(); p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q)
            req.waypoints.append(p)
        res = self._call(self.cart, req)
        if res is None: print("  cartesian: no answer"); return None, 0.0
        print(f"  cartesian: fraction={res.fraction:.2f} code={res.error_code.val} pts={len(res.solution.joint_trajectory.points)}")
        return res.solution.joint_trajectory, res.fraction
    def move_line_tcp(self, tcp_targets, R, avoid=True, min_fraction=0.99):
        traj, frac = self.cartesian([(hand_pose_from_tcp(t, R), R) for t in tcp_targets], avoid=avoid)
        if traj is None or frac < min_fraction: return False
        code, err = self.execute(traj)
        p, _ = self.tcp(); print("  tcp now", np.round(p,3))
        return err < 0.05
