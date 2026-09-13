"""Reusable arm control: FK, IK, trajectory, gripper, joint state. World<->base conversion."""
import sys, time, numpy as np, rclpy, yaml
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from sensor_msgs.msg import JointState

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
JOINTS = FJT["joints"]
BASE_IN_WORLD = np.array([0.0, 0.0, 0.0])  # MoveIt model frame IS world here (verified via /compute_fk: panda_link0 at (-0.51,0,0.42))
TCP = M["hand"]["tcp_offset_m"]

def quat_to_R(x, y, z, w):
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])

def topdown_quat(yaw=0.0):
    """Hand z down; fingers along base y when yaw=0; yaw rotates about world z."""
    # q = qz(yaw) * (1,0,0,0)
    c, s = np.cos(yaw/2), np.sin(yaw/2)
    # (0,0,s,c) * (1,0,0,0): quaternion product (w1 w2 - v1.v2, w1 v2 + w2 v1 + v1 x v2)
    w1, v1 = c, np.array([0, 0, s]); w2, v2 = 0.0, np.array([1.0, 0, 0])
    w = w1*w2 - v1 @ v2; v = w1*v2 + w2*v1 + np.cross(v1, v2)
    return (v[0], v[1], v[2], w)

class Robot:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("arm_ctl")
        self.js = {}
        self.node.create_subscription(JointState, "/joint_states", self._js_cb, 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt.wait_for_server(10); self.grip.wait_for_server(10)
        self.ik.wait_for_service(10); self.fk.wait_for_service(10)
    def _js_cb(self, m): self.js = dict(zip(m.name, m.position))
    def spin(self, t=0.2): rclpy.spin_once(self.node, timeout_sec=t)
    def joints(self):
        self.js = {}
        while not self.js: self.spin()
        return dict(self.js)
    def arm_q(self):
        j = self.joints(); return [j[n] for n in JOINTS]
    def fingers(self):
        j = self.joints(); return j.get("panda_finger_joint1"), j.get("panda_finger_joint2")
    def _call(self, cli, req, timeout=60):
        f = cli.call_async(req); rclpy.spin_until_future_complete(self.node, f, timeout_sec=timeout); return f.result()
    def fk_world(self, q=None):
        req = GetPositionFK.Request(); req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = JOINTS
        req.robot_state.joint_state.position = q or self.arm_q()
        res = self._call(self.fk, req)
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE_IN_WORLD
        return pos, (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w), res.error_code.val
    def ik_world(self, pos_w, quat, at_tcp=False, seed=None):
        pos = np.array(pos_w, float)
        if at_tcp: pos = pos - TCP * quat_to_R(*quat)[:, 2]
        pos = pos - BASE_IN_WORLD
        req = GetPositionIK.Request(); r = req.ik_request
        r.group_name = M["planning"]["group"]; r.ik_link_name = "panda_hand"; r.pose_stamped.header.frame_id = ""
        r.pose_stamped.pose.position.x, r.pose_stamped.pose.position.y, r.pose_stamped.pose.position.z = pos
        r.pose_stamped.pose.orientation.x, r.pose_stamped.pose.orientation.y, r.pose_stamped.pose.orientation.z, r.pose_stamped.pose.orientation.w = quat
        r.robot_state.joint_state.name = JOINTS; r.robot_state.joint_state.position = seed or self.arm_q()
        r.avoid_collisions = False
        r.timeout.sec = 5
        res = self._call(self.ik, req)
        if res is None or res.error_code.val != 1:
            return None, (res.error_code.val if res else "timeout")
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in JOINTS], 1
    def move_q(self, q, seconds=3.0, via=None):
        goal = FollowJointTrajectory.Goal(); goal.trajectory.joint_names = JOINTS
        pts = []
        if via:
            for i, vq in enumerate(via):
                t = seconds * (i+1) / (len(via)+1)
                p = JointTrajectoryPoint(positions=list(map(float, vq))); p.time_from_start = Duration(sec=int(t), nanosec=int((t%1)*1e9)); pts.append(p)
        p = JointTrajectoryPoint(positions=list(map(float, q))); p.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds%1)*1e9)); pts.append(p)
        goal.trajectory.points = pts
        f = self.fjt.send_goal_async(goal); rclpy.spin_until_future_complete(self.node, f)
        rf = f.result().get_result_async(); rclpy.spin_until_future_complete(self.node, rf)
        code = rf.result().result.error_code
        err = np.abs(np.array(self.arm_q()) - np.array(q)).max()
        print(f"  move: error_code={code} max joint err={err:.4f}", flush=True)
        return code, err
    def move_world(self, pos_w, quat, seconds=3.0, at_tcp=True):
        q, code = self.ik_world(pos_w, quat, at_tcp=at_tcp)
        if q is None:
            print(f"  IK FAILED {code} for {pos_w}", flush=True); return False
        self.move_q(q, seconds)
        p, _, _ = self.fk_world()
        tcp = p + TCP * quat_to_R(*quat)[:, 2] if at_tcp else p
        print(f"  now hand={np.round(p,4)} tcp={np.round(tcp,4)} target={np.round(pos_w,4)}", flush=True)
        return True
    def gripper(self, width):
        g = GripperCommand.Goal(); g.command.position = float(width); g.command.max_effort = float(GRIP["max_effort"])
        f = self.grip.send_goal_async(g); rclpy.spin_until_future_complete(self.node, f, timeout_sec=30)
        rf = f.result().get_result_async(); rclpy.spin_until_future_complete(self.node, rf, timeout_sec=120)
        r = rf.result().result
        print(f"  gripper({width}): reached={r.reached_goal} stalled={r.stalled} pos={r.position:.4f} fingers={self.fingers()}", flush=True)
        return r
