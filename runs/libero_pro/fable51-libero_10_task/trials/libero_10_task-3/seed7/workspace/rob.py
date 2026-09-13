"""Reusable helpers: joint state, FK, IK, trajectory, gripper. Import or run functions."""
import math, time, sys
import numpy as np, rclpy, yaml
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import WrenchStamped

M = yaml.safe_load(open("/workspace/machine.yaml"))
ARM = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
JOINTS = ARM["joints"]
BASE = np.array([-0.66, 0.0, 0.912])  # world -> panda_link0

class Robot:
    def __init__(self):
        if not rclpy.ok(): rclpy.init()
        self.node = rclpy.create_node("rob_" + str(int(time.time()*1000) % 100000))
        self._js = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self._wr = None
        self.node.create_subscription(WrenchStamped, "/franka_robot_state_broadcaster/external_wrench", self._on_wr, 10)
        self.ik = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.traj = ActionClient(self.node, FollowJointTrajectory, ARM["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik.wait_for_service(10); self.fk.wait_for_service(10)
        self.traj.wait_for_server(10); self.grip.wait_for_server(10)
    def _on_js(self, m): self._js = m
    def _on_wr(self, m): self._wr = m
    def spin(self, t=0.1):
        end = time.time() + t
        while time.time() < end: rclpy.spin_once(self.node, timeout_sec=0.05)
    def joints(self, fresh=True):
        if fresh:
            self._js = None
            while self._js is None: rclpy.spin_once(self.node, timeout_sec=0.2)
        d = dict(zip(self._js.name, self._js.position))
        return d
    def arm_q(self):
        d = self.joints(); return [d[j] for j in JOINTS]
    def fingers(self):
        d = self.joints(); return d["panda_finger_joint1"], d["panda_finger_joint2"]
    def wrench(self):
        self._wr = None
        while self._wr is None: rclpy.spin_once(self.node, timeout_sec=0.2)
        w = self._wr.wrench
        return np.array([w.force.x, w.force.y, w.force.z, w.torque.x, w.torque.y, w.torque.z])
    def fk_hand(self, q=None, link="panda_hand"):
        """returns (pos_world, quat xyzw) of link for arm config q (default current)."""
        q = q if q is not None else self.arm_q()
        req = GetPositionFK.Request()
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = list(JOINTS)
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {None if res is None else res.error_code.val}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z])
        # planner frame is arm base; convert to world
        pos_w = pos  # service already answers in world coords (verified against camera TF)
        return pos_w, np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]), res.pose_stamped[0].header.frame_id
    def ik_hand(self, pos_world, quat_xyzw, seed=None, attempts=3):
        """IK for hand frame at world pose. Returns joint list or None."""
        pos = np.asarray(pos_world, float)  # world coords (verified: FK/IK operate in world)
        seed = seed if seed is not None else self.arm_q()
        for k in range(attempts):
            req = GetPositionIK.Request()
            req.ik_request.group_name = M["planning"]["group"]
            req.ik_request.ik_link_name = "panda_hand"
            req.ik_request.pose_stamped.header.frame_id = ""
            p = req.ik_request.pose_stamped.pose
            p.position.x, p.position.y, p.position.z = map(float, pos)
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat_xyzw)
            req.ik_request.robot_state.joint_state.name = list(JOINTS)
            req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
            req.ik_request.avoid_collisions = False
            req.ik_request.timeout = Duration(sec=1)
            fut = self.ik.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            res = fut.result()
            if res is not None and res.error_code.val == 1:
                sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
                return [sol[j] for j in JOINTS]
            # perturb seed
            seed = list(np.array(seed) + np.random.uniform(-0.3, 0.3, len(seed)))
        return None
    def move_q(self, q, seconds=3.0, waypoints=None):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        pts = []
        wps = (waypoints or []) + [(q, seconds)]
        for qq, t in wps:
            pt = JointTrajectoryPoint(positions=[float(v) for v in qq])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            pts.append(pt)
        goal.trajectory.points = pts
        send = self.traj.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=600)
        code = rf.result().result.error_code
        qa = np.array(self.arm_q()); err = np.abs(qa - np.array(q)).max()
        return code, err
    def move_q_conv(self, q, seconds=3.0, tol=0.02, retries=4):
        code, err = self.move_q(q, seconds)
        n = 0
        while err > tol and n < retries:
            code, err = self.move_q(q, max(1.5, seconds/2)); n += 1
        return code, err, n
    def move_pose(self, pos_world, quat_xyzw, seconds=3.0, seed=None):
        q = self.ik_hand(pos_world, quat_xyzw, seed=seed)
        if q is None:
            return None, None, None
        code, err = self.move_q(q, seconds)
        return q, code, err
    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP.get("max_effort", 30.0))
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result
        return r.reached_goal, r.stalled, self.fingers()

def quat_from_R(R):
    """xyzw from rotation matrix"""
    m = R; t = np.trace(m)
    if t > 0:
        s = math.sqrt(t + 1) * 2; w = 0.25 * s; x = (m[2,1]-m[1,2])/s; y = (m[0,2]-m[2,0])/s; z = (m[1,0]-m[0,1])/s
    elif m[0,0] > m[1,1] and m[0,0] > m[2,2]:
        s = math.sqrt(1 + m[0,0]-m[1,1]-m[2,2])*2; w=(m[2,1]-m[1,2])/s; x=0.25*s; y=(m[0,1]+m[1,0])/s; z=(m[0,2]+m[2,0])/s
    elif m[1,1] > m[2,2]:
        s = math.sqrt(1 + m[1,1]-m[0,0]-m[2,2])*2; w=(m[0,2]-m[2,0])/s; x=(m[0,1]+m[1,0])/s; y=0.25*s; z=(m[1,2]+m[2,1])/s
    else:
        s = math.sqrt(1 + m[2,2]-m[0,0]-m[1,1])*2; w=(m[1,0]-m[0,1])/s; x=(m[0,2]+m[2,0])/s; y=(m[1,2]+m[2,1])/s; z=0.25*s
    return np.array([x, y, z, w])

def R_from_quat(q):
    x, y, z, w = q
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

def hand_R(approach, closing):
    """Rotation matrix with hand z = approach dir, hand y = closing dir (world vectors)."""
    a = np.asarray(approach, float); a /= np.linalg.norm(a)
    c = np.asarray(closing, float); c -= a * (c @ a); c /= np.linalg.norm(c)
    x = np.cross(c, a)
    return np.stack([x, c, a], axis=1)

if __name__ == "__main__":
    r = Robot()
    print("q", np.round(r.arm_q(), 4))
    print("fingers", r.fingers())
    pos, quat, frame = r.fk_hand()
    print("hand world pos", pos.round(4), "quat", quat.round(4), "frame", frame)
    R = R_from_quat(quat)
    print("hand x", R[:,0].round(3), "y", R[:,1].round(3), "z", R[:,2].round(3))
    print("wrench", r.wrench().round(3))
