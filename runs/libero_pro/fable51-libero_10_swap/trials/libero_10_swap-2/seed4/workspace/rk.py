"""Shared helpers: FK/IK clients, joint state, quaternion utils (reuse in scripts)."""
import time, math, numpy as np, rclpy, yaml
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from moveit_msgs.msg import RobotState
from geometry_msgs.msg import PoseStamped

M = yaml.safe_load(open("/workspace/machine.yaml"))
ARM = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")["joints"]
BASE = np.array([-0.66, 0.0, 0.912])  # world -> panda_link0 translation (no rotation)

def R_from_q(x, y, z, w):
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])

def q_from_R(R):
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t+1)*2; w = 0.25*s; x = (R[2,1]-R[1,2])/s; y = (R[0,2]-R[2,0])/s; z = (R[1,0]-R[0,1])/s
    elif R[0,0] > R[1,1] and R[0,0] > R[2,2]:
        s = math.sqrt(1+R[0,0]-R[1,1]-R[2,2])*2; w = (R[2,1]-R[1,2])/s; x = 0.25*s; y = (R[0,1]+R[1,0])/s; z = (R[0,2]+R[2,0])/s
    elif R[1,1] > R[2,2]:
        s = math.sqrt(1+R[1,1]-R[0,0]-R[2,2])*2; w = (R[0,2]-R[2,0])/s; x = (R[0,1]+R[1,0])/s; y = 0.25*s; z = (R[1,2]+R[2,1])/s
    else:
        s = math.sqrt(1+R[2,2]-R[0,0]-R[1,1])*2; w = (R[1,0]-R[0,1])/s; x = (R[0,2]+R[2,0])/s; y = (R[1,2]+R[2,1])/s; z = 0.25*s
    return np.array([x, y, z, w])

class Robot:
    def __init__(self, name="rk"):
        if not rclpy.ok(): rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states", lambda m: self._js.__setitem__("m", m), 1)
        self.ik = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.ik.wait_for_service(10); self.fk.wait_for_service(10)

    def joints(self, fresh=True):
        if fresh: self._js.pop("m", None)
        end = time.time()+15
        while "m" not in self._js and time.time() < end: rclpy.spin_once(self.node, timeout_sec=0.2)
        m = self._js["m"]; d = dict(zip(m.name, m.position))
        return d

    def arm_q(self):
        d = self.joints(); return [d[j] for j in ARM]

    def fingers(self):
        d = self.joints(); return d.get("panda_finger_joint1"), d.get("panda_finger_joint2")

    def _seed(self, q):
        js = JointState(); js.name = list(ARM); js.position = [float(v) for v in q]; return js

    def fk_world(self, q=None):
        """hand pose in WORLD: (pos, quat)"""
        if q is None: q = self.arm_q()
        req = GetPositionFK.Request(); req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state = self._seed(q)
        f = self.fk.call_async(req); rclpy.spin_until_future_complete(self.node, f, timeout_sec=30)
        r = f.result()
        if r is None or r.error_code.val != 1: raise RuntimeError(f"FK failed {r and r.error_code.val}")
        p = r.pose_stamped[0].pose
        return np.array([p.position.x, p.position.y, p.position.z]), np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])

    def ik_world(self, pos, quat, seed=None, timeout=30):
        """pos in WORLD; returns joint list or None"""
        pos = np.asarray(pos, float)  # model frame == world on this machine (verified by FK/IK)
        req = GetPositionIK.Request(); r = req.ik_request
        r.group_name = "panda_arm"; r.pose_stamped.header.frame_id = ""
        r.ik_link_name = "panda_hand"
        p = r.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pos)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        r.robot_state.joint_state = self._seed(seed if seed is not None else self.arm_q())
        r.avoid_collisions = False
        r.timeout.sec = 2
        f = self.ik.call_async(req); rclpy.spin_until_future_complete(self.node, f, timeout_sec=timeout)
        res = f.result()
        if res is None: raise RuntimeError("IK timeout")
        if res.error_code.val != 1: return None
        d = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [d[j] for j in ARM]

def hand_quat(approach, closing):
    """world-frame quaternion for hand with z=approach, y=closing"""
    z = np.asarray(approach, float); z /= np.linalg.norm(z)
    y = np.asarray(closing, float); y -= z*np.dot(y, z); y /= np.linalg.norm(y)
    x = np.cross(y, z)
    return q_from_R(np.column_stack([x, y, z]))

# ---------------------------------------------------------------- motion
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class Mover(Robot):
    def __init__(self, name="mover"):
        super().__init__(name)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, "/panda_arm_controller/follow_joint_trajectory")
        self.grip = ActionClient(self.node, GripperCommand, "/franka_gripper/gripper_action")
        self.fjt.wait_for_server(10); self.grip.wait_for_server(10)

    def goto_q(self, q, seconds=3.0, via=None):
        goal = FollowJointTrajectory.Goal(); goal.trajectory.joint_names = list(ARM)
        pts = []
        wps = (via or []) + [q]
        n = len(wps)
        for i, w in enumerate(wps):
            t = seconds*(i+1)/n
            pt = JointTrajectoryPoint(positions=[float(v) for v in w])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1)*1e9)); pts.append(pt)
        goal.trajectory.points = pts
        f = self.fjt.send_goal_async(goal); rclpy.spin_until_future_complete(self.node, f, timeout_sec=60)
        h = f.result()
        rf = h.get_result_async(); rclpy.spin_until_future_complete(self.node, rf, timeout_sec=600)
        code = rf.result().result.error_code
        qa = np.array(self.arm_q()); err = np.abs(qa - np.array(q)).max()
        print(f"  traj error_code={code} max_joint_err={err:.4f}", flush=True)
        return code, err

    def goto_pose(self, pos, quat, seconds=3.0, seed=None, max_delta=None):
        q = self.ik_world(pos, quat, seed=seed)
        if q is None:
            print(f"  IK FAILED for {np.round(pos,3)}", flush=True); return None
        cur = np.array(self.arm_q()); d = np.abs(np.array(q)-cur)
        print(f"  IK ok, joint deltas {np.round(d,2)}", flush=True)
        if max_delta is not None and d.max() > max_delta:
            print("  delta too large, refusing", flush=True); return None
        self.goto_q(q, seconds)
        p, _ = self.fk_world(); print(f"  hand now at {np.round(p,4)}", flush=True)
        return q

    def gripper(self, width):
        g = GripperCommand.Goal(); g.command.position = float(width); g.command.max_effort = 30.0
        f = self.grip.send_goal_async(g); rclpy.spin_until_future_complete(self.node, f, timeout_sec=60)
        rf = f.result().get_result_async(); rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        res = rf.result().result
        print(f"  gripper reached={res.reached_goal} stalled={res.stalled} fingers={np.round(self.fingers(),4)}", flush=True)
        return res

TOPDOWN = np.array([1.0, 0.0, 0.0, 0.0])  # hand x=+X, y=-Y, z=-Z (fingers close along world Y)

def slerp(q0, q1, t):
    q0 = np.asarray(q0, float); q1 = np.asarray(q1, float)
    d = np.dot(q0, q1)
    if d < 0: q1 = -q1; d = -d
    if d > 0.9995:
        q = q0 + t*(q1-q0); return q/np.linalg.norm(q)
    th = math.acos(d)
    return (math.sin((1-t)*th)*q0 + math.sin(t*th)*q1)/math.sin(th)

def move_interp(m, pos, quat, n=6, seconds=4.0, max_jump=0.6, dry=False):
    """Cartesian-ish move: n IK waypoints between current hand pose and target, one trajectory."""
    p0, q0 = m.fk_world(); pos = np.asarray(pos, float)
    seed = m.arm_q(); wps = []
    for i in range(1, n+1):
        t = i/n
        q = m.ik_world(p0 + t*(pos-p0), slerp(q0, quat, t), seed=seed)
        if q is None:
            print(f"  IK failed at waypoint {i}/{n}", flush=True); return None
        jump = np.abs(np.array(q)-np.array(seed)).max()
        if jump > max_jump:
            print(f"  joint jump {jump:.2f} at waypoint {i}/{n}: {np.round(q,2)}", flush=True); return None
        wps.append(q); seed = q
    print(f"  {n} waypoints ok, final q {np.round(wps[-1],2)}", flush=True)
    if dry: return wps
    m.goto_q(wps[-1], seconds, via=wps[:-1])
    p, qq = m.fk_world()
    print(f"  hand now {np.round(p,4)} (target {np.round(pos,4)}) err={np.linalg.norm(p-pos)*1000:.1f}mm quat_err={1-abs(np.dot(qq,quat)):.4f}", flush=True)
    return wps
