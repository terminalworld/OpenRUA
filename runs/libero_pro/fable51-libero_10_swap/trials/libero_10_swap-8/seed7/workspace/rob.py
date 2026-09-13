"""Shared robot helper: persistent node, IK/FK, trajectory, gripper, servo, joint read."""
import math, time, sys
import numpy as np, rclpy, yaml
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
TW = next(a for a in M["actuators"] if a["kind"] == "cartesian_twist")
JOINTS = FJT["joints"]
BASE = np.array([0.0, 0.0, 0.0])   # FK/IK poses are already in world (verified via FK vs eye-in-hand TF)
TCP = M["hand"]["tcp_offset_m"]
TABLE = 0.899

def R_from_q(x, y, z, w):
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])

def q_from_R(R):
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t+1)*2; w = s/4; x = (R[2,1]-R[1,2])/s; y = (R[0,2]-R[2,0])/s; z = (R[1,0]-R[0,1])/s
    elif R[0,0] > R[1,1] and R[0,0] > R[2,2]:
        s = math.sqrt(1+R[0,0]-R[1,1]-R[2,2])*2; w = (R[2,1]-R[1,2])/s; x = s/4; y = (R[0,1]+R[1,0])/s; z = (R[0,2]+R[2,0])/s
    elif R[1,1] > R[2,2]:
        s = math.sqrt(1+R[1,1]-R[0,0]-R[2,2])*2; w = (R[0,2]-R[2,0])/s; x = (R[0,1]+R[1,0])/s; y = s/4; z = (R[1,2]+R[2,1])/s
    else:
        s = math.sqrt(1+R[2,2]-R[0,0]-R[1,1])*2; w = (R[1,0]-R[0,1])/s; x = (R[0,2]+R[2,0])/s; y = (R[1,2]+R[2,1])/s; z = s/4
    return np.array([x, y, z, w])

def grasp_R(yaw=0.0, tilt=0.0):
    """Hand rotation: approach down, fingers close along world x (rotated by yaw about z),
    approach tilted by `tilt` rad about the closing axis."""
    hz = np.array([0, 0, -1.0]); hy = np.array([1.0, 0, 0])
    # tilt about hy
    c, s = math.cos(tilt), math.sin(tilt)
    Rt = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])  # rotate about y (=hy) -> tilts hz toward +x/-x
    hz = Rt @ hz
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    hz = Rz @ hz; hy = Rz @ hy
    hx = np.cross(hy, hz)
    return np.column_stack([hx, hy, hz])

class Robot:
    def __init__(self):
        if not rclpy.ok(): rclpy.init()
        self.node = rclpy.create_node("rob_" + str(int(time.time()) % 100000))
        self.js = {}
        self.node.create_subscription(JointState, "/joint_states", self._js_cb, 10)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.tw_pub = self.node.create_publisher(TwistStamped, TW["port"], 10)
        self.fjt.wait_for_server(10); self.grip.wait_for_server(10)
        self.ik.wait_for_service(10); self.fk.wait_for_service(10)
        self.wait_js()

    def _js_cb(self, m):
        self.js = dict(zip(m.name, m.position))

    def spin(self, t=0.05): rclpy.spin_once(self.node, timeout_sec=t)

    def wait_js(self):
        self.js = {}
        while not self.js: self.spin(0.2)
        return dict(self.js)

    def arm_q(self):
        js = self.wait_js(); return [js[j] for j in JOINTS]

    def fingers(self):
        js = self.wait_js(); return js.get("panda_finger_joint1"), js.get("panda_finger_joint2")

    def fk_hand(self, q=None):
        """world pose of panda_hand (pos, quat) and TCP position."""
        req = GetPositionFK.Request(); req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = JOINTS
        req.robot_state.joint_state.position = q if q is not None else self.arm_q()
        fut = self.fk.call_async(req); rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        r = fut.result()
        if r is None or r.error_code.val != 1: return None
        p = r.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE
        q = np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        tcp = pos + R_from_q(*q)[:, 2] * TCP
        return pos, q, tcp

    def ik_tcp(self, tcp_world, R, seed=None, tries=3):
        """IK for a TCP position (world) with rotation R (3x3). returns joint list or None."""
        pos = np.asarray(tcp_world) - R[:, 2] * TCP - BASE
        # IK tip link is panda_link8 = panda_hand rotated +45deg about z (tf: link8->hand is Rz(-45deg))
        c, s_ = math.cos(math.pi/4), math.sin(math.pi/4)
        R8 = R @ np.array([[c, -s_, 0], [s_, c, 0], [0, 0, 1]])
        q = q_from_R(R8)
        seed = seed if seed is not None else self.arm_q()
        for _ in range(tries):
            req = GetPositionIK.Request()
            req.ik_request.group_name = M["planning"]["group"]
            req.ik_request.pose_stamped.header.frame_id = ""
            p = req.ik_request.pose_stamped.pose
            p.position.x, p.position.y, p.position.z = map(float, pos)
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q)
            req.ik_request.robot_state.joint_state.name = JOINTS
            req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
            req.ik_request.timeout.sec = 1
            req.ik_request.avoid_collisions = False
            fut = self.ik.call_async(req); rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            r = fut.result()
            if r is not None and r.error_code.val == 1:
                sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
                return [sol[j] for j in JOINTS]
        return None

    def move_q(self, q, seconds=3.0, via=None):
        goal = FollowJointTrajectory.Goal(); goal.trajectory.joint_names = JOINTS
        pts = []
        if via:
            n = len(via) + 1
            for i, v in enumerate(via):
                t = seconds * (i+1) / n
                pt = JointTrajectoryPoint(positions=[float(x) for x in v]); pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1)*1e9)); pts.append(pt)
        pt = JointTrajectoryPoint(positions=[float(x) for x in q])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1)*1e9)); pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal); rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        res = gh.get_result_async(); rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        err = np.max(np.abs(np.array(self.arm_q()) - np.array(q)))
        print(f"  move: code={code} max_joint_err={err:.4f}", flush=True)
        return code, err

    def gripper(self, width):
        g = GripperCommand.Goal(); g.command.position = float(width); g.command.max_effort = float(GRIP["max_effort"])
        send = self.grip.send_goal_async(g); rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        res = send.result().get_result_async(); rclpy.spin_until_future_complete(self.node, res, timeout_sec=300)
        r = res.result().result
        f = self.fingers()
        print(f"  gripper({width}): reached={r.reached_goal} stalled={r.stalled} fingers={f}", flush=True)
        return f

    def servo(self, v, n, frame=None):
        """publish n twist messages with linear velocity v (m/s, base frame)."""
        msg = TwistStamped(); msg.header.frame_id = frame or TW["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, v)
        for _ in range(n):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.tw_pub.publish(msg); self.spin(0.05)

def xbar_R(tilt):
    """Fingers close along world y (handle bar runs along x); approach pitched by `tilt` rad
    toward +x so the wrist sits closer to the robot. tilt=0 is the plain top-down grasp."""
    R0 = grasp_R(-math.pi/2, 0)
    c, s = math.cos(-tilt), math.sin(-tilt)
    Ry = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    return Ry @ R0
