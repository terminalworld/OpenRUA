"""Reusable robot helpers for this Panda workstation (world<->base handled)."""
import time
import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState, Image
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
TW = next(a for a in M["actuators"] if a["kind"] == "cartesian_twist")
ARM = FJT["joints"]
BASE_IN_WORLD = np.array([0.0, 0.0, 0.0])  # planner model frame == world here (verified by FK vs TF)
TCP = float(M["hand"]["tcp_offset_m"])


def qmul(a, b):
    """quaternion product a*b, (x,y,z,w) order"""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


# machine fact (measured): the IK solver's tip frame is panda_hand yawed by
# -45deg (i.e. panda_link8). Request R_des * Rz(+45deg) to land the HAND at R_des.
IK_YAW_FIX = (0.0, 0.0, np.sin(np.pi / 8), np.cos(np.pi / 8))


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


# hand pointing straight down, fingers separating along world Y
Q_DOWN_Y = (1.0, 0.0, 0.0, 0.0)
# hand pointing down, fingers separating along world X (rotated 90deg about z)
Q_DOWN_X = (0.7071068, 0.7071068, 0.0, 0.0)


class Robot:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("rob_" + str(int(time.time()) % 100000))
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self._js.__setitem__("m", m), 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.tw_pub = self.node.create_publisher(TwistStamped, TW["port"], 10)
        self.fjt.wait_for_server(10)
        self.grip.wait_for_server(10)
        self.ik.wait_for_service(10)
        self.fk.wait_for_service(10)

    def spin(self, t=0.05):
        rclpy.spin_once(self.node, timeout_sec=t)

    # ---------- sensing ----------
    def joints(self, fresh=True):
        if fresh:
            self._js.pop("m", None)
        while "m" not in self._js:
            self.spin(0.2)
        m = self._js["m"]
        return dict(zip(m.name, m.position))

    def arm_q(self):
        j = self.joints()
        return [j[n] for n in ARM]

    def finger_gap(self):
        j = self.joints()
        return abs(j["panda_finger_joint1"]) + abs(j["panda_finger_joint2"])

    def fk_hand(self, q=None):
        """Return (pos_world, quat) of panda_hand."""
        if q is None:
            q = self.arm_q()
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {res and res.error_code.val}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE_IN_WORLD
        return pos, (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)

    def tcp_world(self, q=None):
        pos, quat = self.fk_hand(q)
        R = quat_to_R(*quat)
        return pos + TCP * R[:, 2], quat

    # ---------- planning ----------
    def ik_world(self, pos_world, quat, at_tcp=True, seed=None, attempts=3):
        """IK for the hand (or TCP) at a world pose. Returns arm joint list or None."""
        pos_world = np.array(pos_world, float)
        if at_tcp:
            R = quat_to_R(*quat)
            pos_world = pos_world - TCP * R[:, 2]
        pb = pos_world - BASE_IN_WORLD
        if seed is None:
            seed = self.arm_q()
        for _ in range(attempts):
            req = GetPositionIK.Request()
            req.ik_request.group_name = M["planning"]["group"]
            req.ik_request.pose_stamped.header.frame_id = ""
            p = req.ik_request.pose_stamped.pose
            p.position.x, p.position.y, p.position.z = map(float, pb)
            qreq = qmul(quat, IK_YAW_FIX)
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, qreq)
            req.ik_request.robot_state.joint_state.name = list(ARM)
            req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
            req.ik_request.avoid_collisions = False
            req.ik_request.timeout = Duration(sec=1)
            fut = self.ik.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            res = fut.result()
            if res is not None and res.error_code.val == 1:
                sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
                return [sol[j] for j in ARM]
            print("  IK attempt failed:", res and res.error_code.val, flush=True)
        return None

    # ---------- acting ----------
    def move_q(self, q, seconds=4.0, via=None, retries=2):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pts = []
        if via:
            n = len(via) + 1
            for i, v in enumerate(via):
                pt = JointTrajectoryPoint(positions=[float(x) for x in v])
                tt = seconds * (i + 1) / n
                pt.time_from_start = Duration(sec=int(tt), nanosec=int((tt % 1) * 1e9))
                pts.append(pt)
        pt = JointTrajectoryPoint(positions=[float(x) for x in q])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        if gh is None or not gh.accepted:
            raise RuntimeError("trajectory goal rejected")
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=600)
        res = rf.result().result if rf.result() else None
        code = res.error_code if res else None
        err = np.abs(np.array(self.arm_q()) - np.array(q)).max()
        print(f"  move_q done code={code} ({res and res.error_string}) max_joint_err={err:.4f}", flush=True)
        if (code != 0 or err > 0.02) and retries > 0:
            print("  retrying same goal", flush=True)
            return self.move_q(q, max(seconds, 4.0), retries=retries - 1)
        return code, err

    def move_tcp(self, pos_world, quat, seconds=4.0, seed=None):
        q0 = np.array(self.arm_q())
        best = None
        for _ in range(4):  # prefer the IK branch closest to where we are
            q = self.ik_world(pos_world, quat, at_tcp=True, seed=seed)
            if q is None:
                continue
            d = np.abs(np.array(q) - q0).max()
            if best is None or d < best[0]:
                best = (d, q)
            if d < 0.6:
                break
        if best is None:
            raise RuntimeError(f"IK failed for {pos_world}")
        d, q = best
        seconds = max(seconds, d / 0.1)  # machine fact: controller lags above ~0.1 rad/s
        print(f"  joint delta {d:.3f} rad -> {seconds:.1f}s", flush=True)
        self.move_q(q, seconds)
        p, _ = self.tcp_world()
        print(f"  tcp now {p.round(4)} target {np.round(pos_world,4)}", flush=True)
        return q

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP.get("max_effort", 30.0))
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=120)
        r = rf.result().result
        # a few settle ticks
        for _ in range(5):
            self.spin(0.1)
        print(f"  gripper -> {width}: reached={r.reached_goal} stalled={r.stalled} gap={self.finger_gap():.4f}", flush=True)
        return r

    def servo(self, vx=0, vy=0, vz=0, ticks=20):
        msg = TwistStamped()
        msg.header.frame_id = TW["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = float(vx), float(vy), float(vz)
        for _ in range(ticks):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.tw_pub.publish(msg)
            self.spin(0.05)

    def snap(self, cam, out=None):
        import cv2
        from cv_bridge import CvBridge
        topic = f"/{cam}/color/image_raw"
        got = {}
        sub = self.node.create_subscription(Image, topic, lambda m: got.setdefault("m", m), 1)
        while "m" not in got:
            self.spin(0.2)
        self.node.destroy_subscription(sub)
        out = out or f"/workspace/{cam}.png"
        cv2.imwrite(out, CvBridge().imgmsg_to_cv2(got["m"], "bgr8"))
        return out
