#!/usr/bin/env python3
"""Reusable arm control: joints, FK, IK (yaw-corrected), trajectory, gripper, twist servo.
All poses in WORLD frame (this machine's MoveIt model frame is `world`)."""
import math, sys, time
import numpy as np
import rclpy
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped, WrenchStamped

ARM = [f"panda_joint{i}" for i in range(1, 8)]
LIM = [(-2.9, 2.9), (-1.76, 1.76), (-2.9, 2.9), (-3.07, -0.07), (-2.9, 2.9), (-0.02, 3.75), (-2.9, 2.9)]
TCP = 0.1034
TABLE_Z = 0.425


def quat_R(x, y, z, w):
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def topdown_quat(yaw):
    """Hand z pointing down (-world z), hand x-axis at `yaw` rad from world +x. xyzw."""
    # R = Rz(yaw) * Rx(pi)
    c, s = math.cos(yaw / 2), math.sin(yaw / 2)
    # Rx(pi) = (1,0,0,0); Rz(yaw) = (0,0,s,c);  q = qz * qx
    return (c, s, 0.0, 0.0)  # (x,y,z,w) with w=0 -> (cos(yaw/2), sin(yaw/2), 0, 0)


class Arm:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("arm_ctl")
        self.js = {}
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 1)
        self.wr = {}
        self.node.create_subscription(WrenchStamped, "/franka_robot_state_broadcaster/external_wrench", self._on_wr, 1)
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.ik_cli = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fjt = ActionClient(self.node, FollowJointTrajectory, "/panda_arm_controller/follow_joint_trajectory")
        self.grip = ActionClient(self.node, GripperCommand, "/franka_gripper/gripper_action")
        self.twist_pub = self.node.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)
        self.fk_cli.wait_for_service(10); self.ik_cli.wait_for_service(10)
        self.fjt.wait_for_server(10); self.grip.wait_for_server(10)

    def _on_js(self, m):
        self.js["m"] = m; self.js["n"] = self.js.get("n", 0) + 1

    def _on_wr(self, m):
        self.wr["m"] = m

    def spin(self, t=0.2):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self, fresh=True):
        n0 = self.js.get("n", 0)
        while "m" not in self.js or (fresh and self.js["n"] <= n0):
            self.spin(0.3)
        d = dict(zip(self.js["m"].name, self.js["m"].position))
        return [d[j] for j in ARM], (d.get("panda_finger_joint1", 0.0), d.get("panda_finger_joint2", 0.0))

    def wrench(self):
        self.wr.pop("m", None)
        while "m" not in self.wr:
            self.spin(0.3)
        w = self.wr["m"].wrench
        return np.array([w.force.x, w.force.y, w.force.z, w.torque.x, w.torque.y, w.torque.z])

    def fk(self, q):
        req = GetPositionFK.Request(); req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = ARM; req.robot_state.joint_state.position = list(q)
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        p = fut.result().pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z])
        o = p.orientation
        return pos, (o.x, o.y, o.z, o.w)

    def hand_pose(self):
        q, f = self.joints()
        pos, quat = self.fk(q)
        R = quat_R(*quat)
        tcp = pos + TCP * R[:, 2]
        yaw = math.atan2(R[1, 0], R[0, 0])
        return dict(q=q, fingers=f, hand=pos, tcp=tcp, quat=quat, approach=R[:, 2], yaw=yaw)

    def ik(self, pos, quat, seed=None, timeout=3):
        r = GetPositionIK.Request(); r.ik_request.group_name = "panda_arm"
        r.ik_request.pose_stamped.header.frame_id = ""
        p = r.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pos)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        r.ik_request.robot_state.joint_state.name = ARM
        r.ik_request.robot_state.joint_state.position = list(seed if seed is not None else self.joints()[0])
        r.ik_request.timeout.sec = timeout
        r.ik_request.avoid_collisions = False
        f = self.ik_cli.call_async(r); rclpy.spin_until_future_complete(self.node, f, timeout_sec=90)
        res = f.result()
        if res is None or res.error_code.val != 1:
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in ARM]

    def ik_topdown(self, pos, yaw, seed=None):
        """IK for a top-down hand pose (TCP at pos... no: HAND frame at pos) with hand-x yaw; then
        correct yaw via joint 7 and verify by FK. Returns joints or None."""
        pos = np.asarray(pos, float)
        quat = topdown_quat(yaw)
        best = None
        for attempt in range(6):
            q = self.ik(pos, quat, seed=seed if attempt == 0 else (np.array(seed if seed is not None else self.joints()[0]) + np.random.uniform(-0.3, 0.3, 7)).tolist())
            if q is None:
                continue
            # yaw fix through joint 7 (hand z is down: rotating j7 by +d rotates hand about world -z)
            for _ in range(3):
                fpos, fq = self.fk(q)
                R = quat_R(*fq)
                got_yaw = math.atan2(R[1, 0], R[0, 0])
                dyaw = (yaw - got_yaw + math.pi) % (2 * math.pi) - math.pi
                if abs(dyaw) < 0.01:
                    break
                q7 = q[6] - dyaw  # sign determined empirically below; retried if wrong
                if not LIM[6][0] < q7 < LIM[6][1]:
                    q7 = q[6] + dyaw
                    q7 = ((q7 + math.pi) % (2 * math.pi)) - math.pi
                q = q[:6] + [q7]
            fpos, fq = self.fk(q); R = quat_R(*fq)
            got_yaw = math.atan2(R[1, 0], R[0, 0])
            perr = np.linalg.norm(fpos - pos); tilt = np.degrees(np.arccos(np.clip(-R[2, 2], -1, 1)))
            dyaw = abs((yaw - got_yaw + math.pi) % (2 * math.pi) - math.pi)
            ok = perr < 0.005 and tilt < 3 and dyaw < 0.05 and all(LIM[i][0] <= q[i] <= LIM[i][1] for i in range(7))
            if ok:
                return q
            best = (perr, tilt, dyaw, q)
        print("ik_topdown failed; best:", best, file=sys.stderr)
        return None

    def move(self, q, seconds=3.0, via=None):
        goal = FollowJointTrajectory.Goal(); goal.trajectory.joint_names = ARM
        pts = []
        if via:
            n = len(via) + 1
            for i, v in enumerate(via):
                pt = JointTrajectoryPoint(positions=list(map(float, v)))
                t = seconds * (i + 1) / n
                pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9)); pts.append(pt)
        pt = JointTrajectoryPoint(positions=list(map(float, q)))
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9)); pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal); rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        res = send.result().get_result_async(); rclpy.spin_until_future_complete(self.node, res, timeout_sec=900)
        code = res.result().result.error_code if res.result() else None
        cur, _ = self.joints()
        err = max(abs(a - b) for a, b in zip(cur, q))
        return code, err

    def move_to(self, q, seconds=3.0, tol=0.02, retries=2):
        for i in range(retries + 1):
            code, err = self.move(q, seconds)
            print(f"  move: code={code} max_joint_err={err:.4f}")
            if err < tol:
                return True
            seconds = max(seconds, 2.0)
        return err < tol

    def gripper(self, width):
        g = GripperCommand.Goal(); g.command.position = float(width); g.command.max_effort = 30.0
        f = self.grip.send_goal_async(g); rclpy.spin_until_future_complete(self.node, f, timeout_sec=60)
        r = f.result().get_result_async(); rclpy.spin_until_future_complete(self.node, r, timeout_sec=300)
        res = r.result().result if r.result() else None
        _, fingers = self.joints()
        _, fingers = self.joints()
        print(f"  gripper({width}): reached={getattr(res,'reached_goal',None)} stalled={getattr(res,'stalled',None)} fingers={fingers}")
        return fingers

    def servo(self, vx=0, vy=0, vz=0, ticks=20, frame="panda_link0"):
        msg = TwistStamped(); msg.header.frame_id = frame
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = float(vx), float(vy), float(vz)
        for _ in range(ticks):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.twist_pub.publish(msg); self.spin(0.05)
        return self.hand_pose()


def fmt(p):
    return f"hand={np.round(p['hand'],4)} tcp={np.round(p['tcp'],4)} yaw={math.degrees(p['yaw']):.1f}deg approach={np.round(p['approach'],3)} fingers={np.round(p['fingers'],4)}"


if __name__ == "__main__":
    a = Arm()
    print(fmt(a.hand_pose()))
