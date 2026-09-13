#!/usr/bin/env python3
"""Small helper library: one node, reusable clients.

    from rob import R
    r = R(); r.joints(); r.ik(x,y,z,qx,qy,qz,qw, at_tcp=True); r.move(q, secs)
    r.grip(width); r.fk()
World<->base: base = world - (-0.66, 0, 0.912) (no rotation, from TF).
"""
import sys, time
import numpy as np
import rclpy, yaml
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK, GetPositionFK

BASE = np.array([-0.66, 0.0, 0.912])
M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]
TCP = float(M["hand"]["tcp_offset_m"])


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class R:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("rob_" + str(int(time.time() * 1000) % 100000))
        self._js = None
        self.node.create_subscription(JointState, "/joint_states", self._cb, 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.gr = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ikc = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fkc = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt.wait_for_server(10); self.gr.wait_for_server(10)
        self.ikc.wait_for_service(10); self.fkc.wait_for_service(10)

    def _cb(self, m):
        self._js = m

    def spin(self, t=0.05):
        rclpy.spin_once(self.node, timeout_sec=t)

    def js(self):
        self._js = None
        while self._js is None:
            self.spin(0.2)
        return dict(zip(self._js.name, self._js.position))

    def joints(self):
        j = self.js()
        return [j[n] for n in ARM]

    def fingers(self):
        j = self.js()
        return j["panda_finger_joint1"], j["panda_finger_joint2"]

    def _seed(self):
        s = JointState()
        j = self.js()
        for n in ARM:
            s.name.append(n); s.position.append(j[n])
        return s

    def fk(self):
        """hand pose in WORLD (pos, quat)."""
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state = self._seed()
        fut = self.fkc.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z])  # FK already reports world
        q = (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        return pos, q

    def tcp(self):
        pos, q = self.fk()
        return pos + TCP * quat_R(*q)[:, 2], q

    def ik(self, x, y, z, qx, qy, qz, qw, at_tcp=True, seed=None):
        """world pose -> arm joints (list) or None."""
        p = np.array([x, y, z], float)
        if at_tcp:
            p = p - TCP * quat_R(qx, qy, qz, qw)[:, 2]
        # IK model frame == world here (FK reports panda_link0 at BASE)
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.ik_link_name = "panda_hand"  # default tip is link8 (45deg off the hand)
        req.ik_request.pose_stamped.header.frame_id = ""
        ps = req.ik_request.pose_stamped.pose
        ps.position.x, ps.position.y, ps.position.z = map(float, p)
        ps.orientation.x, ps.orientation.y, ps.orientation.z, ps.orientation.w = map(float, (qx, qy, qz, qw))
        if seed is None:
            req.ik_request.robot_state.joint_state = self._seed()
        else:
            s = JointState(); s.name = list(ARM); s.position = [float(v) for v in seed]
            req.ik_request.robot_state.joint_state = s
        req.ik_request.timeout.sec = 2
        fut = self.ikc.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            print("IK failed", None if res is None else res.error_code.val, file=sys.stderr)
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in ARM]

    def move(self, q, secs=3.0, wait=True):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pt = JointTrajectoryPoint(positions=[float(v) for v in q])
        pt.time_from_start = Duration(sec=int(secs), nanosec=int((secs % 1) * 1e9))
        goal.trajectory.points = [pt]
        fut = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=120)
        h = fut.result()
        rf = h.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=600)
        code = rf.result().result.error_code
        err = np.abs(np.array(self.joints()) - np.array(q)).max()
        print(f"move done code={code} max_err={err:.4f}", flush=True)
        return code

    def move_path(self, qs, secs_each=2.0):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        t = 0.0
        for q in qs:
            t += secs_each
            pt = JointTrajectoryPoint(positions=[float(v) for v in q])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        fut = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=120)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=600)
        code = rf.result().result.error_code
        err = np.abs(np.array(self.joints()) - np.array(qs[-1])).max()
        print(f"path done code={code} max_err={err:.4f}", flush=True)
        return code

    def grip(self, width):
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = float(GRIP["max_effort"])
        fut = self.gr.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result
        print(f"grip -> pos={r.position:.4f} reached={r.reached_goal} stalled={r.stalled} fingers={self.fingers()}", flush=True)
        return r

    def goto(self, x, y, z, q, secs=3.0, at_tcp=True):
        j = self.ik(x, y, z, *q, at_tcp=at_tcp)
        if j is None:
            return None
        code = self.move(j, secs)
        p, _ = self.tcp()
        print(f"tcp now {p.round(4)} target {np.array([x,y,z]).round(4)}", flush=True)
        return code
