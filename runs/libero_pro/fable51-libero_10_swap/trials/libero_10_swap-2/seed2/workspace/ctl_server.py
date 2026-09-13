#!/usr/bin/env python3
"""Persistent controller: builds ROS clients once, serves JSON commands over
a local TCP socket (port 5555).  Run: python3 -u ctl_server.py > ctl.log 2>&1 &
Client: python3 c.py '{"cmd": "js"}'
"""
import json
import socket
import threading
import traceback
from pathlib import Path

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from cv_bridge import CvBridge
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import Image, JointState
from trajectory_msgs.msg import JointTrajectoryPoint

import cv2

M = yaml.safe_load(Path("machine.yaml").read_text())
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
TW = next(a for a in M["actuators"] if a["kind"] == "cartesian_twist")
ARM = FJT["joints"]
# measured: /compute_fk and /compute_ik on this machine work in the WORLD
# frame (they match tf world->panda_hand), so no base offset is applied.
BASE_IN_WORLD = np.array([0.0, 0.0, 0.0])
# IK's tip link is the flange (panda_link8), which is panda_hand rotated
# +45deg about z.  quat args everywhere are the desired panda_hand quat.
def quat_mul(a, b):
    ax, ay, az, aw = a; bx, by, bz, bw = b
    return [aw*bx + ax*bw + ay*bz - az*by,
            aw*by - ax*bz + ay*bw + az*bx,
            aw*bz + ax*by - ay*bx + az*bw,
            aw*bw - ax*bx - ay*by - az*bz]
RZ45 = [0.0, 0.0, np.sin(np.pi/8), np.cos(np.pi/8)]
TCP = float(M["hand"]["tcp_offset_m"])


def quat_to_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class Ctl:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("ctl_server")
        self.js = None
        self.node.create_subscription(JointState, "/joint_states", self._js, 10)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.tw = self.node.create_publisher(TwistStamped, TW["port"], 10)
        self.bridge = CvBridge()
        self.fjt.wait_for_server(30); self.grip.wait_for_server(30)
        self.ik.wait_for_service(30); self.fk.wait_for_service(30)
        self.spin(1.0)

    def spin(self, t):
        end = self.node.get_clock().now().nanoseconds / 1e9 + t
        # sim clock may be paused: also bound by wall iterations
        for _ in range(int(t / 0.05) + 1):
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _js(self, m):
        self.js = m

    # ---------------- sensing ----------------
    def cmd_js(self):
        for _ in range(100):
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.js is not None:
                break
        d = dict(zip(self.js.name, self.js.position))
        return {"arm": [d[j] for j in ARM],
                "fingers": [d.get("panda_finger_joint1"), d.get("panda_finger_joint2")]}

    def arm_seed(self):
        js = self.cmd_js()
        s = JointState(); s.name = list(ARM); s.position = list(js["arm"])
        return s

    def cmd_fk(self, joints=None):
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state = self.arm_seed() if joints is None else self._seed(joints)
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            return {"error": "fk failed", "code": None if r is None else r.error_code.val}
        p = r.pose_stamped[0].pose
        pos_b = np.array([p.position.x, p.position.y, p.position.z])
        q = [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
        pos_w = pos_b + BASE_IN_WORLD
        tcp_w = pos_w + TCP * quat_to_R(q)[:, 2]
        return {"hand_world": pos_w.round(4).tolist(), "tcp_world": tcp_w.round(4).tolist(),
                "quat": [round(v, 4) for v in q]}

    def _seed(self, joints):
        s = JointState(); s.name = list(ARM); s.position = [float(v) for v in joints]
        return s

    # ---------------- IK ----------------
    def cmd_ik(self, pos, quat, at="tcp", seed=None):
        pos = np.array(pos, float)
        if at == "tcp":
            pos = pos - TCP * quat_to_R(quat)[:, 2]
        pb = pos - BASE_IN_WORLD
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pb)
        q8 = quat_mul(list(quat), RZ45)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q8)
        req.ik_request.robot_state.joint_state = self.arm_seed() if seed is None else self._seed(seed)
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = 2
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        r = fut.result()
        if r is None:
            return {"error": "ik timeout"}
        if r.error_code.val != 1:
            return {"error": "ik failed", "code": r.error_code.val}
        sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
        return {"joints": [sol[j] for j in ARM]}

    # ---------------- motion ----------------
    def cmd_joints(self, joints, t=3.0, via=None):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pts = []
        if via:
            n = len(via) + 1
            for i, v in enumerate(via):
                pt = JointTrajectoryPoint(positions=[float(x) for x in v])
                tt = t * (i + 1) / n
                pt.time_from_start = Duration(sec=int(tt), nanosec=int((tt % 1) * 1e9))
                pts.append(pt)
        pt = JointTrajectoryPoint(positions=[float(x) for x in joints])
        pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
        pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        if gh is None or not gh.accepted:
            return {"error": "goal not accepted"}
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        if res.result() is None:
            return {"error": "result timeout"}
        code = res.result().result.error_code
        self.spin(0.3)
        js = self.cmd_js()
        err = float(np.max(np.abs(np.array(js["arm"]) - np.array(joints, float))))
        return {"error_code": code, "max_joint_err": round(err, 4), "arm": [round(v, 4) for v in js["arm"]]}

    def cmd_move(self, pos, quat, t=3.0, at="tcp", seed=None):
        ik = self.cmd_ik(pos, quat, at, seed)
        if "error" in ik:
            return ik
        r = self.cmd_joints(ik["joints"], t)
        r["fk"] = self.cmd_fk()
        r["ik_joints"] = [round(v, 4) for v in ik["joints"]]
        return r

    def cmd_grip(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        send = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=300)
        r = res.result().result
        self.spin(0.3)
        return {"reached": r.reached_goal, "stalled": r.stalled, "position": r.position,
                "fingers": self.cmd_js()["fingers"]}

    def cmd_servo(self, lin, ang=(0, 0, 0), n=20):
        msg = TwistStamped(); msg.header.frame_id = TW["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, lin)
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = map(float, ang)
        for _ in range(int(n)):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.tw.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.spin(0.3)
        return {"fk": self.cmd_fk()}

    def cmd_snap(self, cam, out=None):
        got = {}
        topic = f"/{cam}/color/image_raw"
        sub = self.node.create_subscription(Image, topic, lambda m: got.setdefault("m", m), 1)
        for _ in range(600):
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if "m" in got:
                break
        self.node.destroy_subscription(sub)
        if "m" not in got:
            return {"error": "no image"}
        out = out or f"{cam}.png"
        cv2.imwrite(out, self.bridge.imgmsg_to_cv2(got["m"], "bgr8"))
        return {"saved": out}

    def handle(self, d):
        cmd = d.pop("cmd")
        return getattr(self, "cmd_" + cmd)(**d)


def main():
    ctl = Ctl()
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", 5555)); srv.listen(1)
    print("ready", flush=True)
    while True:
        conn, _ = srv.accept()
        data = b""
        while not data.endswith(b"\n"):
            chunk = conn.recv(65536)
            if not chunk:
                break
            data += chunk
        try:
            d = json.loads(data.decode())
            print(">>", d, flush=True)
            out = ctl.handle(d)
        except Exception as e:  # noqa
            out = {"error": repr(e), "tb": traceback.format_exc()}
        print("<<", out, flush=True)
        conn.sendall((json.dumps(out) + "\n").encode())
        conn.close()


if __name__ == "__main__":
    main()
