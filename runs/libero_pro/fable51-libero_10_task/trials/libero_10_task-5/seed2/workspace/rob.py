#!/usr/bin/env python3
"""Small helper library / CLI for this Panda station.

CLI:
  python3 rob.py state                      # joints, TCP pose (world), fingers, wrench
  python3 rob.py tcp X Y Z [yaw_deg] [secs] # move TCP to world pose, hand pointing down
  python3 rob.py joints p1,...,p7 [secs]
  python3 rob.py grip WIDTH
"""
import sys
import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import WrenchStamped
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from scipy.spatial.transform import Rotation as Rot

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]
TCP_OFF = float(M["hand"]["tcp_offset_m"])
BASE_IN_WORLD = np.zeros(3)  # MoveIt FK/IK here already answer in the WORLD frame (verified empirically)


class Robot:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("rob")
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self._js.__setitem__("m", m), 1)
        self._wr = {}
        self.node.create_subscription(WrenchStamped, "/franka_robot_state_broadcaster/external_wrench",
                                      lambda m: self._wr.__setitem__("m", m), 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")

    def spin(self, t=0.1):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self):
        self._js.clear()
        while "m" not in self._js:
            self.spin(0.2)
        m = self._js["m"]
        return dict(zip(m.name, m.position))

    def wrench(self):
        self._wr.clear()
        end = 50
        while "m" not in self._wr and end > 0:
            self.spin(0.2); end -= 1
        if "m" not in self._wr:
            return None
        w = self._wr["m"].wrench
        return np.array([w.force.x, w.force.y, w.force.z, w.torque.x, w.torque.y, w.torque.z])

    def arm_seed(self, js=None):
        js = js or self.joints()
        s = JointState()
        for j in ARM:
            s.name.append(j); s.position.append(js[j])
        return s

    def fk_hand(self, js=None):
        """hand frame pose in BASE frame -> (pos, quat xyzw)"""
        self.fk.wait_for_service(5)
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state = self.arm_seed(js)
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            raise RuntimeError(f"FK failed {r}")
        p = r.pose_stamped[0].pose
        return (np.array([p.position.x, p.position.y, p.position.z]),
                np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]))

    def tcp_world(self, js=None):
        p, q = self.fk_hand(js)
        R = Rot.from_quat(q).as_matrix()
        tcp = p + R[:, 2] * TCP_OFF
        return tcp + BASE_IN_WORLD, q

    @staticmethod
    def hand_R(point, pads):
        """Rotation whose columns are hand X/Y/Z in world: Z = pointing
        direction (approach), Y = finger opening axis."""
        z = np.asarray(point, float); z /= np.linalg.norm(z)
        y = np.asarray(pads, float); y -= y.dot(z) * z; y /= np.linalg.norm(y)
        x = np.cross(y, z)
        return np.stack([x, y, z], axis=1)

    def ik_tcp_world(self, xyz, yaw_deg=0.0, seed=None, R=None):
        """IK for TCP at world xyz. Default: hand pointing straight down,
        fingers opening along world Y rotated by yaw_deg about world Z.
        R (3x3, see hand_R) overrides the orientation."""
        if R is None:
            R = Rot.from_euler("z", yaw_deg, degrees=True) * Rot.from_quat([1, 0, 0, 0])
        else:
            R = Rot.from_matrix(np.asarray(R, float))
        Rm = R.as_matrix()
        hand = np.asarray(xyz, float) - BASE_IN_WORLD - Rm[:, 2] * TCP_OFF
        q = R.as_quat()
        self.ik.wait_for_service(5)
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.ik_link_name = "panda_hand"
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = hand
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = q
        req.ik_request.robot_state.joint_state = self.arm_seed(seed)
        req.ik_request.timeout.sec = 2
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            raise RuntimeError(f"IK failed code={None if r is None else r.error_code.val}")
        sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
        return [sol[j] for j in ARM]

    def move_joints(self, positions, secs=3.0, via=None):
        self.fjt.wait_for_server(10)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM
        pts = []
        if via:
            for i, (pos, t) in enumerate(via):
                pt = JointTrajectoryPoint(positions=[float(x) for x in pos])
                pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
                pts.append(pt)
        pt = JointTrajectoryPoint(positions=[float(x) for x in positions])
        pt.time_from_start = Duration(sec=int(secs), nanosec=int((secs % 1) * 1e9))
        pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res)
        code = res.result().result.error_code
        js = self.joints()
        err = max(abs(js[j] - p) for j, p in zip(ARM, positions))
        print(f"traj error_code={code} max_joint_err={err:.4f}")
        return code, err

    def move_tcp(self, xyz, yaw_deg=0.0, secs=3.0, R=None):
        sol = self.ik_tcp_world(xyz, yaw_deg, R=R)
        code, err = self.move_joints(sol, secs)
        if code == -5 and err > 0.02:  # controller lagged on a long move: resend
            code, err = self.move_joints(sol, secs)
        tcp, q = self.tcp_world()
        print(f"TCP world now: {np.round(tcp, 4)}  target {np.round(xyz, 4)}  quat {np.round(q, 3)}")
        return tcp

    def hand_axes(self):
        """current hand axes in world: (pointing, pads) unit vectors"""
        _, q = self.fk_hand()
        Rm = Rot.from_quat(q).as_matrix()
        return Rm[:, 2], Rm[:, 1]

    def gripper(self, width):
        self.grip.wait_for_server(10)
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = float(GRIP.get("max_effort", 30.0))
        fut = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=300)
        r = res.result().result
        js = self.joints()
        print(f"gripper reached={r.reached_goal} stalled={r.stalled} fingers="
              f"{js['panda_finger_joint1']:.4f},{js['panda_finger_joint2']:.4f}")
        return js

    def state(self):
        js = self.joints()
        tcp, q = self.tcp_world(js)
        print("arm:", [round(js[j], 4) for j in ARM])
        print("fingers:", round(js["panda_finger_joint1"], 4), round(js["panda_finger_joint2"], 4))
        print("TCP world:", np.round(tcp, 4), "quat", np.round(q, 3))
        w = self.wrench()
        print("wrench:", None if w is None else np.round(w, 2))


if __name__ == "__main__":
    r = Robot()
    cmd = sys.argv[1]
    a = sys.argv[2:]
    if cmd == "state":
        r.state()
    elif cmd == "tcp":
        xyz = [float(v) for v in a[:3]]
        yaw = float(a[3]) if len(a) > 3 else 0.0
        secs = float(a[4]) if len(a) > 4 else 3.0
        r.move_tcp(xyz, yaw, secs)
    elif cmd == "joints":
        pos = [float(v) for v in a[0].split(",")]
        r.move_joints(pos, float(a[1]) if len(a) > 1 else 3.0)
    elif cmd == "grip":
        r.gripper(float(a[0]))
    rclpy.shutdown()
