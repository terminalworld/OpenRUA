"""Helper library for this Panda workstation: state, IK, trajectory, gripper, cameras."""
import sys, time
import numpy as np
import rclpy, yaml
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState, Image, CameraInfo
from geometry_msgs.msg import WrenchStamped
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener
import rclpy.time

M = yaml.safe_load(open('/workspace/machine.yaml'))
FJT = next(a for a in M['actuators'] if a['kind'] == 'joint_trajectory')
GRIP = next(a for a in M['actuators'] if a['kind'] == 'gripper')
JOINTS = FJT['joints']
BASE = np.array([0.0, 0.0, 0.0])  # verified: MoveIt FK/IK here operate in WORLD coords (link0 at -0.75,0,0.912)
TCP = M['hand']['tcp_offset_m']


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def quat_mul(q1, q2):
    x1, y1, z1, w1 = q1; x2, y2, z2, w2 = q2
    return np.array([
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2])


def down_quat(yaw):
    """Gripper pointing straight down, fingers closing along world direction rotated by yaw from +y.
    yaw=0 -> fingers close along world y; yaw=pi/2 -> along world x."""
    qz = np.array([0, 0, np.sin(yaw / 2), np.cos(yaw / 2)])
    return quat_mul(qz, np.array([1.0, 0, 0, 0]))


class Robot:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node('rob_helper')
        self.js = {}
        self.node.create_subscription(JointState, '/joint_states', lambda m: self.js.__setitem__('m', m), 1)
        self.wr = {}
        self.node.create_subscription(WrenchStamped, '/franka_robot_state_broadcaster/external_wrench',
                                      lambda m: self.wr.__setitem__('m', m), 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT['port'])
        self.grip = ActionClient(self.node, GripperCommand, GRIP['port'])
        self.ik = self.node.create_client(GetPositionIK, '/compute_ik')
        self.fk = self.node.create_client(GetPositionFK, '/compute_fk')
        self.tfbuf = Buffer(); TransformListener(self.tfbuf, self.node)
        self.bridge = CvBridge()
        self.fjt.wait_for_server(10); self.grip.wait_for_server(10)
        self.ik.wait_for_service(10); self.fk.wait_for_service(10)

    def spin(self, t=0.1):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self, fresh=True):
        if fresh:
            self.js.pop('m', None)
        while 'm' not in self.js:
            self.spin(0.2)
        m = self.js['m']
        return dict(zip(m.name, m.position))

    def arm_q(self):
        j = self.joints()
        return [j[n] for n in JOINTS]

    def fingers(self):
        j = self.joints()
        return j['panda_finger_joint1'], j['panda_finger_joint2']

    def wrench(self):
        self.wr.pop('m', None)
        while 'm' not in self.wr:
            self.spin(0.2)
        w = self.wr['m'].wrench
        return np.array([w.force.x, w.force.y, w.force.z, w.torque.x, w.torque.y, w.torque.z])

    def hand_pose_world(self):
        """FK of panda_hand in world frame -> (pos, quat)."""
        req = GetPositionFK.Request()
        req.fk_link_names = ['panda_hand']
        seed = JointState()
        j = self.joints()
        for n in JOINTS:
            seed.name.append(n); seed.position.append(j[n])
        req.robot_state.joint_state = seed
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        r = fut.result()
        p = r.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE
        q = np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        return pos, q

    def tcp_world(self):
        pos, q = self.hand_pose_world()
        return pos + TCP * quat_to_R(*q)[:, 2], q

    def solve_ik(self, tcp_world, quat, seed=None, tcp=True):
        """IK for a TCP (or hand) pose in world. Returns joint list or None."""
        quat = np.asarray(quat, float)
        p = np.asarray(tcp_world, float)
        if tcp:
            p = p - TCP * quat_to_R(*quat)[:, 2]
        p = p - BASE
        req = GetPositionIK.Request()
        req.ik_request.group_name = M['planning']['group']
        req.ik_request.pose_stamped.header.frame_id = ''
        ps = req.ik_request.pose_stamped.pose
        ps.position.x, ps.position.y, ps.position.z = map(float, p)
        # IK tip link is panda_link8; panda_hand = link8 rotated -45deg about z
        # (same origin). Convert the requested HAND quaternion to link8.
        q8 = quat_mul(quat, np.array([0, 0, np.sin(np.pi / 8), np.cos(np.pi / 8)]))
        ps.orientation.x, ps.orientation.y, ps.orientation.z, ps.orientation.w = map(float, q8)
        req.ik_request.avoid_collisions = False
        s = JointState()
        seedq = seed if seed is not None else self.arm_q()
        for n, v in zip(JOINTS, seedq):
            s.name.append(n); s.position.append(float(v))
        req.ik_request.robot_state.joint_state = s
        req.ik_request.timeout.sec = 2
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            print('IK failed', None if r is None else r.error_code.val)
            return None
        sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
        return [sol[n] for n in JOINTS]

    def move_joints(self, targets, seconds=3.0, tol=0.01, retries=2):
        """Send a joint trajectory (list of (positions, t) or one positions list). Verify."""
        if not isinstance(targets[0], (list, tuple, np.ndarray)):
            targets = [(targets, seconds)]
        for attempt in range(retries + 1):
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = JOINTS
            for pos, t in targets:
                pt = JointTrajectoryPoint(positions=[float(x) for x in pos])
                pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
                goal.trajectory.points.append(pt)
            send = self.fjt.send_goal_async(goal)
            rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
            res = send.result().get_result_async()
            rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
            code = res.result().result.error_code
            q = np.array(self.arm_q()); err = np.abs(q - np.array(targets[-1][0])).max()
            print(f'  traj code={code} max joint err={err:.4f}')
            if err < tol:
                return True
            targets = [(targets[-1][0], max(2.0, targets[-1][1] / 2))]
        return err < tol

    def move_tcp(self, tcp_world, quat, seconds=3.0, seed=None):
        q = self.solve_ik(tcp_world, quat, seed)
        if q is None:
            return False
        ok = self.move_joints(q, seconds)
        p, _ = self.tcp_world()
        print('  tcp now', p.round(4), 'target', np.round(tcp_world, 4))
        return ok

    def gripper(self, width):
        g = GripperCommand.Goal()
        g.command.position = float(width); g.command.max_effort = float(GRIP['max_effort'])
        fut = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=300)
        r = res.result().result
        f = self.fingers()
        print(f'  gripper reached={r.reached_goal} stalled={r.stalled} fingers={f[0]:.4f},{f[1]:.4f}')
        return f

    def grab(self, topic, msg_type, timeout=30.0):
        got = {}
        sub = self.node.create_subscription(msg_type, topic, lambda m: got.setdefault('m', m), 1)
        t0 = time.time()
        while 'm' not in got and time.time() - t0 < timeout:
            self.spin(0.2)
        self.node.destroy_subscription(sub)
        return got.get('m')

    def snap(self, cam, out=None):
        m = self.grab(f'/{cam}/color/image_raw', Image)
        img = self.bridge.imgmsg_to_cv2(m, 'bgr8')
        if out:
            import cv2; cv2.imwrite(out, img)
        return img

    def depth(self, cam):
        m = self.grab(f'/{cam}/depth/image_raw', Image)
        return self.bridge.imgmsg_to_cv2(m, 'passthrough').astype(np.float32)

    def cam_info(self, cam):
        m = self.grab(f'/{cam}/color/camera_info', CameraInfo)
        return np.array(m.k).reshape(3, 3)

    def cam_tf(self, cam):
        frame = f'{cam}_optical_frame'
        for _ in range(50):
            self.spin(0.1)
            if self.tfbuf.can_transform('world', frame, rclpy.time.Time()):
                break
        t = self.tfbuf.lookup_transform('world', frame, rclpy.time.Time())
        tr = t.transform.translation; q = t.transform.rotation
        return np.array([tr.x, tr.y, tr.z]), quat_to_R(q.x, q.y, q.z, q.w)

    def cloud(self, cam, cam_pos=None, cam_R=None):
        """Depth image -> world XYZ arrays (H,W,3)."""
        d = self.depth(cam); K = self.cam_info(cam)
        if cam_pos is None:
            cam_pos, cam_R = self.cam_tf(cam)
        H, W = d.shape
        v, u = np.mgrid[0:H, 0:W]
        x = (u - K[0, 2]) * d / K[0, 0]; y = (v - K[1, 2]) * d / K[1, 1]
        P = np.stack([x, y, d], -1) @ cam_R.T + cam_pos
        return P, d
