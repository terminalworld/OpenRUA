"""Reusable robot helper: FK/IK, trajectory, gripper, joint state."""
import math, time, numpy as np, rclpy, yaml
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation as Rot

M = yaml.safe_load(open("/workspace/machine.yaml"))
ARM = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
JOINTS = ARM["joints"]
BASE = np.zeros(3)  # FK/IK services already answer in the world frame (verified)
TCP = M["hand"]["tcp_offset_m"]

class Robot(Node):
    def __init__(self):
        super().__init__("rob")
        self.js = None
        self.create_subscription(JointState, "/joint_states", self._js, 10)
        self.ik = self.create_client(GetPositionIK, "/compute_ik")
        self.fk = self.create_client(GetPositionFK, "/compute_fk")
        self.traj = ActionClient(self, FollowJointTrajectory, ARM["port"])
        self.grip = ActionClient(self, GripperCommand, GRIP["port"])
        self.ik.wait_for_service(10); self.fk.wait_for_service(10)
        self.traj.wait_for_server(10); self.grip.wait_for_server(10)
        while self.js is None: rclpy.spin_once(self, timeout_sec=0.2)

    def _js(self, m): self.js = m

    def joints(self, fresh=True):
        if fresh:
            self.js = None
            while self.js is None: rclpy.spin_once(self, timeout_sec=0.2)
        d = dict(zip(self.js.name, self.js.position))
        return [d[j] for j in JOINTS], d

    def finger_gap(self):
        _, d = self.joints()
        return d["panda_finger_joint1"] - d["panda_finger_joint2"]

    def _spin(self, fut, timeout=600):
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return fut.result()

    def fk_world(self, q=None):
        """hand pose in world: (pos[3], quat xyzw)."""
        if q is None: q, _ = self.joints()
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = JOINTS
        req.robot_state.joint_state.position = list(q)
        res = self._spin(self.fk.call_async(req), 60)
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE
        return pos, np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])

    def ik_world(self, pos, quat, seed=None, at_tcp=True):
        """IK for hand pose in world (pos of TCP if at_tcp). returns joint list or None."""
        pos = np.array(pos, float)
        if at_tcp:
            R = Rot.from_quat(quat).as_matrix()
            pos = pos - TCP * R[:, 2]
        pos = pos - BASE
        # the IK tip is panda_link8 = panda_hand rotated +45deg about z (verified via FK)
        quat = (Rot.from_quat(quat) * Rot.from_euler("z", 45, degrees=True)).as_quat()
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = 2
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = pos
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = quat
        if seed is None: seed, _ = self.joints()
        req.ik_request.robot_state.joint_state.name = JOINTS
        req.ik_request.robot_state.joint_state.position = list(seed)
        res = self._spin(self.ik.call_async(req), 60)
        if res is None or res.error_code.val != 1:
            return None
        d = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [d[j] for j in JOINTS]

    def move_joints(self, waypoints, seconds):
        """waypoints: list of joint lists; seconds: total or list of cumulative times."""
        if isinstance(seconds, (int, float)):
            n = len(waypoints); seconds = [seconds * (i + 1) / n for i in range(n)]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        for q, t in zip(waypoints, seconds):
            pt = JointTrajectoryPoint(positions=[float(v) for v in q])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        gh = self._spin(self.traj.send_goal_async(goal), 60)
        res = self._spin(gh.get_result_async(), 900)
        code = res.result.error_code
        q, _ = self.joints()
        err = max(abs(a - b) for a, b in zip(q, waypoints[-1]))
        return code, err

    def gripper(self, width):
        g = GripperCommand.Goal()
        g.command.position = float(width); g.command.max_effort = float(GRIP["max_effort"])
        gh = self._spin(self.grip.send_goal_async(g), 60)
        res = self._spin(gh.get_result_async(), 300)
        return res.result.reached_goal, res.result.stalled, self.finger_gap()

def quat_down(yaw_deg=0.0):
    """hand z pointing down (world -z), fingers closing along world axis rotated by yaw about z.
    yaw=0: hand y axis along world y."""
    R = Rot.from_euler("xyz", [180, 0, yaw_deg], degrees=True)
    return R.as_quat()

def init():
    rclpy.init()
    return Robot()

# ---- task-specific frames ----
TH = np.radians(33.5)
D = np.array([np.cos(TH), np.sin(TH), 0.0])      # bottle axis, neck -> base, when lying in the tray
N = np.array([-np.sin(TH), np.cos(TH), 0.0])     # horizontal, perpendicular to D (finger closing axis)

def R_final():
    """hand orientation for the horizontal bottle: hand z (palm->fingertips) = D (from neck toward base),
    fingers pinching horizontally along N; hand x = y cross z (vertical)."""
    zf = D; yf = -N; xf = np.cross(yf, zf)   # fingers symmetric: sign chosen to keep j7 away from its limit
    return np.column_stack([xf, yf, zf])

def R_grasp():
    """top-down neck grasp orientation: 90deg swing about the horizontal axis brings it to R_final."""
    return R_mid(0.0)

def R_mid(frac):
    Rf = R_final(); zg = np.array([0.0, 0.0, -1.0]); zf = Rf[:, 2]
    k = np.cross(zg, zf); k /= np.linalg.norm(k)
    Rrot_full = Rot.from_rotvec(k * np.pi / 2).as_matrix()
    Rg = Rrot_full.T @ Rf
    assert np.allclose(Rg[:, 2], zg, atol=1e-6)
    return Rot.from_rotvec(k * np.pi / 2 * frac).as_matrix() @ Rg

def q_of(R): return Rot.from_matrix(R).as_quat()
LINKS = ["panda_link1","panda_link2","panda_link3","panda_link4","panda_link5","panda_link6","panda_link7","panda_link8","panda_hand"]
def fk_links(self, q):
    req = GetPositionFK.Request()
    req.fk_link_names = LINKS
    req.robot_state.joint_state.name = JOINTS
    req.robot_state.joint_state.position = list(q)
    res = self._spin(self.fk.call_async(req), 60)
    return {n: np.array([p.pose.position.x, p.pose.position.y, p.pose.position.z]) for n, p in zip(LINKS, res.pose_stamped)}
Robot.fk_links = fk_links

def ik_best(self, pos, quat, seed, tries=8, at_tcp=True, max_step=None):
    """IK with several seeds (given seed + jittered); returns solution closest to seed in joint space."""
    rng = np.random.default_rng(0)
    best = None
    for i in range(tries):
        s = list(seed) if i == 0 else list(np.clip(np.array(seed) + rng.normal(0, 0.3 if i < 4 else 0.8, 7),
                                               [l[0] for l in ARM["limits_rad"]], [l[1] for l in ARM["limits_rad"]]))
        sol = self.ik_world(pos, quat, seed=s, at_tcp=at_tcp)
        if sol is None: continue
        d = max(abs(a - b) for a, b in zip(sol, seed))
        if best is None or d < best[0]: best = (d, sol)
    if best is None: return None
    if max_step is not None and best[0] > max_step: return None
    return best[1]
Robot.ik_best = ik_best

def go(self, q_target, seconds=None, tol=0.02, attempts=5):
    """move to joint target; re-send until converged (controller lag gives -5 on long goals)."""
    q_target = [float(v) for v in q_target]
    for i in range(attempts):
        q, _ = self.joints()
        dist = max(abs(a - b) for a, b in zip(q, q_target))
        if dist < tol: return True, dist
        t = seconds if seconds is not None else max(2.0, min(10.0, dist * 4.0))
        code, err = self.move_joints([q_target], t)
        print(f"  go: attempt {i} dist {dist:.3f} t {t:.1f}s -> code {code} err {err:.4f}")
        if err < tol: return True, err
    return False, err
Robot.go = go

def ik_natural(self, pos, quat, at_tcp=True, tries=24, seed_hint=None, verbose=False):
    """sample many IK solutions and pick a 'natural' one: shoulder pointing at target (j1 ~ azimuth),
    j3 ~ 0, j5 ~ 0, elbow up; all wrist links above the table."""
    lim = np.array(ARM["limits_rad"])
    pos = np.array(pos, float)
    az = np.arctan2(pos[1] - (-0.0), pos[0] - (-0.66))
    rng = np.random.default_rng(1)
    cands = []
    seeds = [[az, -0.3, 0.0, -2.0, 0.0, 1.8, 0.785], [az, 0.3, 0.0, -1.8, 0.0, 2.2, 0.785],
             [az, -0.16, 0.0, -2.44, 0.0, 2.23, 0.785], [az, 0.6, 0.0, -1.5, 0.0, 2.1, 0.0]]
    if seed_hint is not None: seeds.insert(0, list(seed_hint))
    for i in range(tries):
        if i < len(seeds): s = seeds[i]
        else:
            b = seeds[i % len(seeds)]
            s = list(np.clip(np.array(b) + rng.normal(0, 0.5, 7), lim[:, 0], lim[:, 1]))
        sol = self.ik_world(pos, quat, seed=s, at_tcp=at_tcp)
        if sol is None: continue
        links = self.fk_links(sol)
        if min(links[k][2] for k in ("panda_link5", "panda_link6")) < 1.0: continue
        if min(links[k][2] for k in ("panda_link7", "panda_hand")) < 0.95: continue
        cost = 2 * abs(sol[0] - az) + abs(sol[2]) + abs(sol[4]) + 0.3 * abs(sol[6] - 0.785)
        if seed_hint is not None: cost += 0.5 * max(abs(a - b) for a, b in zip(sol, seed_hint))
        cands.append((cost, sol))
    if not cands: return None
    cands.sort(key=lambda c: c[0])
    if verbose:
        for c, s in cands[:5]: print(f"   cand cost {c:.2f} q {np.round(s,2)}")
    return cands[0][1]
Robot.ik_natural = ik_natural
