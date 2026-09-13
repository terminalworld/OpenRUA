"""Small controller: IK (MoveIt) -> FollowJointTrajectory, gripper, joint state, FK.
All public poses are in WORLD frame; converted to panda_link0 for MoveIt."""
import time, sys
import numpy as np, rclpy, yaml
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from scipy.spatial.transform import Rotation as Rot

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
JOINTS = FJT["joints"]
BASE_IN_WORLD = np.zeros(3)   # FK check: planner model frame == world on this machine
TCP_OFF = float(M["hand"]["tcp_offset_m"])

def log(*a):
    print(time.strftime("%H:%M:%S"), *a, flush=True)

class Robot:
    def __init__(self):
        if not rclpy.ok(): rclpy.init()
        self.node = rclpy.create_node("ctl")
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        assert self.fjt.wait_for_server(20), "no fjt"
        assert self.grip.wait_for_server(20), "no gripper"
        assert self.ik.wait_for_service(20), "no ik"
        self.fk.wait_for_service(5)

    def _on_js(self, m): self._js["m"] = m

    def spin(self, t=0.05): rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self, fresh=True):
        if fresh:
            self._js.pop("m", None)
        t0 = time.time()
        while "m" not in self._js and time.time() - t0 < 20: self.spin(0.1)
        m = self._js["m"]
        d = dict(zip(m.name, m.position))
        return d

    def arm_q(self):
        d = self.joints(); return np.array([d[j] for j in JOINTS])

    def fingers(self):
        d = self.joints(); return d["panda_finger_joint1"], d["panda_finger_joint2"]

    # ---- kinematics ----
    @staticmethod
    def quat_topdown(yaw_deg=0.0):
        """hand z pointing down (world -z); fingers open along world y rotated by yaw about z."""
        r = Rot.from_euler("z", yaw_deg, degrees=True) * Rot.from_quat([1, 0, 0, 0])
        return r.as_quat()  # x,y,z,w

    def ik_world(self, xyz_world, quat, at_tcp=True, seed=None, attempts=3):
        xyz = np.array(xyz_world, float) - BASE_IN_WORLD
        R = Rot.from_quat(quat).as_matrix()
        if at_tcp:
            xyz = xyz - TCP_OFF * R[:, 2]
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, xyz)
        # IK tip link is panda_link8 = panda_hand rotated +45deg about z (checked via /compute_fk)
        q8 = (Rot.from_quat(quat) * Rot.from_euler("z", 45, degrees=True)).as_quat()
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q8)
        req.ik_request.timeout.sec = 2
        req.ik_request.avoid_collisions = True
        q0 = self.arm_q() if seed is None else np.array(seed)
        for k in range(attempts):
            seedjs = JointState(); seedjs.name = list(JOINTS); seedjs.position = [float(v) for v in q0]
            req.ik_request.robot_state.joint_state = seedjs
            fut = self.ik.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            res = fut.result()
            if res is not None and res.error_code.val == 1:
                sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
                return np.array([sol[j] for j in JOINTS])
            log(f"IK attempt {k} failed code={None if res is None else res.error_code.val}")
            q0 = q0 + np.random.uniform(-0.3, 0.3, 7)
        return None

    def fk_world(self, q=None):
        """TCP pose in world via /compute_fk of panda_hand; returns (xyz_tcp, quat)."""
        if q is None: q = self.arm_q()
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = ["panda_hand"]
        js = JointState(); js.name = list(JOINTS); js.position = [float(v) for v in q]
        req.robot_state.joint_state = js
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            return None
        p = res.pose_stamped[0].pose
        quat = np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        xyz = np.array([p.position.x, p.position.y, p.position.z]) + BASE_IN_WORLD
        R = Rot.from_quat(quat).as_matrix()
        return xyz + TCP_OFF * R[:, 2], quat

    # ---- motion ----
    def move_q(self, q, seconds=3.0, waypoints=None, retries=2):
        # pace by the largest joint displacement (~0.6 rad/s max) and retry slower on tolerance violations
        dist = np.abs(self.arm_q() - np.array(q)).max()
        seconds = max(seconds, dist / 0.6)
        for attempt in range(retries + 1):
            ok = self._move_q_once(q, seconds, waypoints)
            if ok: return True
            seconds *= 1.6
            if waypoints:
                # drop waypoints already passed so the retry does not replay the path from its start
                cur = self.arm_q()
                pts = [w[0] for w in waypoints] + [np.array(q)]
                k = int(np.argmin([np.abs(p - cur).max() for p in pts]))
                rem = waypoints[k + 1:]
                if rem:
                    t0 = waypoints[k][1] if k < len(waypoints) else 0.0
                    scale = seconds / max(1e-6, (rem[-1][1] if rem else 1.0) - t0 + (waypoints[-1][1] - waypoints[-2][1] if len(waypoints) > 1 else 1.0))
                    waypoints = [(w[0], (w[1] - t0) * scale) for w in rem]
                else:
                    waypoints = None
            log(f"retrying slower: {seconds:.1f}s with {0 if not waypoints else len(waypoints)} waypoints")
        return False

    def _move_q_once(self, q, seconds, waypoints=None):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        pts = []
        wps = (waypoints or []) + [(q, seconds)]
        for qq, t in wps:
            pt = JointTrajectoryPoint(positions=[float(v) for v in qq])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            pts.append(pt)
        # settle point: hold the final target a moment so the goal-tolerance check sees a converged arm
        settle = JointTrajectoryPoint(positions=[float(v) for v in q])
        ts = seconds + 1.0
        settle.time_from_start = Duration(sec=int(ts), nanosec=int((ts % 1) * 1e9))
        pts.append(settle)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        h = send.result()
        if h is None or not h.accepted:
            log("FJT goal not accepted"); return False
        rf = h.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=600)
        code = rf.result().result.error_code if rf.result() else None
        err = np.abs(self.arm_q() - np.array(q)).max()
        log(f"move done code={code} max joint err={err:.4f}")
        return code == 0 and err < 0.05

    def move_world(self, xyz, quat, seconds=3.0, at_tcp=True, seed=None):
        q = self.ik_world(xyz, quat, at_tcp=at_tcp, seed=seed)
        if q is None:
            log("IK failed for", xyz); return None
        ok = self.move_q(q, seconds)
        return q if ok else None

    def gripper(self, width):
        g = GripperCommand.Goal(); g.command.position = float(width); g.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result
        f = self.fingers()
        log(f"gripper({width}) reached={r.reached_goal} stalled={r.stalled} fingers={f[0]:.4f},{f[1]:.4f}")
        return f

def _cont(seed, q):
    """wrap-free joint distance check"""
    return np.abs(np.array(q) - np.array(seed)).max()

def move_line(self, xyz_target, quat, step=0.02, speed=0.06, at_tcp=True, max_jump=0.35):
    """Straight Cartesian line from the current TCP to xyz_target as one multi-point trajectory."""
    start, _ = self.fk_world()
    target = np.array(xyz_target, float)
    n = max(1, int(np.ceil(np.linalg.norm(target - start) / step)))
    seed = self.arm_q()
    wps = []
    t = 0.0
    dt = max(0.3, (np.linalg.norm(target - start) / n) / speed)
    for i in range(1, n + 1):
        p = start + (target - start) * i / n
        q = self.ik_world(p, quat, at_tcp=at_tcp, seed=seed, attempts=1)
        if q is None or _cont(seed, q) > max_jump:
            log(f"line: IK jump/fail at waypoint {i}/{n} {p.round(3)} " +
                ("" if q is None else f"jump={_cont(seed, q):.2f}"))
            return False
        t += dt
        wps.append((q, t))
        seed = q
    qf, tf = wps[-1]
    ok = self.move_q(qf, tf, waypoints=wps[:-1])
    pos, _ = self.fk_world()
    log(f"line end tcp {pos.round(4)} (target {target.round(4)}) err {np.linalg.norm(pos-target)*1000:.1f} mm")
    return ok
Robot.move_line = move_line

LIMITS = np.array(FJT["limits_rad"])

def plan_descent(self, xyz_grasp, quat, z_top, step=0.02, n_seeds=25, max_jump=0.35, margin=0.12):
    """Find a grasp IK solution + a joint-continuous straight vertical path from z_top down to it.
    Returns (chain) where chain[0] is at z_top and chain[-1] at the grasp, or None."""
    xyz_grasp = np.array(xyz_grasp, float)
    rng = np.random.default_rng(0)
    cands = []
    seeds = [self.arm_q()] + [rng.uniform(LIMITS[:, 0] + 0.2, LIMITS[:, 1] - 0.2) for _ in range(n_seeds)]
    for s in seeds:
        q = self.ik_world(xyz_grasp, quat, seed=s, attempts=1)
        if q is None: continue
        if any(np.abs(q - c).max() < 0.05 for c in cands): continue
        if (q - LIMITS[:, 0] < margin).any() or (LIMITS[:, 1] - q < margin).any(): continue
        cands.append(q)
    log(f"plan_descent: {len(cands)} distinct grasp solutions")
    best = None
    n = int(np.ceil((z_top - xyz_grasp[2]) / step))
    for q in cands:
        chain = [q]; ok = True; seed = q
        for i in range(1, n + 1):
            p = xyz_grasp + [0, 0, (z_top - xyz_grasp[2]) * i / n]
            qi = self.ik_world(p, quat, seed=seed, attempts=1)
            if qi is None or np.abs(qi - seed).max() > max_jump: ok = False; break
            chain.append(qi); seed = qi
        if not ok: continue
        length = sum(np.abs(chain[i + 1] - chain[i]).sum() for i in range(len(chain) - 1))
        lim = min((q - LIMITS[:, 0]).min(), (LIMITS[:, 1] - q).min())
        log(f"  candidate q={q.round(2)} pathlen={length:.2f} limit-margin={lim:.2f}")
        approach = np.abs(chain[-1] - self.arm_q()).max()   # how far the chain top is from where we are
        score = length + 1.5 * approach - 0.5 * lim
        log(f"    approach-jump={approach:.2f} score={score:.2f}")
        if best is None or score < best[0]: best = (score, chain[::-1])
    return None if best is None else best[1]
Robot.plan_descent = plan_descent

def run_chain(self, chain, speed=0.05, step=0.02):
    """Execute a joint chain as one trajectory (first element should be near the current config)."""
    dt = max(0.4, step / speed)
    wps = [(q, dt * (i + 1)) for i, q in enumerate(chain[1:])]
    qf, tf = wps[-1]
    return self.move_q(qf, tf, waypoints=wps[:-1])
Robot.run_chain = run_chain
