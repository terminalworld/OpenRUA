#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 2 -type f | head -50

# openrua op 1
cat /workspace/tools/README.md; ls -R /workspace/tools; cat /workspace/.gitignore; ls -la /workspace/.ros

# openrua op 2
cat tools/perception/cam_snap.py tools/perception/px2world.py

# openrua op 3
cat tools/action/fjt_send.py tools/action/gripper_cmd.py tools/action/ik_move.py

# openrua op 4
ros2 topic list && echo --- && ros2 action list && echo --- && ros2 service list | grep -v parameter && echo --- && ros2 node list

# openrua op 5
ros2 topic echo /joint_states --once && for c in agentview birdview frontview sideview robot0_eye_in_hand; do python3 tools/perception/cam_snap.py $c; done

# openrua op 6
timeout 20 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -12; echo ---; ros2 topic echo /birdview/color/camera_info --once | grep -A12 '^k:'; echo ---; for p in "245 288" "392 288" "285 265" "350 270" "315 240"; do echo -n "$p -> "; python3 tools/perception/px2world.py birdview $p; done

# openrua op 7
mkdir -p "$(dirname /workspace/scan.py)"
cat > /workspace/scan.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Point-cloud the birdview depth into world frame and cluster objects above the table."""
import struct, sys
import numpy as np
import rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge
import cv2

cam = sys.argv[1] if len(sys.argv) > 1 else "birdview"

def grab(node, topic, T):
    got = {}
    sub = node.create_subscription(T, topic, lambda m: got.setdefault("m", m), 1)
    while "m" not in got:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    return got["m"]

rclpy.init(); node = rclpy.create_node("scan")
tfbuf = Buffer(); TransformListener(tfbuf, node)
depth = CvBridge().imgmsg_to_cv2(grab(node, f"/{cam}/depth/image_raw", Image), "passthrough").astype(np.float64)
color = CvBridge().imgmsg_to_cv2(grab(node, f"/{cam}/color/image_raw", Image), "bgr8")
info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
frame = f"{cam}_optical_frame"
while not tfbuf.can_transform("world", frame, rclpy.time.Time()):
    rclpy.spin_once(node, timeout_sec=0.2)
t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
q = t.transform.rotation; x, y, z, w = q.x, q.y, q.z, q.w
R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
              [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
              [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
tr = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
H, W = depth.shape
vv, uu = np.mgrid[0:H, 0:W]
Z = depth
X = (uu - cx) * Z / fx; Y = (vv - cy) * Z / fy
P = np.stack([X, Y, Z], -1) @ R.T + tr  # world coords per pixel
np.save(f"{cam}_world.npy", P)
zw = P[..., 2]
valid = np.isfinite(zw)
# table height: mode of z in the central region
tab = np.median(zw[valid & (np.abs(P[...,0]) < 0.3) & (np.abs(P[...,1]) < 0.5)])
print("table z ~", round(tab, 4))
mask = (valid & (zw > tab + 0.02) & (zw < tab + 0.4)).astype(np.uint8)
n, lab, stats, cents = cv2.connectedComponentsWithStats(mask)
for i in range(1, n):
    if stats[i, cv2.CC_STAT_AREA] < 30: continue
    m = lab == i
    pts = P[m]
    bgr = color[m].mean(0)
    print(f"blob {i}: px_centroid=({cents[i][0]:.0f},{cents[i][1]:.0f}) area={stats[i,4]} "
          f"world x=[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y=[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] "
          f"ztop={pts[:,2].max():.3f} mean=({pts[:,0].mean():.3f},{pts[:,1].mean():.3f}) color(bgr)={bgr.round()}")
rclpy.shutdown()
OPENRUA_EOF

# openrua op 8
python3 scan.py birdview

# openrua op 9
mkdir -p "$(dirname /workspace/fk.py)"
cat > /workspace/fk.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Print the current hand pose (panda_hand, and TCP) in panda_link0 and world frames via /compute_fk."""
import numpy as np, rclpy, yaml
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState

M = yaml.safe_load(open("/workspace/machine.yaml"))
ARM = M["actuators"][0]["joints"]
BASE = np.array([-0.51, 0.0, 0.42])  # world -> panda_link0 (identity rotation)
TCP = M["hand"]["tcp_offset_m"]

def quat_R(x, y, z, w):
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])

def main():
    rclpy.init(); node = rclpy.create_node("fk")
    js = {}
    node.create_subscription(JointState, "/joint_states", lambda m: js.setdefault("m", m), 1)
    while "m" not in js: rclpy.spin_once(node, timeout_sec=0.2)
    d = dict(zip(js["m"].name, js["m"].position))
    print("joints:", ",".join(f"{d[j]:.4f}" for j in ARM))
    print("fingers:", d.get("panda_finger_joint1"), d.get("panda_finger_joint2"))
    cli = node.create_client(GetPositionFK, "/compute_fk")
    cli.wait_for_service(10)
    req = GetPositionFK.Request()
    req.fk_link_names = ["panda_hand"]
    req.robot_state.joint_state.name = ARM
    req.robot_state.joint_state.position = [d[j] for j in ARM]
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=30)
    res = fut.result()
    p = res.pose_stamped[0].pose
    pos = np.array([p.position.x, p.position.y, p.position.z])
    q = p.orientation
    R = quat_R(q.x, q.y, q.z, q.w)
    tcp = pos + TCP * R[:, 2]
    print(f"hand(base): {pos.round(4)} quat xyzw=({q.x:.4f},{q.y:.4f},{q.z:.4f},{q.w:.4f})")
    print(f"hand(world): {(pos+BASE).round(4)}   tcp(world): {(tcp+BASE).round(4)}")
    print("hand z-axis (approach) in world:", R[:, 2].round(3), " x-axis (finger-open dir):", R[:, 0].round(3))
    rclpy.shutdown()

if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 10
python3 fk.py

# openrua op 11
timeout 8 ros2 run tf2_ros tf2_echo panda_link0 panda_hand 2>&1 | grep -A1 Translation | head -3; ros2 topic hz /joint_states --window 5 2>&1 | head -3 & sleep 6; kill %1 2>/dev/null; ros2 topic info /joint_states -v | grep -E 'Node name|Publisher count|Subscription count'

# openrua op 12
mkdir -p "$(dirname /workspace/iktest.py)"
cat > /workspace/iktest.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
import sys, numpy as np, rclpy
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from sensor_msgs.msg import JointState
ARM = [f"panda_joint{i}" for i in range(1, 8)]
rclpy.init(); node = rclpy.create_node("iktest")
js = {}
node.create_subscription(JointState, "/joint_states", lambda m: js.setdefault("m", m), 1)
while "m" not in js: rclpy.spin_once(node, timeout_sec=0.2)
d = dict(zip(js["m"].name, js["m"].position))
cur = [d[j] for j in ARM]
fk = node.create_client(GetPositionFK, "/compute_fk"); fk.wait_for_service(10)
req = GetPositionFK.Request(); req.fk_link_names = ["panda_hand"]
req.robot_state.joint_state.name = ARM; req.robot_state.joint_state.position = cur
fut = fk.call_async(req); rclpy.spin_until_future_complete(node, fut, timeout_sec=30)
ps = fut.result().pose_stamped[0]
print("FK header frame:", repr(ps.header.frame_id), "pos:", ps.pose.position)
ik = node.create_client(GetPositionIK, "/compute_ik"); ik.wait_for_service(10)
for label, off in [("world-coords", (0, 0, 0)), ("base-coords", (0.51, 0, -0.42))]:
    r = GetPositionIK.Request(); r.ik_request.group_name = "panda_arm"
    r.ik_request.pose_stamped.header.frame_id = ""
    r.ik_request.pose_stamped.pose = ps.pose
    r.ik_request.pose_stamped.pose.position.x = ps.pose.position.x + off[0]
    r.ik_request.pose_stamped.pose.position.y = ps.pose.position.y + off[1]
    r.ik_request.pose_stamped.pose.position.z = ps.pose.position.z + off[2]
    r.ik_request.robot_state.joint_state.name = ARM
    r.ik_request.robot_state.joint_state.position = cur
    r.ik_request.timeout.sec = 2
    f = ik.call_async(r); rclpy.spin_until_future_complete(node, f, timeout_sec=60)
    res = f.result()
    if res is None or res.error_code.val != 1:
        print(label, "IK failed", None if res is None else res.error_code.val); continue
    sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
    print(label, "sol:", [round(sol[j], 3) for j in ARM], "cur:", [round(c, 3) for c in cur])
rclpy.shutdown()
OPENRUA_EOF

# openrua op 13
python3 iktest.py

# openrua op 14
python3 -c "
import numpy as np
P=np.load('birdview_world.npy'); z=P[...,2]
# table-region z histogram
sel=(np.abs(P[...,0])<0.35)&(np.abs(P[...,1])<0.6)&np.isfinite(z)
h,e=np.histogram(z[sel],bins=np.arange(0.38,0.7,0.01)); 
for a,b in zip(e[:-1],h): print(f'{a:.2f} {b}')
print('plate L center pixel z:', z[287,248], ' plate R:', z[287,392], ' table next to plate:', z[287,200], z[330,248])
print('red mug top:', z[236,320], ' max in red mug region:', np.nanmax(z[225:250,305:335]))
"


# openrua op 15
python3 tools/action/ik_move.py -0.05 0 1.0 1 0 0 0 3 && python3 fk.py

# openrua op 16
mkdir -p "$(dirname /workspace/arm.py)"
cat > /workspace/arm.py <<'OPENRUA_EOF'
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
OPENRUA_EOF

# openrua op 17
python3 arm.py && python3 scan.py birdview && python3 tools/perception/cam_snap.py birdview && python3 tools/perception/cam_snap.py agentview

# openrua op 18
mkdir -p "$(dirname /workspace/mugs.py)"
cat > /workspace/mugs.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Fit rim circles to the mugs in birdview_world.npy (run scan.py first)."""
import numpy as np, cv2, sys
P = np.load("birdview_world.npy"); z = P[..., 2]
color = cv2.imread("birdview.png")
TABLE = 0.425
regions = {"white": (-0.088, -0.147), "yellow": (-0.029, 0.117), "red": (-0.216, -0.002)}
for name, (cx0, cy0) in regions.items():
    near = (np.abs(P[..., 0] - cx0) < 0.09) & (np.abs(P[..., 1] - cy0) < 0.09) & np.isfinite(z)
    obj = near & (z > TABLE + 0.03)
    pts = P[obj]
    ztop = pts[:, 2].max()
    # rim: highest 2 cm
    rim = pts[pts[:, 2] > ztop - 0.02]
    # algebraic circle fit x^2+y^2+Dx+Ey+F=0
    A = np.c_[rim[:, 0], rim[:, 1], np.ones(len(rim))]
    b = -(rim[:, 0] ** 2 + rim[:, 1] ** 2)
    D, E, F = np.linalg.lstsq(A, b, rcond=None)[0]
    cx, cy = -D / 2, -E / 2; r = np.sqrt(cx * cx + cy * cy - F)
    # handle: object pixels farther than r+1.5cm from center
    d = np.hypot(pts[:, 0] - cx, pts[:, 1] - cy)
    h = pts[d > r + 0.012]
    hdir = None
    if len(h):
        hv = h[:, :2].mean(0) - [cx, cy]; hdir = np.degrees(np.arctan2(hv[1], hv[0]))
    print(f"{name}: rim center=({cx:.4f},{cy:.4f}) r={r:.4f} (diam {2*r*100:.1f}cm) ztop={ztop:.3f} "
          f"height={ztop-TABLE:.3f} n_rim={len(rim)} handle_dir={None if hdir is None else round(hdir)}deg n_handle={len(h)} "
          f"body_z_range=[{pts[:,2].min():.3f},{ztop:.3f}]")
    # z profile of the mug: extent in xy at several heights
    for zl in np.arange(TABLE + 0.03, ztop, 0.02):
        s = pts[(pts[:, 2] > zl) & (pts[:, 2] <= zl + 0.02)]
        if len(s):
            dd = np.hypot(s[:, 0] - cx, s[:, 1] - cy)
            print(f"   z {zl:.3f}-{zl+0.02:.3f}: n={len(s)} maxdist={dd.max():.3f} mediandist={np.median(dd):.3f}")
OPENRUA_EOF

# openrua op 19
python3 mugs.py

# openrua op 20
sed -i 's/obj = near \& (z > TABLE + 0.03)/obj = near \& (z > TABLE + 0.03) \& (z < 0.7)/' mugs.py && python3 mugs.py

# openrua op 21
mkdir -p "$(dirname /workspace/task.py)"
cat > /workspace/task.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Phase-driven pick & place. Usage: python3 task.py <phase> [args]
Phases: pose | pregrasp X Y ZTOP YAW | descend Z | grasp | lift Z | carry X Y Z | place Z | release | retreat Z
World frame; X,Y = rim grasp point (TCP), YAW = hand x-axis yaw (fingers open along yaw+90deg)."""
import math, sys, json
import numpy as np
from arm import Arm, fmt, TCP, LIM

def ik_tcp(a, tcp, yaw, seed=None):
    hand = np.array(tcp, float) + np.array([0, 0, TCP])
    return a.ik_topdown(hand, yaw, seed=seed)

def cart_move(a, tcp_goal, yaw, seconds=None, steps=None):
    """Move TCP straight to tcp_goal keeping top-down orientation; IK per waypoint, one trajectory."""
    p = a.hand_pose(); start = p["tcp"]
    goal = np.array(tcp_goal, float)
    dist = np.linalg.norm(goal - start)
    n = steps or max(2, int(dist / 0.05) + 1)
    seconds = seconds or max(1.5, dist * 6)
    seed = p["q"]; via = []
    for i in range(1, n + 1):
        wp = start + (goal - start) * i / n
        q = ik_tcp(a, wp, yaw, seed=seed)
        if q is None:
            print(f"IK failed at waypoint {wp}"); return False
        if max(abs(x - y) for x, y in zip(q, seed)) > 1.0:
            print(f"WARNING big joint jump at waypoint {i}: {np.round(np.array(q)-np.array(seed),2)}")
        via.append(q); seed = q
    ok = a.move_to(via[-1], seconds) if n == 1 else _move_via(a, via, seconds)
    p = a.hand_pose(); err = np.linalg.norm(p["tcp"] - goal)
    print(f"  arrived: {fmt(p)}  tcp_err={err*1000:.1f}mm")
    return err < 0.008

def _move_via(a, via, seconds):
    code, err = a.move(via[-1], seconds, via=via[:-1])
    print(f"  move(via {len(via)}): code={code} max_joint_err={err:.4f}")
    if err > 0.02:
        code, err = a.move(via[-1], 2.0)
        print(f"  re-send final: code={code} max_joint_err={err:.4f}")
    return err < 0.02

def main():
    a = Arm()
    ph = sys.argv[1]; args = [float(x) for x in sys.argv[2:]]
    p = a.hand_pose(); print("start:", fmt(p))
    if ph == "pose":
        print("wrench:", a.wrench().round(2))
    elif ph == "goto":  # goto X Y Z YAW(deg): TCP target, straight line
        x, y, z, yaw = args; cart_move(a, [x, y, z], math.radians(yaw))
    elif ph == "grasp":
        a.gripper(0.0)
        print("wrench:", a.wrench().round(2))
    elif ph == "release":
        a.gripper(0.04)
    elif ph == "servo":  # servo VX VY VZ TICKS (m/s, base frame)
        vx, vy, vz, t = args; print(fmt(a.servo(vx, vy, vz, int(t))))
    print("end:", fmt(a.hand_pose()))

if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 22
python3 task.py goto -0.0548 0.152 0.62 0 2>&1 | tail -8

# openrua op 23
sed -i 's/seconds = seconds or max(1.5, dist \* 6)/seconds = seconds or max(2.5, dist * 10)/' task.py && python3 tools/perception/cam_snap.py robot0_eye_in_hand && python3 tools/perception/cam_snap.py agentview

# openrua op 24
python3 task.py goto -0.0548 0.152 0.511 0 2>&1 | tail -4 && python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 25
python3 task.py grasp 2>&1 | tail -3

# openrua op 26
python3 task.py goto -0.0548 0.152 0.76 0 2>&1 | tail -2 && python3 tools/perception/cam_snap.py agentview

# openrua op 27
python3 task.py goto 0.006 -0.261 0.76 0 2>&1 | grep -E 'arrived|WARN|fail' ; python3 task.py goto 0.006 -0.261 0.56 0 2>&1 | grep -E 'arrived|WARN|fail'; python3 tools/perception/cam_snap.py agentview

# openrua op 28
python3 task.py release 2>&1 | grep gripper; python3 task.py goto 0.006 -0.261 0.76 0 2>&1 | grep -E 'arrived|fail'; python3 tools/perception/cam_snap.py agentview

# openrua op 29
python3 scan.py birdview 2>&1 | grep -v 'blob 1:' && python3 mugs.py 2>&1 | grep -E '^(white|red|yellow)'

# openrua op 30
python3 task.py goto -0.0459 -0.1463 0.62 90 2>&1 | grep -E 'arrived|WARN|fail'; python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 31
python3 task.py goto -0.0459 -0.1463 0.62 90 2>&1 | grep -v XMLPARSER

# openrua op 32
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/task.py", "old_string": "    ok = a.move_to(via[-1], seconds) if n == 1 else _move_via(a, via, seconds)\\n    p = a.hand_pose(); err = np.linalg.norm(p[\\"tcp\\"] - goal)\\n    print(f\\"  arrived: {fmt(p)}  tcp_err={err*1000:.1f}mm\\")\\n    return err < 0.008", "new_string": "    ok = a.move_to(via[-1], seconds) if n == 1 else _move_via(a, via, seconds)\\n    for _ in range(3):\\n        p = a.hand_pose(); err = np.linalg.norm(p[\\"tcp\\"] - goal)\\n        dyaw = abs((yaw - p[\\"yaw\\"] + math.pi) % (2 * math.pi) - math.pi)\\n        print(f\\"  arrived: {fmt(p)}  tcp_err={err*1000:.1f}mm yaw_err={math.degrees(dyaw):.1f}deg\\")\\n        if err < 0.008 and dyaw < 0.05:\\n            return True\\n        print(\\"  re-sending final point\\")\\n        a.move_to(via[-1], 2.5)\\n    return False", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 33
python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 34
python3 task.py goto -0.0459 -0.1463 0.519 90 2>&1 | grep -E 'arrived|fail|re-send'; python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 35
python3 task.py grasp 2>&1 | grep -E 'gripper|wrench'; python3 task.py goto -0.0459 -0.1463 0.76 90 2>&1 | grep -E 'arrived|fail'; python3 tools/perception/cam_snap.py agentview

# openrua op 36
python3 task.py goto 0.048 0.317 0.76 90 2>&1 | grep -E 'arrived|fail|re-send'; python3 task.py goto 0.048 0.317 0.565 90 2>&1 | grep -E 'arrived|fail|re-send'; python3 tools/perception/cam_snap.py agentview

# openrua op 37
python3 task.py release 2>&1 | grep gripper; python3 task.py goto 0.0 0.15 0.85 0 2>&1 | grep -E 'arrived|fail|re-send'; python3 tools/perception/cam_snap.py agentview; python3 tools/perception/cam_snap.py birdview

# openrua op 38
python3 task.py goto 0.0 0.15 0.85 0 2>&1 | grep -v XMLPARSER | tail -5

# openrua op 39
python3 tools/perception/cam_snap.py agentview; python3 scan.py birdview 2>&1 | grep -v 'blob 1:'

# openrua op 40
python3 -c "
import numpy as np
P=np.load('birdview_world.npy'); z=P[...,2]
for name,(cx0,cy0),plate in [('yellow/white mug on LEFT plate',(0.002,-0.309),(0.006,-0.307)),('white mug on RIGHT plate',(-0.004,0.322),(0.003,0.317))]:
    near=(np.abs(P[...,0]-cx0)<0.1)&(np.abs(P[...,1]-cy0)<0.1)&np.isfinite(z)
    pts=P[near&(z>0.50)&(z<0.7)]; ztop=pts[:,2].max(); rim=pts[pts[:,2]>ztop-0.02]
    A=np.c_[rim[:,0],rim[:,1],np.ones(len(rim))]; b=-(rim[:,0]**2+rim[:,1]**2)
    D,E,F=np.linalg.lstsq(A,b,rcond=None)[0]; cx,cy=-D/2,-E/2; r=np.sqrt(cx*cx+cy*cy-F)
    off=np.hypot(cx-plate[0],cy-plate[1])
    print(f'{name}: rim center=({cx:.3f},{cy:.3f}) r={r:.3f} ztop={ztop:.3f}; plate center={plate}; offset from plate center={off*100:.1f}cm (plate radius ~6.5cm)')
"
