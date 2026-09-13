#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 2 -type f | head -50

# openrua op 1
cat README.md machine.yaml docs/10-machine.md docs/20-perception.md docs/30-action.md docs/40-patterns.md tools/README.md .gitignore; ls -la .ros tools

# openrua op 2
ls -la tools/action tools/perception; cat tools/perception/*.py tools/action/*.py

# openrua op 3
timeout 30 ros2 topic list; echo ---; timeout 30 ros2 action list; echo ---; timeout 30 ros2 service list | grep -iv parameter; echo ---; timeout 30 ros2 node list

# openrua op 4
timeout 60 ros2 topic echo /joint_states --once; for c in agentview birdview frontview sideview robot0_robotview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c; done

# openrua op 5
timeout 20 ros2 topic echo /tf_static --once 2>&1 | head -80; echo ----; timeout 20 ros2 topic echo /tf --once 2>&1 | grep -E "frame_id|child" | sort | uniq -c

# openrua op 6
timeout 20 ros2 topic echo /tf --once 2>&1 | grep -A 12 -E "child_frame_id: (panda_link0|agentview_optical_frame|birdview_optical_frame)"

# openrua op 7
timeout 20 ros2 topic echo /tf --once 2>&1 > /tmp/tf.yaml; grep -n -B1 -A11 "panda_link0\|agentview_optical\|birdview_optical" /tmp/tf.yaml

# openrua op 8
cat > /workspace/tfdump.py <<'EOF'
import rclpy, time
from tf2_msgs.msg import TFMessage
rclpy.init(); n = rclpy.create_node("tfdump")
seen = {}
def cb(m):
    for t in m.transforms:
        seen[(t.header.frame_id, t.child_frame_id)] = t.transform
n.create_subscription(TFMessage, "/tf", cb, 50)
n.create_subscription(TFMessage, "/tf_static", cb, 50)
t0 = time.time()
while time.time() - t0 < 3: rclpy.spin_once(n, timeout_sec=0.1)
for (p, c), t in sorted(seen.items()):
    tr, q = t.translation, t.rotation
    print(f"{p:>14} -> {c:<32} t=({tr.x:.4f},{tr.y:.4f},{tr.z:.4f}) q=({q.x:.4f},{q.y:.4f},{q.z:.4f},{q.w:.4f})")
EOF
timeout 60 python3 /workspace/tfdump.py

# openrua op 9
sed -i 's/while time.time() - t0 < 3/while time.time() - t0 < 12/' tfdump.py && timeout 60 python3 tfdump.py | grep world

# openrua op 10
python3 -c "
import cv2
im = cv2.imread('birdview.png')
crop = im[200:320, 200:440]
crop = cv2.resize(crop, None, fx=4, fy=4, interpolation=cv2.INTER_NEAREST)
cv2.imwrite('birdview_crop.png', crop)
"

# openrua op 11
cat > /workspace/px2w_batch.py <<'EOF'
"""Batch pixel->world for one camera. Usage: px2w_batch.py <cam> u,v [u,v ...]
Uses TF captured by spinning up to 15s (world->cam_optical published slowly)."""
import sys, struct, time, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage
cam = sys.argv[1]; pix = [tuple(map(int, a.split(','))) for a in sys.argv[2:]]
rclpy.init(); n = rclpy.create_node("px2w")
got = {}
n.create_subscription(Image, f"/{cam}/depth/image_raw", lambda m: got.setdefault("d", m), 1)
n.create_subscription(CameraInfo, f"/{cam}/color/camera_info", lambda m: got.setdefault("i", m), 1)
def cb(m):
    for t in m.transforms:
        if t.header.frame_id == "world" and t.child_frame_id == f"{cam}_optical_frame":
            got["tf"] = t.transform
n.create_subscription(TFMessage, "/tf", cb, 50)
t0 = time.time()
while not all(k in got for k in ("d", "i", "tf")) and time.time() - t0 < 30:
    rclpy.spin_once(n, timeout_sec=0.1)
d, info, t = got["d"], got["i"], got["tf"]
q = t.rotation; x, y, z, w = q.x, q.y, q.z, q.w
R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
              [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
              [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
T = np.array([t.translation.x, t.translation.y, t.translation.z])
fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
depth = np.frombuffer(d.data, dtype=np.float32).reshape(d.height, d.width)
for u, v in pix:
    zc = float(depth[v, u])
    p = R @ np.array([(u-cx)*zc/fx, (v-cy)*zc/fy, zc]) + T
    print(f"px({u},{v}) depth={zc:.4f} -> world ({p[0]:.4f}, {p[1]:.4f}, {p[2]:.4f})")
EOF
timeout 90 python3 px2w_batch.py birdview 247,284 390,283 275,257 324,234 320,150 200,250 450,250

# openrua op 12
timeout 90 python3 px2w_batch.py agentview 83,295 548,290 195,185 325,165 390,195 320,400 320,350 195,230 390,250 190,240

# openrua op 13
timeout 300 python3 tools/action/fjt_send.py 0.0,-0.161037389,0.0,-2.44459747,0.0,2.2267522,0.7853981633974483 2.0 && timeout 90 python3 px2w_batch.py agentview 83,295 548,290 320,400 && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 14
timeout 60 python3 tools/perception/cam_snap.py birdview && python3 -c "
import cv2
im = cv2.imread('birdview.png')
crop = im[200:320, 200:440]
crop = cv2.resize(crop, None, fx=4, fy=4, interpolation=cv2.INTER_NEAREST)
cv2.imwrite('birdview_crop.png', crop)
"

# openrua op 15
cat > /workspace/bird_objs.py <<'EOF'
"""Segment objects above the table in the birdview depth image; print world centroid/extents."""
import time, numpy as np, rclpy, cv2
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage
cam = "birdview"
rclpy.init(); n = rclpy.create_node("bo")
got = {}
n.create_subscription(Image, f"/{cam}/depth/image_raw", lambda m: got.setdefault("d", m), 1)
n.create_subscription(Image, f"/{cam}/color/image_raw", lambda m: got.setdefault("c", m), 1)
n.create_subscription(CameraInfo, f"/{cam}/color/camera_info", lambda m: got.setdefault("i", m), 1)
def cb(m):
    for t in m.transforms:
        if t.header.frame_id == "world" and t.child_frame_id == f"{cam}_optical_frame":
            got["tf"] = t.transform
n.create_subscription(TFMessage, "/tf", cb, 50)
t0 = time.time()
while not all(k in got for k in ("d", "i", "tf", "c")) and time.time() - t0 < 30:
    rclpy.spin_once(n, timeout_sec=0.1)
d, info, t = got["d"], got["i"], got["tf"]
q = t.rotation; x, y, z, w = q.x, q.y, q.z, q.w
R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
              [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
              [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
T = np.array([t.translation.x, t.translation.y, t.translation.z])
fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
depth = np.frombuffer(d.data, dtype=np.float32).reshape(d.height, d.width)
vv, uu = np.mgrid[0:d.height, 0:d.width]
P = np.stack([(uu-cx)*depth/fx, (vv-cy)*depth/fy, depth], -1) @ R.T + T
color = np.frombuffer(got["c"].data, dtype=np.uint8).reshape(d.height, d.width, -1)[..., :3]
Z = P[..., 2]
mask = ((Z > 0.435) & (Z < 0.7)).astype(np.uint8)
nlab, lab, stats, cents = cv2.connectedComponentsWithStats(mask)
for i in range(1, nlab):
    if stats[i, cv2.CC_STAT_AREA] < 15: continue
    sel = lab == i
    pts = P[sel]
    col = color[sel].mean(0)
    print(f"blob{i}: px area={stats[i,4]} centroid px=({cents[i][0]:.0f},{cents[i][1]:.0f}) "
          f"world xy=({pts[:,0].mean():.3f},{pts[:,1].mean():.3f}) "
          f"x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] "
          f"zmax={pts[:,2].max():.3f} zmin={pts[:,2].min():.3f} rgb={col.round(0)}")
EOF
timeout 90 python3 bird_objs.py

# openrua op 16
cat >> /workspace/bird_objs.py <<'EOF'

print("--- rim fits (z>0.50) ---")
rim = ((Z > 0.50) & (Z < 0.7)).astype(np.uint8)
nlab, lab, stats, cents = cv2.connectedComponentsWithStats(rim)
for i in range(1, nlab):
    if stats[i, cv2.CC_STAT_AREA] < 10: continue
    pts = P[lab == i][:, :2]
    # algebraic circle fit
    A = np.c_[2*pts[:,0], 2*pts[:,1], np.ones(len(pts))]
    b = (pts**2).sum(1)
    (a, bb, c), *_ = np.linalg.lstsq(A, b, rcond=None)
    r = np.sqrt(c + a*a + bb*bb)
    print(f"rim{i}: n={len(pts)} center=({a:.3f},{bb:.3f}) r={r:.3f} zmax={P[lab==i][:,2].max():.3f}")
EOF
timeout 90 python3 bird_objs.py | grep -A20 rim; timeout 20 ros2 topic echo /birdview/color/camera_info --once | grep -A3 "^k:"

# openrua op 17
cat >> /workspace/bird_objs.py <<'EOF'

print("--- rim fits excluding handle side ---")
def fit(pts):
    A = np.c_[2*pts[:,0], 2*pts[:,1], np.ones(len(pts))]
    b = (pts**2).sum(1)
    (a, bb, c), *_ = np.linalg.lstsq(A, b, rcond=None)
    return a, bb, np.sqrt(c + a*a + bb*bb)
sel = (Z > 0.50) & (P[...,0] < -0.09) & (P[...,0] > -0.18) & (P[...,1] < -0.16) & (P[...,1] > -0.25)
print("white body-side rim:", fit(P[sel][:, :2]), len(P[sel]))
sel = (Z > 0.50) & (P[...,0] < -0.18) & (P[...,0] > -0.28) & (P[...,1] > 0.0) & (P[...,1] < 0.08)
print("yellow body-side rim:", fit(P[sel][:, :2]), len(P[sel]))
# save a depth-coded zoom for inspection
zc = np.clip((Z - 0.42) / 0.16, 0, 1)
img = cv2.applyColorMap((zc*255).astype(np.uint8), cv2.COLORMAP_JET)
img[Z < 0.435] = color[Z < 0.435][..., ::-1] // 2
crop = cv2.resize(img[210:300, 240:420], None, fx=6, fy=6, interpolation=cv2.INTER_NEAREST)
cv2.imwrite("bird_depth_zoom.png", crop)
EOF
timeout 90 python3 bird_objs.py | tail -3

# openrua op 18
cat > /workspace/rob.py <<'EOF'
"""Reusable robot helpers for this Panda workstation (clients built once per process)."""
import time, math, numpy as np, rclpy, yaml
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState, Image, CameraInfo
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from geometry_msgs.msg import TwistStamped, WrenchStamped
from tf2_msgs.msg import TFMessage

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
TW = next(a for a in M["actuators"] if a["kind"] == "cartesian_twist")
JOINTS = FJT["joints"]
BASE_IN_WORLD = np.array([-0.51, 0.0, 0.42])   # world -> panda_link0 (from /tf)
TCP_OFF = float(M["hand"]["tcp_offset_m"])

def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)]])

def topdown_quat(yaw):
    """Hand z pointing down (world -z); hand x rotated by `yaw` about world z.
    yaw=0 -> fingers close along world y."""
    # R = Rz(yaw) * Rx(pi)
    cy, sy = math.cos(yaw/2), math.sin(yaw/2)
    # q_z(yaw) * q_x(pi) with q_x(pi) = (1,0,0,0)
    qz = (0, 0, sy, cy)
    # quaternion product qz * qx
    x1, y1, z1, w1 = qz; x2, y2, z2, w2 = (1, 0, 0, 0)
    return (w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2,
            w1*w2 - x1*x2 - y1*y2 - z1*z2)

class Robot:
    def __init__(self, name="rob"):
        if not rclpy.ok():
            rclpy.init()
        self.n = rclpy.create_node(name)
        self.js = {}
        self.n.create_subscription(JointState, "/joint_states", self._js_cb, 10)
        self.fjt = ActionClient(self.n, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.n, GripperCommand, GRIP["port"])
        self.ik = self.n.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.n.create_client(GetPositionFK, "/compute_fk")
        self.twist_pub = self.n.create_publisher(TwistStamped, TW["port"], 10)
        self.wrench = {}
        self.n.create_subscription(WrenchStamped, "/franka_robot_state_broadcaster/external_wrench",
                                   lambda m: self.wrench.update(m=m), 10)
        self.fjt.wait_for_server(10); self.grip.wait_for_server(10)
        self.ik.wait_for_service(10); self.fk.wait_for_service(10)
        self.spin(0.5)

    def _js_cb(self, m):
        self.js = dict(zip(m.name, m.position))

    def spin(self, sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(self.n, timeout_sec=0.05)

    def joints(self):
        self.js = {}
        while len(self.js) < 7:
            rclpy.spin_once(self.n, timeout_sec=0.1)
        return [self.js[j] for j in JOINTS]

    def fingers(self):
        self.joints()
        return self.js.get("panda_finger_joint1"), self.js.get("panda_finger_joint2")

    def _arm_state(self):
        q = self.joints()
        s = JointState(); s.name = list(JOINTS); s.position = list(q)
        return s

    def hand_pose_world(self):
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state = self._arm_state()
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.n, fut, timeout_sec=30)
        r = fut.result()
        p = r.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE_IN_WORLD
        return pos, (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w), r.pose_stamped[0].header.frame_id

    def tcp_pose_world(self):
        pos, q, f = self.hand_pose_world()
        R = quat_to_R(*q)
        return pos + TCP_OFF * R[:, 2], q

    def ik_world(self, pos_world, quat, at_tcp=True, seed=None):
        pos = np.array(pos_world, float)
        if at_tcp:
            pos = pos - TCP_OFF * quat_to_R(*quat)[:, 2]
        pos = pos - BASE_IN_WORLD
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pos)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        s = self._arm_state()
        if seed is not None:
            s.position = list(seed)
        req.ik_request.robot_state.joint_state = s
        req.ik_request.avoid_collisions = False
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.n, fut, timeout_sec=60)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            return None
        sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
        return [sol[j] for j in JOINTS]

    def move_joints(self, targets, seconds, waypoints=None):
        """targets: list of 7; waypoints: optional list of (positions, t)."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        pts = []
        for pos, t in (waypoints or []) + [(targets, seconds)]:
            pt = JointTrajectoryPoint(positions=[float(x) for x in pos])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            pts.append(pt)
        goal.trajectory.points = pts
        fut = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.n, fut, timeout_sec=60)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.n, res, timeout_sec=600)
        code = res.result().result.error_code
        self.spin(0.3)
        q = self.joints()
        err = max(abs(a - b) for a, b in zip(q, targets))
        print(f"  traj done code={code} max_joint_err={err:.4f}")
        return code, err

    def move_tcp(self, pos_world, quat, seconds=3.0, seed=None):
        q = self.ik_world(pos_world, quat, at_tcp=True, seed=seed)
        if q is None:
            print(f"  IK FAILED for tcp {np.round(pos_world,3)}")
            return None
        # sanity: joint limits
        for v, (lo, hi) in zip(q, FJT["limits_rad"]):
            if not lo <= v <= hi:
                print(f"  IK solution outside limits: {np.round(q,3)}")
                return None
        code, err = self.move_joints(q, seconds)
        tcp, _ = self.tcp_pose_world()
        print(f"  tcp now {np.round(tcp,4)} (target {np.round(pos_world,4)})")
        return q

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.n, fut, timeout_sec=30)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.n, res, timeout_sec=120)
        r = res.result().result
        self.spin(0.3)
        f = self.fingers()
        print(f"  gripper -> {width}: reached={r.reached_goal} stalled={r.stalled} fingers={f}")
        return f

    def servo(self, vx=0, vy=0, vz=0, ticks=20):
        msg = TwistStamped(); msg.header.frame_id = TW["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = float(vx), float(vy), float(vz)
        for _ in range(ticks):
            msg.header.stamp = self.n.get_clock().now().to_msg()
            self.twist_pub.publish(msg)
            rclpy.spin_once(self.n, timeout_sec=0.05)
        stop = TwistStamped(); stop.header.frame_id = TW["frame"]
        for _ in range(3):
            self.twist_pub.publish(stop); rclpy.spin_once(self.n, timeout_sec=0.05)
        self.spin(0.3)

    def read_wrench(self):
        self.wrench = {}
        t0 = time.time()
        while "m" not in self.wrench and time.time() - t0 < 5:
            rclpy.spin_once(self.n, timeout_sec=0.1)
        m = self.wrench.get("m")
        if m is None: return None
        f = m.wrench.force
        return (f.x, f.y, f.z)
EOF
cat > /workspace/s0_fk.py <<'EOF'
import numpy as np
from rob import Robot, topdown_quat, quat_to_R
r = Robot("s0")
print("joints", np.round(r.joints(), 4))
pos, q, frame = r.hand_pose_world()
print("hand world", np.round(pos, 4), "quat", np.round(q, 4), "frame", frame)
print("hand R:\n", np.round(quat_to_R(*q), 3))
print("tcp world", np.round(r.tcp_pose_world()[0], 4))
print("topdown yaw0", np.round(topdown_quat(0), 4), "R:\n", np.round(quat_to_R(*topdown_quat(0)), 3))
print("fingers", r.fingers(), "wrench", r.read_wrench())
EOF
timeout 120 python3 -u s0_fk.py

# openrua op 19
cat > /workspace/chk_fk.py <<'EOF'
import numpy as np, math
def Rq(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
def T(t,q):
    M=np.eye(4); M[:3,:3]=Rq(*q); M[:3,3]=t; return M
chain=[((0,0,0.333),(0,0,0,1)),((0,0,0),(-0.7048,-0.0569,-0.0569,0.7048)),((0,-0.316,0),(0.7071,0,0,0.7071)),
((0.0825,0,0),(-0.2415,-0.6646,0.6646,-0.2415)),((-0.0825,0.384,0),(-0.7071,0,0,0.7071)),((0,0,0),(0.3123,-0.6344,0.6344,0.3123)),
((0.088,0,0),(0.6533,-0.2706,0.2706,0.6533)),((0,0,0.107),(0,0,0,1)),((0,0,0),(0,0,-0.38268,0.92388))]
M=np.eye(4)
for t,q in chain: M=M@T(t,q)
print("hand in panda_link0 via TF chain:", np.round(M[:3,3],4)); print(np.round(M[:3,:3],3))
# Standard Panda DH FK
def dh(a,d,alpha,theta):
    ca,sa,ct,st=math.cos(alpha),math.sin(alpha),math.cos(theta),math.sin(theta)
    return np.array([[ct,-st,0,a],[st*ca,ct*ca,-sa,-d*sa],[st*sa,ct*sa,ca,d*ca],[0,0,0,1]])
q=[0,-0.161037389,0,-2.44459747,0,2.2267522,0.7853981633974483]
params=[(0,0.333,0),(0,0,-math.pi/2),(0,0.316,math.pi/2),(0.0825,0,math.pi/2),(-0.0825,0.384,-math.pi/2),(0,0,math.pi/2),(0.088,0,math.pi/2)]
F=np.eye(4)
for (a,d,al),th in zip(params,q): F=F@dh(a,d,al,th)
F=F@dh(0,0.107,0,0)
print("flange via DH:", np.round(F[:3,3],4))
EOF
python3 chk_fk.py

# openrua op 20
cat > /workspace/s0b_ik.py <<'EOF'
import numpy as np
import rob
from rob import Robot
rob.BASE_IN_WORLD = np.zeros(3)   # test: assume MoveIt frame == world
r = Robot("s0b")
q0 = r.joints()
quat = (0.9996, 0.0, -0.0284, 0.0)
for label, pos in [("world-coords", (-0.053, 0.0, 0.7776)), ("base-coords", (0.457, 0.0, 0.3577))]:
    q = r.ik_world(pos, quat, at_tcp=False)
    print(label, None if q is None else np.round(q, 3), "current", np.round(q0, 3))
EOF
timeout 120 python3 -u s0b_ik.py

# openrua op 21
python3 - <<'EOF'
import re
s = open("rob.py").read()
s = s.replace('BASE_IN_WORLD = np.array([-0.51, 0.0, 0.42])   # world -> panda_link0 (from /tf)',
              'BASE_IN_WORLD = np.zeros(3)   # verified: MoveIt FK/IK poses are already in world coords')
# add fk_of(q) method and use it in move_tcp
s = s.replace('''    def hand_pose_world(self):
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state = self._arm_state()''',
'''    def hand_pose_world(self, q=None):
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        s = self._arm_state()
        if q is not None:
            s.position = [float(v) for v in q]
        req.robot_state.joint_state = s''')
s = s.replace('''    def tcp_pose_world(self):
        pos, q, f = self.hand_pose_world()''',
'''    def tcp_pose_world(self, joints=None):
        pos, q, f = self.hand_pose_world(joints)''')
s = s.replace('''        code, err = self.move_joints(q, seconds)
        tcp, _ = self.tcp_pose_world()''',
'''        ptcp, pq = self.tcp_pose_world(q)
        dpos = np.linalg.norm(ptcp - np.array(pos_world))
        dq = min(np.linalg.norm(np.array(pq) - np.array(quat)), np.linalg.norm(np.array(pq) + np.array(quat)))
        if dpos > 0.01 or dq > 0.05:
            print(f"  IK solution FK mismatch: pos err {dpos:.4f} quat err {dq:.3f} -> abort")
            return None
        code, err = self.move_joints(q, seconds)
        tcp, _ = self.tcp_pose_world()''')
open("rob.py", "w").write(s)
EOF
grep -n "BASE_IN_WORLD\|FK mismatch" rob.py

# openrua op 22
cat > /workspace/agent_heights.py <<'EOF'
import time, numpy as np, rclpy, sys
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage
cam = sys.argv[1] if len(sys.argv) > 1 else "agentview"
rclpy.init(); n = rclpy.create_node("ah")
got = {}
n.create_subscription(Image, f"/{cam}/depth/image_raw", lambda m: got.setdefault("d", m), 1)
n.create_subscription(CameraInfo, f"/{cam}/color/camera_info", lambda m: got.setdefault("i", m), 1)
def cb(m):
    for t in m.transforms:
        if t.header.frame_id == "world" and t.child_frame_id == f"{cam}_optical_frame":
            got["tf"] = t.transform
n.create_subscription(TFMessage, "/tf", cb, 50)
t0 = time.time()
while not all(k in got for k in ("d", "i", "tf")) and time.time() - t0 < 30:
    rclpy.spin_once(n, timeout_sec=0.1)
d, info, t = got["d"], got["i"], got["tf"]
q = t.rotation; x, y, z, w = q.x, q.y, q.z, q.w
R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],[2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],[2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
T = np.array([t.translation.x, t.translation.y, t.translation.z])
fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
depth = np.frombuffer(d.data, dtype=np.float32).reshape(d.height, d.width)
vv, uu = np.mgrid[0:d.height, 0:d.width]
P = np.stack([(uu-cx)*depth/fx, (vv-cy)*depth/fy, depth], -1) @ R.T + T
np.save(f"{cam}_P.npy", P)
def box(name, x0, x1, y0, y1):
    s = (P[...,0]>x0)&(P[...,0]<x1)&(P[...,1]>y0)&(P[...,1]<y1)&np.isfinite(P[...,2])
    zs = P[s][:,2]
    if len(zs)==0: print(name, "none"); return
    print(f"{name}: n={len(zs)} z min={zs.min():.4f} p5={np.percentile(zs,5):.4f} med={np.median(zs):.4f} p95={np.percentile(zs,95):.4f} max={zs.max():.4f}")
box("table front", 0.05, 0.25, -0.1, 0.1)
box("left plate", -0.05, 0.03, -0.35, -0.28)
box("right plate", -0.05, 0.03, 0.27, 0.34)
box("white mug", -0.18, -0.08, -0.24, -0.14)
box("yellow mug", -0.28, -0.18, -0.01, 0.08)
box("red mug", -0.11, -0.01, 0.05, 0.15)
# white mug: width along y at several heights (handle at +y so use x-extent instead)
for z0 in np.arange(0.43, 0.56, 0.01):
    s = (P[...,2]>z0)&(P[...,2]<z0+0.01)&(P[...,0]>-0.20)&(P[...,0]<-0.06)&(P[...,1]>-0.26)&(P[...,1]<-0.12)
    if s.sum() > 3:
        print(f"  white z {z0:.2f}: x[{P[s][:,0].min():.3f},{P[s][:,0].max():.3f}] y[{P[s][:,1].min():.3f},{P[s][:,1].max():.3f}] n={s.sum()}")
EOF
timeout 90 python3 agent_heights.py agentview

# openrua op 23
cat > /workspace/s1_pregrasp_white.py <<'EOF'
import numpy as np
from rob import Robot, topdown_quat
r = Robot("s1")
Q = topdown_quat(0.0)           # fingers close along world y
WHITE_C = np.array([-0.130, -0.189]); R_WALL = 0.042; RIM_Z = 0.549
pinch = np.array([WHITE_C[0], WHITE_C[1] - R_WALL])
print("pinch point", pinch)
r.gripper(0.04)
print("pre-grasp above pinch, z=0.65")
r.move_tcp([pinch[0], pinch[1], 0.65], Q, seconds=4.0)
print("joints", np.round(r.joints(), 4))
EOF
timeout 900 python3 -u s1_pregrasp_white.py 2>&1 | tee s1.log; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand; timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 24
cat > /workspace/dbg_ik.py <<'EOF'
import numpy as np
from rob import Robot, topdown_quat, quat_to_R
r = Robot("dbg")
Q = topdown_quat(0.0)
q = r.ik_world([-0.13, -0.231, 0.65], Q, at_tcp=True)
print("sol", np.round(q, 4))
pos, fq, _ = r.hand_pose_world(q)
print("FK hand pos", np.round(pos, 4), "quat", np.round(fq, 4), "requested", Q)
print(np.round(quat_to_R(*fq), 3))
tcp, _ = r.tcp_pose_world(q); print("tcp", np.round(tcp, 4))
EOF
timeout 120 python3 -u dbg_ik.py

# openrua op 25
python3 - <<'EOF'
s = open("rob.py").read()
s = s.replace('''def topdown_quat(yaw):''', '''def qmul(a, b):
    x1, y1, z1, w1 = a; x2, y2, z2, w2 = b
    return (w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2,
            w1*w2 - x1*x2 - y1*y2 - z1*z2)

# panda_hand = panda_link8 * Rz(-45deg); the IK service's tip is panda_link8,
# so a desired HAND orientation must be sent as q_hand * Rz(+45deg)
Q_HAND_TO_LINK8 = (0.0, 0.0, math.sin(math.pi/8), math.cos(math.pi/8))

def topdown_quat(yaw):''')
s = s.replace('''        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)''',
'''        q8 = qmul(quat, Q_HAND_TO_LINK8)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q8)''')
open("rob.py", "w").write(s)
EOF
timeout 120 python3 -u dbg_ik.py

# openrua op 26
timeout 900 python3 -u s1_pregrasp_white.py 2>&1 | tee s1.log; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand; timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 27
cat > /workspace/cam_objs.py <<'EOF'
"""Segment raised objects in any camera's depth image; print world extents & rim circle fits.
Usage: cam_objs.py <cam> [zmin_rim=0.50]"""
import sys, time, numpy as np, rclpy, cv2
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage
cam = sys.argv[1]; zrim = float(sys.argv[2]) if len(sys.argv) > 2 else 0.50
rclpy.init(); n = rclpy.create_node("co")
got = {}
n.create_subscription(Image, f"/{cam}/depth/image_raw", lambda m: got.setdefault("d", m), 1)
n.create_subscription(Image, f"/{cam}/color/image_raw", lambda m: got.setdefault("c", m), 1)
n.create_subscription(CameraInfo, f"/{cam}/color/camera_info", lambda m: got.setdefault("i", m), 1)
def cb(m):
    for t in m.transforms:
        if t.header.frame_id == "world" and t.child_frame_id == f"{cam}_optical_frame":
            got["tf"] = t.transform
n.create_subscription(TFMessage, "/tf", cb, 50)
t0 = time.time()
while not all(k in got for k in ("d", "i", "tf", "c")) and time.time() - t0 < 30:
    rclpy.spin_once(n, timeout_sec=0.1)
d, info, t = got["d"], got["i"], got["tf"]
q = t.rotation; x, y, z, w = q.x, q.y, q.z, q.w
R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],[2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],[2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
T = np.array([t.translation.x, t.translation.y, t.translation.z])
print("cam at", np.round(T, 4))
fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
depth = np.frombuffer(d.data, dtype=np.float32).reshape(d.height, d.width)
vv, uu = np.mgrid[0:d.height, 0:d.width]
P = np.stack([(uu-cx)*depth/fx, (vv-cy)*depth/fy, depth], -1) @ R.T + T
color = np.frombuffer(got["c"].data, dtype=np.uint8).reshape(d.height, d.width, -1)[..., :3]
Z = P[..., 2]
np.save(f"{cam}_P.npy", P)
def fit(pts):
    A = np.c_[2*pts[:,0], 2*pts[:,1], np.ones(len(pts))]
    b = (pts**2).sum(1)
    (a, bb, c), *_ = np.linalg.lstsq(A, b, rcond=None)
    return a, bb, np.sqrt(c + a*a + bb*bb)
mask = ((Z > 0.435) & (Z < 0.62) & np.isfinite(Z)).astype(np.uint8)
nlab, lab, stats, cents = cv2.connectedComponentsWithStats(mask)
for i in range(1, nlab):
    if stats[i, cv2.CC_STAT_AREA] < 30: continue
    sel = lab == i; pts = P[sel]; col = color[sel].mean(0)
    print(f"blob{i}: area={stats[i,4]} px=({cents[i][0]:.0f},{cents[i][1]:.0f}) xy=({pts[:,0].mean():.3f},{pts[:,1].mean():.3f}) "
          f"x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] z[{pts[:,2].min():.3f},{pts[:,2].max():.3f}] rgb={col.round(0)}")
    rim = sel & (Z > zrim)
    if rim.sum() > 10:
        a, bb, rr = fit(P[rim][:, :2])
        print(f"   rim fit (z>{zrim}): center=({a:.4f},{bb:.4f}) r={rr:.4f} n={rim.sum()} zmax={P[rim][:,2].max():.4f}")
EOF
timeout 90 python3 cam_objs.py robot0_eye_in_hand 0.52

# openrua op 28
cat > /workspace/s2_grasp_white.py <<'EOF'
import numpy as np
from rob import Robot, topdown_quat
r = Robot("s2")
Q = topdown_quat(0.0)
C = np.array([-0.127, -0.188]); RIM_Z = 0.549; R_WALL = 0.042
pinch = [C[0], C[1] - R_WALL]
print("descend to just above rim")
if r.move_tcp([pinch[0], pinch[1], RIM_Z + 0.035], Q, seconds=3.0) is None: raise SystemExit("fail1")
print("descend into pinch depth")
if r.move_tcp([pinch[0], pinch[1], RIM_Z - 0.025], Q, seconds=2.5) is None: raise SystemExit("fail2")
print("wrench before close", r.read_wrench())
f = r.gripper(0.0)
print("wrench after close", r.read_wrench())
print("lift")
r.move_tcp([pinch[0], pinch[1], 0.70], Q, seconds=3.0)
print("fingers after lift", r.fingers(), "wrench", r.read_wrench())
EOF
timeout 1200 python3 -u s2_grasp_white.py 2>&1 | tee s2.log; timeout 60 python3 tools/perception/cam_snap.py agentview; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 29
sed -i 's/mask = ((Z > 0.435) \& (Z < 0.62)/zlo = float(sys.argv[3]) if len(sys.argv) > 3 else 0.435; zhi = float(sys.argv[4]) if len(sys.argv) > 4 else 0.62\nmask = ((Z > zlo) \& (Z < zhi)/' cam_objs.py && timeout 90 python3 cam_objs.py robot0_eye_in_hand 0.70 0.60 0.80

# openrua op 30
cat > /workspace/s3_place_white.py <<'EOF'
import numpy as np
from rob import Robot, topdown_quat
r = Robot("s3")
Q = topdown_quat(0.0)
PLATE = np.array([-0.004, -0.315]); R_WALL = 0.042; PLATE_TOP = 0.445; MUG_H = 0.123
tcp_xy = [PLATE[0], PLATE[1] - R_WALL]
print("traverse to above left plate")
if r.move_tcp([tcp_xy[0], tcp_xy[1], 0.70], Q, seconds=4.0) is None: raise SystemExit("fail")
print("lower to 0.62")
if r.move_tcp([tcp_xy[0], tcp_xy[1], 0.62], Q, seconds=2.5) is None: raise SystemExit("fail")
print("wrench", r.read_wrench())
print("lower to placement height")
z_place = PLATE_TOP + MUG_H - 0.025 + 0.004
if r.move_tcp([tcp_xy[0], tcp_xy[1], z_place], Q, seconds=3.0) is None: raise SystemExit("fail")
print("wrench", r.read_wrench(), "fingers", r.fingers())
EOF
timeout 1200 python3 -u s3_place_white.py 2>&1 | tee s3.log; timeout 60 python3 tools/perception/cam_snap.py agentview; timeout 60 python3 tools/perception/cam_snap.py frontview

# openrua op 31
cat > /workspace/s3b_lower.py <<'EOF'
import numpy as np, sys
from rob import Robot, topdown_quat
r = Robot("s3b")
Q = topdown_quat(0.0)
tcp, _ = r.tcp_pose_world()
xy = tcp[:2]
z = tcp[2]
base = r.read_wrench()
print("start", np.round(tcp, 4), "wrench", np.round(base, 3))
zmin = float(sys.argv[1]) if len(sys.argv) > 1 else 0.505
while z > zmin + 1e-4:
    z = max(zmin, z - 0.01)
    if r.move_tcp([xy[0], xy[1], z], Q, seconds=1.5) is None: raise SystemExit("fail")
    w = r.read_wrench()
    print(f"z={z:.3f} wrench={np.round(w,3)} dFz={w[2]-base[2]:+.3f} fingers={np.round(r.fingers(),4)}")
    if abs(w[2] - base[2]) > 1.5:
        print("contact detected"); break
EOF
timeout 1200 python3 -u s3b_lower.py 0.505 2>&1 | tee s3b.log; timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 32
cat > /workspace/s4_release.py <<'EOF'
import numpy as np, sys
from rob import Robot, topdown_quat
r = Robot("s4")
Q = topdown_quat(0.0)
tcp, _ = r.tcp_pose_world()
r.gripper(0.04)
print("wrench after release", np.round(r.read_wrench(), 3))
z_up = float(sys.argv[1]) if len(sys.argv) > 1 else 0.70
r.move_tcp([tcp[0], tcp[1], z_up], Q, seconds=3.0)
EOF
timeout 1200 python3 -u s4_release.py 0.70 2>&1 | tee s4.log; timeout 60 python3 tools/perception/cam_snap.py agentview; timeout 90 python3 cam_objs.py birdview 0.50

# openrua op 33
cat > /workspace/s5_pregrasp_yellow.py <<'EOF'
import numpy as np
from rob import Robot, topdown_quat
r = Robot("s5")
Q = topdown_quat(0.0)
C = np.array([-0.229, 0.030]); R_WALL = 0.040
pinch = [C[0], C[1] + R_WALL]
print("pinch", pinch)
if r.move_tcp([pinch[0], pinch[1], 0.70], Q, seconds=4.0) is None: raise SystemExit("fail")
print("joints", np.round(r.joints(), 4))
EOF
timeout 1200 python3 -u s5_pregrasp_yellow.py 2>&1 | tee s5.log; timeout 90 python3 cam_objs.py robot0_eye_in_hand 0.50; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 34
cat > /workspace/s5b_retry.py <<'EOF'
import numpy as np
from rob import Robot, topdown_quat
r = Robot("s5b")
Q = topdown_quat(0.0)
pinch = [-0.229, 0.070]
q = r.ik_world([pinch[0], pinch[1], 0.70], Q)
print("IK sol", np.round(q, 4), "\ncurrent", np.round(r.joints(), 4))
for attempt in range(3):
    code, err = r.move_joints(q, 5.0)
    print("attempt", attempt, "code", code, "err", err, "joints", np.round(r.joints(), 4))
    if err < 0.02: break
tcp, _ = r.tcp_pose_world(); print("tcp", np.round(tcp, 4))
EOF
timeout 1200 python3 -u s5b_retry.py 2>&1 | tee s5b.log

# openrua op 35
timeout 90 python3 cam_objs.py robot0_eye_in_hand 0.50 2>&1 | grep -A1 "blob"; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 36
cat > /workspace/s6_grasp_yellow.py <<'EOF'
import numpy as np
from rob import Robot, topdown_quat
r = Robot("s6")
Q = topdown_quat(0.0)
C = np.array([-0.226, 0.033]); RIM_Z = 0.5415; R_WALL = 0.039
pinch = [C[0], C[1] + R_WALL]
print("pinch", pinch)
for z, t in [(RIM_Z + 0.035, 3.0), (RIM_Z - 0.025, 2.5)]:
    for attempt in range(3):
        q = r.ik_world([pinch[0], pinch[1], z], Q)
        if q is None: raise SystemExit(f"IK failed at z={z}")
        code, err = r.move_joints(q, t)
        if err < 0.02: break
        print("  retrying (tolerance violation)")
    tcp, _ = r.tcp_pose_world(); print(f"tcp {np.round(tcp,4)} target z {z:.4f}")
    if err >= 0.02: raise SystemExit("could not converge")
print("wrench before close", np.round(r.read_wrench(), 3))
r.gripper(0.0)
print("wrench after close", np.round(r.read_wrench(), 3))
print("lift to 0.80")
for attempt in range(3):
    q = r.ik_world([pinch[0], pinch[1], 0.80], Q)
    code, err = r.move_joints(q, 3.5)
    if err < 0.02: break
tcp, _ = r.tcp_pose_world(); print("tcp", np.round(tcp, 4), "fingers", np.round(r.fingers(), 4), "wrench", np.round(r.read_wrench(), 3))
EOF
timeout 1500 python3 -u s6_grasp_yellow.py 2>&1 | tee s6.log; timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 37
cat > /workspace/s7_place_yellow.py <<'EOF'
import numpy as np
from rob import Robot, topdown_quat
r = Robot("s7")
Q = topdown_quat(0.0)
PLATE = np.array([-0.016, 0.305]); R_WALL = 0.039
xy = [PLATE[0], PLATE[1] - R_WALL]
def go(z, t):
    for attempt in range(3):
        q = r.ik_world([xy[0], xy[1], z], Q)
        if q is None: raise SystemExit(f"IK failed z={z}")
        code, err = r.move_joints(q, t)
        if err < 0.02: return
        print("  retrying")
    raise SystemExit("no convergence")
print("traverse at 0.80"); go(0.80, 4.5)
print("tcp", np.round(r.tcp_pose_world()[0], 4))
print("lower to 0.60"); go(0.60, 3.0)
base = r.read_wrench(); print("wrench", np.round(base, 3))
z = 0.60
while z > 0.505 + 1e-4:
    z = max(0.505, z - 0.01)
    go(z, 1.5)
    w = r.read_wrench(); tcp = r.tcp_pose_world()[0]
    print(f"z_cmd={z:.3f} tcp_z={tcp[2]:.4f} dFz={w[2]-base[2]:+.3f} fingers={np.round(r.fingers(),4)}")
    if abs(w[2] - base[2]) > 1.5:
        print("contact detected"); break
EOF
timeout 1500 python3 -u s7_place_yellow.py 2>&1 | tee s7.log; timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 38
timeout 1200 python3 -u s4_release.py 0.75 2>&1 | tee s8.log; timeout 60 python3 tools/perception/cam_snap.py agentview; timeout 60 python3 tools/perception/cam_snap.py birdview

# openrua op 39
timeout 90 python3 cam_objs.py birdview 0.47 2>&1; python3 -c "
import cv2
im = cv2.imread('birdview.png')
crop = im[240:320, 340:440]
cv2.imwrite('birdview_crop.png', cv2.resize(crop, None, fx=6, fy=6, interpolation=cv2.INTER_CUBIC))
"

# openrua op 40
cat > /workspace/s9_park.py <<'EOF'
import numpy as np, sys
from rob import Robot, topdown_quat
r = Robot("s9")
Q = topdown_quat(0.0)
x, y, z = map(float, sys.argv[1:4])
for attempt in range(3):
    q = r.ik_world([x, y, z], Q)
    if q is None: raise SystemExit("IK failed")
    code, err = r.move_joints(q, 4.0)
    if err < 0.02: break
print("tcp", np.round(r.tcp_pose_world()[0], 4))
EOF
timeout 900 python3 -u s9_park.py -0.25 0.0 0.80 2>&1 | tee s9.log; timeout 90 python3 cam_objs.py birdview 0.47 2>&1 | grep -A1 blob; timeout 60 python3 tools/perception/cam_snap.py agentview; timeout 60 python3 tools/perception/cam_snap.py birdview; python3 -c "
import cv2
im = cv2.imread('birdview.png')
crop = im[240:320, 340:440]
cv2.imwrite('birdview_crop.png', cv2.resize(crop, None, fx=6, fy=6, interpolation=cv2.INTER_CUBIC))
"

# openrua op 41
timeout 900 python3 -u s9_park.py -0.08 0.20 0.78 2>&1 | tail -1; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand; timeout 90 python3 cam_objs.py robot0_eye_in_hand 0.47 0.44 0.62 | grep -A1 blob

# openrua op 42
cat > /workspace/ana_yellow.py <<'EOF'
import numpy as np
P = np.load("robot0_eye_in_hand_P.npy")
Z = P[..., 2]
sel = (P[...,0] > -0.08) & (P[...,0] < 0.04) & (P[...,1] > 0.13) & (P[...,1] < 0.27) & (Z > 0.44) & np.isfinite(Z)
pts = P[sel]
print("n", len(pts), "zmax", pts[:,2].max())
for y0 in np.arange(0.13, 0.27, 0.01):
    s = (pts[:,1] >= y0) & (pts[:,1] < y0 + 0.01)
    if s.sum() < 5: continue
    q = pts[s]
    i = q[:,2].argmax()
    print(f"y[{y0:.2f},{y0+0.01:.2f}] n={s.sum():4d} zmax={q[i,2]:.4f} at x={q[i,0]:+.4f}  x-range[{q[:,0].min():+.3f},{q[:,0].max():+.3f}]  z>0.46 x-range: "
          + (f"[{q[q[:,2]>0.46][:,0].min():+.3f},{q[q[:,2]>0.46][:,0].max():+.3f}]" if (q[:,2]>0.46).any() else "-"))
EOF
python3 ana_yellow.py

# openrua op 43
sed -i 's/(Z > 0.44) \& np.isfinite(Z)/(Z > 0.44) \& (Z < 0.65) \& np.isfinite(Z)/' ana_yellow.py && python3 ana_yellow.py

# openrua op 44
cat > /workspace/hmap.py <<'EOF'
import numpy as np, cv2, sys
P = np.load(sys.argv[1] if len(sys.argv) > 1 else "robot0_eye_in_hand_P.npy")
x0, x1, y0, y1 = -0.16, 0.06, 0.05, 0.36   # region
res = 0.002
W, H = int((y1-y0)/res), int((x1-x0)/res)
hm = np.full((H, W), np.nan)
sel = np.isfinite(P[...,2]) & (P[...,2] < 0.65) & (P[...,0]>x0)&(P[...,0]<x1)&(P[...,1]>y0)&(P[...,1]<y1)
pts = P[sel]
ix = ((pts[:,0]-x0)/res).astype(int); iy = ((pts[:,1]-y0)/res).astype(int)
for a, b, z in zip(ix, iy, pts[:,2]):
    if np.isnan(hm[a,b]) or z > hm[a,b]: hm[a,b] = z
img = np.zeros((H, W, 3), np.uint8)
v = np.clip((hm - 0.43)/0.15, 0, 1); v[np.isnan(v)] = 0
img = cv2.applyColorMap((v*255).astype(np.uint8), cv2.COLORMAP_JET)
img[np.isnan(hm)] = 0
# grid lines every 5 cm
for xx in np.arange(-0.15, 0.06, 0.05):
    r = int((xx-x0)/res); cv2.line(img, (0, r), (W-1, r), (255,255,255), 1); cv2.putText(img, f"x={xx:.2f}", (2, r-2), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255,255,255), 1)
for yy in np.arange(0.05, 0.36, 0.05):
    c = int((yy-y0)/res); cv2.line(img, (c, 0), (c, H-1), (255,255,255), 1); cv2.putText(img, f"y={yy:.2f}", (c+2, 12), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255,255,255), 1)
cv2.imwrite("hmap.png", cv2.resize(img, None, fx=4, fy=4, interpolation=cv2.INTER_NEAREST))
print("saved hmap.png; rows = x (down = +x), cols = y (right = +y)")
EOF
python3 hmap.py

# openrua op 45
timeout 60 python3 tools/perception/cam_snap.py agentview >/dev/null; timeout 60 python3 tools/perception/cam_snap.py frontview >/dev/null; python3 -c "
import cv2
a = cv2.imread('agentview.png'); cv2.imwrite('crop_a.png', cv2.resize(a[190:360, 400:640], None, fx=3, fy=3, interpolation=cv2.INTER_CUBIC))
f = cv2.imread('frontview.png'); cv2.imwrite('crop_f.png', cv2.resize(f[170:270, 330:520], None, fx=4, fy=4, interpolation=cv2.INTER_CUBIC))
"

# openrua op 46
cat > /workspace/s10_regrasp_yellow.py <<'EOF'
import numpy as np, math, sys
from rob import Robot, topdown_quat
r = Robot("s10")
MC = np.array([-0.011, 0.206, 0.55])   # mouth center estimate
R_WALL = 0.040
tcp_xy = [MC[0] + R_WALL, MC[1]]        # pinch the +x side of the rim
Z_GRASP = 0.54
yaw = None
for y_ in (math.pi/2, -math.pi/2):
    q = r.ik_world([tcp_xy[0], tcp_xy[1], Z_GRASP], topdown_quat(y_))
    print("yaw", y_, "IK", None if q is None else np.round(q, 3))
    if q is not None and yaw is None: yaw = y_
if yaw is None: raise SystemExit("no IK")
Q = topdown_quat(yaw)
def go(xyz, t):
    for attempt in range(3):
        q = r.ik_world(xyz, Q)
        if q is None: raise SystemExit(f"IK failed {xyz}")
        code, err = r.move_joints(q, t)
        if err < 0.02: return
        print("  retrying")
    raise SystemExit("no convergence")
r.gripper(0.04)
print("pre-grasp"); go([tcp_xy[0], tcp_xy[1], 0.70], 4.0)
print("tcp", np.round(r.tcp_pose_world()[0], 4), "hand q", np.round(r.tcp_pose_world()[1], 3))
if "--stop" in sys.argv: raise SystemExit(0)
print("descend to 0.60"); go([tcp_xy[0], tcp_xy[1], 0.60], 2.5)
base = r.read_wrench(); print("wrench", np.round(base, 3))
print("descend to grasp z"); go([tcp_xy[0], tcp_xy[1], Z_GRASP], 2.5)
w = r.read_wrench(); print("wrench", np.round(w, 3), "dFz", round(w[2]-base[2], 3))
print("tcp", np.round(r.tcp_pose_world()[0], 4))
EOF
timeout 1500 python3 -u s10_regrasp_yellow.py --stop 2>&1 | tee s10.log; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 47
timeout 90 python3 cam_objs.py robot0_eye_in_hand 0.50 0.44 0.62 | grep -A1 blob; python3 hmap.py robot0_eye_in_hand_P.npy; python3 - <<'EOF'
import numpy as np
P = np.load("robot0_eye_in_hand_P.npy"); Z = P[...,2]
sel = (P[...,0] > -0.08) & (P[...,0] < 0.06) & (P[...,1] > 0.15) & (P[...,1] < 0.30) & (Z > 0.44) & (Z < 0.62) & np.isfinite(Z)
pts = P[sel]
i = pts[:,2].argmax(); print("highest point", np.round(pts[i], 4))
top = pts[pts[:,2] > pts[:,2].max() - 0.004]
print("top cluster mean", np.round(top.mean(0), 4), "n", len(top), "x-range", np.round([top[:,0].min(), top[:,0].max()], 3))
for y0 in np.arange(0.15, 0.30, 0.01):
    s = (pts[:,1] >= y0) & (pts[:,1] < y0 + 0.01)
    if s.sum() < 5: continue
    q = pts[s]; j = q[:,2].argmax()
    hi = q[q[:,2] > 0.50]
    print(f"y[{y0:.2f}] zmax={q[j,2]:.4f}@x={q[j,0]:+.3f}  x-range(z>0.50)=" + (f"[{hi[:,0].min():+.3f},{hi[:,0].max():+.3f}]" if len(hi) else "-") + f"  zmin={q[:,2].min():.3f}")
EOF

# openrua op 48
mkdir -p "$(dirname /workspace/s11_grasp_yellow2.py)"
cat > /workspace/s11_grasp_yellow2.py <<'OPENRUA_EOF'
import sys, math, time, numpy as np
sys.path.insert(0, "/workspace")
from rob import Robot, topdown_quat

Z_GRASP = float(sys.argv[1]) if len(sys.argv) > 1 else 0.535
r = Robot("s11")
Q = topdown_quat(math.pi / 2)
tcp = r.tcp_pose_world()[0]
xy = [tcp[0], tcp[1]]
print("start tcp", np.round(tcp, 4))

def go(xyz, t):
    for k in range(3):
        j = r.ik_world(xyz, Q, seed=r.joints())
        if j is None:
            print("IK fail", xyz); sys.exit(1)
        code, err = r.move_joints(j, t)
        if err < 0.02:
            break
    print("tcp", np.round(r.tcp_pose_world()[0], 4))

go([xy[0], xy[1], 0.60], 2.5)
base = r.read_wrench()
go([xy[0], xy[1], Z_GRASP], 2.5)
w = r.read_wrench()
print("dFz descend", round(w[2] - base[2], 2))
if abs(w[2] - base[2]) > 2.0:
    print("unexpected contact, abort"); sys.exit(2)
f = r.gripper(0.0)
print("fingers after close", f)
go([xy[0], xy[1], 0.62], 2.5)
print("fingers after lift1", r.fingers())
go([xy[0], xy[1], 0.80], 3.0)
print("fingers after lift2", r.fingers())
OPENRUA_EOF

# openrua op 49
timeout 120 python3 s11_grasp_yellow2.py 0.535 2>&1 | grep -v "^\[" ; timeout 40 python3 tools/cam_snap.py robot0_eye_in_hand 2>&1 | tail -1

# openrua op 50
ls /workspace/tools /workspace/*.py | head -40; grep -n "def \|argv" /workspace/tools/*snap* 2>/dev/null | head

# openrua op 51
ls /workspace/tools/perception /workspace/tools/action; timeout 60 python3 tools/perception/cam_snap.py agentview 2>&1 | tail -2

# openrua op 52
sed -n 1,60p /workspace/cam_objs.py

# openrua op 53
cat > held_mug.py <<'EOF'
"""Find yellow-colored points of the held mug in a camera cloud (uses cam_objs machinery)."""
import sys, numpy as np, subprocess
cam = sys.argv[1]
subprocess.run(["timeout", "60", "python3", "cam_objs.py", cam, "0.9", "0.9", "0.95"], capture_output=True)
P = np.load(f"{cam}_P.npy")
import rclpy, time
from sensor_msgs.msg import Image
rclpy.init(); n = rclpy.create_node("hm"); got = {}
n.create_subscription(Image, f"/{cam}/color/image_raw", lambda m: got.setdefault("c", m), 1)
while "c" not in got: rclpy.spin_once(n, timeout_sec=0.1)
c = np.frombuffer(got["c"].data, dtype=np.uint8).reshape(P.shape[0], P.shape[1], -1)[..., :3].astype(int)
yellow = (c[...,0] > 150) & (c[...,1] > 130) & (c[...,2] < 110) & (c[...,0] - c[...,2] > 70)
sel = yellow & np.isfinite(P[...,2]) & (P[...,2] > 0.55)
pts = P[sel]
print("yellow pts", len(pts))
if len(pts):
    print("x", np.round([pts[:,0].min(), pts[:,0].max()],3), "y", np.round([pts[:,1].min(), pts[:,1].max()],3), "z", np.round([pts[:,2].min(), pts[:,2].max()],3))
    lo = pts[pts[:,2] < pts[:,2].min() + 0.01]
    print("lowest cluster mean", np.round(lo.mean(0),3), "n", len(lo))
    hi = pts[pts[:,2] > pts[:,2].max() - 0.01]
    print("highest cluster mean", np.round(hi.mean(0),3), "n", len(hi))
EOF
timeout 120 python3 held_mug.py agentview 2>&1 | grep -v "^\["; timeout 120 python3 held_mug.py frontview 2>&1 | grep -v "^\["

# openrua op 54
python3 - <<'EOF'
import numpy as np
for cam in ["agentview","frontview"]:
    P = np.load(f"{cam}_P.npy"); Z=P[...,2]
    sel = np.isfinite(Z) & (Z>0.60) & (Z<0.80) & (P[...,1]>0.08) & (P[...,1]<0.32) & (P[...,0]>-0.15) & (P[...,0]<0.12)
    pts = P[sel]; print(cam, "n", len(pts))
    for z0 in np.arange(0.60, 0.80, 0.02):
        s = (pts[:,2]>=z0)&(pts[:,2]<z0+0.02)
        if s.sum()<3: continue
        q=pts[s]; print(f"  z[{z0:.2f}] n={s.sum():4d} x[{q[:,0].min():+.3f},{q[:,0].max():+.3f}] y[{q[:,1].min():.3f},{q[:,1].max():.3f}]")
EOF

# openrua op 55
grep -n "def qmul" -A 8 rob.py; grep -n "def topdown_quat" -A 8 rob.py; grep -n "def move_tcp" -A 30 rob.py | head -50

# openrua op 56
mkdir -p "$(dirname /workspace/s12_tilt.py)"
cat > /workspace/s12_tilt.py <<'OPENRUA_EOF'
"""Tilt the hand about the world x axis (through the TCP) by PHI degrees in steps, keeping TCP position.
Usage: s12_tilt.py <phi_deg_total> [steps]"""
import sys, math, numpy as np
sys.path.insert(0, "/workspace")
from rob import Robot, topdown_quat, qmul

phi = math.radians(float(sys.argv[1])); steps = int(sys.argv[2]) if len(sys.argv) > 2 else 3
r = Robot("s12")
Q0 = topdown_quat(math.pi / 2)
tcp0, q_now = r.tcp_pose_world()
print("start tcp", np.round(tcp0, 4), "quat", np.round(q_now, 3), "fingers", r.fingers())
# current tilt: recover from q_now relative to Q0 -> assume the hand is Rx(phi0)*Q0
# (just compose on top of what was requested before; caller keeps track of cumulative phi)
phi0 = 0.0
if len(sys.argv) > 3:
    phi0 = math.radians(float(sys.argv[3]))
for k in range(1, steps + 1):
    a = phi0 + phi * k / steps
    qx = (math.sin(a / 2), 0, 0, math.cos(a / 2))
    Q = qmul(qx, Q0)
    for t in range(3):
        j = r.ik_world(tcp0, Q, seed=r.joints())
        if j is None:
            print("IK fail at", math.degrees(a)); sys.exit(1)
        ptcp, pq = r.tcp_pose_world(j)
        dq = min(np.linalg.norm(np.array(pq) - np.array(Q)), np.linalg.norm(np.array(pq) + np.array(Q)))
        if np.linalg.norm(ptcp - tcp0) > 0.01 or dq > 0.05:
            print("FK mismatch", np.linalg.norm(ptcp - tcp0), dq); sys.exit(1)
        code, err = r.move_joints(j, 2.5)
        if err < 0.02:
            break
    tcp, qq = r.tcp_pose_world()
    print(f"tilt {math.degrees(a):.1f} deg: tcp {np.round(tcp,4)} quat {np.round(qq,3)} fingers {np.round(r.fingers(),4)}")
OPENRUA_EOF

# openrua op 57
timeout 120 python3 s12_tilt.py -40 3 2>&1 | grep -v "^\["; timeout 60 python3 tools/perception/cam_snap.py agentview 2>&1 | tail -1

# openrua op 58
timeout 60 python3 cam_objs.py agentview 0.9 0.9 0.95 >/dev/null 2>&1; python3 - <<'EOF'
import numpy as np
P = np.load("agentview_P.npy"); Z=P[...,2]
sel = np.isfinite(Z) & (Z>0.60) & (Z<0.80) & (P[...,1]>0.08) & (P[...,1]<0.32) & (P[...,0]>-0.15) & (P[...,0]<0.12)
pts = P[sel]; print("n", len(pts))
for z0 in np.arange(0.66, 0.80, 0.01):
    s = (pts[:,2]>=z0)&(pts[:,2]<z0+0.01)
    if s.sum()<3: continue
    q=pts[s]; print(f"  z[{z0:.2f}] n={s.sum():4d} x[{q[:,0].min():+.3f},{q[:,0].max():+.3f}] y[{q[:,1].min():.3f},{q[:,1].max():.3f}] ymid={(q[:,1].min()+q[:,1].max())/2:.3f}")
EOF

# openrua op 59
timeout 60 python3 tools/perception/cam_snap.py frontview >/dev/null 2>&1; python3 -c "
import cv2; im=cv2.imread('frontview.png'); print(im.shape)
c=im[0:260, 380:640]; c=cv2.resize(c,None,fx=2.5,fy=2.5,interpolation=cv2.INTER_CUBIC); cv2.imwrite('crop.png',c)"

# openrua op 60
python3 -c "
import cv2; im=cv2.imread('frontview.png')
c=im[60:220, 280:480]; c=cv2.resize(c,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC); cv2.imwrite('crop.png',c)"

# openrua op 61
timeout 120 python3 s12_tilt.py -10 1 -40 2>&1 | grep -v "^\["; timeout 60 python3 tools/perception/cam_snap.py frontview >/dev/null 2>&1; python3 -c "
import cv2; im=cv2.imread('frontview.png')
c=im[60:220, 280:480]; c=cv2.resize(c,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC); cv2.imwrite('crop.png',c)"

# openrua op 62
timeout 120 python3 s12_tilt.py 10 1 -50 2>&1 | grep -v "^\[" | tail -1; timeout 60 python3 tools/perception/cam_snap.py frontview >/dev/null 2>&1; python3 -c "
import cv2; im=cv2.imread('frontview.png')
c=im[60:220, 280:480]; c=cv2.resize(c,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC); cv2.imwrite('crop.png',c)"

# openrua op 63
cat > s13a_place2.py <<'EOF'
"""Carry the pendulum-hung yellow mug over the right plate and lower until base contact."""
import sys, math, numpy as np
sys.path.insert(0, "/workspace")
from rob import Robot, topdown_quat
TCP_XY = [float(sys.argv[1]), float(sys.argv[2])] if len(sys.argv) > 2 else [0.0255, 0.285]
r = Robot("s13a"); Q = topdown_quat(math.pi / 2)
def go(xyz, t):
    for k in range(3):
        j = r.ik_world(xyz, Q, seed=r.joints())
        if j is None: print("IK fail", xyz); sys.exit(1)
        code, err = r.move_joints(j, t)
        if err < 0.02: break
    return r.tcp_pose_world()[0]
def hold(t):
    r.move_joints(r.joints(), t)
tcp = r.tcp_pose_world()[0]
print("start", np.round(tcp, 4), "fingers", np.round(r.fingers(), 4))
if "--carry" in sys.argv:
    go([tcp[0], tcp[1], 0.80], 3.0)
    p = go([TCP_XY[0], TCP_XY[1], 0.80], 5.0); hold(3.0); print("over plate", np.round(p, 4))
    p = go([TCP_XY[0], TCP_XY[1], 0.60], 4.0); hold(2.0); print("at 0.60", np.round(p, 4), "fingers", np.round(r.fingers(), 4))
    base = r.read_wrench(); print("base wrench", np.round(base, 2))
    z = 0.60
    while z > 0.515:
        z -= 0.005
        p = go([TCP_XY[0], TCP_XY[1], z], 1.0)
        w = r.read_wrench(); d = w[2] - base[2]
        print(f"z={z:.3f} tcp={np.round(p,4)} dFz={d:+.2f} fingers={np.round(r.fingers(),4)}")
        if d > 0.8:
            print("contact"); break
EOF
timeout 120 python3 s12_tilt.py 40 3 -40 2>&1 | grep -v "^\[" | tail -1 && timeout 300 python3 s13a_place2.py --carry 2>&1 | grep -v "^\["; timeout 60 python3 tools/perception/cam_snap.py agentview >/dev/null 2>&1

# openrua op 64
cat > s13b_upright.py <<'EOF'
"""Shift TCP in +y in small steps (mug base pinned by friction) to bring the mug upright, then release."""
import sys, math, numpy as np
sys.path.insert(0, "/workspace")
from rob import Robot, topdown_quat
DY = float(sys.argv[1]); N = int(sys.argv[2]); DZ = float(sys.argv[3]) if len(sys.argv) > 3 else -0.003
r = Robot("s13b"); Q = topdown_quat(math.pi / 2)
def go(xyz, t):
    for k in range(3):
        j = r.ik_world(xyz, Q, seed=r.joints())
        if j is None: print("IK fail", xyz); sys.exit(1)
        code, err = r.move_joints(j, t)
        if err < 0.02: break
    return r.tcp_pose_world()[0]
tcp = r.tcp_pose_world()[0]; base = r.read_wrench()
print("start", np.round(tcp, 4), "wrench", np.round(base, 2), "fingers", np.round(r.fingers(), 4))
for k in range(1, N + 1):
    p = go([tcp[0], tcp[1] + DY * k / N, tcp[2] + DZ * k / N], 1.5)
    w = r.read_wrench()
    print(f"step {k}: tcp={np.round(p,4)} dFz={w[2]-base[2]:+.2f} dFy={w[1]-base[1]:+.2f} fingers={np.round(r.fingers(),4)}")
    if w[2] - base[2] > 3.0:
        print("too much force, stop"); break
if "--release" in sys.argv:
    r.gripper(0.04)
    p = r.tcp_pose_world()[0]
    go([p[0], p[1], p[2] + 0.12], 3.0)
    print("released & lifted", np.round(r.tcp_pose_world()[0], 4))
EOF
timeout 200 python3 s13b_upright.py 0.02 2 -0.003 2>&1 | grep -v "^\["; timeout 60 python3 tools/perception/cam_snap.py agentview >/dev/null 2>&1

# openrua op 65
timeout 60 python3 tools/perception/cam_snap.py frontview >/dev/null 2>&1; python3 -c "
import cv2; im=cv2.imread('frontview.png')
c=im[200:400, 250:450]; c=cv2.resize(c,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC); cv2.imwrite('crop.png',c)"

# openrua op 66
python3 -c "
import cv2; im=cv2.imread('frontview.png')
c=im[180:280, 380:520]; c=cv2.resize(c,None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC); cv2.imwrite('crop.png',c)"

# openrua op 67
timeout 200 python3 s13b_upright.py 0.0 1 0.005 2>&1 | grep -v "^\["; timeout 60 python3 tools/perception/cam_snap.py frontview >/dev/null 2>&1; python3 -c "
import cv2; im=cv2.imread('frontview.png')
c=im[180:280, 380:520]; c=cv2.resize(c,None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC); cv2.imwrite('crop.png',c)"

# openrua op 68
cat > s13c_release.py <<'EOF'
import sys, math, numpy as np
sys.path.insert(0, "/workspace")
from rob import Robot, topdown_quat
r = Robot("s13c"); Q = topdown_quat(math.pi / 2)
def go(xyz, t):
    for k in range(3):
        j = r.ik_world(xyz, Q, seed=r.joints())
        if j is None: print("IK fail", xyz); sys.exit(1)
        code, err = r.move_joints(j, t)
        if err < 0.02: break
    return r.tcp_pose_world()[0]
tcp = r.tcp_pose_world()[0]; base = r.read_wrench()
print("start", np.round(tcp, 4), "wrench", np.round(base, 2))
z = tcp[2]
while z > 0.53:
    z -= 0.002
    p = go([tcp[0], tcp[1], z], 1.0); w = r.read_wrench()
    print(f"z={z:.4f} tcp={np.round(p,4)} dFz={w[2]-base[2]:+.2f}")
    if w[2] - base[2] > 0.3: print("light contact"); break
r.gripper(0.04)
p = r.tcp_pose_world()[0]
go([p[0], p[1], p[2] + 0.15], 3.0)
print("released & lifted", np.round(r.tcp_pose_world()[0], 4))
EOF
timeout 200 python3 s13c_release.py 2>&1 | grep -v "^\["; timeout 60 python3 tools/perception/cam_snap.py agentview >/dev/null 2>&1

# openrua op 69
timeout 120 python3 s9_park.py -0.25 0.0 0.80 2>&1 | grep -v "^\[" | tail -2; timeout 90 python3 cam_objs.py birdview 0.50 2>&1 | grep -v "^\[" ; timeout 60 python3 tools/perception/cam_snap.py agentview >/dev/null 2>&1
