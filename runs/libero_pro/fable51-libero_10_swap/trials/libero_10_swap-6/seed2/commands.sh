#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat tools/perception/cam_snap.py tools/perception/px2world.py

# openrua op 2
cat tools/action/fjt_send.py tools/action/gripper_cmd.py tools/action/ik_move.py

# openrua op 3
ros2 topic list && echo ---ACTIONS--- && ros2 action list && echo ---SERVICES--- && ros2 service list | head -50 && echo ---NODES--- && ros2 node list

# openrua op 4
mkdir -p snaps && for c in agentview birdview frontview sideview robot0_robotview robot0_eye_in_hand galleryview paperview; do timeout 60 python3 tools/perception/cam_snap.py $c snaps/${c}_0.png; done; ros2 topic echo /joint_states --once

# openrua op 5
timeout 20 ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id|translation|x:|y:|z:|w:" | head -80

# openrua op 6
cat > /workspace/tfdump.py <<'EOF'
import rclpy, yaml
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
rclpy.init(); node = rclpy.create_node("tfdump")
got = {}
def cb(m, which):
    for t in m.transforms:
        got[(which, t.header.frame_id, t.child_frame_id)] = t.transform
qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
node.create_subscription(TFMessage, "/tf_static", lambda m: cb(m, "static"), qos)
node.create_subscription(TFMessage, "/tf", lambda m: cb(m, "dyn"), 10)
import time
t0 = time.time()
while time.time() - t0 < 5: rclpy.spin_once(node, timeout_sec=0.2)
for k, tr in sorted(got.items()):
    print(k, f"t=({tr.translation.x:.4f},{tr.translation.y:.4f},{tr.translation.z:.4f}) q=({tr.rotation.x:.4f},{tr.rotation.y:.4f},{tr.rotation.z:.4f},{tr.rotation.w:.4f})")
EOF
timeout 30 python3 /workspace/tfdump.py

# openrua op 7
cat > /workspace/scene.py <<'EOF'
"""Grab color+depth+info for a camera, save, and provide pixel->world."""
import sys, struct, numpy as np, rclpy, cv2
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage
from cv_bridge import CvBridge

def quat_R(x,y,z,w):
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])

def grab(cam):
    rclpy.init(); node = rclpy.create_node("scene")
    got = {}
    node.create_subscription(Image, f"/{cam}/color/image_raw", lambda m: got.setdefault("c", m), 1)
    node.create_subscription(Image, f"/{cam}/depth/image_raw", lambda m: got.setdefault("d", m), 1)
    node.create_subscription(CameraInfo, f"/{cam}/color/camera_info", lambda m: got.setdefault("i", m), 1)
    def tfcb(m):
        for t in m.transforms:
            if t.child_frame_id == f"{cam}_optical_frame" and t.header.frame_id == "world":
                got["tf"] = t.transform
    node.create_subscription(TFMessage, "/tf", tfcb, 10)
    while not all(k in got for k in "cdi") or "tf" not in got:
        rclpy.spin_once(node, timeout_sec=0.5)
    rclpy.shutdown()
    br = CvBridge()
    color = br.imgmsg_to_cv2(got["c"], "bgr8")
    depth = br.imgmsg_to_cv2(got["d"], "passthrough").astype(np.float32)
    K = np.array(got["i"].k).reshape(3,3)
    t = got["tf"]; T = np.eye(4)
    T[:3,:3] = quat_R(t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w)
    T[:3,3] = [t.translation.x,t.translation.y,t.translation.z]
    return color, depth, K, T

def px2world(u, v, depth, K, T):
    z = depth[v,u]
    p = np.array([(u-K[0,2])*z/K[0,0], (v-K[1,2])*z/K[1,1], z, 1.0])
    return (T@p)[:3]

def cloud(depth, K, T):
    h,w = depth.shape
    vs,us = np.mgrid[0:h,0:w]
    z = depth
    X = (us-K[0,2])*z/K[0,0]; Y = (vs-K[1,2])*z/K[1,1]
    P = np.stack([X,Y,z,np.ones_like(z)],-1) @ T.T
    return P[...,:3]

if __name__ == "__main__":
    cam = sys.argv[1]
    color, depth, K, T = grab(cam)
    cv2.imwrite(f"snaps/{cam}.png", color); np.save(f"snaps/{cam}_depth.npy", depth)
    np.save(f"snaps/{cam}_K.npy", K); np.save(f"snaps/{cam}_T.npy", T)
    print("K", K.tolist()); print("T", T.tolist())
    print("depth range", np.nanmin(depth), np.nanmax(depth))
    for a in sys.argv[2:]:
        u,v = map(int, a.split(","))
        print(f"px({u},{v}) depth={depth[v,u]:.4f} world={px2world(u,v,depth,K,T)}")
EOF
timeout 60 python3 scene.py agentview 222,220 390,240 330,220 325,370 320,450 100,300

# openrua op 8
cat > /workspace/birdseg.py <<'EOF'
import numpy as np, cv2, sys
sys.path.insert(0, "/workspace")
from scene import grab, cloud
color, depth, K, T = grab("birdview")
P = cloud(depth, K, T)
Z = P[...,2]
cv2.imwrite("snaps/birdview.png", color)
np.save("snaps/birdview_P.npy", P)
print("K", K.tolist())
# table height: mode of Z in central region
zc = Z[200:350, 150:500]
hist, edges = np.histogram(zc[np.isfinite(zc)], bins=200)
table_z = edges[np.argmax(hist)]
print("table_z ~", table_z)
mask = (Z > table_z + 0.01) & (Z < table_z + 0.35) & np.isfinite(Z)
# exclude robot: robot is anything with z > table+0.35 or connected... just cluster
n, lab, stats, cents = cv2.connectedComponentsWithStats(mask.astype(np.uint8), 8)
for i in range(1, n):
    x,y,w,h,a = stats[i]
    if a < 15: continue
    m = lab == i
    pts = P[m]
    print(f"comp {i}: px bbox x={x}..{x+w} y={y}..{y+h} area={a} centroid_px={cents[i]} "
          f"world xy=({pts[:,0].mean():.3f},{pts[:,1].mean():.3f}) xrange=({pts[:,0].min():.3f},{pts[:,0].max():.3f}) yrange=({pts[:,1].min():.3f},{pts[:,1].max():.3f}) zmax={pts[:,2].max():.3f} zmed={np.median(pts[:,2]):.3f}")
    bgr = color[m].mean(0)
    print("   mean color BGR", bgr)
EOF
timeout 90 python3 birdseg.py

# openrua op 9
cat > /workspace/seg.py <<'EOF'
import numpy as np, cv2, sys
sys.path.insert(0, "/workspace")
from scene import grab, cloud
cam = sys.argv[1]
color, depth, K, T = grab(cam)
P = cloud(depth, K, T)
Z = P[...,2]
cv2.imwrite(f"snaps/{cam}.png", color)
np.save(f"snaps/{cam}_P.npy", P)
# table: points in workspace region with z within 0.40..0.45
ws = (P[...,0] > -0.4) & (P[...,0] < 0.4) & (np.abs(P[...,1]) < 0.5) & np.isfinite(Z)
tz = Z[ws & (Z > 0.38) & (Z < 0.46)]
hist, edges = np.histogram(tz, bins=100)
table_z = edges[np.argmax(hist)]
print("table_z ~", table_z, "n", tz.size)
mask = ws & (Z > table_z + 0.012) & (Z < table_z + 0.30)
n, lab, stats, cents = cv2.connectedComponentsWithStats(mask.astype(np.uint8), 8)
for i in range(1, n):
    x,y,w,h,a = stats[i]
    if a < 30: continue
    m = lab == i
    pts = P[m]
    print(f"comp {i}: px bbox x={x}..{x+w} y={y}..{y+h} area={a} centroid_px=({cents[i][0]:.0f},{cents[i][1]:.0f}) "
          f"world xy=({pts[:,0].mean():.3f},{pts[:,1].mean():.3f}) xr=({pts[:,0].min():.3f},{pts[:,0].max():.3f}) yr=({pts[:,1].min():.3f},{pts[:,1].max():.3f}) zmax={pts[:,2].max():.3f} zmed={np.median(pts[:,2]):.3f}")
    print("   mean color BGR", color[m].mean(0).round(0))
EOF
timeout 90 python3 seg.py agentview

# openrua op 10
cat > /workspace/rim.py <<'EOF'
import numpy as np
P = np.load("snaps/birdview_P.npy")
Z = P[...,2]
def fit_circle(xy):
    x,y = xy[:,0], xy[:,1]
    A = np.c_[2*x, 2*y, np.ones_like(x)]
    b = x*x+y*y
    c, *_ = np.linalg.lstsq(A, b, rcond=None)
    r = np.sqrt(c[2] + c[0]**2 + c[1]**2)
    return c[0], c[1], r
# white mug: region x -0.2..-0.05, y -0.22..-0.05, z>0.55
for name, box, zlo in [("white mug rim", (-0.22,-0.05,-0.25,-0.04), 0.56),
                       ("red mug rim", (-0.12,0.05,0.0,0.2), 0.55),
                       ("plate", (0.03,0.25,-0.12,0.12), 0.47),
                       ("pudding", (-0.3,-0.15,-0.05,0.08), 0.46)]:
    m = (P[...,0]>box[0])&(P[...,0]<box[1])&(P[...,1]>box[2])&(P[...,1]<box[3])&(Z>zlo)&(Z<0.65)&np.isfinite(Z)
    pts = P[m]
    if len(pts)==0: print(name, "none"); continue
    cx,cy,r = fit_circle(pts[:,:2])
    print(f"{name}: n={len(pts)} circle c=({cx:.3f},{cy:.3f}) r={r:.3f} | xr=({pts[:,0].min():.3f},{pts[:,0].max():.3f}) yr=({pts[:,1].min():.3f},{pts[:,1].max():.3f}) z=({pts[:,2].min():.3f},{pts[:,2].max():.3f})")
EOF
python3 rim.py

# openrua op 11
timeout 300 python3 tools/action/fjt_send.py 0,-0.161037389,0,-2.44459747,0,2.2267522,0.7853981633974483 2 && timeout 90 python3 rim.py 2>/dev/null; timeout 90 python3 birdseg.py > /dev/null; python3 rim.py

# openrua op 12
timeout 90 python3 seg.py agentview && cat > /workspace/rim2.py <<'EOF'
import numpy as np, sys
P = np.load(f"snaps/{sys.argv[1]}_P.npy")
Z = P[...,2]
def fit_circle(xy):
    x,y = xy[:,0], xy[:,1]
    A = np.c_[2*x, 2*y, np.ones_like(x)]
    c, *_ = np.linalg.lstsq(A, x*x+y*y, rcond=None)
    return c[0], c[1], np.sqrt(c[2] + c[0]**2 + c[1]**2)
for name, box, zlo, zhi in [("white mug rim", (-0.22,-0.05,-0.25,-0.04), 0.54, 0.65),
                       ("white mug all", (-0.22,-0.05,-0.25,-0.04), 0.44, 0.65),
                       ("red mug rim", (-0.12,0.05,0.0,0.2), 0.54, 0.65),
                       ("plate", (0.03,0.25,-0.12,0.12), 0.43, 0.47),
                       ("pudding", (-0.3,-0.15,-0.05,0.08), 0.44, 0.50)]:
    m = (P[...,0]>box[0])&(P[...,0]<box[1])&(P[...,1]>box[2])&(P[...,1]<box[3])&(Z>zlo)&(Z<zhi)&np.isfinite(Z)
    pts = P[m]
    if len(pts)==0: print(name, "none"); continue
    cx,cy,r = fit_circle(pts[:,:2])
    print(f"{name}: n={len(pts)} circle c=({cx:.3f},{cy:.3f}) r={r:.3f} | xr=({pts[:,0].min():.3f},{pts[:,0].max():.3f}) yr=({pts[:,1].min():.3f},{pts[:,1].max():.3f}) z=({pts[:,2].min():.3f},{pts[:,2].max():.3f})")
EOF
python3 rim2.py agentview; echo ---bird---; python3 rim2.py birdview

# openrua op 13
cat > /workspace/arm.py <<'EOF'
#!/usr/bin/env python3
"""Arm control helpers: FK, IK, straight-line cartesian trajectories, gripper.
CLI:
  arm.py fk                       -> current TCP pose (world)
  arm.py open | close             -> gripper
  arm.py goto x y z [yaw_deg] [T] -> TCP to world pose, hand down, via IK+FJT (single point)
  arm.py line x y z [yaw_deg] [T] -> straight-line TCP move (multi-waypoint IK)
  arm.py joints p1,..,p7 T
"""
import sys, time, numpy as np, rclpy, yaml
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
TCP_OFF = float(M["hand"]["tcp_offset_m"])
BASE_T = np.eye(4); BASE_T[:3, 3] = [-0.51, 0.0, 0.42]   # world -> panda_link0 (from TF)

class Arm:
    def __init__(self):
        if not rclpy.ok(): rclpy.init()
        self.node = rclpy.create_node("arm_ctl")
        self.js = None
        self.node.create_subscription(JointState, "/joint_states", self._js, 10)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt.wait_for_server(10); self.grip.wait_for_server(10)
        self.ik.wait_for_service(10); self.fk.wait_for_service(10)
        while self.js is None: rclpy.spin_once(self.node, timeout_sec=0.2)

    def _js(self, m): self.js = m

    def spin(self, n=3):
        for _ in range(n): rclpy.spin_once(self.node, timeout_sec=0.1)

    def q(self):
        self.spin()
        d = dict(zip(self.js.name, self.js.position))
        return np.array([d[j] for j in JOINTS])

    def fingers(self):
        self.spin()
        d = dict(zip(self.js.name, self.js.position))
        return d["panda_finger_joint1"], d["panda_finger_joint2"]

    def fk_pose(self, q=None):
        """hand pose in WORLD: (pos, quat xyzw)"""
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = JOINTS
        req.robot_state.joint_state.position = list(self.q() if q is None else q)
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        r = fut.result()
        p = r.pose_stamped[0].pose
        pos = BASE_T[:3, :3] @ np.array([p.position.x, p.position.y, p.position.z]) + BASE_T[:3, 3]
        quat = np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        return pos, quat, r.error_code.val

    def tcp_pose(self, q=None):
        pos, quat, _ = self.fk_pose(q)
        R = Rot.from_quat(quat).as_matrix()
        return pos + TCP_OFF * R[:, 2], quat

    def solve_ik(self, tcp_pos_world, quat, seed=None, attempts=3):
        """IK for a TCP pose (world). Returns joint array or None."""
        R = Rot.from_quat(quat).as_matrix()
        hand_w = np.asarray(tcp_pos_world) - TCP_OFF * R[:, 2]
        hand_b = hand_w - BASE_T[:3, 3]           # base has identity rotation
        seed = self.q() if seed is None else seed
        for k in range(attempts):
            req = GetPositionIK.Request()
            req.ik_request.group_name = M["planning"]["group"]
            req.ik_request.pose_stamped.header.frame_id = ""
            req.ik_request.ik_link_name = "panda_hand"
            p = req.ik_request.pose_stamped.pose
            p.position.x, p.position.y, p.position.z = map(float, hand_b)
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
            req.ik_request.robot_state.joint_state.name = JOINTS
            s = np.array(seed) + (np.random.randn(7) * 0.1 if k else 0)
            req.ik_request.robot_state.joint_state.position = list(map(float, s))
            req.ik_request.avoid_collisions = False
            req.ik_request.timeout.sec = 2
            fut = self.ik.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            r = fut.result()
            if r is not None and r.error_code.val == 1:
                d = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
                sol = np.array([d[j] for j in JOINTS])
                # wrap-safety: pick the branch nearest the seed
                if np.abs(sol - seed).max() < 1.5 or k == attempts - 1:
                    return sol
                seed_alt = sol
            else:
                print(f"  IK attempt {k} failed code={None if r is None else r.error_code.val}")
        return None

    def send_traj(self, points, times, wait=True):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        for p, t in zip(points, times):
            pt = JointTrajectoryPoint(positions=list(map(float, p)))
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        fut = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        gh = fut.result()
        if gh is None or not gh.accepted:
            print("  goal rejected"); return None
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=900)
        res = rf.result()
        code = res.result.error_code if res else None
        err = np.abs(self.q() - np.array(points[-1])).max()
        print(f"  traj done code={code} max_joint_err={err:.4f}")
        return code

    def move_joints(self, qt, T=3.0):
        return self.send_traj([qt], [T])

    def goto(self, tcp, quat, T=3.0):
        sol = self.solve_ik(tcp, quat)
        if sol is None: print("  IK FAILED, no motion"); return None
        return self.move_joints(sol, T)

    def line(self, tcp_to, quat, T=None, step=0.02):
        """straight-line TCP move from current pose; multi-waypoint IK."""
        p0, _ = self.tcp_pose()
        p1 = np.asarray(tcp_to, float)
        n = max(2, int(np.ceil(np.linalg.norm(p1 - p0) / step)) + 1)
        seed = self.q(); pts = []
        for i in range(1, n):
            p = p0 + (p1 - p0) * i / (n - 1)
            sol = self.solve_ik(p, quat, seed=seed)
            if sol is None: print(f"  IK FAILED at waypoint {i}/{n-1} {p}"); return None
            pts.append(sol); seed = sol
        T = T or max(1.0, np.linalg.norm(p1 - p0) / 0.05)
        times = [T * (i + 1) / len(pts) for i in range(len(pts))]
        return self.send_traj(pts, times)

    def gripper(self, width):
        g = GripperCommand.Goal()
        g.command.position = float(width); g.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result
        # settle ticks
        self.spin(10)
        print(f"  gripper reached={r.reached_goal} stalled={r.stalled} fingers={self.fingers()}")
        return r

def down_quat(yaw_deg=0.0):
    """hand pointing down (hand z = -world z); yaw about world z. yaw=0: hand x = world x, hand y = -world y."""
    return (Rot.from_euler("z", yaw_deg, degrees=True) * Rot.from_euler("x", 180, degrees=True)).as_quat()

if __name__ == "__main__":
    a = Arm()
    cmd = sys.argv[1]
    if cmd == "fk":
        p, qt = a.tcp_pose(); print("TCP", p.round(4), "quat", qt.round(4), "joints", a.q().round(3), "fingers", a.fingers())
    elif cmd == "open": a.gripper(GRIP["open_m"])
    elif cmd == "close": a.gripper(GRIP["closed_m"])
    elif cmd in ("goto", "line"):
        x, y, z = map(float, sys.argv[2:5])
        yaw = float(sys.argv[5]) if len(sys.argv) > 5 else 0.0
        T = float(sys.argv[6]) if len(sys.argv) > 6 else None
        (a.goto if cmd == "goto" else a.line)([x, y, z], down_quat(yaw), T or (3.0 if cmd == "goto" else None))
        p, qt = a.tcp_pose(); print("TCP now", p.round(4), "quat", qt.round(4))
    elif cmd == "joints":
        a.move_joints([float(v) for v in sys.argv[2].split(",")], float(sys.argv[3]))
    rclpy.shutdown()
EOF
timeout 120 python3 arm.py fk

# openrua op 14
cat > /workspace/fkcheck.py <<'EOF'
import numpy as np, rclpy, sys
sys.path.insert(0,"/workspace")
from arm import Arm, JOINTS
from moveit_msgs.srv import GetPositionFK
a = Arm()
req = GetPositionFK.Request(); req.header.frame_id = ""
req.fk_link_names = ["panda_link0","panda_link8","panda_hand"]
req.robot_state.joint_state.name = JOINTS
req.robot_state.joint_state.position = list(a.q())
fut = a.fk.call_async(req); rclpy.spin_until_future_complete(a.node, fut, timeout_sec=30)
r = fut.result()
print("code", r.error_code.val)
for ps in r.pose_stamped:
    p = ps.pose; print(ps.header.frame_id, p.position.x, p.position.y, p.position.z, "|", p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
# TF chain
from tf2_ros import Buffer, TransformListener
import rclpy.time
buf = Buffer(); TransformListener(buf, a.node)
import time
t0=time.time()
while time.time()-t0<5: rclpy.spin_once(a.node, timeout_sec=0.1)
for f in ["panda_link0","panda_hand","robot0_eye_in_hand_optical_frame"]:
    try:
        t = buf.lookup_transform("world", f, rclpy.time.Time()).transform
        print("TF world->",f, t.translation.x, t.translation.y, t.translation.z, "|", t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w)
    except Exception as e: print("TF fail", f, e)
EOF
timeout 120 python3 fkcheck.py

# openrua op 15
cat > /workspace/ikcheck.py <<'EOF'
import numpy as np, rclpy, sys
sys.path.insert(0,"/workspace")
from arm import Arm, JOINTS, M
from moveit_msgs.srv import GetPositionIK
a = Arm()
q0 = a.q()
def ik(pos, quat):
    req = GetPositionIK.Request()
    req.ik_request.group_name = M["planning"]["group"]
    req.ik_request.pose_stamped.header.frame_id = ""
    req.ik_request.ik_link_name = "panda_hand"
    p = req.ik_request.pose_stamped.pose
    p.position.x, p.position.y, p.position.z = map(float, pos)
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
    req.ik_request.robot_state.joint_state.name = JOINTS
    req.ik_request.robot_state.joint_state.position = list(map(float, q0))
    req.ik_request.timeout.sec = 2
    fut = a.ik.call_async(req); rclpy.spin_until_future_complete(a.node, fut, timeout_sec=60)
    r = fut.result()
    if r.error_code.val != 1: return r.error_code.val, None
    d = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
    return 1, np.array([d[j] for j in JOINTS])
quat = [0.9995966352021468, 0, -0.028400121347386478, 0]
print("world-frame pose:", ik([-0.052985648078355034, 0, 0.7776238083193782], quat), "q0", q0.round(3))
print("base-frame pose:", ik([-0.052985648078355034+0.51, 0, 0.7776238083193782-0.42], quat))
EOF
timeout 120 python3 ikcheck.py

# openrua op 16
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/arm.py", "old_string": "BASE_T = np.eye(4); BASE_T[:3, 3] = [-0.51, 0.0, 0.42]   # world -> panda_link0 (from TF)", "new_string": "# verified: /compute_fk and /compute_ik on this machine both use WORLD coordinates\\n# (panda_link0 sits at world (-0.51, 0, 0.42)); no base offset needed.\\nBASE_T = np.eye(4)", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 17
timeout 120 python3 arm.py fk

# openrua op 18
timeout 600 python3 arm.py goto -0.126 -0.191 0.63 0 3 && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih_1.png

# openrua op 19
cat > /workspace/eih_fit.py <<'EOF'
import numpy as np, sys
sys.path.insert(0, "/workspace")
from scene import grab, cloud
color, depth, K, T = grab("robot0_eye_in_hand")
P = cloud(depth, K, T); Z = P[...,2]
import cv2; cv2.imwrite("snaps/eih.png", color); np.save("snaps/eih_P.npy", P)
def fit_circle(xy):
    x,y = xy[:,0], xy[:,1]
    A = np.c_[2*x, 2*y, np.ones_like(x)]
    c, *_ = np.linalg.lstsq(A, x*x+y*y, rcond=None)
    return c[0], c[1], np.sqrt(c[2] + c[0]**2 + c[1]**2)
box = [float(v) for v in sys.argv[1:5]]; zlo, zhi = float(sys.argv[5]), float(sys.argv[6])
m = (P[...,0]>box[0])&(P[...,0]<box[1])&(P[...,1]>box[2])&(P[...,1]<box[3])&(Z>zlo)&(Z<zhi)&np.isfinite(Z)
pts = P[m]
print("n", len(pts))
if len(pts):
    cx,cy,r = fit_circle(pts[:,:2])
    print(f"circle c=({cx:.4f},{cy:.4f}) r={r:.4f} | xr=({pts[:,0].min():.3f},{pts[:,0].max():.3f}) yr=({pts[:,1].min():.3f},{pts[:,1].max():.3f}) z=({pts[:,2].min():.3f},{pts[:,2].max():.3f}) mean=({pts[:,0].mean():.4f},{pts[:,1].mean():.4f})")
EOF
timeout 90 python3 eih_fit.py -0.22 -0.05 -0.25 -0.09 0.535 0.56

# openrua op 20
timeout 900 python3 arm.py line -0.124 -0.1905 0.52 0 4 && timeout 300 python3 arm.py close && timeout 60 python3 tools/perception/cam_snap.py agentview snaps/agent_grasp.png

# openrua op 21
timeout 900 python3 arm.py line -0.124 -0.1905 0.66 0 4 && timeout 60 python3 tools/perception/cam_snap.py agentview snaps/agent_lift.png && timeout 90 python3 rim2.py agentview 2>/dev/null | head -0; timeout 90 python3 seg.py agentview | grep -A1 comp

# openrua op 22
timeout 900 python3 arm.py line -0.124 -0.1905 0.74 0 3 && timeout 900 python3 arm.py line 0.130 -0.0375 0.74 0 5 && timeout 60 python3 tools/perception/cam_snap.py agentview snaps/agent_overplate.png

# openrua op 23
timeout 900 python3 arm.py line 0.130 -0.0375 0.56 0 5 && timeout 300 python3 arm.py open && timeout 900 python3 arm.py line 0.130 -0.0375 0.70 0 3 && timeout 60 python3 tools/perception/cam_snap.py agentview snaps/agent_placed.png && timeout 90 python3 seg.py agentview | grep -A1 comp

# openrua op 24
timeout 900 python3 arm.py goto -0.222 0.022 0.62 0 4 && cat > /workspace/boxfit.py <<'EOF'
import numpy as np, sys, cv2
sys.path.insert(0, "/workspace")
from scene import grab, cloud
cam = sys.argv[1]
color, depth, K, T = grab(cam)
P = cloud(depth, K, T); Z = P[...,2]
cv2.imwrite(f"snaps/{cam}_box.png", color)
box = [float(v) for v in sys.argv[2:6]]; zlo, zhi = float(sys.argv[6]), float(sys.argv[7])
m = (P[...,0]>box[0])&(P[...,0]<box[1])&(P[...,1]>box[2])&(P[...,1]<box[3])&(Z>zlo)&(Z<zhi)&np.isfinite(Z)
pts = P[m][:, :2]
print("n", len(pts))
c = pts.mean(0)
u, s, vt = np.linalg.svd(pts - c, full_matrices=False)
ax = vt[0]; yaw = np.degrees(np.arctan2(ax[1], ax[0]))
proj = (pts - c) @ vt.T
print(f"center=({c[0]:.4f},{c[1]:.4f}) long-axis yaw={yaw:.1f}deg extents long={proj[:,0].min():.3f}..{proj[:,0].max():.3f} short={proj[:,1].min():.3f}..{proj[:,1].max():.3f} z=({P[m][:,2].min():.3f},{P[m][:,2].max():.3f})")
EOF
timeout 90 python3 boxfit.py robot0_eye_in_hand -0.30 -0.15 -0.05 0.10 0.455 0.48

# openrua op 25
timeout 60 python3 tools/perception/cam_snap.py agentview snaps/agent_abort.png && timeout 120 python3 arm.py fk

# openrua op 26
cat > /workspace/iktest.py <<'EOF'
import numpy as np, sys
sys.path.insert(0,"/workspace")
from arm import Arm, down_quat
a = Arm()
q0 = a.q(); print("q0", q0.round(3))
for tgt in [(-0.222,0.022,0.62), (-0.222,0.022,0.55), (-0.222,0.022,0.44), (-0.15,0.022,0.62), (-0.222,0.022,0.72)]:
    for seed_name, seed in [("current", q0), ("home", np.array([0,-0.161,0,-2.445,0,2.227,0.785]))]:
        sol = a.solve_ik(tgt, down_quat(0), seed=seed, attempts=1)
        print(tgt, seed_name, None if sol is None else sol.round(3), None if sol is None else f"dist={np.abs(sol-seed).max():.2f}")
EOF
timeout 300 python3 iktest.py

# openrua op 27
cat > /workspace/iktest2.py <<'EOF'
import numpy as np, sys, rclpy
sys.path.insert(0,"/workspace")
from arm import Arm, down_quat, JOINTS, M
from scipy.spatial.transform import Rotation as Rot
from moveit_msgs.srv import GetStateValidity
a = Arm()
sv = a.node.create_client(GetStateValidity, "/check_state_validity"); sv.wait_for_service(10)
def valid(q):
    req = GetStateValidity.Request(); req.group_name = M["planning"]["group"]
    req.robot_state.joint_state.name = JOINTS; req.robot_state.joint_state.position = list(map(float,q))
    fut = sv.call_async(req); rclpy.spin_until_future_complete(a.node, fut, timeout_sec=30)
    r = fut.result(); return r.valid, [(c.contact_body_1, c.contact_body_2) for c in r.contacts]
def pitched(pitch_deg, yaw_deg=0):
    return (Rot.from_euler("z", yaw_deg, degrees=True) * Rot.from_euler("y", pitch_deg, degrees=True) * Rot.from_euler("x", 180, degrees=True)).as_quat()
q0 = a.q(); home = np.array([0,-0.161,0,-2.445,0,2.227,0.785])
print("current valid?", valid(q0))
for pitch in [0, 15, 25, 35]:
    for tgt in [(-0.222,0.022,0.62), (-0.222,0.022,0.52), (-0.222,0.022,0.445)]:
        sol = a.solve_ik(tgt, pitched(pitch), seed=home, attempts=2)
        if sol is None: print(f"pitch={pitch} {tgt}: IK none"); continue
        v = valid(sol)
        print(f"pitch={pitch} {tgt}: q={sol.round(2)} j4={sol[3]:.2f} valid={v}")
EOF
timeout 600 python3 iktest2.py

# openrua op 28
python3 - <<'EOF'
import re
s = open("arm.py").read()
s = s.replace('''    def line(self, tcp_to, quat, T=None, step=0.02):
        """straight-line TCP move from current pose; multi-waypoint IK."""
        p0, _ = self.tcp_pose()
        p1 = np.asarray(tcp_to, float)
        n = max(2, int(np.ceil(np.linalg.norm(p1 - p0) / step)) + 1)
        seed = self.q(); pts = []
        for i in range(1, n):
            p = p0 + (p1 - p0) * i / (n - 1)
            sol = self.solve_ik(p, quat, seed=seed)''',
'''    def line(self, tcp_to, quat, T=None, step=0.02):
        """straight-line TCP move from current pose (slerp orientation); multi-waypoint IK."""
        p0, q0 = self.tcp_pose()
        p1 = np.asarray(tcp_to, float)
        ang = (Rot.from_quat(q0).inv() * Rot.from_quat(quat)).magnitude()
        n = max(2, int(np.ceil(np.linalg.norm(p1 - p0) / step)) + 1, int(np.ceil(ang / 0.1)) + 1)
        slerp = Slerp([0, 1], Rot.from_quat([q0, quat]))
        seed = self.q(); pts = []
        for i in range(1, n):
            f = i / (n - 1)
            p = p0 + (p1 - p0) * f
            sol = self.solve_ik(p, slerp(f).as_quat(), seed=seed)''')
s = s.replace("from scipy.spatial.transform import Rotation as Rot", "from scipy.spatial.transform import Rotation as Rot, Slerp")
s = s.replace('''def down_quat(yaw_deg=0.0):
    """hand pointing down (hand z = -world z); yaw about world z. yaw=0: hand x = world x, hand y = -world y."""
    return (Rot.from_euler("z", yaw_deg, degrees=True) * Rot.from_euler("x", 180, degrees=True)).as_quat()''',
'''def down_quat(yaw_deg=0.0, pitch_deg=0.0):
    """hand pointing down (hand z = -world z); yaw about world z; pitch>0 tilts the
    approach toward -x (hand body sits on the +x side). yaw=0: fingers open along world y."""
    return (Rot.from_euler("z", yaw_deg, degrees=True) * Rot.from_euler("y", pitch_deg, degrees=True)
            * Rot.from_euler("x", 180, degrees=True)).as_quat()''')
s = s.replace('''        yaw = float(sys.argv[5]) if len(sys.argv) > 5 else 0.0
        T = float(sys.argv[6]) if len(sys.argv) > 6 else None
        (a.goto if cmd == "goto" else a.line)([x, y, z], down_quat(yaw), T or (3.0 if cmd == "goto" else None))''',
'''        yaw = float(sys.argv[5]) if len(sys.argv) > 5 else 0.0
        T = float(sys.argv[6]) if len(sys.argv) > 6 else None
        pitch = float(sys.argv[7]) if len(sys.argv) > 7 else 0.0
        (a.goto if cmd == "goto" else a.line)([x, y, z], down_quat(yaw, pitch), T or (3.0 if cmd == "goto" else None))''')
s = s.replace("  arm.py line x y z [yaw_deg] [T] -> straight-line TCP move (multi-waypoint IK)",
              "  arm.py line x y z [yaw_deg] [T] [pitch_deg] -> straight-line TCP move (multi-waypoint IK)")
open("arm.py","w").write(s)
EOF
grep -n "Slerp\|pitch" arm.py | head

# openrua op 29
cat > /workspace/pudding_pre.py <<'EOF'
import numpy as np, sys, rclpy
sys.path.insert(0,"/workspace")
from arm import Arm, down_quat
a = Arm()
HOME = np.array([0,-0.161,0,-2.445,0,2.227,0.785])
PITCH = 20
p, _ = a.tcp_pose()
print("1. rise"); a.line([p[0], p[1], 0.80], down_quat(0, 0), T=3)
print("2. pre-grasp (home-seeded)")
sol = a.solve_ik([-0.222, 0.022, 0.62], down_quat(0, PITCH), seed=HOME)
print("   sol", sol.round(3)); a.move_joints(sol, 5)
p, q = a.tcp_pose(); print("TCP", p.round(4), q.round(3))
EOF
timeout 1200 python3 pudding_pre.py && timeout 90 python3 boxfit.py robot0_eye_in_hand -0.30 -0.15 -0.05 0.10 0.455 0.48

# openrua op 30
timeout 900 python3 arm.py line -0.2105 0.0225 0.445 0 5 20 && timeout 300 python3 arm.py close && timeout 900 python3 arm.py line -0.2105 0.0225 0.60 0 4 20 && timeout 60 python3 tools/perception/cam_snap.py agentview snaps/agent_pudlift.png

# openrua op 31
timeout 900 python3 arm.py line -0.2105 0.0225 0.72 0 3 20 && timeout 1200 python3 arm.py line 0.131 0.145 0.72 0 6 20 && timeout 900 python3 arm.py line 0.131 0.145 0.457 0 5 20 && timeout 300 python3 arm.py open && timeout 900 python3 arm.py line 0.131 0.145 0.62 0 3 20 && timeout 60 python3 tools/perception/cam_snap.py agentview snaps/agent_final.png

# openrua op 32
timeout 900 python3 arm.py line 0.0 0.0 0.75 0 5 0 && timeout 600 python3 arm.py joints 0,-0.161037389,0,-2.44459747,0,2.2267522,0.7853981633974483 4 && timeout 90 python3 seg.py agentview | grep -A1 comp && echo --- && timeout 90 python3 birdseg.py | grep -A1 comp

# openrua op 33
cat > /workspace/final_check.py <<'EOF'
import numpy as np
P = np.load("snaps/birdview_P.npy"); Z = P[...,2]
def fit_circle(xy):
    x,y = xy[:,0], xy[:,1]
    A = np.c_[2*x, 2*y, np.ones_like(x)]
    c, *_ = np.linalg.lstsq(A, x*x+y*y, rcond=None)
    return c[0], c[1], np.sqrt(c[2] + c[0]**2 + c[1]**2)
sel = lambda box, zlo, zhi: P[(P[...,0]>box[0])&(P[...,0]<box[1])&(P[...,1]>box[2])&(P[...,1]<box[3])&(Z>zlo)&(Z<zhi)&np.isfinite(Z)]
rim = sel((0.03,0.25,-0.12,0.12), 0.545, 0.60)
cx,cy,r = fit_circle(rim[:,:2]); print(f"white mug rim on plate: c=({cx:.3f},{cy:.3f}) r={r:.3f} z=({rim[:,2].min():.3f},{rim[:,2].max():.3f}) n={len(rim)}")
plate = sel((0.03,0.25,-0.12,0.12), 0.435, 0.47)
print(f"plate footprint: x=({plate[:,0].min():.3f},{plate[:,0].max():.3f}) y=({plate[:,1].min():.3f},{plate[:,1].max():.3f}) -> center ({(plate[:,0].min()+plate[:,0].max())/2:.3f},{(plate[:,1].min()+plate[:,1].max())/2:.3f})")
box = sel((0.05,0.25,0.10,0.25), 0.44, 0.50)
print(f"pudding: center=({box[:,0].mean():.3f},{box[:,1].mean():.3f}) x=({box[:,0].min():.3f},{box[:,0].max():.3f}) y=({box[:,1].min():.3f},{box[:,1].max():.3f}) top z=({box[:,2].min():.3f},{box[:,2].max():.3f})")
EOF
python3 final_check.py

# openrua op 34
timeout 60 python3 tools/perception/cam_snap.py agentview snaps/agent_done.png
