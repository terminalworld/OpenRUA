#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la && find . -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
cat tools/action/fjt_send.py; echo ----; cat tools/action/gripper_cmd.py; echo ----; cat tools/action/ik_move.py

# openrua op 3
timeout 30 ros2 topic list; echo ----; timeout 20 ros2 action list; echo ----; timeout 20 ros2 service list | grep -iv parameter | head -40; echo ----; timeout 20 ros2 node list

# openrua op 4
for c in agentview frontview sideview birdview robot0_eye_in_hand robot0_robotview galleryview paperview; do timeout 60 python3 tools/perception/cam_snap.py $c; done; timeout 20 ros2 topic echo /joint_states --once

# openrua op 5
timeout 30 ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id|translation|x:|y:|z:|w:" | head -80; echo ----; timeout 20 ros2 topic echo /agentview/color/camera_info --once

# openrua op 6
mkdir -p "$(dirname /workspace/tfdump.py)"
cat > /workspace/tfdump.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Dump all TF frames and world->frame transforms; also FK of panda_hand."""
import rclpy, yaml
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

rclpy.init()
node = rclpy.create_node("tfdump")
buf = Buffer()
TransformListener(buf, node)
frames = {}
def cb(m):
    for t in m.transforms:
        frames[t.child_frame_id] = t.header.frame_id
qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                 reliability=ReliabilityPolicy.RELIABLE)
node.create_subscription(TFMessage, "/tf_static", cb, qos)
node.create_subscription(TFMessage, "/tf", cb, 100)
for _ in range(30):
    rclpy.spin_once(node, timeout_sec=0.2)
print("edges (child <- parent):")
for c, p in sorted(frames.items()):
    print(f"  {c} <- {p}")
print()
for f in sorted(frames):
    try:
        t = buf.lookup_transform("world", f, Time())
        tr, q = t.transform.translation, t.transform.rotation
        print(f"world->{f}: t=({tr.x:.4f},{tr.y:.4f},{tr.z:.4f}) q=({q.x:.4f},{q.y:.4f},{q.z:.4f},{q.w:.4f})")
    except Exception as e:
        print(f"world->{f}: FAIL {type(e).__name__}")
rclpy.shutdown()
OPENRUA_EOF

# openrua op 7
timeout 60 python3 tfdump.py

# openrua op 8
mkdir -p "$(dirname /workspace/scene.py)"
cat > /workspace/scene.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Segment objects above the table from a camera's depth image -> world coords.
Usage: python3 scene.py <camera> [min_height_above_table=0.01]
"""
import sys
import numpy as np
import rclpy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge
import cv2

cam = sys.argv[1]
hmin = float(sys.argv[2]) if len(sys.argv) > 2 else 0.01

rclpy.init()
node = rclpy.create_node("scene")
buf = Buffer(); TransformListener(buf, node)
got = {}
node.create_subscription(Image, f"/{cam}/depth/image_raw", lambda m: got.setdefault("d", m), 1)
node.create_subscription(Image, f"/{cam}/color/image_raw", lambda m: got.setdefault("c", m), 1)
node.create_subscription(CameraInfo, f"/{cam}/color/camera_info", lambda m: got.setdefault("i", m), 1)
frame = f"{cam}_optical_frame"
while not ({"d", "c", "i"} <= set(got) and buf.can_transform("world", frame, Time())):
    rclpy.spin_once(node, timeout_sec=0.2)
depth = CvBridge().imgmsg_to_cv2(got["d"], "passthrough").astype(np.float64)
color = CvBridge().imgmsg_to_cv2(got["c"], "bgr8")
k = got["i"].k; fx, fy, cx, cy = k[0], k[4], k[2], k[5]
t = buf.lookup_transform("world", frame, Time())
q = t.transform.rotation; x, y, z, w = q.x, q.y, q.z, q.w
R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
              [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
              [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
T = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
H, W = depth.shape
vv, uu = np.mgrid[0:H, 0:W]
Z = depth
X = (uu - cx) * Z / fx; Y = (vv - cy) * Z / fy
P = np.stack([X, Y, Z], -1) @ R.T + T   # HxWx3 world
valid = np.isfinite(Z) & (Z > 0.05)
zs = P[..., 2][valid]
# table height = most common z in the middle band
hist, edges = np.histogram(zs, bins=400)
table_z = edges[np.argmax(hist)] + (edges[1]-edges[0])/2
print(f"table_z ~ {table_z:.4f}")
np.save(f"{cam}_world.npy", P)
above = valid & (P[..., 2] > table_z + hmin) & (P[..., 2] < table_z + 0.5)
mask = above.astype(np.uint8)
n, lab, stats, cents = cv2.connectedComponentsWithStats(mask, 8)
for i in range(1, n):
    if stats[i, cv2.CC_STAT_AREA] < 30: continue
    m = lab == i
    pts = P[m]
    lo, hi = pts.min(0), pts.max(0)
    cu, cv_ = cents[i]
    print(f"blob {i}: px=({cu:.0f},{cv_:.0f}) area={stats[i,4]} "
          f"x[{lo[0]:.3f},{hi[0]:.3f}] y[{lo[1]:.3f},{hi[1]:.3f}] z[{lo[2]:.3f},{hi[2]:.3f}] "
          f"center=({pts[:,0].mean():.3f},{pts[:,1].mean():.3f}) top={hi[2]:.3f}")
    x0, y0, w_, h_ = stats[i, :4]
    cv2.rectangle(color, (x0, y0), (x0+w_, y0+h_), (0, 255, 0), 1)
    cv2.putText(color, str(i), (x0, y0-2), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
cv2.imwrite(f"{cam}_blobs.png", color)
rclpy.shutdown()
OPENRUA_EOF

# openrua op 9
timeout 90 python3 scene.py birdview 0.01

# openrua op 10
python3 -c "
import numpy as np
P=np.load('birdview_world.npy'); z=P[...,2]; v=np.isfinite(z)&(z>0.3)
h,e=np.histogram(z[v],bins=np.arange(0.3,1.6,0.01))
for c,a in zip(h,e):
    if c>200: print(f'{a:.2f} {c}')
"

# openrua op 11
python3 - <<'EOF'
import numpy as np, cv2
P=np.load('birdview_world.npy'); z=P[...,2]
table=0.895
valid=np.isfinite(z)&(z>0.3)
above=valid&(z>table+0.008)&(z<table+0.4)&(P[...,0]>-0.4)&(P[...,0]<0.9)
mask=above.astype(np.uint8)
n,lab,stats,cents=cv2.connectedComponentsWithStats(mask,8)
color=cv2.imread('birdview.png')
for i in range(1,n):
    if stats[i,4]<15: continue
    pts=P[lab==i]; lo,hi=pts.min(0),pts.max(0)
    print(f"blob {i}: px=({cents[i][0]:.0f},{cents[i][1]:.0f}) area={stats[i,4]} x[{lo[0]:.3f},{hi[0]:.3f}] y[{lo[1]:.3f},{hi[1]:.3f}] z[{lo[2]:.3f},{hi[2]:.3f}] center=({pts[:,0].mean():.3f},{pts[:,1].mean():.3f})")
    x0,y0,w,h=stats[i,:4]; cv2.rectangle(color,(x0,y0),(x0+w,y0+h),(0,255,0),1); cv2.putText(color,str(i),(x0,y0-2),cv2.FONT_HERSHEY_SIMPLEX,0.4,(0,255,0),1)
cv2.imwrite('birdview_blobs.png',color)
EOF

# openrua op 12
timeout 90 python3 scene.py frontview 0.01 >/dev/null; timeout 90 python3 scene.py sideview 0.01 > /dev/null; python3 - <<'EOF'
import numpy as np
for cam in ['frontview','sideview']:
    P=np.load(f'{cam}_world.npy')
    for name,(cx,cy) in {'potA':(-0.197,-0.20),'potB':(-0.06,0.25),'knob':(0.031,0.029),'stove':(0.183,0.032)}.items():
        m=np.isfinite(P[...,2])&(np.abs(P[...,0]-cx)<0.09)&(np.abs(P[...,1]-cy)<0.09)&(P[...,2]>0.9)
        pts=P[m]
        print(cam,name,len(pts))
        for z0 in np.arange(0.90,1.07,0.01):
            s=pts[(pts[:,2]>=z0)&(pts[:,2]<z0+0.01)]
            if len(s)<3: continue
            print(f"  z {z0:.2f}: n={len(s)} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}] wx={s[:,0].max()-s[:,0].min():.3f} wy={s[:,1].max()-s[:,1].min():.3f}")
EOF

# openrua op 13
python3 -c "
import cv2
im=cv2.imread('frontview.png'); c=im[270:400,170:290]; cv2.imwrite('potA_crop.png',cv2.resize(c,None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('sideview.png'); c=im[220:300,320:400]; cv2.imwrite('potA_side_crop.png',cv2.resize(c,None,fx=5,fy=5,interpolation=cv2.INTER_CUBIC))
"

# openrua op 14
timeout 30 ros2 service call /get_urdf std_srvs/srv/Trigger 2>/dev/null | head -c 300; echo; timeout 30 ros2 interface show std_srvs/srv/Trigger >/dev/null 2>&1; timeout 20 ros2 service type /get_urdf

# openrua op 15
timeout 60 python3 - <<'EOF'
import rclpy
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy
rclpy.init(); n=rclpy.create_node("urdf")
got=[]
n.create_subscription(String,"/robot_description",got.append,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
while not got: rclpy.spin_once(n,timeout_sec=0.5)
open("robot.urdf","w").write(got[0].data); print(len(got[0].data))
EOF
grep -n -A12 'link name="panda_hand"' robot.urdf | head -60; grep -n -B2 -A14 'link name="panda_leftfinger"' robot.urdf | head -60; grep -n -A8 'joint name="panda_finger_joint1"' robot.urdf

# openrua op 16
timeout 120 python3 tools/action/gripper_cmd.py 0.04; timeout 20 ros2 topic echo /joint_states --once | grep -A9 position

# openrua op 17
timeout 90 python3 scene.py sideview 0.01 > /dev/null; timeout 90 python3 scene.py birdview 0.01 > /dev/null; python3 - <<'EOF'
import numpy as np
P=np.load('birdview_world.npy'); z=P[...,2]
for name,(cx,cy) in {'potA':(-0.197,-0.20),'potB':(-0.06,0.25),'knob':(0.031,0.029),'stove':(0.183,0.032)}.items():
    m=np.isfinite(z)&(np.abs(P[...,0]-cx)<0.09)&(np.abs(P[...,1]-cy)<0.09)&(z>0.9)
    pts=P[m]; print(name,'zmin %.3f zmax %.3f'%(pts[:,2].min(),pts[:,2].max()))
    top=pts[pts[:,2]>pts[:,2].max()-0.01]; print('  top center',top[:,:2].mean(0))
P=np.load('sideview_world.npy')
for name,(cx,cy) in {'potA':(-0.197,-0.20),'potB':(-0.06,0.25)}.items():
    m=np.isfinite(P[...,2])&(np.abs(P[...,0]-cx)<0.09)&(np.abs(P[...,1]-cy)<0.09)&(P[...,2]>0.9)
    pts=P[m]
    for z0 in np.arange(0.90,1.06,0.005):
        s=pts[(pts[:,2]>=z0)&(pts[:,2]<z0+0.005)]
        if len(s)<3: continue
        print(f"  {name} z {z0:.3f}: n={len(s)} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] wx={s[:,0].max()-s[:,0].min():.3f} xc={(s[:,0].max()+s[:,0].min())/2:.4f}")
EOF

# openrua op 18
mkdir -p "$(dirname /workspace/rob.py)"
cat > /workspace/rob.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Robot helper: joint state, FK, IK, trajectory, gripper (one node, reused clients)."""
import math, sys, time
import numpy as np
import rclpy, yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import Pose, TwistStamped
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
JOINTS = FJT["joints"]
BASE = np.array([-0.66, 0.0, 0.912])  # world -> panda_link0 (from TF)
TCP = M["hand"]["tcp_offset_m"]


def quat_to_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def R_to_quat(R):
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1) * 2
        return np.array([(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, s / 4])
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(1 + R[i, i] - R[j, j] - R[k, k]) * 2
    q = np.zeros(4)
    q[i] = s / 4
    q[j] = (R[j, i] + R[i, j]) / s
    q[k] = (R[k, i] + R[i, k]) / s
    q[3] = (R[k, j] - R[j, k]) / s
    return q


class Robot:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("rob_" + str(int(time.time() * 1000) % 100000))
        self._js = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik_cli = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.twist_pub = self.node.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)
        self.fjt.wait_for_server(10); self.grip.wait_for_server(10)
        self.ik_cli.wait_for_service(10); self.fk_cli.wait_for_service(10)

    def _on_js(self, m):
        self._js = m

    def spin(self, t=0.1):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self, fresh=True):
        if fresh:
            self._js = None
            while self._js is None:
                self.spin(0.2)
        d = dict(zip(self._js.name, self._js.position))
        return d

    def arm(self):
        d = self.joints()
        return np.array([d[j] for j in JOINTS])

    def fingers(self):
        d = self.joints()
        return d["panda_finger_joint1"], d["panda_finger_joint2"]

    # ---- kinematics (poses in WORLD frame; converted to base frame for MoveIt)
    def fk(self, q=None, link="panda_hand"):
        req = GetPositionFK.Request()
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = list(JOINTS)
        req.robot_state.joint_state.position = list(map(float, self.arm() if q is None else q))
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {None if res is None else res.error_code.val}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE
        quat = np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        return pos, quat

    def ik(self, pos_world, quat, seed=None, timeout=5.0):
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        req.ik_request.ik_link_name = "panda_hand"
        p = req.ik_request.pose_stamped.pose
        pb = np.asarray(pos_world, float) - BASE
        p.position.x, p.position.y, p.position.z = map(float, pb)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.robot_state.joint_state.name = list(JOINTS)
        req.ik_request.robot_state.joint_state.position = list(map(float, self.arm() if seed is None else seed))
        req.ik_request.timeout = Duration(sec=int(timeout), nanosec=int((timeout % 1) * 1e9))
        req.ik_request.avoid_collisions = False
        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None:
            raise RuntimeError("IK no answer")
        if res.error_code.val != 1:
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return np.array([sol[j] for j in JOINTS])

    # ---- motion
    def move_joints(self, targets, seconds, wait=True):
        """targets: list of joint arrays (waypoints); seconds: list of times or total."""
        if isinstance(targets, np.ndarray) and targets.ndim == 1:
            targets = [targets]
        if not isinstance(seconds, (list, tuple)):
            n = len(targets)
            seconds = [seconds * (i + 1) / n for i in range(n)]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        for q, t in zip(targets, seconds):
            pt = JointTrajectoryPoint(positions=list(map(float, q)))
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        if gh is None or not gh.accepted:
            raise RuntimeError("trajectory goal rejected")
        if not wait:
            return gh
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=600)
        code = rf.result().result.error_code
        q = self.arm()
        err = np.abs(q - np.asarray(targets[-1])).max()
        print(f"  traj done code={code} max_joint_err={err:.4f}", flush=True)
        return code, err

    def move_pose(self, pos, quat, seconds=3.0, seed=None, via=None):
        q = self.ik(pos, quat, seed=seed)
        if q is None:
            raise RuntimeError(f"IK failed for {pos}")
        cur = self.arm()
        # keep joint7 / wrist continuity: unwrap solution near current config
        for i in (0, 2, 4, 6):
            while q[i] - cur[i] > math.pi: q[i] -= 2 * math.pi
            while q[i] - cur[i] < -math.pi: q[i] += 2 * math.pi
        lim = FJT["limits_rad"]
        for i, (lo, hi) in enumerate(lim):
            if not (lo - 1e-3 <= q[i] <= hi + 1e-3):
                raise RuntimeError(f"IK solution violates limit joint{i+1}: {q[i]:.3f} not in [{lo},{hi}]")
        r = self.move_joints([q], seconds)
        pos2, quat2 = self.fk()
        print(f"  hand now at {np.round(pos2, 4)} (target {np.round(pos, 4)}) dpos={np.linalg.norm(pos2-pos):.4f}", flush=True)
        return q, pos2

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP.get("max_effort", 30.0))
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=120)
        res = rf.result().result
        f = self.fingers()
        print(f"  gripper -> {width}: reached={res.reached_goal} stalled={res.stalled} fingers={f[0]:.4f},{f[1]:.4f}", flush=True)
        return f

    def servo(self, lin, n=20, frame=None):
        """Stream n twist messages (m/s in panda_link0 frame)."""
        msg = TwistStamped()
        msg.header.frame_id = frame or "panda_link0"
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, lin)
        for _ in range(n):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.twist_pub.publish(msg)
            self.spin(0.05)


# Standard orientations (hand frame axes expressed in world)
def quat_from_axes(x, y, z):
    R = np.column_stack([np.asarray(x, float), np.asarray(y, float), np.asarray(z, float)])
    return R_to_quat(R)

# hand z = approach direction, hand y = finger axis
Q_DOWN = quat_from_axes([1, 0, 0], [0, -1, 0], [0, 0, -1])          # pointing down, fingers along y
Q_FWD_FY_NEG = quat_from_axes([0, 0, 1], [0, -1, 0], [1, 0, 0])     # pointing +x, fingers along y
Q_FWD_FY_POS = quat_from_axes([0, 0, -1], [0, 1, 0], [1, 0, 0])     # pointing +x, fingers along y (flipped)
OPENRUA_EOF

# openrua op 19
timeout 120 python3 - <<'EOF'
import numpy as np
from rob import *
r = Robot()
print("arm", r.arm())
pos, q = r.fk(); print("FK hand world", pos, q)
# test IK for horizontal approach poses at pot A
for name, Q in [("FY_NEG", Q_FWD_FY_NEG), ("FY_POS", Q_FWD_FY_POS)]:
    print(name, Q)
    for target in [np.array([-0.196-0.10-0.16, -0.195, 0.965]), np.array([-0.196-0.10, -0.195, 0.965]), np.array([0.132-0.10-0.16, 0.09, 1.03]), np.array([0.235-0.10, 0.09, 1.03])]:
        sol = r.ik(target, Q)
        print("  ", target, None if sol is None else np.round(sol, 3))
        if sol is not None:
            p, qq = r.fk(sol); print("     fk", np.round(p,4), np.round(qq,3))
EOF

# openrua op 20
grep -o '<link name="world"[^>]*>' robot.urdf; grep -o '<joint name="[^"]*" type="fixed"><origin[^>]*><parent link="world"/><child link="[^"]*"/>' robot.urdf; grep -o '<joint name="virtual_joint"[^/]*/>' robot.urdf; timeout 20 ros2 param get /move_group robot_description_planning.joint_limits 2>/dev/null | head -2; timeout 30 ros2 service call /query_planner_interface moveit_msgs/srv/QueryPlannerInterfaces 2>/dev/null | head -c 600

# openrua op 21
timeout 120 python3 - <<'EOF'
import numpy as np, rclpy
from rob import *
from moveit_msgs.srv import GetPositionFK
r = Robot()
req = GetPositionFK.Request(); req.fk_link_names=["panda_link0","panda_hand"]
req.robot_state.joint_state.name=list(JOINTS); req.robot_state.joint_state.position=list(map(float,r.arm()))
fut=r.fk_cli.call_async(req); rclpy.spin_until_future_complete(r.node,fut,timeout_sec=30); res=fut.result()
for ps in res.pose_stamped: print(ps.header.frame_id, ps.pose.position)
# IK round trip with raw FK pose of hand (no BASE add)
hand = res.pose_stamped[1].pose
pos = np.array([hand.position.x, hand.position.y, hand.position.z]); quat=np.array([hand.orientation.x,hand.orientation.y,hand.orientation.z,hand.orientation.w])
r.BASE = BASE
import rob
rob.BASE = np.zeros(3)
sol = r.ik(pos, quat); print("IK roundtrip raw:", None if sol is None else np.round(sol,3))
rob.BASE = BASE
sol = r.ik(pos, quat); print("IK roundtrip minus BASE:", None if sol is None else np.round(sol,3))
EOF

# openrua op 22
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rob.py", "old_string": "BASE = np.array([-0.66, 0.0, 0.912])  # world -> panda_link0 (from TF)", "new_string": "BASE = np.zeros(3)  # verified: /compute_fk & /compute_ik on this machine work in WORLD coords (link0 at (-0.66,0,0.912))", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 23
timeout 300 python3 - <<'EOF'
import numpy as np
from rob import *
r = Robot()
for name, Q in [("FY_NEG", Q_FWD_FY_NEG), ("FY_POS", Q_FWD_FY_POS)]:
    print(name, Q)
    for target in [np.array([-0.196-0.10-0.16, -0.195, 0.965]), np.array([-0.196-0.10, -0.195, 0.965]), np.array([-0.057-0.10-0.16, 0.257, 0.965]), np.array([-0.057-0.10, 0.257, 0.965]), np.array([0.132-0.10-0.16, 0.09, 1.03]), np.array([0.132-0.10, 0.09, 1.03]), np.array([0.235-0.10, 0.09, 1.03]), np.array([0.235-0.10, 0.09, 1.10])]:
        sol = r.ik(target, Q)
        print("  ", target, None if sol is None else np.round(sol, 3))
        if sol is not None:
            p, qq = r.fk(sol); print("     fk", np.round(p,4), np.round(qq,3))
            for l in ["panda_link6","panda_link7"]:
                pl,_ = r.fk(sol, l); print("     ", l, np.round(pl,3))
EOF

# openrua op 24
mkdir -p "$(dirname /workspace/kin.py)"
cat > /workspace/kin.py <<'OPENRUA_EOF'
"""Analytic Panda FK (modified DH) + numeric IK with posture regularization. World frame."""
import math
import numpy as np
from scipy.optimize import least_squares

BASE_T = np.eye(4); BASE_T[:3, 3] = [-0.66, 0.0, 0.912]   # world -> panda_link0
DH = [  # (a, d, alpha) modified DH, Craig convention, per joint i
    (0.0, 0.333, 0.0),
    (0.0, 0.0, -math.pi / 2),
    (0.0, 0.316, math.pi / 2),
    (0.0825, 0.0, math.pi / 2),
    (-0.0825, 0.384, -math.pi / 2),
    (0.0, 0.0, math.pi / 2),
    (0.088, 0.0, math.pi / 2),
]
FLANGE = (0.0, 0.107, 0.0)
HAND_ROT = -math.pi / 4  # panda_link8 -> panda_hand about z
LIMITS = np.array([[-2.9, 2.9], [-1.76, 1.76], [-2.9, 2.9], [-3.07, -0.07], [-2.9, 2.9], [-0.02, 3.75], [-2.9, 2.9]])


def _tf(a, d, alpha, theta):
    ca, sa, ct, st = math.cos(alpha), math.sin(alpha), math.cos(theta), math.sin(theta)
    return np.array([
        [ct, -st, 0, a],
        [st * ca, ct * ca, -sa, -sa * d],
        [st * sa, ct * sa, ca, ca * d],
        [0, 0, 0, 1]])


def fk_all(q):
    """Return list of 4x4 world transforms: link1..link7, link8(flange), hand."""
    T = BASE_T.copy()
    out = []
    for (a, d, al), th in zip(DH, q):
        T = T @ _tf(a, d, al, th)
        out.append(T)
    T = T @ _tf(*FLANGE, 0.0)
    out.append(T)
    T = T @ _tf(0, 0, 0, HAND_ROT)
    out.append(T)
    return out


def fk_hand(q):
    return fk_all(q)[-1]


def quat_to_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def ik(pos, quat, seed, w_post=0.02, tol=1e-4, tries=8):
    """Numeric IK: hand frame at pos (world) with orientation quat. Returns q or None."""
    pos = np.asarray(pos, float); Rt = quat_to_R(quat)
    seed = np.asarray(seed, float)

    def resid(q):
        T = fk_hand(q)
        ep = (T[:3, 3] - pos) * 10.0
        Re = Rt.T @ T[:3, :3]
        eo = np.array([Re[2, 1] - Re[1, 2], Re[0, 2] - Re[2, 0], Re[1, 0] - Re[0, 1]]) * 0.5 * 3.0
        return np.concatenate([ep, eo, w_post * (q - seed)])

    best = None
    rng = np.random.default_rng(0)
    for k in range(tries):
        q0 = seed if k == 0 else np.clip(seed + rng.normal(0, 0.3 * k / tries + 0.1, 7), LIMITS[:, 0], LIMITS[:, 1])
        r = least_squares(resid, q0, bounds=(LIMITS[:, 0], LIMITS[:, 1]), xtol=1e-10, ftol=1e-10, max_nfev=2000)
        T = fk_hand(r.x)
        perr = np.linalg.norm(T[:3, 3] - pos)
        Re = Rt.T @ T[:3, :3]
        oerr = math.acos(max(-1, min(1, (np.trace(Re) - 1) / 2)))
        if perr < tol and oerr < 2e-3:
            return r.x
        if best is None or r.cost < best[0]:
            best = (r.cost, r.x, perr, oerr)
    print(f"  ik: no exact solution (best perr={best[2]:.4f} oerr={best[3]:.4f})")
    return None


def min_link_z(q):
    return min(T[2, 3] for T in fk_all(q))
OPENRUA_EOF

# openrua op 25
timeout 120 python3 - <<'EOF'
import numpy as np, time
from kin import *
from rob import Robot, Q_FWD_FY_NEG, Q_FWD_FY_POS, Q_DOWN
home = np.array([0, -0.161037389, 0, -2.44459747, 0, 2.2267522, 0.7853981633974483])
T = fk_hand(home); print("my FK hand:", np.round(T[:3,3],4)); 
r = Robot(); p,q = r.fk(home); print("moveit FK:", np.round(p,4), np.round(q,4))
# compare rotation
from rob import R_to_quat
print("my quat:", np.round(R_to_quat(T[:3,:3]),4))
# check all links vs moveit
for i,l in enumerate(["panda_link1","panda_link2","panda_link3","panda_link4","panda_link5","panda_link6","panda_link7","panda_link8","panda_hand"]):
    pl,_ = r.fk(home,l); print(l, np.round(pl,4), np.round(fk_all(home)[i][:3,3],4))
t=time.time()
for Q,name in [(Q_FWD_FY_NEG,"NEG"),(Q_FWD_FY_POS,"POS")]:
    for target in [[-0.456,-0.195,0.965],[-0.296,-0.195,0.965],[-0.157,0.257,0.965],[0.032,0.09,1.03],[0.135,0.09,1.03],[0.135,-0.02,1.03]]:
        s = ik(target, Q, home)
        print(name, target, None if s is None else np.round(s,3), "minz=%.3f"%min_link_z(s) if s is not None else "")
print("time", time.time()-t)
EOF

# openrua op 26
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/kin.py", "old_string": "        Re = Rt.T @ T[:3, :3]\\n        eo = np.array([Re[2, 1] - Re[1, 2], Re[0, 2] - Re[2, 0], Re[1, 0] - Re[0, 1]]) * 0.5 * 3.0\\n        return np.concatenate([ep, eo, w_post * (q - seed)])", "new_string": "        eo = (T[:3, :3] - Rt).ravel() * 2.0\\n        return np.concatenate([ep, eo, w_post * (q - seed)])", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 27
timeout 120 python3 - <<'EOF'
import numpy as np, time
from kin import *
from rob import Q_FWD_FY_NEG, Q_FWD_FY_POS, Q_DOWN
home = np.array([0, -0.161037389, 0, -2.44459747, 0, 2.2267522, 0.7853981633974483])
t=time.time()
for Q,name in [(Q_FWD_FY_NEG,"NEG"),(Q_FWD_FY_POS,"POS")]:
    for target in [[-0.456,-0.195,0.965],[-0.296,-0.195,0.965],[-0.157,0.257,0.965],[0.032,0.09,1.03],[0.135,0.09,1.03],[0.135,-0.02,1.03]]:
        s = ik(target, Q, home)
        if s is not None:
            zs = [round(T[2,3],3) for T in fk_all(s)]
            print(name, target, np.round(s,3), "link z:", zs[3:8])
        else: print(name, target, None)
print("time", time.time()-t)
EOF

# openrua op 28
timeout 120 python3 - <<'EOF'
import numpy as np
from kin import *
res=[]
for q2 in np.arange(-1.7,1.71,0.05):
  for q4 in np.arange(-3.0,-0.1,0.05):
    for q6 in np.arange(0,3.75,0.05):
        q=np.array([0,q2,0,q4,0,q6,0.785])
        T=fk_hand(q); z=T[:3,2]
        if abs(z[0]-1)<0.02:  # hand z points +x
            p=T[:3,3]
            res.append((p[0]+0.66,p[2],q2,q4,q6, min(t[2,3] for t in fk_all(q)), fk_all(q)[3][2,3]))
res=np.array(res)
print(len(res))
# bin by reach x and height
for xr in np.arange(0.3,0.9,0.1):
    sel=res[(res[:,0]>=xr)&(res[:,0]<xr+0.1)]
    if len(sel)==0: continue
    print(f"reach {xr:.1f}-{xr+0.1:.1f}: z range {sel[:,1].min():.3f}-{sel[:,1].max():.3f} n={len(sel)}")
    low=sel[np.abs(sel[:,1]-1.0)<0.05]
    for r in low[:6]: print("   ", np.round(r,3))
EOF

# openrua op 29
timeout 90 python3 scene.py frontview 0.01 > /dev/null; python3 - <<'EOF'
import numpy as np
for cam in ['sideview','frontview']:
    P=np.load(f'{cam}_world.npy')
    m=np.isfinite(P[...,2])&(np.abs(P[...,0]+0.20)<0.15)&(np.abs(P[...,1])<0.15)&(P[...,2]>1.15)&(P[...,2]<1.5)
    pts=P[m]
    print(cam, len(pts))
    for z0 in np.arange(1.15,1.5,0.01):
        s=pts[(pts[:,2]>=z0)&(pts[:,2]<z0+0.01)]
        if len(s)<3: continue
        print(f"  z {z0:.2f}: n={len(s)} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]")
EOF

# openrua op 30
timeout 120 python3 - <<'EOF'
import numpy as np
from kin import *
import kin
kin.LIMITS = np.array([[-2.85,2.85],[-1.72,1.72],[-2.85,2.85],[-3.0,-0.1],[-2.85,2.85],[0.0,3.7],[-2.85,2.85]])
from rob import Q_FWD_FY_NEG, Q_FWD_FY_POS
home = np.array([0, -0.161037389, 0, -2.44459747, 0, 2.2267522, 0.7853981633974483])
potA = np.array([-0.196,-0.195]); 
def hand_from_tcp(tcp): return np.array([tcp[0]-0.1034, tcp[1], tcp[2]])
wp1 = hand_from_tcp([potA[0]-0.01-0.07, potA[1], 1.15])
wp2 = hand_from_tcp([potA[0]-0.01-0.07, potA[1], 0.965])
wp3 = hand_from_tcp([potA[0]-0.01, potA[1], 0.965])
for Q,name in [(Q_FWD_FY_NEG,"NEG"),(Q_FWD_FY_POS,"POS")]:
    print("==",name)
    q1 = ik(wp1, Q, home, w_post=0.05)
    if q1 is None: continue
    q2 = ik(wp2, Q, q1, w_post=0.05); q3 = ik(wp3, Q, q2, w_post=0.05)
    for nm,q in [("q1",q1),("q2",q2),("q3",q3)]:
        Ts = fk_all(q)
        print(nm, np.round(q,3), "hand", np.round(Ts[-1][:3,3],3), "link z:", [round(T[2,3],3) for T in Ts[2:8]], "link x:", [round(T[0,3],3) for T in Ts[2:8]])
    # sweep home->q1
    print(" sweep home->q1 min hand z / fingertip:")
    for s in np.linspace(0,1,11):
        q = home*(1-s)+q1*s; T = fk_hand(q); tip = T[:3,3]+T[:3,2]*0.1134
        print(f"   s={s:.1f} hand={np.round(T[:3,3],3)} tip={np.round(tip,3)} z_dir={np.round(T[:3,2],2)}")
EOF

# openrua op 31
timeout 300 python3 - <<'EOF'
import numpy as np
from kin import *
import kin
kin.LIMITS = np.array([[-2.85,2.85],[-1.72,1.72],[-2.85,2.85],[-3.0,-0.1],[-2.85,2.85],[0.0,3.7],[-2.85,2.85]])
from rob import Q_FWD_FY_NEG, Q_FWD_FY_POS, Robot
home = np.array([0, -0.161037389, 0, -2.44459747, 0, 2.2267522, 0.7853981633974483])
r = Robot()
for Q,name in [(Q_FWD_FY_NEG,"NEG"),(Q_FWD_FY_POS,"POS")]:
    for z in [0.965, 1.15]:
        for x in np.arange(-0.45, 0.21, 0.05):
            hand = [x, -0.195, z]
            s = ik(hand, Q, home, w_post=0.02, tries=12)
            m = r.ik(hand, Q)
            print(name, f"z={z} x={x:.2f}", "mine:", None if s is None else np.round(s,2), "| moveit:", None if m is None else np.round(m,2))
EOF

# openrua op 32
mkdir -p "$(dirname /workspace/task.py)"
cat > /workspace/task.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Moka pot pick & place phases. Usage: python3 task.py <phase> [--dry]
Phases: approachA graspA liftA placeA  approachB graspB liftB placeB  home
All positions are TCP (fingertip pad centre) in world; hand origin = TCP - 0.1034*zhat.
"""
import math, sys, time
import numpy as np
import kin
from kin import fk_all, fk_hand, ik
from rob import Robot, R_to_quat, quat_to_R

kin.LIMITS = np.array([[-2.85, 2.85], [-1.72, 1.72], [-2.85, 2.85], [-3.0, -0.1], [-2.85, 2.85], [0.0, 3.7], [-2.85, 2.85]])
TCP_OFF = 0.1034
HOME = np.array([0, -0.161037389, 0, -2.44459747, 0, 2.2267522, 0.7853981633974483])

# scene (measured)
POTS = {"A": np.array([-0.196, -0.195]), "B": np.array([-0.057, 0.257])}
WAIST_Z = 0.965
SPOTS = {"A": (np.array([0.17, -0.008]), math.radians(45)),   # -y spot, yaw +45
         "B": (np.array([0.17, 0.072]), math.radians(-45))}   # +y spot, yaw -45
Z_CARRY = 1.10
Z_RELEASE = 1.01
GRASP_BACK = 0.01   # TCP sits 1 cm behind pot axis (palm clearance)
PRE = 0.07          # pre-grasp standoff along approach


def R_yaw(phi):
    c, s = math.cos(phi), math.sin(phi)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


R_POS = np.column_stack([[0, 0, -1], [0, 1, 0], [1, 0, 0]])  # hand x=-Z, y=+Y, z=+X


def hand_pose(tcp, phi):
    R = R_yaw(phi) @ R_POS
    zhat = R[:, 2]
    return np.asarray(tcp) - TCP_OFF * zhat, R_to_quat(R)


class Task:
    def __init__(self, dry):
        self.dry = dry
        self.r = None if dry else Robot()
        self.q = HOME.copy() if dry else self.r.arm()

    def cur(self):
        return self.q if self.dry else self.r.arm()

    def solve(self, tcp, phi, seed):
        pos, quat = hand_pose(tcp, phi)
        q = ik(pos, quat, seed, w_post=0.03, tries=12)
        if q is None:
            raise RuntimeError(f"IK failed for tcp={tcp} phi={math.degrees(phi):.0f}")
        return q

    def go(self, tcps, phis, seconds, label=""):
        """Cartesian waypoint list -> sequential IK -> one trajectory."""
        seed = self.cur()
        qs = []
        for tcp, phi in zip(tcps, phis):
            q = self.solve(tcp, phi, seed)
            qs.append(q); seed = q
        n = len(qs)
        times = [seconds * (i + 1) / n for i in range(n)]
        print(f"[{label}] {n} waypoints, {seconds}s", flush=True)
        for q, tcp in zip(qs, tcps):
            Ts = fk_all(q)
            tip = Ts[-1][:3, 3] + Ts[-1][:3, 2] * TCP_OFF
            print(f"   q={np.round(q, 3)} tcp={np.round(tip, 3)} minlinkz={min(T[2, 3] for T in Ts[2:]):.3f}", flush=True)
        # sweep check between consecutive configs
        prev = self.cur()
        for q in qs:
            for s in np.linspace(0, 1, 8)[1:]:
                qq = prev * (1 - s) + q * s
                Ts = fk_all(qq); tip = Ts[-1][:3, 3] + Ts[-1][:3, 2] * TCP_OFF
                if tip[2] < 0.93 or Ts[-1][2, 3] < 0.95:
                    print(f"   !! sweep low: tip z={tip[2]:.3f} hand z={Ts[-1][2,3]:.3f} at s={s:.2f}", flush=True)
            prev = q
        if self.dry:
            self.q = qs[-1]
            return
        code, err = self.r.move_joints(qs, times)
        if code != 0 or err > 0.02:
            print(f"   retrying final waypoint (code={code}, err={err:.4f})", flush=True)
            code, err = self.r.move_joints([qs[-1]], 2.0)
        pos, quat = self.r.fk()
        R = quat_to_R(quat); tip = pos + R[:, 2] * TCP_OFF
        print(f"   TCP now {np.round(tip, 4)} (target {np.round(tcps[-1], 4)}) err={np.linalg.norm(tip - tcps[-1]):.4f}", flush=True)
        return tip

    def gripper(self, w):
        if self.dry:
            print(f"[gripper {w}]"); return
        return self.r.gripper(w)

    # ---- phases
    def approach(self, P):
        p = POTS[P]
        far = np.array([p[0] - GRASP_BACK - PRE, p[1]])
        self.gripper(0.04)
        self.go([[far[0], far[1], 1.15]], [0.0], 4.0, f"approach{P}: high pre-grasp")
        self.go([[far[0], far[1], WAIST_Z]], [0.0], 3.0, f"approach{P}: descend")

    def grasp(self, P):
        p = POTS[P]
        self.go([[p[0] - GRASP_BACK, p[1], WAIST_Z]], [0.0], 2.0, f"grasp{P}: advance")
        f = self.gripper(0.0)
        return f

    def lift(self, P):
        p = POTS[P]
        self.go([[p[0] - GRASP_BACK, p[1], Z_CARRY]], [0.0], 3.0, f"lift{P}")

    def place(self, P):
        p = POTS[P]
        s, phi = SPOTS[P]
        start = np.array([p[0] - GRASP_BACK, p[1], Z_CARRY])
        end = np.array([s[0] - GRASP_BACK * math.cos(phi), s[1] - GRASP_BACK * math.sin(phi), Z_CARRY])
        tcps, phis = [], []
        for t in np.linspace(0, 1, 5)[1:]:
            tcps.append(start * (1 - t) + end * t); phis.append(phi * t)
        self.go(tcps, phis, 6.0, f"place{P}: transport")
        self.go([[end[0], end[1], Z_RELEASE]], [phi], 3.0, f"place{P}: lower")
        self.gripper(0.04)
        back = end - 0.06 * np.array([math.cos(phi), math.sin(phi), 0])
        self.go([[back[0], back[1], Z_RELEASE]], [phi], 2.0, f"place{P}: retreat")
        self.go([[back[0], back[1], 1.15]], [phi], 2.0, f"place{P}: rise")

    def home(self):
        if self.dry:
            return
        self.r.move_joints([HOME], 4.0)


if __name__ == "__main__":
    dry = "--dry" in sys.argv
    phases = [a for a in sys.argv[1:] if not a.startswith("--")]
    t = Task(dry)
    for ph in phases:
        name, P = ph[:-1], ph[-1]
        if ph == "home":
            t.home()
        else:
            getattr(t, name)(P)
    if not dry:
        print("fingers:", t.r.fingers())
OPENRUA_EOF

# openrua op 33
timeout 300 python3 task.py approachA graspA liftA placeA approachB graspB liftB placeB --dry 2>&1

# openrua op 34
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/kin.py", "old_string": "        r = least_squares(resid, q0, bounds=(LIMITS[:, 0], LIMITS[:, 1]), xtol=1e-10, ftol=1e-10, max_nfev=2000)\\n        T = fk_hand(r.x)", "new_string": "        r = least_squares(resid, q0, bounds=(LIMITS[:, 0], LIMITS[:, 1]), xtol=1e-10, ftol=1e-10, max_nfev=2000)\\n        # polish: drop the posture term so the pose is met exactly\\n        wp, w_post = w_post, 1e-4\\n        r = least_squares(resid, r.x, bounds=(LIMITS[:, 0], LIMITS[:, 1]), xtol=1e-12, ftol=1e-12, max_nfev=2000)\\n        w_post = wp\\n        T = fk_hand(r.x)", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 35
timeout 300 python3 task.py approachA graspA liftA placeA approachB graspB liftB placeB --dry 2>&1 | grep -v "^   q=" 

# openrua op 36
timeout 300 python3 - <<'EOF'
import numpy as np, math
import kin
from kin import *
from task import hand_pose, HOME, R_POS
kin.LIMITS = np.array([[-2.85, 2.85], [-1.72, 1.72], [-2.85, 2.85], [-3.0, -0.1], [-2.85, 2.85], [0.0, 3.7], [-2.85, 2.85]])
from rob import Robot
r = Robot()
for z in [1.15, 1.10, 1.05, 0.965]:
    pos, quat = hand_pose([-0.137, 0.257, z], 0.0)
    s = ik(pos, quat, HOME, w_post=0.03, tries=12)
    m = r.ik(pos, quat)
    print(z, "mine", None if s is None else np.round(s,3), "moveit", None if m is None else np.round(m,3))
# also try yaw variants at 1.15
for phi in [-20, -10, 10, 20]:
    pos, quat = hand_pose([-0.137, 0.257, 1.15], math.radians(phi))
    s = ik(pos, quat, HOME, w_post=0.03, tries=12)
    print("phi", phi, None if s is None else np.round(s,3))
EOF

# openrua op 37
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/task.py", "old_string": "        self.gripper(0.04)\\n        self.go([[far[0], far[1], 1.15]], [0.0], 4.0, f\\"approach{P}: high pre-grasp\\")", "new_string": "        self.gripper(0.04)\\n        if np.abs(self.cur() - HOME).max() > 0.05:\\n            print(f\\"[approach{P}: via HOME]\\", flush=True)\\n            if self.dry:\\n                self.q = HOME.copy()\\n            else:\\n                self.r.move_joints([HOME], 4.0)\\n        self.go([[far[0], far[1], 1.15]], [0.0], 4.0, f\\"approach{P}: high pre-grasp\\")", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 38
timeout 300 python3 task.py approachA graspA liftA placeA approachB graspB liftB placeB --dry 2>&1 | grep -v "^   q=" ; timeout 300 python3 task.py approachB graspB liftB placeB --dry 2>&1 | grep "^   q="

# openrua op 39
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/task.py", "old_string": "                if tip[2] < 0.93 or Ts[-1][2, 3] < 0.95:\\n                    print(f\\"   !! sweep low: tip z={tip[2]:.3f} hand z={Ts[-1][2,3]:.3f} at s={s:.2f}\\", flush=True)", "new_string": "                if tip[2] < 0.93 or Ts[-1][2, 3] < 0.95:\\n                    print(f\\"   !! sweep low: tip z={tip[2]:.3f} hand z={Ts[-1][2,3]:.3f} at s={s:.2f}\\", flush=True)\\n                for nm, pt in self.obstacles.items():\\n                    for pname, p in ((\\"tip\\", tip), (\\"hand\\", Ts[-1][:3, 3])):\\n                        if np.linalg.norm(p[:2] - pt[:2]) < 0.07 and p[2] < pt[2] + 0.01:\\n                            print(f\\"   !! sweep near {nm} ({pname} at {np.round(p,3)}) s={s:.2f}\\", flush=True)", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 40
python3 - <<'EOF'
import re
s = open('task.py').read()
s = s.replace("""        self.q = HOME.copy() if dry else self.r.arm()
""", """        self.q = HOME.copy() if dry else self.r.arm()
        # xy + top z of things the hand must not sweep through (pots on table, knob, pots on stove)
        self.obstacles = {"potA": np.array([*POTS["A"], 1.053]), "potB": np.array([*POTS["B"], 1.053]),
                          "knob": np.array([0.036, 0.03, 0.959])}
""")
s = s.replace("""    def lift(self, P):
        p = POTS[P]
""", """    def lift(self, P):
        p = POTS[P]
        self.obstacles.pop("pot" + P, None)   # it is in the hand now
""")
s = s.replace("""        self.gripper(0.04)
        back = end - 0.06""", """        self.gripper(0.04)
        self.obstacles["stove" + P] = np.array([*s, 1.09])
        back = end - 0.06""")
open('task.py','w').write(s)
EOF
timeout 300 python3 task.py approachA graspA liftA placeA approachB graspB liftB placeB --dry 2>&1 | grep -v "^   q="

# openrua op 41
timeout 900 python3 -u task.py approachA 2>&1 | tee approachA.log | grep -v "^   q="

# openrua op 42
timeout 120 python3 - <<'EOF'
import numpy as np
from rob import Robot
from kin import fk_all
r = Robot(); q = r.arm(); print("q", np.round(q,3)); 
Ts = fk_all(q); T=Ts[-1]; print("hand", np.round(T[:3,3],4), "zhat", np.round(T[:3,2],3), "yhat", np.round(T[:3,1],3)); print("tcp", np.round(T[:3,3]+T[:3,2]*0.1034,4))
EOF
timeout 60 python3 tools/perception/cam_snap.py frontview; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 43
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/task.py", "old_string": "        code, err = self.r.move_joints(qs, times)\\n        if code != 0 or err > 0.02:\\n            print(f\\"   retrying final waypoint (code={code}, err={err:.4f})\\", flush=True)\\n            code, err = self.r.move_joints([qs[-1]], 2.0)", "new_string": "        # pace by the largest joint excursion (controller lags fast goals)\\n        dq = max(np.abs(np.diff(np.vstack([self.cur()] + qs), axis=0)).max(axis=1).sum(), 1e-3)\\n        scale = max(1.0, 2.0 * dq / seconds)\\n        times = [t * scale for t in times]\\n        code, err = self.r.move_joints(qs, times)\\n        for k in range(5):\\n            if code == 0 and err < 0.01:\\n                break\\n            print(f\\"   retrying final waypoint (code={code}, err={err:.4f})\\", flush=True)\\n            code, err = self.r.move_joints([qs[-1]], max(2.0, 2.0 * err))", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 44
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/task.py", "old_string": "    for ph in phases:\\n        name, P = ph[:-1], ph[-1]\\n        if ph == \\"home\\":\\n            t.home()\\n        else:\\n            getattr(t, name)(P)", "new_string": "    if phases and phases[0] == \\"goto\\":   # goto x y z phi_deg seconds\\n        x, y, z, phi, sec = map(float, phases[1:6])\\n        t.go([[x, y, z]], [math.radians(phi)], sec, \\"goto\\")\\n        phases = []\\n    if phases and phases[0] == \\"grip\\":\\n        t.gripper(float(phases[1])); phases = []\\n    for ph in phases:\\n        name, P = ph[:-1], ph[-1]\\n        if ph == \\"home\\":\\n            t.home()\\n        else:\\n            getattr(t, name)(P)", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 45
timeout 900 python3 -u task.py goto -0.276 -0.195 0.965 0 3 2>&1 | grep -v "^   q="

# openrua op 46
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand; timeout 60 python3 tools/perception/cam_snap.py sideview

# openrua op 47
timeout 90 python3 scene.py robot0_eye_in_hand 0.01 > /dev/null; python3 - <<'EOF'
import numpy as np
P=np.load('robot0_eye_in_hand_world.npy')
m=np.isfinite(P[...,2])&(np.abs(P[...,0]+0.196)<0.08)&(np.abs(P[...,1]+0.195)<0.08)&(P[...,2]>0.9)&(P[...,2]<1.06)
pts=P[m]; print(len(pts))
for z0 in np.arange(0.90,1.06,0.01):
    s=pts[(pts[:,2]>=z0)&(pts[:,2]<z0+0.01)]
    if len(s)<3: continue
    print(f"z {z0:.2f}: n={len(s)} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}] yc={(s[:,1].min()+s[:,1].max())/2:.4f} xmin={s[:,0].min():.4f}")
EOF

# openrua op 48
timeout 900 python3 -u task.py goto -0.206 -0.194 0.965 0 2 2>&1 | grep -v "^   q=" && timeout 300 python3 -u task.py grip 0.0 && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand && timeout 60 python3 tools/perception/cam_snap.py frontview

# openrua op 49
timeout 900 python3 -u task.py liftA placeA 2>&1 | grep -v "^   q=" | tee liftplaceA.log

# openrua op 50
python3 tools/perception/cam_snap.py frontview frontview.png 2>&1 | tail -1; python3 tools/perception/cam_snap.py birdview birdview.png 2>&1 | tail -1

# openrua op 51
timeout 900 python3 -u task.py approachB 2>&1 | grep -v "^   q=" | tee approachB.log

# openrua op 52
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/task.py", "old_string": "    def approach(self, P):\\n        p = POTS[P]\\n        far = np.array([p[0] - GRASP_BACK - PRE, p[1]])\\n        self.gripper(0.04)\\n        if np.abs(self.cur() - HOME).max() > 0.05:\\n            print(f\\"[approach{P}: via HOME]\\", flush=True)\\n            if self.dry:\\n                self.q = HOME.copy()\\n            else:\\n                self.r.move_joints([HOME], 4.0)", "new_string": "    def home(self):\\n        if self.dry:\\n            self.q = HOME.copy(); return\\n        dq = np.abs(self.cur() - HOME).sum()\\n        code, err = self.r.move_joints([HOME], max(4.0, 2.0 * dq))\\n        for k in range(5):\\n            if code == 0 and err < 0.01:\\n                break\\n            print(f\\"   retrying HOME (code={code}, err={err:.4f})\\", flush=True)\\n            code, err = self.r.move_joints([HOME], max(2.0, 2.0 * err))\\n\\n    def approach(self, P):\\n        p = POTS[P]\\n        far = np.array([p[0] - GRASP_BACK - PRE, p[1]])\\n        self.gripper(0.04)\\n        if np.abs(self.cur() - HOME).max() > 0.05:\\n            print(f\\"[approach{P}: via HOME]\\", flush=True)\\n            self.home()", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 53
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/task.py", "old_string": "    def home(self):\\n        if self.dry:\\n            return\\n        self.r.move_joints([HOME], 4.0)\\n\\n\\nif", "new_string": "if", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 54
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/kin.py", "old_string": "        if perr < tol and oerr < 2e-3:", "new_string": "        if perr < tol and oerr < 6e-3:", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 55
timeout 900 python3 -u task.py approachB 2>&1 | grep -v "^   q=" | tee approachB.log

# openrua op 56
python3 scene.py robot0_eye_in_hand 0.02 2>&1 | tail -5; python3 - <<'EOF'
import numpy as np
P=np.load("robot0_eye_in_hand_world.npy")
z=P[...,2]; ok=np.isfinite(z)
# pot B boiler region: near expected (-0.057,0.257), z 0.905-0.95
m=ok&(z>0.905)&(z<0.95)&(np.abs(P[...,0]+0.057)<0.08)&(np.abs(P[...,1]-0.257)<0.08)
pts=P[m]
print("boiler pts",len(pts))
print("x range",pts[:,0].min(),pts[:,0].max(),"y range",pts[:,1].min(),pts[:,1].max())
# rear face at min x per y slice
for yc in np.arange(0.20,0.32,0.01):
    s=pts[np.abs(pts[:,1]-yc)<0.005]
    if len(s): print(f"y={yc:.2f} n={len(s)} xmin={s[:,0].min():.3f}")
# waist
m2=ok&(z>0.958)&(z<0.973)&(np.abs(P[...,0]+0.057)<0.08)&(np.abs(P[...,1]-0.257)<0.08)
w=P[m2]; print("waist y range",w[:,1].min(),w[:,1].max(),"xmin",w[:,0].min())
EOF

# openrua op 57
timeout 900 python3 -u task.py graspB 2>&1 | grep -v "^   q=" | tee graspB.log; python3 tools/perception/cam_snap.py robot0_eye_in_hand eih.png | tail -1

# openrua op 58
python3 scene.py robot0_eye_in_hand 0.02 2>&1 | head -1; python3 - <<'EOF'
import numpy as np
P=np.load("robot0_eye_in_hand_world.npy")
for (u,v) in [(120,240),(40,320),(60,300),(500,250),(320,300),(150,380),(480,380)]:
    print((u,v), np.round(P[v,u],3))
EOF

# openrua op 59
timeout 900 python3 -u task.py liftB placeB 2>&1 | grep -v "^   q=" | grep -v "sweep near stove" | tee liftplaceB.log

# openrua op 60
timeout 600 python3 -u task.py home 2>&1 | tail -3; for c in frontview birdview agentview sideview; do python3 tools/perception/cam_snap.py $c $c.png | tail -1; done

# openrua op 61
python3 scene.py birdview 0.03 2>&1 | head -1; python3 - <<'EOF'
import numpy as np, cv2
P=np.load("birdview_world.npy")
z=P[...,2]; ok=np.isfinite(z)
reg=ok&(P[...,0]>0.05)&(P[...,0]<0.32)&(P[...,1]>-0.12)&(P[...,1]<0.18)
for lo,hi in [(0.90,0.935),(0.935,0.96),(0.96,0.99),(0.99,1.02),(1.02,1.05),(1.05,1.09),(1.09,1.2)]:
    m=reg&(z>=lo)&(z<hi)
    pts=P[m]
    if len(pts): print(f"z[{lo},{hi}) n={len(pts)} x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}]")
# max height map on grid of 1cm
m=reg&(z>0.935)
pts=P[m]
gx=np.round((pts[:,0]-0.05)/0.01).astype(int); gy=np.round((pts[:,1]+0.12)/0.01).astype(int)
H=np.zeros((28,31)); 
for a,b,zz in zip(gx,gy,pts[:,2]):
    if 0<=a<28 and 0<=b<31: H[a,b]=max(H[a,b],zz)
np.set_printoptions(linewidth=250)
print("rows x from 0.05 step 1cm; cols y from -0.12 step 1cm; values = max z*100-90")
for a in range(28):
    print(f"x={0.05+a*0.01:.2f} "+" ".join(f"{int(round((v-0.9)*100)) if v>0 else 0:2d}" for v in H[a]))
EOF

# openrua op 62
python3 - <<'EOF'
import numpy as np
P=np.load("birdview_world.npy")
z=P[...,2]; ok=np.isfinite(z)
# plate free region x 0.20-0.27
for y0 in np.arange(-0.06,0.13,0.01):
    m=ok&(P[...,0]>0.20)&(P[...,0]<0.27)&(P[...,1]>=y0)&(P[...,1]<y0+0.01)
    if m.sum(): print(f"y={y0:.2f} n={m.sum()} zmin={z[m].min():.3f} zmed={np.median(z[m]):.3f} zmax={z[m].max():.3f}")
m=ok&(P[...,0]>0.30)&(P[...,0]<0.40)&(np.abs(P[...,1])<0.1)
print("table z", np.median(z[m]))
EOF

# openrua op 63
python3 - <<'EOF'
import numpy as np
P=np.load("birdview_world.npy")
z=P[...,2]; ok=np.isfinite(z)
reg=ok&(P[...,0]>0.04)&(P[...,0]<0.32)&(P[...,1]>-0.12)&(P[...,1]<0.20)&(z>0.94)
pts=P[reg]
# pot A: lid top (z>1.05) cluster near (0.15,-0.02)
a=pts[(pts[:,2]>1.04)&(np.linalg.norm(pts[:,:2]-[0.15,-0.02],axis=1)<0.06)]
print("potA lid pts",len(a),"xy mean",a[:,:2].mean(0),"x",a[:,0].min(),a[:,0].max(),"y",a[:,1].min(),a[:,1].max(),"zmax",a[:,2].max())
# all pot A pts above 0.99 (collector+lid, radius<=0.0385)
a2=pts[(pts[:,2]>0.99)&(np.linalg.norm(pts[:,:2]-[0.15,-0.02],axis=1)<0.06)]
print("potA upper x",a2[:,0].min(),a2[:,0].max(),"y",a2[:,1].min(),a2[:,1].max())
# pot B: exclude pot A and knob
b=pts[(np.linalg.norm(pts[:,:2]-[0.15,-0.02],axis=1)>0.055)&(np.linalg.norm(pts[:,:2]-[0.036,0.03],axis=1)>0.04)&(pts[:,1]>0.0)]
print("potB pts",len(b),"x",b[:,0].min(),b[:,0].max(),"y",b[:,1].min(),b[:,1].max(),"z",b[:,2].min(),b[:,2].max())
# PCA on xy
c=b[:,:2].mean(0); u,s,vt=np.linalg.svd(b[:,:2]-c,full_matrices=False)
d=vt[0]; print("center",c,"axis dir",d,"sv",s)
proj=(b[:,:2]-c)@d
print("extent along axis",proj.min(),proj.max())
# height profile along axis
for t in np.arange(proj.min(),proj.max(),0.01):
    s_=b[(proj>=t)&(proj<t+0.01)]
    perp=(s_[:,:2]-c)@np.array([-d[1],d[0]])
    print(f"t={t:+.3f} n={len(s_)} zmax={s_[:,2].max():.3f} perp[{perp.min():+.3f},{perp.max():+.3f}]")
EOF

# openrua op 64
python3 - <<'EOF'
import numpy as np, math, kin
from kin import ik, fk_hand
from rob import R_to_quat
kin.LIMITS = np.array([[-2.85,2.85],[-1.72,1.72],[-2.85,2.85],[-3.0,-0.1],[-2.85,2.85],[0.0,3.7],[-2.85,2.85]])
HOME=np.array([0,-0.161037389,0,-2.44459747,0,2.2267522,0.7853981633974483])
d0=np.array([-0.476,-0.879]); d0/=np.linalg.norm(d0)
def R_top(tilt):
    d3=np.array([d0[0]*math.cos(tilt), d0[1]*math.cos(tilt), math.sin(tilt)])
    x=-d3; y=np.array([-d0[1],d0[0],0.0]); z=np.cross(x,y)
    return np.column_stack([x,y,z])
for tilt in [0, 15, 32]:
    R=R_top(math.radians(tilt)); q=R_to_quat(R)
    for tcp in [(0.13,0.115,1.005),(0.13,0.115,1.10),(0.13,0.115,1.18)]:
        hand=np.array(tcp)-0.1034*R[:,2]
        sol=ik(hand,q,HOME,w_post=0.03,tries=12)
        print(f"tilt={tilt} tcp={tcp} hand={np.round(hand,3)} ->", None if sol is None else np.round(sol,2))
EOF

# openrua op 65
(pip list 2>/dev/null | grep -i -E "robosuite|mujoco|robocasa|mimicgen"; find / -iname "*moka*" -not -path "*/proc/*" 2>/dev/null | head -20; ls /workspace) 2>&1 | head -40

# openrua op 66
mkdir -p "$(dirname /workspace/recover.py)"
cat > /workspace/recover.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Recover a moka pot lying on its side: top-down grasp at the waist, lift, reorient to
vertical (compensating the measured in-hand tilt), place with a no-drag release.
Usage:
  python3 recover.py grasp   wx wy wz  dx dy        # waist axis point (world) + horizontal axis dir (bottom->lid)
  python3 recover.py lift
  python3 recover.py place   theta_deg sx sy phi_deg # theta: pot up-dir tilt in hand x-z plane (lid-up positive)
  python3 recover.py open | home
"""
import math, sys, time
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
import kin
from kin import fk_all, fk_hand, ik
from rob import R_to_quat, quat_to_R
import task
from task import Task, TCP_OFF, HOME, R_POS, R_yaw

Z_BOTTOM_TO_WAIST = 0.065     # pot bottom -> waist centre
PLATE_TOP = 0.930             # burner ring top
GRASP_UP = 0.01               # TCP sits 1 cm off the pot axis, away from the pot (palm side)


def Ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


class Recover(Task):
    def go_R(self, tcps, Rs, seconds, label="", retry=True):
        seed = self.cur(); qs = []
        for tcp, R in zip(tcps, Rs):
            pos = np.asarray(tcp) - TCP_OFF * R[:, 2]
            q = ik(pos, R_to_quat(R), seed, w_post=0.03, tries=12)
            if q is None:
                raise RuntimeError(f"IK failed for tcp={np.round(tcp,3)}")
            qs.append(q); seed = q
        n = len(qs)
        print(f"[{label}] {n} waypoints, {seconds}s", flush=True)
        for q, tcp in zip(qs, tcps):
            Ts = fk_all(q); tip = Ts[-1][:3, 3] + Ts[-1][:3, 2] * TCP_OFF
            print(f"   q={np.round(q, 3)} tcp={np.round(tip, 3)} minlinkz={min(T[2, 3] for T in Ts[2:]):.3f}", flush=True)
        if self.dry:
            self.q = qs[-1]; return
        dq = max(np.abs(np.diff(np.vstack([self.cur()] + qs), axis=0)).max(axis=1).sum(), 1e-3)
        scale = max(1.0, 2.0 * dq / seconds)
        times = [seconds * (i + 1) / n * scale for i in range(n)]
        code, err = self.r.move_joints(qs, times)
        for k in range(5 if retry else 0):
            if code == 0 and err < 0.01:
                break
            print(f"   retrying final waypoint (code={code}, err={err:.4f})", flush=True)
            code, err = self.r.move_joints([qs[-1]], max(2.0, 2.0 * err))
        pos, quat = self.r.fk(); R = quat_to_R(quat); tip = pos + R[:, 2] * TCP_OFF
        print(f"   TCP now {np.round(tip, 4)} (target {np.round(tcps[-1], 4)}) err={np.linalg.norm(tip - tcps[-1]):.4f}", flush=True)
        return tip

    def hand_R(self):
        if self.dry:
            return fk_hand(self.q)[:3, :3]
        pos, quat = self.r.fk(); return quat_to_R(quat)

    def grasp(self, waist, d0):
        d0 = np.asarray(d0, float); d0 /= np.linalg.norm(d0)
        x = np.array([-d0[0], -d0[1], 0.0]); z = np.array([0, 0, -1.0]); y = np.cross(z, x)
        R = np.column_stack([x, y, z])
        tcp = np.asarray(waist) + np.array([0, 0, GRASP_UP])
        self.gripper(0.04)
        self.go_R([tcp + [0, 0, 0.10]], [R], 4.0, "grasp: above")
        self.go_R([tcp], [R], 3.0, "grasp: descend")
        return self.gripper(0.0)

    def lift(self, z=1.10):
        R = self.hand_R()
        pos, quat = (fk_hand(self.q)[:3, 3], None) if self.dry else self.r.fk()
        tip = pos + R[:, 2] * TCP_OFF
        self.go_R([[tip[0], tip[1], z]], [R], 3.0, "lift")

    def place(self, theta, spot, phi):
        """theta: tilt of pot up-direction in the hand x-z plane; up = -cos(th) x - sin(th) z (hand frame)."""
        R0 = self.hand_R()
        pos = fk_hand(self.q)[:3, 3] if self.dry else self.r.fk()[0]
        tip0 = pos + R0[:, 2] * TCP_OFF
        Rf = R_yaw(phi) @ R_POS @ Ry(theta)
        # check: pot up in world
        up_hand = np.array([-math.cos(theta), 0, -math.sin(theta)])
        print("   final pot up (world):", np.round(Rf @ up_hand, 3), " final hand z:", np.round(Rf[:, 2], 3))
        waist_z = PLATE_TOP + Z_BOTTOM_TO_WAIST
        waist_f = np.array([spot[0], spot[1], waist_z])
        tcp_f = waist_f - GRASP_UP * Rf[:, 2]
        tcp_high = tcp_f.copy(); tcp_high[2] = tip0[2]
        # slerp orientation while translating at constant height
        sl = Slerp([0, 1], Rotation.from_matrix([R0, Rf]))
        ts = np.linspace(0, 1, 6)[1:]
        tcps = [tip0 * (1 - t) + tcp_high * t for t in ts]
        Rs = [sl(t).as_matrix() for t in ts]
        for t, R in zip(ts, Rs):
            up = R @ up_hand
            print(f"   t={t:.1f} pot up={np.round(up,2)} lowest pot pt z~{(tip0[2]-0.065*max(up[2],0)-0.09*max(-up[2],0)-0.04):.3f}")
        self.go_R(tcps, Rs, 8.0, "place: reorient")
        self.go_R([tcp_f + [0, 0, 0.005]], [Rf], 4.0, "place: lower", retry=False)
        self.gripper(0.04)
        if not self.dry:
            for _ in range(30):
                f = self.r.fingers()
                if f[0] > 0.039 and -f[1] > 0.039:
                    break
                time.sleep(0.2)
            print("   fingers after open:", np.round(self.r.fingers(), 4))
        back = np.array([math.cos(phi), math.sin(phi), 0.0])
        self.go_R([tcp_f + [0, 0, 0.005] - 0.06 * back], [Rf], 2.0, "place: retreat")
        self.go_R([tcp_f - 0.06 * back + [0, 0, 0.15]], [Rf], 2.0, "place: rise")


if __name__ == "__main__":
    dry = "--dry" in sys.argv
    a = [x for x in sys.argv[1:] if not x.startswith("--")]
    t = Recover(dry)
    if a[0] == "grasp":
        t.grasp([float(a[1]), float(a[2]), float(a[3])], [float(a[4]), float(a[5])])
    elif a[0] == "lift":
        t.lift(float(a[1]) if len(a) > 1 else 1.10)
    elif a[0] == "place":
        t.place(math.radians(float(a[1])), (float(a[2]), float(a[3])), math.radians(float(a[4])))
    elif a[0] == "open":
        t.gripper(0.04)
    elif a[0] == "home":
        t.home()
    if not dry:
        print("fingers:", t.r.fingers())
OPENRUA_EOF

# openrua op 67
python3 - <<'EOF' 2>&1 | grep -v "^   q="
import math, numpy as np
from recover import Recover
t=Recover(True)
t.grasp([0.130,0.115,0.995],[-0.476,-0.879])
t.lift(1.10)
for th in (32, 50, 10):
    t2=Recover(True); t2.q=t.q.copy()
    print("=== theta",th)
    t2.place(math.radians(th),(0.17,0.08),math.radians(-20))
EOF

# openrua op 68
timeout 900 python3 -u recover.py grasp 0.130 0.115 0.995 -0.476 -0.879 2>&1 | grep -v "^   q=" | tee recover_grasp.log; python3 tools/perception/cam_snap.py agentview agentview.png | tail -1

# openrua op 69
timeout 600 python3 -u recover.py open 2>&1 | tail -1; timeout 600 python3 -u recover.py lift 1.10 2>&1 | grep -v "^   q=" | tail -2; timeout 600 python3 -u recover.py home 2>&1 | tail -1; python3 scene.py birdview 0.03 2>&1 | head -1; python3 tools/perception/cam_snap.py agentview agentview.png | tail -1

# openrua op 70
cat > /workspace/fitpot.py <<'EOF'
"""Fit lying pot B from birdview cloud: axis dir, extent, height profile."""
import numpy as np, sys
P=np.load("birdview_world.npy")
z=P[...,2]; ok=np.isfinite(z)
reg=ok&(P[...,0]>0.03)&(P[...,0]<0.35)&(P[...,1]>-0.12)&(P[...,1]<0.25)&(z>0.94)
pts=P[reg]
a=pts[(pts[:,2]>0.99)&(np.linalg.norm(pts[:,:2]-[0.156,-0.023],axis=1)<0.06)]
print("potA upper x[%.3f,%.3f] y[%.3f,%.3f] zmax %.3f -> center est (%.3f,%.3f)"%(a[:,0].min(),a[:,0].max(),a[:,1].min(),a[:,1].max(),a[:,2].max(),a[:,0].max()-0.0385,a[:,1].max()-0.0385))
b=pts[(np.linalg.norm(pts[:,:2]-[0.156,-0.023],axis=1)>0.06)&(np.linalg.norm(pts[:,:2]-[0.036,0.03],axis=1)>0.04)&(pts[:,1]>0.0)]
print("potB pts",len(b),"x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]"%(b[:,0].min(),b[:,0].max(),b[:,1].min(),b[:,1].max(),b[:,2].min(),b[:,2].max()))
c=b[:,:2].mean(0); u,s,vt=np.linalg.svd(b[:,:2]-c,full_matrices=False)
d=vt[0]; proj=(b[:,:2]-c)@d
print("center",np.round(c,3),"axis dir",np.round(d,3),"extent",round(proj.min(),3),round(proj.max(),3))
for t in np.arange(proj.min(),proj.max(),0.01):
    s_=b[(proj>=t)&(proj<t+0.01)]
    if len(s_)==0: continue
    perp=(s_[:,:2]-c)@np.array([-d[1],d[0]])
    top=s_[np.argsort(s_[:,2])[-5:]]
    print(f"t={t:+.3f} n={len(s_):3d} zmax={s_[:,2].max():.3f} perp[{perp.min():+.3f},{perp.max():+.3f}] top-perp={np.mean((top[:,:2]-c)@np.array([-d[1],d[0]])):+.3f}")
EOF
python3 fitpot.py

# openrua op 71
python3 - <<'EOF'
import numpy as np, math, kin
from kin import ik
from rob import R_to_quat
kin.LIMITS = np.array([[-2.85,2.85],[-1.72,1.72],[-2.85,2.85],[-3.0,-0.1],[-2.85,2.85],[0.0,3.7],[-2.85,2.85]])
HOME=np.array([0,-0.161037389,0,-2.44459747,0,2.2267522,0.7853981633974483])
d0=np.array([0.244,0.97]); d0/=np.linalg.norm(d0)
x=np.array([-d0[0],-d0[1],0]); z=np.array([0,0,-1.0]); y=np.cross(z,x); R=np.column_stack([x,y,z]); q=R_to_quat(R)
for tcp in [(0.20,0.10,0.976),(0.20,0.10,1.08),(0.22,0.10,0.976),(0.20,0.10,1.12)]:
    hand=np.array(tcp)-0.1034*R[:,2]
    sol=ik(hand,q,HOME,w_post=0.03,tries=12)
    print(tcp, None if sol is None else np.round(sol,2))
EOF

# openrua op 72
python3 - <<'EOF'
import numpy as np, math, kin
from kin import ik
from rob import R_to_quat
from scipy.spatial.transform import Rotation
kin.LIMITS = np.array([[-2.85,2.85],[-1.72,1.72],[-2.85,2.85],[-3.0,-0.1],[-2.85,2.85],[0.0,3.7],[-2.85,2.85]])
HOME=np.array([0,-0.161037389,0,-2.44459747,0,2.2267522,0.7853981633974483])
d0=np.array([0.244,0.97]); d0/=np.linalg.norm(d0); d3=np.array([d0[0],d0[1],0])
x=-d3; z=np.array([0,0,-1.0]); y=np.cross(z,x); R0=np.column_stack([x,y,z])
for beta in [20,30,35,45]:
    Rb=Rotation.from_rotvec(math.radians(beta)*d3).as_matrix()@R0
    q=R_to_quat(Rb)
    for tcp in [(0.20,0.10,0.976),(0.20,0.10,1.06)]:
        hand=np.array(tcp)-0.1034*Rb[:,2]
        sol=ik(hand,q,HOME,w_post=0.03,tries=12)
        print(f"beta={beta} tcp={tcp} handz={np.round(Rb[:,2],2)} hand={np.round(hand,3)} ->", None if sol is None else np.round(sol,2))
EOF

# openrua op 73
python3 - <<'EOF'
import numpy as np, math, kin
from kin import ik
from rob import R_to_quat
from scipy.spatial.transform import Rotation
kin.LIMITS = np.array([[-2.85,2.85],[-1.72,1.72],[-2.85,2.85],[-3.0,-0.1],[-2.85,2.85],[0.0,3.7],[-2.85,2.85]])
HOME=np.array([0,-0.161037389,0,-2.44459747,0,2.2267522,0.7853981633974483])
d0=np.array([0.244,0.97]); d0/=np.linalg.norm(d0); d3=np.array([d0[0],d0[1],0])
x=-d3; z=np.array([0,0,-1.0]); y=np.cross(z,x); R0=np.column_stack([x,y,z])
for beta in [-20,-30,-35]:
    Rb=Rotation.from_rotvec(math.radians(beta)*d3).as_matrix()@R0
    q=R_to_quat(Rb)
    for tcp in [(0.20,0.10,0.976),(0.20,0.10,1.06)]:
        hand=np.array(tcp)-0.1034*Rb[:,2]
        sol=ik(hand,q,HOME,w_post=0.03,tries=12)
        print(f"beta={beta} tcp={tcp} handz={np.round(Rb[:,2],2)} hand={np.round(hand,3)} ->", None if sol is None else np.round(sol,2))
EOF

# openrua op 74
python3 - <<'EOF'
import re
s=open("recover.py").read()
# generalize grasp: beta tilt about pot axis, sign option for hand x
s=s.replace('''    def grasp(self, waist, d0):
        d0 = np.asarray(d0, float); d0 /= np.linalg.norm(d0)
        x = np.array([-d0[0], -d0[1], 0.0]); z = np.array([0, 0, -1.0]); y = np.cross(z, x)
        R = np.column_stack([x, y, z])
        tcp = np.asarray(waist) + np.array([0, 0, GRASP_UP])
        self.gripper(0.04)
        self.go_R([tcp + [0, 0, 0.10]], [R], 4.0, "grasp: above")
        self.go_R([tcp], [R], 3.0, "grasp: descend")
        return self.gripper(0.0)''',
'''    def grasp(self, waist, d0, beta=0.0, sign=-1):
        """Top-down grasp of a lying pot. d0: horizontal bottom->lid dir. beta: hand tilt about the
        pot axis (negative leans the hand toward the robot). sign=-1: hand x = -d0 ; +1: hand x = +d0."""
        d0 = np.asarray(d0, float); d0 /= np.linalg.norm(d0); d3 = np.array([d0[0], d0[1], 0.0])
        x = sign * d3; z = np.array([0, 0, -1.0]); y = np.cross(z, x)
        R = Rotation.from_rotvec(beta * d3).as_matrix() @ np.column_stack([x, y, z])
        print("   grasp hand z:", np.round(R[:, 2], 3), "hand y (fingers):", np.round(R[:, 1], 3))
        tcp = np.asarray(waist) - GRASP_UP * R[:, 2]
        self.gripper(0.04)
        self.go_R([tcp - 0.10 * R[:, 2]], [R], 4.0, "grasp: above")
        self.go_R([tcp], [R], 3.0, "grasp: descend", retry=False)
        return self.gripper(0.0)''')
s=s.replace('''        Rf = R_yaw(phi) @ R_POS @ Ry(theta)
''','''        Rf = R_yaw(phi) @ R_POS @ Ry(theta)
        if Rf[:, 2] @ np.array([math.cos(phi), math.sin(phi), 0]) < 0:   # keep approach azimuth = phi
            Rf = R_yaw(phi + math.pi) @ R_POS @ Ry(theta)
        if self.handle_world is not None:
            print("   handle dir now:", np.round(self.handle_world, 2), "-> final:", np.round(Rf @ R0.T @ self.handle_world, 2))
''')
s=s.replace('''class Recover(Task):
''','''class Recover(Task):
    handle_world = None
''')
s=s.replace('''    if a[0] == "grasp":
        t.grasp([float(a[1]), float(a[2]), float(a[3])], [float(a[4]), float(a[5])])''',
'''    if a[0] == "grasp":   # grasp wx wy wz dx dy [beta_deg] [sign]
        t.grasp([float(a[1]), float(a[2]), float(a[3])], [float(a[4]), float(a[5])],
                math.radians(float(a[6])) if len(a) > 6 else 0.0, int(a[7]) if len(a) > 7 else -1)''')
open("recover.py","w").write(s)
EOF
python3 - <<'EOF' 2>&1 | grep -v "^   q="
import math, numpy as np
from recover import Recover
waist=[0.2016,0.1026,0.966]; d0=[0.244,0.97]
perp=np.array([-0.97,0.244,0]); h=0.3*perp+0.95*np.array([0,0,1]); h/=np.linalg.norm(h)
for sign in (+1,-1):
    print("======== sign",sign)
    t=Recover(True); t.handle_world=h
    try:
        t.grasp(waist,d0,math.radians(-30),sign)
    except Exception as e:
        print("FAIL",e); continue
    t.lift(1.06)
    for phi in (-20,0,20,-45):
        for spot in ((0.17,0.08),):
            t2=Recover(True); t2.q=t.q.copy(); t2.handle_world=h
            print("=== phi",phi,"spot",spot)
            try:
                t2.place(math.pi if sign>0 else 0.0,spot,math.radians(phi))
            except Exception as e:
                print("FAIL",e)
EOF

# openrua op 75
python3 - <<'EOF'
s=open("recover.py").read()
s=s.replace('''        tcp_high = tcp_f.copy(); tcp_high[2] = tip0[2]
        # slerp orientation while translating at constant height
        sl = Slerp([0, 1], Rotation.from_matrix([R0, Rf]))
        ts = np.linspace(0, 1, 6)[1:]
        tcps = [tip0 * (1 - t) + tcp_high * t for t in ts]
        Rs = [sl(t).as_matrix() for t in ts]
        for t, R in zip(ts, Rs):
            up = R @ up_hand
            print(f"   t={t:.1f} pot up={np.round(up,2)} lowest pot pt z~{(tip0[2]-0.065*max(up[2],0)-0.09*max(-up[2],0)-0.04):.3f}")
        self.go_R(tcps, Rs, 8.0, "place: reorient")''',
'''        tcp_high = tcp_f.copy(); tcp_high[2] = tip0[2]
        stage = np.array([self.stage[0], self.stage[1], tip0[2]])
        # 1) translate closer to the robot (better dexterity), 2) slerp orientation there, 3) translate above spot
        sl = Slerp([0, 1], Rotation.from_matrix([R0, Rf]))
        ts = np.linspace(0, 1, 6)[1:]
        tcps = [tip0 * (1 - t) + stage * t for t in (0.5, 1.0)] + [stage] * 5 + [stage * (1 - t) + tcp_high * t for t in (0.5, 1.0)]
        Rs = [R0, R0] + [sl(t).as_matrix() for t in ts] + [Rf, Rf]
        for t, R in zip(ts, Rs[2:7]):
            up = R @ up_hand
            print(f"   t={t:.1f} pot up={np.round(up,2)} lowest pot pt z~{(tip0[2]-0.065*max(up[2],0)-0.09*max(-up[2],0)-0.04):.3f}")
        self.go_R(tcps, Rs, 12.0, "place: stage+reorient+transport")''')
s=s.replace('''    handle_world = None
''','''    handle_world = None
    stage = (0.09, 0.13)
''')
open("recover.py","w").write(s)
EOF
python3 - <<'EOF' 2>&1 | grep -v "^   q=" | grep -v "^   t="
import math, numpy as np
from recover import Recover
waist=[0.2016,0.1026,0.966]; d0=[0.244,0.97]
perp=np.array([-0.97,0.244,0]); h=0.3*perp+0.95*np.array([0,0,1]); h/=np.linalg.norm(h)
for sign in (-1,+1):
    print("======== sign",sign)
    t=Recover(True); t.handle_world=h
    t.grasp(waist,d0,math.radians(-30),sign)
    t.lift(1.06)
    for phi in (-20,0,20,-45):
        t2=Recover(True); t2.q=t.q.copy(); t2.handle_world=h
        print("=== phi",phi)
        try:
            t2.place(math.pi if sign>0 else 0.0,(0.17,0.08),math.radians(phi))
        except Exception as e:
            print("FAIL",e)
EOF

# openrua op 76
python3 - <<'EOF' 2>&1 | grep -v "^   q=" | grep -v "^   t="
import math, numpy as np, kin
from kin import ik, fk_hand
from rob import R_to_quat
from scipy.spatial.transform import Rotation, Slerp
from recover import Recover, Ry
from task import R_POS, R_yaw, TCP_OFF, HOME
waist=[0.2016,0.1026,0.966]; d0=np.array([0.244,0.97]); d0/=np.linalg.norm(d0); d3=np.array([d0[0],d0[1],0])
t=Recover(True); t.grasp(waist,d0,math.radians(-30),-1); t.lift(1.06)
R0=fk_hand(t.q)[:3,:3]
phi=math.radians(-20); Rf=R_yaw(phi)@R_POS
sl=Slerp([0,1],Rotation.from_matrix([R0,Rf]))
stage=np.array([0.09,0.13,1.06])
seed=t.q.copy()
for s in np.linspace(0,1,6):
    R=sl(s).as_matrix(); pos=stage-TCP_OFF*R[:,2]
    q=ik(pos,R_to_quat(R),seed,w_post=0.03,tries=12)
    if q is None:
        q2=ik(pos,R_to_quat(R),HOME,w_post=0.03,tries=20)
        print(f"s={s:.1f} FAIL from seed; from HOME:", None if q2 is None else np.round(q2,2))
        if q2 is not None: seed=q2
    else:
        print(f"s={s:.1f} ok q={np.round(q,2)}"); seed=q
EOF

# openrua op 77
python3 - <<'EOF' 2>&1 | grep -v "^   q=" | grep -v "^   t="
import math, numpy as np, kin
from kin import ik, fk_hand
from rob import R_to_quat
from scipy.spatial.transform import Rotation, Slerp
from recover import Recover, Ry
from task import R_POS, R_yaw, TCP_OFF, HOME
waist=[0.2016,0.1026,0.966]; d0=np.array([0.244,0.97]); d0/=np.linalg.norm(d0); d3=np.array([d0[0],d0[1],0])
t=Recover(True); t.grasp(waist,d0,math.radians(-30),+1); t.lift(1.06)
R0=fk_hand(t.q)[:3,:3]
for phideg in (-20,-45,-70):
    phi=math.radians(phideg); Rf=R_yaw(phi+math.pi)@R_POS@Ry(math.pi)
    print("phi",phideg,"final hand z",np.round(Rf[:,2],2),"hand x",np.round(Rf[:,0],2))
    sl=Slerp([0,1],Rotation.from_matrix([R0,Rf]))
    for stage in (np.array([0.09,0.13,1.06]), np.array([0.05,0.10,1.10])):
        seed=t.q.copy(); out=[]
        for s in np.linspace(0,1,6):
            R=sl(s).as_matrix(); pos=stage-TCP_OFF*R[:,2]
            q=ik(pos,R_to_quat(R),seed,w_post=0.03,tries=12)
            out.append(None if q is None else np.round(q,2))
            if q is not None: seed=q
        print("  stage",stage); [print("    ",o) for o in out]
EOF

# openrua op 78
python3 - <<'EOF' 2>&1 | grep -v "^   q=" | grep -v "^   t=" | grep -v "^\[gripper" | grep -v waypoints
import math, numpy as np
from recover import Recover
waist=[0.2016,0.1026,0.966]; d0=[0.244,0.97]
perp=np.array([-0.97,0.244,0]); h=0.3*perp+0.95*np.array([0,0,1]); h/=np.linalg.norm(h)
t=Recover(True); t.handle_world=h
t.grasp(waist,d0,math.radians(-30),+1)
t.lift(1.06)
for stage in ((0.09,0.13),(0.12,0.12),(0.05,0.10),(0.15,0.12)):
  for phi in (-45,-60,-35):
    for spot in ((0.15,0.08),(0.17,0.08)):
        t2=Recover(True); t2.q=t.q.copy(); t2.handle_world=h; t2.stage=stage
        try:
            t2.place(math.pi,spot,math.radians(phi)); print("OK   stage",stage,"phi",phi,"spot",spot)
        except Exception as e:
            print("FAIL stage",stage,"phi",phi,"spot",spot,e)
EOF

# openrua op 79
python3 - <<'EOF' 2>&1 | grep -v "^   t=" | grep -v "^\[gripper"
import math, numpy as np
from recover import Recover
waist=[0.2016,0.1026,0.966]; d0=[0.244,0.97]
t=Recover(True)
t.grasp(waist,d0,math.radians(-30),+1)
t.lift(1.06)
t.stage=(0.09,0.13)
t.place(math.pi,(0.15,0.08),math.radians(-60))
EOF

# openrua op 80
timeout 900 python3 -u recover.py grasp 0.2016 0.1026 0.968 0.244 0.97 -30 1 2>&1 | grep -v "^   q=" | tee recover_grasp2.log; python3 tools/perception/cam_snap.py agentview agentview.png | tail -1

# openrua op 81
timeout 600 python3 -u recover.py lift 1.06 2>&1 | grep -v "^   q=" | tail -3; python3 scene.py frontview 0.03 2>&1 | head -1; python3 tools/perception/cam_snap.py agentview agentview.png | tail -1; python3 - <<'EOF'
import numpy as np
P=np.load("frontview_world.npy")
z=P[...,2]; ok=np.isfinite(z)
# pot B held: region around (0.20,0.10), z 0.95-1.12, exclude pot A
m=ok&(np.abs(P[...,0]-0.19)<0.09)&(P[...,1]>0.0)&(P[...,1]<0.25)&(z>0.95)&(z<1.20)
pts=P[m]
print("pts",len(pts),"x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]"%(pts[:,0].min(),pts[:,0].max(),pts[:,1].min(),pts[:,1].max(),pts[:,2].min(),pts[:,2].max()))
# slices along y: z range and x range
for y0 in np.arange(0.0,0.25,0.01):
    s=pts[(pts[:,1]>=y0)&(pts[:,1]<y0+0.01)]
    if len(s)>5: print(f"y={y0:.2f} n={len(s):4d} z[{s[:,2].min():.3f},{s[:,2].max():.3f}] x[{s[:,0].min():.3f},{s[:,0].max():.3f}]")
EOF

# openrua op 82
sed -i 's/^Z_BOTTOM_TO_WAIST = 0.065 .*/Z_BOTTOM_TO_WAIST = 0.058     # pot bottom -> grasp point (measured after lift)/' recover.py && grep -n "Z_BOTTOM_TO_WAIST =" recover.py && timeout 1200 python3 -u recover.py place 180 0.15 0.08 -60 2>&1 | grep -v "^   q=" | tee recover_place.log

# openrua op 83
for c in agentview frontview; do python3 tools/perception/cam_snap.py $c $c.png | tail -1; done

# openrua op 84
timeout 600 python3 -u recover.py home 2>&1 | tail -5 && python3 scene.py birdview 0.03 2>&1 | grep -v table_z && python3 fitpot.py 2>&1 | head -30

# openrua op 85
python3 - <<'EOF'
import numpy as np
from scipy.cluster.vq import kmeans2
P=np.load("birdview_world.npy").reshape(-1,3)
P=P[np.isfinite(P).all(1)]
# stove region
m=(P[:,0]>0.0)&(P[:,0]<0.35)&(P[:,1]>-0.15)&(P[:,1]<0.2)&(P[:,2]>0.935)
Q=P[m]
print("pts above 0.935:",len(Q), "z range",Q[:,2].min(),Q[:,2].max())
# tops (lid knobs) => z>1.07
T=Q[Q[:,2]>1.075]
print("top pts",len(T)); 
if len(T):
    c,l=kmeans2(T[:,:2],2,seed=1,minit='++')
    for i in range(2):
        s=T[l==i]; print(f"top cluster {i}: n={len(s)} center={s[:,:2].mean(0).round(3)} zmax={s[:,2].max():.3f}")
# boiler level slices z in [0.94,0.96] -> footprint
for zlo,zhi in [(0.935,0.955),(0.955,0.975),(0.975,1.0),(1.0,1.04),(1.04,1.07)]:
    S=Q[(Q[:,2]>=zlo)&(Q[:,2]<zhi)]
    print(f"z[{zlo},{zhi}] n={len(S)} x[{S[:,0].min():.3f},{S[:,0].max():.3f}] y[{S[:,1].min():.3f},{S[:,1].max():.3f}]" if len(S) else f"z[{zlo},{zhi}] none")
EOF

# openrua op 86
python3 tools/perception/cam_snap.py birdview birdview.png >/dev/null 2>&1; python3 tools/perception/cam_snap.py frontview frontview.png >/dev/null 2>&1; python3 tools/perception/cam_snap.py sideview sideview.png >/dev/null 2>&1; echo ok

# openrua op 87
python3 -c "
import cv2
im=cv2.imread('birdview.png'); cv2.imwrite('bird_crop.png', cv2.resize(im[260:400,240:400],None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('frontview.png'); cv2.imwrite('front_crop.png', im)
"

# openrua op 88
python3 - <<'EOF'
import numpy as np
P=np.load("birdview_world.npy").reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,0]>-0.1)&(P[:,0]<0.35)&(P[:,1]>-0.2)&(P[:,1]<0.2)&(P[:,2]>0.905)
Q=P[m]
# upright pot: near top clusters
up=Q[np.linalg.norm(Q[:,:2]-[0.127,0.04],axis=1)<0.055]
print("upright pot: n",len(up),"z",up[:,2].min().round(3),up[:,2].max().round(3))
for zlo,zhi in [(0.93,0.96),(0.96,0.99),(0.99,1.03),(1.03,1.07),(1.07,1.1)]:
    S=up[(up[:,2]>=zlo)&(up[:,2]<zhi)]
    if len(S): print(f"  z[{zlo},{zhi}] n={len(S)} center={S[:,:2].mean(0).round(3)} x[{S[:,0].min():.3f},{S[:,0].max():.3f}] y[{S[:,1].min():.3f},{S[:,1].max():.3f}]")
# lying pot: exclude upright pot & knob
knob=np.linalg.norm(Q[:,:2]-[0.036,0.03],axis=1)<0.04
ly=Q[(np.linalg.norm(Q[:,:2]-[0.127,0.04],axis=1)>=0.06)&(~knob)]
print("lying pot: n",len(ly),"x",ly[:,0].min().round(3),ly[:,0].max().round(3),"y",ly[:,1].min().round(3),ly[:,1].max().round(3),"z",ly[:,2].min().round(3),ly[:,2].max().round(3))
c=ly[:,:2].mean(0); U,S,Vt=np.linalg.svd(ly[:,:2]-c); d=Vt[0]
t=(ly[:,:2]-c)@d
print("center",c.round(3),"axis",d.round(3),"t range",t.min().round(3),t.max().round(3))
perp=(ly[:,:2]-c)@np.array([-d[1],d[0]])
for tl in np.arange(t.min(),t.max(),0.01):
    s=ly[(t>=tl)&(t<tl+0.01)]
    if len(s)<3: continue
    pp=perp[(t>=tl)&(t<tl+0.01)]
    print(f"  t={tl:+.3f} n={len(s):3d} zmax={s[:,2].max():.3f} perp[{pp.min():+.3f},{pp.max():+.3f}] xy={s[:,:2].mean(0).round(3)}")
EOF

# openrua op 89
python3 scene.py frontview 0.03 >/dev/null 2>&1; python3 scene.py sideview 0.03 >/dev/null 2>&1; python3 - <<'EOF'
import numpy as np
def load(c):
    P=np.load(f"{c}_world.npy").reshape(-1,3); return P[np.isfinite(P).all(1)]
for cam in ["birdview","frontview","sideview"]:
    P=load(cam)
    # upright pot lid rim: z in [1.075,1.09], near (0.125,0.04)
    r=P[(P[:,2]>1.072)&(P[:,2]<1.095)&(np.linalg.norm(P[:,:2]-[0.125,0.04],axis=1)<0.07)]
    if len(r)>5:
        c=r[:,:2].mean(0); print(f"{cam} lidrim n={len(r)} x[{r[:,0].min():.3f},{r[:,0].max():.3f}] y[{r[:,1].min():.3f},{r[:,1].max():.3f}] mean={c.round(3)} zmax={r[:,2].max():.3f}")
    # boiler slice z 0.94-0.98 near the upright pot
    b=P[(P[:,2]>0.94)&(P[:,2]<0.985)&(np.linalg.norm(P[:,:2]-[0.125,0.04],axis=1)<0.05)]
    if len(b)>5: print(f"   boiler n={len(b)} x[{b[:,0].min():.3f},{b[:,0].max():.3f}] y[{b[:,1].min():.3f},{b[:,1].max():.3f}]")
# lying pot from birdview
P=load("birdview")
m=(P[:,0]>-0.06)&(P[:,0]<0.115)&(P[:,1]>-0.16)&(P[:,1]<0.0)&(P[:,2]>0.935)
ly=P[m]; ly=ly[np.linalg.norm(ly[:,:2]-[0.036,0.03],axis=1)>0.045]
c=ly[:,:2].mean(0); U,S,Vt=np.linalg.svd(ly[:,:2]-c); d=Vt[0]
if d[0]<0: d=-d
t=(ly[:,:2]-c)@d; perp=(ly[:,:2]-c)@np.array([-d[1],d[0]])
print("lying pot n",len(ly),"center",c.round(3),"axis",d.round(3),"t",t.min().round(3),t.max().round(3),"zmax",ly[:,2].max().round(3))
for tl in np.arange(t.min(),t.max(),0.01):
    s=(t>=tl)&(t<tl+0.01)
    if s.sum()<3: continue
    print(f"  t={tl:+.3f} n={s.sum():3d} zmax={ly[s,2].max():.3f} perp[{perp[s].min():+.3f},{perp[s].max():+.3f}] top-perp={perp[s][ly[s,2].argmax()]:+.3f} xy={ly[s,:2].mean(0).round(3)}")
EOF

# openrua op 90
mkdir -p "$(dirname /workspace/moveB.py)"
cat > /workspace/moveB.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Shift the upright pot B inward on the plate with a side grasp.
Usage: python3 moveB.py [--dry]  (phases: pick | place | all)"""
import math, sys
import numpy as np
from task import Task, HOME

C_B = np.array([0.128, 0.034])       # measured centre of upright pot B
S_B = np.array([0.16, 0.078])        # target centre
PHI = math.radians(-45)              # approach azimuth (hand z direction)
GZ = 0.998                           # grasp TCP z: ~6.8 cm above pot bottom (bottom on plate 0.930)
BACK = 0.01
DIR = np.array([math.cos(PHI), math.sin(PHI), 0.0])


class MoveB(Task):
    def __init__(self, dry):
        super().__init__(dry)
        self.obstacles = {"potA_lying": np.array([0.03, -0.06, 1.01]), "knob": np.array([0.036, 0.03, 0.959]),
                          "potB": np.array([*C_B, 1.09])}

    def open_full(self):
        for k in range(6):
            f = self.gripper(0.04)
            if self.dry or (f[0] > 0.039 and -f[1] > 0.039):
                break
            print("   fingers not fully open yet, re-issuing", flush=True)

    def pick(self):
        tcp = np.array([*(C_B - BACK * DIR[:2]), GZ])
        pre = tcp - 0.07 * DIR
        self.open_full()
        if np.abs(self.cur() - HOME).max() > 0.05:
            self.home()
        self.go([[pre[0], pre[1], 1.15]], [PHI], 4.0, "pickB: high pre-grasp")
        self.go([pre], [PHI], 3.0, "pickB: descend")
        self.go([tcp], [PHI], 2.0, "pickB: advance")
        f = self.gripper(0.0)
        self.obstacles.pop("potB")
        self.go([[tcp[0], tcp[1], 1.10]], [PHI], 3.0, "pickB: lift")
        return f

    def place(self):
        start = np.array([*(C_B - BACK * DIR[:2]), 1.10])
        end = np.array([*(S_B - BACK * DIR[:2]), 1.10])
        self.go([start * (1 - t) + end * t for t in (0.5, 1.0)], [PHI, PHI], 4.0, "placeB: transport")
        low = np.array([end[0], end[1], GZ + 0.005])
        self.go([low], [PHI], 3.0, "placeB: lower")
        self.open_full()
        back = low - 0.06 * DIR
        self.go([back], [PHI], 2.0, "placeB: retreat")
        self.go([[back[0], back[1], 1.15]], [PHI], 2.0, "placeB: rise")
        self.home()


if __name__ == "__main__":
    dry = "--dry" in sys.argv
    a = [x for x in sys.argv[1:] if not x.startswith("--")] or ["all"]
    t = MoveB(dry)
    if a[0] in ("pick", "all"):
        t.pick()
    if a[0] in ("place", "all"):
        t.place()
    if not dry:
        print("fingers:", t.r.fingers())
OPENRUA_EOF

# openrua op 91
python3 moveB.py --dry 2>&1 | grep -v "^   q="

# openrua op 92
timeout 900 python3 -u moveB.py pick 2>&1 | grep -v "^   q=" | tee pickB2.log

# openrua op 93
python3 tools/perception/cam_snap.py agentview agentview.png >/dev/null 2>&1; python3 tools/perception/cam_snap.py birdview birdview.png >/dev/null 2>&1; python3 -c "
import cv2
im=cv2.imread('birdview.png'); cv2.imwrite('bird_crop.png', cv2.resize(im[240:400,220:420],None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))"

# openrua op 94
timeout 600 python3 -u recover.py home 2>&1 | tail -2 && python3 scene.py birdview 0.03 >/dev/null 2>&1 && python3 scene.py frontview 0.03 >/dev/null 2>&1 && python3 - <<'EOF'
import numpy as np
def load(c):
    P=np.load(f"{c}_world.npy").reshape(-1,3); return P[np.isfinite(P).all(1)]
P=load("birdview")
m=(P[:,0]>-0.1)&(P[:,0]<0.35)&(P[:,1]>-0.2)&(P[:,1]<0.2)&(P[:,2]>0.936)
Q=P[m]; Q=Q[np.linalg.norm(Q[:,:2]-[0.036,0.03],axis=1)>0.045]
print("n",len(Q),"x",Q[:,0].min().round(3),Q[:,0].max().round(3),"y",Q[:,1].min().round(3),Q[:,1].max().round(3),"zmax",Q[:,2].max().round(3))
# split by a line: pot A at y<-0.03 & x<0.11 ; pot B elsewhere
A=Q[(Q[:,1]<-0.02)&(Q[:,0]<0.11)]
B=Q[~((Q[:,1]<-0.02)&(Q[:,0]<0.11))]
for name,S in (("A",A),("B",B)):
    c=S[:,:2].mean(0); U,s,Vt=np.linalg.svd(S[:,:2]-c); d=Vt[0]
    if d[0]<0: d=-d
    t=(S[:,:2]-c)@d; perp=(S[:,:2]-c)@np.array([-d[1],d[0]])
    print(f"pot {name}: n={len(S)} center={c.round(3)} axis={d.round(3)} t[{t.min():.3f},{t.max():.3f}] zmax={S[:,2].max():.3f}")
    for tl in np.arange(t.min(),t.max(),0.01):
        s=(t>=tl)&(t<tl+0.01)
        if s.sum()<3: continue
        print(f"  t={tl:+.3f} n={s.sum():3d} zmax={S[s,2].max():.3f} perp[{perp[s].min():+.3f},{perp[s].max():+.3f}] top-perp={perp[s][S[s,2].argmax()]:+.3f} xy={S[s,:2].mean(0).round(3)}")
EOF

# openrua op 95
python3 tools/perception/cam_snap.py frontview frontview.png >/dev/null 2>&1; python3 tools/perception/cam_snap.py birdview birdview.png >/dev/null 2>&1; python3 -c "
import cv2
im=cv2.imread('birdview.png'); cv2.imwrite('bird_crop.png', cv2.resize(im[260:400,240:420],None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))"

# openrua op 96
python3 - <<'EOF'
import numpy as np
P=np.load("birdview_world.npy").reshape(-1,3); P=P[np.isfinite(P).all(1)]
S=P[(P[:,0]>0.14)&(P[:,0]<0.35)&(P[:,1]>-0.15)&(P[:,1]<0.1)&(P[:,2]>0.94)]
c=S[:,:2].mean(0); U,s,Vt=np.linalg.svd(S[:,:2]-c); d=Vt[0]
if d[0]<0: d=-d
for it in range(3):
    t=(S[:,:2]-c)@d; perp=(S[:,:2]-c)@np.array([-d[1],d[0]])
    ridge=[]
    for tl in np.arange(t.min(),t.max(),0.005):
        m=(t>=tl)&(t<tl+0.005)
        if m.sum()<5: continue
        i=np.argmax(np.where(m,S[:,2],-1)); ridge.append(S[i])
    R=np.array(ridge); c=R[:,:2].mean(0); U,s,Vt=np.linalg.svd(R[:,:2]-c); d=Vt[0]
    if d[0]<0: d=-d
    print(f"iter{it}: ridge n={len(R)} center={c.round(3)} axis={d.round(3)}")
t=(S[:,:2]-c)@d; perp=(S[:,:2]-c)@np.array([-d[1],d[0]])
tr=(R[:,:2]-c)@d; pr=(R[:,:2]-c)@np.array([-d[1],d[0]])
for tl in np.arange(t.min(),t.max(),0.01):
    m=(t>=tl)&(t<tl+0.01); mr=(tr>=tl)&(tr<tl+0.01)
    if m.sum()<3: continue
    print(f"  t={tl:+.3f} n={m.sum():3d} zmax={S[m,2].max():.3f} perp[{perp[m].min():+.3f},{perp[m].max():+.3f}] ridge-perp={pr[mr].mean() if mr.sum() else float('nan'):+.3f} ridge-z={R[mr,2].mean() if mr.sum() else float('nan'):.3f} xy={S[m,:2].mean(0).round(3)}")
# knob-side anomaly
A=P[(P[:,0]>0.0)&(P[:,0]<0.12)&(P[:,1]>-0.03)&(P[:,1]<0.02)&(P[:,2]>0.94)]
print("anomaly pts",len(A), A[:,0].min().round(3),A[:,0].max().round(3),A[:,1].min().round(3),A[:,1].max().round(3),A[:,2].min().round(3),A[:,2].max().round(3))
EOF

# openrua op 97
python3 -c "
import cv2
im=cv2.imread('frontview.png'); cv2.imwrite('front_crop.png', cv2.resize(im[330:480,200:440],None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))"

# openrua op 98
python3 - <<'EOF' 2>&1 | grep -v "^  ik:"
import math, numpy as np
from scipy.spatial.transform import Rotation
import task, kin
from kin import ik, fk_all
from rob import R_to_quat
from task import HOME, TCP_OFF
c=np.array([0.231,-0.025]); d=np.array([0.494,-0.87,0.0])   # lid at -t, bottom at +t => bottom->lid = -d
d0=-d
def pose(t, beta, sign, up=0.01):
    p=np.array([*(c+t*d[:2]), 0.966])
    x=sign*d0; z=np.array([0,0,-1.0]); y=np.cross(z,x)
    R=Rotation.from_rotvec(beta*d0).as_matrix()@np.column_stack([x,y,z])
    tcp=p-up*R[:,2]
    return tcp,R
for t in (0.0, 0.02, 0.045):
    for sign in (1,-1):
        for bdeg in (0,-20,-30,-40,-50,-60,20,30,40,50,60):
            tcp,R=pose(t,math.radians(bdeg),sign)
            pos=tcp-TCP_OFF*R[:,2]
            q=ik(pos,R_to_quat(R),HOME,w_post=0.03,tries=6)
            ok = q is not None
            q2=None
            if ok:
                tcp2=tcp-0.08*R[:,2]; q2=ik(tcp2-TCP_OFF*R[:,2],R_to_quat(R),q,w_post=0.03,tries=6)
                Ts=fk_all(q); mz=min(T[2,3] for T in Ts[2:])
            print(f"t={t:+.3f} sign={sign:+d} beta={bdeg:+d}: grasp {'OK' if ok else '--'} above {'OK' if q2 is not None else '--'} origin={np.round(pos,3)} handz={np.round(R[:,2],2)}" + (f" minlinkz={mz:.3f}" if ok else ""))
EOF

# openrua op 99
python3 - <<'EOF' 2>&1 | grep -v "^  ik:"
import math, numpy as np
from scipy.spatial.transform import Rotation
import task, kin
from kin import ik, fk_all
from rob import R_to_quat
from task import HOME, TCP_OFF
c=np.array([0.231,-0.025]); d=np.array([0.494,-0.87,0.0]); d0=-d
def pose(t, beta, sign, up=0.01):
    p=np.array([*(c+t*d[:2]), 0.967])
    x=sign*d0; z=np.array([0,0,-1.0]); y=np.cross(z,x)
    R=Rotation.from_rotvec(beta*d0).as_matrix()@np.column_stack([x,y,z])
    return p-up*R[:,2],R
for t in (0.015,0.02):
    for bdeg in (-42,-45,-48,-50):
        tcp,R=pose(t,math.radians(bdeg),1)
        pos=tcp-TCP_OFF*R[:,2]
        q=ik(pos,R_to_quat(R),HOME,w_post=0.03,tries=8)
        if q is None: print(f"t={t} beta={bdeg}: grasp --"); continue
        qa=ik(pos-0.10*R[:,2],R_to_quat(R),q,w_post=0.03,tries=8)
        ql=ik(np.array([pos[0],pos[1],pos[2]+0.13]),R_to_quat(R),q,w_post=0.03,tries=8)
        # lowest finger point: pads at tcp +/- 0.03*y, finger tip +0.01 z, half thickness 0.006 y, half width 0.009 x
        pts=[tcp+a*R[:,1]+b*R[:,2]+cx*R[:,0] for a in (-0.036,0.036) for b in (0.0,0.01) for cx in (-0.009,0.009)]
        print(f"t={t} beta={bdeg}: grasp OK above {'OK' if qa is not None else '--'} lift {'OK' if ql is not None else '--'} lowest finger z={min(p[2] for p in pts):.3f} q={np.round(q,2)}")
EOF

# openrua op 100
python3 - <<'EOF'
import re
s=open("recover.py").read()
s=s.replace('''Z_BOTTOM_TO_WAIST = 0.058     # pot bottom -> grasp point (measured after lift)
PLATE_TOP = 0.930             # burner ring top
GRASP_UP = 0.01               # TCP sits 1 cm off the pot axis, away from the pot (palm side)
''','''Z_BOTTOM_TO_WAIST = 0.058     # pot bottom -> grasp point (measured after lift)
PLATE_TOP = 0.930             # burner ring top
GRASP_UP = 0.01               # TCP sits 1 cm off the pot axis, away from the pot (palm side)


def opt(name, default, conv=float):
    for a in sys.argv:
        if a.startswith(f"--{name}="):
            v = a.split("=", 1)[1]
            return [conv(x) for x in v.split(",")] if "," in v else conv(v)
    return default
''')
s=s.replace('''    def open_full(self):''','''''')
s=s.replace('''    def hand_R(self):''','''    def open_full(self):
        """The sim clock only advances while a command runs: re-issue open until fully open."""
        for k in range(6):
            f = self.gripper(0.04)
            if self.dry or (f[0] > 0.039 and -f[1] > 0.039):
                break
            print("   fingers not fully open yet, re-issuing", flush=True)

    def hand_R(self):''')
s=s.replace('''        self.go_R([tcp_f + [0, 0, 0.005]], [Rf], 4.0, "place: lower", retry=False)
        self.gripper(0.04)
        if not self.dry:
            for _ in range(30):
                f = self.r.fingers()
                if f[0] > 0.039 and -f[1] > 0.039:
                    break
                time.sleep(0.2)
            print("   fingers after open:", np.round(self.r.fingers(), 4))
        back = np.array([math.cos(phi), math.sin(phi), 0.0])
        self.go_R([tcp_f + [0, 0, 0.005] - 0.06 * back], [Rf], 2.0, "place: retreat")
        self.go_R([tcp_f - 0.06 * back + [0, 0, 0.15]], [Rf], 2.0, "place: rise")''','''        low = tcp_f + [0, 0, 0.005]
        tip = self.go_R([low], [Rf], 4.0, "place: lower", retry=False)
        if not self.dry and tip[2] > low[2] + 0.012:
            print("   !! lower blocked, releasing anyway", flush=True)
        self.open_full()
        back = np.array([math.cos(phi), math.sin(phi), 0.0])
        self.go_R([low - 0.06 * back], [Rf], 2.0, "place: retreat")
        self.go_R([low - 0.06 * back + [0, 0, 0.15]], [Rf], 2.0, "place: rise")''')
s=s.replace('''    t = Recover(dry)
''','''    Z_BOTTOM_TO_WAIST = opt("zbw", Z_BOTTOM_TO_WAIST)
    GRASP_UP = opt("up", GRASP_UP)
    t = Recover(dry)
    st = opt("stage", None)
    if st is not None:
        t.stage = tuple(st)
    hw = opt("handle", None)
    if hw is not None:
        t.handle_world = np.array([hw[0], hw[1], 0.0])
''')
open("recover.py","w").write(s)
EOF
grep -n "Z_BOTTOM_TO_WAIST\|GRASP_UP" recover.py

# openrua op 101
cat > dryB.py <<'EOF'
import math, sys, numpy as np, recover
from recover import Recover
recover.GRASP_UP = 0.008
recover.Z_BOTTOM_TO_WAIST = 0.062
spot = tuple(map(float, sys.argv[1:3])); phi = float(sys.argv[3]); stage = tuple(map(float, sys.argv[4:6]))
t = Recover(True); t.stage = stage; t.handle_world = np.array([0.87, 0.494, 0.0])
t.grasp([0.235, -0.032, 0.967], [-0.494, 0.87], math.radians(-45), 1)
t.lift(1.10)
t.place(math.radians(180), spot, math.radians(phi))
print("DRY OK")
EOF
for args in "0.15 0.078 -60 0.09 0.13" "0.15 0.078 -45 0.09 0.13" "0.15 0.078 -90 0.10 0.10" "0.155 -0.012 60 0.09 -0.13" "0.155 -0.012 45 0.09 -0.13" "0.155 -0.012 0 0.05 0.0"; do echo "=== $args"; python3 dryB.py $args 2>&1 | grep -E "handle dir|final pot up|DRY OK|Error|IK failed|ik: no" | head -8; done

# openrua op 102
cat > dryA.py <<'EOF'
import math, sys, numpy as np, recover
from recover import Recover
recover.GRASP_UP = 0.008
recover.Z_BOTTOM_TO_WAIST = 0.065
spot = tuple(map(float, sys.argv[1:3])); phi = float(sys.argv[3]); stage = tuple(map(float, sys.argv[4:6])); theta=float(sys.argv[6])
t = Recover(True); t.stage = stage; t.handle_world = np.array([0.136, -0.991, 0.0])
t.grasp([0.044, -0.058, 0.961], [-0.991, -0.136], 0.0, 1)
t.lift(1.10)
t.place(math.radians(theta), spot, math.radians(phi))
print("DRY OK")
EOF
for args in "0.14 0.08 -60 0.09 0.13 196" "0.14 0.08 -45 0.09 0.13 196" "0.14 0.08 -75 0.08 0.14 196"; do echo "=== $args"; python3 dryA.py $args 2>&1 | grep -E "handle dir|final pot up|DRY OK|Error|IK failed|ik: no|sweep" | head -8; done

# openrua op 103
python3 - <<'EOF'
s=open("recover.py").read()
s=s.replace('''    def grasp(self, waist, d0, beta=0.0, sign=-1):''','''    def grasp(self, waist, d0, beta=0.0, sign=-1, pitch=0.0):''')
s=s.replace('''        R = Rotation.from_rotvec(beta * d3).as_matrix() @ np.column_stack([x, y, z])
''','''        R = Rotation.from_rotvec(beta * d3).as_matrix() @ np.column_stack([x, y, z]) @ Ry(pitch)  # pitch: about finger axis
''')
s=s.replace('''                math.radians(float(a[6])) if len(a) > 6 else 0.0, int(a[7]) if len(a) > 7 else -1)''','''                math.radians(float(a[6])) if len(a) > 6 else 0.0, int(a[7]) if len(a) > 7 else -1,
                math.radians(float(a[8])) if len(a) > 8 else 0.0)''')
open("recover.py","w").write(s)
s=open("dryA.py").read()
s=s.replace("t.grasp([0.044, -0.058, 0.961], [-0.991, -0.136], 0.0, 1)","t.grasp([0.044, -0.058, 0.961], [-0.991, -0.136], 0.0, 1, math.radians(-16))")
open("dryA.py","w").write(s)
EOF
for args in "0.14 0.08 -60 0.09 0.13 180" "0.14 0.08 -45 0.09 0.13 180" "0.14 0.08 -75 0.08 0.14 180"; do echo "=== $args"; python3 dryA.py $args 2>&1 | grep -E "grasp hand z|handle dir|final pot up|DRY OK|Error|IK failed|ik: no|sweep" | head -8; done

# openrua op 104
sed -i 's/t.grasp(\[0.044, -0.058, 0.961\], \[-0.991, -0.136\], 0.0, 1, math.radians(-16))/t.grasp([0.044, -0.058, 0.961], [-0.991, -0.136], 0.0, -1, math.radians(16))/' dryA.py
for args in "0.14 0.08 -60 0.09 0.13 0" "0.14 0.08 -45 0.09 0.13 0" "0.14 0.08 -45 0.06 0.10 0" "0.14 0.08 -30 0.06 0.08 0"; do echo "=== $args"; python3 dryA.py $args 2>&1 | grep -E "grasp hand z|handle dir|final pot up|DRY OK|Error|IK failed|ik: no|sweep" | head -8; done

# openrua op 105
python3 dryA.py 0.14 0.08 -45 0.09 0.13 0 2>&1 | grep -vE "^\s*$"

# openrua op 106
timeout 900 python3 -u recover.py grasp 0.044 -0.058 0.961 -0.991 -0.136 0 -1 16 --up=0.008 2>&1 | grep -v "^   q=" | tee graspA2.log

# openrua op 107
timeout 600 python3 -u recover.py lift 1.10 2>&1 | grep -v "^   q=" && python3 scene.py frontview 0.03 >/dev/null 2>&1 && python3 scene.py sideview 0.03 >/dev/null 2>&1 && python3 - <<'EOF'
import numpy as np
for cam in ("frontview","sideview"):
    P=np.load(f"{cam}_world.npy").reshape(-1,3); P=P[np.isfinite(P).all(1)]
    # pot in hand around TCP (0.042,-0.058,1.10): pot axis ~ along (0.991,0.136), so x from -0.05 to 0.15
    S=P[(P[:,2]>1.02)&(P[:,2]<1.16)&(P[:,1]>-0.13)&(P[:,1]<0.0)&(P[:,0]>-0.08)&(P[:,0]<0.16)]
    print(cam,"n",len(S))
    d=np.array([0.991,0.136]); c=np.array([0.042,-0.058])
    t=(S[:,:2]-c)@d
    for tl in np.arange(-0.10,0.12,0.01):
        m=(t>=tl)&(t<tl+0.01)
        if m.sum()<3: continue
        print(f"  t={tl:+.3f} n={m.sum():3d} z[{S[m,2].min():.3f},{S[m,2].max():.3f}] xy={S[m,:2].mean(0).round(3)}")
EOF

# openrua op 108
timeout 1500 python3 -u recover.py place 0 0.14 0.08 -45 --stage=0.09,0.13 --zbw=0.064 --up=0.008 --handle=0.136,-0.991 2>&1 | grep -v "^   q=" | tee placeA2.log

# openrua op 109
timeout 600 python3 -u recover.py home 2>&1 | tail -1 && python3 scene.py birdview 0.03 >/dev/null 2>&1 && python3 tools/perception/cam_snap.py birdview birdview.png >/dev/null 2>&1 && python3 - <<'EOF'
import numpy as np, cv2
P=np.load("birdview_world.npy").reshape(-1,3); P=P[np.isfinite(P).all(1)]
S=P[(np.linalg.norm(P[:,:2]-[0.14,0.08],axis=1)<0.07)&(P[:,2]>0.94)]
print("potA n",len(S),"zmax",S[:,2].max().round(3))
top=S[S[:,2]>1.07]; print("top pts",len(top),"center",top[:,:2].mean(0).round(3),"x[%.3f,%.3f] y[%.3f,%.3f]"%(top[:,0].min(),top[:,0].max(),top[:,1].min(),top[:,1].max()))
for zlo,zhi in [(0.94,0.98),(0.98,1.02),(1.02,1.06)]:
    m=(S[:,2]>=zlo)&(S[:,2]<zhi)
    if m.sum(): print(f"  z[{zlo},{zhi}] n={m.sum()} x[{S[m,0].min():.3f},{S[m,0].max():.3f}] y[{S[m,1].min():.3f},{S[m,1].max():.3f}]")
im=cv2.imread('birdview.png'); cv2.imwrite('bird_crop.png', cv2.resize(im[260:400,240:420],None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))
EOF

# openrua op 110
python3 tools/perception/cam_snap.py agentview agentview.png 2>&1 | tail -2; python3 tools/perception/cam_snap.py birdview birdview.png 2>&1 | tail -2; ls -la --time-style=full-iso agentview.png birdview.png; date

# openrua op 111
python3 -c "
import cv2
im=cv2.imread('birdview.png'); cv2.imwrite('bird_crop.png', cv2.resize(im[250:410,230:430],None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))"

# openrua op 112
python3 scene.py birdview 0.03 >/dev/null 2>&1 && python3 - <<'EOF'
import numpy as np
P=np.load("birdview_world.npy").reshape(-1,3); P=P[np.isfinite(P).all(1)]
S=P[(P[:,0]>0.05)&(P[:,0]<0.35)&(P[:,1]>0.06)&(P[:,1]<0.3)&(P[:,2]>0.91)]
c=S[:,:2].mean(0); U,s,Vt=np.linalg.svd(S[:,:2]-c); d=Vt[0]
if d[0]<0: d=-d
for it in range(3):
    t=(S[:,:2]-c)@d
    ridge=[]
    for tl in np.arange(t.min(),t.max(),0.005):
        m=(t>=tl)&(t<tl+0.005)
        if m.sum()<5: continue
        i=np.argmax(np.where(m,S[:,2],-1)); ridge.append(S[i])
    R=np.array(ridge); c=R[:,:2].mean(0); U,s,Vt=np.linalg.svd(R[:,:2]-c); d=Vt[0]
    if d[0]<0: d=-d
print(f"pot A: n={len(S)} ridge n={len(R)} center={c.round(3)} axis={d.round(3)} zmax={S[:,2].max():.3f}")
t=(S[:,:2]-c)@d; perp=(S[:,:2]-c)@np.array([-d[1],d[0]]); tr=(R[:,:2]-c)@d; pr=(R[:,:2]-c)@np.array([-d[1],d[0]])
for tl in np.arange(t.min(),t.max(),0.01):
    m=(t>=tl)&(t<tl+0.01); mr=(tr>=tl)&(tr<tl+0.01)
    if m.sum()<3: continue
    print(f"  t={tl:+.3f} n={m.sum():3d} zmax={S[m,2].max():.3f} perp[{perp[m].min():+.3f},{perp[m].max():+.3f}] ridge-perp={pr[mr].mean() if mr.sum() else float('nan'):+.3f} xy={S[m,:2].mean(0).round(3)}")
EOF

# openrua op 113
python3 - <<'EOF'
import numpy as np
P=np.load("birdview_world.npy").reshape(-1,3); P=P[np.isfinite(P).all(1)]
S=P[(P[:,0]>0.12)&(P[:,0]<0.4)&(P[:,1]>0.08)&(P[:,1]<0.3)&(P[:,2]>0.94)]
c=S[:,:2].mean(0); U,s,Vt=np.linalg.svd(S[:,:2]-c); d=Vt[0]
if d[0]<0: d=-d
for it in range(3):
    t=(S[:,:2]-c)@d
    ridge=[]
    for tl in np.arange(t.min(),t.max(),0.005):
        m=(t>=tl)&(t<tl+0.005)
        if m.sum()<5: continue
        i=np.argmax(np.where(m,S[:,2],-1)); ridge.append(S[i])
    R=np.array(ridge); c=R[:,:2].mean(0); U,s,Vt=np.linalg.svd(R[:,:2]-c); d=Vt[0]
    if d[0]<0: d=-d
print(f"pot A: n={len(S)} ridge n={len(R)} center={c.round(3)} axis={d.round(3)} zmax={S[:,2].max():.3f}")
t=(S[:,:2]-c)@d; perp=(S[:,:2]-c)@np.array([-d[1],d[0]]); tr=(R[:,:2]-c)@d; pr=(R[:,:2]-c)@np.array([-d[1],d[0]])
for tl in np.arange(t.min(),t.max(),0.01):
    m=(t>=tl)&(t<tl+0.01); mr=(tr>=tl)&(tr<tl+0.01)
    if m.sum()<3: continue
    print(f"  t={tl:+.3f} n={m.sum():3d} zmax={S[m,2].max():.3f} perp[{perp[m].min():+.3f},{perp[m].max():+.3f}] ridge-perp={pr[mr].mean() if mr.sum() else float('nan'):+.3f} ridge-z={R[mr,2].mean() if mr.sum() else float('nan'):.3f} xy={S[m,:2].mean(0).round(3)}")
EOF

# openrua op 114
python3 - <<'EOF' 2>&1 | grep -v "^  ik:"
import math, numpy as np
from scipy.spatial.transform import Rotation
import task, kin
from kin import ik
from rob import R_to_quat
from task import HOME, TCP_OFF
# hand z leaned from -Z toward the robot (-x) by beta; hand y horizontal along world y (fingers along y); x = y cross z
def R_lean(beta, yaw=0.0):
    z=np.array([-math.sin(beta),0,-math.cos(beta)]); y=np.array([0,1.0,0]); x=np.cross(y,z)
    R=np.column_stack([x,y,z]); return task.R_yaw(yaw)@R
print("TCP z=0.975; entries: max reachable x (m) for each beta (lean toward robot)")
for y in (-0.08,0.0,0.08,0.16):
    row=[]
    for bdeg in (0,20,30,40,50,60):
        best=None
        for x in np.arange(0.12,0.34,0.02):
            R=R_lean(math.radians(bdeg)); tcp=np.array([x,y,0.975]); pos=tcp-TCP_OFF*R[:,2]
            q=ik(pos,R_to_quat(R),HOME,w_post=0.03,tries=4)
            if q is not None: best=x
            elif best is not None: break
        row.append(f"b{bdeg}:{best if best is not None else '--'}")
    print(f"y={y:+.2f}  "+"  ".join(row))
EOF

# openrua op 115
cat > feas.py <<'EOF'
"""Brute-force feasible top-down-ish pinch orientations for a lying pot.
usage: feas.py wx wy wz ax ay   (waist point, horizontal axis dir)"""
import math, sys, numpy as np
import task, kin
from kin import ik, fk_all
from rob import R_to_quat
from task import HOME, TCP_OFF
p=np.array(list(map(float,sys.argv[1:4]))); a=np.array([float(sys.argv[4]),float(sys.argv[5]),0.0]); a/=np.linalg.norm(a)
Z=np.array([0,0,1.0]); h=np.cross(a,Z)   # horizontal, perpendicular to axis
res=[]
for gam in range(-60,61,15):            # tilt of the pinch line from horizontal
    g=math.radians(gam); y=math.cos(g)*h+math.sin(g)*Z
    # z must be perpendicular to y: z = cos(b)*(-Z') + sin(b)*a  where Z' = component of -Z perp to y
    zd=-Z-(-Z@y)*y; zd/=np.linalg.norm(zd)
    for bdeg in range(-60,61,15):        # lean along the pot axis (pitch)
        b=math.radians(bdeg); z=math.cos(b)*zd+math.sin(b)*a; x=np.cross(y,z)
        R=np.column_stack([x,y,z])
        if R[2,2]>-0.4: continue
        tcp=p-0.008*z; pos=tcp-TCP_OFF*z
        q=ik(pos,R_to_quat(R),HOME,w_post=0.03,tries=4)
        if q is None: continue
        qa=ik(pos-0.10*z,R_to_quat(R),q,w_post=0.03,tries=4)
        if qa is None: continue
        pts=[tcp+s*y+t*z+u*x for s in (-0.039,0.039) for t in (0.0,0.01) for u in (-0.009,0.009)]
        lowest=min(pt[2] for pt in pts)
        res.append((abs(gam),gam,bdeg,lowest,q))
        print(f"gamma={gam:+d} pitch={bdeg:+d}: OK lowest finger z={lowest:.3f} hand z={np.round(z,2)} q={np.round(q,2)}", flush=True)
print("feasible:",len(res))
EOF
echo "== pot B waist"; python3 feas.py 0.235 -0.032 0.967 -0.494 0.87 2>&1 | grep -v "^  ik:"
echo "== pot A waist"; python3 feas.py 0.234 0.144 0.945 -0.738 0.675 2>&1 | grep -v "^  ik:"

# openrua op 116
python3 - <<'EOF'
s=open("feas.py").read()
s=s.replace("for gam in range(-60,61,15):            # tilt of the pinch line from horizontal\n    g=math.radians(gam); y=math.cos(g)*h+math.sin(g)*Z",
"for gam in range(-60,61,15):            # tilt of the pinch line from horizontal\n  for sgn in (1,-1):\n    g=math.radians(gam); y=sgn*(math.cos(g)*h+math.sin(g)*Z)")
# indent the body of the gamma loop by 2 more spaces
lines=s.split("\n"); out=[]; inloop=False
for L in lines:
    if L.startswith("    g=math.radians(gam)"): inloop=True; out.append(L); continue
    if inloop and (L.startswith("    ") ):
        out.append("  "+L); continue
    if inloop and not L.startswith("    "): inloop=False
    out.append(L)
s="\n".join(out)
s=s.replace('print(f"gamma={gam:+d} pitch={bdeg:+d}:','print(f"gamma={gam:+d} sgn={sgn:+d} pitch={bdeg:+d}:')
open("feas.py","w").write(s)
EOF
python3 -c "import ast;ast.parse(open('feas.py').read())" && echo "== pot A waist"; python3 feas.py 0.234 0.144 0.945 -0.738 0.675 2>&1 | grep -v "^  ik:"

# openrua op 117
mkdir -p "$(dirname /workspace/feas.py)"
cat > /workspace/feas.py <<'OPENRUA_EOF'
"""Brute-force feasible top-down-ish pinch orientations for a lying pot.
usage: feas.py wx wy wz ax ay [up]  (waist point, horizontal axis dir bottom->lid)
Prints feasible (gamma, sgn, pitch) with the lowest finger point; y = sgn*(cos g*h + sin g*Z)."""
import math, sys, numpy as np
import task, kin
from kin import ik
from rob import R_to_quat
from task import HOME, TCP_OFF

p = np.array(list(map(float, sys.argv[1:4])))
a = np.array([float(sys.argv[4]), float(sys.argv[5]), 0.0]); a /= np.linalg.norm(a)
up = float(sys.argv[6]) if len(sys.argv) > 6 else 0.008
Z = np.array([0, 0, 1.0]); h = np.cross(a, Z)


def hand_R(gam, sgn, pitch):
    y = sgn * (math.cos(gam) * h + math.sin(gam) * Z)
    zd = -Z - (-Z @ y) * y; zd /= np.linalg.norm(zd)
    z = math.cos(pitch) * zd + math.sin(pitch) * a
    x = np.cross(y, z)
    return np.column_stack([x, y, z])


if __name__ == "__main__":
    n = 0
    for gam in range(-60, 61, 15):
        for sgn in (1, -1):
            for bdeg in range(-60, 61, 15):
                R = hand_R(math.radians(gam), sgn, math.radians(bdeg))
                if R[2, 2] > -0.4:
                    continue
                z = R[:, 2]; tcp = p - up * z; pos = tcp - TCP_OFF * z
                q = ik(pos, R_to_quat(R), HOME, w_post=0.03, tries=4)
                if q is None:
                    continue
                qa = ik(pos - 0.10 * z, R_to_quat(R), q, w_post=0.03, tries=4)
                if qa is None:
                    continue
                pts = [tcp + s * R[:, 1] + t * z + u * R[:, 0] for s in (-0.039, 0.039) for t in (0.0, 0.01) for u in (-0.009, 0.009)]
                low = min(pts, key=lambda v: v[2])
                n += 1
                print(f"gamma={gam:+d} sgn={sgn:+d} pitch={bdeg:+d}: lowest finger {np.round(low,3)} hand z={np.round(z,2)} q={np.round(q,2)}", flush=True)
    print("feasible:", n)
OPENRUA_EOF

# openrua op 118
echo "== pot A waist"; python3 feas.py 0.234 0.144 0.945 -0.738 0.675 2>&1 | grep -v "^  ik:"

# openrua op 119
python3 - <<'EOF' 2>&1 | grep -v "^  ik:"
import math, numpy as np, feas
from feas import hand_R
from kin import ik
from rob import R_to_quat
from task import HOME, TCP_OFF
p=np.array([0.234,0.144,0.945]); a=np.array([-0.738,0.675,0.0]); a/=np.linalg.norm(a)
feas.a=a; feas.h=np.cross(a,feas.Z)
seeds=[HOME, np.array([0.2,1.2,-0.3,-0.8,-0.5,2.8,-0.5]), np.array([0.3,1.5,-0.7,-0.5,-0.3,3.0,-0.3]), np.array([-0.3,1.4,0.6,-0.7,-0.2,2.7,-1.5]), np.array([0.0,1.0,0.0,-1.2,0.0,2.2,0.8])]
for gam in range(-60,61,15):
    for sgn in (1,-1):
        for bdeg in range(-60,61,15):
            R=hand_R(math.radians(gam),sgn,math.radians(bdeg))
            if R[2,2]>-0.4: continue
            z=R[:,2]; tcp=p-0.008*z; pos=tcp-TCP_OFF*z
            q=None
            for s in seeds:
                q=ik(pos,R_to_quat(R),s,w_post=0.03,tries=3)
                if q is not None: break
            if q is None: continue
            qa=ik(pos-0.10*z,R_to_quat(R),q,w_post=0.03,tries=4)
            pts=[tcp+s_*R[:,1]+t*z+u*R[:,0] for s_ in (-0.051,0.051) for t in (0.0,0.01) for u in (-0.009,0.009)]
            low=min(pts,key=lambda v:v[2])
            print(f"gamma={gam:+d} sgn={sgn:+d} pitch={bdeg:+d}: above {'OK' if qa is not None else '--'} lowest outer corner {np.round(low,3)} hand z={np.round(z,2)} q={np.round(q,2)}",flush=True)
EOF

# openrua op 120
python3 - <<'EOF' 2>&1 | grep -v "^  ik:"
import sys; sys.argv=["feas.py","0.234","0.144","0.945","-0.738","0.675"]
import math, numpy as np, feas
from feas import hand_R
from kin import ik
from rob import R_to_quat
from task import HOME, TCP_OFF
p=np.array([0.234,0.144,0.945]); a=feas.a
seeds=[HOME, np.array([0.2,1.2,-0.3,-0.8,-0.5,2.8,-0.5]), np.array([0.3,1.5,-0.7,-0.5,-0.3,3.0,-0.3]), np.array([-0.3,1.4,0.6,-0.7,-0.2,2.7,-1.5]), np.array([0.0,1.0,0.0,-1.2,0.0,2.2,0.8])]
for gam in range(-60,61,15):
    for sgn in (1,-1):
        for bdeg in range(-60,61,15):
            R=hand_R(math.radians(gam),sgn,math.radians(bdeg))
            if R[2,2]>-0.4: continue
            z=R[:,2]; tcp=p-0.008*z; pos=tcp-TCP_OFF*z
            q=None
            for s in seeds:
                q=ik(pos,R_to_quat(R),s,w_post=0.03,tries=3)
                if q is not None: break
            if q is None: continue
            qa=ik(pos-0.10*z,R_to_quat(R),q,w_post=0.03,tries=4)
            pts=[tcp+s_*R[:,1]+t*z+u*R[:,0] for s_ in (-0.051,0.051) for t in (0.0,0.01) for u in (-0.009,0.009)]
            low=min(pts,key=lambda v:v[2])
            print(f"gamma={gam:+d} sgn={sgn:+d} pitch={bdeg:+d}: above {'OK' if qa is not None else '--'} lowest outer corner {np.round(low,3)} hand z={np.round(z,2)} q={np.round(q,2)}",flush=True)
EOF

# openrua op 121
ls *.npy && python3 - <<'EOF'
import numpy as np
W=np.load("birdview_world.npy"); P=W.reshape(-1,3); P=P[np.isfinite(P).all(1)]
# pot A region
m=(P[:,0]>0.15)&(P[:,0]<0.32)&(P[:,1]>0.08)&(P[:,1]<0.24)&(P[:,2]>0.905)
A=P[m]
print("A pts",len(A))
# profile along y bins: for each y bin, max z and x range
for y0 in np.arange(0.09,0.23,0.01):
    s=A[(A[:,1]>=y0)&(A[:,1]<y0+0.01)]
    if len(s): print(f"y {y0:.2f}-{y0+0.01:.2f}: n={len(s)} zmax={s[:,2].max():.3f} x[{s[:,0].min():.3f},{s[:,0].max():.3f}]")
# plate edge near y=0.122: points with z 0.92-0.93
m2=(P[:,0]>0.15)&(P[:,0]<0.30)&(P[:,1]>0.10)&(P[:,1]<0.15)&(P[:,2]>0.915)&(P[:,2]<0.935)
E=P[m2]; print("plate-top-ish pts near edge: y max", E[:,1].max() if len(E) else None)
# pot near-side: points z>0.93 with min y per x
for x0 in np.arange(0.17,0.30,0.01):
    s=A[(A[:,0]>=x0)&(A[:,0]<x0+0.01)&(A[:,2]>0.93)]
    if len(s): print(f"x {x0:.2f}: ymin={s[:,1].min():.3f} ymax={s[:,1].max():.3f} zmax={s[:,2].max():.3f}")
EOF

# openrua op 122
ls -la --time-style=+%H:%M:%S *.npy *.png *.log | sort -k6 | tail -20; ls /workspace; head -40 /workspace/scene.py

# openrua op 123
python3 - <<'EOF'
import numpy as np
W=np.load("birdview_world.npy"); P=W.reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,0]>0.14)&(P[:,0]<0.33)&(P[:,1]>0.07)&(P[:,1]<0.24)&(P[:,2]>0.905)
A=P[m]
for zt in (0.95,0.96,0.965,0.97):
    R=A[A[:,2]>zt]
    c=R[:,:2].mean(0); u,s,vt=np.linalg.svd(R[:,:2]-c)
    print(f"z>{zt}: n={len(R)} centre={np.round(c,3)} axis={np.round(vt[0],3)} sv={np.round(s,3)}")
R=A[A[:,2]>0.96]; c=R[:,:2].mean(0); u,s,vt=np.linalg.svd(R[:,:2]-c); a=vt[0]; b=vt[1]
t=(A[:,:2]-c)@a; w=(A[:,:2]-c)@b
for t0 in np.arange(-0.10,0.10,0.01):
    s=A[(t>=t0)&(t<t0+0.01)]
    ws=w[(t>=t0)&(t<t0+0.01)]
    if len(s): print(f"t {t0:+.2f}: n={len(s)} zmax={s[:,2].max():.3f} w[{ws.min():+.3f},{ws.max():+.3f}]  w(z>0.95)[{ws[s[:,2]>0.95].min() if (s[:,2]>0.95).any() else 0:+.3f},{ws[s[:,2]>0.95].max() if (s[:,2]>0.95).any() else 0:+.3f}]")
EOF

# openrua op 124
python3 - <<'EOF'
import numpy as np, cv2
W=np.load("birdview_world.npy"); img=cv2.imread("birdview.png")
H,Wd=W.shape[:2]
X=W[...,0]; Y=W[...,1]; Z=W[...,2]
m=(X>0.10)&(X<0.35)&(Y>0.05)&(Y<0.26)
vs,us=np.where(m); v0,v1,u0,u1=vs.min(),vs.max(),us.min(),us.max()
print(v0,v1,u0,u1)
crop=img[v0:v1,u0:u1].copy()
# height colouring overlay: draw contours of z>0.93 and z>0.96
for zt,col in ((0.93,(0,255,0)),(0.96,(255,0,0)),(0.975,(0,0,255))):
    mk=((Z>zt)&m).astype(np.uint8)[v0:v1,u0:u1]
    cs,_=cv2.findContours(mk,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(crop,cs,-1,col,1)
# grid lines every 2 cm: find pixel where X≈x0
for x0 in np.arange(0.10,0.35,0.02):
    mm=np.abs(X-x0)<0.001
    vv=np.where(mm.any(1))[0]; 
    if len(vv): 
        v=int(np.median(vv))-v0; cv2.line(crop,(0,v),(crop.shape[1],v),(80,80,80),1); cv2.putText(crop,f"{x0:.2f}",(0,v-2),0,0.35,(0,0,0),1)
for y0 in np.arange(0.06,0.26,0.02):
    mm=np.abs(Y-y0)<0.001
    uu=np.where(mm.any(0))[0]
    if len(uu):
        u=int(np.median(uu))-u0; cv2.line(crop,(u,0),(u,crop.shape[0]),(80,80,80),1); cv2.putText(crop,f"{y0:.2f}",(u,10),0,0.35,(0,0,0),1)
crop=cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_NEAREST)
cv2.imwrite("potA_grid.png",crop)
EOF

# openrua op 125
python3 - <<'EOF'
import numpy as np, cv2
W=np.load("birdview_world.npy"); img=cv2.imread("birdview.png")
X=W[...,0]; Y=W[...,1]; Z=W[...,2]
m=(X>0.10)&(X<0.35)&(Y>0.05)&(Y<0.26)
vs,us=np.where(m); v0,v1,u0,u1=vs.min(),vs.max()+1,us.min(),us.max()+1
S=8
crop=cv2.resize(img[v0:v1,u0:u1],None,fx=S,fy=S,interpolation=cv2.INTER_NEAREST)
for zt,col in ((0.93,(0,255,0)),(0.955,(255,0,0)),(0.972,(0,0,255))):
    mk=((Z>zt)&m).astype(np.uint8)[v0:v1,u0:u1]
    mk=cv2.resize(mk,None,fx=S,fy=S,interpolation=cv2.INTER_NEAREST)
    cs,_=cv2.findContours(mk,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(crop,cs,-1,col,1)
def px_of_x(x0):
    vv=np.where((np.abs(X-x0)<0.002).any(1))[0]; return (int(np.median(vv))-v0)*S+S//2
def px_of_y(y0):
    uu=np.where((np.abs(Y-y0)<0.002).any(0))[0]; return (int(np.median(uu))-u0)*S+S//2
for x0 in np.arange(0.10,0.35,0.05):
    v=px_of_x(x0); cv2.line(crop,(0,v),(crop.shape[1],v),(0,0,0),1); cv2.putText(crop,f"x{x0:.2f}",(2,v-3),0,0.45,(0,0,0),1)
for y0 in np.arange(0.06,0.26,0.04):
    u=px_of_y(y0); cv2.line(crop,(u,0),(u,crop.shape[0]),(0,0,0),1); cv2.putText(crop,f"y{y0:.2f}",(u+2,14),0,0.45,(0,0,0),1)
# plate edge y=0.122 dashed
u=px_of_y(0.122); cv2.line(crop,(u,0),(u,crop.shape[0]),(0,200,255),2)
cv2.imwrite("potA_grid.png",crop)
print(crop.shape)
EOF

# openrua op 126
python3 - <<'EOF'
import numpy as np
W=np.load("birdview_world.npy"); P=W.reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,0]>0.14)&(P[:,0]<0.34)&(P[:,1]>0.10)&(P[:,1]<0.24)&(P[:,2]>0.945)
A=P[m]
c=np.array([0.24,0.165]); a=np.array([1.0,0]); 
for it in range(6):
    b=np.array([-a[1],a[0]])
    w=(A[:,:2]-c)@b; t=(A[:,:2]-c)@a
    body=A[(np.abs(w)<0.037)&(np.abs(t)<0.085)]
    # ridge: for each t bin take the highest points, fit line through top ridge (points within 8mm of local max)
    keep=[]
    tb=(body[:,:2]-c)@a
    for t0 in np.arange(-0.085,0.085,0.005):
        s=body[(tb>=t0)&(tb<t0+0.005)]
        if len(s): keep.append(s[s[:,2]>s[:,2].max()-0.006])
    ridge=np.vstack(keep)
    c=ridge[:,:2].mean(0); u,sv,vt=np.linalg.svd(ridge[:,:2]-c); a=vt[0]
    if a[0]<0: a=-a
    print(it,"centre",np.round(c,3),"axis",np.round(a,3),"n",len(ridge),"ridge z mean",ridge[:,2].mean().round(3))
b=np.array([-a[1],a[0]])
t=(A[:,:2]-c)@a; w=(A[:,:2]-c)@b
print("t range of body pts:", t[np.abs(w)<0.037].min().round(3), t[np.abs(w)<0.037].max().round(3))
for t0 in np.arange(-0.10,0.10,0.01):
    s=A[(t>=t0)&(t<t0+0.01)&(np.abs(w)<0.05)]; ws=w[(t>=t0)&(t<t0+0.01)&(np.abs(w)<0.05)]
    if len(s): 
        top=s[s[:,2]>s[:,2].max()-0.004]
        print(f"t {t0:+.2f}: n={len(s)} zmax={s[:,2].max():.3f} w(top)={((top[:,:2]-c)@b).mean():+.3f} w[{ws.min():+.3f},{ws.max():+.3f}]")
# handle points: w<-0.04
h=A[(w<-0.037)&(A[:,2]>0.95)]
print("handle pts",len(h),"centre",np.round(h.mean(0),3), "w range",np.round(((h[:,:2]-c)@b).min(),3),np.round(((h[:,:2]-c)@b).max(),3), "t range",np.round(((h[:,:2]-c)@a).min(),3),np.round(((h[:,:2]-c)@a).max(),3))
EOF

# openrua op 127
python3 -u feas.py 0.236 0.167 0.936 1 0.018 0.008 2>&1 | grep -v "^  ik:"

# openrua op 128
python3 - <<'EOF' 2>&1 | grep -v "^  ik:"
import sys; sys.argv=["feas.py","0.236","0.167","0.936","1","0.018"]
import math, numpy as np, feas
from feas import hand_R
from kin import ik
from rob import R_to_quat
from task import HOME, TCP_OFF
p=feas.p
seeds=[HOME, np.array([0.2,1.2,-0.3,-0.8,-0.5,2.8,-0.5]), np.array([0.3,1.5,-0.7,-0.5,-0.3,3.0,-0.3]), np.array([-0.3,1.4,0.6,-0.7,-0.2,2.7,-1.5]), np.array([0.0,1.0,0.0,-1.2,0.0,2.2,0.8]), np.array([0.32,1.68,-0.69,-0.5,-0.27,3.05,-0.29])]
for gam in range(-75,76,15):
    for sgn in (1,-1):
        for bdeg in range(-75,76,15):
            R=hand_R(math.radians(gam),sgn,math.radians(bdeg))
            if R[2,2]>-0.2: continue
            z=R[:,2]; tcp=p-0.008*z; pos=tcp-TCP_OFF*z
            q=None
            for s in seeds:
                q=ik(pos,R_to_quat(R),s,w_post=0.03,tries=2)
                if q is not None: break
            if q is None: continue
            qa=ik(pos-0.08*z,R_to_quat(R),q,w_post=0.03,tries=4)
            pts=[tcp+s_*R[:,1]+t*z+u*R[:,0] for s_ in (-0.051,0.051) for t in (0.0,0.01) for u in (-0.009,0.009)]
            low=min(pts,key=lambda v:v[2])
            print(f"gamma={gam:+d} sgn={sgn:+d} pitch={bdeg:+d}: above {'OK' if qa is not None else '--'} lowest corner {np.round(low,3)} hand z={np.round(z,2)} q={np.round(q,2)}",flush=True)
EOF

# openrua op 129
python3 - <<'EOF' 2>&1 | grep -v "^  ik:" | tail -30
import sys; sys.argv=["feas.py","0.236","0.167","0.936","1","0.018"]
import math, numpy as np, feas
from feas import hand_R
from kin import ik
from rob import R_to_quat
from task import HOME, TCP_OFF
p=feas.p
seeds=[HOME, np.array([0.2,1.2,-0.3,-0.8,-0.5,2.8,-0.5]), np.array([0.3,1.5,-0.7,-0.5,-0.3,3.0,-0.3]), np.array([-0.3,1.4,0.6,-0.7,-0.2,2.7,-1.5]), np.array([0.0,1.0,0.0,-1.2,0.0,2.2,0.8]), np.array([0.32,1.68,-0.69,-0.5,-0.27,3.05,-0.29])]
n=0
for gam in range(-75,76,15):
    for sgn in (1,-1):
        for bdeg in range(-75,76,15):
            R=hand_R(math.radians(gam),sgn,math.radians(bdeg))
            if R[2,2]>-0.2: continue
            z=R[:,2]; tcp=p-0.008*z; pos=tcp-TCP_OFF*z
            q=None
            for s in seeds:
                q=ik(pos,R_to_quat(R),s,w_post=0.03,tries=2)
                if q is not None: break
            if q is None: continue
            n+=1
            qa=ik(pos-0.08*z,R_to_quat(R),q,w_post=0.03,tries=4)
            pts=[tcp+s_*R[:,1]+t*z+u*R[:,0] for s_ in (-0.051,0.051) for t in (0.0,0.01) for u in (-0.009,0.009)]
            low=min(pts,key=lambda v:v[2])
            print(f"gamma={gam:+d} sgn={sgn:+d} pitch={bdeg:+d}: above {'OK' if qa is not None else '--'} lowest corner {np.round(low,3)} hand z={np.round(z,2)} q={np.round(q,2)}",flush=True)
print("feasible",n)
EOF

# openrua op 130
python3 - <<'EOF' 2>&1 | grep -v "^  ik:" | tail -40
import math, numpy as np
from kin import ik
from rob import R_to_quat
from task import HOME, TCP_OFF
import kin
p=np.array([0.236,0.167,0.936]); a=np.array([1,0.018,0]); a/=np.linalg.norm(a)
seeds=[HOME, np.array([0.2,1.2,-0.3,-0.8,-0.5,2.8,-0.5]), np.array([0.3,1.5,-0.7,-0.5,-0.3,3.0,-0.3]), np.array([-0.3,1.4,0.6,-0.7,-0.2,2.7,-1.5]), np.array([0.32,1.68,-0.69,-0.5,-0.27,3.05,-0.29])]
def R_from(z, roll):
    z=z/np.linalg.norm(z)
    ref=np.array([0,1.0,0]) if abs(z[1])<0.9 else np.array([1.0,0,0])
    y0=ref-(ref@z)*z; y0/=np.linalg.norm(y0)
    x0=np.cross(y0,z)
    c,s=math.cos(roll),math.sin(roll)
    y=c*y0+s*x0; x=np.cross(y,z)
    return np.column_stack([x,y,z])
for elev in (45,55,65,75):
    for az in (-30,-15,0,15,30):
        e=math.radians(elev); az_=math.radians(az)
        z=np.array([math.sin(e)*math.cos(az_), math.sin(e)*math.sin(az_), -math.cos(e)])
        for roll in range(0,180,30):
            R=R_from(z,math.radians(roll))
            tcp=p-0.008*z; pos=tcp-TCP_OFF*z
            q=None
            for s in seeds:
                q=ik(pos,R_to_quat(R),s,w_post=0.03,tries=2)
                if q is not None: break
            if q is None: continue
            perp=abs(R[:,1]@a)
            print(f"elev={elev} az={az} roll={roll}: y.a={perp:.2f} hand y={np.round(R[:,1],2)} q={np.round(q,2)}",flush=True)
EOF

# openrua op 131
python3 - <<'EOF' 2>&1 | grep -v "^  ik:" | tail -60
import math, numpy as np
from kin import ik
from rob import R_to_quat
from task import HOME, TCP_OFF
p=np.array([0.20,0.167,0.936]); a=np.array([1,0.018,0]); a/=np.linalg.norm(a)
seeds=[HOME, np.array([0.2,1.2,-0.3,-0.8,-0.5,2.8,-0.5]), np.array([0.3,1.5,-0.7,-0.5,-0.3,3.0,-0.3]), np.array([-0.3,1.4,0.6,-0.7,-0.2,2.7,-1.5]), np.array([0.32,1.68,-0.69,-0.5,-0.27,3.05,-0.29])]
def R_from(z, roll):
    z=z/np.linalg.norm(z)
    ref=np.array([0,1.0,0]) if abs(z[1])<0.9 else np.array([1.0,0,0])
    y0=ref-(ref@z)*z; y0/=np.linalg.norm(y0)
    x0=np.cross(y0,z)
    c,s=math.cos(roll),math.sin(roll)
    y=c*y0+s*x0; x=np.cross(y,z)
    return np.column_stack([x,y,z])
for elev in (30,45,55,65,75):
    for az in (-30,-15,0,15,30):
        e=math.radians(elev); az_=math.radians(az)
        z=np.array([math.sin(e)*math.cos(az_), math.sin(e)*math.sin(az_), -math.cos(e)])
        for roll in range(0,180,30):
            R=R_from(z,math.radians(roll))
            tcp=p-0.008*z; pos=tcp-TCP_OFF*z
            q=None
            for s in seeds:
                q=ik(pos,R_to_quat(R),s,w_post=0.03,tries=2)
                if q is not None: break
            if q is None: continue
            perp=abs(R[:,1]@a)
            print(f"elev={elev} az={az} roll={roll}: y.a={perp:.2f} hand y={np.round(R[:,1],2)} q={np.round(q,2)}",flush=True)
EOF

# openrua op 132
cat /workspace/dryB.py; cat /workspace/placeA2.log; grep -n "def gripper\|def move_joints\|def fk\|def fingers" -A12 /workspace/rob.py | head -80

# openrua op 133
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/recover.py", "old_string": "    def lift(self, z=1.10):", "new_string": "    def grasp_R(self, waist, a, gam, sgn, pitch, close=True):\\n        \\"\\"\\"Pinch of a lying pot with a general orientation (feas.py parametrization).\\n        a: horizontal pot axis dir (bottom->lid). Fingers along y = sgn*(cos(gam)*h + sin(gam)*Z), h = a x Z;\\n        hand z = tilt of the pinch normal by `pitch` toward a (negative: hand leans from the lid side).\\"\\"\\"\\n        a = np.array([a[0], a[1], 0.0]); a /= np.linalg.norm(a)\\n        Z = np.array([0, 0, 1.0]); h = np.cross(a, Z)\\n        y = sgn * (math.cos(gam) * h + math.sin(gam) * Z)\\n        zd = -Z - (-Z @ y) * y; zd /= np.linalg.norm(zd)\\n        z = math.cos(pitch) * zd + math.sin(pitch) * a\\n        x = np.cross(y, z)\\n        R = np.column_stack([x, y, z])\\n        print(\\"   grasp hand z:\\", np.round(z, 3), \\"hand y (fingers):\\", np.round(y, 3), flush=True)\\n        tcp = np.asarray(waist) - GRASP_UP * z\\n        pts = [tcp + s * y + t * z + u * x for s in (-0.051, 0.051) for t in (0.0, 0.01) for u in (-0.009, 0.009)]\\n        print(\\"   lowest finger corner:\\", np.round(min(pts, key=lambda v: v[2]), 3), flush=True)\\n        self.open_full()\\n        self.go_R([tcp - 0.10 * z], [R], 4.0, \\"grasp: above\\")\\n        self.go_R([tcp], [R], 3.0, \\"grasp: descend\\", retry=False)\\n        return self.gripper(0.0) if close else None\\n\\n    def place_auto(self, a_world, spot, phi):\\n        \\"\\"\\"Place using the pot up-direction measured in world (assumed rigid in the hand since grasp).\\"\\"\\"\\n        R0 = self.hand_R()\\n        a = np.array([a_world[0], a_world[1], a_world[2] if len(a_world) > 2 else 0.0]); a /= np.linalg.norm(a)\\n        up_hand = R0.T @ a\\n        theta = math.atan2(-up_hand[2], -up_hand[0])\\n        print(f\\"   pot up in hand frame {np.round(up_hand,3)} -> theta={math.degrees(theta):.1f} deg\\", flush=True)\\n        if abs(up_hand[1]) > 0.1:\\n            print(\\"   !! pot axis not in hand x-z plane; placement will be tilted\\", flush=True)\\n        return self.place(theta, spot, phi)\\n\\n    def relocate(self, new_xy, z_carry=1.05):\\n        \\"\\"\\"Carry the (lying) pot with the current hand orientation to new_xy at the grasp height, release, back off.\\"\\"\\"\\n        R = self.hand_R()\\n        pos = fk_hand(self.q)[:3, 3] if self.dry else self.r.fk()[0]\\n        tip0 = pos + R[:, 2] * TCP_OFF\\n        z_g = self.grasp_tcp_z\\n        self.go_R([[tip0[0], tip0[1], z_carry]], [R], 3.0, \\"relocate: lift\\")\\n        self.go_R([[new_xy[0], new_xy[1], z_carry]], [R], 4.0, \\"relocate: transport\\")\\n        self.go_R([[new_xy[0], new_xy[1], z_g + 0.004]], [R], 3.0, \\"relocate: lower\\", retry=False)\\n        self.open_full()\\n        back = np.array([new_xy[0], new_xy[1], z_g + 0.004]) - 0.09 * R[:, 2]\\n        self.go_R([back], [R], 3.0, \\"relocate: retreat\\")\\n        self.go_R([[back[0], back[1], back[2] + 0.12]], [R], 2.0, \\"relocate: rise\\")\\n\\n    def lift(self, z=1.10):", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 134
python3 - <<'EOF'
s=open("recover.py").read()
s=s.replace('''    def relocate(self, new_xy, z_carry=1.05):
        """Carry the (lying) pot with the current hand orientation to new_xy at the grasp height, release, back off."""
        R = self.hand_R()
        pos = fk_hand(self.q)[:3, 3] if self.dry else self.r.fk()[0]
        tip0 = pos + R[:, 2] * TCP_OFF
        z_g = self.grasp_tcp_z
''','''    def relocate(self, new_xy, z_g, z_carry=1.05):
        """Carry the (lying) pot with the current hand orientation to new_xy, set it down with the TCP at z_g, release, back off."""
        R = self.hand_R()
        pos = fk_hand(self.q)[:3, 3] if self.dry else self.r.fk()[0]
        tip0 = pos + R[:, 2] * TCP_OFF
''')
open("recover.py","w").write(s)
EOF

# openrua op 135
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/recover.py", "old_string": "        low = tcp_f + [0, 0, 0.005]\\n        tip = self.go_R([low], [Rf], 4.0, \\"place: lower\\", retry=False)\\n        if not self.dry and tip[2] > low[2] + 0.012:\\n            print(\\"   !! lower blocked, releasing anyway\\", flush=True)\\n        self.open_full()\\n        back = np.array([math.cos(phi), math.sin(phi), 0.0])\\n        self.go_R([low - 0.06 * back], [Rf], 2.0, \\"place: retreat\\")\\n        self.go_R([low - 0.06 * back + [0, 0, 0.15]], [Rf], 2.0, \\"place: rise\\")", "new_string": "        # lower to 1.5 cm above the nominal touchdown, then creep down until the arm can no longer\\n        # follow (pot bottom on the plate) or a 1 cm floor below nominal is reached\\n        low = tcp_f + [0, 0, 0.015]\\n        self.go_R([low], [Rf], 4.0, \\"place: lower\\")\\n        tip = low.copy()\\n        while low[2] > tcp_f[2] - 0.010 + 1e-6:\\n            low = low - [0, 0, 0.004]\\n            tip = self.go_R([low], [Rf], 1.0, \\"place: creep\\", retry=False)\\n            if not self.dry and tip[2] > low[2] + 0.0025:\\n                print(f\\"   touchdown detected: TCP z {tip[2]:.4f} vs cmd {low[2]:.4f}\\", flush=True)\\n                break\\n        if self.dry:\\n            tip = low\\n        self.open_full()\\n        back = np.array([math.cos(phi), math.sin(phi), 0.0])\\n        rel = np.array([tip[0], tip[1], tip[2]])\\n        self.go_R([rel - 0.03 * back], [Rf], 2.0, \\"place: retreat 1\\")\\n        self.go_R([rel - 0.07 * back], [Rf], 2.0, \\"place: retreat 2\\")\\n        self.go_R([rel - 0.07 * back + [0, 0, 0.15]], [Rf], 2.0, \\"place: rise\\")", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 136
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/recover.py", "old_string": "    elif a[0] == \\"lift\\":", "new_string": "    elif a[0] == \\"graspR\\":   # graspR wx wy wz ax ay gamma_deg sgn pitch_deg\\n        t.grasp_R([float(a[1]), float(a[2]), float(a[3])], [float(a[4]), float(a[5])],\\n                  math.radians(float(a[6])), int(a[7]), math.radians(float(a[8])))\\n    elif a[0] == \\"placeauto\\":   # placeauto ax ay az sx sy phi_deg   (a: pot bottom->lid dir in world, as at grasp)\\n        t.place_auto([float(a[1]), float(a[2]), float(a[3])], (float(a[4]), float(a[5])), math.radians(float(a[6])))\\n    elif a[0] == \\"relocate\\":   # relocate nx ny z_tcp\\n        t.relocate((float(a[1]), float(a[2])), float(a[3]))\\n    elif a[0] == \\"lift\\":", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 137
cat > /workspace/dryB2.py <<'EOF'
import math, sys, numpy as np, recover
from recover import Recover
recover.GRASP_UP = 0.008
recover.Z_BOTTOM_TO_WAIST = 0.062
gam, sgn, pitch = float(sys.argv[1]), int(sys.argv[2]), float(sys.argv[3])
spot = tuple(map(float, sys.argv[4:6])); phi = float(sys.argv[6]); stage = tuple(map(float, sys.argv[7:9]))
a = [-0.494, 0.87]
t = Recover(True); t.stage = stage; t.handle_world = np.array([0.87, 0.494, 0.0])
t.grasp_R([0.235, -0.032, 0.967], a, math.radians(gam), sgn, math.radians(pitch))
t.lift(1.10)
t.place_auto([a[0], a[1], 0.0], spot, math.radians(phi))
print("DRY OK")
EOF
python3 -u dryB2.py 30 1 -30 0.17 -0.012 60 0.09 -0.13 2>&1 | grep -v "^  ik:"

# openrua op 138
python3 -u recover.py graspR 0.235 -0.032 0.967 -0.494 0.87 30 1 -30 --up=0.008 2>&1 | grep -v "^  ik:" | tee graspB3.log

# openrua op 139
ls /workspace/tools /workspace/docs; sed -n 40,80p /workspace/scene.py

# openrua op 140
ls tools/perception tools/action && python3 scene.py frontview 0.02 2>&1 | tail -5 && python3 scene.py agentview 0.02 2>&1 | tail -3 && python3 scene.py robot0_eye_in_hand 0.02 2>&1 | tail -3

# openrua op 141
python3 tools/perception/cam_snap.py --help 2>&1 | head -20

# openrua op 142
sed -n 1,46p tools/perception/cam_snap.py | grep -n "argv\|imwrite" ; python3 tools/perception/cam_snap.py frontview && python3 tools/perception/cam_snap.py robot0_eye_in_hand && python3 tools/perception/cam_snap.py agentview; ls -t *.png | head -4

# openrua op 143
python3 - <<'EOF'
import cv2, numpy as np
W=np.load("frontview_world.npy"); img=cv2.imread("frontview.png")
X=W[...,0];Y=W[...,1];Z=W[...,2]
m=np.isfinite(Z)&(X>0.05)&(X<0.40)&(Y>-0.20)&(Y<0.15)&(Z>0.90)&(Z<1.20)
vs,us=np.where(m); print(vs.min(),vs.max(),us.min(),us.max())
crop=img[vs.min()-10:vs.max()+10, us.min()-10:us.max()+10]
cv2.imwrite("front_crop.png", cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
EOF

# openrua op 144
python3 -u - <<'EOF' 2>&1 | grep -v "^  ik:"
import numpy as np, recover
from recover import Recover
t=Recover(False)
t.open_full()
R=t.hand_R(); pos,_=t.r.fk(); tip=pos+R[:,2]*recover.TCP_OFF
print("tip",tip)
t.go_R([tip-0.10*R[:,2]],[R],3.0,"back off")
t.go_R([[tip[0]-0.06,tip[1],1.10]],[R],3.0,"up")
EOF
python3 scene.py birdview 0.02 2>&1 | tail -4; python3 tools/perception/cam_snap.py birdview

# openrua op 145
python3 -u recover.py home 2>&1 | tail -1 && python3 scene.py birdview 0.02 2>&1 | grep -v "blob [0-9]*: .*x\[-" ; python3 tools/perception/cam_snap.py birdview

# openrua op 146
cat > /workspace/potfit.py <<'EOF'
"""Fit a lying pot's axis from a birdview cloud region. usage: potfit.py x0 x1 y0 y1 [zmin]"""
import sys, numpy as np
x0,x1,y0,y1=map(float,sys.argv[1:5]); zmin=float(sys.argv[5]) if len(sys.argv)>5 else 0.945
W=np.load("birdview_world.npy"); P=W.reshape(-1,3); P=P[np.isfinite(P).all(1)]
A=P[(P[:,0]>x0)&(P[:,0]<x1)&(P[:,1]>y0)&(P[:,1]<y1)&(P[:,2]>zmin)]
print("pts",len(A),"zmax",A[:,2].max().round(3))
c=A[:,:2].mean(0); u,s,vt=np.linalg.svd(A[:,:2]-c); a=vt[0]
for it in range(8):
    b=np.array([-a[1],a[0]]); w=(A[:,:2]-c)@b; t=(A[:,:2]-c)@a
    body=A[(np.abs(w)<0.037)&(np.abs(t)<0.09)]; tb=(body[:,:2]-c)@a
    keep=[]
    for t0 in np.arange(-0.09,0.09,0.005):
        s_=body[(tb>=t0)&(tb<t0+0.005)]
        if len(s_): keep.append(s_[s_[:,2]>s_[:,2].max()-0.006])
    ridge=np.vstack(keep); c=ridge[:,:2].mean(0); u,s,vt=np.linalg.svd(ridge[:,:2]-c); a=vt[0]
print("centre",np.round(c,3),"axis",np.round(a,3))
b=np.array([-a[1],a[0]]); t=(A[:,:2]-c)@a; w=(A[:,:2]-c)@b
for t0 in np.arange(-0.10,0.10,0.01):
    s_=(t>=t0)&(t<t0+0.01)&(np.abs(w)<0.06); S=A[s_]; ws=w[s_]
    if len(S):
        top=S[S[:,2]>S[:,2].max()-0.004]
        print(f"t {t0:+.2f}: n={len(S)} zmax={S[:,2].max():.3f} w(top)={((top[:,:2]-c)@b).mean():+.3f} w[{ws.min():+.3f},{ws.max():+.3f}]")
h=A[(np.abs(w)>0.037)&(np.abs(w)<0.10)]
if len(h): print("side pts (handle?)",len(h),"mean",np.round(h.mean(0),3),"w",np.round(((h[:,:2]-c)@b).mean(),3),"t range",np.round(((h[:,:2]-c)@a).min(),3),np.round(((h[:,:2]-c)@a).max(),3))
EOF
python3 potfit.py 0.15 0.33 -0.12 0.09

# openrua op 147
python3 - <<'EOF'
import numpy as np
W=np.load("birdview_world.npy"); P=W.reshape(-1,3); P=P[np.isfinite(P).all(1)]
c=np.array([0.251,-0.026]); a=np.array([-0.385,0.923]); b=np.array([-a[1],a[0]])
A=P[(P[:,0]>0.15)&(P[:,0]<0.36)&(P[:,1]>-0.14)&(P[:,1]<0.09)&(P[:,2]>0.935)]
t=(A[:,:2]-c)@a; w=(A[:,:2]-c)@b
H=A[w<-0.038]; th=t[w<-0.038]; wh=w[w<-0.038]
print("handle-side pts",len(H))
for t0 in np.arange(-0.05,0.07,0.01):
    m=(th>=t0)&(th<t0+0.01)
    if m.sum(): print(f"t {t0:+.2f}: n={m.sum()} w[{wh[m].min():+.3f},{wh[m].max():+.3f}] z[{H[m][:,2].min():.3f},{H[m][:,2].max():.3f}]")
# profile of body cross-section width vs t at z>0.95 (pot sides)
print("body:")
for t0 in np.arange(-0.09,0.07,0.01):
    m=(t>=t0)&(t<t0+0.01)&(w>-0.038)&(w<0.06)
    if m.sum(): print(f"t {t0:+.2f}: w[{w[m].min():+.3f},{w[m].max():+.3f}] zmax={A[m][:,2].max():.3f}")
EOF

# openrua op 148
python3 -u feas.py 0.257 -0.040 0.968 -0.385 0.923 0.008 2>&1 | grep -v "^  ik:"

# openrua op 149
cat > /workspace/feas2.py <<'EOF'
"""Multi-seed feasibility scan. usage: feas2.py wx wy wz ax ay [up] [above]"""
import math, sys, numpy as np
from kin import ik
from rob import R_to_quat
from task import HOME, TCP_OFF
from feas import hand_R as _hr
import feas

def hand_R(a, gam, sgn, pitch):
    Z = np.array([0, 0, 1.0]); h = np.cross(a, Z)
    y = sgn * (math.cos(gam) * h + math.sin(gam) * Z)
    zd = -Z - (-Z @ y) * y; zd /= np.linalg.norm(zd)
    z = math.cos(pitch) * zd + math.sin(pitch) * a
    x = np.cross(y, z)
    return np.column_stack([x, y, z])

SEEDS = [HOME, np.array([0.2,1.2,-0.3,-0.8,-0.5,2.8,-0.5]), np.array([0.3,1.5,-0.7,-0.5,-0.3,3.0,-0.3]),
         np.array([-0.3,1.4,0.6,-0.7,-0.2,2.7,-1.5]), np.array([0.0,1.0,0.0,-1.2,0.0,2.2,0.8]),
         np.array([0.32,1.68,-0.69,-0.5,-0.27,3.05,-0.29]), np.array([-0.06,1.38,0.04,-0.75,-0.64,2.77,-0.69]),
         np.array([0.46,1.65,-1.11,-0.59,-1.33,3.29,0.16])]

def solve(pos, R, tries=2):
    for s in SEEDS:
        q = ik(pos, R_to_quat(R), s, w_post=0.03, tries=tries)
        if q is not None:
            return q
    return None

if __name__ == "__main__":
    p = np.array(list(map(float, sys.argv[1:4])))
    a = np.array([float(sys.argv[4]), float(sys.argv[5]), 0.0]); a /= np.linalg.norm(a)
    up = float(sys.argv[6]) if len(sys.argv) > 6 else 0.008
    above = float(sys.argv[7]) if len(sys.argv) > 7 else 0.08
    n = 0
    for gam in range(-60, 61, 15):
        for sgn in (1, -1):
            for bdeg in range(-75, 76, 15):
                R = hand_R(a, math.radians(gam), sgn, math.radians(bdeg))
                if R[2, 2] > -0.3:
                    continue
                z = R[:, 2]; tcp = p - up * z; pos = tcp - TCP_OFF * z
                q = solve(pos, R)
                if q is None:
                    continue
                qa = solve(pos - above * z, R)
                pts = [tcp + s * R[:, 1] + t * z + u * R[:, 0] for s in (-0.051, 0.051) for t in (0.0, 0.01) for u in (-0.009, 0.009)]
                low = min(pts, key=lambda v: v[2])
                n += 1
                print(f"gamma={gam:+d} sgn={sgn:+d} pitch={bdeg:+d}: above {'OK' if qa is not None else '--'} lowest corner {np.round(low,3)} hand z={np.round(z,2)} q={np.round(q,2)}", flush=True)
    print("feasible:", n)
EOF
sed -i 's/^p = np.array(list(map(float, sys.argv\[1:4\])))$/p = np.array(list(map(float, sys.argv[1:4]))) if len(sys.argv) > 5 else None/; s/^a = np.array(\[float(sys.argv\[4\]), float(sys.argv\[5\]), 0.0\]); a \/= np.linalg.norm(a)$/a = (np.array([float(sys.argv[4]), float(sys.argv[5]), 0.0]) if len(sys.argv) > 5 else np.array([1.0, 0, 0])); a \/= np.linalg.norm(a)/' feas.py
python3 -u feas2.py 0.257 -0.040 0.968 -0.385 0.923 0.008 2>&1 | grep -v "^  ik:"

# openrua op 150
cat /workspace/machine.yaml; sed -n 1,60p tools/action/base_goto.py; grep -n -i "base\|mobile\|wheel" docs/10-machine.md docs/30-action.md | head -20

# openrua op 151
python3 - <<'EOF' 2>&1 | grep -v "^  ik:" | sort -t= -k5 -n | head -30
import math, numpy as np, kin, task
from kin import ik
from rob import R_to_quat
from task import TCP_OFF
from feas2 import hand_R, SEEDS
kin.LIMITS = np.array([[-2.89,2.89],[-1.75,1.75],[-2.89,2.89],[-3.06,-0.08],[-2.89,2.89],[-0.01,3.74],[-2.89,2.89]])
p=np.array([0.257,-0.040,0.968]); a=np.array([-0.385,0.923,0]); a/=np.linalg.norm(a)
from scipy.optimize import least_squares
def best(pos,R):
    Rt=R; b=None
    for s in SEEDS:
        def resid(q):
            T=kin.fk_hand(q); return np.concatenate([(T[:3,3]-pos)*10,(T[:3,:3]-Rt).ravel()*2])
        r=least_squares(resid,s,bounds=(kin.LIMITS[:,0],kin.LIMITS[:,1]),xtol=1e-10,ftol=1e-10,max_nfev=3000)
        T=kin.fk_hand(r.x); perr=np.linalg.norm(T[:3,3]-pos); Re=Rt.T@T[:3,:3]; oerr=math.degrees(math.acos(max(-1,min(1,(np.trace(Re)-1)/2))))
        if b is None or perr+0.001*oerr<b[0]+0.001*b[1]: b=(perr,oerr,r.x)
    return b
for gam in (0,15,30,45,60):
    for bdeg in (-60,-45,-30,-15,0):
        R=hand_R(a,math.radians(gam),1,math.radians(bdeg)); z=R[:,2]
        if R[2,2]>-0.3: continue
        pos=p-0.008*z-TCP_OFF*z
        perr,oerr,q=best(pos,R)
        print(f"gam={gam} pitch={bdeg} best perr={perr:.4f} oerr={oerr:.1f} q={np.round(q,2)}")
EOF

# openrua op 152
python3 - <<'EOF' 2>&1 | tail -12
import math, numpy as np, kin, task
from task import TCP_OFF
from feas2 import hand_R
from scipy.optimize import least_squares
kin.LIMITS = np.array([[-2.89,2.89],[-1.75,1.75],[-2.89,2.89],[-3.06,-0.08],[-2.89,2.89],[-0.01,3.74],[-2.89,2.89]])
p=np.array([0.257,-0.040,0.968]); a=np.array([-0.385,0.923,0]); a/=np.linalg.norm(a)
rng=np.random.default_rng(1)
for gam,bdeg in ((45,-30),(30,-30),(45,-15),(60,-30)):
    R=hand_R(a,math.radians(gam),1,math.radians(bdeg)); z=R[:,2]; pos=p-0.008*z-TCP_OFF*z
    def resid(q):
        T=kin.fk_hand(q); return np.concatenate([(T[:3,3]-pos)*10,(T[:3,:3]-R).ravel()*2])
    best=None
    for k in range(150):
        s=rng.uniform(kin.LIMITS[:,0],kin.LIMITS[:,1])
        r=least_squares(resid,s,bounds=(kin.LIMITS[:,0],kin.LIMITS[:,1]),xtol=1e-9,ftol=1e-9,max_nfev=1500)
        T=kin.fk_hand(r.x); perr=np.linalg.norm(T[:3,3]-pos); Re=R.T@T[:3,:3]; oerr=math.degrees(math.acos(max(-1,min(1,(np.trace(Re)-1)/2))))
        if best is None or r.cost<best[0]: best=(r.cost,perr,oerr,r.x)
    print(f"gam={gam} pitch={bdeg}: best perr={best[1]:.4f} oerr={best[2]:.2f} q={np.round(best[3],2)}")
EOF

# openrua op 153
python3 -u feas2.py 0.221 0.046 0.968 -0.385 0.923 0.0 0.08 2>&1 | grep -v "^  ik:"

# openrua op 154
mkdir -p "$(dirname /workspace/knob.py)"
cat > /workspace/knob.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Recover a lying moka pot by its lid knob: pinch the knob, lift (the pot swings upright and hangs
from the knob), carry over the spot, lower until the bottom touches, open, rise.
Usage:
  python3 knob.py pick  kx ky kz  ax ay  gamma_deg sgn pitch_deg [--dry]   # knob centre, pot axis (bottom->lid)
  python3 knob.py place sx sy drop_m [--dry]      # drop_m: TCP z - pot bottom z while hanging
  python3 knob.py rise | home
"""
import math, sys
import numpy as np
from recover import Recover, opt, PLATE_TOP
from task import TCP_OFF
from kin import fk_hand


class Knob(Recover):
    def pick(self, knob, a, gam, sgn, pitch, lift=0.20):
        a = np.array([a[0], a[1], 0.0]); a /= np.linalg.norm(a)
        Z = np.array([0, 0, 1.0]); h = np.cross(a, Z)
        y = sgn * (math.cos(gam) * h + math.sin(gam) * Z)
        zd = -Z - (-Z @ y) * y; zd /= np.linalg.norm(zd)
        z = math.cos(pitch) * zd + math.sin(pitch) * a
        x = np.cross(y, z)
        R = np.column_stack([x, y, z])
        tcp = np.asarray(knob, float)
        pts = [tcp + s * y + t * z + u * x for s in (-0.051, 0.051) for t in (0.0, 0.01) for u in (-0.009, 0.009)]
        print("   hand z:", np.round(z, 3), "fingers:", np.round(y, 3), "lowest finger corner:", np.round(min(pts, key=lambda v: v[2]), 3), flush=True)
        self.open_full()
        self.go_R([tcp - 0.08 * z], [R], 4.0, "knob: above")
        self.go_R([tcp], [R], 3.0, "knob: descend", retry=False)
        f = self.gripper(0.0)
        if not self.dry:
            w = f[0] - f[1]
            print(f"   pinch width {w*100:.2f} cm", flush=True)
            if w < 0.004:
                print("   !! nothing in the fingers", flush=True)
                return False
        self.go_R([tcp + [0, 0, 0.5 * lift], tcp + [0, 0, lift]], [R, R], 6.0, "knob: lift")
        return True

    def place(self, spot, drop):
        R = self.hand_R()
        pos = fk_hand(self.q)[:3, 3] if self.dry else self.r.fk()[0]
        tip0 = pos + R[:, 2] * TCP_OFF
        z_touch = PLATE_TOP + drop
        high = np.array([spot[0], spot[1], tip0[2]])
        self.go_R([tip0 * 0.5 + high * 0.5, high], [R, R], 6.0, "knob place: transport")
        low = np.array([spot[0], spot[1], z_touch + 0.015])
        self.go_R([low], [R], 4.0, "knob place: lower")
        tip = low.copy()
        while low[2] > z_touch - 0.012 + 1e-6:
            low = low - [0, 0, 0.004]
            tip = self.go_R([low], [R], 1.0, "knob place: creep", retry=False)
            if not self.dry and tip[2] > low[2] + 0.0025:
                print(f"   touchdown: TCP z {tip[2]:.4f} vs cmd {low[2]:.4f}", flush=True)
                break
        self.open_full()
        if self.dry:
            tip = low
        self.go_R([[tip[0], tip[1], tip[2] + 0.15]], [R], 3.0, "knob place: rise")


if __name__ == "__main__":
    dry = "--dry" in sys.argv
    a = [x for x in sys.argv[1:] if not x.startswith("--")]
    t = Knob(dry)
    if a[0] == "pick":
        ok = t.pick([float(a[1]), float(a[2]), float(a[3])], [float(a[4]), float(a[5])],
                    math.radians(float(a[6])), int(a[7]), math.radians(float(a[8])))
        print("PICK", "OK" if ok else "FAILED")
    elif a[0] == "place":
        t.place((float(a[1]), float(a[2])), float(a[3]))
    elif a[0] == "rise":
        R = t.hand_R(); pos = t.r.fk()[0]; tip = pos + R[:, 2] * TCP_OFF
        t.go_R([[tip[0], tip[1], tip[2] + 0.15]], [R], 3.0, "rise")
    elif a[0] == "home":
        t.home()
    if not dry:
        print("fingers:", t.r.fingers())
OPENRUA_EOF

# openrua op 155
cat > /workspace/dryK.py <<'EOF'
import math, sys, numpy as np
from knob import Knob
k = list(map(float, sys.argv[1:9])); spot = (float(sys.argv[9]), float(sys.argv[10])); drop = float(sys.argv[11])
t = Knob(True)
t.pick(k[0:3], k[3:5], math.radians(k[5]), int(k[6]), math.radians(k[7]))
t.place(spot, drop)
print("DRY OK")
EOF
python3 -u dryK.py 0.2195 0.0497 0.973 -0.385 0.923 30 1 -30 0.17 -0.012 0.168 2>&1 | grep -v "^  ik:"

# openrua op 156
python3 - <<'EOF'
import numpy as np, cv2
W=np.load("birdview_world.npy"); img=cv2.imread("birdview.png")
X=W[...,0]; Y=W[...,1]; Z=W[...,2]
m=(X>0.15)&(X<0.33)&(Y>-0.13)&(Y<0.10)
vs,us=np.where(m); v0,v1,u0,u1=vs.min(),vs.max()+1,us.min(),us.max()+1
S=8
crop=cv2.resize(img[v0:v1,u0:u1],None,fx=S,fy=S,interpolation=cv2.INTER_NEAREST)
for zt,col in ((0.94,(0,255,0)),(0.96,(255,0,0)),(0.99,(0,0,255))):
    mk=((Z>zt)&m).astype(np.uint8)[v0:v1,u0:u1]
    mk=cv2.resize(mk,None,fx=S,fy=S,interpolation=cv2.INTER_NEAREST)
    cs,_=cv2.findContours(mk,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(crop,cs,-1,col,1)
# mark knob target and plate corners using table-level pixel mapping: find nearest pixel with world xy (at pot height use Z of that pixel...) 
def px(x0,y0,z0):
    d=(X-x0)**2+(Y-y0)**2+(Z-z0)**2; v,u=np.unravel_index(np.nanargmin(np.where(np.isfinite(d),d,9)),d.shape); return ((u-u0)*S+S//2,(v-v0)*S+S//2)
cv2.circle(crop,px(0.2195,0.0497,0.973),6,(0,255,255),2)
cv2.circle(crop,px(0.257,-0.040,0.968),6,(255,0,255),2)
cv2.imwrite("potB_grid.png",crop); print(crop.shape)
EOF

# openrua op 157
python3 -u knob.py pick 0.2195 0.0497 0.973 -0.385 0.923 30 1 -30 2>&1 | grep -v "^  ik:" | tee knobB.log

# openrua op 158
python3 tools/perception/cam_snap.py frontview && python3 tools/perception/cam_snap.py agentview && python3 - <<'EOF'
import cv2
img=cv2.imread("frontview.png"); cv2.imwrite("front_crop.png", cv2.resize(img[230:480,150:450],None,fx=2.5,fy=2.5,interpolation=cv2.INTER_CUBIC))
img=cv2.imread("agentview.png"); print(img.shape)
EOF

# openrua op 159
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/knob.py", "old_string": "    def place(self, spot, drop):", "new_string": "    def drag(self, target_xy, seconds=8.0):\\n        \\"\\"\\"Slide the lying pot along the plate by pulling the pinched knob horizontally.\\"\\"\\"\\n        R = self.hand_R()\\n        pos = fk_hand(self.q)[:3, 3] if self.dry else self.r.fk()[0]\\n        tip0 = pos + R[:, 2] * TCP_OFF\\n        end = np.array([target_xy[0], target_xy[1], tip0[2]])\\n        wps = [tip0 * (1 - s) + end * s for s in (0.25, 0.5, 0.75, 1.0)]\\n        self.go_R(wps, [R] * 4, seconds, \\"knob: drag\\")\\n        if not self.dry:\\n            f = self.r.fingers(); print(f\\"   pinch width after drag {(f[0]-f[1])*100:.2f} cm\\", flush=True)\\n        self.open_full()\\n        self.go_R([end + [0, 0, 0.12]], [R], 3.0, \\"knob: rise\\")\\n\\n    def place(self, spot, drop):", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 160
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/knob.py", "old_string": "    elif a[0] == \\"place\\":", "new_string": "    elif a[0] == \\"drag\\":   # pick (no lift) then pull the knob horizontally to (tx, ty), release, rise\\n        ok = t.pick([float(a[1]), float(a[2]), float(a[3])], [float(a[4]), float(a[5])],\\n                    math.radians(float(a[6])), int(a[7]), math.radians(float(a[8])), lift=0.0)\\n        if ok:\\n            t.drag((float(a[9]), float(a[10])))\\n        else:\\n            t.open_full()\\n    elif a[0] == \\"place\\":", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 161
python3 -u -c "
from knob import Knob
t=Knob(False); t.open_full(); t.home(); print('fingers',t.r.fingers())
" 2>&1 | tail -5 && python3 -u scene.py birdview 0.02 2>&1 | tail -3 && python3 -u tools/perception/cam_snap.py birdview 2>&1 | tail -1 && python3 -u potfit.py 0.12 0.33 -0.13 0.10 2>&1 | tail -30

# openrua op 162
python3 -u scene.py frontview 0.02 2>&1 | tail -1; python3 - <<'EOF'
import numpy as np
c=np.array([0.251,-0.026]); a=np.array([-0.385,0.923]); h=np.array([0.923,0.385])
for cam in ("birdview","frontview"):
    W=np.load(f"{cam}_world.npy").reshape(-1,3); W=W[np.isfinite(W).all(1)]
    d=W[:,:2]-c; t=d@a; w=d@h
    m=(t>0.045)&(t<0.10)&(abs(w)<0.03)&(W[:,2]>0.945)
    P=W[m]; print(cam, len(P))
    for tt in np.arange(0.045,0.10,0.005):
        s=(t[m]>=tt)&(t[m]<tt+0.005)
        if s.sum(): print(f"  t {tt:.3f}: n={s.sum():3d} w[{w[m][s].min():+.3f},{w[m][s].max():+.3f}] z[{P[s,2].min():.3f},{P[s,2].max():.3f}]")
EOF

# openrua op 163
grep -n "effort\|GripperCommand" /workspace/rob.py | head; grep -rn "effort\|force" /workspace/docs 2>/dev/null | head; ls /workspace; ls /workspace/docs 2>/dev/null

# openrua op 164
grep -n -B2 -A8 "gripper" /workspace/machine.yaml | head -40; sed -n 110,125p /workspace/docs/30-action.md

# openrua op 165
timeout 600 python3 -u feas2.py 0.225 0.041 0.971 -0.385 0.923 0.0 0.08 2>&1 | grep -v "^  ik" | awk '$0 ~ /gamma=\+(0|15|30)/' | head -30

# openrua op 166
python3 -u knob.py drag 0.225 0.041 0.971 -0.385 0.923 30 1 -30 0.189 0.089 2>&1 | tee dragB.log | grep -v "^  ik" | tail -25

# openrua op 167
head -20 dragB.log | grep -v "^  ik"; python3 -u knob.py home 2>&1 | tail -1; python3 -u scene.py birdview 0.02 2>&1 | grep "blob" | awk '$0 ~ /x\[0\./' ; python3 -u tools/perception/cam_snap.py birdview >/dev/null; python3 -u potfit.py 0.10 0.33 -0.13 0.13 2>&1 | tail -22

# openrua op 168
python3 - <<'EOF'
import numpy as np
from PIL import Image, ImageDraw
W=np.load("birdview_world.npy"); img=Image.open("birdview.png").convert("RGB")
x=W[...,0]; y=W[...,1]
m=np.isfinite(x)&(x>0.08)&(x<0.34)&(y>-0.15)&(y<0.25)
rows,cols=np.where(m); r0,r1,c0,c1=rows.min(),rows.max(),cols.min(),cols.max()
crop=img.crop((c0,r0,c1+1,r1+1)); s=4; crop=crop.resize((crop.width*s,crop.height*s),Image.NEAREST)
d=ImageDraw.Draw(crop)
# grid: mark world points every 5 cm
for gx in np.arange(0.10,0.34,0.05):
    for gy in np.arange(-0.15,0.26,0.05):
        dd=(x-gx)**2+(y-gy)**2; dd[~np.isfinite(dd)]=9
        r,c=np.unravel_index(np.argmin(dd),dd.shape)
        if dd[r,c]<1e-4:
            px,py=(c-c0)*s,(r-r0)*s; d.ellipse((px-3,py-3,px+3,py+3),outline=(255,0,0))
            d.text((px+4,py-4),f"{gx:.2f},{gy:.2f}",fill=(255,255,0))
crop.save("potB_grid2.png"); print(crop.size, r0,r1,c0,c1)
EOF

# openrua op 169
python3 -u tools/perception/cam_snap.py frontview >/dev/null; python3 -u tools/perception/cam_snap.py sideview >/dev/null; python3 - <<'EOF'
import numpy as np
from PIL import Image
for cam in ("frontview","sideview"):
    W=np.load(f"{cam}_world.npy"); img=Image.open(f"{cam}.png").convert("RGB")
    x=W[...,0]; y=W[...,1]; z=W[...,2]
    m=np.isfinite(x)&(x>0.08)&(x<0.34)&(y>-0.15)&(y<0.25)&(z>0.90)
    rows,cols=np.where(m); r0,r1,c0,c1=rows.min()-40,rows.max()+10,cols.min()-20,cols.max()+20
    crop=img.crop((max(c0,0),max(r0,0),c1,r1)); s=3; crop=crop.resize((crop.width*s,crop.height*s),Image.LANCZOS)
    crop.save(f"{cam}_crop2.png"); print(cam,crop.size)
EOF

# openrua op 170
python3 - <<'EOF'
import numpy as np
for cam in ("birdview","frontview","sideview"):
    W=np.load(f"{cam}_world.npy").reshape(-1,3); W=W[np.isfinite(W).all(1)]
    m=(W[:,1]>0.10)&(W[:,1]<0.26)&(W[:,0]>0.12)&(W[:,0]<0.36)&(W[:,2]>0.915)
    P=W[m]; print(cam,len(P))
    for x0 in np.arange(0.13,0.35,0.01):
        s=(P[:,0]>=x0)&(P[:,0]<x0+0.01)
        if s.sum()>2: print(f"  x {x0:.2f}: n={s.sum():3d} y[{P[s,1].min():+.3f},{P[s,1].max():+.3f}] z[{P[s,2].min():.3f},{P[s,2].max():.3f}]")
EOF

# openrua op 171
python3 - <<'EOF'
import numpy as np
W=np.load("frontview_world.npy"); H,Wd,_=W.shape
print("shape",W.shape)
# world coords at a few pixel positions along the middle row / columns
for (r,c) in [(H//2,20),(H//2,Wd//2),(H//2,Wd-20),(H-20,Wd//2),(40,Wd//2)]:
    print((r,c), np.round(W[r,c],3))
# rays: for pixels in the same column, points at different depth; estimate camera by least squares on lines
pts=[];dirs=[]
for r in range(0,H,40):
    for c in range(0,Wd,40):
        p=W[r,c]
        if np.isfinite(p).all(): pts.append((r,c,p))
# fit: camera position C such that all pixel rays pass ... simpler: check the direction of increasing world x with row
xs=[(r,W[r,Wd//2,0]) for r in range(0,H,20) if np.isfinite(W[r,Wd//2,0])]
print("row vs x:",[(r,round(v,2)) for r,v in xs])
ys=[(c,W[H//2,c,1]) for c in range(0,Wd,40) if np.isfinite(W[H//2,c,1])]
print("col vs y:",[(c,round(v,2)) for c,v in ys])
EOF

# openrua op 172
python3 - <<'EOF'
import numpy as np
from PIL import Image, ImageDraw
W=np.load("birdview_world.npy"); img=Image.open("birdview.png").convert("RGB")
x=W[...,0]; y=W[...,1]
m=np.isfinite(x)&(x>0.14)&(x<0.31)&(y>-0.11)&(y<0.13)
rows,cols=np.where(m); r0,r1,c0,c1=rows.min(),rows.max(),cols.min(),cols.max()
crop=img.crop((c0,r0,c1+1,r1+1)); s=7; crop=crop.resize((crop.width*s,crop.height*s),Image.LANCZOS)
d=ImageDraw.Draw(crop)
for gx in np.arange(0.15,0.31,0.05):
    for gy in np.arange(-0.10,0.13,0.05):
        dd=(x-gx)**2+(y-gy)**2; dd[~np.isfinite(dd)]=9
        r,c=np.unravel_index(np.argmin(dd),dd.shape)
        if dd[r,c]<1e-4:
            px,py=(c-c0)*s,(r-r0)*s; d.ellipse((px-4,py-4,px+4,py+4),outline=(255,0,0))
            d.text((px+5,py-5),f"{gx:.2f},{gy:.2f}",fill=(255,255,0))
crop.save("potB_grid3.png"); print(crop.size)
EOF

# openrua op 173
timeout 900 python3 -u feas2.py 0.226 0.007 0.968 -0.399 0.917 0.01 0.10 2>&1 | grep -v "^  ik" | head -40

# openrua op 174
cat dryB2.py; grep -n "def place\b\|def place(" -A30 recover.py | head -80

# openrua op 175
cat > dryB3.py <<'EOF'
import math, sys, numpy as np, recover
from recover import Recover
recover.GRASP_UP = 0.01
recover.Z_BOTTOM_TO_WAIST = 0.062
waist = [0.226, 0.007, 0.968]; a = [-0.399, 0.917]
gam, sgn, pitch = float(sys.argv[1]), int(sys.argv[2]), float(sys.argv[3])
spot = tuple(map(float, sys.argv[4:6])); phi = float(sys.argv[6]); stage = tuple(map(float, sys.argv[7:9]))
live = "--live" in sys.argv
t = Recover(not live); t.stage = stage; t.handle_world = np.array([0.917, 0.399, 0.0])
t.grasp_R(waist, a, math.radians(gam), sgn, math.radians(pitch))
if live:
    f = t.r.fingers(); print("   grasp width %.2f cm" % ((f[0]-f[1])*100), flush=True)
    if f[0]-f[1] < 0.03:
        print("!! grasp failed, releasing"); t.open_full(); sys.exit(1)
t.lift(1.10)
t.place_auto([a[0], a[1], 0.0], spot, math.radians(phi))
print("DRY OK" if not live else "LIVE DONE")
if live: print("fingers:", t.r.fingers())
EOF
python3 -u dryB3.py 30 1 -15 0.17 -0.012 30 0.08 -0.06 2>&1 | grep -v "^  ik" | tail -40

# openrua op 176
python3 -u dryB3.py 30 1 -15 0.17 -0.012 30 0.08 -0.06 2>&1 | grep -v "^  ik" | head -30

# openrua op 177
sed -i 's/recover.Z_BOTTOM_TO_WAIST = 0.062/recover.Z_BOTTOM_TO_WAIST = 0.050/; s/waist = \[0.226, 0.007, 0.968\]/waist = [0.231, -0.005, 0.968]/' dryB3.py && python3 -u dryB3.py 30 1 -15 0.17 -0.012 30 0.08 -0.06 2>&1 | grep -v "^  ik" | grep -v "^   q=" | head -40

# openrua op 178
python3 -u dryB3.py 30 1 -15 0.17 -0.012 30 0.08 -0.06 --live 2>&1 | tee placeB3.log | grep -v "^  ik" | grep -v "^   q=" | tail -45

# openrua op 179
grep -v "^  ik" placeB3.log | grep -v "^   q=" | head -22

# openrua op 180
python3 -u recover.py home 2>&1 | tail -1; python3 -u scene.py birdview 0.02 2>&1 | grep blob | awk '$0 ~ /x\[0\./'; python3 -u tools/perception/cam_snap.py birdview >/dev/null; python3 -u tools/perception/cam_snap.py frontview >/dev/null; python3 - <<'EOF'
import numpy as np
from PIL import Image, ImageDraw
W=np.load("birdview_world.npy"); img=Image.open("birdview.png").convert("RGB")
x=W[...,0]; y=W[...,1]
m=np.isfinite(x)&(x>0.08)&(x<0.34)&(y>-0.15)&(y<0.25)
rows,cols=np.where(m); r0,r1,c0,c1=rows.min(),rows.max(),cols.min(),cols.max()
crop=img.crop((c0,r0,c1+1,r1+1)); s=4; crop=crop.resize((crop.width*s,crop.height*s),Image.LANCZOS)
d=ImageDraw.Draw(crop)
for gx in np.arange(0.10,0.34,0.05):
    for gy in np.arange(-0.15,0.26,0.05):
        dd=(x-gx)**2+(y-gy)**2; dd[~np.isfinite(dd)]=9
        r,c=np.unravel_index(np.argmin(dd),dd.shape)
        if dd[r,c]<1e-4:
            px,py=(c-c0)*s,(r-r0)*s; d.ellipse((px-3,py-3,px+3,py+3),outline=(255,0,0))
            d.text((px+4,py-4),f"{gx:.2f},{gy:.2f}",fill=(255,255,0))
crop.save("potB_grid4.png")
img=Image.open("frontview.png").convert("RGB"); img.crop((150,100,600,350)).resize((900,500),Image.LANCZOS).save("front_crop3.png")
EOF

# openrua op 181
python3 -u potfit.py 0.10 0.33 -0.13 0.13 2>&1 | tail -21

# openrua op 182
python3 - <<'EOF'
import numpy as np
W=np.load("birdview_world.npy").reshape(-1,3); W=W[np.isfinite(W).all(1)]
m=(W[:,0]>0.10)&(W[:,0]<0.33)&(W[:,1]>-0.13)&(W[:,1]<0.125)&(W[:,2]>0.995)
P=W[m]; print("ridge pts",len(P))
c=P[:,:2].mean(0); u,s,vt=np.linalg.svd(P[:,:2]-c); a=vt[0]; print("centre",np.round(c,3),"axis",np.round(a,3),"sv",np.round(s,3))
t=(P[:,:2]-c)@a; print("t range",t.min().round(3),t.max().round(3))
# all body points z>0.94 projected
m2=(W[:,0]>0.10)&(W[:,0]<0.33)&(W[:,1]>-0.13)&(W[:,1]<0.125)&(W[:,2]>0.94)
Q=W[m2]; h=np.array([a[1],-a[0]]); tq=(Q[:,:2]-c)@a; wq=(Q[:,:2]-c)@h
for t0 in np.arange(-0.10,0.10,0.01):
    s=(tq>=t0)&(tq<t0+0.01)
    if s.sum(): print(f"t {t0:+.2f}: n={s.sum():3d} w[{wq[s].min():+.3f},{wq[s].max():+.3f}] zmax={Q[s,2].max():.3f}")
EOF

# openrua op 183
python3 - <<'EOF'
import numpy as np
from PIL import Image
W=np.load("frontview_world.npy"); img=Image.open("frontview.png").convert("RGB")
x=W[...,0]; y=W[...,1]; z=W[...,2]
m=np.isfinite(x)&(x>0.08)&(x<0.34)&(y>-0.15)&(y<0.25)&(z>0.90)
rows,cols=np.where(m); r0,r1,c0,c1=rows.min()-40,rows.max()+10,cols.min()-20,cols.max()+20
crop=img.crop((max(c0,0),max(r0,0),c1,r1)); s=3; crop=crop.resize((crop.width*s,crop.height*s),Image.LANCZOS)
crop.save("frontview_crop3.png"); print(crop.size)
EOF

# openrua op 184
python3 - <<'EOF'
import numpy as np
W=np.load("birdview_world.npy").reshape(-1,3); W=W[np.isfinite(W).all(1)]
c=np.array([0.231,0.038]); a=np.array([0.041,-0.999]); h=np.array([a[1],-a[0]])  # h = (-0.999,-0.041)?? 
h=np.array([-0.999,-0.041]); 
# check: potfit's w sign: handle +w; frontview shows handle at +x -> so w>0 should be +x. use h=(0.999,0.041)
h=np.array([0.999,0.041])
d=W[:,:2]-c; t=d@a; w=d@h
m=(t>-0.09)&(t<0.09)&(abs(w)<0.09)&(W[:,2]>0.935)
P=W[m]; T=t[m]; Wd=w[m]
for t0 in np.arange(-0.09,0.09,0.01):
    s=(T>=t0)&(T<t0+0.01)
    if not s.sum(): continue
    row=[]
    for w0 in np.arange(-0.07,0.07,0.01):
        ss=s&(Wd>=w0)&(Wd<w0+0.01)
        row.append(f"{P[ss,2].max():.3f}" if ss.sum() else "  .  ")
    print(f"t {t0:+.2f}: "+" ".join(row))
print("cols w from -0.07 to +0.06 step 0.01")
EOF

# openrua op 185
python3 - <<'EOF'
import numpy as np
W=np.load("birdview_world.npy").reshape(-1,3); W=W[np.isfinite(W).all(1)]
c=np.array([0.231,0.038]); a=np.array([0.041,-0.999])
for it in range(6):
    h=np.array([a[1],-a[0]])
    d=W[:,:2]-c; t=d@a; w=d@h
    # crest: per t-bin the top point(s) within |w|<0.03
    m=(abs(t)<0.075)&(abs(w)<0.03)&(W[:,2]>0.94)
    P=W[m]; T=t[m]
    crest=[]
    for t0 in np.arange(-0.075,0.075,0.005):
        s=(T>=t0)&(T<t0+0.005)
        if s.sum()<3: continue
        zs=P[s,2]; top=zs>=zs.max()-0.002
        crest.append(P[s][top].mean(0))
    C=np.array(crest); c=C[:,:2].mean(0); u,sv,vt=np.linalg.svd(C[:,:2]-c); a=vt[0]
    if a[1]>0: a=-a
    print(it, "centre",np.round(c,4),"axis",np.round(a,3),"n",len(C))
h=np.array([a[1],-a[0]]); d=W[:,:2]-c; t=d@a; w=d@h
m=(abs(t)<0.1)&(abs(w)<0.08)&(W[:,2]>0.935); P=W[m]; T=t[m]; Wd=w[m]
print("h =",np.round(h,3))
for t0 in np.arange(-0.09,0.09,0.01):
    s=(T>=t0)&(T<t0+0.01)
    if not s.sum(): continue
    row=[]
    for w0 in np.arange(-0.06,0.07,0.01):
        ss=s&(Wd>=w0)&(Wd<w0+0.01)
        row.append(f"{P[ss,2].max():.3f}" if ss.sum() else "  .  ")
    print(f"t {t0:+.2f}: "+" ".join(row))
print("cols w from -0.06 to +0.06 step 0.01")
EOF

# openrua op 186
cat > dryB3.py <<'EOF'
import math, sys, numpy as np, recover
from recover import Recover
from task import TCP_OFF
recover.GRASP_UP = 0.01
recover.Z_BOTTOM_TO_WAIST = 0.053
waist = [0.236, 0.012, 0.968]; a = [0.065, 0.998]
gam, sgn, pitch = float(sys.argv[1]), int(sys.argv[2]), float(sys.argv[3])
spot = tuple(map(float, sys.argv[4:6])); phi = float(sys.argv[6]); stage = tuple(map(float, sys.argv[7:9]))
live = "--live" in sys.argv
t = Recover(not live); t.stage = stage; t.handle_world = np.array([0.998, -0.065, 0.0])
t.grasp_R(waist, a, math.radians(gam), sgn, math.radians(pitch), close=False)
if live:
    R = t.hand_R(); tip = t.r.fk()[0] + R[:, 2] * TCP_OFF
    want = np.asarray(waist) - recover.GRASP_UP * R[:, 2]
    short = (want - tip) @ R[:, 2]
    print(f"   descend shortfall along z: {short*1000:.1f} mm", flush=True)
    if short > 0.005:
        print("!! descent blocked, backing off"); t.go_R([tip - 0.10 * R[:, 2]], [R], 3.0, "abort: retreat"); sys.exit(1)
    f = t.gripper(0.0); w = f[0] - f[1]
    print("   grasp width %.2f cm" % (w * 100), flush=True)
    if not (0.060 < w < 0.078):
        print("!! unexpected grasp width, releasing"); t.open_full()
        t.go_R([tip - 0.10 * R[:, 2]], [R], 3.0, "abort: retreat"); sys.exit(1)
else:
    t.gripper(0.0)
t.lift(1.10)
t.place_auto([a[0], a[1], 0.0], spot, math.radians(phi))
print("DRY OK" if not live else "LIVE DONE")
if live: print("fingers:", t.r.fingers())
EOF
python3 -u dryB3.py 30 1 -15 0.17 -0.012 30 0.08 -0.06 2>&1 | grep -v "^  ik" | grep -v "^   q=" | head -14

# openrua op 187
timeout 900 python3 -u feas2.py 0.236 0.012 0.968 0.065 0.998 0.01 0.10 2>&1 | grep -v "^  ik" | head -30

# openrua op 188
sed -i 's/recover.Z_BOTTOM_TO_WAIST = 0.053/recover.Z_BOTTOM_TO_WAIST = 0.050/; s/waist = \[0.236, 0.012, 0.968\]/waist = [0.2357, 0.007, 0.968]/' dryB3.py && python3 -u dryB3.py 45 1 0 0.17 -0.012 30 0.08 -0.06 2>&1 | grep -v "^  ik" | grep -v "^   q=" | head -20

# openrua op 189
python3 -u dryB3.py 45 1 0 0.17 -0.012 30 0.08 -0.06 2>&1 | grep -v "^   q=" | tail -12

# openrua op 190
for phi in 0 15 45 60; do echo "== phi $phi"; python3 -u dryB3.py 45 1 0 0.17 -0.012 $phi 0.08 -0.06 2>&1 | grep -v "^   q=" | grep -v "^  ik" | grep -E "DRY OK|IK failed|final hand z"; done

# openrua op 191
python3 -u dryB3.py 45 1 0 0.17 -0.012 45 0.08 -0.06 2>&1 | grep -v "^  ik" | sed -n 1,40p

# openrua op 192
python3 -u dryB3.py 45 1 0 0.17 -0.012 45 0.08 -0.06 --live 2>&1 | tee placeB4.log | grep -v "^  ik" | grep -v "^   q=" | grep -v "creep\]" | tail -40

# openrua op 193
grep -n -A12 "panda_leftfinger\|panda_hand_tcp\|panda_finger_joint1" /workspace/robot.urdf | grep -n "origin\|box\|mesh\|joint name\|link name" | head -30

# openrua op 194
timeout 900 python3 -u feas2.py 0.239 0.132 0.971 0.065 0.998 0.0 0.08 2>&1 | grep -v "^  ik" | head; echo ---; timeout 900 python3 -u feas2.py 0.236 0.10 0.971 0.065 0.998 0.0 0.08 2>&1 | grep -v "^  ik" | head -5

# openrua op 195
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/knob.py", "old_string": "    def place(self, spot, drop):", "new_string": "    def roll(self, crest, dx, press=0.002, step=0.02):\\n        \\"\\"\\"Roll a lying pot (axis along y) toward -x: closed fingertip edge pressed on the crest, dragged -x.\\n        Hand z = (0.707,0,-0.707), fingers along y; the block\'s lowest edge is 1.27 cm below the TCP.\\"\\"\\"\\n        z = np.array([0.707, 0.0, -0.707]); y = np.array([0.0, 1.0, 0.0]); x = np.cross(y, z)\\n        R = np.column_stack([x, y, z])\\n        self.gripper(0.0)\\n        tcp = np.array([crest[0], crest[1], crest[2] + 0.0127 - press])\\n        self.go_R([tcp + [0, 0, 0.08]], [R], 4.0, \\"roll: above\\")\\n        tip = self.go_R([tcp], [R], 3.0, \\"roll: press\\", retry=False)\\n        if not self.dry:\\n            print(f\\"   press: TCP z {tip[2]:.4f} vs cmd {tcp[2]:.4f} (shortfall {(tip[2]-tcp[2])*1000:.1f} mm)\\", flush=True)\\n        n = int(round(abs(dx) / step)); sgn = -1 if dx < 0 else 1\\n        for k in range(1, n + 1):\\n            tgt = tcp + [sgn * k * step, 0, 0]\\n            tip = self.go_R([tgt], [R], 1.5, f\\"roll: drag {k}/{n}\\", retry=False)\\n            if not self.dry:\\n                print(f\\"   TCP {np.round(tip, 4)} (cmd {np.round(tgt, 4)})\\", flush=True)\\n        self.go_R([tip + [0, 0, 0.10]], [R], 3.0, \\"roll: rise\\")\\n        self.open_full()\\n\\n    def place(self, spot, drop):", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 196
python3 - <<'EOF'
import re
s=open("knob.py").read()
s=s.replace('''    elif a[0] == "place":''','''    elif a[0] == "roll":   # roll cx cy cz dx
        t.roll([float(a[1]), float(a[2]), float(a[3])], float(a[4]))
    elif a[0] == "place":''',1)
open("knob.py","w").write(s)
EOF
python3 -u knob.py roll 0.235 -0.003 1.005 -0.06 --dry 2>&1 | grep -v "^  ik"

# openrua op 197
python3 - <<'EOF'
import numpy as np, feas2
z=np.array([0.707,0,-0.707]); y=np.array([0,1.0,0]); x=np.cross(y,z); R=np.column_stack([x,y,z])
for p in [(0.235,-0.003,1.0157),(0.235,-0.003,1.096),(0.178,-0.003,1.072),(0.20,-0.003,1.05),(0.175,-0.003,1.0157)]:
    q=feas2.solve(np.array(p),R)
    print(p, None if q is None else np.round(q,2))
EOF

# openrua op 198
sed -i 's/z = np.array(\[0.707, 0.0, -0.707\]); y = np.array(\[0.0, 1.0, 0.0\]); x = np.cross(y, z)/z = np.array([1.0, 0.0, -1.0]) \/ math.sqrt(2); y = np.array([0.0, 1.0, 0.0]); x = np.cross(y, z)/' knob.py && sed -i 's/self.go_R(\[tcp + \[0, 0, 0.08\]\], \[R\], 4.0, "roll: above")/self.go_R([tcp - 0.08 * z], [R], 4.0, "roll: above")/' knob.py && python3 -u knob.py roll 0.235 -0.003 1.005 -0.06 --dry 2>&1 | grep -v "^  ik"

# openrua op 199
sed -i 's/        self.go_R(\[tip + \[0, 0, 0.10\]\], \[R\], 3.0, "roll: rise")/        if self.dry: tip = tgt\n        self.go_R([tip + [0, 0, 0.10]], [R], 3.0, "roll: rise")/' knob.py && python3 -u knob.py roll 0.235 -0.003 1.005 -0.06 --dry 2>&1 | grep -v "^  ik" | tail -3 && python3 -u knob.py roll 0.235 -0.003 1.005 -0.06 2>&1 | tee rollB.log | grep -v "^  ik" | grep -v "^   q="

# openrua op 200
python3 -u knob.py home 2>&1 | tail -2 && python3 -u scene.py birdview 0.02 2>&1 | tail -15

# openrua op 201
sed -n 1,60p scene.py

# openrua op 202
python3 - <<'EOF'
import numpy as np
P=np.load("birdview_world.npy")
v=np.isfinite(P[...,2])
sel=v&(P[...,0]>0.05)&(P[...,0]<0.35)&(P[...,1]>-0.12)&(P[...,1]<0.25)&(P[...,2]>0.94)
pts=P[sel]
print("n",len(pts))
# height map on 1cm grid
xs=np.arange(0.05,0.35,0.01); ys=np.arange(-0.12,0.25,0.01)
print("     "+" ".join(f"{y*100:4.0f}" for y in ys))
for x in xs:
    row=[]
    for y in ys:
        m=(pts[:,0]>=x)&(pts[:,0]<x+0.01)&(pts[:,1]>=y)&(pts[:,1]<y+0.01)
        row.append(f"{(pts[m,2].max()-0.9)*100:4.1f}" if m.any() else "   .")
    print(f"{x*100:4.0f} "+" ".join(row))
EOF

# openrua op 203
python3 - <<'EOF'
import re
s=open("knob.py").read()
old=s[s.index("    def roll("):s.index("    def place(")]
new='''    def roll(self, crest, dx, press=0.004, step=0.02):
        """Roll a lying pot (axis along y) toward -x: closed fingertip edge lowered onto the crest until
        contact (TCP shortfall), pushed `press` further, then dragged along x in `step` increments.
        Hand z = (0.707,0,-0.707), fingers along y; the block's lowest edge is ~1.27 cm below the TCP."""
        z = np.array([1.0, 0.0, -1.0]) / math.sqrt(2); y = np.array([0.0, 1.0, 0.0]); x = np.cross(y, z)
        R = np.column_stack([x, y, z])
        self.gripper(0.0)
        tcp = np.array([crest[0], crest[1], crest[2] + 0.0127 + 0.012])   # nominal edge 1.2 cm above crest
        self.go_R([tcp - 0.08 * z], [R], 4.0, "roll: above")
        tip = self.go_R([tcp], [R], 3.0, "roll: start", retry=False)
        touched = None
        for k in range(10):                                              # creep down 3 mm at a time
            tcp = tcp - [0, 0, 0.003]
            tip = self.go_R([tcp], [R], 1.0, f"roll: creep {k}", retry=False)
            if self.dry:
                tip = tcp
            short = tip[2] - tcp[2]
            print(f"   creep: TCP z {tip[2]:.4f} vs cmd {tcp[2]:.4f} (shortfall {short*1000:.1f} mm)", flush=True)
            if short > 0.0015:
                touched = tip[2]; break
        if touched is None and not self.dry:
            print("   !! never touched the pot", flush=True)
            self.go_R([tip + [0, 0, 0.10]], [R], 3.0, "roll: rise"); self.open_full(); return False
        tcp = np.array([tcp[0], tcp[1], (touched if touched else tcp[2]) - press])
        tip = self.go_R([tcp], [R], 1.0, "roll: press", retry=False)
        n = int(round(abs(dx) / step)); sgn = -1 if dx < 0 else 1
        for k in range(1, n + 1):
            tgt = tcp + [sgn * k * step, 0, 0]
            tip = self.go_R([tgt], [R], 1.5, f"roll: drag {k}/{n}", retry=False)
            if not self.dry:
                print(f"   TCP {np.round(tip, 4)} (cmd {np.round(tgt, 4)})", flush=True)
        if self.dry: tip = tgt
        self.go_R([tip + [0, 0, 0.10]], [R], 3.0, "roll: rise")
        self.open_full()
        return True

'''
open("knob.py","w").write(s.replace(old,new))
EOF
python3 -u knob.py roll 0.245 -0.003 1.005 -0.07 --dry 2>&1 | grep -E "^\[|fail|Error" | tail -20

# openrua op 204
python3 -u knob.py roll 0.245 -0.003 1.005 -0.07 2>&1 | tee rollB2.log | grep -E "^\[|creep:|TCP \[|!!|touch|fingers" | grep -v "^\[gripper"

# openrua op 205
python3 -u knob.py home 2>&1 | tail -1 && python3 -u scene.py birdview 0.02 >/dev/null 2>&1; cat > hmap.py <<'EOF'
import sys, numpy as np
P=np.load("birdview_world.npy")
x0,x1,y0,y1=[float(v) for v in sys.argv[1:5]]
v=np.isfinite(P[...,2])
sel=v&(P[...,0]>x0)&(P[...,0]<x1)&(P[...,1]>y0)&(P[...,1]<y1)&(P[...,2]>0.94)
pts=P[sel]
xs=np.arange(x0,x1,0.01); ys=np.arange(y0,y1,0.01)
print("     "+" ".join(f"{y*100:4.0f}" for y in ys))
for x in xs:
    row=[]
    for y in ys:
        m=(pts[:,0]>=x)&(pts[:,0]<x+0.01)&(pts[:,1]>=y)&(pts[:,1]<y+0.01)
        row.append(f"{(pts[m,2].max()-0.9)*100:4.1f}" if m.any() else "   .")
    print(f"{x*100:4.0f} "+" ".join(row))
EOF
python3 hmap.py 0.05 0.35 -0.12 0.25

# openrua op 206
python3 tools/perception/cam_snap.py frontview 2>&1 | tail -1; python3 tools/perception/cam_snap.py sideview 2>&1 | tail -1; python3 - <<'EOF'
import cv2
for c in ("frontview","sideview"):
    im=cv2.imread(f"{c}.png"); h,w=im.shape[:2]; print(c,w,h)
    cv2.imwrite(f"{c}_big.png", cv2.resize(im,(w*2,h*2),interpolation=cv2.INTER_CUBIC))
EOF

# openrua op 207
python3 -u scene.py sideview 0.02 >/dev/null 2>&1; python3 - <<'EOF'
import numpy as np
P=np.load("sideview_world.npy")
v=np.isfinite(P[...,2])
for (y0,y1) in [(-0.05,0.0),(0.0,0.05),(0.05,0.10),(0.10,0.14)]:
    sel=v&(P[...,0]>0.15)&(P[...,0]<0.36)&(P[...,1]>y0)&(P[...,1]<y1)&(P[...,2]>0.925)&(P[...,2]<1.10)
    pts=P[sel]
    print(f"\n y in [{y0},{y1}]  n={len(pts)}   (rows z cm above 0.90, cols x cm) — count of points")
    xs=np.arange(0.15,0.36,0.01); zs=np.arange(0.10,0.02,-0.01)
    print("     "+" ".join(f"{x*100:3.0f}" for x in xs))
    for z in zs:
        row=[]
        for x in xs:
            m=(pts[:,0]>=x)&(pts[:,0]<x+0.01)&(pts[:,2]>=z+0.9)&(pts[:,2]<z+0.91)
            row.append(f"{m.sum():3d}" if m.any() else "  .")
        print(f"{z*100:4.0f} "+" ".join(row))
EOF

# openrua op 208
mkdir -p "$(dirname /workspace/push.py)"
cat > /workspace/push.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Push a lying pot sideways with the closed fingertip: approach its flank horizontally at height z,
detect contact (TCP shortfall), then keep pushing to target x while reading the TCP each step.
Usage: python3 push.py  px py pz  x_target [tilt_deg=30] [--dry]
  (px,py,pz): point ~3 cm outside the pot flank on the +x side, at pushing height; pushes toward -x.
"""
import math, sys
import numpy as np
from recover import Recover
from task import TCP_OFF


class Push(Recover):
    def push(self, start, x_target, tilt=math.radians(30), step=0.01):
        # hand z leans toward +x by `tilt` from vertical (wrist stays on the robot side), fingers along y
        z = np.array([math.sin(tilt), 0.0, -math.cos(tilt)]); y = np.array([0.0, 1.0, 0.0]); x = np.cross(y, z)
        R = np.column_stack([x, y, z])
        self.gripper(0.0)
        tcp = np.asarray(start, float)
        self.go_R([tcp + [0, 0, 0.10]], [R], 4.0, "push: above")
        tip = self.go_R([tcp], [R], 3.0, "push: descend", retry=False)
        if not self.dry:
            print(f"   descend: TCP z {tip[2]:.4f} vs cmd {tcp[2]:.4f} (shortfall {(tip[2]-tcp[2])*1000:.1f} mm)", flush=True)
            if tip[2] - tcp[2] > 0.003:
                print("   !! blocked while descending", flush=True)
                self.go_R([tip + [0, 0, 0.10]], [R], 3.0, "push: rise"); self.open_full(); return False
        contact = False
        while tcp[0] - x_target > 1e-6:
            tcp = tcp - [step, 0, 0]
            tip = self.go_R([tcp], [R], 1.0, f"push: x={tcp[0]:.3f}", retry=False)
            if self.dry:
                tip = tcp
            lag = tip[0] - tcp[0]
            print(f"   TCP {np.round(tip, 4)} lag {lag*1000:.1f} mm", flush=True)
            if not contact and lag > 0.003:
                contact = True
                print("   contact", flush=True)
        if not contact and not self.dry:
            print("   !! no contact detected over the whole push", flush=True)
        self.go_R([tip + [0.02, 0, 0]], [R], 1.5, "push: back off")
        self.go_R([tip + [0.02, 0, 0.10]], [R], 3.0, "push: rise")
        self.open_full()
        return contact


if __name__ == "__main__":
    dry = "--dry" in sys.argv
    a = [x for x in sys.argv[1:] if not x.startswith("--")]
    t = Push(dry)
    tilt = math.radians(float(a[4])) if len(a) > 4 else math.radians(30)
    ok = t.push([float(a[0]), float(a[1]), float(a[2])], float(a[3]), tilt)
    print("PUSH", "contact" if ok else "no-contact")
    if not dry:
        print("fingers:", t.r.fingers())
OPENRUA_EOF

# openrua op 209
python3 -u push.py 0.305 0.0 0.968 0.245 30 --dry 2>&1 | grep -E "^\[|fail|Error|q=" | head -30

# openrua op 210
python3 - <<'EOF' 2>&1 | grep -v "^  ik"
import math, numpy as np
from kin import ik
from rob import R_to_quat
from task import HOME, TCP_OFF
for tilt in (30,40,50,60):
    t=math.radians(tilt); z=np.array([math.sin(t),0,-math.cos(t)]); y=np.array([0,1.0,0]); x=np.cross(y,z); R=np.column_stack([x,y,z])
    for tcp in ([0.305,0,0.968],[0.305,0,1.02],[0.29,0,0.968],[0.28,0,0.965]):
        pos=np.array(tcp)-TCP_OFF*z
        q=ik(pos,R_to_quat(R),HOME,w_post=0.03,tries=4)
        print(tilt, tcp, "OK" if q is not None else "--", np.round(q,2) if q is not None else "")
EOF

# openrua op 211
ls /workspace; ls /workspace/tools /workspace/docs 2>/dev/null; find / -name "*.xml" -path "*gripper*" 2>/dev/null | head; find / -iname "*panda*finger*" 2>/dev/null | head

# openrua op 212
python3 - <<'EOF'
import numpy as np, rclpy
from rob import Robot
from task import TCP_OFF
r=Robot(); pos,R=r.fk()[0], r.fk()[1] if False else None
import rob; print([n for n in dir(rob.Robot) if not n.startswith('_')])
EOF

# openrua op 213
grep -n "def fk" -A12 rob.py | head -30

# openrua op 214
python3 - <<'EOF' 2>&1 | grep -v "^\[INFO"
import numpy as np
from rob import Robot
from kin import quat_to_R
from task import TCP_OFF
r=Robot(); pos,quat=r.fk(); R=quat_to_R(quat); tcp=pos+R[:,2]*TCP_OFF
print("hand pos",np.round(pos,4),"tcp",np.round(tcp,4)); print("R\n",np.round(R,3)); print("fingers",r.fingers())
for cam in ("sideview","frontview","birdview"):
    P=np.load(f"{cam}_world.npy"); v=np.isfinite(P[...,2])
    d=P-tcp; loc=d@R   # hand coords
    sel=v&(np.abs(loc[...,0])<0.06)&(np.abs(loc[...,1])<0.08)&(loc[...,2]>-0.07)&(loc[...,2]<0.03)
    L=loc[sel]; print(cam,"n",len(L))
    # fingertip region: hand z > -0.01
    for zlo,zhi in [(-0.07,-0.05),(-0.05,-0.03),(-0.03,-0.01),(-0.01,0.0),(0.0,0.005),(0.005,0.01),(0.01,0.02)]:
        m=(L[:,2]>=zlo)&(L[:,2]<zhi)
        if m.any(): print(f"  z[{zlo:+.3f},{zhi:+.3f}] n={m.sum():4d} x[{L[m,0].min():+.3f},{L[m,0].max():+.3f}] y[{L[m,1].min():+.3f},{L[m,1].max():+.3f}]  |y| min {np.abs(L[m,1]).min():.3f}")
EOF

# openrua op 215
grep -vE "^  ik|^   q=" placeB4.log | head -40; echo ----; head -30 dryB3.py

# openrua op 216
python3 - <<'EOF'
import numpy as np
P=np.load("birdview_world.npy"); v=np.isfinite(P[...,2])
sel=v&(P[...,0]>0.17)&(P[...,0]<0.32)&(P[...,1]>-0.06)&(P[...,1]<0.16)&(P[...,2]>0.935)
pts=P[sel]
print("  y    crest  x@crest  | body x-extent (z>0.975)      | all x-extent | n")
for y in np.arange(-0.055,0.155,0.005):
    m=(pts[:,1]>=y)&(pts[:,1]<y+0.005)
    if not m.any(): continue
    q=pts[m]; i=q[:,2].argmax()
    b=q[q[:,2]>0.975]
    bx=f"[{b[:,0].min():.3f},{b[:,0].max():.3f}] w={100*(b[:,0].max()-b[:,0].min()):.1f} c={0.5*(b[:,0].min()+b[:,0].max()):.4f}" if len(b) else "-"
    print(f"{y:+.3f}  {q[i,2]:.4f}  {q[i,0]:.3f}  | {bx:38s} | [{q[:,0].min():.3f},{q[:,0].max():.3f}] | {m.sum()}")
EOF

# openrua op 217
sed -i 's/^recover.Z_BOTTOM_TO_WAIST = 0.050/recover.Z_BOTTOM_TO_WAIST = 0.057/; s/^waist = \[0.2357, 0.007, 0.968\]/waist = [0.237, 0.012, 0.968]/; s/if not (0.060 < w < 0.078)/if not (0.052 < w < 0.070)/' dryB3.py && grep -n "waist =\|Z_BOTTOM\|0.052" dryB3.py && python3 -u dryB3.py 45 -1 0 0.17 -0.012 45 0.08 -0.06 2>&1 | grep -vE "^  ik|^   q=" | head -40

# openrua op 218
python3 -u dryB3.py 45 -1 0 0.17 -0.012 45 0.08 -0.06 --live 2>&1 | tee placeB5.log | grep -vE "^  ik|^   q=|^  traj done|retrying" 

# openrua op 219
python3 -u knob.py home 2>&1 | tail -1 && python3 -u scene.py birdview 0.02 >/dev/null 2>&1; python3 hmap.py 0.15 0.35 -0.08 0.20

# openrua op 220
python3 - <<'EOF'
import numpy as np, cv2
P=np.load("sideview_world.npy"); im=cv2.imread("sideview.png")
# find pixels of A's knob region: x 0.29-0.34, y 0.12-0.20, z 0.92-0.97
v=np.isfinite(P[...,2]); m=v&(P[...,0]>0.29)&(P[...,0]<0.34)&(P[...,1]>0.12)&(P[...,1]<0.21)&(P[...,2]>0.915)&(P[...,2]<0.975)
ys,xs=np.nonzero(m); print(len(ys), ys.min(),ys.max(),xs.min(),xs.max())
r0,r1,c0,c1=ys.min()-25,ys.max()+25,xs.min()-40,xs.max()+40
crop=im[r0:r1,c0:c1]; cv2.imwrite("knobA_side.png", cv2.resize(crop,None,fx=6,fy=6,interpolation=cv2.INTER_CUBIC))
# also print knob profile: for x bins, z range
pts=P[m]
for x in np.arange(0.29,0.34,0.005):
    q=pts[(pts[:,0]>=x)&(pts[:,0]<x+0.005)]
    if len(q): print(f"x {x:.3f}: z [{q[:,2].min():.3f},{q[:,2].max():.3f}] y[{q[:,1].min():.3f},{q[:,1].max():.3f}] n={len(q)}")
EOF

# openrua op 221
grep -rn -i "upright\|success\|criteri\|on the stove\|goal\|done" README.md docs/*.md machine.yaml tools/README.md | head -30

# openrua op 222
cat > prof.py <<'EOF'
import sys, numpy as np
P=np.load("birdview_world.npy"); v=np.isfinite(P[...,2])
x0,x1,y0,y1=[float(a) for a in sys.argv[1:5]]; zcut=float(sys.argv[5]) if len(sys.argv)>5 else 0.975
axis=sys.argv[6] if len(sys.argv)>6 else "y"   # profile along this axis
sel=v&(P[...,0]>x0)&(P[...,0]<x1)&(P[...,1]>y0)&(P[...,1]<y1)&(P[...,2]>0.935)
pts=P[sel]; i=1 if axis=="y" else 0; j=1-i
lo,hi=(y0,y1) if axis=="y" else (x0,x1)
print(f"  {axis}    crest  {'x' if j==0 else 'y'}@crest | body extent (z>{zcut})           | all extent | n")
for s in np.arange(lo,hi,0.005):
    m=(pts[:,i]>=s)&(pts[:,i]<s+0.005)
    if not m.any(): continue
    q=pts[m]; k=q[:,2].argmax(); b=q[q[:,2]>zcut]
    bx=f"[{b[:,j].min():.3f},{b[:,j].max():.3f}] w={100*(b[:,j].max()-b[:,j].min()):4.1f} c={0.5*(b[:,j].min()+b[:,j].max()):.4f}" if len(b) else "-"
    print(f"{s:+.3f}  {q[k,2]:.4f}  {q[k,j]:.3f}  | {bx:40s} | [{q[:,j].min():.3f},{q[:,j].max():.3f}] | {m.sum()}")
EOF
echo "== B (along y) =="; python3 prof.py 0.17 0.34 -0.08 0.12; echo "== A (along x) =="; python3 prof.py 0.14 0.35 0.125 0.22 0.955 x

# openrua op 223
timeout 900 python3 feas2.py 0.23 0.174 0.939 1 0 0.01 0.08 2>&1 | grep -v "^  ik" | grep -E "gamma=(\+0|-15|\+15) " | head -30

# openrua op 224
timeout 900 python3 feas2.py 0.23 0.174 0.939 1 0 0.01 0.08 2>&1 | grep -v "^  ik" | tail -25

# openrua op 225
sed -n 1,80p feas2.py

# openrua op 226
python3 - <<'EOF' 2>&1 | grep -v "^  ik"
import math, numpy as np
from feas2 import hand_R, solve
from task import TCP_OFF
a=np.array([1.0,0,0])
for x in (0.20,0.21,0.22,0.23):
    for pitch in (55,60,65,70,75,80):
        for sgn in (1,-1):
            R=hand_R(a,0.0,sgn,math.radians(pitch)); z=R[:,2]
            p=np.array([x,0.174,0.939]); tcp=p-0.01*z; pos=tcp-TCP_OFF*z
            q=solve(pos,R); qa=solve(pos-0.08*z,R) if q is not None else None
            print(f"x={x} pitch={pitch} sgn={sgn:+d}: grasp {'OK' if q is not None else '--'} above {'OK' if qa is not None else '--'} hand z={np.round(z,2)} " + (f"q={np.round(q,2)}" if q is not None else ""))
EOF

# openrua op 227
timeout 1200 python3 feas2.py 0.255 0.0 0.968 0.065 0.998 0.01 0.08 2>&1 | grep -v "^  ik" | tail -12

# openrua op 228
python3 - <<'EOF'
s=open("knob.py").read()
old=s[s.index("        tcp = np.array([tcp[0], tcp[1], (touched if touched else tcp[2]) - press])"):s.index("    def place(")]
new='''        z_touch = touched if touched else tcp[2]
        tcp = np.array([tcp[0], tcp[1], z_touch - press])        # soft arm: deep command = real normal force
        tip = self.go_R([tcp], [R], 2.0, "roll: press", retry=False)
        if not self.dry:
            print(f"   press: TCP z {tip[2]:.4f} (touch {z_touch:.4f}, cmd {tcp[2]:.4f})", flush=True)
        n = int(round(abs(dx) / step)); sgn = -1 if dx < 0 else 1
        for k in range(1, n + 1):
            tgt = tcp + [sgn * k * step, 0, 0]
            tip = self.go_R([tgt], [R], 1.0, f"roll: drag {k}/{n}", retry=False)
            if self.dry:
                tip = tgt
            else:
                print(f"   TCP {np.round(tip, 4)} (cmd {np.round(tgt, 4)}) lag x {(tip[0]-tgt[0])*1000:+.1f} mm, z above touch {(tip[2]-z_touch)*1000:+.1f} mm", flush=True)
                if tip[2] < z_touch - 0.012:
                    print("   !! finger dropped past the crest, aborting drag", flush=True)
                    break
        self.go_R([tip + [0, 0, 0.10]], [R], 3.0, "roll: rise")
        self.open_full()
        return True

'''
s=s.replace(old,new)
s=s.replace("def roll(self, crest, dx, press=0.004, step=0.02):","def roll(self, crest, dx, press=0.03, step=0.01):")
open("knob.py","w").write(s)
EOF
python3 -u knob.py roll 0.262 -0.02 1.005 -0.08 --dry 2>&1 | grep -E "^\[|Error|fail" | tail -5

# openrua op 229
python3 - <<'EOF' 2>&1 | grep -v "^  ik"
import math, numpy as np
from feas2 import solve
from task import TCP_OFF
for tilt in (45,55):
    t=math.radians(tilt); z=np.array([math.sin(t),0,-math.cos(t)]); y=np.array([0,1.0,0]); x=np.cross(y,z); R=np.column_stack([x,y,z])
    for X in (0.245,0.25,0.255,0.26,0.265):
        res=[]
        for Z in (1.03,1.01,0.975):
            q=solve(np.array([X,-0.02,Z])-TCP_OFF*z,R)
            res.append("OK" if q is not None else "--")
        print(tilt,X,res)
EOF

# openrua op 230
python3 - <<'EOF' 2>&1 | grep -v "^  ik"
import math, numpy as np
from feas2 import solve
from task import TCP_OFF
for tilt in (45,55,65):
    t=math.radians(tilt); z=np.array([math.sin(t),0,-math.cos(t)]); y=np.array([0,1.0,0]); x=np.cross(y,z); R=np.column_stack([x,y,z])
    for X in (0.2425,0.245,0.2475,0.25,0.2525,0.255):
        deepest=None
        for Z in np.arange(1.005,0.969,-0.005):
            q=solve(np.array([X,-0.02,Z])-TCP_OFF*z,R,tries=1)
            if q is None: break
            deepest=Z
        print(f"tilt {tilt} x {X:.4f}: deepest z {deepest}")
EOF

# openrua op 231
python3 - <<'EOF'
s=open("knob.py").read()
s=s.replace('''    elif a[0] == "roll":   # roll cx cy cz dx
        t.roll([float(a[1]), float(a[2]), float(a[3])], float(a[4]))''','''    elif a[0] == "roll":   # roll cx cy cz dx [--press=m]
        press = float(next((x[8:] for x in sys.argv if x.startswith("--press=")), 0.03))
        t.roll([float(a[1]), float(a[2]), float(a[3])], float(a[4]), press=press)''')
open("knob.py","w").write(s)
EOF
python3 -u knob.py roll 0.2475 -0.02 1.003 -0.0575 --press=0.025 --dry 2>&1 | grep -E "^\[|Error|fail" | tail -4

# openrua op 232
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/knob.py", "old_string": "        tcp = np.array([tcp[0], tcp[1], z_touch - press])        # soft arm: deep command = real normal force\\n        tip = self.go_R([tcp], [R], 2.0, \\"roll: press\\", retry=False)", "new_string": "        while True:                                              # deepest reachable press command\\n            tcp = np.array([tcp[0], tcp[1], z_touch - press])    # soft arm: deep command = real normal force\\n            try:\\n                tip = self.go_R([tcp], [R], 2.0, f\\"roll: press {press*1000:.0f} mm\\", retry=False); break\\n            except RuntimeError:\\n                press -= 0.005\\n                if press < 0.005:\\n                    print(\\"   !! press point unreachable\\", flush=True); self.open_full(); return False", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 233
python3 -u knob.py roll 0.2475 -0.02 1.003 -0.0575 --press=0.025 --dry 2>&1 | grep -E "^\[|Error|fail|!!" | tail -12

# openrua op 234
python3 -u knob.py roll 0.2475 -0.02 1.003 -0.0575 --press=0.025 2>&1 | tee rollB3.log | grep -vE "^\[" | tail -30

# openrua op 235
python3 -u knob.py home 2>&1 | tail -1; python3 -u scene.py birdview 0.02 >/dev/null 2>&1; python3 prof.py 0.17 0.34 -0.08 0.12 2>&1 | head -50

# openrua op 236
cd /workspace; grep -o '"command": *"[^"]*knob.py pick[^"]*"' /claude-config/projects/-workspace/7ac4390e-9f64-4e6b-af95-bcb87e83fe1a.jsonl | head; grep -o 'pinch width [0-9.]* cm' /claude-config/projects/-workspace/7ac4390e-9f64-4e6b-af95-bcb87e83fe1a.jsonl | head; ls *.log

# openrua op 237
cd /workspace; grep -vE "^\[|q=|traj done" knobB.log | tail -25; echo ----; grep -vE "^\[|q=|traj done" dragB.log | tail -15

# openrua op 238
cd /workspace; grep -n "def \|Z_BOTTOM_TO_WAIST\|handle_world\|stage\|GRASP_UP\|PLATE_TOP" recover.py | head -60

# openrua op 239
cd /workspace; python3 - <<'EOF'
import numpy as np
P=np.load("birdview_world.npy")
m=(P[:,0]>0.08)&(P[:,0]<0.28)&(P[:,1]>-0.07)&(P[:,1]<0.13)&(P[:,2]<0.94)
Q=P[m]
xs=np.arange(0.08,0.28,0.01); ys=np.arange(-0.07,0.13,0.01)
print("      "+" ".join(f"{y*100:+4.0f}" for y in ys))
for x in xs:
    row=[]
    for y in ys:
        s=Q[(abs(Q[:,0]-x-0.005)<0.005)&(abs(Q[:,1]-y-0.005)<0.005)]
        row.append(f"{np.median(s[:,2])*1000-900:4.0f}" if len(s)>3 else "   .")
    print(f"{x*100:5.1f} "+" ".join(row))
EOF

# openrua op 240
cd /workspace; python3 - <<'EOF'
import numpy as np
P=np.load("birdview_world.npy"); print(P.shape); P=P.reshape(-1,P.shape[-1]) if P.ndim==3 else P
P=P[np.isfinite(P).all(1)]
m=(P[:,0]>0.08)&(P[:,0]<0.28)&(P[:,1]>-0.07)&(P[:,1]<0.13)&(P[:,2]<0.94)
Q=P[m]
xs=np.arange(0.08,0.28,0.01); ys=np.arange(-0.07,0.13,0.01)
print("      "+" ".join(f"{y*100:+4.0f}" for y in ys))
for x in xs:
    row=[]
    for y in ys:
        s=Q[(abs(Q[:,0]-x-0.005)<0.005)&(abs(Q[:,1]-y-0.005)<0.005)]
        row.append(f"{np.median(s[:,2])*1000-900:4.0f}" if len(s)>3 else "   .")
    print(f"{x*100:5.1f} "+" ".join(row))
EOF

# openrua op 241
mkdir -p "$(dirname /workspace/dryA3.py)"
cat > /workspace/dryA3.py <<'OPENRUA_EOF'
"""Pot A (lying on the table, axis along +x, lid at +x): fork/pinch the boiler flanks, lift, place upright.
usage: dryA3.py gam sgn pitch sx sy phi stagex stagey [--live] [--wx=0.215] [--lift=1.16]"""
import math, sys, numpy as np, recover
from recover import Recover
from task import TCP_OFF
recover.GRASP_UP = 0.01
recover.Z_BOTTOM_TO_WAIST = 0.045          # bottom face x~0.170 -> grasp x 0.215
opt = lambda k, d: float(next((x[len(k) + 3:] for x in sys.argv if x.startswith("--" + k + "=")), d))
wx = opt("wx", 0.215); zlift = opt("lift", 1.16)
waist = [wx, 0.173, 0.939]; a = [1.0, 0.0]
gam, sgn, pitch = float(sys.argv[1]), int(sys.argv[2]), float(sys.argv[3])
spot = tuple(map(float, sys.argv[4:6])); phi = float(sys.argv[6]); stage = tuple(map(float, sys.argv[7:9]))
live = "--live" in sys.argv
t = Recover(not live); t.stage = stage; t.handle_world = np.array([0.0, -1.0, 0.0])
t.grasp_R(waist, a, math.radians(gam), sgn, math.radians(pitch), close=False)
if live:
    R = t.hand_R(); tip = t.r.fk()[0] + R[:, 2] * TCP_OFF
    want = np.asarray(waist) - recover.GRASP_UP * R[:, 2]
    d = want - tip
    short = d @ R[:, 2]; lat = np.linalg.norm(d - short * R[:, 2])
    print(f"   descend shortfall along z: {short*1000:.1f} mm, lateral {lat*1000:.1f} mm", flush=True)
    if short > 0.005 or lat > 0.006:
        print("!! descent deflected, backing off"); t.go_R([tip - 0.10 * R[:, 2]], [R], 3.0, "abort: retreat"); sys.exit(1)
    f = t.gripper(0.0); w = f[0] - f[1]
    print("   grasp width %.2f cm" % (w * 100), flush=True)
    if not (0.060 < w < 0.078):
        print("!! unexpected grasp width, releasing"); t.open_full()
        t.go_R([tip - 0.10 * R[:, 2]], [R], 3.0, "abort: retreat"); sys.exit(1)
else:
    t.gripper(0.0)
t.lift(zlift)
if live:
    f = t.r.fingers(); print("   width after lift %.2f cm" % ((f[0] - f[1]) * 100), flush=True)
    if f[0] - f[1] < 0.055:
        print("!! pot lost during lift"); t.open_full(); sys.exit(1)
t.place_auto([a[0], a[1], 0.0], spot, math.radians(phi))
print("DRY OK" if not live else "LIVE DONE")
if live: print("fingers:", t.r.fingers())
OPENRUA_EOF

# openrua op 242
cd /workspace; for s in 1 -1; do echo "=== sgn $s"; python3 -u dryA3.py 0 $s 60 0.17 0.025 45 0.09 0.10 2>&1 | grep -vE "^\[|traj done" | grep -E "hand|corner|IK|Error|theta|handle|final|t=|DRY|!!" ; done

# openrua op 243
cd /workspace; for s in 1 -1; do echo "=== sgn $s"; python3 -u dryA3.py 0 $s 0 0.17 0.025 45 0.09 0.10 --wx=0.085 2>&1 | grep -vE "^\[|traj done" | grep -E "hand|corner|IK|Error|theta|handle|final|t=|DRY|!!" ; done
