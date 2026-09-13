#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la && find . -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
cat /workspace/tools/action/fjt_send.py; echo ----; cat /workspace/tools/action/gripper_cmd.py; echo ----; cat /workspace/tools/action/ik_move.py

# openrua op 3
ros2 topic list && echo ---- && ros2 action list && echo ---- && ros2 service list | head -40 && echo ---- && ros2 node list

# openrua op 4
ros2 topic echo /joint_states --once 2>&1 | head -40; for c in agentview frontview birdview sideview robot0_robotview; do python3 tools/perception/cam_snap.py $c & done; wait

# openrua op 5
timeout 20 ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id|x:|y:|z:|w:" | head -80

# openrua op 6
cat > tfdump.py <<'EOF'
import rclpy, yaml
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
rclpy.init(); node = rclpy.create_node("tfdump")
got = {}
qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
def cb(m, k):
    for t in m.transforms:
        got[(k, t.header.frame_id, t.child_frame_id)] = t.transform
node.create_subscription(TFMessage, "/tf_static", lambda m: cb(m, "static"), qos)
node.create_subscription(TFMessage, "/tf", lambda m: cb(m, "dyn"), 10)
import time
t0 = time.time()
while time.time() - t0 < 3: rclpy.spin_once(node, timeout_sec=0.2)
for k, t in sorted(got.items()):
    print(k, f"t=({t.translation.x:.3f},{t.translation.y:.3f},{t.translation.z:.3f}) q=({t.rotation.x:.3f},{t.rotation.y:.3f},{t.rotation.z:.3f},{t.rotation.w:.3f})")
EOF
python3 tfdump.py

# openrua op 7
cat > cloud.py <<'EOF'
"""Dump a world-frame point cloud (HxWx3) + color for a camera to <cam>_cloud.npz"""
import sys, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage
from cv_bridge import CvBridge
cam = sys.argv[1]
rclpy.init(); node = rclpy.create_node("cloud")
got = {}
node.create_subscription(Image, f"/{cam}/depth/image_raw", lambda m: got.setdefault("d", m), 1)
node.create_subscription(Image, f"/{cam}/color/image_raw", lambda m: got.setdefault("c", m), 1)
node.create_subscription(CameraInfo, f"/{cam}/color/camera_info", lambda m: got.setdefault("i", m), 1)
def tfcb(m):
    for t in m.transforms:
        if t.child_frame_id == f"{cam}_optical_frame": got["tf"] = t.transform
node.create_subscription(TFMessage, "/tf", tfcb, 10)
while not all(k in got for k in "d c i tf".split()): rclpy.spin_once(node, timeout_sec=0.2)
br = CvBridge()
depth = br.imgmsg_to_cv2(got["d"], "passthrough").astype(np.float64)
color = br.imgmsg_to_cv2(got["c"], "bgr8")
k = got["i"].k; fx, fy, cx, cy = k[0], k[4], k[2], k[5]
H, W = depth.shape
u, v = np.meshgrid(np.arange(W), np.arange(H))
pc = np.stack([(u - cx) * depth / fx, (v - cy) * depth / fy, depth], -1)
t = got["tf"]; q = t.rotation; x, y, z, w = q.x, q.y, q.z, q.w
R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
              [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
              [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
T = np.array([t.translation.x, t.translation.y, t.translation.z])
pw = pc @ R.T + T
np.savez(f"{cam}_cloud.npz", pw=pw, color=color, depth=depth)
print(cam, pw.shape, "depth range", np.nanmin(depth), np.nanmax(depth))
rclpy.shutdown()
EOF
python3 cloud.py birdview & python3 cloud.py agentview & python3 cloud.py robot0_robotview & wait

# openrua op 8
cat > analyze.py <<'EOF'
import numpy as np
d = np.load("birdview_cloud.npz"); pw = d["pw"]; col = d["color"]
z = pw[..., 2]
# table height: mode of z in the region
print("table z candidates:", np.percentile(z[np.isfinite(z)], [5, 25, 50, 75, 95]))
# Print a coarse height map (world x along rows, y along cols)
xs, ys = pw[...,0], pw[...,1]
# grid over the table
for xg in np.arange(-0.5, 0.6, 0.05):
    row = ""
    for yg in np.arange(-0.6, 0.65, 0.05):
        m = (np.abs(xs-xg)<0.025)&(np.abs(ys-yg)<0.025)
        if m.sum()==0: row += "   . "; continue
        row += f"{np.nanmax(z[m]):5.2f}"
    print(f"x={xg:5.2f} {row}")
print("cols y from -0.6 to 0.6 step 0.05")
EOF
python3 analyze.py

# openrua op 9
cat > analyze2.py <<'EOF'
import numpy as np, cv2
for cam in ["agentview", "robot0_robotview"]:
    d = np.load(f"{cam}_cloud.npz"); pw = d["pw"]; col = d["color"]
    x, y, z = pw[...,0], pw[...,1], pw[...,2]
    # objects above table, in the central region (exclude cabinet y<-0.15 and shelf y>0.15)
    m = (z > 0.915) & (z < 1.05) & (x > -0.3) & (x < 0.3) & (y > -0.17) & (y < 0.17)
    print(cam, "candidate pts:", m.sum())
    if m.sum():
        ys, xs = np.where(m)
        print("  pixel bbox u:", xs.min(), xs.max(), "v:", ys.min(), ys.max())
        print("  world x:", x[m].min(), x[m].max(), " y:", y[m].min(), y[m].max(), " z:", z[m].min(), z[m].max())
        print("  centroid:", x[m].mean(), y[m].mean(), z[m].mean())
    # bottle: tall thin
    mb = (z > 1.05) & (z < 1.3) & (x > -0.3) & (x < 0.3) & (y > -0.3) & (y < 0.0)
    if mb.sum():
        print("  bottle pts", mb.sum(), "x:", x[mb].min(), x[mb].max(), "y:", y[mb].min(), y[mb].max(), "ztop", z[mb].max())
EOF
python3 analyze2.py

# openrua op 10
cat > analyze3.py <<'EOF'
import numpy as np
d = np.load("robot0_robotview_cloud.npz"); pw = d["pw"]; col = d["color"]
x, y, z = pw[...,0], pw[...,1], pw[...,2]
# bowl region in robot0_robotview pixels
sub = np.zeros_like(z, bool); sub[60:160, 305:445] = True
m = sub & (z > 0.905)
print("bowl pts", m.sum())
print(" x:", x[m].min(), x[m].max(), " y:", y[m].min(), y[m].max(), " z:", z[m].min(), z[m].max())
# rim = highest points
rim = m & (z > z[m].max() - 0.01)
print(" rim x:", x[rim].min(), x[rim].max(), " y:", y[rim].min(), y[rim].max(), "n", rim.sum())
print(" rim center:", x[rim].mean(), y[rim].mean(), " top z:", z[m].max())
# bottle region: pixels ~ u 235-295, v 70-265
sub = np.zeros_like(z, bool); sub[65:270, 230:300] = True
mb = sub & (z > 0.92)
print("bottle pts", mb.sum(), " x:", x[mb].min(), x[mb].max(), " y:", y[mb].min(), y[mb].max(), " ztop", z[mb].max())
top = mb & (z > z[mb].max()-0.03)
print(" bottle cap center:", x[top].mean(), y[top].mean())
# drawer: the open bottom drawer, pixels u 60-250, v 90-300
sub = np.zeros_like(z, bool); sub[85:300, 55:250] = True
md = sub & (z > 0.905)
print("drawer pts", md.sum(), " x:", x[md].min(), x[md].max(), " y:", y[md].min(), y[md].max(), " z:", z[md].min(), z[md].max())
# histogram of drawer z
h, e = np.histogram(z[md], bins=20)
for hh, ee in zip(h, e): print(f"  z>{ee:.3f}: {hh}")
EOF
python3 analyze3.py

# openrua op 11
cat > analyze4.py <<'EOF'
import numpy as np
for cam, (v0,v1,u0,u1) in {"agentview": (185,245,305,395), "birdview": (240,290,305,370)}.items():
    d = np.load(f"{cam}_cloud.npz"); pw = d["pw"]; col = d["color"]
    x, y, z = pw[...,0], pw[...,1], pw[...,2]
    sub = np.zeros_like(z, bool); sub[v0:v1, u0:u1] = True
    m = sub & (z > 0.905) & (z < 1.25) & (y > -0.02) & (y < 0.13) & (x > -0.25) & (x < -0.05)
    print(cam, "bowl pts", m.sum())
    if m.sum():
        print(" x:", x[m].min(), x[m].max(), " y:", y[m].min(), y[m].max(), " z:", z[m].min(), z[m].max())
        h, e = np.histogram(z[m], bins=12)
        for hh, ee in zip(h, e): print(f"  z>{ee:.3f}: {hh}")
        rim = m & (z > z[m].max() - 0.01)
        print(" rim center:", x[rim].mean(), y[rim].mean(), "x range", x[rim].min(), x[rim].max(), "y range", y[rim].min(), y[rim].max())
EOF
python3 analyze4.py

# openrua op 12
cat > analyze5.py <<'EOF'
import numpy as np
d = np.load("robot0_robotview_cloud.npz"); pw = d["pw"]; col = d["color"]
x, y, z = pw[...,0], pw[...,1], pw[...,2]
cx, cy = -0.150, 0.051
r = np.hypot(x-cx, y-cy)
# column through the bowl center in the image: find pixel closest to center
for v in range(60, 175, 5):
    row = []
    for u in range(300, 460, 10):
        row.append(f"{z[v,u]:.3f}")
    print(v, " ".join(row))
print("u from 300 to 450 step 10")
EOF
python3 analyze5.py

# openrua op 13
python3 cloud.py sideview && cat > analyze6.py <<'EOF'
import numpy as np
d = np.load("sideview_cloud.npz"); pw = d["pw"]; col = d["color"]
x, y, z = pw[...,0], pw[...,1], pw[...,2]
ok = np.isfinite(z)
# drawer region: x -0.25..0.05, y -0.25..0.0, z 0.905..1.15
m = ok & (x>-0.26)&(x<0.06)&(y>-0.26)&(y<0.0)&(z>0.905)&(z<1.15)
print("drawer+bottle pts", m.sum())
# slice by y to find the front face: histogram of y for points with z in 0.93..1.0 and x in -0.2..-0.05 (away from bottle)
mm = m & (x>-0.2)&(x<-0.06)&(z>0.93)&(z<1.0)
h,e = np.histogram(y[mm], bins=25)
for hh,ee in zip(h,e): print(f"  y>{ee:.3f}: {hh}")
# top of drawer front
mf = m & (x>-0.2)&(x<-0.06)&(y>-0.07)&(y<-0.02)
print("front face x range", x[mf].min(), x[mf].max(), "z range", z[mf].min(), z[mf].max(), "y range", y[mf].min(), y[mf].max())
# bottle: cylinder near (0.01,-0.044)
mb = ok & (x>-0.05)&(x<0.08)&(y>-0.12)&(y<0.02)&(z>0.905)&(z<1.4)
print("bottle pts", mb.sum(), "x", x[mb].min(), x[mb].max(), "y", y[mb].min(), y[mb].max(), "z", z[mb].min(), z[mb].max())
for zl in np.arange(0.9,1.35,0.05):
    s = mb&(z>zl)&(z<zl+0.05)
    if s.sum(): print(f"  z {zl:.2f}: x {x[s].min():.3f}..{x[s].max():.3f} y {y[s].min():.3f}..{y[s].max():.3f} n={s.sum()}")
EOF
python3 analyze6.py

# openrua op 14
cat > analyze7.py <<'EOF'
import numpy as np
d = np.load("robot0_robotview_cloud.npz"); pw = d["pw"]
x, y, z = pw[...,0], pw[...,1], pw[...,2]
ok = np.isfinite(z)
xs = np.arange(-0.30, 0.10, 0.02); ys = np.arange(-0.44, 0.02, 0.02)
print("     y:" + " ".join(f"{yy:5.2f}" for yy in ys))
for xg in xs:
    row = []
    for yg in ys:
        m = ok&(np.abs(x-xg)<0.01)&(np.abs(y-yg)<0.01)
        row.append(f"{z[m].max():5.3f}" if m.sum() else "  .  ")
    print(f"x={xg:5.2f} " + " ".join(row))
EOF
python3 analyze7.py

# openrua op 15
timeout 120 python3 tools/action/gripper_cmd.py 0.04; ros2 topic echo /joint_states --once 2>&1 | grep -A9 "^position" | tail -2; python3 cloud.py robot0_robotview && python3 analyze5.py | head -22

# openrua op 16
cat > measure.py <<'EOF'
import numpy as np, sys
cam = sys.argv[1] if len(sys.argv)>1 else "robot0_robotview"
d = np.load(f"{cam}_cloud.npz"); pw = d["pw"]
x, y, z = pw[...,0], pw[...,1], pw[...,2]
ok = np.isfinite(z)
# bowl: above table in central area, excluding drawer (y<-0.02) and shelf(y>0.15)
m = ok&(z>0.905)&(z<1.0)&(x>-0.3)&(x<0.1)&(y>-0.01)&(y<0.15)
print("bowl pts", m.sum(), "x", x[m].min(), x[m].max(), "y", y[m].min(), y[m].max(), "z", z[m].min(), z[m].max())
rim = m&(z>z[m].max()-0.008)
print(" rim center", x[rim].mean(), y[rim].mean(), "rim x", x[rim].min(), x[rim].max(), "rim y", y[rim].min(), y[rim].max())
# bottle
mb = ok&(z>0.905)&(x>-0.05)&(x<0.08)&(y>-0.12)&(y<0.0)
print("bottle pts", mb.sum(), "x", x[mb].min(), x[mb].max(), "y", y[mb].min(), y[mb].max(), "ztop", z[mb].max())
for zl in [0.92, 1.0, 1.1, 1.2]:
    s = mb&(z>zl)&(z<zl+0.05)
    if s.sum(): print(f"  z {zl:.2f}: x {x[s].mean():.3f} y {y[s].mean():.3f} n={s.sum()}")
# drawer front face & floor
mf = ok&(x>-0.19)&(x<-0.07)&(y>-0.10)&(y<0.0)&(z>0.93)&(z<1.0)
h,e = np.histogram(y[mf], bins=20)
print("drawer front y-hist:", [(round(ee,3),hh) for hh,ee in zip(h,e) if hh>0])
fl = ok&(x>-0.18)&(x<-0.04)&(y>-0.19)&(y<-0.09)&(z>0.905)&(z<0.96)
print("floor z", np.median(z[fl]), "n", fl.sum())
EOF
python3 measure.py

# openrua op 17
python3 tools/perception/cam_snap.py robot0_robotview && python3 tools/perception/cam_snap.py agentview

# openrua op 18
cat > measure2.py <<'EOF'
import numpy as np
d = np.load("robot0_robotview_cloud.npz"); pw = d["pw"]; col=d["color"]
x, y, z = pw[...,0], pw[...,1], pw[...,2]
ok = np.isfinite(z)
sub = np.zeros_like(ok); sub[115:210, 310:435] = True
m = ok&sub&(z>0.905)
# exclude the bottle cap: dark/brown; bowl is grey-ish. Use x<0.0 or y>0.03
m2 = m & ((x < -0.0) | (y > 0.03))
print("bowl pts", m2.sum(), "x", x[m2].min(), x[m2].max(), "y", y[m2].min(), y[m2].max(), "z", z[m2].min(), z[m2].max())
rim = m2&(z>z[m2].max()-0.006)
print(" rim center", x[rim].mean(), y[rim].mean(), "rim x", x[rim].min(), x[rim].max(), "rim y", y[rim].min(), y[rim].max(), "n", rim.sum())
# bottle: region below the bowl in image
sub = np.zeros_like(ok); sub[160:330, 295:370] = True
mb = ok&sub&(z>0.905)
print("bottle pts", mb.sum(), "x", x[mb].min(), x[mb].max(), "y", y[mb].min(), y[mb].max(), "ztop", z[mb].max())
for xl in np.arange(-0.06, 0.26, 0.04):
    s = mb&(x>xl)&(x<xl+0.04)
    if s.sum(): print(f"  x {xl:.2f}: y {y[s].min():.3f}..{y[s].max():.3f} ztop {z[s].max():.3f} n={s.sum()}")
EOF
python3 measure2.py

# openrua op 19
cat > hmap.py <<'EOF'
import numpy as np, sys
cam, x0, x1, y0, y1 = sys.argv[1], *map(float, sys.argv[2:6])
d = np.load(f"{cam}_cloud.npz"); pw = d["pw"]
x, y, z = pw[...,0], pw[...,1], pw[...,2]
ok = np.isfinite(z)
xs = np.arange(x0, x1, 0.02); ys = np.arange(y0, y1, 0.02)
print("     y:" + " ".join(f"{yy:5.2f}" for yy in ys))
for xg in xs:
    row = []
    for yg in ys:
        m = ok&(np.abs(x-xg)<0.01)&(np.abs(y-yg)<0.01)
        row.append(f"{z[m].max():5.3f}" if m.sum() else "  .  ")
    print(f"x={xg:5.2f} " + " ".join(row))
EOF
python3 hmap.py robot0_robotview -0.26 0.32 -0.16 0.14

# openrua op 20
python3 cloud.py agentview && python3 hmap.py agentview -0.10 0.20 -0.10 0.10 && python3 -c "
import numpy as np
d=np.load('agentview_cloud.npz'); pw=d['pw']; x,y,z=pw[...,0],pw[...,1],pw[...,2]
m=np.isfinite(z)&(z>0.905)&(x>-0.06)&(x<0.2)&(y>-0.1)&(y<0.1)
print('bottle pts',m.sum(),'ztop',z[m].max())
for zl in np.arange(0.9,1.3,0.04):
    s=m&(z>zl)&(z<zl+0.04)
    if s.sum(): print(f'  z {zl:.2f}: x {x[s].min():.3f}..{x[s].max():.3f} (mean {x[s].mean():.3f}) y {y[s].min():.3f}..{y[s].max():.3f} (mean {y[s].mean():.3f}) n={s.sum()}')
"

# openrua op 21
mkdir -p "$(dirname /workspace/robot.py)"
cat > /workspace/robot.py <<'OPENRUA_EOF'
"""Small helper lib: joint state, FK, IK, trajectories, gripper. World frame."""
import time
import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]
BASE = np.array([-0.660, 0.0, 0.912])  # panda_link0 in world (from TF)
TCP_OFF = M["hand"]["tcp_offset_m"]

# hand pointing down, fingers along world X
Q_DOWN_FX = (0.7071068, 0.7071068, 0.0, 0.0)
# hand pointing down, fingers along world Y
Q_DOWN_FY = (1.0, 0.0, 0.0, 0.0)


def quat_to_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def yaw_quat(q, yaw):
    """Rotate quaternion q by yaw (rad) about world Z."""
    c, s = np.cos(yaw / 2), np.sin(yaw / 2)
    w1, x1, y1, z1 = c, 0.0, 0.0, s
    x2, y2, z2, w2 = q
    w = w1 * w2 - (x1 * x2 + y1 * y2 + z1 * z2)
    x = w1 * x2 + w2 * x1 + (y1 * z2 - z1 * y2)
    y = w1 * y2 + w2 * y1 + (z1 * x2 - x1 * z2)
    z = w1 * z2 + w2 * z1 + (x1 * y2 - y1 * x2)
    return (x, y, z, w)


class Robot:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("robot_lib")
        self._js = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.ik_cli = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        assert self.fjt.wait_for_server(10), "no FJT server"
        assert self.grip.wait_for_server(10), "no gripper server"
        assert self.fk_cli.wait_for_service(10), "no FK"
        assert self.ik_cli.wait_for_service(10), "no IK"

    def _on_js(self, m):
        self._js = m

    def spin(self, t=0.05):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self, fresh=True):
        if fresh:
            self._js = None
        while self._js is None:
            self.spin(0.1)
        d = dict(zip(self._js.name, self._js.position))
        return d

    def arm_q(self):
        d = self.joints()
        return [d[j] for j in ARM]

    def fingers(self):
        d = self.joints()
        return d.get("panda_finger_joint1"), d.get("panda_finger_joint2")

    # ---- kinematics ----
    def fk(self, q=None, link="panda_hand"):
        """Returns (pos_world, quat) of link for arm config q (default current)."""
        q = q if q is not None else self.arm_q()
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        assert res is not None and res.error_code.val == 1, f"FK failed {res and res.error_code.val}"
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE
        return pos, (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)

    def tcp(self, q=None):
        pos, quat = self.fk(q)
        return pos + TCP_OFF * quat_to_R(quat)[:, 2], quat

    def ik(self, pos_world, quat, seed=None, tcp=True, timeout=1.0):
        """IK for hand (or TCP if tcp=True) at world pose. Returns list of 7 or None."""
        pos = np.array(pos_world, float)
        if tcp:
            pos = pos - TCP_OFF * quat_to_R(quat)[:, 2]
        pos = pos - BASE
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pos)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        seed = seed if seed is not None else self.arm_q()
        req.ik_request.robot_state.joint_state.name = list(ARM)
        req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
        req.ik_request.timeout.sec = int(timeout)
        req.ik_request.timeout.nanosec = int((timeout % 1) * 1e9)
        req.ik_request.avoid_collisions = False
        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in ARM]

    # ---- motion ----
    def move_joints(self, waypoints, times):
        """waypoints: list of 7-vectors; times: cumulative seconds per waypoint."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        for q, t in zip(waypoints, times):
            pt = JointTrajectoryPoint(positions=[float(v) for v in q])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        assert gh is not None and gh.accepted, "goal rejected"
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        q = self.arm_q()
        err = float(np.max(np.abs(np.array(q) - np.array(waypoints[-1]))))
        print(f"  traj done code={code} max_joint_err={err:.4f}")
        return code, err

    def move_tcp(self, pos, quat, secs=3.0, seed=None):
        q = self.ik(pos, quat, seed=seed)
        if q is None:
            raise RuntimeError(f"IK failed for {pos}")
        return self.move_joints([q], [secs])

    def move_tcp_line(self, p0, p1, quat, n=5, secs=3.0):
        """Straight TCP line p0->p1 via n IK waypoints (seeded consecutively)."""
        seed = self.arm_q()
        wps, ts = [], []
        for i in range(1, n + 1):
            p = np.array(p0) + (np.array(p1) - np.array(p0)) * i / n
            q = self.ik(p, quat, seed=seed)
            if q is None:
                raise RuntimeError(f"IK failed at waypoint {i}: {p}")
            seed = q
            wps.append(q)
            ts.append(secs * i / n)
        return self.move_joints(wps, ts)

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP.get("max_effort", 30.0))
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=300)
        r = res.result().result
        f = self.fingers()
        print(f"  gripper reached={r.reached_goal} stalled={r.stalled} fingers={f}")
        return f
OPENRUA_EOF

# openrua op 22
timeout 120 python3 -u -c "
from robot import *
r = Robot()
q = r.arm_q(); print('q', [round(v,3) for v in q])
pos, quat = r.fk(); print('hand world', pos.round(4), 'quat', [round(v,3) for v in quat])
print('R cols (hand x,y,z in world):'); print(quat_to_R(quat).round(3))
tcp, _ = r.tcp(); print('tcp world', tcp.round(4))
print('fingers', r.fingers())
"

# openrua op 23
timeout 120 python3 -u -c "
from robot import *
from moveit_msgs.srv import GetPositionFK
r = Robot()
q = r.arm_q()
for frame in ['', 'panda_link0', 'world']:
    for link in ['panda_link0','panda_link1','panda_hand']:
        req = GetPositionFK.Request(); req.header.frame_id = frame
        req.fk_link_names=[link]; req.robot_state.joint_state.name=list(ARM); req.robot_state.joint_state.position=q
        fut = r.fk_cli.call_async(req); rclpy.spin_until_future_complete(r.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val!=1: print(frame, link, 'ERR', res and res.error_code.val); continue
        p = res.pose_stamped[0]
        print(repr(frame), link, 'in', repr(p.header.frame_id), round(p.pose.position.x,4), round(p.pose.position.y,4), round(p.pose.position.z,4))
"


# openrua op 24
timeout 200 python3 -u -c "
from robot import *
import numpy as np
r = Robot()
q = r.arm_q()
quat = (1.0, 0.0, -0.028, 0.0)
# raw IK request without BASE adjust
def ik_raw(pos, quat, seed):
    req = GetPositionIK.Request()
    req.ik_request.group_name = 'panda_arm'; req.ik_request.pose_stamped.header.frame_id = ''
    p = req.ik_request.pose_stamped.pose
    p.position.x, p.position.y, p.position.z = map(float,pos)
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float,quat)
    req.ik_request.robot_state.joint_state.name=list(ARM); req.ik_request.robot_state.joint_state.position=[float(v) for v in seed]
    req.ik_request.timeout.sec=1; req.ik_request.avoid_collisions=False
    fut = r.ik_cli.call_async(req); rclpy.spin_until_future_complete(r.node, fut, timeout_sec=60)
    res = fut.result()
    if res is None or res.error_code.val!=1: return None, (res and res.error_code.val)
    sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position)); return [sol[j] for j in ARM], 1
for label,pos in [('world',(-0.203,0,1.2696)),('link0',(0.457,0,0.3576))]:
    sol, code = ik_raw(pos, quat, q)
    print(label, code, sol and np.round(np.array(sol)-np.array(q),3))
"


# openrua op 25
python3 - <<'EOF'
import re
s = open("robot.py").read()
s = s.replace("BASE = np.array([-0.660, 0.0, 0.912])  # panda_link0 in world (from TF)",
              "BASE = np.array([0.0, 0.0, 0.0])  # FK/IK with empty frame_id are already in WORLD (verified)")
open("robot.py","w").write(s)
EOF
timeout 200 python3 -u -c "
from robot import *
import numpy as np
r = Robot()
q = r.arm_q()
pos, quat = r.fk(); print('hand', pos.round(4), np.round(quat,4))
sol = r.ik(pos, quat, tcp=False)
print('ik sol', np.round(sol,3)); print('cur   ', np.round(q,3))
p2, q2 = r.fk(sol); print('fk(sol)', p2.round(4), np.round(q2,4))
print('R(sol)'); print(quat_to_R(q2).round(3))
"

# openrua op 26
python3 - <<'EOF'
s = open("robot.py").read()
s = s.replace('''def yaw_quat(q, yaw):''', '''def qmul(a, b):
    """Hamilton product a*b, quats as (x, y, z, w)."""
    x1, y1, z1, w1 = a
    x2, y2, z2, w2 = b
    return (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)


# IK tip link is panda_link8; panda_hand = link8 * rotz(-45deg). So
# link8 = hand * rotz(+45deg) (right-multiply, local z).
Q_HAND_TO_LINK8 = (0.0, 0.0, 0.3826834, 0.9238795)


def hand_to_link8(q_hand):
    return qmul(q_hand, Q_HAND_TO_LINK8)


def yaw_quat(q, yaw):''')
s = s.replace('''        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        seed = seed if seed is not None else self.arm_q()''',
'''        q8 = hand_to_link8(quat)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q8)
        seed = seed if seed is not None else self.arm_q()''')
open("robot.py","w").write(s)
EOF
timeout 200 python3 -u -c "
from robot import *
import numpy as np
r = Robot()
q = r.arm_q()
pos, quat = r.fk()
sol = r.ik(pos, quat, tcp=False)
print('ik sol', np.round(sol,3)); print('cur   ', np.round(q,3))
p2, q2 = r.fk(sol); print('fk(sol)', p2.round(4), np.round(q2,4), 'requested', np.round(quat,4))
# Also test a target: TCP above bowl, fingers along X
tgt = (-0.213, 0.056, 1.05)
sol2 = r.ik(tgt, Q_DOWN_FX)
print('above-bowl sol', sol2 and np.round(sol2,3))
if sol2:
    tp, tq = r.tcp(sol2); print(' tcp fk', tp.round(4)); print(quat_to_R(tq).round(3))
"

# openrua op 27
timeout 60 python3 -c "
import rclpy
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy
rclpy.init(); n = rclpy.create_node('urdf')
got=[]
n.create_subscription(String, '/robot_description', got.append, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
import time; t=time.time()
while not got and time.time()-t<10: rclpy.spin_once(n, timeout_sec=0.5)
open('robot.urdf','w').write(got[0].data if got else '')
print(len(got[0].data) if got else 'none')
" && python3 - <<'EOF'
import re
u = open("robot.urdf").read()
for link in ["panda_hand", "panda_leftfinger", "panda_rightfinger", "panda_link7", "panda_link8"]:
    m = re.search(rf'<link name="{link}">(.*?)</link>', u, re.S)
    if not m: print(link, "not found"); continue
    body = m.group(1)
    col = re.search(r'<collision>(.*?)</collision>', body, re.S)
    print(link, "collision:", re.sub(r'\s+', ' ', col.group(1))[:400] if col else None)
# joints of interest
for j in ["panda_hand_joint", "panda_finger_joint1", "panda_joint8", "panda_hand_tcp_joint"]:
    m = re.search(rf'<joint name="{j}"(.*?)</joint>', u, re.S)
    print(j, re.sub(r'\s+', ' ', m.group(1))[:300] if m else "not found")
EOF

# openrua op 28
python3 - <<'EOF'
import numpy as np, struct
def stl_bounds(path):
    data = open(path,'rb').read()
    if data[:5] == b'solid' and b'facet' in data[:300]:
        import re
        v = np.array([list(map(float, m.groups())) for m in re.finditer(rb'vertex\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)', data)])
    else:
        n = struct.unpack_from('<I', data, 80)[0]
        arr = np.frombuffer(data[84:84+n*50], dtype=np.dtype([('n','<3f'),('v','<9f'),('a','<u2')]))
        v = arr['v'].reshape(-1,3)
    return v.min(0), v.max(0)
base = "/opt/ros/jazzy/share/moveit_resources_panda_description/meshes/collision/"
for f in ["hand.stl", "finger.stl", "link7.stl"]:
    lo, hi = stl_bounds(base+f); print(f, "min", lo.round(4), "max", hi.round(4))
EOF

# openrua op 29
find / -name "hand.stl" -o -name "finger.stl" 2>/dev/null | head; find / -iname "*panda*" -maxdepth 6 -type d 2>/dev/null | head

# openrua op 30
python3 - <<'EOF'
import numpy as np
d = np.load("robot0_robotview_cloud.npz"); pw = d["pw"]
x, y, z = pw[...,0], pw[...,1], pw[...,2]
ok = np.isfinite(z)
# side walls: z>0.97, y in -0.19..-0.10
m = ok&(z>0.97)&(z<1.0)&(y>-0.19)&(y<-0.10)&(x>-0.3)&(x<0.1)
h,e = np.histogram(x[m], bins=np.arange(-0.26,0.06,0.005))
print("side-wall top x-hist:"); print([(round(ee,3),hh) for hh,ee in zip(h,e) if hh>0])
# front wall: z>0.97, x in -0.18..-0.05
m = ok&(z>0.97)&(z<1.0)&(x>-0.18)&(x<-0.05)&(y>-0.12)&(y<0.0)
h,e = np.histogram(y[m], bins=np.arange(-0.12,0.0,0.005))
print("front-wall top y-hist:"); print([(round(ee,3),hh) for hh,ee in zip(h,e) if hh>0])
# handle (z 0.94-0.965, y>-0.06)
m = ok&(z>0.935)&(z<0.97)&(x>-0.2)&(x<0.0)&(y>-0.06)&(y<0.0)
print("bottom handle: x", x[m].min(), x[m].max(), "y", y[m].min(), y[m].max(), "z", z[m].min(), z[m].max())
# upper handles: y in -0.21..-0.17, z>1.0
m = ok&(z>1.0)&(y>-0.215)&(y<-0.16)&(x>-0.25)&(x<0.05)
print("upper handles: n", m.sum(), "x", x[m].min(), x[m].max(), "y", y[m].min(), y[m].max())
h,e = np.histogram(z[m], bins=np.arange(1.0,1.14,0.01)); print([(round(ee,3),hh) for hh,ee in zip(h,e) if hh>0])
# cabinet face y
m = ok&(z>1.0)&(z<1.135)&(y>-0.26)&(y<-0.15)&(x>-0.2)&(x<-0.05)
h,e = np.histogram(y[m], bins=np.arange(-0.26,-0.15,0.005)); print("cabinet face/handles y-hist:", [(round(ee,3),hh) for hh,ee in zip(h,e) if hh>0])
EOF

# openrua op 31
mkdir -p "$(dirname /workspace/task.py)"
cat > /workspace/task.py <<'OPENRUA_EOF'
"""Stages of: put the black bowl in the bottom drawer, close it.
Usage: python3 -u task.py <stage> [stage...]   (stages run in order)"""
import sys
import numpy as np
from robot import *

# ---- scene measurements (world frame) ----
BOWL = np.array([-0.158, 0.056])       # bowl center xy
BOWL_RIM_Z = 0.951
BOWL_R = 0.055
GRASP_DY = 0.02                        # grasp the rim 2 cm toward +y from the -x point
GRASP_DX = -np.sqrt(BOWL_R**2 - GRASP_DY**2)   # ~ -0.0512
GRASP_Z = BOWL_RIM_Z - 0.018           # fingertips 1.8 cm below the rim
PLACE = np.array([-0.095, -0.140])     # bowl center xy inside the drawer
DRAWER_FLOOR_Z = 0.924
BOWL_BOTTOM_BELOW_TCP = (BOWL_RIM_Z - 0.90) - 0.018   # 0.033
RELEASE_Z = DRAWER_FLOOR_Z + BOWL_BOTTOM_BELOW_TCP + 0.008
SAFE_Z = 1.12
Q = Q_DOWN_FX  # pointing down, fingers along world X

pick_xy = BOWL + [GRASP_DX, GRASP_DY]
place_xy = PLACE + [GRASP_DX, GRASP_DY]


def report(r, label):
    p, q = r.tcp()
    print(f"[{label}] TCP world = {p.round(4)}  fingers = {np.round(r.fingers(), 4)}")
    return p


def approach(r):
    print("approach above pick point", pick_xy)
    r.move_tcp([*pick_xy, 1.03], Q, secs=4.0)
    report(r, "above bowl")


def descend(r):
    p = report(r, "before descend")
    r.move_tcp_line(p, [*pick_xy, GRASP_Z], Q, n=3, secs=2.5)
    report(r, "at grasp depth")


def grasp(r):
    r.gripper(0.0)
    report(r, "closed")


def lift(r):
    p = report(r, "before lift")
    r.move_tcp_line(p, [p[0], p[1], SAFE_Z], Q, n=3, secs=2.5)
    report(r, "lifted")


def transfer(r):
    print("transfer above place point", place_xy)
    r.move_tcp([*place_xy, SAFE_Z], Q, secs=4.0)
    report(r, "above drawer")


def lower(r):
    p = report(r, "before lower")
    r.move_tcp_line(p, [*place_xy, RELEASE_Z], Q, n=4, secs=3.0)
    report(r, "at release height")


def release(r):
    r.gripper(0.04)
    report(r, "opened")


def retreat(r):
    p = report(r, "before retreat")
    r.move_tcp_line(p, [p[0], p[1], SAFE_Z], Q, n=3, secs=2.5)
    report(r, "retreated")


STAGES = dict(approach=approach, descend=descend, grasp=grasp, lift=lift,
              transfer=transfer, lower=lower, release=release, retreat=retreat)

if __name__ == "__main__":
    r = Robot()
    print("pick_xy", pick_xy.round(4), "place_xy", place_xy.round(4),
          "GRASP_Z", round(GRASP_Z, 4), "RELEASE_Z", round(RELEASE_Z, 4))
    for s in sys.argv[1:]:
        STAGES[s](r)
OPENRUA_EOF

# openrua op 32
timeout 900 python3 -u task.py approach descend 2>&1 | tee stage1.log

# openrua op 33
python3 tools/perception/cam_snap.py robot0_eye_in_hand & python3 tools/perception/cam_snap.py robot0_robotview & python3 tools/perception/cam_snap.py sideview & wait; ros2 topic echo /joint_states --once 2>&1 | grep -A9 "^position"

# openrua op 34
timeout 600 python3 -u task.py grasp 2>&1 | tee stage2.log

# openrua op 35
timeout 600 python3 -u task.py lift 2>&1 | tee stage3.log && python3 tools/perception/cam_snap.py robot0_robotview && python3 tools/perception/cam_snap.py sideview

# openrua op 36
python3 cloud.py agentview >/dev/null && python3 cloud.py sideview > /dev/null && python3 - <<'EOF'
import numpy as np
for cam in ["agentview", "sideview"]:
    d = np.load(f"{cam}_cloud.npz"); pw = d["pw"]
    x, y, z = pw[...,0], pw[...,1], pw[...,2]
    ok = np.isfinite(z)
    # bowl hanging: near TCP (-0.209, 0.076, 1.118), bowl center expected ~(-0.158, 0.056), z 1.05-1.14
    m = ok&(z>1.03)&(z<1.135)&(x>-0.30)&(x<-0.03)&(y>-0.05)&(y<0.17)
    print(cam, "pts", m.sum())
    if m.sum():
        print("  x", x[m].min().round(3), x[m].max().round(3), " y", y[m].min().round(3), y[m].max().round(3), " z", z[m].min().round(3), z[m].max().round(3))
        for zl in np.arange(1.03, 1.14, 0.02):
            s = m&(z>zl)&(z<zl+0.02)
            if s.sum(): print(f"   z {zl:.2f}: x {x[s].min():.3f}..{x[s].max():.3f} y {y[s].min():.3f}..{y[s].max():.3f} n={s.sum()}")
EOF

# openrua op 37
python3 - <<'EOF'
s = open("task.py").read()
s = s.replace("place_xy = PLACE + [GRASP_DX, GRASP_DY]", "place_xy = PLACE - [0.051, 0.002]   # measured bowl-center offset from TCP after lift")
s = s.replace("BOWL_BOTTOM_BELOW_TCP = (BOWL_RIM_Z - 0.90) - 0.018   # 0.033", "BOWL_BOTTOM_BELOW_TCP = 0.037   # measured after lift")
open("task.py","w").write(s)
EOF
timeout 900 python3 -u task.py transfer 2>&1 | tee stage4.log && python3 tools/perception/cam_snap.py robot0_robotview

# openrua op 38
python3 cloud.py agentview >/dev/null && python3 - <<'EOF'
import numpy as np
d = np.load("agentview_cloud.npz"); pw = d["pw"]
x, y, z = pw[...,0], pw[...,1], pw[...,2]
ok = np.isfinite(z)
m = ok&(z>1.07)&(z<1.14)&(x>-0.25)&(x<0.0)&(y>-0.25)&(y<-0.03)
print("pts", m.sum(), "x", x[m].min().round(3), x[m].max().round(3), " y", y[m].min().round(3), y[m].max().round(3), " z", z[m].min().round(3), z[m].max().round(3))
rim = m&(z>1.10)
print("rim-level center", x[rim].mean().round(3), y[rim].mean().round(3), "x", x[rim].min().round(3), x[rim].max().round(3), "y", y[rim].min().round(3), y[rim].max().round(3))
low = m&(z<1.095)
print("bottom-level", x[low].min().round(3), x[low].max().round(3), y[low].min().round(3), y[low].max().round(3), "zmin", z[m].min().round(3))
EOF

# openrua op 39
python3 - <<'EOF'
import numpy as np
d = np.load("agentview_cloud.npz"); pw = d["pw"]
x, y, z = pw[...,0], pw[...,1], pw[...,2]
ok = np.isfinite(z)
m = ok&(z>1.075)&(z<1.126)&(x>-0.19)&(x<-0.01)&(y>-0.20)&(y<-0.06)
print("pts", m.sum())
for zl in np.arange(1.075, 1.13, 0.01):
    s = m&(z>zl)&(z<zl+0.01)
    if s.sum(): print(f"   z {zl:.3f}: x {x[s].min():.3f}..{x[s].max():.3f} y {y[s].min():.3f}..{y[s].max():.3f} n={s.sum()}")
EOF

# openrua op 40
timeout 900 python3 -u task.py lower 2>&1 | tee stage5.log && python3 tools/perception/cam_snap.py sideview && python3 tools/perception/cam_snap.py agentview

# openrua op 41
timeout 900 python3 -u task.py release retreat 2>&1 | tee stage6.log && python3 tools/perception/cam_snap.py agentview && python3 tools/perception/cam_snap.py robot0_robotview

# openrua op 42
python3 cloud.py robot0_robotview >/dev/null && python3 cloud.py agentview >/dev/null && python3 - <<'EOF'
import numpy as np
for cam in ["robot0_robotview", "agentview"]:
    d = np.load(f"{cam}_cloud.npz"); pw = d["pw"]
    x, y, z = pw[...,0], pw[...,1], pw[...,2]
    ok = np.isfinite(z)
    # bowl in drawer: z 0.93..0.98, inside drawer box, excluding walls (thin) - use rim: highest ring
    m = ok&(z>0.955)&(z<0.982)&(x>-0.20)&(x<-0.01)&(y>-0.215)&(y<-0.08)
    if m.sum():
        print(cam, "bowl-in-drawer pts", m.sum(), "x", x[m].min().round(3), x[m].max().round(3), "y", y[m].min().round(3), y[m].max().round(3), "ztop", z[m].max().round(3), "center", x[m].mean().round(3), y[m].mean().round(3))
    # drawer front face & handle after place
    mf = ok&(x>-0.19)&(x<-0.07)&(y>-0.10)&(y<0.02)&(z>0.93)&(z<1.0)
    h,e = np.histogram(y[mf], bins=np.arange(-0.10,0.02,0.005))
    print("  front/handle y-hist:", [(round(ee,3),hh) for hh,ee in zip(h,e) if hh>50])
EOF
python3 - <<'EOF'
import numpy as np, sys
sys.path.insert(0, "/workspace")
from robot import quat_to_R
phi = np.radians(15)
c, s = np.cos(phi), np.sin(phi)
R = np.array([[0, -1, 0], [-s, 0, -c], [c, 0, -s]])  # columns: hand x, y, z in world
# quaternion from R (Shepperd)
def R2q(R):
    t = np.trace(R)
    if t > 0:
        w = np.sqrt(1+t)/2; x=(R[2,1]-R[1,2])/(4*w); y=(R[0,2]-R[2,0])/(4*w); z=(R[1,0]-R[0,1])/(4*w)
    else:
        i = np.argmax(np.diag(R))
        if i==0: x=np.sqrt(1+R[0,0]-R[1,1]-R[2,2])/2; w=(R[2,1]-R[1,2])/(4*x); y=(R[0,1]+R[1,0])/(4*x); z=(R[0,2]+R[2,0])/(4*x)
        elif i==1: y=np.sqrt(1+R[1,1]-R[0,0]-R[2,2])/2; w=(R[0,2]-R[2,0])/(4*y); x=(R[0,1]+R[1,0])/(4*y); z=(R[1,2]+R[2,1])/(4*y)
        else: z=np.sqrt(1+R[2,2]-R[0,0]-R[1,1])/2; w=(R[1,0]-R[0,1])/(4*z); x=(R[0,2]+R[2,0])/(4*z); y=(R[1,2]+R[2,1])/(4*z)
    return (x,y,z,w)
q = R2q(R); print("q push =", np.round(q,6)); print(quat_to_R(q).round(3)); print("det", np.linalg.det(R).round(3))
EOF

# openrua op 43
timeout 300 python3 -u - <<'EOF'
import numpy as np
from robot import *
r = Robot()
QP = (0.560986, -0.560986, 0.430459, 0.430459)
seed = r.arm_q()
tests = [("wp_down", (-0.15, 0.10, 1.10), Q_DOWN_FX), ("push_start_hi", (-0.15, 0.10, 1.00), QP),
         ("push_start", (-0.15, 0.10, 0.947), QP), ("push_mid", (-0.15, -0.05, 0.947), QP), ("push_end", (-0.15, -0.183, 0.947), QP)]
for name, p, q in tests:
    sol = r.ik(p, q, seed=seed)
    if sol is None: print(name, "IK FAILED"); continue
    tp, tq = r.tcp(sol); hp, _ = r.fk(sol)
    print(f"{name}: q={np.round(sol,2)}  tcp={tp.round(3)} hand_origin={hp.round(3)} lims_ok={all(lo<=v<=hi for v,(lo,hi) in zip(sol, FJT['limits_rad']))}")
    seed = sol
EOF

# openrua op 44
timeout 300 python3 -u - <<'EOF'
import numpy as np
from robot import *
r = Robot()
QP = (0.560986, -0.560986, 0.430459, 0.430459)
best = None
for q1 in [0.0, 0.3, 0.6, 0.9]:
  for q7 in [-2.0, -1.0, 0.0, 1.0, 2.0]:
    for q4 in [-1.5, -2.0, -2.5]:
        seed = [q1, 0.5, 0.0, q4, 0.0, 2.0, q7]
        sol = r.ik((-0.15, 0.10, 0.947), QP, seed=seed, timeout=0.3)
        if sol is None: continue
        margin = min(min(v-lo, hi-v) for v,(lo,hi) in zip(sol, FJT['limits_rad']))
        print(f"seed q1={q1} q4={q4} q7={q7}: sol={np.round(sol,2)} min_margin={margin:.2f}")
        if best is None or margin > best[0]: best = (margin, sol)
print("BEST", best and (round(best[0],2), np.round(best[1],3)))
EOF

# openrua op 45
timeout 300 python3 -u - <<'EOF'
import numpy as np
from robot import *
from moveit_msgs.srv import GetPositionFK
r = Robot()
QP = (0.560986, -0.560986, 0.430459, 0.430459)
def fk_links(q):
    req = GetPositionFK.Request(); req.header.frame_id = ""
    req.fk_link_names = ["panda_link3","panda_link4","panda_link5","panda_link6","panda_link7","panda_hand"]
    req.robot_state.joint_state.name=list(ARM); req.robot_state.joint_state.position=[float(v) for v in q]
    fut = r.fk_cli.call_async(req); rclpy.spin_until_future_complete(r.node, fut, timeout_sec=30)
    return {n: np.array([p.pose.position.x,p.pose.position.y,p.pose.position.z]).round(3) for n,p in zip(req.fk_link_names, fut.result().pose_stamped)}
cands = {"A": [-0.077, 1.105, 0.734, -1.719, -1.442, 1.119, -2.635], "B": [0.31, 0.92, 0.26, -1.74, -1.13, 1.43, -2.71], "C": [0.77, 0.91, -0.27, -1.77, -0.81, 1.69, -2.86]}
for name, seed in cands.items():
    print("==", name)
    s = seed
    for y in [0.10, -0.05, -0.183]:
        sol = r.ik((-0.15, y, 0.947), QP, seed=s, timeout=0.5)
        if sol is None: print("  y", y, "IK fail"); break
        s = sol
        margin = min(min(v-lo, hi-v) for v,(lo,hi) in zip(sol, FJT['limits_rad']))
        L = fk_links(sol)
        print(f"  y={y}: q={np.round(sol,2)} margin={margin:.2f} link4(elbow)={L['panda_link4']} link5={L['panda_link5']} link6={L['panda_link6']} link7={L['panda_link7']}")
EOF

# openrua op 46
python3 cloud.py birdview >/dev/null && python3 - <<'EOF'
import numpy as np
d = np.load("birdview_cloud.npz"); pw = d["pw"]
x, y, z = pw[...,0], pw[...,1], pw[...,2]
ok = np.isfinite(z)
m = ok&(z>0.93)&(y>0.12)&(y<0.6)&(x>-0.4)&(x<0.4)
print("stand pts", m.sum(), "x", x[m].min().round(3), x[m].max().round(3), "y", y[m].min().round(3), y[m].max().round(3), "ztop", z[m].max().round(3))
for yl in np.arange(0.12, 0.45, 0.03):
    s = m&(y>yl)&(y<yl+0.03)
    if s.sum(): print(f"  y {yl:.2f}: x {x[s].min():.3f}..{x[s].max():.3f} zmax {z[s].max():.3f} n={s.sum()}")
EOF

# openrua op 47
mkdir -p "$(dirname /workspace/push.py)"
cat > /workspace/push.py <<'OPENRUA_EOF'
"""Close the drawer: horizontal push on the bottom-drawer handle.
Usage: python3 -u push.py check|run [stage...]"""
import sys
import numpy as np
from robot import *
from moveit_msgs.srv import GetPositionFK

PSI, PHI = np.radians(20), np.radians(15)   # yaw toward +x of the approach, pitch down
PUSH_X = -0.14
PUSH_Z = 0.947                               # handle bar centre height
Y_PRE = 0.0                                  # fingertips 3 cm in front of the handle face (-0.03)
Y_END = -0.183                               # handle face at -0.19 when the drawer is closed


def push_quat(psi=PSI, phi=PHI):
    d = np.array([np.sin(psi) * np.cos(phi), -np.cos(psi) * np.cos(phi), -np.sin(phi)])  # hand z
    xh = np.array([np.sin(psi) * np.sin(phi), -np.cos(psi) * np.sin(phi), np.cos(phi)])  # hand x (up-ish)
    yh = np.cross(d, xh)
    R = np.column_stack([xh, yh, d])
    t = np.trace(R)
    w = np.sqrt(1 + t) / 2
    return (float((R[2, 1] - R[1, 2]) / (4 * w)), float((R[0, 2] - R[2, 0]) / (4 * w)),
            float((R[1, 0] - R[0, 1]) / (4 * w)), float(w))


QP = push_quat()
SEED_B = [0.31, 0.92, 0.26, -1.74, -1.13, 1.43, -2.71]

# ---- obstacles as axis-aligned boxes (world) ----
OBST = {
    "table": (-1.0, 1.0, -1.0, 1.0, 0.0, 0.90),
    "stand": (-0.134, 0.136, 0.19, 0.36, 0.90, 1.25),
    "bottle": (-0.025, 0.05, -0.03, 0.05, 0.90, 1.065),
    "cabinet": (-0.30, 0.02, -0.60, -0.19, 0.90, 1.135),   # body + protruding handles
    "drawer": (-0.23, 0.01, -0.225, -0.03, 0.90, 0.985),   # open drawer + its handle
}
LINK_R = {"panda_link5": 0.06, "panda_link6": 0.06, "panda_link7": 0.055}


def link_pts(r, q):
    """Sample points of the arm's distal links + hand + fingers in world."""
    req = GetPositionFK.Request()
    req.header.frame_id = ""
    req.fk_link_names = list(LINK_R) + ["panda_hand"]
    req.robot_state.joint_state.name = list(ARM)
    req.robot_state.joint_state.position = [float(v) for v in q]
    fut = r.fk_cli.call_async(req)
    rclpy.spin_until_future_complete(r.node, fut, timeout_sec=30)
    pts = []
    for name, ps in zip(req.fk_link_names, fut.result().pose_stamped):
        p = np.array([ps.pose.position.x, ps.pose.position.y, ps.pose.position.z])
        if name in LINK_R:
            rad = LINK_R[name]
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    for dz in (-1, 0, 1):
                        v = np.array([dx, dy, dz], float)
                        if np.linalg.norm(v) > 0:
                            v = v / np.linalg.norm(v) * rad
                        pts.append((name, p + v))
        else:
            o = ps.pose.orientation
            R = quat_to_R((o.x, o.y, o.z, o.w))
            for hx in (-0.035, 0.035):
                for hy in (-0.10, 0.10):
                    for hz in (0.0, 0.066):
                        pts.append(("palm", p + R @ [hx, hy, hz]))
            for hy in (-0.045, 0.045):          # fingers (open-ish envelope)
                for hz in (0.058, 0.115):
                    pts.append(("finger", p + R @ [0.0, hy, hz]))
            pts.append(("camera", p + R @ [0.06, 0.0, 0.0]))
    return pts


def check_path(r, q0, q1, steps=12, ignore=()):
    """Linear joint interpolation; report obstacle hits."""
    hits = []
    for i in range(steps + 1):
        q = np.array(q0) + (np.array(q1) - np.array(q0)) * i / steps
        for name, p in link_pts(r, q):
            for ob, (x0, x1, y0, y1, z0, z1) in OBST.items():
                if ob in ignore:
                    continue
                if x0 <= p[0] <= x1 and y0 <= p[1] <= y1 and z0 <= p[2] <= z1:
                    hits.append((i, name, ob, p.round(3)))
    return hits


def plan(r):
    """IK for all waypoints; returns list of (label, q, secs)."""
    q_now = r.arm_q()
    wps = []
    q = r.ik((PUSH_X, Y_PRE, 1.12), Q_DOWN_FX, seed=q_now)
    assert q is not None, "IK wp1"
    wps.append(("over_prepush_down", q, 4.0))
    q = r.ik((PUSH_X, Y_PRE, 1.10), QP, seed=SEED_B)
    assert q is not None, "IK wp2"
    wps.append(("reorient_horizontal", q, 4.0))
    seed = q
    q = r.ik((PUSH_X, Y_PRE, PUSH_Z), QP, seed=seed)
    assert q is not None, "IK wp3"
    wps.append(("descend_prepush", q, 3.0))
    seed = q
    line = []
    for y in np.linspace(Y_PRE, Y_END, 7)[1:]:
        q = r.ik((PUSH_X, y, PUSH_Z), QP, seed=seed)
        assert q is not None, f"IK push y={y}"
        seed = q
        line.append(q)
    wps.append(("push", line, 6.0))
    return wps


def do_check(r, wps):
    q_prev = r.arm_q()
    ok = True
    for label, q, _ in wps:
        qs = q if isinstance(q, list) and isinstance(q[0], (list, np.ndarray)) else [q]
        for k, qq in enumerate(qs):
            ign = ("drawer",) if label == "push" else ()
            hits = check_path(r, q_prev, qq, ignore=ign)
            margin = min(min(v - lo, hi - v) for v, (lo, hi) in zip(qq, FJT["limits_rad"]))
            tp, _ = r.tcp(qq)
            print(f"{label}[{k}]: tcp={tp.round(3)} q={np.round(qq, 2)} jmargin={margin:.2f} hits={len(hits)}")
            for h in hits[:8]:
                print("    ", h)
            ok &= not hits
            q_prev = qq
    return ok


if __name__ == "__main__":
    r = Robot()
    print("QP", np.round(QP, 4))
    print(quat_to_R(QP).round(3))
    wps = plan(r)
    mode = sys.argv[1]
    if mode == "check":
        print("ALL CLEAR" if do_check(r, wps) else "COLLISION RISK")
    elif mode == "run":
        wanted = sys.argv[2:] or [w[0] for w in wps]
        for label, q, secs in wps:
            if label not in wanted:
                continue
            print("==", label)
            if label == "push":
                n = len(q)
                r.move_joints(q, [secs * (i + 1) / n for i in range(n)])
            else:
                r.move_joints([q], [secs])
            tp, _ = r.tcp()
            print(f"   TCP now {tp.round(4)} fingers {np.round(r.fingers(), 4)}")
OPENRUA_EOF

# openrua op 48
timeout 600 python3 -u push.py check 2>&1 | grep -v BrokenPipe | tail -60

# openrua op 49
timeout 300 python3 - <<'EOF' 2>&1 | grep -v BrokenPipe
import numpy as np
from robot import *
from push import *
r = Robot()
for z in (1.10, 1.06, 1.03, 1.00):
    for seed in (SEED_B, [0.16,0.74,0.21,-2.12,-1.57,1.7,-2.66]):
        q = r.ik((PUSH_X, Y_PRE, z), QP, seed=seed)
        if q is None: print(z, "none"); continue
        m = min(min(v-lo, hi-v) for v,(lo,hi) in zip(q, FJT["limits_rad"]))
        print(f"z={z} seed={seed[-1]} q={np.round(q,2)} margin={m:.2f}")
EOF

# openrua op 50
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/push.py", "old_string": "    q = r.ik((PUSH_X, Y_PRE, 1.10), QP, seed=SEED_B)", "new_string": "    q = r.ik((PUSH_X, Y_PRE, 1.00), QP, seed=SEED_B)", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 51
timeout 600 python3 -u push.py check 2>&1 | grep -v BrokenPipe | tail -30

# openrua op 52
timeout 600 python3 -u - <<'EOF' 2>&1 | grep -v BrokenPipe
from robot import *
r = Robot()
print("fingers before", r.fingers())
r.gripper(0.0)
p,_ = r.tcp(); print("TCP", p.round(4))
EOF
timeout 600 python3 -u push.py run over_prepush_down reorient_horizontal 2>&1 | grep -v BrokenPipe | tail -8

# openrua op 53
timeout 300 python3 -u - <<'EOF' 2>&1 | grep -v BrokenPipe
import numpy as np
from robot import *
from push import *
r = Robot()
tgt = r.ik((PUSH_X, Y_PRE, 1.00), QP, seed=SEED_B)
for i in range(3):
    q = r.arm_q()
    p, qu = r.tcp()
    print("q   ", np.round(q,3), "TCP", p.round(4))
print("tgt ", np.round(tgt,3))
print("diff", np.round(np.array(q)-np.array(tgt),3))
print("R now\n", quat_to_R(qu).round(3))
EOF
ls tools/perception/ | head; 

# openrua op 54
timeout 60 ros2 topic echo --once /franka_robot_state_broadcaster/external_wrench 2>&1 | grep -v BrokenPipe | head -20; for c in robot0_robotview agentview sideview; do timeout 60 python3 tools/perception/cam_snap.py $c 2>&1 | grep -v BrokenPipe | tail -1; done; ls -t *.png | head -4

# openrua op 55
timeout 600 python3 -u - <<'EOF' 2>&1 | grep -v BrokenPipe
import numpy as np
from robot import *
from push import *
r = Robot()
tgt = r.ik((PUSH_X, Y_PRE, 1.00), QP, seed=SEED_B)
q0 = r.arm_q()
hits = check_path(r, q0, tgt)
print("hits", len(hits), hits[:5])
if not hits:
    r.move_joints([tgt], [12.0])
    q = r.arm_q(); p,_ = r.tcp()
    print("diff", np.round(np.array(q)-np.array(tgt),3), "TCP", p.round(4))
EOF

# openrua op 56
ls *_cloud.npz; timeout 300 python3 -u - <<'EOF' 2>&1 | grep -v BrokenPipe
import numpy as np, glob
from robot import *
from moveit_msgs.srv import GetPositionFK
r = Robot()
q = r.arm_q()
req = GetPositionFK.Request(); req.header.frame_id=""
req.fk_link_names=["panda_link5","panda_link6","panda_link7","panda_hand"]
req.robot_state.joint_state.name=list(ARM); req.robot_state.joint_state.position=[float(v) for v in q]
fut=r.fk_cli.call_async(req); rclpy.spin_until_future_complete(r.node,fut,timeout_sec=30)
L={n:np.array([ps.pose.position.x,ps.pose.position.y,ps.pose.position.z]) for n,ps in zip(req.fk_link_names,fut.result().pose_stamped)}
for n,p in L.items(): print(n, p.round(3))
# stand points from existing clouds (stand is static)
for f in glob.glob("*_cloud.npz"):
    d=np.load(f); pw=d["pw"].reshape(-1,3); pw=pw[np.isfinite(pw).all(1)]
    m=(pw[:,0]>-0.16)&(pw[:,0]<0.15)&(pw[:,1]>0.15)&(pw[:,1]<0.4)&(pw[:,2]>0.905)
    st=pw[m]
    if len(st)==0: continue
    for n in ("panda_link6","panda_link7","panda_hand"):
        dd=np.linalg.norm(st-L[n],axis=1); i=dd.argmin()
        print(f"{f:28s} {n}: nearest stand pt {st[i].round(3)} dist {dd[i]:.3f}")
EOF

# openrua op 57
timeout 600 python3 -u - <<'EOF' 2>&1 | grep -v BrokenPipe
import numpy as np
from robot import *
from push import *
r = Robot()
tgt = r.ik((PUSH_X, Y_PRE, 1.00), QP, seed=SEED_B)
r.move_joints([tgt], [12.0])
q = r.arm_q(); p,qu = r.tcp()
print("diff", np.round(np.array(q)-np.array(tgt),3), "TCP", p.round(4))
print("R\n", quat_to_R(qu).round(3))
EOF

# openrua op 58
timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand 2>&1 | grep -v BrokenPipe | tail -1; timeout 120 python3 tools/perception/cam_snap.py sideview 2>&1 | grep -v BrokenPipe | tail -1

# openrua op 59
timeout 300 python3 cloud.py robot0_eye_in_hand 2>&1 | grep -v BrokenPipe | tail -2; timeout 120 python3 - <<'EOF'
import numpy as np
d=np.load("robot0_eye_in_hand_cloud.npz"); pw=d["pw"].reshape(-1,3); pw=pw[np.isfinite(pw).all(1)]
m=(pw[:,1]>-0.065)&(pw[:,1]<-0.02)&(pw[:,2]>0.93)&(pw[:,2]<0.965)&(pw[:,0]>-0.3)&(pw[:,0]<0.05)
h=pw[m]; print("handle pts",len(h))
if len(h): print("x",h[:,0].min().round(3),h[:,0].max().round(3),"y",h[:,1].min().round(3),h[:,1].max().round(3),"z",h[:,2].min().round(3),h[:,2].max().round(3))
for x0 in np.arange(-0.20,-0.02,0.02):
    s=h[(h[:,0]>=x0)&(h[:,0]<x0+0.02)]
    print(f"x {x0:+.2f}: n={len(s):4d} ymax={s[:,1].max() if len(s) else float('nan'):.3f}")
EOF

# openrua op 60
timeout 120 python3 - <<'EOF'
import numpy as np
for f in ("birdview_cloud.npz","agentview_cloud.npz","sideview_cloud.npz","robot0_robotview_cloud.npz"):
    d=np.load(f); pw=d["pw"].reshape(-1,3); pw=pw[np.isfinite(pw).all(1)]
    m=(pw[:,1]>-0.055)&(pw[:,1]<-0.02)&(pw[:,2]>0.925)&(pw[:,2]<0.97)&(pw[:,0]>-0.3)&(pw[:,0]<0.05)
    h=pw[m]
    print(f, len(h), "x range", (h[:,0].min().round(3), h[:,0].max().round(3)) if len(h) else None)
    print("   ", " ".join(f"{x0:+.2f}:{((h[:,0]>=x0)&(h[:,0]<x0+0.02)).sum()}" for x0 in np.arange(-0.20,-0.02,0.02)))
EOF

# openrua op 61
python3 - <<'EOF'
import re
s=open("push.py").read()
s=s.replace("PSI, PHI = np.radians(20), np.radians(15)","PSI, PHI = np.radians(25), np.radians(15)")
s=s.replace("PUSH_X = -0.14","PUSH_X = -0.12")
s=s.replace('    "stand": (-0.134, 0.136, 0.19, 0.36, 0.90, 1.25),',
 '    "stand_lo": (-0.134, 0.136, 0.185, 0.215, 0.90, 1.03),   # board edge is low near y=0.19\n    "stand": (-0.134, 0.136, 0.215, 0.36, 0.90, 1.25),')
s=s.replace("Y_END = -0.183                               # handle face at -0.19 when the drawer is closed",
 "Y_END = -0.183                               # handle face at -0.19 when the drawer is closed (face now -0.028)")
open("push.py","w").write(s)
EOF
grep -n "PSI, PHI\|PUSH_X =\|stand" push.py | head; timeout 600 python3 -u push.py check 2>&1 | grep -v BrokenPipe | tail -22

# openrua op 62
timeout 900 python3 -u - <<'EOF' 2>&1 | grep -v BrokenPipe
import numpy as np
from robot import *
from push import *
r = Robot()
wps = plan(r)
for label, q, secs in wps[1:3]:
    q0 = r.arm_q()
    hits = check_path(r, q0, q)
    print("==", label, "hits", len(hits), hits[:3])
    if hits: break
    r.move_joints([q], [secs * 2])
    p,_ = r.tcp(); print("   TCP", p.round(4), "fingers", np.round(r.fingers(),4))
EOF

# openrua op 63
timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand 2>&1 | grep -v BrokenPipe | tail -1

# openrua op 64
timeout 900 python3 -u - <<'EOF' 2>&1 | grep -v BrokenPipe
import numpy as np
from robot import *
from push import *
r = Robot()
wps = plan(r)
label, line, secs = wps[3]
q0 = r.arm_q()
hits = check_path(r, q0, line[0], ignore=("drawer",))
print("first-seg hits", len(hits))
n = len(line)
r.move_joints(line, [12.0 * (i + 1) / n for i in range(n)])
p,_ = r.tcp(); print("TCP", p.round(4), "fingers", np.round(r.fingers(),4))
EOF
timeout 60 ros2 topic echo --once /franka_robot_state_broadcaster/external_wrench 2>&1 | grep -A3 force

# openrua op 65
for c in birdview agentview sideview robot0_eye_in_hand; do timeout 300 python3 cloud.py $c 2>&1 | grep -v BrokenPipe | tail -1; done
timeout 120 python3 - <<'EOF'
import numpy as np
for f in ("birdview","agentview","sideview"):
    d=np.load(f+"_cloud.npz"); pw=d["pw"].reshape(-1,3); pw=pw[np.isfinite(pw).all(1)]
    # anything in the region where the open drawer used to be (y > -0.19), at drawer-wall height, x in drawer span
    m=(pw[:,0]>-0.23)&(pw[:,0]<0.0)&(pw[:,1]>-0.19)&(pw[:,1]<-0.03)&(pw[:,2]>0.905)&(pw[:,2]<0.99)
    s=pw[m]
    print(f"{f}: pts in old-drawer volume (z .905-.99): {len(s)}", (s[:,1].min().round(3), s[:,1].max().round(3), s[:,2].max().round(3)) if len(s) else "")
    # cabinet/drawer front face: histogram of y for x in [-0.22,-0.19] (drawer -x edge), z .92-.98
    m2=(pw[:,0]>-0.22)&(pw[:,0]<-0.16)&(pw[:,2]>0.92)&(pw[:,2]<0.98)&(pw[:,1]<0.0)&(pw[:,1]>-0.30)
    s2=pw[m2]
    if len(s2):
        h,e=np.histogram(s2[:,1],bins=np.arange(-0.30,0.0,0.01)); 
        print("   y-hist (x -0.22..-0.16, z .92-.98):", " ".join(f"{e[i]:.2f}:{h[i]}" for i in range(len(h)) if h[i]>0))
EOF

# openrua op 66
for c in agentview sideview birdview; do timeout 120 python3 tools/perception/cam_snap.py $c 2>&1 | grep -v BrokenPipe | tail -1; done

# openrua op 67
timeout 120 python3 - <<'EOF'
import numpy as np
for f in ("birdview","agentview","sideview"):
    d=np.load(f+"_cloud.npz"); pw=d["pw"].reshape(-1,3); pw=pw[np.isfinite(pw).all(1)]
    for name,(x0,x1,z0,z1) in {"bottom handle (x -.10..-.05)":(-0.10,-0.05,0.93,0.965),
                               "middle handle":(-0.13,-0.05,1.005,1.035),
                               "top handle":(-0.13,-0.05,1.075,1.105)}.items():
        m=(pw[:,0]>x0)&(pw[:,0]<x1)&(pw[:,2]>z0)&(pw[:,2]<z1)&(pw[:,1]<-0.10)&(pw[:,1]>-0.40)
        s=pw[m]
        if len(s): print(f"{f:9s} {name:30s} n={len(s):4d} ymax={s[:,1].max():.3f}  y90={np.percentile(s[:,1],90):.3f}")
    # drawer +x side wall top (x -0.005..0.015, z .975-.99), y > -0.20
    m=(pw[:,0]>-0.01)&(pw[:,0]<0.02)&(pw[:,2]>0.97)&(pw[:,2]<0.995)&(pw[:,1]>-0.22)&(pw[:,1]<-0.02)
    print(f"{f:9s} +x drawer wall top pts:", m.sum())
    # anything at drawer-wall height in front of cabinet face, x in [-0.23,-0.19] (the -x wall), excluding hand region y>-0.19
    m=(pw[:,0]>-0.235)&(pw[:,0]<-0.20)&(pw[:,2]>0.97)&(pw[:,2]<0.995)&(pw[:,1]>-0.22)&(pw[:,1]<-0.02)
    print(f"{f:9s} -x drawer wall top pts:", m.sum())
EOF

# openrua op 68
timeout 120 python3 - <<'EOF'
import numpy as np
d=np.load("agentview_cloud.npz"); pw=d["pw"].reshape(-1,3); pw=pw[np.isfinite(pw).all(1)]
m=(pw[:,0]>-0.235)&(pw[:,0]<-0.20)&(pw[:,2]>0.97)&(pw[:,2]<0.995)&(pw[:,1]>-0.22)&(pw[:,1]<-0.02)
s=pw[m]; print("y range", s[:,1].min().round(3), s[:,1].max().round(3), "median", np.median(s[:,1]).round(3))
EOF

# openrua op 69
timeout 900 python3 -u - <<'EOF' 2>&1 | grep -v BrokenPipe
import numpy as np
from robot import *
from push import *
r = Robot()
p,_ = r.tcp()
seed = r.arm_q(); wps=[]
for y in np.linspace(p[1], 0.0, 6)[1:]:
    q = r.ik((PUSH_X, y, PUSH_Z), QP, seed=seed); assert q is not None; seed=q; wps.append(q)
r.move_joints(wps, [10.0*(i+1)/len(wps) for i in range(len(wps))])
p,_ = r.tcp(); print("TCP after withdraw", p.round(4))
q = r.ik((PUSH_X, 0.0, 1.06), QP, seed=seed); assert q is not None
print("hits", len(check_path(r, r.arm_q(), q)))
r.move_joints([q], [6.0])
p,_ = r.tcp(); print("TCP final", p.round(4), "fingers", np.round(r.fingers(),4))
EOF
for c in agentview sideview robot0_eye_in_hand; do timeout 120 python3 tools/perception/cam_snap.py $c 2>&1 | grep -v BrokenPipe | tail -1; done

# openrua op 70
mkdir -p "$(dirname /claude-config/projects/-workspace/memory/panda-sim-machine-quirks.md)"
cat > /claude-config/projects/-workspace/memory/panda-sim-machine-quirks.md <<'OPENRUA_EOF'
---
name: panda-sim-machine-quirks
description: Verified facts about the /workspace Franka Panda ROS2 sim that contradict or extend the generic docs (frames, IK tip link, controller timing)
metadata:
  type: project
---

Verified on the /workspace Panda sim (Sept 2026), where the generic docs/ are wrong or silent:
- `/compute_fk` and `/compute_ik` with empty `frame_id` return/accept poses already in the WORLD frame (world→panda_link0 = (-0.66, 0, 0.912)); do not add the base offset. Requesting frame `panda_link0` gives error -31.
- IK tip link is `panda_link8`, rotated -45° about z relative to `panda_hand`: right-multiply the desired hand quaternion by (0,0,0.3826834,0.9238795). TCP is 0.1034 m along hand +z.
- Paused-clock sim: nothing converges after a trajectory result arrives. Big joint swings (~2 rad) in 4 s abort with error -5 midway (arm stops where it is); use ≥10 s for large reorientations. Small moves in 3-4 s return code 0.
- Gripper only stops at open/closed; scene objects may be floating at session start until the first command advances the sim.

**Why:** cost several debugging rounds; README/docs claim IK is in the arm-base frame.
**How to apply:** reuse `/workspace/robot.py` (`Robot` class) and `/workspace/push.py` (`check_path` swept-volume box check) if the workspace persists; otherwise start from these facts.
OPENRUA_EOF

# openrua op 71
cd /claude-config/projects/-workspace/memory && (test -f MEMORY.md && cat MEMORY.md || true) && echo "- [Panda sim machine quirks](panda-sim-machine-quirks.md) — FK/IK in world frame, IK tip = panda_link8, slow big swings" >> MEMORY.md
