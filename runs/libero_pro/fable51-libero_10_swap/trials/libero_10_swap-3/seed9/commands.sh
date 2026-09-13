#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la && find . -maxdepth 3 -type f | head -100

# openrua op 1
cat tools/perception/cam_snap.py tools/perception/px2world.py tools/action/fjt_send.py tools/action/gripper_cmd.py tools/action/ik_move.py

# openrua op 2
timeout 30 ros2 topic list; echo ---; timeout 30 ros2 action list; echo ---; timeout 30 ros2 service list | grep -v -E "parameter|describe|get_type" ; echo ---; timeout 20 ros2 node list

# openrua op 3
for c in agentview frontview sideview birdview robot0_eye_in_hand robot0_robotview; do timeout 60 python3 tools/perception/cam_snap.py $c & done; wait; timeout 20 ros2 topic echo /joint_states --once

# openrua op 4
timeout 30 ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id|x:|y:|z:|w:" | head -80; echo ---; timeout 15 ros2 topic echo /tf --once 2>&1 | grep -E "frame_id" | head

# openrua op 5
timeout 15 ros2 topic echo /tf --once 2>&1 | head -120

# openrua op 6
mkdir -p "$(dirname /workspace/cam_dump.py)"
cat > /workspace/cam_dump.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Dump color, depth, intrinsics and world<-optical TF of a camera to <cam>.npz
Usage: python3 cam_dump.py <cam> [<cam> ...]
Then: python3 -c "import camq; camq.px('birdview', u, v)"
"""
import sys
import numpy as np
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
import cv2


def grab(node, topic, typ, timeout=30.0):
    got = {}
    sub = node.create_subscription(typ, topic, lambda m: got.setdefault("m", m), 1)
    import time
    t0 = time.time()
    while "m" not in got and time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    if "m" not in got:
        raise SystemExit(f"no message on {topic}")
    return got["m"]


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main():
    rclpy.init()
    node = rclpy.create_node("cam_dump")
    tfbuf = Buffer()
    TransformListener(tfbuf, node)
    br = CvBridge()
    import time
    for cam in sys.argv[1:]:
        color = grab(node, f"/{cam}/color/image_raw", Image)
        depth = grab(node, f"/{cam}/depth/image_raw", Image)
        info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
        frame = f"{cam}_optical_frame"
        t0 = time.time()
        while time.time() - t0 < 15 and not tfbuf.can_transform("world", frame, rclpy.time.Time()):
            rclpy.spin_once(node, timeout_sec=0.2)
        t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
        q = t.transform.rotation
        T = np.eye(4)
        T[:3, :3] = quat_to_R(q.x, q.y, q.z, q.w)
        T[:3, 3] = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
        img = br.imgmsg_to_cv2(color, "bgr8")
        d = br.imgmsg_to_cv2(depth, "passthrough").astype(np.float32)
        cv2.imwrite(f"{cam}.png", img)
        np.savez(f"{cam}.npz", color=img, depth=d, K=np.array(info.k).reshape(3, 3), T=T)
        print(cam, img.shape, d.shape, "T=", T[:3, 3])
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 7
mkdir -p "$(dirname /workspace/camq.py)"
cat > /workspace/camq.py <<'OPENRUA_EOF'
import numpy as np


def load(cam):
    z = np.load(f"{cam}.npz")
    return z["color"], z["depth"], z["K"], z["T"]


def px(cam, u, v, verbose=True):
    _, d, K, T = load(cam)
    zc = float(d[v, u])
    if not np.isfinite(zc) or zc <= 0:
        raise ValueError(f"bad depth {zc}")
    p = np.array([(u - K[0, 2]) * zc / K[0, 0], (v - K[1, 2]) * zc / K[1, 1], zc, 1.0])
    w = T @ p
    if verbose:
        print(f"{cam} ({u},{v}) depth={zc:.3f} -> world {w[0]:.4f} {w[1]:.4f} {w[2]:.4f}")
    return w[:3]


def cloud(cam):
    """full point cloud in world frame, shape (H,W,3)"""
    _, d, K, T = load(cam)
    H, W = d.shape
    us, vs = np.meshgrid(np.arange(W), np.arange(H))
    X = (us - K[0, 2]) * d / K[0, 0]
    Y = (vs - K[1, 2]) * d / K[1, 1]
    P = np.stack([X, Y, d, np.ones_like(d)], -1)
    return (P @ T.T)[..., :3]


def world2px(cam, xyz):
    _, d, K, T = load(cam)
    Ti = np.linalg.inv(T)
    p = Ti @ np.array([*xyz, 1.0])
    u = K[0, 0] * p[0] / p[2] + K[0, 2]
    v = K[1, 1] * p[1] / p[2] + K[1, 2]
    return u, v
OPENRUA_EOF

# openrua op 8
timeout 120 python3 cam_dump.py birdview agentview frontview sideview

# openrua op 9
python3 -c "
import camq, numpy as np
# table surface
camq.px('birdview', 320, 400)
camq.px('birdview', 500, 400)
# bowl center guess
camq.px('birdview', 311, 308)
# drawer region
camq.px('birdview', 270, 265)
camq.px('birdview', 220, 265)
# robot base
camq.px('birdview', 320, 230)
# world -> birdview px of arm base
print(camq.world2px('birdview', (-0.66,0,0.912)))
"

# openrua op 10
python3 -c "
import cv2, numpy as np
img = cv2.imread('birdview.png')
crop = img[200:360, 160:480]
crop = cv2.resize(crop, None, fx=3, fy=3, interpolation=cv2.INTER_NEAREST)
# draw grid every 20px original
for i in range(0, crop.shape[1], 60): cv2.line(crop,(i,0),(i,crop.shape[0]),(0,255,0),1); cv2.putText(crop,str(160+i//3),(i,12),cv2.FONT_HERSHEY_SIMPLEX,0.4,(0,255,0),1)
for j in range(0, crop.shape[0], 60): cv2.line(crop,(0,j),(crop.shape[1],j),(0,255,0),1); cv2.putText(crop,str(200+j//3),(0,j+12),cv2.FONT_HERSHEY_SIMPLEX,0.4,(0,255,0),1)
cv2.imwrite('bird_crop.png', crop)
"

# openrua op 11
python3 -c "
import camq, numpy as np
P = camq.cloud('agentview')
img,_,_,_ = camq.load('agentview')
H,W,_ = P.shape
# table height estimate
m = np.isfinite(P[...,2])
z = P[...,2]
print('table z hist', np.percentile(z[m & (P[...,0]>-0.3)&(P[...,0]<0.5)&(np.abs(P[...,1])<0.5)],[5,25,50,75,95]))
sel = m & (z>0.905) & (z<1.25) & (P[...,0]>-0.4)&(P[...,0]<0.6)&(np.abs(P[...,1])<0.7)
vs,us = np.nonzero(sel)
print('n', len(vs))
# coarse cluster by grid of 2cm in xy
pts = P[sel]
import collections
grid = collections.Counter()
for p in pts:
    grid[(round(p[0],1), round(p[1],1))]+=1
for k,v in sorted(grid.items(), key=lambda kv:-kv[1])[:40]: print(k,v)
"

# openrua op 12
python3 -c "
import cv2, numpy as np
img = cv2.imread('agentview.png')
x0,y0=20,120
crop = img[y0:340, x0:420]
crop = cv2.resize(crop, None, fx=2.4, fy=2.4, interpolation=cv2.INTER_NEAREST)
s=2.4
for i in range(0, 400, 20): X=int(i*s); cv2.line(crop,(X,0),(X,crop.shape[0]),(0,255,0),1); cv2.putText(crop,str(x0+i),(X,12),cv2.FONT_HERSHEY_SIMPLEX,0.4,(0,255,0),1)
for j in range(0, 220, 20): Y=int(j*s); cv2.line(crop,(0,Y),(crop.shape[1],Y),(0,255,0),1); cv2.putText(crop,str(y0+j),(0,Y+12),cv2.FONT_HERSHEY_SIMPLEX,0.4,(0,255,0),1)
cv2.imwrite('agent_crop.png', crop)
"

# openrua op 13
python3 -c "
import camq
print('bowl rim/center:')
for uv in [(350,188),(350,205),(322,210),(378,210),(350,232)]: camq.px('agentview',*uv)
print('drawer floor:')
for uv in [(200,240),(240,240),(270,240),(200,280),(240,280),(270,280),(230,225)]: camq.px('agentview',*uv)
print('drawer walls/front:')
for uv in [(240,296),(190,260),(280,250),(200,215),(260,213)]: camq.px('agentview',*uv)
print('cabinet top:')
for uv in [(100,160),(170,200)]: camq.px('agentview',*uv)
print('bottle:')
for uv in [(295,235),(295,300)]: camq.px('agentview',*uv)
"

# openrua op 14
python3 -c "
import camq, numpy as np
P = camq.cloud('birdview')
# drawer region in birdview
sub = P[225:300, 250:310]
z = sub[...,2]
print('z percentiles', np.percentile(z[np.isfinite(z)],[1,10,50,90,99]))
# drawer floor points: z in 0.915..0.935
for lo,hi,name in [(0.915,0.935,'floor'),(0.95,1.0,'walls'),(1.0,1.2,'cabinet-top?')]:
    m=(z>lo)&(z<hi)
    if m.sum():
        pts=sub[m]; print(name, m.sum(), 'x',pts[:,0].min().round(3),pts[:,0].max().round(3),'y',pts[:,1].min().round(3),pts[:,1].max().round(3))
# cabinet
sub = P[220:300, 190:262]; z=sub[...,2]
m=(z>1.05); pts=sub[m]; print('cabinet top', m.sum(), 'x',pts[:,0].min().round(3),pts[:,0].max().round(3),'y',pts[:,1].min().round(3),pts[:,1].max().round(3),'z',np.median(pts[:,2]).round(3))
# whole table: any thing above table in x -0.45..0.6
z=P[...,2]; m=np.isfinite(z)&(z>0.905)&(z<1.5)&(P[...,0]>-0.4)&(P[...,0]<0.6)&(np.abs(P[...,1])<0.7)
pts=P[m]
# histogram in y
h,edges=np.histogram(pts[:,1],bins=np.arange(-0.7,0.71,0.05))
for c,e in zip(h,edges): print(f'y {e:+.2f}: {c}')
"

# openrua op 15
python3 -c "
import camq, numpy as np
for cam in ['agentview','frontview','sideview']:
    P = camq.cloud(cam)
    z=P[...,2]
    m=np.isfinite(z)&(z>0.905)&(z<1.2)&(P[...,0]>-0.30)&(P[...,0]<-0.08)&(P[...,1]>-0.03)&(P[...,1]<0.14)
    pts=P[m]
    if len(pts)==0: print(cam,'none'); continue
    print(cam,'bowl pts',len(pts),'x',pts[:,0].min().round(3),pts[:,0].max().round(3),'y',pts[:,1].min().round(3),pts[:,1].max().round(3),'z',pts[:,2].min().round(3),np.percentile(pts[:,2],95).round(3),pts[:,2].max().round(3))
    # rim: highest points
    top=pts[pts[:,2]>np.percentile(pts[:,2],90)]
    print('   rim centroid',top.mean(0).round(3))
P=camq.cloud('birdview'); z=P[...,2]
m=np.isfinite(z)&(z>0.93)&(z<1.1)&(P[...,1]>-0.19)&(P[...,1]<-0.03)&(P[...,0]>-0.3)&(P[...,0]<0.05)
pts=P[m]; print('drawer walls z', np.percentile(pts[:,2],[50,90,99]).round(3), 'y range', pts[:,1].min().round(3), pts[:,1].max().round(3))
# drawer front panel: points y>-0.09
fp=pts[pts[:,1]>-0.09]; print('front panel', len(fp), 'y',fp[:,1].min().round(3),fp[:,1].max().round(3),'z max',fp[:,2].max().round(3), 'x', fp[:,0].min().round(3), fp[:,0].max().round(3))
"

# openrua op 16
python3 -c "
import camq, numpy as np
for cam in ['agentview','frontview','sideview']:
    P = camq.cloud(cam)
    z=P[...,2]
    m=np.isfinite(z)&(z>0.905)&(z<1.06)&(P[...,0]>-0.30)&(P[...,0]<-0.08)&(P[...,1]>-0.03)&(P[...,1]<0.14)
    pts=P[m]
    print(cam,'bowl pts',len(pts),'x',pts[:,0].min().round(3),pts[:,0].max().round(3),'y',pts[:,1].min().round(3),pts[:,1].max().round(3),'z',pts[:,2].min().round(3),np.percentile(pts[:,2],[50,90,99]).round(3))
    top=pts[pts[:,2]>np.percentile(pts[:,2],80)]
    print('   rim centroid',top.mean(0).round(3), 'rim x',top[:,0].min().round(3),top[:,0].max().round(3),'y',top[:,1].min().round(3),top[:,1].max().round(3))
P=camq.cloud('birdview'); z=P[...,2]
m=np.isfinite(z)&(z>0.93)&(z<1.1)&(P[...,1]>-0.19)&(P[...,1]<-0.03)&(P[...,0]>-0.3)&(P[...,0]<0.02)
pts=P[m]
fp=pts[pts[:,1]>-0.09]; print('front panel', len(fp), 'y',fp[:,1].min().round(3),fp[:,1].max().round(3),'z', np.percentile(fp[:,2],[50,90,99]).round(3), 'x', fp[:,0].min().round(3), fp[:,0].max().round(3))
sw=pts[(pts[:,1]<-0.09)]; print('side walls', len(sw), 'z', np.percentile(sw[:,2],[50,90,99]).round(3), 'x', np.percentile(sw[:,0],[1,50,99]).round(3))
"

# openrua op 17
timeout 300 python3 tools/action/gripper_cmd.py 0.04; timeout 20 ros2 topic echo /joint_states --once | grep -A9 position; timeout 10 ros2 topic echo /clock --once

# openrua op 18
timeout 120 python3 cam_dump.py agentview birdview frontview sideview >/dev/null && python3 -c "
import camq, numpy as np
for cam in ['agentview','frontview','sideview']:
    P = camq.cloud(cam)
    z=P[...,2]
    m=np.isfinite(z)&(z>0.905)&(z<1.06)&(P[...,0]>-0.30)&(P[...,0]<-0.08)&(P[...,1]>-0.03)&(P[...,1]<0.14)
    pts=P[m]
    print(cam,'bowl pts',len(pts),'x',pts[:,0].min().round(3),pts[:,0].max().round(3),'y',pts[:,1].min().round(3),pts[:,1].max().round(3),'z',pts[:,2].min().round(3),np.percentile(pts[:,2],[50,90,99]).round(3))
    top=pts[pts[:,2]>np.percentile(pts[:,2],80)]
    print('   rim centroid',top.mean(0).round(3), 'rim x',top[:,0].min().round(3),top[:,0].max().round(3),'y',top[:,1].min().round(3),top[:,1].max().round(3))
"

# openrua op 19
python3 -c "
import camq, numpy as np
P=camq.cloud('sideview'); z=P[...,2]
m=np.isfinite(z)&(P[...,0]>-0.20)&(P[...,0]<-0.03)&(P[...,1]<-0.02)&(P[...,1]>-0.45)&(z>0.90)&(z<1.15)
pts=P[m]
# bin by z, report median y (closest to camera = max y actually since camera at +y sees front surfaces)
for lo in np.arange(0.90,1.15,0.01):
    s=pts[(pts[:,2]>=lo)&(pts[:,2]<lo+0.01)]
    if len(s): print(f'z {lo:.2f}-{lo+0.01:.2f}: n={len(s):4d} y max {s[:,1].max():.3f} y median {np.median(s[:,1]):.3f}  y min {s[:,1].min():.3f}')
"

# openrua op 20
python3 -c "
import camq, numpy as np
P=camq.cloud('birdview'); z=P[...,2]
m=np.isfinite(z)&(P[...,0]>-0.15)&(P[...,0]<-0.07)&(P[...,1]<0.0)&(P[...,1]>-0.45)
pts=P[m]
for lo in np.arange(-0.45,0.0,0.01):
    s=pts[(pts[:,1]>=lo)&(pts[:,1]<lo+0.01)]
    if len(s): print(f'y {lo:+.2f}: n={len(s):3d} z max {s[:,2].max():.3f} z median {np.median(s[:,2]):.3f} z min {s[:,2].min():.3f}')
print('--- x profile of drawer at y in [-0.17,-0.10]')
m=np.isfinite(z)&(P[...,1]>-0.17)&(P[...,1]<-0.10)&(P[...,0]>-0.35)&(P[...,0]<0.1)
pts=P[m]
for lo in np.arange(-0.35,0.1,0.01):
    s=pts[(pts[:,0]>=lo)&(pts[:,0]<lo+0.01)]
    if len(s): print(f'x {lo:+.2f}: n={len(s):3d} z max {s[:,2].max():.3f} z median {np.median(s[:,2]):.3f}')
"

# openrua op 21
timeout 20 ros2 interface show moveit_msgs/srv/GetPositionFK | head -30; timeout 20 ros2 interface show control_msgs/action/GripperCommand | head -20

# openrua op 22
mkdir -p "$(dirname /workspace/rob.py)"
cat > /workspace/rob.py <<'OPENRUA_EOF'
"""Robot helper: joint state, FK, IK, trajectory, gripper, servo. World-frame poses.

world -> panda_link0 is a pure translation BASE (from /tf).
"""
import time
import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import TwistStamped, WrenchStamped
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
TWIST = next(a for a in M["actuators"] if a["kind"] == "cartesian_twist")
ARM = FJT["joints"]
BASE = np.array([-0.66, 0.0, 0.912])  # world -> panda_link0 translation
TCP = M["hand"]["tcp_offset_m"]


def quat_to_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def R_to_quat(R):
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1) * 2
        return np.array([(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s])
    i = np.argmax(np.diag(R))
    if i == 0:
        s = np.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return np.array([0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s])
    if i == 1:
        s = np.sqrt(1 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        return np.array([(R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s])
    s = np.sqrt(1 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
    return np.array([(R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s, (R[1, 0] - R[0, 1]) / s])


def down_quat(yaw=0.0):
    """Hand pointing straight down (hand z = world -z); yaw rotates finger axis
    about world z. yaw=0: hand x along world +x (fingers close along... hand y).
    Panda hand: fingers open/close along hand Y axis."""
    # hand z -> -world z; hand x -> world x rotated by yaw; hand y accordingly
    c, s = np.cos(yaw), np.sin(yaw)
    R = np.array([[c, s, 0], [s, -c, 0], [0, 0, -1]])  # columns = hand axes in world
    return R_to_quat(R)


class Robot:
    def __init__(self, name="rob"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = None
        self._wr = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.node.create_subscription(WrenchStamped, "/franka_robot_state_broadcaster/external_wrench", self._on_wr, 10)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.ik_cli = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.twist_pub = self.node.create_publisher(TwistStamped, TWIST["port"], 10)

    def _on_js(self, m):
        self._js = m

    def _on_wr(self, m):
        self._wr = m

    def spin(self, t=0.1):
        rclpy.spin_once(self.node, timeout_sec=t)

    # ---------- sensing ----------
    def joints(self, fresh=True):
        if fresh:
            self._js = None
        t0 = time.time()
        while self._js is None and time.time() - t0 < 10:
            self.spin(0.2)
        d = dict(zip(self._js.name, self._js.position))
        return d

    def arm_q(self):
        d = self.joints()
        return np.array([d[j] for j in ARM])

    def fingers(self):
        d = self.joints()
        return d["panda_finger_joint1"], d["panda_finger_joint2"]

    def wrench(self):
        self._wr = None
        t0 = time.time()
        while self._wr is None and time.time() - t0 < 5:
            self.spin(0.2)
        if self._wr is None:
            return None
        f = self._wr.wrench.force
        t = self._wr.wrench.torque
        return np.array([f.x, f.y, f.z, t.x, t.y, t.z])

    def fk(self, q=None, link="panda_hand"):
        """returns (pos_world, quat) of link"""
        if q is None:
            q = self.arm_q()
        self.fk_cli.wait_for_service(5)
        req = GetPositionFK.Request()
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed {res}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE
        q = np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        return pos, q

    def tcp(self, q=None):
        pos, quat = self.fk(q)
        R = quat_to_R(quat)
        return pos + TCP * R[:, 2], quat

    # ---------- IK ----------
    def ik(self, pos_world, quat, seed=None, at_tcp=True, timeout=60):
        pos = np.array(pos_world, float)
        if at_tcp:
            R = quat_to_R(quat)
            pos = pos - TCP * R[:, 2]
        pos = pos - BASE
        self.ik_cli.wait_for_service(5)
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pos)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        if seed is None:
            seed = self.arm_q()
        req.ik_request.robot_state.joint_state.name = list(ARM)
        req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
        req.ik_request.avoid_collisions = False
        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        res = fut.result()
        if res is None:
            raise RuntimeError("IK timeout")
        if res.error_code.val != 1:
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return np.array([sol[j] for j in ARM])

    # ---------- motion ----------
    def move_q(self, q, seconds=3.0, via=None):
        """via: list of (q, t) intermediate points"""
        self.fjt.wait_for_server(10)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pts = []
        if via:
            for vq, vt in via:
                pt = JointTrajectoryPoint(positions=[float(v) for v in vq])
                pt.time_from_start = Duration(sec=int(vt), nanosec=int((vt % 1) * 1e9))
                pts.append(pt)
        pt = JointTrajectoryPoint(positions=[float(v) for v in q])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=600)
        code = rf.result().result.error_code
        qa = self.arm_q()
        err = np.abs(qa - np.array(q)).max()
        print(f"move_q: code={code} max_joint_err={err:.4f}")
        return code, err

    def move_tcp(self, pos, quat, seconds=3.0, seed=None):
        q = self.ik(pos, quat, seed=seed)
        if q is None:
            print(f"IK FAILED for {pos}")
            return None
        code, err = self.move_q(q, seconds)
        p, _ = self.tcp()
        print(f"  tcp now {p.round(4)} target {np.array(pos).round(4)}")
        return q

    def gripper(self, width):
        self.grip.wait_for_server(10)
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = float(GRIP["max_effort"])
        f = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, f, timeout_sec=60)
        rf = f.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result
        fg = self.fingers()
        print(f"gripper: reached={r.reached_goal} stalled={r.stalled} fingers={fg[0]:.4f},{fg[1]:.4f}")
        return fg

    def servo(self, v, n=20, frame=None):
        """publish n twist messages with linear velocity v (world/base frame, m/s)"""
        msg = TwistStamped()
        msg.header.frame_id = frame or TWIST["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, v)
        for _ in range(n):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.twist_pub.publish(msg)
            self.spin(0.05)
OPENRUA_EOF

# openrua op 23
timeout 120 python3 -c "
import rob, numpy as np
r = rob.Robot()
q = r.arm_q(); print('q', q.round(4))
pos, quat = r.fk(q); print('hand pos', pos.round(4), 'quat', quat.round(4))
print('R', rob.quat_to_R(quat).round(3))
tp,_ = r.tcp(q); print('tcp', tp.round(4))
print('fingers', r.fingers())
print('wrench', r.wrench())
print('down_quat(0)', rob.down_quat(0).round(4), 'down_quat(pi/4)', rob.down_quat(np.pi/4).round(4))
"

# openrua op 24
sed -i 's/^BASE = np.array(\[-0.66, 0.0, 0.912\]).*/BASE = np.array([0.0, 0.0, 0.0])  # FK\/IK on this machine are already in world coords (verified)/' rob.py && timeout 180 python3 -c "
import rob, numpy as np
r = rob.Robot()
q = r.arm_q()
pos, quat = r.fk(q); print('hand world', pos.round(4))
tp,_ = r.tcp(q); print('tcp world', tp.round(4))
sol = r.ik(pos, quat, at_tcp=False); print('ik(hand pose) ->', None if sol is None else sol.round(3), 'current', q.round(3))
sol2 = r.ik(tp, quat, at_tcp=True); print('ik(tcp pose) ->', None if sol2 is None else sol2.round(3))
# test a target: above the bowl
sol3 = r.ik([-0.175, 0.052, 1.10], rob.down_quat(0)); print('ik above bowl ->', None if sol3 is None else sol3.round(3))
if sol3 is not None: print(' fk check', r.tcp(sol3)[0].round(4))
"

# openrua op 25
timeout 180 python3 -c "
import rob, numpy as np
r = rob.Robot()
sol3 = np.array([0.101,0.038,0.005,-2.425,0.,2.463,0.107])
for link in ['panda_link8','panda_hand']:
    p,q = r.fk(sol3, link); print(link, p.round(4), q.round(4))
print('target quat was', rob.down_quat(0))
"

# openrua op 26
python3 - <<'EOF'
import re
s = open('/workspace/rob.py').read()
s = s.replace('''        pos = pos - BASE
        self.ik_cli.wait_for_service(5)''', '''        pos = pos - BASE
        # IK tip link is panda_link8 = hand rotated +45deg about hand z
        Rh = quat_to_R(quat)
        c, s_ = np.cos(np.pi / 4), np.sin(np.pi / 4)
        R8 = Rh @ np.array([[c, -s_, 0], [s_, c, 0], [0, 0, 1]])
        quat = R_to_quat(R8)
        self.ik_cli.wait_for_service(5)''')
open('/workspace/rob.py','w').write(s)
EOF
timeout 180 python3 -c "
import rob, numpy as np
r = rob.Robot()
for yaw in [0, np.pi/2]:
    sol = r.ik([-0.175, 0.052, 1.10], rob.down_quat(yaw))
    print('yaw',yaw,'sol', None if sol is None else sol.round(3))
    if sol is not None:
        p,q = r.fk(sol,'panda_hand'); print('  hand', p.round(4), q.round(4), 'want', rob.down_quat(yaw).round(4)); print('  R hand', rob.quat_to_R(q).round(2).tolist())
"

# openrua op 27
timeout 900 python3 -u -c "
import rob, numpy as np
r = rob.Robot()
r.gripper(0.0)
yaw = np.pi/2
q = r.move_tcp([-0.11, -0.125, 1.07], rob.down_quat(yaw), 3.0)
q = r.move_tcp([-0.11, -0.125, 0.955], rob.down_quat(yaw), 2.0)
print('wrench', r.wrench().round(2))
" 2>&1 | tail -20

# openrua op 28
timeout 300 python3 -u -c "
import rob, numpy as np
r = rob.Robot()
q = r.arm_q(); print('actual q', q.round(3))
p,quat = r.tcp(q); print('tcp', p.round(4), 'R', rob.quat_to_R(quat).round(2).tolist())
sol = r.ik([-0.11, -0.125, 0.955], rob.down_quat(np.pi/2)); print('ik sol', sol.round(3))
sol2 = r.ik([-0.11, -0.125, 0.955], rob.down_quat(-np.pi/2)); print('ik sol yaw -90', sol2.round(3))
"

# openrua op 29
timeout 900 python3 -u -c "
import rob, numpy as np, time
r = rob.Robot()
sol = r.ik([-0.11, -0.125, 1.06], rob.down_quat(np.pi/2))
print('sol', sol.round(3))
for i in range(3):
    code, err = r.move_q(sol, 4.0)
    q = r.arm_q(); print('  actual', q.round(3), 'diff', (q-sol).round(3))
    if err < 0.02: break
p,quat = r.tcp(); print('tcp', p.round(4), 'R', rob.quat_to_R(quat).round(2).tolist())
" 2>&1 | tail -20

# openrua op 30
python3 - <<'EOF'
s = open('/workspace/rob.py').read()
s = s.replace('''    def move_tcp(self, pos, quat, seconds=3.0, seed=None):
        q = self.ik(pos, quat, seed=seed)
        if q is None:
            print(f"IK FAILED for {pos}")
            return None
        code, err = self.move_q(q, seconds)''', '''    def move_tcp(self, pos, quat, seconds=3.0, seed=None, retries=3, tol=0.02):
        q = self.ik(pos, quat, seed=seed)
        if q is None:
            print(f"IK FAILED for {pos}")
            return None
        for _ in range(retries):
            code, err = self.move_q(q, seconds)
            if err < tol:
                break''')
open('/workspace/rob.py','w').write(s)
EOF
timeout 900 python3 -u -c "
import rob, numpy as np
r = rob.Robot()
print('wrench before', r.wrench().round(2))
r.move_tcp([-0.11, -0.125, 0.955], rob.down_quat(np.pi/2), 2.5)
print('wrench after', r.wrench().round(2))
" 2>&1 | tail

# openrua op 31
timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand >/dev/null; timeout 120 python3 tools/perception/cam_snap.py agentview >/dev/null; python3 -c "
import cv2
a=cv2.imread('agentview.png'); b=cv2.imread('robot0_eye_in_hand.png')
import numpy as np
cv2.imwrite('pair.png', np.hstack([a,b]))"

# openrua op 32
timeout 1200 python3 -u -c "
import rob, numpy as np
r = rob.Robot()
yaw=np.pi/2
for y in [-0.09, -0.065, -0.045, -0.025]:
    r.move_tcp([-0.11, y, 0.955], rob.down_quat(yaw), 2.0, retries=2)
    print('  y target', y, 'wrench', r.wrench().round(2))
" 2>&1 | grep -v '^$' | tail -30

# openrua op 33
timeout 900 python3 -u -c "
import rob, numpy as np
r = rob.Robot()
yaw=np.pi/2
r.move_tcp([-0.11, -0.12, 0.955], rob.down_quat(yaw), 1.5)
r.move_tcp([-0.11, -0.12, 1.10], rob.down_quat(yaw), 2.0)
print('wrench', r.wrench().round(2))
" 2>&1 | tail -5 && timeout 120 python3 cam_dump.py birdview agentview > /dev/null && python3 -c "
import camq, numpy as np
P=camq.cloud('birdview'); z=P[...,2]
# bowl from birdview
m=np.isfinite(z)&(z>0.91)&(z<1.0)&(P[...,0]>-0.30)&(P[...,0]<-0.05)&(P[...,1]>-0.05)&(P[...,1]<0.15)
pts=P[m]; print('bowl pts',len(pts),'x',pts[:,0].min().round(4),pts[:,0].max().round(4),'y',pts[:,1].min().round(4),pts[:,1].max().round(4),'z',np.percentile(pts[:,2],[5,50,95]).round(3))
rim=pts[pts[:,2]>0.94]; print('rim x',rim[:,0].min().round(4),rim[:,0].max().round(4),'y',rim[:,1].min().round(4),rim[:,1].max().round(4), 'center', ((rim[:,0].min()+rim[:,0].max())/2).round(4), ((rim[:,1].min()+rim[:,1].max())/2).round(4))
# drawer opening at fine resolution
m=np.isfinite(z)&(P[...,0]>-0.20)&(P[...,0]<-0.03)&(P[...,1]<-0.05)&(P[...,1]>-0.25)
pts=P[m]
for lo in np.arange(-0.25,-0.05,0.005):
    s=pts[(pts[:,1]>=lo)&(pts[:,1]<lo+0.005)]
    if len(s): print(f'y {lo:+.3f}: n={len(s):3d} z max {s[:,2].max():.3f} z median {np.median(s[:,2]):.3f} z min {s[:,2].min():.3f}')
"

# openrua op 34
python3 -c "
import camq, numpy as np
P=camq.cloud('agentview'); z=P[...,2]
# bottle: tall thing near (0.05,-0.035)
m=np.isfinite(z)&(z>0.95)&(z<1.2)&(P[...,0]>-0.02)&(P[...,0]<0.15)&(P[...,1]>-0.12)&(P[...,1]<0.05)
pts=P[m]; print('bottle pts',len(pts),'x',pts[:,0].min().round(3),pts[:,0].max().round(3),'y',pts[:,1].min().round(3),pts[:,1].max().round(3),'zmax',pts[:,2].max().round(3))
top=pts[pts[:,2]>1.08]; print(' bottle top centroid', top.mean(0).round(3))
# drawer front panel / handle: points near y in [-0.1,-0.02], x in [-0.2,-0.02], z 0.93-1.0
m=np.isfinite(z)&(z>0.93)&(z<1.0)&(P[...,0]>-0.2)&(P[...,0]<-0.02)&(P[...,1]>-0.12)&(P[...,1]<0.0)
pts=P[m]
for lo in np.arange(-0.12,0.0,0.005):
    s=pts[(pts[:,1]>=lo)&(pts[:,1]<lo+0.005)]
    if len(s): print(f'y {lo:+.3f}: n={len(s):3d} z max {s[:,2].max():.3f} z med {np.median(s[:,2]):.3f}')
# bowl
m=np.isfinite(z)&(z>0.905)&(z<1.0)&(P[...,0]>-0.30)&(P[...,0]<-0.08)&(P[...,1]>-0.03)&(P[...,1]<0.14)
pts=P[m]; rim=pts[pts[:,2]>0.94]
print('bowl rim x',rim[:,0].min().round(4),rim[:,0].max().round(4),'y',rim[:,1].min().round(4),rim[:,1].max().round(4),'z',np.percentile(rim[:,2],[50,95]).round(3))
"

# openrua op 35
python3 -c "
import camq, numpy as np
cx,cy=-0.175,0.0515
for cam in ['agentview','sideview','frontview']:
    try: P=camq.cloud(cam)
    except Exception as e: print(cam,e); continue
    z=P[...,2]
    m=np.isfinite(z)&(z>0.902)&(z<0.96)&(P[...,0]>-0.30)&(P[...,0]<-0.08)&(P[...,1]>-0.03)&(P[...,1]<0.14)
    pts=P[m]
    r=np.hypot(pts[:,0]-cx,pts[:,1]-cy)
    print(cam)
    for lo in np.arange(0.90,0.96,0.005):
        s=(pts[:,2]>=lo)&(pts[:,2]<lo+0.005)
        if s.sum(): print(f'  z {lo:.3f}: n={s.sum():4d} r max {r[s].max():.3f} r 90% {np.percentile(r[s],90):.3f} r min {r[s].min():.3f}')
"

# openrua op 36
timeout 60 python3 -c "
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
rclpy.init(); n=rclpy.create_node('urdf')
got=[]
qos=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
n.create_subscription(String,'/robot_description',lambda m: got.append(m.data),qos)
import time; t=time.time()
while not got and time.time()-t<10: rclpy.spin_once(n,timeout_sec=0.5)
open('robot.urdf','w').write(got[0]); print(len(got[0]))
" && grep -n -A12 '<link name="panda_hand"' robot.urdf | head -60; grep -n -A12 '<link name="panda_link7"' robot.urdf | grep -E 'collision|geometry|mesh|box|cylinder|origin' | head; grep -n -B2 -A14 '<link name="panda_leftfinger"' robot.urdf | head -40

# openrua op 37
ls /opt/ros/jazzy/share/moveit_resources_panda_description/meshes/collision/ && python3 -c "
import numpy as np, struct
def stl_bounds(path):
    data=open(path,'rb').read()
    if data[:5]==b'solid':
        import re
        v=np.array([list(map(float,m)) for m in re.findall(rb'vertex\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)',data)])
    else:
        n=struct.unpack_from('<I',data,80)[0]
        arr=np.frombuffer(data[84:84+n*50],dtype=np.dtype([('n','<3f4'),('v','<9f4'),('a','<u2')]))
        v=arr['v'].reshape(-1,3)
    return v.min(0), v.max(0)
for name in ['hand','finger','link7','link8']:
    try: print(name, [x.round(4).tolist() for x in stl_bounds(f'/opt/ros/jazzy/share/moveit_resources_panda_description/meshes/collision/{name}.stl')])
    except Exception as e: print(name, e)
"; grep -o '<joint name="panda_finger_joint1"[^/]*/>.\{0,400\}' robot.urdf | head -3; grep -o '<link name="panda_hand">.\{0,300\}' robot.urdf | head -2; grep -o 'panda_hand_joint.\{0,300\}' robot.urdf | head -2

# openrua op 38
find / -name "hand.stl" -o -name "hand.dae" -o -name "finger.stl" 2>/dev/null | head; find / -iname "*panda*" -maxdepth 4 -type d 2>/dev/null | head

# openrua op 39
cat >> rob.py <<'EOF'


def tilt_quat(theta):
    """fingers along world x; hand z points down and toward -y by theta (rad),
    so the wrist leans toward +y. theta=0 == down_quat(pi/2)."""
    c, s = np.cos(theta), np.sin(theta)
    R = np.array([[0, 1, 0], [c, 0, -s], [-s, 0, -c]])  # columns: hand x, y, z in world
    return R_to_quat(R)
EOF
timeout 900 python3 -u -c "
import rob, numpy as np
r = rob.Robot()
print('tilt0 vs down', rob.tilt_quat(0).round(4), rob.down_quat(np.pi/2).round(4))
cx, cy = -0.175, 0.0515
r.gripper(0.04)
r.move_tcp([cx-0.049, cy, 1.06], rob.down_quat(np.pi/2), 3.0)
r.move_tcp([cx-0.049, cy, 0.935], rob.down_quat(np.pi/2), 2.5)
print('wrench', r.wrench().round(2))
" 2>&1 | tail -8

# openrua op 40
timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand >/dev/null; timeout 120 python3 tools/perception/cam_snap.py frontview >/dev/null; python3 -c "
import cv2, numpy as np
a=cv2.imread('frontview.png'); b=cv2.imread('robot0_eye_in_hand.png')
cv2.imwrite('pair.png', np.hstack([a,b]))"

# openrua op 41
timeout 900 python3 -u -c "
import rob, numpy as np
r = rob.Robot()
fg = r.gripper(0.0)
print('wrench', r.wrench().round(2))
# lift
r.move_tcp([-0.224, 0.0515, 1.09], rob.down_quat(np.pi/2), 2.5)
print('fingers after lift', r.fingers(), 'wrench', r.wrench().round(2))
" 2>&1 | tail -6 && timeout 120 python3 tools/perception/cam_snap.py frontview >/dev/null && timeout 120 python3 tools/perception/cam_snap.py agentview >/dev/null && python3 -c "
import cv2, numpy as np
a=cv2.imread('frontview.png'); b=cv2.imread('agentview.png')
cv2.imwrite('pair.png', np.hstack([a,b]))"

# openrua op 42
timeout 120 python3 cam_dump.py agentview frontview birdview sideview >/dev/null && python3 -c "
import camq, numpy as np
for cam in ['agentview','frontview','sideview','birdview']:
    P=camq.cloud(cam); z=P[...,2]
    m=np.isfinite(z)&(z>1.05)&(z<1.115)&(P[...,0]>-0.35)&(P[...,0]<-0.05)&(P[...,1]>-0.05)&(P[...,1]<0.15)
    pts=P[m]
    if len(pts)<10: print(cam,'few'); continue
    rim=pts[pts[:,2]>1.09]
    print(cam,'n',len(pts),'rim x',rim[:,0].min().round(4),rim[:,0].max().round(4),'y',rim[:,1].min().round(4),rim[:,1].max().round(4),'z',np.percentile(rim[:,2],[50,95]).round(3), 'bottom z', pts[:,2].min().round(3))
"

# openrua op 43
timeout 900 python3 -u -c "
import rob, numpy as np
r = rob.Robot()
th = np.deg2rad(10)
Q = rob.tilt_quat(th)
print('R', rob.quat_to_R(Q).round(3).tolist())
wps = [[-0.164,-0.09,1.09],[-0.164,-0.153,1.045],[-0.164,-0.153,0.975]]
seed = r.arm_q()
for w in wps:
    s = r.ik(w, Q, seed=seed); print(w, None if s is None else s.round(3)); 
    if s is not None: seed = s
" 2>&1 | tail -6

# openrua op 44
timeout 900 python3 -u -c "
import rob, numpy as np
r = rob.Robot()
Q = rob.tilt_quat(np.deg2rad(10))
r.move_tcp([-0.164,-0.09,1.09], Q, 3.0)
print('fingers', r.fingers())
r.move_tcp([-0.164,-0.153,1.045], Q, 2.5)
print('fingers', r.fingers(), 'wrench', r.wrench().round(2))
" 2>&1 | tail -6 && timeout 120 python3 cam_dump.py agentview frontview sideview >/dev/null && python3 -c "
import cv2, numpy as np
a=cv2.imread('frontview.png'); b=cv2.imread('sideview.png')
cv2.imwrite('pair.png', np.hstack([a,b]))"

# openrua op 45
python3 -c "
import cv2
a=cv2.imread('frontview.png')[200:380, 120:360]; a=cv2.resize(a,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC)
b=cv2.imread('agentview.png')[100:300, 60:360]; b=cv2.resize(b,None,fx=2.4,fy=2.4,interpolation=cv2.INTER_CUBIC)
import numpy as np
h=min(a.shape[0],b.shape[0]); cv2.imwrite('pair.png', np.hstack([a[:h],b[:h]]))"

# openrua op 46
python3 -c "
import camq, numpy as np
for cam in ['frontview','agentview','sideview']:
    P=camq.cloud(cam); z=P[...,2]
    m=np.isfinite(z)&(P[...,1]<-0.17)&(P[...,1]>-0.235)&(P[...,0]>-0.24)&(P[...,0]<0.02)&(z>0.93)&(z<1.125)
    pts=P[m]
    print(cam, len(pts))
    for lo in np.arange(0.93,1.125,0.01):
        s=pts[(pts[:,2]>=lo)&(pts[:,2]<lo+0.01)]
        if len(s): print(f'  z {lo:.2f}: n={len(s):3d} y max {s[:,1].max():.3f} y med {np.median(s[:,1]):.3f}  x range {s[:,0].min():.3f} {s[:,0].max():.3f}')
"

# openrua op 47
python3 -c "
import camq, numpy as np
for z in [1.09, 1.05, 1.0, 0.95]:
    print(z, np.round(camq.world2px('frontview', (-0.2,-0.19,z)),1), np.round(camq.world2px('agentview', (-0.2,-0.19,z)),1))
# what is at frontview pixel rows 313-327, cols ~ 200-230?
for uv in [(215,290),(215,320),(210,323),(205,318)]:
    try: camq.px('frontview',*uv)
    except Exception as e: print(uv,e)
"

# openrua op 48
python3 -c "
import cv2, numpy as np
a=cv2.imread('frontview.png')
x0,y0=170,250
crop=a[y0:360, x0:330]; s=4
crop=cv2.resize(crop,None,fx=s,fy=s,interpolation=cv2.INTER_CUBIC)
for i in range(0,160,10): X=i*s; cv2.line(crop,(X,0),(X,crop.shape[0]),(0,255,0),1); cv2.putText(crop,str(x0+i),(X,10),cv2.FONT_HERSHEY_SIMPLEX,0.35,(0,255,0),1)
for j in range(0,110,10): Y=j*s; cv2.line(crop,(0,Y),(crop.shape[1],Y),(0,255,0),1); cv2.putText(crop,str(y0+j),(0,Y+10),cv2.FONT_HERSHEY_SIMPLEX,0.35,(0,255,0),1)
cv2.imwrite('crop.png',crop)"

# openrua op 49
python3 -c "
import camq
for v in range(268,320,4):
    row=[]
    for u in range(200,232,4):
        try:
            w=camq.px('frontview',u,v,verbose=False); row.append(f'{w[1]:+.3f}/{w[2]:.3f}')
        except Exception: row.append('   nan   ')
    print(v, ' '.join(row))
"

# openrua op 50
timeout 900 python3 -u -c "
import rob, numpy as np
r = rob.Robot()
Q = rob.tilt_quat(np.deg2rad(10))
print('wrench before', r.wrench().round(2), 'fingers', r.fingers())
r.move_tcp([-0.164,-0.14,1.085], Q, 2.0)
print('wrench after', r.wrench().round(2), 'fingers', r.fingers())
" 2>&1 | tail -4 && timeout 120 python3 cam_dump.py agentview frontview sideview >/dev/null && python3 -c "
import camq, numpy as np
for cam in ['agentview','frontview','sideview']:
    P=camq.cloud(cam); z=P[...,2]
    m=np.isfinite(z)&(z>1.03)&(z<1.12)&(P[...,0]>-0.30)&(P[...,0]<-0.05)&(P[...,1]>-0.24)&(P[...,1]<-0.03)
    pts=P[m]
    # exclude cabinet face/handles: y<-0.185 & z>1.07 ambiguous; print rim extents by z slices
    rim=pts[pts[:,2]>np.percentile(pts[:,2],85)]
    print(cam,'n',len(pts),'rim x',rim[:,0].min().round(4),rim[:,0].max().round(4),'y',rim[:,1].min().round(4),rim[:,1].max().round(4),'z',np.percentile(rim[:,2],[5,50,95]).round(3))
    for lo in np.arange(1.03,1.12,0.01):
        s=pts[(pts[:,2]>=lo)&(pts[:,2]<lo+0.01)]
        if len(s): print(f'   z {lo:.2f}: n={len(s):3d} y {s[:,1].min():.3f}..{s[:,1].max():.3f} x {s[:,0].min():.3f}..{s[:,0].max():.3f}')
"

# openrua op 51
python3 -c "
import cv2, numpy as np
a=cv2.imread('frontview.png')[200:380, 120:360]; a=cv2.resize(a,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC)
b=cv2.imread('sideview.png')[130:330, 150:450]; b=cv2.resize(b,None,fx=2.4,fy=2.4,interpolation=cv2.INTER_CUBIC)
h=min(a.shape[0],b.shape[0]); cv2.imwrite('pair.png', np.hstack([a[:h],b[:h]]))"

# openrua op 52
timeout 900 python3 -u -c "
import rob, numpy as np
r = rob.Robot()
Q = rob.tilt_quat(np.deg2rad(10))
r.move_tcp([-0.164,-0.06,1.10], Q, 2.5)
Q0 = rob.down_quat(np.pi/2)
r.move_tcp([-0.224,0.05,1.10], Q0, 3.0)
base = r.wrench(); print('wrench', base.round(2))
for z in [1.0, 0.975, 0.955, 0.945, 0.935]:
    r.move_tcp([-0.224,0.05,z], Q0, 1.5)
    w = r.wrench(); print('z',z,'wrench', w.round(2), 'fingers', np.round(r.fingers(),4))
    if abs(w[2]-base[2])>1.5 or abs(w[1]-base[1])>1.0: print('contact'); break
r.gripper(0.04)
r.move_tcp([-0.224,0.05,1.06], Q0, 2.0)
" 2>&1 | grep -v move_q | tail -12 && timeout 120 python3 cam_dump.py agentview frontview sideview >/dev/null && python3 -c "
import camq, numpy as np
for cam in ['agentview','frontview','sideview']:
    P=camq.cloud(cam); z=P[...,2]
    m=np.isfinite(z)&(z>0.905)&(z<1.0)&(P[...,0]>-0.32)&(P[...,0]<-0.06)&(P[...,1]>-0.06)&(P[...,1]<0.16)
    pts=P[m]; rim=pts[pts[:,2]>np.percentile(pts[:,2],85)]
    print(cam,'bowl x',pts[:,0].min().round(4),pts[:,0].max().round(4),'y',pts[:,1].min().round(4),pts[:,1].max().round(4),'rim z',np.percentile(rim[:,2],[5,50,95]).round(3),'rim x',rim[:,0].min().round(3),rim[:,0].max().round(3),'y',rim[:,1].min().round(3),rim[:,1].max().round(3))
"

# openrua op 53
python3 -c "
import cv2, numpy as np
a=cv2.imread('agentview.png'); b=cv2.imread('frontview.png')
cv2.imwrite('pair.png', np.hstack([a,b]))"

# openrua op 54
python3 -c "
import camq, numpy as np
for cam in ['agentview','frontview','sideview']:
    P=camq.cloud(cam); z=P[...,2]
    m=np.isfinite(z)&(z>0.905)&(z<1.0)&(P[...,0]>-0.32)&(P[...,0]<-0.08)&(P[...,1]>-0.02)&(P[...,1]<0.16)
    pts=P[m]; rim=pts[pts[:,2]>0.945]
    print(cam,'bowl x',pts[:,0].min().round(4),pts[:,0].max().round(4),'y',pts[:,1].min().round(4),pts[:,1].max().round(4),'rim z',np.percentile(rim[:,2],[5,50,95]).round(3),'rim x',rim[:,0].min().round(3),rim[:,0].max().round(3),'y',rim[:,1].min().round(3),rim[:,1].max().round(3))
"

# openrua op 55
timeout 900 python3 -u -c "
import rob, numpy as np
r = rob.Robot()
Q0 = rob.down_quat(np.pi/2)
cx, cy = -0.186, 0.050
r.move_tcp([cx-0.049, cy, 1.06], Q0, 2.5)
r.move_tcp([cx-0.049, cy, 0.935], Q0, 2.5)
print('wrench', r.wrench().round(2))
r.gripper(0.0)
r.move_tcp([cx-0.049, cy, 1.09], Q0, 2.5)
print('fingers', np.round(r.fingers(),4), 'wrench', r.wrench().round(2))
" 2>&1 | grep -v move_q | tail -8 && timeout 120 python3 cam_dump.py agentview frontview >/dev/null && python3 -c "
import camq, numpy as np
for cam in ['agentview','frontview']:
    P=camq.cloud(cam); z=P[...,2]
    m=np.isfinite(z)&(z>1.04)&(z<1.115)&(P[...,0]>-0.20)&(P[...,0]<-0.05)&(P[...,1]>-0.05)&(P[...,1]<0.15)
    pts=P[m]; rim=pts[pts[:,2]>1.09]
    print(cam,'rim(+x half) x max',rim[:,0].max().round(4),'y',rim[:,1].min().round(4),rim[:,1].max().round(4),'z',np.percentile(rim[:,2],[50,95]).round(3),'bottom',pts[:,2].min().round(3))
"

# openrua op 56
timeout 1200 python3 -u -c "
import rob, numpy as np
r = rob.Robot()
Q = rob.tilt_quat(np.deg2rad(10))
base = None
def step(p, t=2.0):
    global base
    r.move_tcp(p, Q, t)
    w = r.wrench(); f = np.round(r.fingers(),4)
    if base is None: base = w
    d = w - base
    print('   wrench delta', d[:3].round(2), 'fingers', f)
    return d
step([-0.164, -0.08, 1.09], 3.0)
step([-0.164, -0.13, 1.03], 2.5)
d = step([-0.164, -0.13, 1.0], 1.5)
" 2>&1 | grep -v move_q | tail -8 && timeout 120 python3 cam_dump.py frontview sideview >/dev/null && python3 -c "
import cv2, numpy as np
a=cv2.imread('frontview.png')[200:380, 120:360]; a=cv2.resize(a,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC)
b=cv2.imread('sideview.png')[130:330, 150:450]; b=cv2.resize(b,None,fx=2.4,fy=2.4,interpolation=cv2.INTER_CUBIC)
h=min(a.shape[0],b.shape[0]); cv2.imwrite('pair.png', np.hstack([a[:h],b[:h]]))"

# openrua op 57
timeout 1200 python3 -u -c "
import rob, numpy as np
r = rob.Robot()
Q = rob.tilt_quat(np.deg2rad(10))
base = r.wrench()
def step(p, t=2.0):
    r.move_tcp(p, Q, t)
    w = r.wrench(); f = np.round(r.fingers(),4)
    print('   wrench delta', (w-base)[:3].round(2), 'fingers', f)
step([-0.164, -0.13, 0.99], 1.5)
step([-0.164, -0.152, 0.973], 2.0)
r.gripper(0.04)
print('wrench after release', r.wrench()[:3].round(2))
step([-0.164, -0.152, 1.09], 2.0)
step([-0.164, -0.02, 1.15], 3.0)
" 2>&1 | grep -v move_q | tail -12 && timeout 120 python3 cam_dump.py frontview birdview >/dev/null && python3 -c "
import cv2, numpy as np
a=cv2.imread('frontview.png')[200:380, 120:360]; a=cv2.resize(a,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC)
b=cv2.imread('birdview.png')[100:340, 100:380]; b=cv2.resize(b,None,fx=2.25,fy=2.25,interpolation=cv2.INTER_CUBIC)
h=min(a.shape[0],b.shape[0]); cv2.imwrite('pair.png', np.hstack([a[:h],b[:h]]))"

# openrua op 58
python3 -c "
import camq, numpy as np
P = camq.cloud('birdview'); c = camq.load('birdview')[0]
# bowl region: inside drawer cavity, above drawer floor
m = (P[...,0]>-0.232)&(P[...,0]<0.0)&(P[...,1]>-0.23)&(P[...,1]<-0.085)&(P[...,2]>0.93)&(P[...,2]<1.0)
dark = c.mean(-1) < 90
sel = m & dark
pts = P[sel]; print('n', len(pts))
print('x', pts[:,0].min().round(3), pts[:,0].max().round(3), 'y', pts[:,1].min().round(3), pts[:,1].max().round(3), 'z', pts[:,2].min().round(3), pts[:,2].max().round(3))
top = pts[pts[:,2]>pts[:,2].max()-0.008]; print('rim center', top[:,:2].mean(0).round(3), 'rim z', top[:,2].mean().round(3))
# drawer front panel now
m2 = (P[...,0]>-0.2)&(P[...,0]<-0.03)&(P[...,1]>-0.1)&(P[...,1]<0.0)&(P[...,2]>0.93)&(P[...,2]<0.99)
q=P[m2]; print('front panel/handle y range', q[:,1].min().round(3), q[:,1].max().round(3), 'z', q[:,2].max().round(3))
"

# openrua op 59
python3 -c "
import camq, numpy as np, cv2
P = camq.cloud('birdview'); c = camq.load('birdview')[0]
m = (P[...,0]>-0.232)&(P[...,0]<0.0)&(P[...,1]>-0.23)&(P[...,1]<-0.085)&(P[...,2]>0.93)&(P[...,2]<1.0)
g = c.mean(-1)
vals = g[m]; print('gray hist', np.histogram(vals, bins=[0,40,60,80,100,120,140,160,200,255])[0])
sel = m & (g<110)
n, lab = cv2.connectedComponents(sel.astype(np.uint8))
sizes=[(lab==i).sum() for i in range(1,n)]; i=1+int(np.argmax(sizes)); print('blobs', sizes)
pts = P[lab==i]
print('bowl x', pts[:,0].min().round(3), pts[:,0].max().round(3), 'y', pts[:,1].min().round(3), pts[:,1].max().round(3), 'z', pts[:,2].min().round(3), pts[:,2].max().round(3))
print('center', pts[:,:2].mean(0).round(3))
"

# openrua op 60
python3 -c "
import camq, numpy as np, cv2
P = camq.cloud('frontview'); c = camq.load('frontview')[0]
m = (P[...,0]>-0.235)&(P[...,0]<0.01)&(P[...,1]>-0.235)&(P[...,1]<-0.06)&(P[...,2]>0.925)&(P[...,2]<1.0)
g = c.mean(-1)
print('gray hist', np.histogram(g[m], bins=[0,40,60,80,100,120,140,160,200,255])[0])
sel = m & (g<120)
n, lab = cv2.connectedComponents(sel.astype(np.uint8))
sizes=[(lab==i).sum() for i in range(1,n)]; i=1+int(np.argmax(sizes)); print('blobs', sorted(sizes)[-4:])
pts = P[lab==i]
print('bowl x', pts[:,0].min().round(3), pts[:,0].max().round(3), 'y', pts[:,1].min().round(3), pts[:,1].max().round(3), 'z', pts[:,2].min().round(3), pts[:,2].max().round(3))
top = pts[pts[:,2]>pts[:,2].max()-0.006]; print('rim center xy', top[:,:2].mean(0).round(3), 'rim z', top[:,2].mean().round(3), 'n', len(top))
"

# openrua op 61
timeout 300 python3 -c "
import rob, numpy as np
r = rob.Robot()
Q = rob.tilt_quat(np.deg2rad(60))
for p in ([-0.114,-0.01,0.975],[-0.114,-0.10,0.975],[-0.114,-0.195,0.975]):
    q = r.ik(p, Q)
    if q is None: print(p,'IK none'); continue
    hp,_ = r.fk(q); tp,_ = r.tcp(q); l7,_ = r.fk(q,'panda_link7')
    print(p, 'ok q7=%.2f'%q[6], 'hand', hp.round(3), 'tcp', tp.round(3), 'link7', l7.round(3))
" 2>&1 | tail -4

# openrua op 62
timeout 1500 python3 -u -c "
import rob, numpy as np
r = rob.Robot()
Q = rob.tilt_quat(np.deg2rad(60))
r.gripper(0.0)
r.move_tcp([-0.114, 0.05, 1.12], Q, 3.0)
r.move_tcp([-0.114, -0.01, 0.975], Q, 2.5)
base = r.wrench(); print('base wrench', base[:3].round(2))
y = -0.01
while y > -0.20:
    y = max(y-0.02, -0.20)
    q = r.ik([-0.114, y, 0.975], Q)
    code, err = r.move_q(q, 1.2)
    p,_ = r.tcp(); w = r.wrench()-base
    print(f'y_cmd={y:.3f} tcp={p.round(3)} dF={w[:3].round(1)}')
    if abs(p[1]-y) > 0.006 or abs(w[1]) > 15:
        print('STOP: lag/force'); break
" 2>&1 | grep -v move_q

# openrua op 63
timeout 600 python3 -u -c "
import rob, numpy as np
r = rob.Robot()
Q = rob.tilt_quat(np.deg2rad(60))
print('q now', r.arm_q().round(2))
q = r.ik([-0.114, -0.01, 0.975], Q); print('q ik ', q.round(2))
r.move_tcp([-0.114, -0.01, 0.975], Q, 3.0)
print('q now', r.arm_q().round(2), 'wrench', r.wrench()[:3].round(2))
" 2>&1

# openrua op 64
timeout 120 python3 cam_dump.py agentview sideview >/dev/null && python3 -c "
import cv2, numpy as np
a=cv2.imread('agentview.png'); b=cv2.imread('sideview.png')
cv2.imwrite('pair.png', np.hstack([a,b]))"

# openrua op 65
timeout 600 python3 -c "
import rob, numpy as np
r = rob.Robot()
Q = rob.tilt_quat(np.deg2rad(60))
seeds = [[0,-0.5,0,-2.2,0,1.9,0.8],[0.3,0.2,-0.3,-2.0,0.2,2.5,0.5],[-0.5,0.3,0.5,-1.8,-0.5,2.4,1.0],[0,0.5,0,-1.5,0,2.0,0],[0.5,-0.3,-0.5,-2.4,0.5,2.2,-0.5],[-1,0,1,-2,0,2.3,0.5]]
sols=[]
for s in seeds:
    for p in ([-0.114,-0.01,0.975],[-0.114,-0.195,0.975]):
        q = r.ik(p, Q, seed=np.array(s,float))
        print(p, None if q is None else q.round(2))
" 2>&1 | tail -12

# openrua op 66
timeout 600 python3 -c "
import rob, numpy as np
r = rob.Robot()
Q = rob.tilt_quat(np.deg2rad(60))
q0 = r.arm_q()
seed = np.array([0.54,0.69,-0.3,-1.98,-0.93,2.03,0.19])
qh = r.ik([-0.114, 0.05, 1.25], Q, seed=seed); print('high pose q', qh.round(2))
for a in np.linspace(0,1,11):
    q = q0*(1-a)+qh*a
    out=[]
    for l in ['panda_link4','panda_link6','panda_hand']:
        p,_ = r.fk(q,l); out.append(p.round(2))
    t,_ = r.tcp(q)
    print(f'{a:.1f} link4 {out[0]} link6 {out[1]} hand {out[2]} tcp {t.round(2)}')
" 2>&1 | tail -12

# openrua op 67
timeout 1500 python3 -u -c "
import rob, numpy as np
r = rob.Robot()
Q = rob.tilt_quat(np.deg2rad(60))
seed = np.array([0.54,0.69,-0.3,-1.98,-0.93,2.03,0.19])
qh = r.ik([-0.114, 0.05, 1.25], Q, seed=seed)
r.move_q(qh, 5.0); p,_=r.tcp(); print('tcp', p.round(3))
r.move_tcp([-0.114, -0.01, 1.05], Q, 3.0, seed=seed)
r.move_tcp([-0.114, -0.01, 0.975], Q, 2.0)
print('q', r.arm_q().round(2))
base = r.wrench(); print('base wrench', base[:3].round(2))
y = -0.01
while y > -0.20:
    y = max(y-0.02, -0.20)
    q = r.ik([-0.114, y, 0.975], Q)
    code, err = r.move_q(q, 1.2)
    p,_ = r.tcp(); w = r.wrench()-base
    print(f'y_cmd={y:.3f} tcp={p.round(3)} dF={w[:3].round(1)}')
    if abs(p[1]-y) > 0.006 or abs(w[1]) > 15:
        print('STOP: lag/force'); break
" 2>&1 | grep -v move_q

# openrua op 68
timeout 120 python3 cam_dump.py agentview frontview robot0_eye_in_hand >/dev/null && python3 -c "
import cv2, numpy as np
a=cv2.imread('agentview.png')[100:340, 150:450]; a=cv2.resize(a,None,fx=2,fy=2,interpolation=cv2.INTER_CUBIC)
b=cv2.imread('frontview.png')[150:390, 100:400]; b=cv2.resize(b,None,fx=2,fy=2,interpolation=cv2.INTER_CUBIC)
cv2.imwrite('pair.png', np.hstack([a,b]))"

# openrua op 69
python3 -c "
import camq, numpy as np
P = camq.cloud('frontview'); c = camq.load('frontview')[0]
g=c.mean(-1)
# bottle: dark green, x>0.0, y around -0.035, z 0.9-1.13
m=(P[...,0]>0.0)&(P[...,0]<0.12)&(P[...,1]>-0.12)&(P[...,1]<0.05)&(P[...,2]>0.92)&(P[...,2]<1.15)&(g<80)
p=P[m]; print('bottle n',len(p),'x',p[:,0].min().round(3),p[:,0].max().round(3),'y',p[:,1].min().round(3),p[:,1].max().round(3),'z',p[:,2].max().round(3))
# hand (white/bright) points near bottle x>-0.02
m=(P[...,0]>-0.03)&(P[...,0]<0.12)&(P[...,1]>-0.1)&(P[...,1]<0.15)&(P[...,2]>0.95)&(P[...,2]<1.2)&(g>150)
p=P[m]; print('hand n',len(p),'x',p[:,0].min().round(3),p[:,0].max().round(3),'y',p[:,1].min().round(3),p[:,1].max().round(3),'z',p[:,2].min().round(3),p[:,2].max().round(3))
"; python3 -c "
import cv2; a=cv2.imread('robot0_eye_in_hand.png'); cv2.imwrite('eih.png', cv2.resize(a,None,fx=2,fy=2))"

# openrua op 70
python3 -c "
import camq, numpy as np
P = camq.cloud('frontview'); c = camq.load('frontview')[0]
g=c.mean(-1)
for lo,hi in [(-0.1,-0.05),(-0.05,0.0),(0.0,0.05),(0.05,0.1),(0.1,0.15)]:
    m=(P[...,0]>-0.3)&(P[...,0]<0.2)&(P[...,1]>lo)&(P[...,1]<hi)&(P[...,2]>0.99)&(P[...,2]<1.2)&(g>150)
    p=P[m]
    if len(p): print(f'y[{lo},{hi}] bright n={len(p)} x {p[:,0].min():.3f}..{p[:,0].max():.3f} z {p[:,2].min():.3f}..{p[:,2].max():.3f}')
"

# openrua op 71
timeout 300 python3 -c "
import rob, numpy as np
r = rob.Robot()
Q = rob.tilt_quat(np.deg2rad(60))
q0 = r.arm_q(); qt = r.ik([-0.114,-0.01,0.975], Q)
print('q0', q0.round(2)); print('qt', qt.round(2))
for l in ['panda_link0','panda_link1','panda_link2','panda_link3','panda_link4','panda_link5','panda_link6','panda_link7','panda_hand']:
    a,_=r.fk(q0,l); b,_=r.fk(qt,l); print(f'{l:12s} now {a.round(3)}  target {b.round(3)}')
" 2>&1 | tail -11

# openrua op 72
timeout 120 python3 cam_dump.py birdview >/dev/null && python3 -c "
import camq, numpy as np
P = camq.cloud('birdview'); c = camq.load('birdview')[0].astype(int)
b,g,r = c[...,0],c[...,1],c[...,2]
wood = (r>g+15)&(g>b+5)   # brownish
m = wood & (P[...,2]>0.95) & (P[...,2]<1.6) & (P[...,1]>-0.3)
p=P[m]; print('wood n',len(p))
for zlo in np.arange(0.95,1.6,0.05):
    s=p[(p[:,2]>=zlo)&(p[:,2]<zlo+0.05)]
    if len(s)>20: print(f'z[{zlo:.2f},{zlo+0.05:.2f}] n={len(s)} x {s[:,0].min():.2f}..{s[:,0].max():.2f} y {s[:,1].min():.2f}..{s[:,1].max():.2f}')
"

# openrua op 73
timeout 1500 python3 -u -c "
import rob, numpy as np
r = rob.Robot()
Q60 = rob.tilt_quat(np.deg2rad(60)); Q = rob.tilt_quat(np.deg2rad(20))
r.move_tcp([-0.10, -0.02, 1.12], Q60, 2.5)
r.move_tcp([-0.114, 0.0, 1.12], Q, 3.0)
for l in ['panda_link7','panda_hand']:
    p,_=r.fk(None,l); print(l, p.round(3))
r.move_tcp([-0.114, 0.0, 1.0], Q, 2.5)
r.move_tcp([-0.114, 0.0, 0.952], Q, 1.5)
base = r.wrench(); print('base wrench', base[:3].round(2), 'fingers', np.round(r.fingers(),4))
y = 0.0
while y > -0.17:
    y = max(y-0.02, -0.17)
    q = r.ik([-0.114, y, 0.952], Q)
    code, err = r.move_q(q, 1.2)
    p,_ = r.tcp(); w = r.wrench()-base
    print(f'y_cmd={y:.3f} tcp={p.round(3)} dF={w[:3].round(1)}')
    if abs(p[1]-y) > 0.006 or abs(w[1]) > 15:
        print('STOP: lag/force'); break
" 2>&1 | grep -v move_q

# openrua op 74
timeout 120 python3 cam_dump.py agentview frontview >/dev/null && python3 -c "
import cv2, numpy as np
a=cv2.imread('agentview.png')[100:340, 150:450]; a=cv2.resize(a,None,fx=2,fy=2,interpolation=cv2.INTER_CUBIC)
b=cv2.imread('frontview.png')[150:390, 100:400]; b=cv2.resize(b,None,fx=2,fy=2,interpolation=cv2.INTER_CUBIC)
cv2.imwrite('pair.png', np.hstack([a,b]))"

# openrua op 75
timeout 600 python3 -u -c "
import rob, numpy as np
r = rob.Robot()
Q = rob.tilt_quat(np.deg2rad(20))
r.move_tcp([-0.114, -0.17, 1.10], Q, 2.0)
r.move_tcp([-0.114, 0.05, 1.20], Q, 3.0)
print('fingers', np.round(r.fingers(),4), 'wrench', r.wrench()[:3].round(2))
" 2>&1 | grep -v move_q && timeout 120 python3 cam_dump.py frontview birdview sideview >/dev/null && python3 -c "
import camq, numpy as np
P = camq.cloud('frontview')
# bottom drawer front region: x in cabinet, z 0.93..0.98: find the most +y surface
m=(P[...,0]>-0.22)&(P[...,0]<-0.02)&(P[...,2]>0.93)&(P[...,2]<0.98)&(P[...,1]>-0.4)&(P[...,1]<0.1)
p=P[m]; print('bottom-drawer band: y max %.3f, 99pct %.3f'%(p[:,1].max(), np.percentile(p[:,1],99)))
# handle points (protrude most): 
print('y hist', np.histogram(p[:,1], bins=np.arange(-0.24,-0.14,0.01))[0])
# cabinet upper face for reference z 1.04..1.07 (between handles)
m2=(P[...,0]>-0.22)&(P[...,0]<-0.02)&(P[...,2]>1.04)&(P[...,2]<1.07)&(P[...,1]>-0.4)&(P[...,1]<0.1)
q=P[m2]; print('cabinet face ref band y: median %.3f max %.3f'%(np.median(q[:,1]), q[:,1].max()))
# anything (bowl) on the table in front of cabinet?
m3=(P[...,0]>-0.3)&(P[...,0]<0.02)&(P[...,1]>-0.22)&(P[...,1]<0.15)&(P[...,2]>0.91)&(P[...,2]<1.0)
print('points above table in front of cabinet:', int(m3.sum()))
"

# openrua op 76
python3 -c "
import camq, numpy as np
P = camq.cloud('frontview')
def band(zlo,zhi,label):
    m=(P[...,0]>-0.22)&(P[...,0]<-0.02)&(P[...,2]>zlo)&(P[...,2]<zhi)&(P[...,1]>-0.4)&(P[...,1]<0.1)
    p=P[m]; print(f'{label:28s} z[{zlo},{zhi}] n={len(p)} y median {np.median(p[:,1]):.3f} max {p[:,1].max():.3f}')
band(0.93,0.94,'bottom panel below handle')
band(0.965,0.98,'bottom panel above handle')
band(0.99,1.01,'middle panel below handle')
band(1.04,1.07,'middle/top panel between')
band(1.105,1.12,'top panel above handle')
# anything on the table in front of cabinet, excluding drawer fronts (y>-0.17) and bottle (x>0.01)
m3=(P[...,0]>-0.3)&(P[...,0]<0.01)&(P[...,1]>-0.17)&(P[...,1]<0.15)&(P[...,2]>0.905)&(P[...,2]<1.0)
p=P[m3]; print('objects on table in front of cabinet:', len(p), (p[:,:3].min(0).round(3), p[:,:3].max(0).round(3)) if len(p) else '')
"

# openrua op 77
python3 -c "
import camq, numpy as np
P = camq.cloud('birdview')
m=(P[...,0]>-0.22)&(P[...,0]<-0.02)&(P[...,2]>0.93)&(P[...,2]<1.0)&(P[...,1]>-0.3)&(P[...,1]<0.0)
p=P[m]; print('birdview: pts at drawer-front height n=%d y range %.3f..%.3f'%(len(p),p[:,1].min(),p[:,1].max()) if len(p) else 'none')
m=(P[...,0]>-0.22)&(P[...,0]<-0.02)&(P[...,2]>1.1)&(P[...,1]>-0.3)&(P[...,1]<0.0)
p=P[m]; print('birdview: cabinet top y max %.3f'%p[:,1].max())
P = camq.cloud('sideview')
m=(P[...,0]>-0.22)&(P[...,0]<-0.02)&(P[...,2]>0.93)&(P[...,2]<0.985)&(P[...,1]>-0.3)&(P[...,1]<0.0)
p=P[m]; print('sideview: bottom drawer front y: median %.3f max %.3f n=%d'%(np.median(p[:,1]),p[:,1].max(),len(p)))
m=(P[...,0]>-0.22)&(P[...,0]<-0.02)&(P[...,2]>1.04)&(P[...,2]<1.07)&(P[...,1]>-0.3)&(P[...,1]<0.0)
p=P[m]; print('sideview: upper face y: median %.3f max %.3f'%(np.median(p[:,1]),p[:,1].max()))
"

# openrua op 78
timeout 900 python3 -u -c "
import rob, numpy as np
r = rob.Robot()
Q = rob.tilt_quat(np.deg2rad(20))
r.move_tcp([-0.114, -0.16, 1.05], Q, 3.0)
r.move_tcp([-0.114, -0.16, 0.952], Q, 1.5)
base = r.wrench(); print('base', base[:3].round(2))
for y in (-0.17,-0.175,-0.18,-0.185):
    q = r.ik([-0.114, y, 0.952], Q); r.move_q(q, 1.0)
    p,_ = r.tcp(); w = r.wrench()-base
    print(f'y_cmd={y:.3f} tcp y={p[1]:.4f} dF={w[:3].round(1)}')
    if abs(p[1]-y) > 0.004 or abs(w[1]) > 15: print('STOP'); break
r.move_tcp([-0.114, p[1], 1.10], Q, 2.0)
r.move_tcp([-0.114, 0.05, 1.20], Q, 3.0)
" 2>&1 | grep -v move_q && timeout 120 python3 cam_dump.py frontview sideview >/dev/null && python3 -c "
import camq, numpy as np
for cam in ('frontview','sideview'):
    P = camq.cloud(cam)
    m=(P[...,0]>-0.22)&(P[...,0]<-0.02)&(P[...,2]>0.93)&(P[...,2]<0.94)&(P[...,1]>-0.3)&(P[...,1]<0.0)
    p=P[m]; print(cam,'bottom panel y median %.3f'%np.median(p[:,1]))
    m=(P[...,0]>-0.22)&(P[...,0]<-0.02)&(P[...,2]>1.04)&(P[...,2]<1.07)&(P[...,1]>-0.3)&(P[...,1]<0.0)
    p=P[m]; print(cam,'upper face y median %.3f'%np.median(p[:,1]))
"

# openrua op 79
timeout 120 python3 cam_dump.py agentview frontview birdview >/dev/null && python3 -c "
import cv2, numpy as np
a=cv2.imread('agentview.png')[60:380, 100:460]
b=cv2.imread('frontview.png')[120:400, 60:420]
c=cv2.imread('birdview.png')[120:340, 120:340]; c=cv2.resize(c,(c.shape[1]*320//220, 320))
h=320; a=cv2.resize(a,(a.shape[1]*h//a.shape[0],h)); b=cv2.resize(b,(b.shape[1]*h//b.shape[0],h))
cv2.imwrite('pair.png', np.hstack([a,b,c]))
import camq
P=camq.cloud('frontview')
m=(P[...,0]>-0.3)&(P[...,0]<0.01)&(P[...,1]>-0.17)&(P[...,1]<0.15)&(P[...,2]>0.905)&(P[...,2]<1.05)
print('non-table points in front of cabinet (bowl would show here):', int(m.sum()))
"
