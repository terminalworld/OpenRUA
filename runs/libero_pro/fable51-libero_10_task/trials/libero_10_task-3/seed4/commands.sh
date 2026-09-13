#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la && find . -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
cat /workspace/tools/action/fjt_send.py; echo ----; cat /workspace/tools/action/gripper_cmd.py; echo ----; cat /workspace/tools/action/ik_move.py

# openrua op 3
ros2 topic list; echo ----; ros2 action list; echo ----; ros2 service list | grep -iv parameter | head -40; echo ----; ros2 node list

# openrua op 4
mkdir -p snaps && for c in agentview frontview sideview birdview robot0_eye_in_hand robot0_robotview; do timeout 60 python3 tools/perception/cam_snap.py $c snaps/$c.png; done; ros2 topic echo /joint_states --once

# openrua op 5
timeout 30 ros2 run tf2_ros tf2_echo world panda_hand 2>&1 | head -12; echo ----; timeout 20 ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id" | sort | uniq

# openrua op 6
timeout 20 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | grep -A2 "Translation" | head -3; timeout 20 ros2 run tf2_ros tf2_echo world agentview_optical_frame 2>&1 | grep -A3 "Translation" | head -4; timeout 20 ros2 run tf2_ros tf2_echo world birdview_optical_frame 2>&1 | grep -A3 "Translation" | head -4

# openrua op 7
mkdir -p "$(dirname /workspace/scan.py)"
cat > /workspace/scan.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Grab color+depth+intrinsics+TF for a camera; save world XYZ map (HxWx3) and color PNG.

Usage: python3 scan.py <camera> [prefix]
Outputs: <prefix>_color.png, <prefix>_xyz.npy (world coords, NaN where invalid)
"""
import sys
import numpy as np
import rclpy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener


def grab(node, topic, msg_type, timeout=30.0):
    got = {}
    sub = node.create_subscription(msg_type, topic, lambda m: got.setdefault("m", m), 1)
    import time
    end = time.time() + timeout
    while "m" not in got and time.time() < end:
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
    cam = sys.argv[1]
    prefix = sys.argv[2] if len(sys.argv) > 2 else f"snaps/{cam}"
    rclpy.init()
    node = rclpy.create_node("scan")
    tfbuf = Buffer()
    TransformListener(tfbuf, node)
    br = CvBridge()
    color = br.imgmsg_to_cv2(grab(node, f"/{cam}/color/image_raw", Image), "bgr8")
    depth = br.imgmsg_to_cv2(grab(node, f"/{cam}/depth/image_raw", Image), "passthrough").astype(np.float64)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    frame = f"{cam}_optical_frame"
    import time
    end = time.time() + 20
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
        if tfbuf.can_transform("world", frame, rclpy.time.Time()):
            break
    t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    R = quat_to_R(q.x, q.y, q.z, q.w)
    T = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    H, W = depth.shape
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    z = depth
    X = (u - cx) * z / fx
    Y = (v - cy) * z / fy
    pc = np.stack([X, Y, z], -1) @ R.T + T
    bad = ~np.isfinite(z) | (z <= 0)
    pc[bad] = np.nan
    cv2.imwrite(prefix + "_color.png", color)
    np.save(prefix + "_xyz.npy", pc)
    print(prefix, "cam at", T, "H W", H, W)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 8
timeout 120 python3 scan.py birdview && timeout 120 python3 scan.py agentview && timeout 120 python3 scan.py frontview

# openrua op 9
python3 -c "
import numpy as np, cv2
pc=np.load('snaps/birdview_xyz.npy'); img=cv2.imread('snaps/birdview_color.png')
z=pc[...,2]
print('table z median:', np.nanmedian(z[300:450,150:450]))
# height map above table
tab=np.nanmedian(z[350:450,150:450])
hm=z-tab
# find things > 3cm above table within table region
ys,xs=np.where(hm>0.03)
print('raised px count',len(ys))
# bottle: dark object near (325,262) in birdview
for (u,v) in [(325,262),(322,258),(303,300),(360,300),(410,300),(280,270),(270,300)]:
    print((u,v), pc[v,u], img[v,u])
"


# openrua op 10
python3 -c "
import numpy as np, cv2
pc=np.load('snaps/birdview_xyz.npy'); img=cv2.imread('snaps/birdview_color.png')
z=pc[...,2]
# print a coarse grid of height over the cabinet/drawer region
np.set_printoptions(linewidth=250, precision=3, suppress=True)
vs=range(250,350,6); us=range(330,450,6)
print('     '+' '.join(f'{u:6d}' for u in us))
for v in vs:
    print(f'{v:4d} '+' '.join(f'{z[v,u]:6.3f}' for u in us))
print()
print('y coords along row 300:')
print(' '.join(f'{pc[300,u,1]:6.3f}' for u in us))
print('x coords along col 380:')
print(' '.join(f'{pc[v,380,0]:6.3f}' for v in vs))
"


# openrua op 11
python3 -c "
import numpy as np, cv2
pc=np.load('snaps/birdview_xyz.npy'); img=cv2.imread('snaps/birdview_color.png')
z=pc[...,2]
def bbox(mask,name):
    P=pc[mask]; print(name, 'n',mask.sum(), 'x[%.3f %.3f] y[%.3f %.3f] z[%.3f %.3f]'%(np.nanmin(P[:,0]),np.nanmax(P[:,0]),np.nanmin(P[:,1]),np.nanmax(P[:,1]),np.nanmin(P[:,2]),np.nanmax(P[:,2])))
tableish = (pc[...,0]>-0.4)&(pc[...,0]<0.5)&(np.abs(pc[...,1])<0.6)
bbox((np.abs(z-1.126)<0.005)&tableish,'cabinet top')
bbox((np.abs(z-0.924)<0.004)&tableish,'drawer floor')
bbox((np.abs(z-0.982)<0.004)&tableish,'drawer walls')
bbox((np.abs(z-0.968)<0.004)&tableish,'drawer back wall?')
bbox((np.abs(z-0.952)<0.004)&tableish&(pc[...,1]<0.09),'handle?')
bbox((np.abs(z-1.023)<0.004)&tableish,'thing in drawer z1.023')
bbox((np.abs(z-1.094)<0.004)&tableish,'z1.094')
# bottle: dark pixels raised above table near center
dark=(img.sum(-1)<60)&(z>0.95)&(z<1.1)&tableish
bbox(dark,'bottle dark')
bottle=(z>0.93)&(z<1.08)&(pc[...,1]<0.07)&(pc[...,1]>-0.1)&(pc[...,0]>-0.25)&(pc[...,0]<0.0)
bbox(bottle,'bottle raised')
P=pc[bottle]; print('bottle centroid', np.nanmean(P,0))
"


# openrua op 12
python3 -c "
import numpy as np, cv2
pc=np.load('snaps/frontview_xyz.npy'); img=cv2.imread('snaps/frontview_color.png')
np.set_printoptions(linewidth=250, precision=3, suppress=True)
# bottle region in frontview ~ u 325-345, v 260-340
for v in range(255,345,5):
    row=[]
    for u in range(322,348,4):
        p=pc[v,u]; row.append(f'({p[0]:.2f},{p[1]:.2f},{p[2]:.2f})' if np.isfinite(p[0]) else 'nan')
    print(v, ' '.join(row))
"


# openrua op 13
python3 -c "
import numpy as np, cv2
pc=np.load('snaps/frontview_xyz.npy')
for v in [270,290,310,320,330]:
    us=[u for u in range(300,370) if np.isfinite(pc[v,u,0]) and pc[v,u,0]>-0.2 and pc[v,u,0]<0.0]
    ys=pc[v,us,1]; print(v, 'u',min(us),max(us), 'y %.3f..%.3f'%(ys.min(),ys.max()), 'x %.3f'%pc[v,us,0].min(), 'z %.3f'%pc[v,us[0],2])
# body top
for u in [330,334,338]:
    vs=[v for v in range(240,345) if np.isfinite(pc[v,u,0]) and pc[v,u,0]>-0.2 and pc[v,u,0]<0.0]
    print('col',u,'v',min(vs),max(vs),'z %.3f..%.3f'%(pc[max(vs),u,2],pc[min(vs),u,2]))
"


# openrua op 14
python3 -c "
import numpy as np, cv2
pc=np.load('snaps/birdview_xyz.npy'); img=cv2.imread('snaps/birdview_color.png')
z=pc[...,2]
reg=(pc[...,0]>-0.3)&(pc[...,0]<0.05)&(pc[...,1]>-0.1)&(pc[...,1]<0.09)
for lo,hi in [(1.10,1.2),(1.05,1.10),(1.0,1.05),(0.95,1.0),(0.91,0.95)]:
    m=reg&(z>lo)&(z<hi); P=pc[m]
    if len(P): print(lo,hi,'n',len(P),'x %.3f..%.3f y %.3f..%.3f'%(P[:,0].min(),P[:,0].max(),P[:,1].min(),P[:,1].max()), 'mean', P[:,:2].mean(0))
m=reg&(z>1.10); ys,xs=np.where(m); print('px', xs.min(),xs.max(),ys.min(),ys.max())
"


# openrua op 15
python3 -c "
import numpy as np, cv2
pc=np.load('snaps/birdview_xyz.npy'); z=pc[...,2]
reg=(pc[...,0]>-0.3)&(pc[...,0]<-0.04)&(pc[...,1]>-0.03)&(pc[...,1]<0.09)
for lo,hi in [(1.05,1.10),(1.0,1.05),(0.95,1.0),(0.91,0.95)]:
    m=reg&(z>lo)&(z<hi); P=pc[m]
    if len(P): print(lo,hi,'n',len(P),'x %.3f..%.3f y %.3f..%.3f'%(P[:,0].min(),P[:,0].max(),P[:,1].min(),P[:,1].max()), 'mean', P[:,:2].mean(0))
"


# openrua op 16
python3 -c "
import numpy as np
pc=np.load('snaps/birdview_xyz.npy'); z=pc[...,2]
print('row 300 across y:')
for u in range(336,396,1):
    print(f'  u{u} y={pc[300,u,1]:.3f} z={z[300,u]:.3f}')
" | awk 'NR==1 || NR%1==0' | head -70

# openrua op 17
python3 -c "
import numpy as np
pc=np.load('snaps/birdview_xyz.npy'); z=pc[...,2]
print('col 365 across x:')
for v in range(262,336,1):
    print(f'  v{v} x={pc[v,365,0]:.3f} z={z[v,365]:.3f}')
print('cabinet col 420:')
for v in list(range(262,272))+list(range(336,348)):
    print(f'  v{v} x={pc[v,420,0]:.3f} z={z[v,420]:.3f}')
"

# openrua op 18
python3 -c "
import numpy as np
pc=np.load('snaps/agentview_xyz.npy'); z=pc[...,2]
m=(np.abs(z-0.924)<0.006)&(pc[...,0]>-0.1)&(pc[...,0]<0.11)&(pc[...,1]>0.05)&(pc[...,1]<0.5)
P=pc[m]; print('floor pts', len(P), 'y max %.3f'%P[:,1].max(), 'x %.3f..%.3f'%(P[:,0].min(),P[:,0].max()))
# back wall of drawer: vertical surface with y ~ constant >0.2, z 0.93..0.98
m2=(z>0.93)&(z<0.985)&(pc[...,0]>-0.09)&(pc[...,0]<0.10)&(pc[...,1]>0.2)&(pc[...,1]<0.5)
P2=pc[m2]; print('inner wall pts', len(P2)); 
if len(P2): print('y hist', np.histogram(P2[:,1], bins=10)) 
# look at cabinet opening: points x in drawer range, y in 0.2..0.26, z 0.98..1.13
m3=(z>0.985)&(z<1.12)&(pc[...,0]>-0.09)&(pc[...,0]<0.10)&(pc[...,1]>0.15)&(pc[...,1]<0.3)
P3=pc[m3]; print('cabinet face pts', len(P3)); 
if len(P3): print('z hist', np.histogram(P3[:,2], bins=8)); print('y range %.3f..%.3f'%(P3[:,1].min(),P3[:,1].max()))
"

# openrua op 19
python3 -c "
import numpy as np
# bottle 2D profile: body from s=0..0.17 (halfwidth 0.02), neck s=0.17..0.23 (halfwidth 0.0075); s along axis from base
def pts(cx,cy,th,shrink=0):
    P=[]
    for s0,s1,hw in [(0,0.17,0.02),(0.17,0.23,0.0075)]:
        for s in np.linspace(s0,s1,10):
            for w in (-hw,hw):
                P.append((s,w))
    P=np.array(P); P[:,0]-=0.115
    R=np.array([[np.cos(th),-np.sin(th)],[np.sin(th),np.cos(th)]])
    return P@R.T+[cx,cy]
box=(-0.096,0.108,0.082,0.24)
best=None
for th in np.radians(np.arange(0,180,1)):
    for cx in np.arange(-0.03,0.04,0.002):
        for cy in np.arange(0.12,0.21,0.002):
            Q=pts(cx,cy,th)
            m=min(Q[:,0].min()-box[0], box[1]-Q[:,0].max(), Q[:,1].min()-box[2], box[3]-Q[:,1].max())
            if best is None or m>best[0]: best=(m,cx,cy,np.degrees(th))
print('best clearance %.4f at center (%.3f,%.3f) angle %.0f deg'%best)
"

# openrua op 20
python3 -c "
import cv2
img=cv2.imread('snaps/agentview_color.png')
crop=img[150:400,320:640]; cv2.imwrite('snaps/agent_crop.png', cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_NEAREST))
"

# openrua op 21
ros2 interface show moveit_msgs/srv/GetPositionIK | head -60; ros2 interface show moveit_msgs/srv/GetPositionFK | head -30

# openrua op 22
ros2 interface show moveit_msgs/srv/GetPositionIK 2>/dev/null | grep -vE "^\s*#" | grep -E "^\s{0,2}[a-zA-Z]|ik_link|timeout|avoid|pose_stamped|constraints" | head -40

# openrua op 23
mkdir -p "$(dirname /workspace/rob.py)"
cat > /workspace/rob.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Robot helper: world-frame IK moves, FK, gripper, joint state.

Usage:
  python3 rob.py js                         # joint state dict
  python3 rob.py fk                         # hand pose in world (via /compute_fk)
  python3 rob.py move x y z qx qy qz qw [--t 3] [--tcp] [--via]  # IK -> trajectory
  python3 rob.py joints j1,...,j7 [--t 3]   # raw joint target
  python3 rob.py grip open|close|<width_m>
World->base offset from TF: base panda_link0 at world (-0.66, 0, 0.912), same orientation.
"""
import sys
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

ARM = [f"panda_joint{i}" for i in range(1, 8)]
BASE_IN_WORLD = np.array([-0.66, 0.0, 0.912])
TCP_OFF = 0.1034
LIMITS = [(-2.9, 2.9), (-1.76, 1.76), (-2.9, 2.9), (-3.07, -0.07), (-2.9, 2.9), (-0.02, 3.75), (-2.9, 2.9)]


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def R_to_quat(R):
    # returns x,y,z,w
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        return np.array([(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s])
    i = np.argmax(np.diag(R))
    if i == 0:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return np.array([0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s])
    if i == 1:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        return np.array([(R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s])
    s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
    return np.array([(R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s, (R[1, 0] - R[0, 1]) / s])


class Robot:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("rob_helper")
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.ik = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt = ActionClient(self.node, FollowJointTrajectory, "/panda_arm_controller/follow_joint_trajectory")
        self.grip = ActionClient(self.node, GripperCommand, "/franka_gripper/gripper_action")

    def _on_js(self, msg):
        self._js = dict(zip(msg.name, msg.position))
        self._js_msg = msg

    def spin(self, t):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joint_state(self, fresh=True):
        if fresh:
            self._js = {}
        end = time.time() + 15
        while not self._js and time.time() < end:
            self.spin(0.2)
        if not self._js:
            raise RuntimeError("no /joint_states")
        return dict(self._js)

    def arm_q(self):
        js = self.joint_state()
        return np.array([js[j] for j in ARM])

    def finger(self):
        js = self.joint_state()
        return js.get("panda_finger_joint1"), js.get("panda_finger_joint2")

    def _seed(self, q=None):
        q = self.arm_q() if q is None else q
        s = JointState()
        s.name = list(ARM)
        s.position = [float(v) for v in q]
        return s

    def fk_world(self, q=None, link="panda_hand"):
        if not self.fk.wait_for_service(timeout_sec=10):
            raise RuntimeError("no FK service")
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = [link]
        req.robot_state.joint_state = self._seed(q)
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {None if res is None else res.error_code.val}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE_IN_WORLD
        quat = np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        return pos, quat

    def ik_world(self, pos, quat, seed=None, tcp=False, timeout=5.0, avoid=False):
        pos = np.array(pos, float)
        if tcp:
            R = quat_to_R(*quat)
            pos = pos - TCP_OFF * R[:, 2]
        if not self.ik.wait_for_service(timeout_sec=10):
            raise RuntimeError("no IK service")
        req = GetPositionIK.Request()
        req.ik_request.group_name = "panda_arm"
        req.ik_request.ik_link_name = "panda_hand"
        req.ik_request.avoid_collisions = avoid
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        b = pos - BASE_IN_WORLD
        p.position.x, p.position.y, p.position.z = map(float, b)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.robot_state.joint_state = self._seed(seed)
        req.ik_request.timeout = Duration(sec=int(timeout), nanosec=int((timeout % 1) * 1e9))
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=90)
        res = fut.result()
        if res is None:
            raise RuntimeError("IK no answer")
        if res.error_code.val != 1:
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return np.array([sol[j] for j in ARM])

    def traj(self, points, times):
        """points: list of 7-vectors; times: list of seconds (cumulative)."""
        if not self.fjt.wait_for_server(timeout_sec=10):
            raise RuntimeError("no FJT server")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        for q, t in zip(points, times):
            pt = JointTrajectoryPoint(positions=[float(v) for v in q])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        if gh is None or not gh.accepted:
            raise RuntimeError("FJT goal rejected")
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=900)
        r = rf.result()
        if r is None:
            raise RuntimeError("FJT no result (timeout)")
        return r.result.error_code

    def move_q(self, q, t=3.0):
        q = np.array(q, float)
        for i, (lo, hi) in enumerate(LIMITS):
            if not (lo - 1e-6 <= q[i] <= hi + 1e-6):
                raise RuntimeError(f"joint {i+1} target {q[i]:.3f} outside [{lo},{hi}]")
        code = self.traj([q], [t])
        qn = self.arm_q()
        err = np.abs(qn - q).max()
        return code, err

    def move_pose(self, pos, quat, t=3.0, tcp=False, seed=None, max_jump=None):
        q = self.ik_world(pos, quat, seed=seed, tcp=tcp)
        if q is None:
            raise RuntimeError(f"IK failed for {pos} {quat}")
        q0 = self.arm_q()
        jump = np.abs(q - q0).max()
        if max_jump is not None and jump > max_jump:
            raise RuntimeError(f"IK solution jumps {jump:.2f} rad (max {max_jump}); q={q}")
        code, err = self.move_q(q, t)
        return q, code, err, jump

    def gripper(self, width, timeout=300):
        if not self.grip.wait_for_server(timeout_sec=10):
            raise RuntimeError("no gripper server")
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = 30.0
        fut = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=timeout)
        r = rf.result()
        if r is None:
            return None
        return r.result.position, r.result.reached_goal, r.result.stalled


def main():
    a = sys.argv[1:]
    if not a:
        raise SystemExit(__doc__)
    t = 3.0
    if "--t" in a:
        i = a.index("--t"); t = float(a[i + 1]); del a[i:i + 2]
    tcp = "--tcp" in a
    a = [x for x in a if not x.startswith("--")]
    r = Robot()
    cmd = a[0]
    if cmd == "js":
        js = r.joint_state()
        for k, v in js.items():
            print(f"{k}: {v:.4f}")
    elif cmd == "fk":
        pos, quat = r.fk_world()
        R = quat_to_R(*quat)
        print("hand world pos", np.round(pos, 4), "quat xyzw", np.round(quat, 4))
        print("tcp world pos", np.round(pos + TCP_OFF * R[:, 2], 4))
        print("hand axes (cols x,y,z):\n", np.round(R, 3))
    elif cmd == "move":
        vals = list(map(float, a[1:8]))
        q, code, err, jump = r.move_pose(vals[:3], vals[3:7], t=t, tcp=tcp)
        print(f"q={np.round(q,4).tolist()} code={code} final_err={err:.4f} jump={jump:.2f}")
        pos, quat = r.fk_world()
        print("hand now", np.round(pos, 4), np.round(quat, 4))
    elif cmd == "ik":
        vals = list(map(float, a[1:8]))
        q = r.ik_world(vals[:3], vals[3:7], tcp=tcp)
        print("IK:", None if q is None else np.round(q, 4).tolist())
    elif cmd == "joints":
        q = list(map(float, a[1].split(",")))
        code, err = r.move_q(q, t)
        print(f"code={code} final_err={err:.4f}")
    elif cmd == "grip":
        w = {"open": 0.04, "close": 0.0}.get(a[1], None)
        w = float(a[1]) if w is None else w
        print("gripper result", r.gripper(w))
        print("fingers", r.finger())
    else:
        raise SystemExit(__doc__)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 24
timeout 120 python3 rob.py fk

# openrua op 25
timeout 200 python3 -c "
import rob, numpy as np, rclpy
r=rob.Robot()
rob.BASE_IN_WORLD=np.zeros(3)
q0=r.arm_q(); print('q0',np.round(q0,3))
q=r.ik_world([-0.203,0,1.2696],[0.9996,0,-0.0284,0]); print('IK world-coords:',None if q is None else np.round(q,3))
q=r.ik_world([0.457,0,0.358],[0.9996,0,-0.0284,0]); print('IK base-coords:',None if q is None else np.round(q,3))
from moveit_msgs.srv import GetPositionFK
req=GetPositionFK.Request(); req.header.frame_id=''; req.fk_link_names=['panda_hand','panda_link0']; req.robot_state.joint_state=r._seed()
fut=r.fk.call_async(req); rclpy.spin_until_future_complete(r.node,fut,timeout_sec=60); res=fut.result()
for ps in res.pose_stamped: print(ps.header.frame_id, ps.pose.position)
"

# openrua op 26
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rob.py", "old_string": "BASE_IN_WORLD = np.array([-0.66, 0.0, 0.912])", "new_string": "# verified: /compute_fk and /compute_ik on this machine work in the WORLD\\n# frame (FK of panda_link0 returns (-0.66, 0, 0.912)); no offset needed\\nBASE_IN_WORLD = np.array([0.0, 0.0, 0.0])", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 27
timeout 60 python3 -c "
import rclpy
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
rclpy.init(); n=rclpy.create_node('rd')
qos=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
got=[]; n.create_subscription(String,'/robot_description',got.append,qos)
import time; t=time.time()
while not got and time.time()-t<20: rclpy.spin_once(n,timeout_sec=0.5)
open('robot.urdf','w').write(got[0].data); print(len(got[0].data))
" && grep -n -A12 'link name="panda_hand"' robot.urdf | head -40; grep -n -B2 -A10 'link name="panda_leftfinger"' robot.urdf | head -30

# openrua op 28
python3 -c "
import numpy as np, struct
def stl_bbox(fn):
    d=open(fn,'rb').read()
    if d[:5]==b'solid':
        import re; v=np.array([[float(x) for x in l.split()[1:4]] for l in d.decode().splitlines() if l.strip().startswith('vertex')])
    else:
        n=struct.unpack('<I',d[80:84])[0]; a=np.frombuffer(d[84:84+n*50],dtype=np.dtype([('n','<f4',3),('v','<f4',(3,3)),('a','<u2')])); v=a['v'].reshape(-1,3)
    return v.min(0), v.max(0)
for f in ['hand','finger']:
    print(f, stl_bbox(f'/opt/ros/jazzy/share/moveit_resources_panda_description/meshes/collision/{f}.stl'))
"

# openrua op 29
find / -iname "hand*.stl" -o -iname "finger*.stl" 2>/dev/null | head; find / -ipath "*panda*" -name "*.stl" 2>/dev/null | head -5; find / -name "*.xml" -ipath "*cabinet*" 2>/dev/null | head; find / -iname "*wine*" 2>/dev/null | head

# openrua op 30
timeout 600 python3 rob.py grip open

# openrua op 31
timeout 600 python3 rob.py move -0.132 0.029 1.20 1 0 0 0 --tcp --t 4

# openrua op 32
timeout 600 python3 rob.py move -0.132 0.029 1.035 1 0 0 0 --tcp --t 3 && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih1.png && timeout 60 python3 tools/perception/cam_snap.py frontview snaps/front1.png

# openrua op 33
timeout 600 python3 rob.py grip close

# openrua op 34
timeout 600 python3 rob.py move -0.132 0.029 1.25 1 0 0 0 --tcp --t 3 && timeout 60 python3 tools/perception/cam_snap.py frontview snaps/front2.png && timeout 100 python3 rob.py js | grep finger

# openrua op 35
timeout 300 python3 -c "
import rob, numpy as np
R=np.array([[0,0,1],[0,-1,0],[1,0,0]],float)
q=rob.R_to_quat(R); print('quat',q, 'check', np.round(rob.quat_to_R(*q),3).tolist())
r=rob.Robot()
q0=r.arm_q()
for name,pos,quat in [('vert high',[-0.20,0,1.25],[1,0,0,0]),('horiz high',[-0.20,0,1.15],q),('horiz low',[-0.20,0,0.96],q),('horiz low2',[-0.22,0,0.955],q)]:
    s=r.ik_world(pos,quat,tcp=True); print(name, None if s is None else np.round(s,3).tolist())
"

# openrua op 36
timeout 300 python3 -c "
import rob, numpy as np
r=rob.Robot()
q=[0.70710678,0,0.70710678,0]
seed=np.array([0,0.6,0,-2.0,0,1.03,0.785])
p,qq=r.fk_world(seed); print('seed fk', np.round(p,3), np.round(qq,3))
for name,pos in [('horiz high',[-0.20,0,1.15]),('horiz low',[-0.20,0,0.96]),('horiz low2',[-0.22,0,0.955])]:
    s=r.ik_world(pos,q,seed=seed,tcp=True); print(name, None if s is None else np.round(s,3).tolist())
    if s is not None: p,qq=r.fk_world(s); print('   fk', np.round(p+0.1034*rob.quat_to_R(*qq)[:,2],3), np.round(qq,3))
"

# openrua op 37
timeout 600 python3 -c "
import rob, numpy as np
r=rob.Robot()
q0=r.arm_q()
def test(pos,quat,label):
    sols=[]
    rng=np.random.default_rng(0)
    seeds=[q0]+[np.array([0,a,0,b,c,d,0.785]) for a in (-0.3,0.3,0.8) for b in (-2.6,-2.0,-1.5) for c in (0.0,) for d in (0.5,1.5,2.5,3.5)]
    for s in seeds:
        sol=r.ik_world(pos,quat,seed=s,tcp=True,timeout=0.5)
        if sol is not None: sols.append(sol)
    if not sols: print(label,'NO SOLUTION'); return
    sols=np.array(sols)
    d=np.abs(sols-q0).max(1)
    i=np.argmin(d)
    print(label,'nsol',len(sols),'best (jump %.2f):'%d[i],np.round(sols[i],3).tolist())
qa=[0.70710678,0,0.70710678,0]   # hand z=+x
qb=[0.70710678,0,-0.70710678,0]  # hand z=-x
test([-0.20,0,1.15],qa,'+x high')
test([-0.20,0,0.96],qa,'+x low')
test([-0.33,0,1.15],qb,'-x high')
test([-0.33,0,0.96],qb,'-x low')
"

# openrua op 38
timeout 900 python3 -c "
import rob, numpy as np
r=rob.Robot()
q,code,err,jump=r.move_pose([-0.33,0,1.25],[1,0,0,0],t=4,tcp=True); print('step1',np.round(q,3).tolist(),code,err,jump)
qb=[0.70710678,0,-0.70710678,0]
seed=np.array([0,0.523,0,-1.916,0,0.868,0.785])
sol=r.ik_world([-0.33,0,1.15],qb,seed=seed,tcp=True); print('ik',np.round(sol,3).tolist())
code,err=r.move_q(sol,t=5); print('step2',code,err)
p,qq=r.fk_world(); print('hand',np.round(p,3),np.round(qq,3),'fingers',r.finger())
" && timeout 60 python3 tools/perception/cam_snap.py frontview snaps/front3.png

# openrua op 39
timeout 900 python3 -c "
import rob, numpy as np
r=rob.Robot()
sol=np.array([0.0, 0.523, 0.0, -1.916, 0.0, 0.868, 0.785])
print('before',np.round(r.arm_q(),3).tolist())
code,err=r.move_q(sol,t=4); print('resend',code,err)
print('after',np.round(r.arm_q(),3).tolist())
p,qq=r.fk_world(); print('hand',np.round(p,3),np.round(qq,3),'fingers',r.finger())
" && timeout 60 python3 tools/perception/cam_snap.py sideview snaps/side4.png

# openrua op 40
timeout 900 python3 -c "
import rob, numpy as np
r=rob.Robot()
qb=[0.70710678,0,-0.70710678,0]
sol=r.ik_world([-0.33,0,0.96],qb,seed=r.arm_q(),tcp=True); print('ik',np.round(sol,3).tolist())
code,err=r.move_q(sol,t=4); print('lower',code,err)
if err>0.05:
    code,err=r.move_q(sol,t=3); print('resend',code,err)
p,qq=r.fk_world(); print('hand',np.round(p,3),np.round(qq,3),'tcp',np.round(p+0.1034*rob.quat_to_R(*qq)[:,2],3))
" && timeout 60 python3 tools/perception/cam_snap.py sideview snaps/side5.png

# openrua op 41
timeout 900 python3 -c "
import rob, numpy as np
r=rob.Robot()
print('open',r.gripper(0.04), r.finger())
qb=[0.70710678,0,-0.70710678,0]
sol=r.ik_world([-0.33,0,1.15],qb,seed=r.arm_q(),tcp=True); print('ik',np.round(sol,3).tolist())
code,err=r.move_q(sol,t=3); print('raise',code,err)
" && timeout 120 python3 scan.py birdview snaps/bird2 && timeout 60 python3 tools/perception/cam_snap.py frontview snaps/front6.png

# openrua op 42
python3 -c "
import numpy as np, cv2
pc=np.load('snaps/bird2_xyz.npy'); z=pc[...,2]
reg=(pc[...,0]>-0.55)&(pc[...,0]<-0.05)&(np.abs(pc[...,1])<0.15)&(z>0.905)&(z<1.0)
P=pc[reg]; print('n',len(P))
print('x %.3f..%.3f y %.3f..%.3f z %.3f..%.3f'%(P[:,0].min(),P[:,0].max(),P[:,1].min(),P[:,1].max(),P[:,2].min(),P[:,2].max()))
# profile along x: for x bins, y-center and max z
for xb in np.arange(-0.50,-0.10,0.02):
    m=(P[:,0]>=xb)&(P[:,0]<xb+0.02)
    if m.sum(): print('x %.2f: n %3d y %.3f..%.3f (c %.3f) zmax %.3f'%(xb,m.sum(),P[m,1].min(),P[m,1].max(),P[m,1].mean(),P[m,2].max()))
"

# openrua op 43
timeout 900 python3 -c "
import rob, numpy as np
r=rob.Robot()
q,code,err,jump=r.move_pose([-0.10,0.15,1.35],[1,0,0,0],t=4,tcp=True); print(np.round(q,3).tolist(),code,err,jump)
if err>0.05: print(r.move_q(q,t=3))
" && timeout 120 python3 scan.py birdview snaps/bird2 && python3 -c "
import numpy as np, cv2
pc=np.load('snaps/bird2_xyz.npy'); z=pc[...,2]
reg=(pc[...,0]>-0.55)&(pc[...,0]<-0.05)&(np.abs(pc[...,1])<0.15)&(z>0.905)&(z<1.0)&(pc[...,1]<0.06)
P=pc[reg]; print('n',len(P))
print('x %.3f..%.3f y %.3f..%.3f z %.3f..%.3f'%(P[:,0].min(),P[:,0].max(),P[:,1].min(),P[:,1].max(),P[:,2].min(),P[:,2].max()))
for xb in np.arange(-0.50,-0.10,0.02):
    m=(P[:,0]>=xb)&(P[:,0]<xb+0.02)
    if m.sum(): print('x %.2f: n %3d y %.3f..%.3f (c %.3f) zmax %.3f'%(xb,m.sum(),P[m,1].min(),P[m,1].max(),P[m,1].mean(),P[m,2].max()))
"

# openrua op 44
python3 -c "
import numpy as np, cv2
pc=np.load('snaps/bird2_xyz.npy'); z=pc[...,2]; img=cv2.imread('snaps/bird2_color.png')
reg=(pc[...,0]>-0.65)&(pc[...,0]<-0.05)&(np.abs(pc[...,1])<0.15)&(z>0.903)&(z<1.0)&(pc[...,1]<0.06)
P=pc[reg]
for xb in np.arange(-0.60,-0.28,0.01):
    m=(P[:,0]>=xb)&(P[:,0]<xb+0.01)
    if m.sum(): print('x %.2f: n %3d y %.3f..%.3f (c %.3f) zmax %.3f'%(xb,m.sum(),P[m,1].min(),P[m,1].max(),P[m,1].mean(),P[m,2].max()))
# table extent near robot: z==0.90 along column 320
col=320
for v in range(100,200,5): print(v, np.round(pc[v,col],3), img[v,col])
"

# openrua op 45
timeout 600 python3 -c "
import rob, numpy as np
r=rob.Robot(); q0=r.arm_q()
for x in (-0.46,-0.44,-0.42,-0.40):
    for zz in (0.925,1.05):
        best=None
        for s in [q0, np.array([0,-0.5,0,-2.8,0,2.3,0.785]), np.array([0,-0.2,0,-2.9,0,2.7,0.785]), np.array([0,0.2,0,-2.6,0,2.8,0.785])]:
            sol=r.ik_world([x,-0.002,zz],[1,0,0,0],seed=s,tcp=True,timeout=0.5)
            if sol is not None and (best is None or np.abs(sol-q0).max()<np.abs(best-q0).max()): best=sol
        print(x,zz, None if best is None else np.round(best,3).tolist())
"

# openrua op 46
timeout 600 python3 -c "
import rob, numpy as np
r=rob.Robot(); q0=r.arm_q()
def Rtilt(th):
    th=np.radians(th); z=np.array([-np.sin(th),0,-np.cos(th)]); y=np.array([0,-1,0]); x=np.cross(y,z)
    return np.column_stack([x,y,z])
for th in (30,45,60):
    R=Rtilt(th); q=rob.R_to_quat(R)
    for x in (-0.46,-0.44):
        for zz in (0.925,1.0):
            best=None
            for s in [q0, np.array([0,-0.3,0,-2.6,0,2.3,0.785]), np.array([0,0.0,0,-2.4,0,2.4,0.785]), np.array([0,0.3,0,-2.2,0,2.5,0.785]),np.array([0,0.5,0,-2.0,0,2.5,0.785])]:
                sol=r.ik_world([x,-0.002,zz],q,seed=s,tcp=True,timeout=0.5)
                if sol is not None and (best is None or np.abs(sol-q0).max()<np.abs(best-q0).max()): best=sol
            print('tilt',th,x,zz, None if best is None else np.round(best,3).tolist())
"

# openrua op 47
timeout 120 python3 scan.py sideview snaps/side2 && python3 -c "
import numpy as np, cv2
pc=np.load('snaps/side2_xyz.npy'); z=pc[...,2]
reg=(pc[...,0]>-0.60)&(pc[...,0]<-0.25)&(np.abs(pc[...,1])<0.08)&(z>0.903)&(z<0.99)
P=pc[reg]; print('n',len(P), 'x %.3f..%.3f y %.3f..%.3f z %.3f..%.3f'%(P[:,0].min(),P[:,0].max(),P[:,1].min(),P[:,1].max(),P[:,2].min(),P[:,2].max()))
for xb in np.arange(-0.58,-0.28,0.01):
    m=(P[:,0]>=xb)&(P[:,0]<xb+0.01)
    if m.sum(): print('x %.2f: n %3d y %.3f..%.3f z %.3f..%.3f'%(xb,m.sum(),P[m,1].min(),P[m,1].max(),P[m,2].min(),P[m,2].max()))
"

# openrua op 48
cat >> /workspace/rob.py <<'EOF'


def tilt_quat(th_deg):
    """Hand pointing down, pitched th_deg toward -x (toward the robot); fingers close along world y."""
    th = np.radians(th_deg)
    z = np.array([-np.sin(th), 0, -np.cos(th)])
    y = np.array([0, -1, 0])
    x = np.cross(y, z)
    return R_to_quat(np.column_stack([x, y, z]))


def best_ik(r, pos, quat, tcp=True, extra_seeds=()):
    """Try several seeds; prefer solutions with small |j1|,|j3|,|j5| and near current q."""
    q0 = r.arm_q()
    seeds = [q0] + list(extra_seeds) + [np.array([0, a, 0, b, 0, c, 0.785])
                                          for a in (-0.3, 0.0, 0.3, 0.6)
                                          for b in (-2.8, -2.4, -2.0)
                                          for c in (1.8, 2.3, 2.8)]
    best, bs = None, None
    for s in seeds:
        sol = r.ik_world(pos, quat, seed=s, tcp=tcp, timeout=0.3)
        if sol is None:
            continue
        score = np.abs(sol - q0).max() + 0.5 * (abs(sol[0]) + abs(sol[2]) + abs(sol[4]))
        if best is None or score < bs:
            best, bs = sol, score
    return best


def go(r, pos, quat, t=3.0, tcp=True, label=""):
    sol = best_ik(r, pos, quat, tcp=tcp)
    if sol is None:
        raise RuntimeError(f"no IK for {label} {pos}")
    code, err = r.move_q(sol, t)
    if err > 0.03:
        code, err = r.move_q(sol, t)
    p, qq = r.fk_world()
    tcp_p = p + TCP_OFF * quat_to_R(*qq)[:, 2]
    print(f"{label}: q={np.round(sol,3).tolist()} code={code} err={err:.4f} tcp={np.round(tcp_p,3).tolist()}")
    return sol
EOF
timeout 900 python3 -c "
import rob, numpy as np
r=rob.Robot()
q=rob.tilt_quat(30)
rob.go(r,[-0.43,-0.001,1.00],q,t=4,label='pregrasp')
rob.go(r,[-0.43,-0.001,0.925],q,t=3,label='grasp')
" && timeout 60 python3 tools/perception/cam_snap.py sideview snaps/side6.png

# openrua op 49
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih2.png

# openrua op 50
timeout 600 python3 -c "
import rob, numpy as np
r=rob.Robot()
print('close',r.gripper(0.0), r.finger())
q=rob.tilt_quat(30)
rob.go(r,[-0.43,-0.001,1.08],q,t=3,label='lift')
print('fingers',r.finger())
" && timeout 60 python3 tools/perception/cam_snap.py sideview snaps/side7.png

# openrua op 51
timeout 900 python3 -c "
import rob, numpy as np
r=rob.Robot()
q=rob.tilt_quat(30)
rob.go(r,[-0.034,0.16,1.10],q,t=5,label='over drawer')
print('fingers',r.finger())
" && timeout 60 python3 tools/perception/cam_snap.py agentview snaps/agent8.png

# openrua op 52
timeout 900 python3 -c "
import rob, numpy as np
r=rob.Robot()
tgt=np.array([0.107, 0.821, 0.168, -1.164, 0.003, 1.469, 1.038])
print('now',np.round(r.arm_q(),3).tolist())
code,err=r.move_q(tgt,t=6); print('resend',code,err)
if err>0.03:
    code,err=r.move_q(tgt,t=6); print('resend2',code,err)
p,qq=r.fk_world(); print('tcp',np.round(p+0.1034*rob.quat_to_R(*qq)[:,2],3),'fingers',r.finger())
"

# openrua op 53
timeout 60 python3 tools/perception/cam_snap.py frontview snaps/front9.png; timeout 60 python3 tools/perception/cam_snap.py sideview snaps/side9.png; timeout 100 python3 rob.py js

# openrua op 54
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih3.png; timeout 60 python3 tools/perception/cam_snap.py robot0_robotview snaps/robotview3.png

# openrua op 55
timeout 600 python3 -c "
import rob, numpy as np
r=rob.Robot()
q=r.arm_q(); print('q',np.round(q,3).tolist())
p,qq=r.fk_world(); R=rob.quat_to_R(*qq); print('tcp',np.round(p+0.1034*R[:,2],3)); print('hand x (bottle axis)',np.round(R[:,0],3),'z (approach)',np.round(R[:,2],3))
# bottle cap position estimate: cap is along -hand_x? bottle axis along hand x; cap toward +hand x or -hand x?
for s in (0.12,-0.12):
    print('end at s=%.2f'%s, np.round(p+0.1034*R[:,2]+s*R[:,0],3))
for dj2 in (-0.1,-0.2,-0.3):
    q2=q.copy(); q2[1]+=dj2
    p2,qq2=r.fk_world(q2); R2=rob.quat_to_R(*qq2); print('dj2',dj2,'tcp',np.round(p2+0.1034*R2[:,2],3),'x',np.round(R2[:,0],2))
"

# openrua op 56
timeout 600 python3 -c "
import rob, numpy as np
r=rob.Robot()
q=r.arm_q(); q2=q.copy(); q2[1]-=0.3
code,err=r.move_q(q2,t=2); print('lift j2',code,err, np.round(r.arm_q(),3).tolist())
p,qq=r.fk_world(); R=rob.quat_to_R(*qq); print('tcp',np.round(p+0.1034*R[:,2],3),'fingers',r.finger())
" && timeout 60 python3 tools/perception/cam_snap.py robot0_robotview snaps/robotview4.png

# openrua op 57
python3 -c "
import rob, numpy as np
r=rob.Robot()
q=r.arm_q(); print('q',np.round(q,3)); print('fingers',r.finger())
p,qt=r.fk_world(); print('hand',np.round(p,3),np.round(qt,3))
" 2>&1 | tail -5; timeout 5 ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once 2>&1 | tail -12

# openrua op 58
python3 -c "
import rob, numpy as np
r=rob.Robot()
q=r.arm_q()
p,qt=r.fk_world()
tcp,_=r.fk_world(link='panda_hand_tcp') if False else (None,None)
# small retreat: -x, -y, +z of the hand
target=p+np.array([-0.04,-0.04,0.03])
sol=r.ik_world(target,qt,seed=q,tcp=False)
print('sol',None if sol is None else np.round(sol,3))
if sol is not None and np.abs(sol-q).max()<0.5:
    code,err=r.move_q(sol,2.0); print('code',code,'err',err)
    print('q now',np.round(r.arm_q(),3)); print('hand',np.round(r.fk_world()[0],3))
" 2>&1 | tail -6

# openrua op 59
timeout 5 ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once 2>&1 | grep -A3 force; python3 -c "
import rob;r=rob.Robot();print(r.finger())" 2>&1|tail -1

# openrua op 60
python3 -c "
import rob, numpy as np
r=rob.Robot()
tq=rob.tilt_quat(30)
tcp,_=r.fk_world(link='panda_hand_tcp')
print('tcp now',np.round(tcp,3))
rob.go(r,[tcp[0],tcp[1],1.25],tq,t=3,label='raise')
print('fingers',r.finger())
" 2>&1 | tail -8

# openrua op 61
grep -n "def fk_world\|def go\|def best_ik\|tcp" /workspace/rob.py | head -30

# openrua op 62
python3 -c "
import rob, numpy as np
r=rob.Robot()
tq=rob.tilt_quat(30)
p,qq=r.fk_world(); tcp=p+rob.TCP_OFF*rob.quat_to_R(*qq)[:,2]
print('tcp now',np.round(tcp,3))
rob.go(r,[tcp[0],tcp[1],1.25],tq,t=3,label='raise')
print('fingers',r.finger())
" 2>&1 | tail -8

# openrua op 63
python3 tools/perception/cam_snap.py robot0_robotview snaps/rv5.png >/dev/null 2>&1; python3 tools/perception/cam_snap.py frontview snaps/fv5.png 2>&1 | tail -1

# openrua op 64
python3 scan.py sideview snaps/side5 2>&1 | tail -1 && python3 -c "
import numpy as np
pc=np.load('snaps/side5_xyz.npy')
# points above z 1.0 near tcp (-0.255,0.094,1.25), exclude arm: pick x>-0.45
m=np.isfinite(pc[...,0])&(pc[...,2]>1.0)&(pc[...,2]<1.35)&(pc[...,0]>-0.4)&(pc[...,0]<0.0)&(np.abs(pc[...,1]-0.094)<0.12)
P=pc[m]; print(len(P))
for zlo in np.arange(1.0,1.35,0.025):
    s=P[(P[:,2]>=zlo)&(P[:,2]<zlo+0.025)]
    if len(s): print(round(zlo,3), len(s), 'x',np.round([s[:,0].min(),s[:,0].max()],3),'y',np.round([s[:,1].min(),s[:,1].max()],3))
"

# openrua op 65
python3 -c "
import numpy as np
pc=np.load('snaps/side5_xyz.npy')
m=np.isfinite(pc[...,0])&(pc[...,2]>1.17)&(pc[...,2]<1.30)&(pc[...,0]>-0.4)&(pc[...,0]<0.0)&(np.abs(pc[...,1]-0.094)<0.08)
P=pc[m]
for xlo in np.arange(-0.34,-0.10,0.01):
    s=P[(P[:,0]>=xlo)&(P[:,0]<xlo+0.01)]
    if len(s): print(round(xlo,2), len(s), 'z',np.round([s[:,2].min(),s[:,2].max()],3),'y',np.round([s[:,1].min(),s[:,1].max()],3))
"

# openrua op 66
python3 -c "
import rob, numpy as np
r=rob.Robot()
tq=rob.tilt_quat(30)
rob.go(r,[-0.036,0.16,1.25],tq,t=4,label='over')
print('fingers',r.finger())
" 2>&1 | tail -4; timeout 5 ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once 2>&1 | grep -A3 force

# openrua op 67
python3 -c "
import rob, numpy as np, subprocess
r=rob.Robot()
tq=rob.tilt_quat(30)
for z in [1.12,1.03,0.985,0.96]:
    rob.go(r,[-0.036,0.16,z],tq,t=2.5,label=f'z{z}')
    print('fingers',np.round(r.finger(),4))
" 2>&1 | tail -10; timeout 5 ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once 2>&1 | grep -A3 force

# openrua op 68
python3 tools/perception/cam_snap.py robot0_robotview snaps/rv6.png >/dev/null 2>&1; python3 tools/perception/cam_snap.py frontview snaps/fv6.png >/dev/null 2>&1; python3 tools/perception/cam_snap.py sideview snaps/sv6.png 2>&1|tail -1

# openrua op 69
python3 -c "
import rob, numpy as np
r=rob.Robot()
tq=rob.tilt_quat(30)
rob.go(r,[-0.05,0.14,1.30],tq,t=3,label='up')
" 2>&1 | tail -2; python3 scan.py birdview snaps/bird6 2>&1 | tail -1; python3 -c "
import numpy as np
pc=np.load('snaps/bird6_xyz.npy')
# drawer footprint region
m=np.isfinite(pc[...,0])&(pc[...,0]>-0.13)&(pc[...,0]<0.14)&(pc[...,1]>0.03)&(pc[...,1]<0.25)&(pc[...,2]>0.93)&(pc[...,2]<0.99)
P=pc[m]; print('pts',len(P))
for xlo in np.arange(-0.13,0.14,0.02):
    s=P[(P[:,0]>=xlo)&(P[:,0]<xlo+0.02)]
    if len(s): print(round(xlo,2),len(s),'z',np.round([s[:,2].min(),s[:,2].max()],3),'y',np.round([s[:,1].min(),s[:,1].max()],3))
"

# openrua op 70
python3 -c "
import rob, numpy as np
r=rob.Robot()
# park arm back toward robot, high, away from the cabinet
rob.go(r,[-0.35,-0.05,1.30],rob.tilt_quat(30),t=4,label='park')
" 2>&1 | tail -1; python3 scan.py birdview snaps/bird7 2>&1 | tail -1; python3 -c "
import numpy as np
pc=np.load('snaps/bird7_xyz.npy')
m=np.isfinite(pc[...,0])&(pc[...,0]>-0.13)&(pc[...,0]<0.14)&(pc[...,1]>0.03)&(pc[...,1]<0.25)&(pc[...,2]>0.93)&(pc[...,2]<0.99)
P=pc[m]; print('pts',len(P))
for xlo in np.arange(-0.13,0.14,0.02):
    s=P[(P[:,0]>=xlo)&(P[:,0]<xlo+0.02)]
    if len(s): print(round(xlo,2),len(s),'z',np.round([s[:,2].min(),s[:,2].max()],3),'y',np.round([s[:,1].min(),s[:,1].max()],3))
# bottle: z between 0.93 and 0.975, y in (0.09,0.22)
b=P[(P[:,2]<0.975)&(P[:,1]>0.09)&(P[:,1]<0.22)]
print('bottle-ish pts',len(b),'x',np.round([b[:,0].min(),b[:,0].max()],3),'y',np.round([b[:,1].min(),b[:,1].max()],3),'zmax',round(b[:,2].max(),3))
"

# openrua op 71
python3 -c "
import rob, numpy as np
R=np.column_stack([[0,1,0],[-1,0,0],[0,0,-1]]).astype(float)
q=rob.R_to_quat(R); print('quat',np.round(q,4))
r=rob.Robot()
print('grip',r.gripper(0.0))
rob.go(r,[0.0,0.0,1.15],q,t=4,label='pre-push high')
rob.go(r,[0.0,0.0,0.99],q,t=3,label='pre-push')
" 2>&1 | tail -4

# openrua op 72
sed -n '/def R_to_quat/,/^def \|^class /p' /workspace/rob.py | head -30; timeout 5 ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once 2>&1 | grep -A3 force; python3 -c "
import rob,numpy as np
r=rob.Robot(); p,q=r.fk_world(); print(np.round(p,3),np.round(q,3)); print(np.round(rob.quat_to_R(*q),2))" 2>&1 | tail -4

# openrua op 73
python3 -c "
import numpy as np
pc=np.load('snaps/bird7_xyz.npy')
m=np.isfinite(pc[...,0])&(pc[...,0]>-0.13)&(pc[...,0]<0.14)&(pc[...,1]>0.02)&(pc[...,1]<0.09)
P=pc[m]
for ylo in np.arange(0.02,0.09,0.01):
    s=P[(P[:,1]>=ylo)&(P[:,1]<ylo+0.01)]
    if len(s): print(round(ylo,2),len(s),'z',np.round([s[:,2].min(),s[:,2].max()],3),'x',np.round([s[:,0].min(),s[:,0].max()],3))
"

# openrua op 74
python3 -c "
import rob, numpy as np
r=rob.Robot()
q=r.arm_q(); p,qq=r.fk_world()
sol=r.ik_world(p+np.array([0,0,0.10]),qq,seed=q,tcp=False)
print(np.round(sol,3)); code,err=r.move_q(sol,2.5); print(code,err)
R=np.column_stack([[0,1,0],[1,0,0],[0,0,-1]]).astype(float)
print('det',np.linalg.det(R)); qn=rob.R_to_quat(R); print('quat',np.round(qn,4))
rob.go(r,[0.0,0.0,1.10],qn,t=3,label='rot')
p,qq=r.fk_world(); print(np.round(rob.quat_to_R(*qq),2))
" 2>&1 | tail -8; timeout 5 ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once 2>&1 | grep -A3 force

# openrua op 75
python3 -c "
import rob, numpy as np
r=rob.Robot()
qn=np.array([0.7071,0.7071,0,0])
q=r.arm_q(); print('q',np.round(q,3))
tgt=[0.0, 0.486, -0.0, -1.759, 0.0, 2.245, -0.785]
code,err=r.move_q(tgt,3); print(code,err, np.round(r.arm_q(),3))
p,qq=r.fk_world(); print(np.round(rob.quat_to_R(*qq),2)); print('tcp',np.round(p+rob.TCP_OFF*rob.quat_to_R(*qq)[:,2],3))
" 2>&1 | tail -6

# openrua op 76
python3 -c "
import rob, numpy as np
r=rob.Robot()
qn=np.array([0.7071,0.7071,0,0])
rob.go(r,[0.0,0.02,1.02],qn,t=2.5,label='down1')
rob.go(r,[0.0,0.02,0.975],qn,t=2.5,label='down2')
" 2>&1 | tail -2; timeout 5 ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once 2>&1 | grep -A3 force

# openrua op 77
python3 -c "
import rob, numpy as np, subprocess
r=rob.Robot()
qn=np.array([0.7071,0.7071,0,0])
for y in [0.06,0.10,0.14,0.18,0.214]:
    rob.go(r,[0.0,y,0.975],qn,t=2.5,label=f'push y={y}')
    w=subprocess.run('timeout 5 ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once | grep -A3 force | tr -d \"\\n\"',shell=True,capture_output=True,text=True).stdout
    print('  ',w.strip())
" 2>&1 | tail -12

# openrua op 78
python3 tools/perception/cam_snap.py robot0_robotview snaps/rv7.png >/dev/null 2>&1; python3 tools/perception/cam_snap.py sideview snaps/sv7.png 2>&1|tail -1

# openrua op 79
python3 -c "
import rob, numpy as np
r=rob.Robot()
q=r.arm_q(); p,qq=r.fk_world()
sol=r.ik_world(p+np.array([0,0,0.15]),qq,seed=q,tcp=False)
code,err=r.move_q(sol,3); print('lift',code,err)
rob.go(r,[-0.35,-0.05,1.30],rob.tilt_quat(30),t=4,label='park')
" 2>&1 | tail -2; python3 scan.py birdview snaps/bird8 2>&1 | tail -1; python3 -c "
import numpy as np
pc=np.load('snaps/bird8_xyz.npy')
m=np.isfinite(pc[...,0])&(pc[...,0]>-0.13)&(pc[...,0]<0.14)&(pc[...,1]>0.0)&(pc[...,1]<0.45)&(pc[...,2]>0.905)
P=pc[m]
print('min y of anything above table in cabinet x-range:',round(P[:,1].min(),3))
for ylo in np.arange(0.0,0.45,0.02):
    s=P[(P[:,1]>=ylo)&(P[:,1]<ylo+0.02)]
    if len(s): print(round(ylo,2),len(s),'z',np.round([s[:,2].min(),s[:,2].max()],3),'x',np.round([s[:,0].min(),s[:,0].max()],3))
"

# openrua op 80
python3 -c "
import rob, numpy as np
r=rob.Robot()
for name,cols in [('A',[[0,1,0],[1,0,0],[0,0,-1]]),('B',[[0,-1,0],[-1,0,0],[0,0,-1]])]:
    R=np.column_stack(cols).astype(float); qn=rob.R_to_quat(R)
    sol=rob.best_ik(r,[0.0,0.20,0.975],qn,tcp=True)
    print(name,np.round(qn,3),'q',None if sol is None else np.round(sol,3))
    if sol is not None:
        for l in ['panda_link5','panda_link6','panda_link7']:
            p,_=r.fk_world(sol,link=l); print('   ',l,np.round(p,3))
" 2>&1 | tail -8

# openrua op 81
python3 -c "
import rob, numpy as np, subprocess
th=np.radians(30); c,s=np.cos(th),np.sin(th)
R=np.column_stack([[0,c,s],[1,0,0],[0,s,-c]]).astype(float); print('det',round(np.linalg.det(R),3))
qn=rob.R_to_quat(R); print('quat',np.round(qn,4))
r=rob.Robot()
sol=rob.best_ik(r,[0.0,0.20,0.975],qn,tcp=True); print('q',np.round(sol,3))
for l in ['panda_link6','panda_link7','panda_hand']:
    p,_=r.fk_world(sol,link=l); print('   ',l,np.round(p,3))
" 2>&1 | tail -6

# openrua op 82
python3 -c "
import rob, numpy as np, subprocess
qn=np.array([0.683,0.683,0.183,-0.183]); qn/=np.linalg.norm(qn)
r=rob.Robot()
rob.go(r,[0.0,0.12,1.12],qn,t=4,label='pre high')
rob.go(r,[0.0,0.12,0.975],qn,t=3,label='pre low')
for y in [0.16,0.19,0.214,0.228]:
    rob.go(r,[0.0,y,0.975],qn,t=2.5,label=f'push y={y}')
    w=subprocess.run('timeout 5 ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once | grep -A3 force | tr -d \"\\n\"',shell=True,capture_output=True,text=True).stdout
    print('  ',w.strip())
" 2>&1 | tail -12

# openrua op 83
python3 -c "
import rob, numpy as np
r=rob.Robot()
q=r.arm_q(); p,qq=r.fk_world()
sol=r.ik_world(p+np.array([0,-0.03,0.15]),qq,seed=q,tcp=False)
code,err=r.move_q(sol,3); print('lift',code,err)
rob.go(r,[-0.35,-0.05,1.30],rob.tilt_quat(30),t=4,label='park')
" 2>&1 | tail -2; python3 scan.py birdview snaps/bird9 2>&1 | tail -1; python3 scan.py agentview snaps/agent9 2>&1 | tail -1; python3 -c "
import numpy as np
pc=np.load('snaps/bird9_xyz.npy')
m=np.isfinite(pc[...,0])&(pc[...,0]>-0.13)&(pc[...,0]<0.14)&(pc[...,1]>0.0)&(pc[...,1]<0.45)&(pc[...,2]>0.905)
P=pc[m]
print('min y above table in cabinet x-range:',round(P[:,1].min(),3))
for ylo in np.arange(0.14,0.30,0.01):
    s=P[(P[:,1]>=ylo)&(P[:,1]<ylo+0.01)]
    if len(s): print(round(ylo,2),len(s),'z',np.round([s[:,2].min(),s[:,2].max()],3),'x',np.round([s[:,0].min(),s[:,0].max()],3))
"

# openrua op 84
python3 -c "
import numpy as np
pc=np.load('snaps/bird9_xyz.npy')
m=np.isfinite(pc[...,0])&(pc[...,0]>0.06)&(pc[...,0]<0.10)&(pc[...,1]>0.1)&(pc[...,1]<0.3)
P=pc[m]
for zlo,zhi in [(0.92,0.985),(0.99,1.05),(1.06,1.10)]:
    s=P[(P[:,2]>=zlo)&(P[:,2]<zhi)]
    print((zlo,zhi),len(s),'min y',round(s[:,1].min(),4) if len(s) else None)
"

# openrua op 85
python3 scan.py frontview snaps/front9 2>&1 | tail -1; python3 -c "
import numpy as np
pc=np.load('snaps/front9_xyz.npy')
m=np.isfinite(pc[...,0])&(pc[...,0]>-0.10)&(pc[...,0]<0.10)&(pc[...,1]>0.1)&(pc[...,1]<0.3)
P=pc[m]
for zlo in np.arange(0.92,1.13,0.02):
    s=P[(P[:,2]>=zlo)&(P[:,2]<zlo+0.02)]
    if len(s): print(round(zlo,2),len(s),'y min/median',np.round([s[:,1].min(),np.median(s[:,1])],4))
"

# openrua op 86
python3 -c "
import rob, numpy as np
r=rob.Robot()
q=[-0.018,0.817,0.319,-1.665,0.487,2.504,-0.981]
for l in ['panda_link5','panda_link6','panda_link7','panda_hand','panda_leftfinger','panda_rightfinger']:
    p,qq=r.fk_world(q,link=l); print(l,np.round(p,3), np.round(rob.quat_to_R(*qq)[:,2],2))
" 2>&1 | tail -6

# openrua op 87
python3 -c "
import rob, numpy as np
th=np.radians(60); c,s=np.cos(th),np.sin(th)
R=np.column_stack([[0,c,s],[1,0,0],[0,s,-c]]).astype(float)
qn=rob.R_to_quat(R); print('quat',np.round(qn,4))
r=rob.Robot()
for tgt in [[0.08,0.17,0.965],[0.08,0.21,0.965]]:
    sol=rob.best_ik(r,tgt,qn,tcp=True); print(tgt,'q',None if sol is None else np.round(sol,3))
    if sol is not None:
        for l in ['panda_link6','panda_link7','panda_hand']:
            p,qq=r.fk_world(sol,link=l); print('   ',l,np.round(p,3))
        p,qq=r.fk_world(sol,link='panda_hand'); Rh=rob.quat_to_R(*qq)
        print('    hand body far face center',np.round(p+0.058*Rh[:,2],3),'+x-edge',np.round(p+0.058*Rh[:,2]+0.03*Rh[:,0],3))
" 2>&1 | tail -12

# openrua op 88
python3 -c "
import rob, numpy as np, subprocess
qn=np.array([0.6124,0.6124,0.3536,-0.3536]); qn/=np.linalg.norm(qn)
r=rob.Robot()
rob.go(r,[0.08,0.12,1.10],qn,t=4,label='pre high')
rob.go(r,[0.08,0.17,0.965],qn,t=3,label='pre low')
for y in [0.20,0.215,0.225,0.232]:
    rob.go(r,[0.08,y,0.965],qn,t=2.5,label=f'push y={y}')
    w=subprocess.run('timeout 5 ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once | grep -A3 force | tr -d \"\\n\"',shell=True,capture_output=True,text=True).stdout
    print('  ',w.strip())
" 2>&1 | tail -12

# openrua op 89
python3 -c "
import rob, numpy as np
r=rob.Robot()
q=r.arm_q(); p,qq=r.fk_world()
sol=r.ik_world(p+np.array([0,-0.04,0.12]),qq,seed=q,tcp=False)
code,err=r.move_q(sol,3); print('lift',code,err)
code,err=r.move_q([0,-0.4,0,-2.3,0,1.9,0.785],4); print('home',code,err)
" 2>&1 | tail -2; python3 scan.py frontview snaps/front10 2>&1 | tail -1; python3 -c "
import numpy as np
pc=np.load('snaps/front10_xyz.npy')
m=np.isfinite(pc[...,0])&(pc[...,0]>-0.10)&(pc[...,0]<0.10)&(pc[...,1]>0.1)&(pc[...,1]<0.3)
P=pc[m]
for zlo in np.arange(0.92,1.13,0.02):
    s=P[(P[:,2]>=zlo)&(P[:,2]<zlo+0.02)]
    if len(s): print(round(zlo,2),len(s),'y min/median',np.round([s[:,1].min(),np.median(s[:,1])],4))
"

# openrua op 90
python3 -c "
import rob, numpy as np
r=rob.Robot()
for i in range(3):
    code,err=r.move_q([0,-0.4,0,-2.3,0,1.9,0.785],4); print('home',code,round(err,4))
    if err<0.03: break
print(np.round(r.arm_q(),3))
" 2>&1 | tail -4; python3 tools/perception/cam_snap.py frontview snaps/final_front.png >/dev/null 2>&1; python3 tools/perception/cam_snap.py agentview snaps/final_agent.png 2>&1 | tail -1

# openrua op 91
python3 scan.py birdview snaps/final_bird 2>&1 | tail -1; python3 -c "
import numpy as np
pc=np.load('snaps/final_bird_xyz.npy')
m=np.isfinite(pc[...,0])&(pc[...,2]>0.905)&(pc[...,2]<1.2)&(pc[...,0]>-0.45)&(pc[...,0]<0.5)&(np.abs(pc[...,1])<0.6)
P=pc[m]
# exclude cabinet, bowl, shelf regions
cab=(P[:,0]>-0.13)&(P[:,0]<0.14)&(P[:,1]>0.19)&(P[:,1]<0.42)
bowl=(np.hypot(P[:,0]-0.01,P[:,1]+0.06)<0.09)
shelf=(P[:,1]<-0.15)&(P[:,0]>-0.2)&(P[:,0]<0.08)
rest=P[~(cab|bowl|shelf)]
print('unexplained pts above table:',len(rest))
if len(rest): print(np.round(rest[:,0].min(),3),np.round(rest[:,0].max(),3),np.round(rest[:,1].min(),3),np.round(rest[:,1].max(),3),np.round(rest[:,2].max(),3))
"
