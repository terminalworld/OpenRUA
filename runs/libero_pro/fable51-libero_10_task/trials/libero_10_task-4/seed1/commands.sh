#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
cat /workspace/tools/action/fjt_send.py; echo ----; cat /workspace/tools/action/gripper_cmd.py; echo ----; cat /workspace/tools/action/ik_move.py

# openrua op 3
ros2 topic list && echo ---- && ros2 action list && echo ---- && ros2 service list | grep -iE "ik|fk|plan|scene" && echo ---- && ros2 node list

# openrua op 4
for c in agentview birdview frontview sideview robot0_robotview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c; done; ros2 topic echo /joint_states --once

# openrua op 5
ros2 topic echo /agentview/color/camera_info --once | head -20; timeout 20 ros2 run tf2_ros tf2_echo world agentview_optical_frame 2>&1 | head -12; timeout 20 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -8

# openrua op 6
mkdir -p "$(dirname /workspace/scene.py)"
cat > /workspace/scene.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Grab color+depth+info+TF for a camera; convert pixels to world.

Usage: python3 scene.py <camera> u,v [u,v ...]
Also saves <camera>_depth.npy and <camera>.png.
"""
import sys
import numpy as np
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
import cv2


def grab(node, topic, typ):
    got = {}
    sub = node.create_subscription(typ, topic, lambda m: got.setdefault("m", m), 1)
    while "m" not in got:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    return got["m"]


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main():
    cam = sys.argv[1]
    pix = [tuple(int(t) for t in a.split(",")) for a in sys.argv[2:]]
    rclpy.init()
    node = rclpy.create_node("scene")
    buf = Buffer(); TransformListener(buf, node)
    br = CvBridge()
    color = br.imgmsg_to_cv2(grab(node, f"/{cam}/color/image_raw", Image), "bgr8")
    depth = br.imgmsg_to_cv2(grab(node, f"/{cam}/depth/image_raw", Image), "passthrough")
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    cv2.imwrite(f"{cam}.png", color)
    np.save(f"{cam}_depth.npy", depth)
    frame = f"{cam}_optical_frame"
    while not buf.can_transform("world", frame, rclpy.time.Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    t = buf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    T = np.eye(4); T[:3, :3] = quat_R(q.x, q.y, q.z, q.w)
    T[:3, 3] = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
    np.save(f"{cam}_T.npy", T)
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    print("depth shape", depth.shape, "finite range",
          np.nanmin(depth[np.isfinite(depth)]), np.nanmax(depth[np.isfinite(depth)]))
    for (u, v) in pix:
        z = float(depth[v, u])
        p = T @ np.array([(u - cx) * z / fx, (v - cy) * z / fy, z, 1.0])
        print(f"px({u},{v}) depth={z:.4f} -> world {p[0]:.4f} {p[1]:.4f} {p[2]:.4f}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 7
timeout 120 python3 scene.py agentview 75,295 555,300 215,215 410,235 330,200 320,400 && timeout 120 python3 scene.py birdview 243,285 390,287 283,257 320,273 320,150

# openrua op 8
python3 -c "
import numpy as np, cv2
d=np.load('birdview_depth.npy'); T=np.load('birdview_T.npy'); img=cv2.imread('birdview.png')
import yaml
fx=fy=None
# intrinsics: read from camera_info saved? recompute via ros
" ; ros2 topic echo /birdview/color/camera_info --once | grep -A9 '^k:'

# openrua op 9
python3 -c "
import numpy as np, cv2
d=np.load('birdview_depth.npy'); T=np.load('birdview_T.npy'); img=cv2.imread('birdview.png')
fx=fy=579.4112549695428; cx=320; cy=240
H,W=d.shape
v,u=np.mgrid[0:H,0:W]
pc=np.stack([(u-cx)*d/fx,(v-cy)*d/fy,d,np.ones_like(d)],-1)
pw=pc@T.T
z=pw[...,2]
print('table z stats', np.percentile(z[(z>0.40)&(z<0.44)],[5,50,95]))
mask=(z>0.45)&(z<0.75)
mask=mask.astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,4]<20: continue
    m=lab==i
    zs=z[m]; xs=pw[...,0][m]; ys=pw[...,1][m]
    col=img[m].mean(0)
    print(f'blob {i}: px area={stats[i,4]} centroid=({cent[i][0]:.0f},{cent[i][1]:.0f}) world x={xs.mean():.3f} y={ys.mean():.3f} zmax={zs.max():.3f} zmed={np.median(zs):.3f} xr=[{xs.min():.3f},{xs.max():.3f}] yr=[{ys.min():.3f},{ys.max():.3f}] bgr={col.round()}')
"


# openrua op 10
python3 -c "
import numpy as np, cv2
d=np.load('agentview_depth.npy'); T=np.load('agentview_T.npy'); img=cv2.imread('agentview.png')
fx=fy=579.4112549695428; cx=320; cy=240
H,W=d.shape
v,u=np.mgrid[0:H,0:W]
pc=np.stack([(u-cx)*d/fx,(v-cy)*d/fy,d,np.ones_like(d)],-1)
pw=pc@T.T
x,y,z=pw[...,0],pw[...,1],pw[...,2]
# mugs region: table objects in x in [-0.3,0.1], z between 0.43 and 0.65, not robot
mask=(z>0.44)&(z<0.65)&(x>-0.3)&(x<0.15)&(np.abs(y)<0.45)
hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
# yellow: hue 20-35, sat>100 ; red: hue<10 or >170, sat>100 ; white/gray: sat<40
yel=mask&(hsv[...,0]>=18)&(hsv[...,0]<=38)&(hsv[...,1]>90)
red=mask&((hsv[...,0]<=8)|(hsv[...,0]>=172))&(hsv[...,1]>90)
wht=mask&(hsv[...,1]<45)&(hsv[...,2]>120)
for name,m in [('yellow',yel),('red',red),('white',wht)]:
    n,lab,stats,cent=cv2.connectedComponentsWithStats(m.astype(np.uint8))
    for i in range(1,n):
        if stats[i,4]<60: continue
        mm=lab==i
        print(f'{name} blob area={stats[i,4]} px=({cent[i][0]:.0f},{cent[i][1]:.0f}) x=[{x[mm].min():.3f},{x[mm].max():.3f}] y=[{y[mm].min():.3f},{y[mm].max():.3f}] z=[{z[mm].min():.3f},{z[mm].max():.3f}]')
"


# openrua op 11
python3 -c "
import numpy as np, cv2
d=np.load('agentview_depth.npy'); T=np.load('agentview_T.npy'); img=cv2.imread('agentview.png')
fx=fy=579.4112549695428; cx=320; cy=240
H,W=d.shape
v,u=np.mgrid[0:H,0:W]
pc=np.stack([(u-cx)*d/fx,(v-cy)*d/fy,d,np.ones_like(d)],-1)
pw=pc@T.T
x,y,z=pw[...,0],pw[...,1],pw[...,2]
for name,(y0,y1) in [('white',(-0.25,-0.07)),('yellow',(0.03,0.2)),('red',(-0.07,0.08))]:
  for zlo in [0.45,0.50,0.53,0.56,0.58]:
    m=(z>zlo)&(z<0.65)&(x>-0.3)&(x<0.05)&(y>y0)&(y<y1)
    if m.sum()<5: continue
    print(f'{name} z>{zlo}: n={m.sum()} x=[{x[m].min():.3f},{x[m].max():.3f}] y=[{y[m].min():.3f},{y[m].max():.3f}] zmax={z[m].max():.3f}')
"


# openrua op 12
python3 -c "
import cv2
img=cv2.imread('agentview.png')
crop=img[140:300,150:480]
cv2.imwrite('crop.png', cv2.resize(crop,None,fx=2,fy=2,interpolation=cv2.INTER_NEAREST))
"

# openrua op 13
python3 -c "
import numpy as np
d=np.load('agentview_depth.npy'); T=np.load('agentview_T.npy')
fx=fy=579.4112549695428; cx=320; cy=240
def w(u,v):
    z=d[v,u]; p=T@np.array([(u-cx)*z/fx,(v-cy)*z/fy,z,1]); return p
for v in range(230,285,3):
    p=w(210,v); print(v, p.round(3))
"


# openrua op 14
timeout 300 python3 tools/action/gripper_cmd.py 0.04; ros2 topic echo /joint_states --once | grep -A9 position

# openrua op 15
timeout 120 python3 scene.py agentview 320,400 && timeout 120 python3 scene.py birdview 320,400 && python3 -c "
import numpy as np, cv2
for cam in ['birdview','agentview']:
    d=np.load(f'{cam}_depth.npy'); T=np.load(f'{cam}_T.npy'); img=cv2.imread(f'{cam}.png')
    fx=fy=579.4112549695428; cx=320; cy=240
    H,W=d.shape
    v,u=np.mgrid[0:H,0:W]
    pc=np.stack([(u-cx)*d/fx,(v-cy)*d/fy,d,np.ones_like(d)],-1)
    pw=pc@T.T
    x,y,z=pw[...,0],pw[...,1],pw[...,2]
    mask=((z>0.432)&(z<0.6)&(x>-0.3)&(x<0.15)&(np.abs(y)<0.45)).astype(np.uint8)
    n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
    print(cam)
    for i in range(1,n):
        if stats[i,4]<30: continue
        m=lab==i
        print(f'  blob area={stats[i,4]} px=({cent[i][0]:.0f},{cent[i][1]:.0f}) x=[{x[m].min():.3f},{x[m].max():.3f}] y=[{y[m].min():.3f},{y[m].max():.3f}] z=[{z[m].min():.3f},{z[m].max():.3f}] bgr={img[m].mean(0).round()}')
"


# openrua op 16
python3 -c "
import cv2
img=cv2.imread('birdview.png')
crop=img[220:310,220:420]
cv2.imwrite('crop2.png', cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST))
"

# openrua op 17
python3 -c "
import numpy as np, cv2
def fit_circle(x,y):
    A=np.c_[2*x,2*y,np.ones_like(x)]; b=x**2+y**2
    c=np.linalg.lstsq(A,b,rcond=None)[0]; r=np.sqrt(c[2]+c[0]**2+c[1]**2); return c[0],c[1],r
for cam in ['agentview','birdview']:
    d=np.load(f'{cam}_depth.npy'); T=np.load(f'{cam}_T.npy'); img=cv2.imread(f'{cam}.png')
    fx=fy=579.4112549695428; cx=320; cy=240
    H,W=d.shape
    v,u=np.mgrid[0:H,0:W]
    pc=np.stack([(u-cx)*d/fx,(v-cy)*d/fy,d,np.ones_like(d)],-1)
    pw=pc@T.T
    x,y,z=pw[...,0],pw[...,1],pw[...,2]
    print(cam)
    for name,(x0,x1,y0,y1,zlo) in {'white':(-0.2,-0.05,-0.25,-0.10,0.53),'yellow':(-0.13,0.0,0.03,0.2,0.52),'red':(-0.25,-0.13,-0.07,0.09,0.555)}.items():
        m=(z>zlo)&(x>x0)&(x<x1)&(y>y0)&(y<y1)
        if m.sum()<10: print(' ',name,'few pts',m.sum()); continue
        c=fit_circle(x[m],y[m])
        print(f'  {name}: n={m.sum()} rim z=[{z[m].min():.3f},{z[m].max():.3f}] circle center=({c[0]:.3f},{c[1]:.3f}) r={c[2]:.3f}  extents x=[{x[m].min():.3f},{x[m].max():.3f}] y=[{y[m].min():.3f},{y[m].max():.3f}]')
    for name,(x0,x1,y0,y1) in {'Lplate':(-0.1,0.1,-0.42,-0.22),'Rplate':(-0.1,0.1,0.2,0.4)}.items():
        m=(z>0.44)&(z<0.47)&(x>x0)&(x<x1)&(y>y0)&(y<y1)
        c=fit_circle(x[m],y[m])
        print(f'  {name}: n={m.sum()} z=[{z[m].min():.3f},{z[m].max():.3f}] circle center=({c[0]:.3f},{c[1]:.3f}) r={c[2]:.3f} mean=({x[m].mean():.3f},{y[m].mean():.3f})')
"


# openrua op 18
python3 -c "
import numpy as np, cv2
def fit_circle(x,y):
    A=np.c_[2*x,2*y,np.ones_like(x)]; b=x**2+y**2
    c=np.linalg.lstsq(A,b,rcond=None)[0]; r=np.sqrt(c[2]+c[0]**2+c[1]**2); return c[0],c[1],r
for cam in ['agentview','birdview']:
    d=np.load(f'{cam}_depth.npy'); T=np.load(f'{cam}_T.npy'); img=cv2.imread(f'{cam}.png')
    fx=fy=579.4112549695428; cx=320; cy=240
    H,W=d.shape
    v,u=np.mgrid[0:H,0:W]
    pc=np.stack([(u-cx)*d/fx,(v-cy)*d/fy,d,np.ones_like(d)],-1)
    pw=pc@T.T
    x,y,z=pw[...,0],pw[...,1],pw[...,2]
    hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    m=(z>0.515)&(z<0.56)&(x>-0.13)&(x<0.0)&(y>0.03)&(y<0.2)
    c=fit_circle(x[m],y[m])
    print(f'{cam} yellow: n={m.sum()} rim z=[{z[m].min():.3f},{z[m].max():.3f}] circle center=({c[0]:.3f},{c[1]:.3f}) r={c[2]:.3f}  extents x=[{x[m].min():.3f},{x[m].max():.3f}] y=[{y[m].min():.3f},{y[m].max():.3f}]')
    # yellow handle: yellow color, low z
    yel=(hsv[...,0]>=18)&(hsv[...,0]<=38)&(hsv[...,1]>90)&(z>0.44)&(z<0.56)&(x>-0.15)&(x<0.0)&(y>0.0)&(y<0.2)
    print(f'   yellow-colored pts n={yel.sum()} x=[{x[yel].min():.3f},{x[yel].max():.3f}] y=[{y[yel].min():.3f},{y[yel].max():.3f}] z=[{z[yel].min():.3f},{z[yel].max():.3f}]')
"


# openrua op 19
ros2 interface show moveit_msgs/srv/GetPositionFK | head -30; timeout 60 ros2 service call /compute_fk moveit_msgs/srv/GetPositionFK "{header: {frame_id: ''}, fk_link_names: ['panda_hand'], robot_state: {joint_state: {name: [panda_joint1,panda_joint2,panda_joint3,panda_joint4,panda_joint5,panda_joint6,panda_joint7], position: [0.0,-0.161037389,0.0,-2.44459747,0.0,2.2267522,0.7853981633974483]}}}" 2>&1 | tail -30

# openrua op 20
timeout 20 ros2 run tf2_ros tf2_echo world robot0_eye_in_hand_optical_frame 2>&1 | grep -A2 "At time" | head -4; timeout 20 ros2 run tf2_ros tf2_echo world panda_hand 2>&1 | grep -A2 -E "At time|unconnected|Invalid" | head -6; ros2 topic echo /tf_static --once 2>/dev/null | grep -E "frame_id|x:|y:|z:" | head -60

# openrua op 21
mkdir -p "$(dirname /workspace/ctl.py)"
cat > /workspace/ctl.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Small controller library for the Panda: IK/FK, trajectories, gripper,
servo bursts, joint-state reads. Import or run as a CLI:

  python3 ctl.py js                         # joint state
  python3 ctl.py fk                         # hand pose (from current joints)
  python3 ctl.py ik x y z qx qy qz qw       # IK only, print joints
  python3 ctl.py move x y z qx qy qz qw [sec] [--tcp]   # IK + trajectory
  python3 ctl.py joints p1,...,p7 [sec]
  python3 ctl.py grip open|close
  python3 ctl.py servo dx dy dz [n_ticks]   # m/s in base frame, ~20 Hz
"""
import sys
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
TW = next(a for a in M["actuators"] if a["kind"] == "cartesian_twist")
ARM = FJT["joints"]
TCP_OFF = float(M["hand"]["tcp_offset_m"])
BASE_IN_WORLD = np.array([-0.51, 0.0, 0.42])


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class Ctl:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("ctl")
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      self._on_js, 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grp = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.tw = self.node.create_publisher(TwistStamped, TW["port"], 10)

    def _on_js(self, msg):
        self._js = dict(zip(msg.name, msg.position))
        self._js_eff = dict(zip(msg.name, msg.effort))

    def spin(self, t=0.1):
        rclpy.spin_once(self.node, timeout_sec=t)

    def js(self, fresh=True):
        if fresh:
            self._js = {}
        while not self._js:
            self.spin(0.2)
        return dict(self._js)

    def arm_q(self):
        j = self.js()
        return [j[n] for n in ARM]

    def finger_gap(self):
        j = self.js()
        return abs(j["panda_finger_joint1"]) + abs(j["panda_finger_joint2"])

    # ---- kinematics --------------------------------------------------
    def _seed(self, q=None):
        q = q if q is not None else self.arm_q()
        s = JointState(); s.name = list(ARM); s.position = [float(v) for v in q]
        return s

    def fk_pose(self, q=None, link="panda_hand"):
        self.fk.wait_for_service(10)
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = [link]
        req.robot_state.joint_state = self._seed(q)
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        r = fut.result()
        p = r.pose_stamped[0].pose
        return (np.array([p.position.x, p.position.y, p.position.z]),
                np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]),
                r.pose_stamped[0].header.frame_id)

    def ik_q(self, pos, quat, seed=None, attempts=3):
        self.ik.wait_for_service(10)
        for _ in range(attempts):
            req = GetPositionIK.Request()
            req.ik_request.group_name = M["planning"]["group"]
            req.ik_request.pose_stamped.header.frame_id = ""
            p = req.ik_request.pose_stamped.pose
            p.position.x, p.position.y, p.position.z = map(float, pos)
            (p.orientation.x, p.orientation.y,
             p.orientation.z, p.orientation.w) = map(float, quat)
            req.ik_request.robot_state.joint_state = self._seed(seed)
            req.ik_request.avoid_collisions = False
            req.ik_request.timeout = Duration(sec=1)
            fut = self.ik.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            r = fut.result()
            if r is not None and r.error_code.val == 1:
                sol = dict(zip(r.solution.joint_state.name,
                               r.solution.joint_state.position))
                return [sol[j] for j in ARM]
            code = None if r is None else r.error_code.val
        raise RuntimeError(f"IK failed (code {code}) for {pos}")

    # ---- motion ------------------------------------------------------
    def move_joints(self, q, seconds=3.0, via=None):
        """One trajectory through optional via points then q."""
        self.fjt.wait_for_server(10)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pts = list(via or []) + [q]
        n = len(pts)
        for i, p in enumerate(pts):
            t = seconds * (i + 1) / n
            pt = JointTrajectoryPoint(positions=[float(v) for v in p])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res)
        code = res.result().result.error_code
        err = np.abs(np.array(self.arm_q()) - np.array(q)).max()
        print(f"traj error_code={code} max_joint_err={err:.4f}")
        return code, err

    def move_pose(self, pos, quat, seconds=3.0, tcp=False, seed=None):
        pos = np.array(pos, float)
        if tcp:
            pos = pos - TCP_OFF * quat_R(*quat)[:, 2]
        q = self.ik_q(pos, quat, seed=seed)
        return q, self.move_joints(q, seconds)

    def grip(self, what):
        self.grp.wait_for_server(10)
        g = GripperCommand.Goal()
        g.command.position = float(GRIP["open_m"] if what == "open" else GRIP["closed_m"])
        g.command.max_effort = float(GRIP["max_effort"])
        fut = self.grp.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=300)
        r = res.result().result
        gap = self.finger_gap()
        print(f"grip {what}: reached={r.reached_goal} stalled={r.stalled} gap={gap:.4f}")
        return gap

    def servo(self, v, ticks=20):
        msg = TwistStamped(); msg.header.frame_id = TW["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, v)
        for _ in range(ticks):
            self.tw.publish(msg); self.spin(0.05)


def main():
    a = sys.argv[1:]
    c = Ctl()
    cmd = a[0]
    if cmd == "js":
        print(c.js())
    elif cmd == "fk":
        p, q, f = c.fk_pose()
        print("frame", f, "pos", p.round(4), "quat", q.round(4))
    elif cmd == "ik":
        print(c.ik_q(list(map(float, a[1:4])), list(map(float, a[4:8]))))
    elif cmd == "move":
        nums = [float(x) for x in a[1:] if not x.startswith("--")]
        sec = nums[7] if len(nums) > 7 else 3.0
        c.move_pose(nums[0:3], nums[3:7], sec, tcp="--tcp" in a)
    elif cmd == "joints":
        c.move_joints([float(x) for x in a[1].split(",")], float(a[2]) if len(a) > 2 else 3.0)
    elif cmd == "grip":
        c.grip(a[1])
    elif cmd == "servo":
        c.servo(list(map(float, a[1:4])), int(a[4]) if len(a) > 4 else 20)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 22
timeout 200 python3 -c "
from ctl import *
c=Ctl()
q0=c.arm_q(); print('current', np.round(q0,3))
p,q,f=c.fk_pose(); print('fk', f, p.round(3), q.round(3))
# IK at the FK pose (world coords)
try:
    s=c.ik_q(p, q); print('IK world-coords sol', np.round(s,3)); print('  fk of sol', c.fk_pose(s)[0].round(3))
except Exception as e: print(e)
# IK at the base-frame coords
try:
    s=c.ik_q(p-BASE_IN_WORLD, q); print('IK base-coords sol', np.round(s,3)); print('  fk of sol', c.fk_pose(s)[0].round(3))
except Exception as e: print(e)
"

# openrua op 23
timeout 600 python3 -c "
from ctl import *
c=Ctl()
Q_Y=[1,0,0,0]   # top-down, fingers close along world y
tcp=[-0.127,-0.203,0.65]
q,(code,err)=c.move_pose(tcp,Q_Y,4.0,tcp=True)
print('q',np.round(q,3))
p,qq,f=c.fk_pose(); print('hand now',p.round(4),qq.round(3), 'tcp z', round(p[2]-TCP_OFF,4))
" && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 24
timeout 100 python3 -c "
from ctl import *
c=Ctl()
q=[-0.15,-0.208,-0.342,-2.568,-0.099,2.369,-0.413]
cur=c.arm_q(); print('cur',np.round(cur,3)); print('diff',np.round(np.array(cur)-q,3))
p,qq,f=c.fk_pose(); print('hand',p.round(4),qq.round(3))
"

# openrua op 25
timeout 300 python3 -c "
from ctl import *
c=Ctl()
q=[-0.15,-0.208,-0.342,-2.568,-0.099,2.369,-0.413]
c.move_joints(q,3.0)
cur=c.arm_q(); print('diff',np.round(np.array(cur)-q,3))
p,qq,f=c.fk_pose(); print('hand',p.round(4),qq.round(3))
"

# openrua op 26
timeout 300 python3 -c "
from ctl import *
c=Ctl()
cur=c.arm_q()
print('cur', np.round(cur,3))
p,qq,f=c.fk_pose(cur); print('FK(cur)',p.round(4),qq.round(3))
sol=c.ik_q([-0.127,-0.203,0.7534],[1,0,0,0], seed=cur)
print('IK sol', np.round(sol,3))
p,qq,f=c.fk_pose(sol); print('FK(sol)',p.round(4),qq.round(3))
sol2=c.ik_q([-0.127,-0.203,0.7534],[1,0,0,0], seed=[0,-0.161,0,-2.445,0,2.227,0.785])
print('IK sol2', np.round(sol2,3))
p,qq,f=c.fk_pose(sol2); print('FK(sol2)',p.round(4),qq.round(3))
"

# openrua op 27
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/ctl.py", "old_string": "            req.ik_request.group_name = M[\\"planning\\"][\\"group\\"]\\n            req.ik_request.pose_stamped.header.frame_id = \\"\\"", "new_string": "            req.ik_request.group_name = M[\\"planning\\"][\\"group\\"]\\n            # the group\'s default tip is panda_link8 (45 deg yawed from the\\n            # hand); solve for the hand frame explicitly\\n            req.ik_request.ik_link_name = \\"panda_hand\\"\\n            req.ik_request.pose_stamped.header.frame_id = \\"\\"", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 28
timeout 300 python3 -c "
from ctl import *
c=Ctl()
cur=c.arm_q()
sol=c.ik_q([-0.127,-0.203,0.7534],[1,0,0,0], seed=cur)
print('IK sol', np.round(sol,3))
p,qq,f=c.fk_pose(sol); print('FK(sol)',p.round(4),qq.round(3))
"

# openrua op 29
timeout 600 python3 -c "
from ctl import *
c=Ctl()
q,(code,err)=c.move_pose([-0.127,-0.203,0.65],[1,0,0,0],3.0,tcp=True)
if err>0.01: c.move_joints(q,2.0)
p,qq,f=c.fk_pose(); print('hand',p.round(4),qq.round(3),'tcp z',round(p[2]-TCP_OFF,4))
" && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 30
timeout 120 python3 scene.py robot0_eye_in_hand 250,310 && python3 -c "
import numpy as np
T=np.load('robot0_eye_in_hand_T.npy'); print(T.round(3))
" && ros2 topic echo /robot0_eye_in_hand/color/camera_info --once | grep -A3 '^k:'

# openrua op 31
python3 -c "
import numpy as np, cv2
def fit_circle(x,y):
    A=np.c_[2*x,2*y,np.ones_like(x)]; b=x**2+y**2
    c=np.linalg.lstsq(A,b,rcond=None)[0]; r=np.sqrt(c[2]+c[0]**2+c[1]**2); return c[0],c[1],r
cam='robot0_eye_in_hand'
d=np.load(f'{cam}_depth.npy'); T=np.load(f'{cam}_T.npy')
fx=fy=312.77408948188935; cx=320; cy=240
H,W=d.shape
v,u=np.mgrid[0:H,0:W]
pc=np.stack([(u-cx)*d/fx,(v-cy)*d/fy,d,np.ones_like(d)],-1)
pw=pc@T.T
x,y,z=pw[...,0],pw[...,1],pw[...,2]
m=(z>0.52)&(z<0.56)&(x>-0.2)&(x<-0.05)&(y>-0.25)&(y<-0.1)
c=fit_circle(x[m],y[m]); print('white rim from wrist cam: n',m.sum(),'center',np.round(c,4),'z',z[m].min().round(3),z[m].max().round(3))
m2=(z>0.60)&(z<0.70); print('fingers: n',m2.sum(),'x',x[m2].min().round(3),x[m2].max().round(3),'y',y[m2].min().round(3),y[m2].max().round(3),'z',z[m2].min().round(3),z[m2].max().round(3))
"


# openrua op 32
timeout 600 python3 -c "
from ctl import *
c=Ctl()
q,(code,err)=c.move_pose([-0.127,-0.203,0.524],[1,0,0,0],2.5,tcp=True)
if err>0.01: c.move_joints(q,1.5)
p,qq,f=c.fk_pose(); print('hand',p.round(4),qq.round(3),'tcp z',round(p[2]-TCP_OFF,4))
gap=c.grip('close')
p,qq,f=c.fk_pose(); print('after close hand',p.round(4))
" && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 33
timeout 600 python3 -c "
from ctl import *
c=Ctl()
q,(code,err)=c.move_pose([-0.127,-0.203,0.70],[1,0,0,0],2.5,tcp=True)
if err>0.01: c.move_joints(q,1.5)
p,qq,f=c.fk_pose(); print('hand',p.round(4),'tcp z',round(p[2]-TCP_OFF,4),'gap',round(c.finger_gap(),4))
" && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 34
timeout 900 python3 -c "
from ctl import *
c=Ctl()
Q=[1,0,0,0]
def tcp2hand(t): return np.array(t)-TCP_OFF*quat_R(*Q)[:,2]
q1=c.ik_q(tcp2hand([-0.06,0.03,0.72]),Q)
q2=c.ik_q(tcp2hand([0.007,0.257,0.72]),Q,seed=q1)
print('q1',np.round(q1,3)); print('q2',np.round(q2,3))
code,err=c.move_joints(q2,5.0,via=[q1])
if err>0.01: c.move_joints(q2,2.0)
p,qq,f=c.fk_pose(); print('hand',p.round(4),qq.round(3),'tcp z',round(p[2]-TCP_OFF,4),'gap',round(c.finger_gap(),4))
" && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 35
timeout 120 python3 scene.py robot0_eye_in_hand 320,240 && python3 -c "
import numpy as np, cv2
def fit_circle(x,y):
    A=np.c_[2*x,2*y,np.ones_like(x)]; b=x**2+y**2
    c=np.linalg.lstsq(A,b,rcond=None)[0]; r=np.sqrt(c[2]+c[0]**2+c[1]**2); return c[0],c[1],r
cam='robot0_eye_in_hand'
d=np.load(f'{cam}_depth.npy'); T=np.load(f'{cam}_T.npy')
fx=fy=312.77408948188935; cx=320; cy=240
H,W=d.shape
v,u=np.mgrid[0:H,0:W]
pc=np.stack([(u-cx)*d/fx,(v-cy)*d/fy,d,np.ones_like(d)],-1)
pw=pc@T.T
x,y,z=pw[...,0],pw[...,1],pw[...,2]
m=(z>0.44)&(z<0.47)&(np.abs(x)<0.15)&(y>0.15)&(y<0.45)
if m.sum()>10:
    c=fit_circle(x[m],y[m]); print('plate visible n',m.sum(),'center',np.round(c,4),'ext x',x[m].min().round(3),x[m].max().round(3),'y',y[m].min().round(3),y[m].max().round(3))
m=(z>0.5)&(z<0.62)&(np.abs(x)<0.2)&(y>0.1)&(y<0.45)
print('mug pts n',m.sum(),'x',x[m].min().round(3),x[m].max().round(3),'y',y[m].min().round(3),y[m].max().round(3),'z',z[m].min().round(3),z[m].max().round(3))
"

# openrua op 36
timeout 900 python3 -c "
from ctl import *
c=Ctl()
Q=[1,0,0,0]
q,(code,err)=c.move_pose([0.007,0.257,0.57],Q,3.0,tcp=True)
if err>0.01: c.move_joints(q,1.5)
p,qq,f=c.fk_pose(); print('hand',p.round(4),'tcp z',round(p[2]-TCP_OFF,4),'gap',round(c.finger_gap(),4))
c.grip('open')
q,(code,err)=c.move_pose([0.007,0.257,0.70],Q,2.5,tcp=True)
if err>0.01: c.move_joints(q,1.5)
p,qq,f=c.fk_pose(); print('hand',p.round(4),'tcp z',round(p[2]-TCP_OFF,4))
" && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 37
timeout 120 python3 scene.py birdview 320,400 >/dev/null && timeout 120 python3 scene.py agentview 320,400 > /dev/null && python3 -c "
import numpy as np, cv2
def fit_circle(x,y):
    A=np.c_[2*x,2*y,np.ones_like(x)]; b=x**2+y**2
    c=np.linalg.lstsq(A,b,rcond=None)[0]; r=np.sqrt(c[2]+c[0]**2+c[1]**2); return c[0],c[1],r
for cam in ['agentview','birdview']:
    d=np.load(f'{cam}_depth.npy'); T=np.load(f'{cam}_T.npy')
    fx=fy=579.4112549695428; cx=320; cy=240
    H,W=d.shape
    v,u=np.mgrid[0:H,0:W]
    pc=np.stack([(u-cx)*d/fx,(v-cy)*d/fy,d,np.ones_like(d)],-1)
    pw=pc@T.T
    x,y,z=pw[...,0],pw[...,1],pw[...,2]
    print(cam)
    for name,(x0,x1,y0,y1,zlo,zhi) in {'yellow rim':(-0.13,0.0,0.03,0.2,0.515,0.56),'white rim (on plate)':(-0.1,0.12,0.2,0.42,0.53,0.6),'red rim':(-0.25,-0.13,-0.07,0.09,0.555,0.6),'Lplate':(-0.1,0.1,-0.42,-0.22,0.44,0.47)}.items():
        m=(z>zlo)&(z<zhi)&(x>x0)&(x<x1)&(y>y0)&(y<y1)
        if m.sum()<10: print(' ',name,'few pts',m.sum()); continue
        c=fit_circle(x[m],y[m])
        print(f'  {name}: n={m.sum()} z=[{z[m].min():.3f},{z[m].max():.3f}] circle=({c[0]:.3f},{c[1]:.3f}) r={c[2]:.3f} ext x=[{x[m].min():.3f},{x[m].max():.3f}] y=[{y[m].min():.3f},{y[m].max():.3f}]')
"

# openrua op 38
timeout 900 python3 -c "
from ctl import *
c=Ctl()
QX=[0.70710678,0.70710678,0,0]
print('R hand cols', quat_R(*QX).round(2))
q,(code,err)=c.move_pose([-0.018,0.12,0.65],QX,4.0,tcp=True)
if err>0.01: c.move_joints(q,2.0)
p,qq,f=c.fk_pose(); print('hand',p.round(4),qq.round(3),'tcp z',round(p[2]-TCP_OFF,4))
" && timeout 120 python3 scene.py robot0_eye_in_hand 320,240 && python3 -c "
import numpy as np
def fit_circle(x,y):
    A=np.c_[2*x,2*y,np.ones_like(x)]; b=x**2+y**2
    c=np.linalg.lstsq(A,b,rcond=None)[0]; r=np.sqrt(c[2]+c[0]**2+c[1]**2); return c[0],c[1],r
cam='robot0_eye_in_hand'
d=np.load(f'{cam}_depth.npy'); T=np.load(f'{cam}_T.npy')
fx=fy=312.77408948188935; cx=320; cy=240
H,W=d.shape
v,u=np.mgrid[0:H,0:W]
pc=np.stack([(u-cx)*d/fx,(v-cy)*d/fy,d,np.ones_like(d)],-1)
pw=pc@T.T
x,y,z=pw[...,0],pw[...,1],pw[...,2]
m=(z>0.51)&(z<0.55)&(x>-0.15)&(x<0.05)&(y>0.05)&(y<0.2)
c=fit_circle(x[m],y[m]); print('yellow rim from wrist cam: n',m.sum(),'center',np.round(c,4),'z',z[m].min().round(3),z[m].max().round(3))
m2=(z>0.60)&(z<0.70); print('fingers: n',m2.sum(),'x',x[m2].min().round(3),x[m2].max().round(3),'y',y[m2].min().round(3),y[m2].max().round(3))
"

# openrua op 39
timeout 900 python3 -c "
from ctl import *
c=Ctl()
QX=[0.70710678,0.70710678,0,0]
q=c.ik_q(np.array([-0.018,0.12,0.65])-TCP_OFF*quat_R(*QX)[:,2],QX)
print('target q',np.round(q,3)); print('cur',np.round(c.arm_q(),3))
for i in range(4):
    code,err=c.move_joints(q,4.0)
    if err<0.01: break
p,qq,f=c.fk_pose(); print('hand',p.round(4),qq.round(3),'tcp z',round(p[2]-TCP_OFF,4))
"

# openrua op 40
timeout 120 python3 scene.py robot0_eye_in_hand 320,240 >/dev/null && python3 -c "
import numpy as np
def fit_circle(x,y):
    A=np.c_[2*x,2*y,np.ones_like(x)]; b=x**2+y**2
    c=np.linalg.lstsq(A,b,rcond=None)[0]; r=np.sqrt(c[2]+c[0]**2+c[1]**2); return c[0],c[1],r
cam='robot0_eye_in_hand'
d=np.load(f'{cam}_depth.npy'); T=np.load(f'{cam}_T.npy')
fx=fy=312.77408948188935; cx=320; cy=240
H,W=d.shape
v,u=np.mgrid[0:H,0:W]
pc=np.stack([(u-cx)*d/fx,(v-cy)*d/fy,d,np.ones_like(d)],-1)
pw=pc@T.T
x,y,z=pw[...,0],pw[...,1],pw[...,2]
m=(z>0.51)&(z<0.55)&(x>-0.15)&(x<0.05)&(y>0.05)&(y<0.2)
c=fit_circle(x[m],y[m]); print('yellow rim: n',m.sum(),'center',np.round(c,4),'z',z[m].min().round(3),z[m].max().round(3))
m2=(z>0.60)&(z<0.70); print('fingers: x',x[m2].min().round(3),x[m2].max().round(3),'y',y[m2].min().round(3),y[m2].max().round(3))
"

# openrua op 41
timeout 900 python3 -c "
from ctl import *
c=Ctl()
QX=[0.70710678,0.70710678,0,0]
q,(code,err)=c.move_pose([-0.018,0.125,0.517],QX,2.5,tcp=True)
for i in range(3):
    if err<0.01: break
    code,err=c.move_joints(q,2.0)
p,qq,f=c.fk_pose(); print('hand',p.round(4),qq.round(3),'tcp z',round(p[2]-TCP_OFF,4))
c.grip('close')
q,(code,err)=c.move_pose([-0.018,0.125,0.72],QX,2.5,tcp=True)
for i in range(3):
    if err<0.01: break
    code,err=c.move_joints(q,2.0)
p,qq,f=c.fk_pose(); print('hand',p.round(4),qq.round(3),'tcp z',round(p[2]-TCP_OFF,4),'gap',round(c.finger_gap(),4))
" && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 42
timeout 1200 python3 -c "
from ctl import *
c=Ctl()
QX=[0.70710678,0.70710678,0,0]
def h(t): return np.array(t)-TCP_OFF*quat_R(*QX)[:,2]
q1=c.ik_q(h([0.01,-0.10,0.72]),QX)
q2=c.ik_q(h([0.034,-0.325,0.72]),QX,seed=q1)
code,err=c.move_joints(q2,5.0,via=[q1])
for i in range(3):
    if err<0.01: break
    code,err=c.move_joints(q2,3.0)
p,qq,f=c.fk_pose(); print('hand',p.round(4),qq.round(3),'tcp z',round(p[2]-TCP_OFF,4),'gap',round(c.finger_gap(),4))
" && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 43
timeout 1200 python3 -c "
from ctl import *
c=Ctl()
QX=[0.70710678,0.70710678,0,0]
q,(code,err)=c.move_pose([0.034,-0.325,0.57],QX,3.0,tcp=True)
for i in range(3):
    if err<0.01: break
    code,err=c.move_joints(q,2.0)
p,qq,f=c.fk_pose(); print('hand',p.round(4),'tcp z',round(p[2]-TCP_OFF,4),'gap',round(c.finger_gap(),4))
c.grip('open')
q,(code,err)=c.move_pose([0.034,-0.325,0.72],QX,2.5,tcp=True)
for i in range(3):
    if err<0.01: break
    code,err=c.move_joints(q,2.0)
p,qq,f=c.fk_pose(); print('hand',p.round(4),'tcp z',round(p[2]-TCP_OFF,4))
" && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 44
timeout 120 python3 scene.py birdview 320,400 >/dev/null && timeout 120 python3 scene.py agentview 320,400 >/dev/null && python3 -c "
import numpy as np
def fit_circle(x,y):
    A=np.c_[2*x,2*y,np.ones_like(x)]; b=x**2+y**2
    c=np.linalg.lstsq(A,b,rcond=None)[0]; r=np.sqrt(c[2]+c[0]**2+c[1]**2); return c[0],c[1],r
for cam in ['agentview','birdview']:
    d=np.load(f'{cam}_depth.npy'); T=np.load(f'{cam}_T.npy')
    fx=fy=579.4112549695428; cx=320; cy=240
    H,W=d.shape
    v,u=np.mgrid[0:H,0:W]
    pc=np.stack([(u-cx)*d/fx,(v-cy)*d/fy,d,np.ones_like(d)],-1)
    pw=pc@T.T
    x,y,z=pw[...,0],pw[...,1],pw[...,2]
    print(cam)
    for name,(x0,x1,y0,y1,zlo,zhi) in {'yellow mug rim':(-0.15,0.15,-0.45,-0.2,0.52,0.60),'white mug rim':(-0.15,0.15,0.2,0.45,0.52,0.60),'left plate':(-0.15,0.15,-0.45,-0.2,0.44,0.47),'right plate':(-0.15,0.15,0.2,0.45,0.44,0.47),'red mug rim':(-0.3,-0.1,-0.1,0.1,0.55,0.6)}.items():
        m=(z>zlo)&(z<zhi)&(x>x0)&(x<x1)&(y>y0)&(y<y1)
        if m.sum()<10: print(' ',name,'few pts',m.sum()); continue
        c=fit_circle(x[m],y[m])
        print(f'  {name}: n={m.sum()} z=[{z[m].min():.3f},{z[m].max():.3f}] center=({c[0]:.3f},{c[1]:.3f}) r={c[2]:.3f}')
"

# openrua op 45
timeout 900 python3 -c "
from ctl import *
c=Ctl()
home=[0.0,-0.161037389,0.0,-2.44459747,0.0,2.2267522,0.7853981633974483]
for i in range(3):
    code,err=c.move_joints(home,4.0)
    if err<0.01: break
p,qq,f=c.fk_pose(); print('hand',p.round(4),qq.round(3))
" && timeout 60 python3 tools/perception/cam_snap.py agentview && timeout 60 python3 tools/perception/cam_snap.py frontview
