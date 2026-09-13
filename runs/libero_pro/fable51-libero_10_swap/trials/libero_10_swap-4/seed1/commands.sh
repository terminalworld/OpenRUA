#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 2 -type f | head -50

# openrua op 1
cat /workspace/tools/README.md; ls -R /workspace/tools /workspace/.ros

# openrua op 2
cat tools/action/ik_move.py tools/action/fjt_send.py tools/action/gripper_cmd.py

# openrua op 3
cat tools/perception/cam_snap.py tools/perception/px2world.py

# openrua op 4
timeout 30 ros2 topic list; echo ---; timeout 20 ros2 action list; echo ---; timeout 20 ros2 service list | grep -iv parameter | head -40; echo ---; timeout 20 ros2 node list

# openrua op 5
timeout 30 ros2 topic echo /joint_states --once; for c in agentview frontview birdview sideview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c; done

# openrua op 6
timeout 30 ros2 topic echo /tf_static --once 2>&1 | head -120; echo ---; timeout 20 ros2 topic echo /agentview/color/camera_info --once

# openrua op 7
mkdir -p "$(dirname /workspace/tfdump.py)"
cat > /workspace/tfdump.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Dump all TF frames (world -> X) once, plus FK of the hand via /compute_fk."""
import rclpy, yaml
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState

rclpy.init()
node = rclpy.create_node("tfdump")
buf = Buffer(); TransformListener(buf, node)
for _ in range(20):
    rclpy.spin_once(node, timeout_sec=0.2)
frames = yaml.safe_load(buf.all_frames_as_yaml()) or {}
print("frames:", sorted(frames.keys()))
for f in sorted(frames):
    for root in ("world", "panda_link0"):
        try:
            t = buf.lookup_transform(root, f, Time())
            tr, q = t.transform.translation, t.transform.rotation
            print(f"{root}->{f}: t=({tr.x:.4f},{tr.y:.4f},{tr.z:.4f}) q=({q.x:.4f},{q.y:.4f},{q.z:.4f},{q.w:.4f})")
        except Exception as e:
            print(f"{root}->{f}: FAIL {type(e).__name__}")

# FK of the hand
js = {}
node.create_subscription(JointState, "/joint_states", lambda m: js.setdefault("m", m), 1)
while "m" not in js:
    rclpy.spin_once(node, timeout_sec=0.2)
cli = node.create_client(GetPositionFK, "/compute_fk")
cli.wait_for_service(timeout_sec=10)
req = GetPositionFK.Request()
req.fk_link_names = ["panda_hand", "panda_link8"]
arm = [f"panda_joint{i}" for i in range(1, 8)]
seed = JointState()
for n, p in zip(js["m"].name, js["m"].position):
    if n in arm:
        seed.name.append(n); seed.position.append(p)
req.robot_state.joint_state = seed
fut = cli.call_async(req)
rclpy.spin_until_future_complete(node, fut, timeout_sec=30)
res = fut.result()
if res:
    for n, ps in zip(res.fk_link_names, res.pose_stamped):
        p, q = ps.pose.position, ps.pose.orientation
        print(f"FK {n} [{ps.header.frame_id}]: p=({p.x:.4f},{p.y:.4f},{p.z:.4f}) q=({q.x:.4f},{q.y:.4f},{q.z:.4f},{q.w:.4f})")
rclpy.shutdown()
OPENRUA_EOF

# openrua op 8
timeout 90 python3 tfdump.py 2>&1 | grep -v FAIL

# openrua op 9
mkdir -p "$(dirname /workspace/cloud.py)"
cat > /workspace/cloud.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Grab color+depth+info from a camera, save world-frame point cloud (npz) and PNG.
Usage: python3 cloud.py <camera>
"""
import sys
import numpy as np
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time
import cv2


def grab(node, topic, typ, timeout=30):
    got = {}
    sub = node.create_subscription(typ, topic, lambda m: got.setdefault("m", m), 1)
    end = node.get_clock().now().nanoseconds / 1e9 + timeout
    while "m" not in got and node.get_clock().now().nanoseconds / 1e9 < end:
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
    rclpy.init()
    node = rclpy.create_node("cloud")
    buf = Buffer(); TransformListener(buf, node)
    br = CvBridge()
    color = br.imgmsg_to_cv2(grab(node, f"/{cam}/color/image_raw", Image), "bgr8")
    depth = br.imgmsg_to_cv2(grab(node, f"/{cam}/depth/image_raw", Image), "passthrough").astype(np.float32)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    frame = f"{cam}_optical_frame"
    while not buf.can_transform("world", frame, Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    t = buf.lookup_transform("world", frame, Time())
    q = t.transform.rotation
    R = quat_R(q.x, q.y, q.z, q.w)
    T = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    h, w = depth.shape
    u, v = np.meshgrid(np.arange(w), np.arange(h))
    X = (u - cx) * depth / fx
    Y = (v - cy) * depth / fy
    P = np.stack([X, Y, depth], -1) @ R.T + T
    np.savez(f"{cam}_cloud.npz", P=P, color=color, depth=depth)
    cv2.imwrite(f"{cam}.png", color)
    print("saved", cam, "depth range", np.nanmin(depth), np.nanmax(depth))
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 10
timeout 120 python3 cloud.py birdview && timeout 120 python3 cloud.py agentview

# openrua op 11
python3 -c "
import numpy as np, cv2
d=np.load('birdview_cloud.npz'); P=d['P']; C=d['color']
z=P[...,2]
# histogram of heights
hist,edges=np.histogram(z[np.isfinite(z)],bins=60,range=(0,1.2))
for h,e in zip(hist,edges):
    if h>50: print(f'{e:.3f} {h}')
"


# openrua op 12
python3 -c "
import numpy as np, cv2
d=np.load('birdview_cloud.npz'); P=d['P']; C=d['color']
z=P[...,2]
mask=((z>0.428)&(z<0.65)).astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,4]<30: continue
    m=lab==i
    pts=P[m]; col=C[m].mean(0)
    print(f'comp{i}: px={stats[i,4]} centroid_px=({cent[i][0]:.0f},{cent[i][1]:.0f}) world x=[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y=[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] zmax={pts[:,2].max():.3f} mean=({pts[:,0].mean():.3f},{pts[:,1].mean():.3f}) bgr={col.astype(int)}')
"


# openrua op 13
python3 -c "
import numpy as np, cv2
d=np.load('agentview_cloud.npz'); P=d['P']; C=d['color']
z=P[...,2]; x=P[...,0]; y=P[...,1]
ok=np.isfinite(z)&(x>-0.45)&(x<0.4)&(np.abs(y)<0.6)
print('table z near plates:', np.percentile(z[ok&(z<0.435)],[5,50,95]))
mask=(ok&(z>0.432)&(z<0.8)).astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,4]<40: continue
    m=lab==i; pts=P[m]; col=C[m].mean(0)
    print(f'comp{i}: px={stats[i,4]} px_bbox={stats[i,:4]} x=[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y=[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] z=[{pts[:,2].min():.3f},{pts[:,2].max():.3f}] bgr={col.astype(int)}')
"


# openrua op 14
timeout 120 python3 tools/action/gripper_cmd.py 0.04; timeout 20 ros2 topic echo /joint_states --once | grep -A9 position

# openrua op 15
timeout 120 python3 cloud.py agentview && python3 -c "
import numpy as np, cv2
d=np.load('agentview_cloud.npz'); P=d['P']; C=d['color']
z=P[...,2]; x=P[...,0]; y=P[...,1]
ok=np.isfinite(z)&(x>-0.45)&(x<0.4)&(np.abs(y)<0.6)
print('table z:', np.percentile(z[ok&(z<0.435)],[5,50,95]))
mask=(ok&(z>0.432)&(z<0.8)).astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,4]<40: continue
    m=lab==i; pts=P[m]; col=C[m].mean(0)
    print(f'comp{i}: px={stats[i,4]} px_bbox={stats[i,:4]} x=[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y=[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] z=[{pts[:,2].min():.3f},{pts[:,2].max():.3f}] bgr={col.astype(int)}')
"

# openrua op 16
timeout 120 python3 cloud.py birdview && python3 -c "
import numpy as np, cv2
d=np.load('birdview_cloud.npz'); P=d['P']; C=d['color']
z=P[...,2]
mask=((z>0.46)&(z<0.65)).astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,4]<30: continue
    m=lab==i
    pts=P[m]; col=C[m].mean(0)
    print(f'comp{i}: px={stats[i,4]} bbox={stats[i,:4]} x=[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y=[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] zmax={pts[:,2].max():.3f} mean=({pts[:,0].mean():.3f},{pts[:,1].mean():.3f}) bgr={col.astype(int)}')
# plates: thin ring 0.435-0.46
mask=((z>0.437)&(z<0.462)).astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,4]<100: continue
    m=lab==i; pts=P[m]
    print(f'plate-ish comp{i}: px={stats[i,4]} x=[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y=[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] center=({(pts[:,0].min()+pts[:,0].max())/2:.3f},{(pts[:,1].min()+pts[:,1].max())/2:.3f})')
"

# openrua op 17
python3 -c "
import numpy as np
d=np.load('agentview_cloud.npz'); P=d['P']; C=d['color']
x,y,z=P[...,0],P[...,1],P[...,2]
for name,(x0,x1,y0,y1) in {'white':(-0.16,-0.04,-0.22,-0.06),'yellow':(-0.28,-0.17,-0.06,0.10),'red':(-0.12,-0.02,0.09,0.19)}.items():
    m=(x>x0)&(x<x1)&(y>y0)&(y<y1)&(z>0.44)&(z<0.6)
    print(name, 'n',m.sum(),'y range',y[m].min().round(3),y[m].max().round(3),'x range',x[m].min().round(3),x[m].max().round(3),'z',z[m].min().round(3),z[m].max().round(3))
    for zl in [0.46,0.48,0.50,0.52,0.54]:
        mm=m&(z>zl)&(z<zl+0.02)
        if mm.sum(): print(f'  z{zl}: y[{y[mm].min():.3f},{y[mm].max():.3f}] x[{x[mm].min():.3f},{x[mm].max():.3f}] n={mm.sum()}')
"

# openrua op 18
python3 -c "
import cv2
im=cv2.imread('agentview.png')
crop=im[150:330,140:480]
cv2.imwrite('crop_mugs.png', cv2.resize(crop,None,fx=2,fy=2,interpolation=cv2.INTER_CUBIC))
"

# openrua op 19
python3 -c "
import numpy as np
d=np.load('birdview_cloud.npz'); P=d['P']
x,y,z=P[...,0],P[...,1],P[...,2]
for name,(x0,x1,y0,y1) in {'white':(-0.17,-0.03,-0.23,-0.05),'yellow':(-0.29,-0.16,-0.07,0.11),'red':(-0.13,-0.01,0.08,0.20)}.items():
    for zl in [0.44,0.46,0.48,0.50,0.52,0.53,0.54,0.55,0.56]:
        m=(x>x0)&(x<x1)&(y>y0)&(y<y1)&(z>zl)
        if m.sum(): print(f'{name} z>{zl}: n={m.sum()} x[{x[m].min():.3f},{x[m].max():.3f}] y[{y[m].min():.3f},{y[m].max():.3f}] mean=({x[m].mean():.3f},{y[m].mean():.3f})')
"

# openrua op 20
python3 -c "
import numpy as np
d=np.load('birdview_cloud.npz'); P=d['P']
x,y,z=P[...,0],P[...,1],P[...,2]
def fit(pts):
    # algebraic circle fit
    A=np.c_[2*pts[:,0],2*pts[:,1],np.ones(len(pts))]; b=(pts**2).sum(1)
    c=np.linalg.lstsq(A,b,rcond=None)[0]; r=np.sqrt(c[2]+c[0]**2+c[1]**2); return c[0],c[1],r
for name,(x0,x1,y0,y1) in {'white':(-0.17,-0.03,-0.23,-0.05),'yellow':(-0.29,-0.16,-0.07,0.11),'red':(-0.13,-0.01,0.08,0.20)}.items():
    m=(x>x0)&(x<x1)&(y>y0)&(y<y1)&(z>0.50)&(z<0.65)
    pts=np.c_[x[m],y[m]]
    cx,cy,r=fit(pts)
    dist=np.hypot(pts[:,0]-cx,pts[:,1]-cy)
    keep=dist<r*1.15  # drop handle-ish outliers
    cx,cy,r=fit(pts[keep])
    dist=np.hypot(pts[:,0]-cx,pts[:,1]-cy)
    print(f'{name}: n={m.sum()} center=({cx:.3f},{cy:.3f}) r={r:.3f} zmax={z[m].max():.3f}  dist pct',np.percentile(dist,[5,50,95]).round(3))
"

# openrua op 21
mkdir -p "$(dirname /workspace/rob.py)"
cat > /workspace/rob.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Reusable robot helpers: joint state, FK, IK, trajectory, gripper, twist."""
import math
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
BASE_IN_WORLD = np.array([-0.51, 0.0, 0.42])  # world->panda_link0 (from TF)
TCP = M["hand"]["tcp_offset_m"]


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def yaw_down_quat(yaw):
    """Hand pointing straight down (hand z = -world z), hand x rotated by yaw about world z.
    yaw=0: hand x = world x, hand y = -world y (fingers close along world y)."""
    # q = Rz(yaw) * Rx(pi)
    cz, sz = math.cos(yaw / 2), math.sin(yaw / 2)
    # Rx(pi) = (1,0,0,0); Rz(yaw) = (0,0,sz,cz); product Rz*Rx:
    # (w1w2 - v1.v2, w1 v2 + w2 v1 + v1 x v2)
    w = cz * 0 - (0 * 1 + 0 * 0 + sz * 0)
    v = cz * np.array([1, 0, 0]) + 0 * np.array([0, 0, sz]) + np.cross([0, 0, sz], [1, 0, 0])
    return (v[0], v[1], v[2], w)


class Robot:
    def __init__(self, name="rob"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik_cli = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.tw_pub = self.node.create_publisher(TwistStamped, TW["port"], 10)
        self.fjt.wait_for_server(timeout_sec=20)
        self.grip.wait_for_server(timeout_sec=20)
        self.ik_cli.wait_for_service(timeout_sec=20)
        self.fk_cli.wait_for_service(timeout_sec=20)
        while self._js is None:
            rclpy.spin_once(self.node, timeout_sec=0.2)

    def _on_js(self, m):
        self._js = m

    def spin(self, n=3):
        for _ in range(n):
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def joints(self):
        self._js = None
        while self._js is None:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        d = dict(zip(self._js.name, self._js.position))
        return d

    def arm_q(self):
        d = self.joints()
        return [d[j] for j in ARM]

    def finger_gap(self):
        d = self.joints()
        return abs(d["panda_finger_joint1"]) + abs(d["panda_finger_joint2"])

    def _seed(self, q=None):
        js = JointState()
        q = q if q is not None else self.arm_q()
        js.name = list(ARM)
        js.position = [float(v) for v in q]
        return js

    def fk(self, q=None, link="panda_hand"):
        req = GetPositionFK.Request()
        req.fk_link_names = [link]
        req.robot_state.joint_state = self._seed(q)
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        ps = res.pose_stamped[0]
        p, o = ps.pose.position, ps.pose.orientation
        return np.array([p.x, p.y, p.z]), np.array([o.x, o.y, o.z, o.w]), ps.header.frame_id

    def ik(self, pos, quat, seed=None, at_tcp=False, attempts=3):
        """pos in the frame IK uses (see test); quat (x,y,z,w). Returns arm q or None."""
        pos = np.array(pos, dtype=float)
        if at_tcp:
            R = quat_R(*quat)
            pos = pos - TCP * R[:, 2]
        for _ in range(attempts):
            req = GetPositionIK.Request()
            req.ik_request.group_name = M["planning"]["group"]
            req.ik_request.pose_stamped.header.frame_id = ""
            p = req.ik_request.pose_stamped.pose
            p.position.x, p.position.y, p.position.z = map(float, pos)
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
            req.ik_request.robot_state.joint_state = self._seed(seed)
            req.ik_request.avoid_collisions = False
            req.ik_request.timeout = Duration(sec=2)
            fut = self.ik_cli.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            res = fut.result()
            if res is not None and res.error_code.val == 1:
                sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
                return [sol[j] for j in ARM]
        print("IK failed", None if res is None else res.error_code.val)
        return None

    def move_q(self, q, seconds=3.0, waypoints=None):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pts = []
        wps = (waypoints or []) + [q]
        for i, w in enumerate(wps):
            t = seconds * (i + 1) / len(wps)
            pt = JointTrajectoryPoint(positions=[float(v) for v in w])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        err = np.max(np.abs(np.array(self.arm_q()) - np.array(q)))
        print(f"move_q: error_code={code} max_joint_err={err:.4f}")
        return code, err

    def move_pose(self, pos, quat, seconds=3.0, at_tcp=True, seed=None):
        q = self.ik(pos, quat, seed=seed, at_tcp=at_tcp)
        if q is None:
            return None
        return self.move_q(q, seconds)

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result
        gap = self.finger_gap()
        print(f"gripper({width}): reached={r.reached_goal} stalled={r.stalled} gap={gap:.4f}")
        return gap

    def twist(self, lin, ang=(0, 0, 0), ticks=20):
        msg = TwistStamped()
        msg.header.frame_id = TW["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, lin)
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = map(float, ang)
        for _ in range(ticks):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.tw_pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def tcp_world(self):
        p, q, f = self.fk()
        R = quat_R(*q)
        return p + TCP * R[:, 2], q, f
OPENRUA_EOF

# openrua op 22
timeout 200 python3 -c "
from rob import *
r=Robot('iktest')
p,q,f=r.fk(); print('FK hand', p.round(4), q.round(4), f)
q0=r.arm_q(); print('q0', np.round(q0,4))
print('yaw_down_quat(0)', np.round(yaw_down_quat(0),4))
# IK for current pose given in world coords vs base coords
for label,pos in [('world',p),('base',p-BASE_IN_WORLD)]:
    s=r.ik(pos,q)
    print(label, None if s is None else np.round(s,4))
"

# openrua op 23
timeout 200 python3 -c "
from rob import *
r=Robot('iktest2')
sol=[0.0585,-0.1613,-0.0579,-2.4446,-0.0116,2.2267,0.0089]
p,q,f=r.fk(sol); print('FK of world-IK solution:', p.round(4), q.round(4))
p,q,f=r.fk(); print('FK current:', p.round(4), q.round(4))
# Try IK for a target above the white mug in world coords
tgt=np.array([-0.103,-0.193,0.65])
s=r.ik(tgt, yaw_down_quat(0), at_tcp=True)
print('sol', None if s is None else np.round(s,4))
if s is not None:
    p,q,f=r.fk(s); R=quat_R(*q); print('FK hand',p.round(4),q.round(4),'TCP',(p+TCP*R[:,2]).round(4))
"

# openrua op 24
timeout 200 python3 -c "
from rob import *
import math
def qmul(a,b):
    x1,y1,z1,w1=a; x2,y2,z2,w2=b
    return (w1*x2+x1*w2+y1*z2-z1*y2, w1*y2-x1*z2+y1*w2+z1*x2, w1*z2+x1*y2-y1*x2+z1*w2, w1*w2-x1*x2-y1*y2-z1*z2)
r=Robot('iktest3')
qh=yaw_down_quat(0)
q8=qmul(qh,(0,0,math.sin(math.pi/8),math.cos(math.pi/8)))
print('q8',np.round(q8,4))
tgt=np.array([-0.103,-0.193,0.65])
s=r.ik(tgt, q8, at_tcp=True)
print('sol', None if s is None else np.round(s,4))
if s is not None:
    p,q,f=r.fk(s); R=quat_R(*q); print('FK hand',p.round(4),q.round(4),'TCP',(p+TCP*R[:,2]).round(4))
"

# openrua op 25
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rob.py", "old_string": "def yaw_down_quat(yaw):", "new_string": "def qmul(a, b):\\n    x1, y1, z1, w1 = a\\n    x2, y2, z2, w2 = b\\n    return (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,\\n            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,\\n            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,\\n            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)\\n\\n\\n# machine fact (measured): /compute_ik solves for panda_link8, which is the\\n# hand frame rotated +45 deg about z; positions are in WORLD coordinates.\\nHAND_TO_LINK8 = (0.0, 0.0, math.sin(math.pi / 8), math.cos(math.pi / 8))\\n\\n\\ndef yaw_down_quat(yaw):", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 26
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rob.py", "old_string": "        pos = np.array(pos, dtype=float)\\n        if at_tcp:\\n            R = quat_R(*quat)\\n            pos = pos - TCP * R[:, 2]\\n        for _ in range(attempts):", "new_string": "        pos = np.array(pos, dtype=float)\\n        if at_tcp:\\n            R = quat_R(*quat)\\n            pos = pos - TCP * R[:, 2]\\n        quat = qmul(quat, HAND_TO_LINK8)  # hand orientation -> link8 orientation\\n        for _ in range(attempts):", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 27
timeout 600 python3 -u -c "
from rob import *
r=Robot('stage1')
q=yaw_down_quat(0)
# white mug: center (-0.103,-0.156), grasp the rim wall on the -y side
tgt=np.array([-0.103,-0.196,0.65])
res=r.move_pose(tgt,q,seconds=4.0)
print('res',res)
p,qq,f=r.tcp_world(); print('TCP now', p.round(4), qq.round(4))
print('gap', round(r.finger_gap(),4))
" && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 28
timeout 120 python3 cloud.py robot0_eye_in_hand && python3 -c "
import numpy as np
d=np.load('robot0_eye_in_hand_cloud.npz'); P=d['P']
x,y,z=P[...,0],P[...,1],P[...,2]
m=np.isfinite(z)&(z>0.50)&(z<0.60)&(x>-0.2)&(x<0.0)&(y>-0.25)&(y<-0.05)
pts=np.c_[x[m],y[m]]
def fit(pts):
    A=np.c_[2*pts[:,0],2*pts[:,1],np.ones(len(pts))]; b=(pts**2).sum(1)
    c=np.linalg.lstsq(A,b,rcond=None)[0]; return c[0],c[1],np.sqrt(c[2]+c[0]**2+c[1]**2)
cx,cy,r=fit(pts)
for _ in range(3):
    dist=np.hypot(pts[:,0]-cx,pts[:,1]-cy); keep=np.abs(dist-r)<0.006
    cx,cy,r=fit(pts[keep])
print('rim pts',m.sum(),'fit center',round(cx,4),round(cy,4),'r',round(r,4),'zmax',z[m].max().round(4))
# rim only (top 1cm)
mt=m&(z>z[m].max()-0.01)
print('top ring x',x[mt].min().round(3),x[mt].max().round(3),'y',y[mt].min().round(3),y[mt].max().round(3),'n',mt.sum())
"

# openrua op 29
python3 -c "
import numpy as np
d=np.load('robot0_eye_in_hand_cloud.npz'); P=d['P']
x,y,z=P[...,0],P[...,1],P[...,2]
for dz in [0.003,0.006,0.01]:
    m=np.isfinite(z)&(z>0.5489-dz)&(x>-0.2)&(x<0.0)&(y>-0.25)&(y<-0.05)
    pts=np.c_[x[m],y[m]]
    A=np.c_[2*pts[:,0],2*pts[:,1],np.ones(len(pts))]; b=(pts**2).sum(1)
    c=np.linalg.lstsq(A,b,rcond=None)[0]; r=np.sqrt(c[2]+c[0]**2+c[1]**2)
    print(f'dz={dz}: n={m.sum()} center=({c[0]:.4f},{c[1]:.4f}) r={r:.4f} x[{x[m].min():.3f},{x[m].max():.3f}] y[{y[m].min():.3f},{y[m].max():.3f}]')
# where is the wall on the -y side at x=center? sample points near x=-0.098 with y<-0.17, z>0.53
m=np.isfinite(z)&(z>0.53)&(np.abs(x+0.098)<0.005)&(y<-0.15)&(y>-0.25)
print('wall -y side y-range at x~center:', y[m].min().round(4), y[m].max().round(4), 'n', m.sum())
"

# openrua op 30
python3 -c "
import numpy as np
d=np.load('robot0_eye_in_hand_cloud.npz'); P=d['P']
x,y,z=P[...,0],P[...,1],P[...,2]
for dz in [0.003,0.006,0.01]:
    m=np.isfinite(z)&(z>0.5489-dz)&(z<0.60)&(x>-0.2)&(x<0.0)&(y>-0.25)&(y<-0.05)
    pts=np.c_[x[m],y[m]]
    A=np.c_[2*pts[:,0],2*pts[:,1],np.ones(len(pts))]; b=(pts**2).sum(1)
    c=np.linalg.lstsq(A,b,rcond=None)[0]; r=np.sqrt(c[2]+c[0]**2+c[1]**2)
    print(f'dz={dz}: n={m.sum()} center=({c[0]:.4f},{c[1]:.4f}) r={r:.4f} x[{x[m].min():.3f},{x[m].max():.3f}] y[{y[m].min():.3f},{y[m].max():.3f}]')
m=np.isfinite(z)&(z>0.53)&(z<0.6)&(np.abs(x+0.098)<0.005)&(y<-0.15)&(y>-0.25)
print('wall -y side y-range at x~center:', y[m].min().round(4), y[m].max().round(4), 'n', m.sum())
m=np.isfinite(z)&(z>0.53)&(z<0.6)&(np.abs(y+0.155)<0.005)&(x<0)&(x>-0.2)
print('wall x-range at y~center:', x[m].min().round(4), x[m].max().round(4), 'n', m.sum())
"

# openrua op 31
timeout 900 python3 -u -c "
from rob import *
r=Robot('stage2')
q=yaw_down_quat(0)
cur=r.arm_q()
q1=r.ik([-0.098,-0.199,0.60],q,at_tcp=True,seed=cur)
q2=r.ik([-0.098,-0.199,0.524],q,at_tcp=True,seed=q1)
print('q1',np.round(q1,3)); print('q2',np.round(q2,3))
print(r.move_q(q1,2.5))
print(r.move_q(q2,2.5))
p,qq,f=r.tcp_world(); print('TCP', p.round(4), qq.round(4))
" && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 32
timeout 600 python3 -u -c "
from rob import *
r=Robot('stage3')
gap=r.gripper(0.0)
print('gap after close', round(gap,4))
d=r.joints(); print('fingers', round(d['panda_finger_joint1'],4), round(d['panda_finger_joint2'],4))
"

# openrua op 33
timeout 900 python3 -u -c "
from rob import *
r=Robot('stage4')
q=yaw_down_quat(0)
cur=r.arm_q()
q1=r.ik([-0.098,-0.199,0.70],q,at_tcp=True,seed=cur)
print(r.move_q(q1,2.5))
p,qq,f=r.tcp_world(); print('TCP', p.round(4)); print('gap', round(r.finger_gap(),4))
" && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 34
timeout 120 python3 cloud.py birdview && python3 -c "
import numpy as np
d=np.load('birdview_cloud.npz'); P=d['P']
x,y,z=P[...,0],P[...,1],P[...,2]
# mug hanging: rim at ~0.72, body 0.60-0.72; hand above 0.75
m=np.isfinite(z)&(z>0.58)&(z<0.735)&(x>-0.25)&(x<0.05)&(y>-0.35)&(y<-0.05)
print('n',m.sum(),'x',x[m].min().round(3),x[m].max().round(3),'y',y[m].min().round(3),y[m].max().round(3),'z',z[m].min().round(3),z[m].max().round(3))
pts=np.c_[x[m],y[m]]
A=np.c_[2*pts[:,0],2*pts[:,1],np.ones(len(pts))]; b=(pts**2).sum(1)
c=np.linalg.lstsq(A,b,rcond=None)[0]; r=np.sqrt(c[2]+c[0]**2+c[1]**2)
print('fit center',c[:2].round(4),'r',round(r,4))
mt=m&(z>z[m].max()-0.01); print('top ring x',x[mt].min().round(3),x[mt].max().round(3),'y',y[mt].min().round(3),y[mt].max().round(3),'center',((x[mt].min()+x[mt].max())/2).round(4),((y[mt].min()+y[mt].max())/2).round(4))
"

# openrua op 35
timeout 900 python3 -u -c "
from rob import *
r=Robot('stage5')
q=yaw_down_quat(0)
cur=r.arm_q()
q1=r.ik([-0.024,-0.325,0.70],q,at_tcp=True,seed=cur)
print(r.move_q(q1,3.0))
p,qq,f=r.tcp_world(); print('TCP', p.round(4)); print('gap', round(r.finger_gap(),4))
" && timeout 60 python3 tools/perception/cam_snap.py agentview && timeout 120 python3 cloud.py frontview

# openrua op 36
python3 -c "
import numpy as np
d=np.load('frontview_cloud.npz'); P=d['P']
x,y,z=P[...,0],P[...,1],P[...,2]
m=np.isfinite(z)&(z>0.60)&(z<0.73)&(x>-0.3)&(x<0.2)&(y>-0.5)&(y<-0.15)
print('n',m.sum(),'x',x[m].min().round(3),x[m].max().round(3),'y',y[m].min().round(3),y[m].max().round(3),'z',z[m].min().round(3),z[m].max().round(3))
for zl in [0.60,0.63,0.66,0.69]:
    mm=m&(z>zl)&(z<zl+0.03)
    if mm.sum(): print(f' z{zl}: y[{y[mm].min():.3f},{y[mm].max():.3f}] x[{x[mm].min():.3f},{x[mm].max():.3f}] n={mm.sum()}')
# plate
m=np.isfinite(z)&(z>0.435)&(z<0.47)&(x>-0.2)&(x<0.2)&(y>-0.5)&(y<-0.15)
print('plate x',x[m].min().round(3),x[m].max().round(3),'y',y[m].min().round(3),y[m].max().round(3),'n',m.sum())
"

# openrua op 37
timeout 900 python3 -u -c "
from rob import *
r=Robot('stage6')
q=yaw_down_quat(0)
cur=r.arm_q()
q1=r.ik([-0.024,-0.325,0.54],q,at_tcp=True,seed=cur)
print(r.move_q(q1,2.5))
p,qq,f=r.tcp_world(); print('TCP', p.round(4))
gap=r.gripper(0.04)
q2=r.ik([-0.024,-0.325,0.68],q,at_tcp=True,seed=r.arm_q())
print(r.move_q(q2,2.5))
p,qq,f=r.tcp_world(); print('TCP', p.round(4))
" && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 38
timeout 900 python3 -u -c "
from rob import *
r=Robot('stage7')
q=yaw_down_quat(0)
# pre-grasp above yellow mug, rim wall on the +y side (handle is on -y)
q1=r.ik([-0.227,0.078,0.66],q,at_tcp=True,seed=r.arm_q())
print('q1',None if q1 is None else np.round(q1,3))
print(r.move_q(q1,4.0))
p,qq,f=r.tcp_world(); print('TCP', p.round(4), qq.round(4))
" && timeout 120 python3 cloud.py robot0_eye_in_hand && python3 -c "
import numpy as np
d=np.load('robot0_eye_in_hand_cloud.npz'); P=d['P']
x,y,z=P[...,0],P[...,1],P[...,2]
box=(x>-0.32)&(x<-0.13)&(y>-0.08)&(y<0.15)
m=np.isfinite(z)&box&(z>0.50)&(z<0.60)
print('n',m.sum(),'zmax',z[m].max().round(4))
zt=z[m].max()
for dz in [0.003,0.006]:
    mm=m&(z>zt-dz)
    pts=np.c_[x[mm],y[mm]]
    A=np.c_[2*pts[:,0],2*pts[:,1],np.ones(len(pts))]; b=(pts**2).sum(1)
    c=np.linalg.lstsq(A,b,rcond=None)[0]; rr=np.sqrt(c[2]+c[0]**2+c[1]**2)
    print(f'dz={dz}: n={mm.sum()} center=({c[0]:.4f},{c[1]:.4f}) r={rr:.4f} x[{x[mm].min():.3f},{x[mm].max():.3f}] y[{y[mm].min():.3f},{y[mm].max():.3f}]')
cx=c[0]
mm=m&(z>0.52)&(np.abs(x-cx)<0.005)&(y>0.0)
print('+y wall y-range at x~center:', y[mm].min().round(4), y[mm].max().round(4), 'n', mm.sum())
"

# openrua op 39
timeout 60 python3 tools/perception/cam_snap.py agentview && timeout 30 ros2 topic echo /joint_states --once | grep -A9 '^position'

# openrua op 40
timeout 900 python3 -u -c "
from rob import *
r=Robot('stage7b')
q1=[-0.263,-0.831,0.604,-2.937,0.521,2.137,0.659]
print(r.move_q(q1,5.0))
print('q now',np.round(r.arm_q(),3))
p,qq,f=r.tcp_world(); print('TCP', p.round(4), qq.round(4))
"

# openrua op 41
timeout 120 python3 cloud.py robot0_eye_in_hand && python3 -c "
import numpy as np
d=np.load('robot0_eye_in_hand_cloud.npz'); P=d['P']
x,y,z=P[...,0],P[...,1],P[...,2]
box=(x>-0.32)&(x<-0.13)&(y>-0.08)&(y<0.15)
m=np.isfinite(z)&box&(z>0.50)&(z<0.60)
zt=z[m].max(); print('n',m.sum(),'zmax',zt.round(4))
for dz in [0.003,0.006]:
    mm=m&(z>zt-dz)
    pts=np.c_[x[mm],y[mm]]
    A=np.c_[2*pts[:,0],2*pts[:,1],np.ones(len(pts))]; b=(pts**2).sum(1)
    c=np.linalg.lstsq(A,b,rcond=None)[0]; rr=np.sqrt(c[2]+c[0]**2+c[1]**2)
    print(f'dz={dz}: n={mm.sum()} center=({c[0]:.4f},{c[1]:.4f}) r={rr:.4f} x[{x[mm].min():.3f},{x[mm].max():.3f}] y[{y[mm].min():.3f},{y[mm].max():.3f}]')
mm=m&(z>zt-0.004)&(np.abs(x-c[0])<0.004)&(y>0.05)
print('+y rim top y-range at x~center:', y[mm].min().round(4), y[mm].max().round(4), 'n', mm.sum())
mm=m&(z>zt-0.004)&(np.abs(y-c[1])<0.004)
print('rim top x-range at y~center:', x[mm].min().round(4), x[mm].max().round(4), 'n', mm.sum())
"

# openrua op 42
timeout 900 python3 -u -c "
from rob import *
r=Robot('stage8')
q=yaw_down_quat(0)
cur=r.arm_q()
q1=r.ik([-0.2247,0.091,0.60],q,at_tcp=True,seed=cur)
q2=r.ik([-0.2247,0.091,0.517],q,at_tcp=True,seed=q1)
print('q1',np.round(q1,3)); print('q2',np.round(q2,3))
for qq_,t in [(q1,3.0),(q2,3.0)]:
    code,err=r.move_q(qq_,t)
    if err>0.02:
        print('retrying'); code,err=r.move_q(qq_,4.0)
p,qq,f=r.tcp_world(); print('TCP', p.round(4), qq.round(4))
" && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 43
timeout 900 python3 -u -c "
from rob import *
r=Robot('stage9')
gap=r.gripper(0.0)
if gap<0.002:
    raise SystemExit('closed on air')
q=yaw_down_quat(0)
q1=r.ik([-0.2247,0.091,0.70],q,at_tcp=True,seed=r.arm_q())
code,err=r.move_q(q1,3.0)
if err>0.02: code,err=r.move_q(q1,4.0)
p,qq,f=r.tcp_world(); print('TCP', p.round(4)); print('gap', round(r.finger_gap(),4))
" && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 44
timeout 900 python3 -u -c "
from rob import *
r=Robot('stage10')
q=yaw_down_quat(0)
q1=r.ik([0.002,0.364,0.70],q,at_tcp=True,seed=r.arm_q())
print('q1',np.round(q1,3))
code,err=r.move_q(q1,4.0)
if err>0.02: code,err=r.move_q(q1,4.0)
p,qq,f=r.tcp_world(); print('TCP', p.round(4)); print('gap', round(r.finger_gap(),4))
" && timeout 120 python3 cloud.py frontview && python3 -c "
import numpy as np
d=np.load('frontview_cloud.npz'); P=d['P']
x,y,z=P[...,0],P[...,1],P[...,2]
m=np.isfinite(z)&(z>0.58)&(z<0.73)&(x>-0.3)&(x<0.2)&(y>0.2)&(y<0.5)
print('n',m.sum(),'x',x[m].min().round(3),x[m].max().round(3),'y',y[m].min().round(3),y[m].max().round(3),'z',z[m].min().round(3),z[m].max().round(3))
for zl in [0.58,0.61,0.64,0.67,0.70]:
    mm=m&(z>zl)&(z<zl+0.03)
    if mm.sum(): print(f' z{zl}: y[{y[mm].min():.3f},{y[mm].max():.3f}] x[{x[mm].min():.3f},{x[mm].max():.3f}] n={mm.sum()}')
m=np.isfinite(z)&(z>0.435)&(z<0.47)&(x>-0.2)&(x<0.2)&(y>0.2)&(y<0.5)
print('plate x',x[m].min().round(3),x[m].max().round(3),'y',y[m].min().round(3),y[m].max().round(3),'n',m.sum())
"

# openrua op 45
timeout 900 python3 -u -c "
from rob import *
r=Robot('stage11')
q=yaw_down_quat(0)
q1=r.ik([0.002,0.364,0.538],q,at_tcp=True,seed=r.arm_q())
code,err=r.move_q(q1,3.0)
if err>0.02: code,err=r.move_q(q1,4.0)
p,qq,f=r.tcp_world(); print('TCP', p.round(4))
gap=r.gripper(0.04)
q2=r.ik([0.002,0.364,0.70],q,at_tcp=True,seed=r.arm_q())
code,err=r.move_q(q2,3.0)
if err>0.02: code,err=r.move_q(q2,4.0)
p,qq,f=r.tcp_world(); print('TCP', p.round(4))
" && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 46
timeout 120 python3 cloud.py birdview && python3 -c "
import numpy as np
d=np.load('birdview_cloud.npz'); P=d['P']
x,y,z=P[...,0],P[...,1],P[...,2]
def fit(pts):
    A=np.c_[2*pts[:,0],2*pts[:,1],np.ones(len(pts))]; b=(pts**2).sum(1)
    c=np.linalg.lstsq(A,b,rcond=None)[0]; return c[0],c[1],np.sqrt(c[2]+c[0]**2+c[1]**2)
for name,(x0,x1,y0,y1) in {'LEFT plate region (y<0)':(-0.15,0.12,-0.42,-0.15),'RIGHT plate region (y>0)':(-0.15,0.12,0.18,0.45)}.items():
    box=(x>x0)&(x<x1)&(y>y0)&(y<y1)&np.isfinite(z)
    plate=box&(z>0.437)&(z<0.462)
    mug=box&(z>0.50)&(z<0.60)
    pts=np.c_[x[mug],y[mug]]; cx,cy,r=fit(pts)
    dist=np.hypot(pts[:,0]-cx,pts[:,1]-cy); keep=np.abs(dist-r)<0.008; cx,cy,r=fit(pts[keep])
    px=(x[plate].min()+x[plate].max())/2; py=(y[plate].min()+y[plate].max())/2
    print(f'{name}: plate center=({px:.3f},{py:.3f}) plate y-extent [{y[plate].min():.3f},{y[plate].max():.3f}] | mug rim center=({cx:.3f},{cy:.3f}) r={r:.3f} rim z={z[mug].max():.3f} | offset={np.hypot(cx-px,cy-py)*100:.1f} cm')
# red mug untouched?
box=(x>-0.15)&(x<0.0)&(y>0.05)&(y<0.2)&np.isfinite(z)&(z>0.5)&(z<0.6)
print('red mug top z',z[box].max().round(3),'center approx',x[box].mean().round(3),y[box].mean().round(3))
"

# openrua op 47
python3 -c "
import numpy as np
d=np.load('birdview_cloud.npz'); P=d['P']
x,y,z=P[...,0],P[...,1],P[...,2]
box=(x>-0.15)&(x<0.12)&(y>0.18)&(y<0.45)&np.isfinite(z)
mug=box&(z>0.52)&(z<0.60)
print('yellow mug top-ring: x[%.3f,%.3f] y[%.3f,%.3f] n=%d'%(x[mug].min(),x[mug].max(),y[mug].min(),y[mug].max(),mug.sum()))
print('center est from x-extent: %.3f ; from +y edge minus r(0.048): %.3f'%((x[mug].min()+x[mug].max())/2, y[mug].max()-0.048))
plate=box&(z>0.437)&(z<0.462)
print('plate x[%.3f,%.3f] y[%.3f,%.3f]'%(x[plate].min(),x[plate].max(),y[plate].min(),y[plate].max()))
# earlier full plate measurement: center (0.002,0.316), radius ~0.066
"

# openrua op 48
timeout 900 python3 -u -c "
from rob import *
r=Robot('home')
home=[0.0,-0.161,0.0,-2.4446,0.0,2.2268,0.7854]
code,err=r.move_q(home,4.0)
if err>0.02: code,err=r.move_q(home,4.0)
" && timeout 120 python3 cloud.py birdview && timeout 60 python3 tools/perception/cam_snap.py agentview && python3 -c "
import numpy as np
d=np.load('birdview_cloud.npz'); P=d['P']
x,y,z=P[...,0],P[...,1],P[...,2]
def fit(pts):
    A=np.c_[2*pts[:,0],2*pts[:,1],np.ones(len(pts))]; b=(pts**2).sum(1)
    c=np.linalg.lstsq(A,b,rcond=None)[0]; return c[0],c[1],np.sqrt(c[2]+c[0]**2+c[1]**2)
for name,(x0,x1,y0,y1),pc in [('LEFT plate / white mug',(-0.15,0.12,-0.42,-0.15),(-0.024,-0.281)),('RIGHT plate / yellow mug',(-0.15,0.12,0.18,0.45),(0.002,0.316))]:
    box=(x>x0)&(x<x1)&(y>y0)&(y<y1)&np.isfinite(z)
    mug=box&(z>0.52)&(z<0.60)
    pts=np.c_[x[mug],y[mug]]; cx,cy,r=fit(pts)
    dist=np.hypot(pts[:,0]-cx,pts[:,1]-cy); keep=np.abs(dist-r)<0.008; cx,cy,r=fit(pts[keep])
    print(f'{name}: n={mug.sum()} mug rim center=({cx:.3f},{cy:.3f}) r={r:.3f} rim z={z[mug].max():.3f} | plate center (pre-task) {pc} | offset={np.hypot(cx-pc[0],cy-pc[1])*100:.1f} cm (plate radius ~6.6 cm, mug base radius ~3.5 cm)')
"

# openrua op 49
mkdir -p "$(dirname /claude-config/projects/-workspace/memory/panda-ik-frame-and-tip-link.md)"
cat > /claude-config/projects/-workspace/memory/panda-ik-frame-and-tip-link.md <<'OPENRUA_EOF'
---
name: panda-ik-frame-and-tip-link
description: On this Panda workstation /compute_ik takes WORLD coordinates (not panda_link0 as docs say) and solves for panda_link8 (hand rotated 45° about z); TF world tree IS connected
metadata:
  type: project
---

Measured 2026-09-12 on the Franka Panda sim workstation in /workspace:
- `/compute_ik` (frame_id left empty) interprets positions in the **world** frame
  (world->panda_link0 = (-0.51, 0, 0.42)); a base-frame position returns -31.
- IK solves for **panda_link8**, not panda_hand: request quat = q_hand ⊗ Rz(+45°)
  or the hand ends up yawed 45°. `/compute_fk` returns world-frame poses.
- TF world->panda_hand and world->camera optical frames are connected (docs warn
  they may not be). Table top z≈0.426 world; plates ≈0.44-0.46; mugs ≈12 cm tall.
- Gripper opens to only ~7.7 cm gap; LIBERO mugs are 9-10 cm across, so grasp the
  **rim** (one finger inside, TCP ~2.5 cm below the rim). Closed-on-wall gap ≈8-13 mm.
- FollowJointTrajectory often returns error -5 on long moves; resending the same
  goal converges (docs are right about this).
- Scene is unsettled at start (objects float ~6 cm); one gripper command settles it.

**Why:** the generic docs/30-action.md "Planning facts" claim IK is in the arm-base
frame; following that wastes a session on failed IK.
**How to apply:** use /workspace/rob.py (Robot.ik handles both corrections) if it
still exists; otherwise apply the world-frame + link8 quaternion correction.
OPENRUA_EOF

# openrua op 50
cd /claude-config/projects/-workspace/memory && (test -f MEMORY.md && cat MEMORY.md || echo "(no index yet)")

# openrua op 51
mkdir -p "$(dirname /claude-config/projects/-workspace/memory/MEMORY.md)"
cat > /claude-config/projects/-workspace/memory/MEMORY.md <<'OPENRUA_EOF'
- [Panda IK frame and tip link](panda-ik-frame-and-tip-link.md) — /compute_ik uses WORLD coords and panda_link8 (45° yaw); rim-grasp wide mugs; retry -5 trajectories
OPENRUA_EOF
