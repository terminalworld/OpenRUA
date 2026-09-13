#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace; echo ---; find /workspace -maxdepth 2 -type f | head -50

# openrua op 1
cat /workspace/tools/README.md; echo ---; find /workspace/tools -type f; echo ---; ls -la /workspace/.ros

# openrua op 2
cat tools/perception/cam_snap.py tools/perception/px2world.py

# openrua op 3
cat tools/action/fjt_send.py tools/action/gripper_cmd.py tools/action/ik_move.py

# openrua op 4
ros2 topic list; echo ---; ros2 action list; echo ---; ros2 service list | grep -iE "ik|fk|plan|scene" ; echo ---; ros2 node list

# openrua op 5
for c in agentview birdview frontview sideview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c; done; ros2 topic echo /joint_states --once

# openrua op 6
timeout 30 ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id|x:|y:|z:|w:" | head -80

# openrua op 7
mkdir -p "$(dirname /workspace/tfdump.py)"
cat > /workspace/tfdump.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Dump all TF frames and their transform relative to world (if connected)."""
import rclpy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
import yaml

rclpy.init()
node = rclpy.create_node("tfdump")
buf = Buffer()
TransformListener(buf, node)
for _ in range(30):
    rclpy.spin_once(node, timeout_sec=0.2)
frames = yaml.safe_load(buf.all_frames_as_yaml()) or {}
for f, info in sorted(frames.items()):
    parent = info["parent"]
    line = f"{f:32s} parent={parent:20s}"
    for ref in ("world", "panda_link0"):
        try:
            t = buf.lookup_transform(ref, f, Time())
            tr, q = t.transform.translation, t.transform.rotation
            line += f" | {ref}: t=({tr.x:.4f},{tr.y:.4f},{tr.z:.4f}) q=({q.x:.4f},{q.y:.4f},{q.z:.4f},{q.w:.4f})"
        except Exception as e:
            line += f" | {ref}: n/a"
    print(line)
rclpy.shutdown()
OPENRUA_EOF

# openrua op 8
timeout 90 python3 tfdump.py

# openrua op 9
mkdir -p "$(dirname /workspace/cloud.py)"
cat > /workspace/cloud.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Grab color+depth+intrinsics+TF for a camera; save world-frame point cloud.

Usage: python3 cloud.py <camera>
Writes <camera>_cloud.npz with keys: xyz (H,W,3 world metres), bgr (H,W,3).
"""
import struct
import sys

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener


def grab(node, topic, msg_type, timeout=60.0):
    got = {}
    sub = node.create_subscription(msg_type, topic,
                                   lambda m: got.setdefault("m", m), 1)
    import time
    end = time.time() + timeout
    while "m" not in got and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    if "m" not in got:
        raise SystemExit(f"no message on {topic}")
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
    buf = Buffer()
    TransformListener(buf, node)
    bridge = CvBridge()
    color = bridge.imgmsg_to_cv2(grab(node, f"/{cam}/color/image_raw", Image), "bgr8")
    depth = bridge.imgmsg_to_cv2(grab(node, f"/{cam}/depth/image_raw", Image), "passthrough").astype(np.float64)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    frame = f"{cam}_optical_frame"
    import time
    end = time.time() + 20
    while time.time() < end and not buf.can_transform("world", frame, Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    t = buf.lookup_transform("world", frame, Time())
    q = t.transform.rotation
    R = quat_R(q.x, q.y, q.z, q.w)
    tr = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    H, W = depth.shape
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    X = (u - cx) * depth / fx
    Y = (v - cy) * depth / fy
    pc = np.stack([X, Y, depth], -1)
    xyz = pc @ R.T + tr
    xyz[~np.isfinite(depth) | (depth <= 0)] = np.nan
    np.savez(f"{cam}_cloud.npz", xyz=xyz, bgr=color)
    print(f"{cam}_cloud.npz  shape={xyz.shape}  cam_pos={tr}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 10
timeout 120 python3 cloud.py birdview && timeout 120 python3 cloud.py agentview

# openrua op 11
python3 -c "
import numpy as np
d=np.load('birdview_cloud.npz'); xyz=d['xyz']; bgr=d['bgr']
z=xyz[...,2]
print('z percentiles', np.nanpercentile(z,[1,5,25,50,75,95,99]))
# table height: mode of z in central region
zc=z[150:350,150:500]
h,e=np.histogram(zc[np.isfinite(zc)],bins=200)
print('table z mode', e[h.argmax()], e[h.argmax()+1])
"


# openrua op 12
python3 -c "
import numpy as np, cv2
d=np.load('birdview_cloud.npz'); xyz=d['xyz']; bgr=d['bgr']
z=xyz[...,2]
mask=(z>0.435)&(z<0.75)&np.isfinite(z)
mask=mask.astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,4]<15: continue
    m=lab==i
    p=xyz[m]; c=bgr[m].mean(0)
    print(f'comp{i} px={stats[i,4]:5d} pix_centroid=({cent[i][0]:.0f},{cent[i][1]:.0f}) x=[{np.nanmin(p[:,0]):.3f},{np.nanmax(p[:,0]):.3f}] y=[{np.nanmin(p[:,1]):.3f},{np.nanmax(p[:,1]):.3f}] ztop={np.nanmax(p[:,2]):.3f} mean=({np.nanmean(p[:,0]):.3f},{np.nanmean(p[:,1]):.3f}) bgr={c.astype(int)}')
"


# openrua op 13
python3 -c "
import numpy as np, cv2
d=np.load('agentview_cloud.npz'); xyz=d['xyz']; bgr=d['bgr']
z=xyz[...,2]; x=xyz[...,0]
mask=(z>0.432)&(z<0.75)&np.isfinite(z)&(x>-0.35)&(x<0.5)
mask=mask.astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,4]<30: continue
    m=lab==i
    p=xyz[m]; c=bgr[m].mean(0)
    print(f'comp{i} px={stats[i,4]:5d} pix_bbox={stats[i,:4]} x=[{np.nanmin(p[:,0]):.3f},{np.nanmax(p[:,0]):.3f}] y=[{np.nanmin(p[:,1]):.3f},{np.nanmax(p[:,1]):.3f}] z=[{np.nanmin(p[:,2]):.3f},{np.nanmax(p[:,2]):.3f}] bgr={c.astype(int)}')
"


# openrua op 14
python3 -c "
import numpy as np
for cam in ['agentview','birdview']:
    d=np.load(cam+'_cloud.npz'); xyz=d['xyz']
    print(cam)
    if cam=='agentview':
        for (u,v) in [(200,400),(450,400),(320,300),(320,450),(100,300),(320,380)]:
            print('  px',(u,v),'->',np.round(xyz[v,u],3))
    else:
        for (u,v) in [(320,319),(320,290),(320,350),(280,319),(200,319),(320,400),(279,260),(323,238)]:
            print('  px',(u,v),'->',np.round(xyz[v,u],3))
"


# openrua op 15
timeout 300 python3 tools/action/fjt_send.py 0.0,-0.161037389,0.0,-2.44459747,0.0,2.2267522,0.7853981633974483 2.0 && timeout 120 python3 cloud.py birdview && python3 -c "
import numpy as np
d=np.load('birdview_cloud.npz'); xyz=d['xyz']
for (u,v) in [(320,319),(279,260),(323,238),(280,319)]:
    print('  px',(u,v),'->',np.round(xyz[v,u],3))
"

# openrua op 16
timeout 120 python3 cloud.py agentview >/dev/null && python3 -c "
import numpy as np, cv2
for cam in ['birdview','agentview']:
    d=np.load(cam+'_cloud.npz'); xyz=d['xyz']; bgr=d['bgr']
    z=xyz[...,2]; x=xyz[...,0]
    mask=((z>0.435)&(z<0.70)&np.isfinite(z)&(x>-0.35)&(x<0.5)).astype(np.uint8)
    n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
    print(cam)
    for i in range(1,n):
        if stats[i,4]<30: continue
        m=lab==i; p=xyz[m]; c=bgr[m].mean(0)
        print(f'  comp{i} px={stats[i,4]:5d} bbox={stats[i,:4]} x=[{np.nanmin(p[:,0]):.3f},{np.nanmax(p[:,0]):.3f}] y=[{np.nanmin(p[:,1]):.3f},{np.nanmax(p[:,1]):.3f}] z=[{np.nanmin(p[:,2]):.3f},{np.nanmax(p[:,2]):.3f}] mean=({np.nanmean(p[:,0]):.3f},{np.nanmean(p[:,1]):.3f}) bgr={c.astype(int)}')
"


# openrua op 17
python3 -c "
import numpy as np, cv2
d=np.load('agentview_cloud.npz'); xyz=d['xyz']; bgr=d['bgr']
z=xyz[...,2]; x=xyz[...,0]; y=xyz[...,1]
# pudding: behind red mug
m=(z>0.435)&(z<0.60)&(x<-0.14)&(x>-0.30)&(y>-0.05)&(y<0.12)
p=xyz[m]; print('pudding cand n=',m.sum(),'x=[%.3f,%.3f] y=[%.3f,%.3f] z=[%.3f,%.3f] mean=(%.3f,%.3f)'%(p[:,0].min(),p[:,0].max(),p[:,1].min(),p[:,1].max(),p[:,2].min(),p[:,2].max(),p[:,0].mean(),p[:,1].mean()), 'bgr', bgr[m].mean(0).astype(int))
vs,us=np.where(m); print('pix bbox u',us.min(),us.max(),'v',vs.min(),vs.max())
# red mug
m2=(z>0.435)&(z<0.60)&(x>-0.14)&(x<0.0)&(y>0.0)&(y<0.2)
p=xyz[m2]; print('red mug n=',m2.sum(),'x=[%.3f,%.3f] y=[%.3f,%.3f] z=[%.3f,%.3f] mean=(%.3f,%.3f)'%(p[:,0].min(),p[:,0].max(),p[:,1].min(),p[:,1].max(),p[:,2].min(),p[:,2].max(),p[:,0].mean(),p[:,1].mean()))
img=bgr.copy(); img[m]=(0,255,0); cv2.imwrite('agentview_pudding_mask.png',img[150:350,250:450])
cv2.imwrite('agentview_crop.png',cv2.resize(bgr[150:350,250:450],None,fx=3,fy=3,interpolation=cv2.INTER_NEAREST))
"


# openrua op 18
timeout 300 python3 tools/action/fjt_send.py 1.3,-0.161037389,0.0,-2.44459747,0.0,2.2267522,0.7853981633974483 3.0 && timeout 120 python3 cloud.py birdview && python3 -c "
import numpy as np, cv2
d=np.load('birdview_cloud.npz'); xyz=d['xyz']; bgr=d['bgr']
z=xyz[...,2]; x=xyz[...,0]
mask=((z>0.435)&(z<0.70)&np.isfinite(z)&(x>-0.35)&(x<0.5)).astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,4]<30: continue
    m=lab==i; p=xyz[m]; c=bgr[m].mean(0)
    print(f'  comp{i} px={stats[i,4]:5d} bbox={stats[i,:4]} x=[{np.nanmin(p[:,0]):.3f},{np.nanmax(p[:,0]):.3f}] y=[{np.nanmin(p[:,1]):.3f},{np.nanmax(p[:,1]):.3f}] z=[{np.nanmin(p[:,2]):.3f},{np.nanmax(p[:,2]):.3f}] mean=({np.nanmean(p[:,0]):.3f},{np.nanmean(p[:,1]):.3f}) bgr={c.astype(int)}')
cv2.imwrite('bird_crop.png',cv2.resize(bgr[200:340,240:400],None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST))
"

# openrua op 19
timeout 30 ros2 topic echo /joint_states --once | grep -A9 position:

# openrua op 20
timeout 300 python3 tools/action/fjt_send.py 1.3,-0.161037389,0.0,-2.44459747,0.0,2.2267522,0.7853981633974483 3.0; timeout 30 ros2 topic echo /joint_states --once | grep -A7 position:

# openrua op 21
timeout 300 python3 tools/action/fjt_send.py 1.3,-0.161037389,0.0,-2.44459747,0.0,2.2267522,0.7853981633974483 2.0; timeout 30 ros2 topic echo /joint_states --once | grep -A7 position:

# openrua op 22
timeout 120 python3 cloud.py birdview && python3 -c "
import numpy as np, cv2
d=np.load('birdview_cloud.npz'); xyz=d['xyz']; bgr=d['bgr']
z=xyz[...,2]; x=xyz[...,0]
mask=((z>0.435)&(z<0.70)&np.isfinite(z)&(x>-0.35)&(x<0.5)).astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,4]<30: continue
    m=lab==i; p=xyz[m]; c=bgr[m].mean(0)
    print(f'  comp{i} px={stats[i,4]:5d} bbox={stats[i,:4]} x=[{np.nanmin(p[:,0]):.3f},{np.nanmax(p[:,0]):.3f}] y=[{np.nanmin(p[:,1]):.3f},{np.nanmax(p[:,1]):.3f}] z=[{np.nanmin(p[:,2]):.3f},{np.nanmax(p[:,2]):.3f}] mean=({np.nanmean(p[:,0]):.3f},{np.nanmean(p[:,1]):.3f}) bgr={c.astype(int)}')
cv2.imwrite('bird_crop.png',cv2.resize(bgr[200:340,240:400],None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST))
"

# openrua op 23
python3 -c "
import numpy as np
d=np.load('birdview_cloud.npz'); xyz=d['xyz']
z=xyz[...,2]; x=xyz[...,0]; y=xyz[...,1]
def fit_circle(px,py):
    A=np.c_[2*px,2*py,np.ones_like(px)]; b=px**2+py**2
    c=np.linalg.lstsq(A,b,rcond=None)[0]; r=np.sqrt(c[2]+c[0]**2+c[1]**2); return c[0],c[1],r
# white mug rim: high points near mug
m=(z>0.52)&(x>-0.2)&(x<-0.03)&(y>-0.26)&(y<-0.09)
cx,cy,r=fit_circle(x[m],y[m]); print('white mug rim circle: center=(%.4f,%.4f) r=%.4f n=%d'%(cx,cy,r,m.sum()))
print('  rim z range',z[m].min(),z[m].max())
# red mug rim
m=(z>0.55)&(x>-0.15)&(x<0.02)&(y>0.0)&(y<0.16)
cx,cy,r=fit_circle(x[m],y[m]); print('red mug rim circle: center=(%.4f,%.4f) r=%.4f n=%d'%(cx,cy,r,m.sum()))
# plate: outer edge points
m=(z>0.44)&(x>0.05)&(x<0.25)&(y>-0.1)&(y<0.1)
print('plate pts mean=(%.4f,%.4f) xr=[%.3f,%.3f] yr=[%.3f,%.3f]'%(x[m].mean(),y[m].mean(),x[m].min(),x[m].max(),y[m].min(),y[m].max()))
# pudding
m=(z>0.455)&(z<0.48)&(x>-0.26)&(x<-0.12)&(y>-0.03)&(y<0.06)
print('pudding top pts mean=(%.4f,%.4f) xr=[%.3f,%.3f] yr=[%.3f,%.3f] z=[%.3f,%.3f] n=%d'%(x[m].mean(),y[m].mean(),x[m].min(),x[m].max(),y[m].min(),y[m].max(),z[m].min(),z[m].max(),m.sum()))
"


# openrua op 24
python3 -c "
import numpy as np
d=np.load('birdview_cloud.npz'); xyz=d['xyz']
z=xyz[...,2]; x=xyz[...,0]; y=xyz[...,1]
def fit_circle(px,py):
    A=np.c_[2*px,2*py,np.ones_like(px)]; b=px**2+py**2
    c=np.linalg.lstsq(A,b,rcond=None)[0]; r=np.sqrt(c[2]+c[0]**2+c[1]**2); return c[0],c[1],r
for zt in [0.50,0.52,0.53,0.54]:
    m=(z>zt)&(x>-0.2)&(x<-0.03)&(y>-0.26)&(y<-0.135)
    cx,cy,r=fit_circle(x[m],y[m]); print('zt=%.2f white mug rim: center=(%.4f,%.4f) r=%.4f n=%d'%(zt,cx,cy,r,m.sum()))
m=(z>0.5)&(x>-0.2)&(x<-0.03)&(y>-0.26)&(y<-0.09)
print('handle pts y>-0.135:', ((y[m]>-0.135)).sum(), 'zmax handle', z[m&(y>-0.13)].max() if (m&(y>-0.13)).any() else None)
"


# openrua op 25
mkdir -p "$(dirname /workspace/robot.py)"
cat > /workspace/robot.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Small motion library for this Panda: IK, trajectories, gripper, FK.

World frame <-> panda_link0: base sits at world (-0.51, 0, 0.42), identity
rotation (from TF). MoveIt plans in panda_link0, so world poses are
shifted by BASE before IK.
"""
import math
import sys
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
JOINTS = FJT["joints"]
LIMITS = FJT["limits_rad"]
BASE = np.array([-0.51, 0.0, 0.42])
TCP = M["hand"]["tcp_offset_m"]
RATE = 0.15  # rad/s the controller seems to sustain; used to size durations

# top-down grasp orientations (hand z down). yaw = rotation about world z
# of the finger axis: yaw 0 -> fingers along world y, yaw 90deg -> along x.
def down_quat(yaw_deg=0.0):
    # q = Rz(yaw) * Rx(180)
    h = math.radians(yaw_deg) / 2
    qz = np.array([0, 0, math.sin(h), math.cos(h)])
    qx = np.array([1, 0, 0, 0])
    return quat_mul(qz, qx)


def quat_mul(a, b):
    x1, y1, z1, w1 = a
    x2, y2, z2, w2 = b
    return np.array([
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ])


def quat_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class Robot:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("robot_lib")
        self._js = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik_cli = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        assert self.fjt.wait_for_server(timeout_sec=20), "no FJT server"
        assert self.grip.wait_for_server(timeout_sec=20), "no gripper server"
        assert self.ik_cli.wait_for_service(timeout_sec=20), "no IK"
        assert self.fk_cli.wait_for_service(timeout_sec=20), "no FK"

    def _on_js(self, msg):
        self._js = msg

    # ---------- sensing ----------
    def joints(self, fresh=True):
        """Arm joint positions in manifest order (dict also has fingers)."""
        if fresh:
            self._js = None
        t0 = time.time()
        while self._js is None and time.time() - t0 < 30:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        d = dict(zip(self._js.name, self._js.position))
        return np.array([d[j] for j in JOINTS]), d

    def finger_gap(self):
        _, d = self.joints()
        return d["panda_finger_joint1"] - d["panda_finger_joint2"]

    def fk(self, q=None, link="panda_hand"):
        """World pose (xyz, quat) of a link for arm joints q (default current)."""
        if q is None:
            q, _ = self.joints()
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = list(JOINTS)
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        assert res is not None and res.error_code.val == 1, f"FK failed {res}"
        p = res.pose_stamped[0].pose
        xyz = np.array([p.position.x, p.position.y, p.position.z]) + BASE
        q = np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        return xyz, q

    def tcp(self, q=None):
        """World position of the fingertip centre (TCP)."""
        xyz, quat = self.fk(q)
        return xyz + TCP * quat_R(quat)[:, 2], quat

    # ---------- planning ----------
    def ik(self, xyz_world, quat, seed=None, at_tcp=True, tries=5):
        """IK for a world pose of the HAND (or TCP if at_tcp). Returns q or None."""
        xyz = np.array(xyz_world, float)
        if at_tcp:
            xyz = xyz - TCP * quat_R(quat)[:, 2]
        p_base = xyz - BASE
        if seed is None:
            seed, _ = self.joints()
        for k in range(tries):
            req = GetPositionIK.Request()
            req.ik_request.group_name = M["planning"]["group"]
            req.ik_request.ik_link_name = "panda_hand"
            req.ik_request.pose_stamped.header.frame_id = ""
            pp = req.ik_request.pose_stamped.pose
            pp.position.x, pp.position.y, pp.position.z = map(float, p_base)
            pp.orientation.x, pp.orientation.y, pp.orientation.z, pp.orientation.w = map(float, quat)
            req.ik_request.robot_state.joint_state.name = list(JOINTS)
            s = np.array(seed, float)
            if k > 0:  # perturb the seed to escape a failed branch
                s = s + np.random.uniform(-0.3, 0.3, size=len(s))
                s = np.clip(s, [l[0] + 0.05 for l in LIMITS], [l[1] - 0.05 for l in LIMITS])
            req.ik_request.robot_state.joint_state.position = [float(v) for v in s]
            req.ik_request.avoid_collisions = False
            req.ik_request.timeout = Duration(sec=1)
            fut = self.ik_cli.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            res = fut.result()
            if res is not None and res.error_code.val == 1:
                sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
                q = np.array([sol[j] for j in JOINTS])
                # verify with FK
                got, _ = self.fk(q)
                err = np.linalg.norm(got - xyz)
                if err < 0.005:
                    return q
                print(f"  ik: FK mismatch {err:.4f}, retry", file=sys.stderr)
            else:
                code = None if res is None else res.error_code.val
                print(f"  ik: fail code={code} (try {k})", file=sys.stderr)
        return None

    # ---------- acting ----------
    def move_joints(self, q_target, duration=None, tol=0.01, max_rounds=6, verbose=True):
        """Send a trajectory; resend until joints are within tol of target."""
        q_target = np.array(q_target, float)
        for r in range(max_rounds):
            q_now, _ = self.joints()
            dq = np.abs(q_target - q_now).max()
            if dq < tol:
                if verbose:
                    print(f"  move_joints: converged (max err {dq:.4f})")
                return True
            dur = duration if (duration and r == 0) else max(1.0, dq / RATE + 0.5)
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = list(JOINTS)
            pt = JointTrajectoryPoint(positions=[float(v) for v in q_target])
            pt.time_from_start = Duration(sec=int(dur), nanosec=int((dur % 1) * 1e9))
            goal.trajectory.points = [pt]
            send = self.fjt.send_goal_async(goal)
            rclpy.spin_until_future_complete(self.node, send, timeout_sec=120)
            gh = send.result()
            res = gh.get_result_async()
            rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
            code = res.result().result.error_code
            q_now, _ = self.joints()
            dq = np.abs(q_target - q_now).max()
            if verbose:
                print(f"  move_joints round {r}: dur={dur:.1f}s code={code} max err {dq:.4f}")
        return dq < tol

    def move_tcp(self, xyz_world, quat, duration=None, seed=None, **kw):
        q = self.ik(xyz_world, quat, seed=seed, at_tcp=True)
        if q is None:
            print(f"  move_tcp: IK failed for {np.round(xyz_world, 3)}")
            return False
        ok = self.move_joints(q, duration, **kw)
        p, _ = self.tcp()
        print(f"  move_tcp: target {np.round(xyz_world, 3)} -> tcp at {np.round(p, 3)} (err {np.linalg.norm(p - xyz_world):.4f})")
        return ok

    def move_tcp_line(self, xyz_from, xyz_to, quat, steps=4, **kw):
        """Straight-ish Cartesian move via several IK waypoints (each seeded
        from the previous) executed one after another."""
        seed, _ = self.joints()
        ok = True
        for i in range(1, steps + 1):
            p = np.array(xyz_from) + (np.array(xyz_to) - np.array(xyz_from)) * i / steps
            q = self.ik(p, quat, seed=seed, at_tcp=True)
            if q is None:
                print(f"  line: IK failed at {np.round(p, 3)}")
                return False
            ok = self.move_joints(q, verbose=False, **kw) and ok
            seed = q
        p, _ = self.tcp()
        print(f"  line: target {np.round(xyz_to, 3)} -> tcp {np.round(p, 3)} (err {np.linalg.norm(p - np.array(xyz_to)):.4f})")
        return ok

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=300)
        r = res.result().result
        gap = self.finger_gap()
        print(f"  gripper({width}): reached={r.reached_goal} stalled={r.stalled} gap={gap:.4f}")
        return gap

    def open(self):
        return self.gripper(GRIP["open_m"])

    def close(self):
        return self.gripper(GRIP["closed_m"])
OPENRUA_EOF

# openrua op 26
timeout 300 python3 -c "
from robot import *
r=Robot()
q,d=r.joints(); print('q',np.round(q,3)); print('fingers',d['panda_finger_joint1'],d['panda_finger_joint2'])
xyz,quat=r.fk(); print('hand world (FK)',np.round(xyz,4),np.round(quat,4))
p,_=r.tcp(); print('tcp world',np.round(p,4))
# IK for current hand pose should return ~current joints
sol=r.ik(xyz,quat,at_tcp=False); print('ik(current) ->',None if sol is None else np.round(sol,3))
# IK for a top-down pose above the pudding
sol2=r.ik([-0.187,0.013,0.60],down_quat(0),at_tcp=True); print('ik above pudding ->',None if sol2 is None else np.round(sol2,3))
if sol2 is not None: print('  fk check tcp', np.round(r.tcp(sol2)[0],4), np.round(r.tcp(sol2)[1],4))
" 2>&1 | grep -v XMLPARSER

# openrua op 27
timeout 300 python3 -c "
from robot import *
import robot
r=Robot()
req = GetPositionFK.Request(); req.header.frame_id=''; req.fk_link_names=['panda_hand','panda_link0']
req.robot_state.joint_state.name=list(JOINTS); req.robot_state.joint_state.position=[0.0,-0.161037389,0.0,-2.44459747,0.0,2.2267522,0.7853981633974483]
fut=r.fk_cli.call_async(req); rclpy.spin_until_future_complete(r.node,fut,timeout_sec=60); res=fut.result()
for ps in res.pose_stamped: print(ps.header.frame_id, np.round([ps.pose.position.x,ps.pose.position.y,ps.pose.position.z],4))
print('expected hand world (-0.053,0,0.7776); link0 world (-0.51,0,0.42)')
" 2>&1 | grep -v XMLPARSER

# openrua op 28
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/robot.py", "old_string": "BASE = np.array([-0.51, 0.0, 0.42])", "new_string": "# Verified: /compute_fk and /compute_ik on this machine work in the WORLD\\n# frame (FK of link0 returns (-0.51,0,0.42)), so no base offset is applied.\\nBASE = np.array([0.0, 0.0, 0.0])", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 29
timeout 300 python3 -c "
from robot import *
r=Robot()
xyz,quat=r.fk(); print('hand world',np.round(xyz,4)); p,_=r.tcp(); print('tcp',np.round(p,4))
for name,tgt in [('above pudding',[-0.187,0.013,0.60]),('pudding grasp',[-0.187,0.013,0.445]),('above mug',[-0.1175,-0.218,0.65]),('mug grasp',[-0.1175,-0.218,0.524]),('above plate',[0.141,-0.041,0.65]),('plate place',[0.141,-0.041,0.555]),('pudding place',[0.14,0.17,0.455])]:
    sol=r.ik(tgt,down_quat(0),at_tcp=True)
    if sol is None: print(name,'-> IK FAIL'); continue
    p,q=r.tcp(sol); print(name,'-> q',np.round(sol,2),' tcp',np.round(p,4),'quat',np.round(q,3))
" 2>&1 | grep -v XMLPARSER

# openrua op 30
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/robot.py", "old_string": "    def ik(self, xyz_world, quat, seed=None, at_tcp=True, tries=5):\\n        \\"\\"\\"IK for a world pose of the HAND (or TCP if at_tcp). Returns q or None.\\"\\"\\"\\n        xyz = np.array(xyz_world, float)\\n        if at_tcp:\\n            xyz = xyz - TCP * quat_R(quat)[:, 2]\\n        p_base = xyz - BASE\\n        if seed is None:\\n            seed, _ = self.joints()\\n        for k in range(tries):", "new_string": "    def ik(self, xyz_world, quat, seed=None, at_tcp=True, tries=8, max_dist=1.5):\\n        \\"\\"\\"IK for a world pose of the HAND (or TCP if at_tcp). Returns the\\n        solution closest (in joint space) to seed, or None. max_dist is the\\n        largest acceptable single-joint deviation from the seed.\\"\\"\\"\\n        xyz = np.array(xyz_world, float)\\n        if at_tcp:\\n            xyz = xyz - TCP * quat_R(quat)[:, 2]\\n        p_base = xyz - BASE\\n        if seed is None:\\n            seed, _ = self.joints()\\n        best, best_d = None, 1e9\\n        for k in range(tries):", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 31
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/robot.py", "old_string": "                got, _ = self.fk(q)\\n                err = np.linalg.norm(got - xyz)\\n                if err < 0.005:\\n                    return q\\n                print(f\\"  ik: FK mismatch {err:.4f}, retry\\", file=sys.stderr)\\n            else:\\n                code = None if res is None else res.error_code.val\\n                print(f\\"  ik: fail code={code} (try {k})\\", file=sys.stderr)\\n        return None", "new_string": "                got, _ = self.fk(q)\\n                err = np.linalg.norm(got - xyz)\\n                if err < 0.005:\\n                    d = np.abs(q - np.array(seed, float)).max()\\n                    if d < best_d:\\n                        best, best_d = q, d\\n                    if d < 0.6:  # clearly same branch; good enough\\n                        break\\n                else:\\n                    print(f\\"  ik: FK mismatch {err:.4f}, retry\\", file=sys.stderr)\\n            else:\\n                code = None if res is None else res.error_code.val\\n                print(f\\"  ik: fail code={code} (try {k})\\", file=sys.stderr)\\n        if best is not None and best_d > max_dist:\\n            print(f\\"  ik: best solution is {best_d:.2f} rad from seed (> {max_dist}); rejecting\\", file=sys.stderr)\\n            return None\\n        return best", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 32
timeout 600 python3 -c "
from robot import *
r=Robot()
READY=np.array([0,-0.785,0,-2.356,0,1.571,0.785])
p,q=r.tcp(READY); print('ready tcp',np.round(p,3),np.round(q,3))
seed=READY
for name,tgt in [('above pudding',[-0.187,0.013,0.60]),('pudding grasp',[-0.187,0.013,0.445]),('above pudding',[-0.187,0.013,0.60]),('pudding place high',[0.14,0.17,0.60]),('pudding place',[0.14,0.17,0.455]),('above mug',[-0.1175,-0.218,0.65]),('mug grasp',[-0.1175,-0.218,0.524]),('above mug',[-0.1175,-0.218,0.65]),('above plate',[0.141,-0.041,0.65]),('plate place',[0.141,-0.041,0.555])]:
    sol=r.ik(tgt,down_quat(0),seed=seed,at_tcp=True)
    if sol is None: print(name,'-> IK FAIL'); continue
    p,q=r.tcp(sol); print(f'{name:20s} -> q',np.round(sol,2),' tcp',np.round(p,4),'dseed %.2f'%np.abs(sol-seed).max())
    seed=sol
" 2>&1 | grep -v XMLPARSER

# openrua op 33
mkdir -p "$(dirname /workspace/stage_a_pudding.py)"
cat > /workspace/stage_a_pudding.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Stage A: pick the chocolate pudding, place it to the right (+y) of the plate."""
from robot import *

PUD = np.array([-0.187, 0.013])        # pudding centre (world xy), from birdview
PLATE = np.array([0.141, 0.000])
TABLE = 0.425
DEST = np.array([0.14, 0.17])          # right of the plate (agentview right = +y)
Q = down_quat(0)                       # fingers along world y (box's 4.4 cm side)
READY = np.array([0, -0.785, 0, -2.356, 0, 1.571, 0.785])

r = Robot()
print("== open gripper"); r.open()
print("== ready pose"); r.move_joints(READY)
print("== above pudding"); assert r.move_tcp([*PUD, 0.60], Q)
print("== descend");       assert r.move_tcp_line([*PUD, 0.60], [*PUD, 0.438], Q, steps=4)
p, _ = r.tcp(); print("   tcp now", np.round(p, 4))
print("== close");         gap = r.close()
print("   finger gap after close:", round(gap, 4), "(>0 means something is held)")
print("== lift");          assert r.move_tcp_line([*PUD, 0.438], [*PUD, 0.60], Q, steps=3)
print("   gap while lifted:", round(r.finger_gap(), 4))
print("== over destination"); assert r.move_tcp([*DEST, 0.60], Q)
print("== lower");         assert r.move_tcp_line([*DEST, 0.60], [*DEST, 0.447], Q, steps=4)
print("== release");       r.open()
print("== retreat up");    assert r.move_tcp_line([*DEST, 0.447], [*DEST, 0.62], Q, steps=3)
print("== back to ready"); r.move_joints(READY)
print("STAGE A DONE")
OPENRUA_EOF

# openrua op 34
nohup python3 -u stage_a_pudding.py > stage_a.log 2>&1 &
echo started

# openrua op 35
sleep 60; grep -v XMLPARSER stage_a.log

# openrua op 36
sleep 90; grep -v XMLPARSER stage_a.log | tail -n +11

# openrua op 37
sleep 45; grep -v XMLPARSER stage_a.log | tail -n 5; pgrep -f stage_a_pudding.py || echo "process finished"

# openrua op 38
sleep 3; pgrep -f stage_a_pudding.py || echo "process finished"; timeout 120 python3 cloud.py birdview && timeout 60 python3 tools/perception/cam_snap.py agentview && python3 -c "
import numpy as np, cv2
d=np.load('birdview_cloud.npz'); xyz=d['xyz']; bgr=d['bgr']
z=xyz[...,2]; x=xyz[...,0]
mask=((z>0.435)&(z<0.70)&np.isfinite(z)&(x>-0.35)&(x<0.5)).astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,4]<30: continue
    m=lab==i; p=xyz[m]; c=bgr[m].mean(0)
    print(f'  comp{i} px={stats[i,4]:5d} x=[{np.nanmin(p[:,0]):.3f},{np.nanmax(p[:,0]):.3f}] y=[{np.nanmin(p[:,1]):.3f},{np.nanmax(p[:,1]):.3f}] z=[{np.nanmin(p[:,2]):.3f},{np.nanmax(p[:,2]):.3f}] mean=({np.nanmean(p[:,0]):.3f},{np.nanmean(p[:,1]):.3f}) bgr={c.astype(int)}')
cv2.imwrite('bird_crop.png',cv2.resize(bgr[200:340,240:400],None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST))
"

# openrua op 39
mkdir -p "$(dirname /workspace/stage_b_mug.py)"
cat > /workspace/stage_b_mug.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Stage B: pick the white mug by its rim (side opposite the handle) and set it on the plate."""
from robot import *

MUG = np.array([-0.1175, -0.177])      # mug axis (world xy), rim fit from birdview
R_MUG = 0.0447                         # outer radius; wall ~5 mm thick
RIM_Z = 0.549
PLATE = np.array([0.141, 0.000])
PLATE_Z = 0.443                        # plate inner surface
OFF = np.array([0.0, -0.041])          # gripper centre sits on the -y wall (handle is +y)
GRASP_Z = 0.520                        # TCP 2.9 cm below the rim
MUG_BOTTOM_BELOW_TCP = (RIM_Z - 0.425) - (RIM_Z - GRASP_Z)   # 0.095
Q = down_quat(0)                       # fingers along world y (radial at the -y wall)
READY = np.array([0, -0.785, 0, -2.356, 0, 1.571, 0.785])

r = Robot()
print("== open gripper"); r.open()
g = MUG + OFF
print("== above mug wall"); assert r.move_tcp([*g, 0.65], Q)
print("== descend");        assert r.move_tcp_line([*g, 0.65], [*g, GRASP_Z], Q, steps=4)
p, _ = r.tcp(); print("   tcp now", np.round(p, 4))
print("== close");          gap = r.close()
print("   finger gap after close:", round(gap, 4), "(expect ~wall thickness, >0)")
print("== lift");           assert r.move_tcp_line([*g, GRASP_Z], [*g, 0.66], Q, steps=3)
print("   gap while lifted:", round(r.finger_gap(), 4))
dest = PLATE + OFF
print("== over plate");     assert r.move_tcp([*dest, 0.66], Q)
place_z = PLATE_Z + MUG_BOTTOM_BELOW_TCP + 0.006
print(f"== lower to place z={place_z:.3f}"); assert r.move_tcp_line([*dest, 0.66], [*dest, place_z], Q, steps=4)
print("== release");        r.open()
print("== retreat up");     assert r.move_tcp_line([*dest, place_z], [*dest, 0.68], Q, steps=3)
print("== back to ready");  r.move_joints(READY)
print("STAGE B DONE")
OPENRUA_EOF

# openrua op 40
nohup python3 -u stage_b_mug.py > stage_b.log 2>&1 &
sleep 100; grep -v XMLPARSER stage_b.log

# openrua op 41
sleep 75; grep -v XMLPARSER stage_b.log | tail -n 8; pgrep -f stage_b_mug.py || echo "process finished"

# openrua op 42
sleep 3; pgrep -f stage_b_mug.py || echo "process finished"; timeout 120 python3 cloud.py birdview && timeout 60 python3 tools/perception/cam_snap.py agentview && timeout 60 python3 tools/perception/cam_snap.py frontview && python3 -c "
import numpy as np, cv2
d=np.load('birdview_cloud.npz'); xyz=d['xyz']; bgr=d['bgr']
z=xyz[...,2]; x=xyz[...,0]; y=xyz[...,1]
mask=((z>0.435)&(z<0.70)&np.isfinite(z)&(x>-0.35)&(x<0.5)).astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,4]<30: continue
    m=lab==i; p=xyz[m]; c=bgr[m].mean(0)
    print(f'  comp{i} px={stats[i,4]:5d} x=[{np.nanmin(p[:,0]):.3f},{np.nanmax(p[:,0]):.3f}] y=[{np.nanmin(p[:,1]):.3f},{np.nanmax(p[:,1]):.3f}] z=[{np.nanmin(p[:,2]):.3f},{np.nanmax(p[:,2]):.3f}] mean=({np.nanmean(p[:,0]):.3f},{np.nanmean(p[:,1]):.3f}) bgr={c.astype(int)}')
def fit_circle(px,py):
    A=np.c_[2*px,2*py,np.ones_like(px)]; b=px**2+py**2
    c=np.linalg.lstsq(A,b,rcond=None)[0]; return c[0],c[1],np.sqrt(c[2]+c[0]**2+c[1]**2)
m=(z>0.52)&(x>0.0)&(x<0.3)&(y>-0.12)&(y<0.04)
cx,cy,rr=fit_circle(x[m],y[m]); print('white mug rim now: center=(%.4f,%.4f) r=%.4f ztop=%.3f n=%d'%(cx,cy,rr,z[m].max(),m.sum()))
cv2.imwrite('bird_crop.png',cv2.resize(bgr[200:340,240:400],None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST))
"

# openrua op 43
timeout 120 python3 cloud.py agentview >/dev/null && python3 -c "
import numpy as np
d=np.load('agentview_cloud.npz'); xyz=d['xyz']
z=xyz[...,2]; x=xyz[...,0]; y=xyz[...,1]
# white mug rim (top points) seen from agentview
m=(z>0.53)&(x>0.05)&(x<0.25)&(y>-0.10)&(y<0.05)
print('mug rim pts: n=%d ztop=%.3f z_rim_range=[%.3f,%.3f] x=[%.3f,%.3f] y=[%.3f,%.3f]'%(m.sum(),z[m].max(),np.percentile(z[m],5),np.percentile(z[m],95),x[m].min(),x[m].max(),y[m].min(),y[m].max()))
# rim height on near (+x) vs far side: tilt check
near=m&(x>0.15); far=m&(x<0.13)
print('rim z near(+x) side: %.3f   far(-x) side: %.3f'%(z[near].max(), z[far].max()))
# plate surface visible around the mug (in front of mug, +x side)
pm=(z>0.43)&(z<0.47)&(x>0.17)&(x<0.21)&(y>-0.03)&(y<0.03)
print('plate surface in front of mug: z=[%.3f,%.3f] n=%d'%(z[pm].min(),z[pm].max(),pm.sum()))
# pudding
pud=(z>0.44)&(z<0.48)&(x>0.09)&(x<0.19)&(y>0.13)&(y<0.20)
print('pudding: mean=(%.3f,%.3f) ztop=%.3f n=%d'%(x[pud].mean(),y[pud].mean(),z[pud].max(),pud.sum()))
"

