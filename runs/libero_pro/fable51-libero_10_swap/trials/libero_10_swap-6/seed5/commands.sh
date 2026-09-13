#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 2 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; ls -R /workspace/tools /workspace/.ros

# openrua op 2
cat tools/action/ik_move.py tools/action/fjt_send.py tools/action/gripper_cmd.py

# openrua op 3
cat tools/perception/cam_snap.py tools/perception/px2world.py

# openrua op 4
timeout 30 ros2 topic list; echo ---; timeout 20 ros2 action list; echo ---; timeout 20 ros2 service list | grep -v -E "parameter|describe|get_type" ; echo ---; timeout 20 ros2 node list

# openrua op 5
timeout 60 ros2 topic echo /joint_states --once; for c in agentview frontview birdview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c; done

# openrua op 6
timeout 30 ros2 topic echo /tf_static --once 2>&1 | head -80; timeout 20 ros2 topic echo /agentview/color/camera_info --once

# openrua op 7
mkdir -p "$(dirname /workspace/scene.py)"
cat > /workspace/scene.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Grab color+depth+intrinsics+TF for cameras, save world-frame point clouds.

Usage: python3 scene.py <cam> [<cam> ...]
Writes <cam>_rgb.png, <cam>_xyz.npy (HxWx3 world coords, nan where invalid).
Also prints all TF frames known and world->hand pose if available.
"""
import sys
import numpy as np
import rclpy
from rclpy.time import Time
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
import cv2


def grab(node, topic, typ, timeout=30.0):
    got = {}
    sub = node.create_subscription(typ, topic, lambda m: got.setdefault("m", m), 1)
    end = node.get_clock().now().nanoseconds / 1e9 + timeout
    while "m" not in got and node.get_clock().now().nanoseconds / 1e9 < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    if "m" not in got:
        raise SystemExit(f"no msg on {topic}")
    return got["m"]


def T_of(t):
    q = t.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
    return T


def main():
    cams = sys.argv[1:]
    rclpy.init()
    node = rclpy.create_node("scene")
    buf = Buffer()
    TransformListener(buf, node)
    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=0.2)
    br = CvBridge()
    for cam in cams:
        rgb = grab(node, f"/{cam}/color/image_raw", Image)
        dep = grab(node, f"/{cam}/depth/image_raw", Image)
        info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
        img = br.imgmsg_to_cv2(rgb, "bgr8")
        d = br.imgmsg_to_cv2(dep, "passthrough").astype(np.float64)
        frame = f"{cam}_optical_frame"
        end = node.get_clock().now().nanoseconds / 1e9 + 10
        while node.get_clock().now().nanoseconds / 1e9 < end:
            rclpy.spin_once(node, timeout_sec=0.2)
            if buf.can_transform("world", frame, Time()):
                break
        t = buf.lookup_transform("world", frame, Time())
        T = T_of(t)
        fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
        H, W = d.shape
        u, v = np.meshgrid(np.arange(W), np.arange(H))
        X = (u - cx) * d / fx
        Y = (v - cy) * d / fy
        pts = np.stack([X, Y, d, np.ones_like(d)], -1) @ T.T
        xyz = pts[..., :3]
        bad = ~np.isfinite(d) | (d <= 0)
        xyz[bad] = np.nan
        cv2.imwrite(f"{cam}_rgb.png", img)
        np.save(f"{cam}_xyz.npy", xyz)
        print(cam, "cam pos", np.round(T[:3, 3], 3), "depth range",
              np.nanmin(d[~bad]) if (~bad).any() else None, np.nanmax(d[~bad]) if (~bad).any() else None)
    print(buf.all_frames_as_string())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 8
timeout 120 python3 scene.py agentview birdview frontview 2>&1 | tail -40

# openrua op 9
python3 -c "
import numpy as np, cv2
xyz=np.load('birdview_xyz.npy'); img=cv2.imread('birdview_rgb.png')
z=xyz[...,2]
ok=np.isfinite(z)
print('z hist', np.histogram(z[ok], bins=20))
# table height: mode
vals=np.round(z[ok],3); u,c=np.unique(vals,return_counts=True); print('mode z', u[np.argsort(c)[-5:]], c[np.argsort(c)[-5:]])
"


# openrua op 10
python3 -c "
import numpy as np, cv2
xyz=np.load('birdview_xyz.npy'); img=cv2.imread('birdview_rgb.png')
z=xyz[...,2]
table=0.425
mask=np.isfinite(z)&(z>table+0.01)&(z<0.65)
# connected components
m8=mask.astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(m8)
for i in range(1,n):
    if stats[i,4]<15: continue
    sel=lab==i
    p=xyz[sel]; col=img[sel].mean(0)
    print(i,'px',stats[i,4],'cent uv',cent[i].round(0),'xyz min',np.nanmin(p,0).round(3),'max',np.nanmax(p,0).round(3),'mean',np.nanmean(p,0).round(3),'bgr',col.round(0))
"


# openrua op 11
python3 -c "
import numpy as np, cv2
xyz=np.load('agentview_xyz.npy'); img=cv2.imread('agentview_rgb.png')
z=xyz[...,2]
ok=np.isfinite(z)
# table height around plate region row 300-330, col 100-200 (bare table)
print('table z sample', np.nanmedian(z[300:330,100:200]), np.nanmedian(z[440:470,500:600]))
table=np.nanmedian(z[300:330,100:200])
mask=ok&(z>table+0.006)&(z<0.75)&(xyz[...,0]<0.45)
m8=mask.astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(m8)
for i in range(1,n):
    if stats[i,4]<30: continue
    sel=lab==i
    p=xyz[sel]; col=img[sel].mean(0)
    print(i,'px',stats[i,4],'cent uv',cent[i].round(0),'bbox',stats[i,:4],'xyz min',np.nanmin(p,0).round(3),'max',np.nanmax(p,0).round(3),'bgr',col.round(0))
"


# openrua op 12
timeout 300 python3 tools/action/fjt_send.py 0.0,-0.161037389,0.0,-2.44459747,0.0,2.2267522,0.7853981633974483 2 && timeout 120 python3 scene.py agentview 2>&1 | head -2 && python3 -c "
import numpy as np, cv2
xyz=np.load('agentview_xyz.npy'); img=cv2.imread('agentview_rgb.png')
z=xyz[...,2]; ok=np.isfinite(z)
table=np.nanmedian(z[300:330,100:200]); print('table',table)
mask=ok&(z>table+0.006)&(z<0.75)&(xyz[...,0]<0.45)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask.astype(np.uint8))
for i in range(1,n):
    if stats[i,4]<30: continue
    sel=lab==i; p=xyz[sel]; col=img[sel].mean(0)
    print(i,'px',stats[i,4],'cent uv',cent[i].round(0),'bbox',stats[i,:4],'xyz min',np.nanmin(p,0).round(3),'max',np.nanmax(p,0).round(3),'bgr',col.round(0))
"

# openrua op 13
python3 -c "
import numpy as np, cv2
xyz=np.load('agentview_xyz.npy'); img=cv2.imread('agentview_rgb.png')
z=xyz[...,2]
def region(name, xr, yr, zmin):
    sel=np.isfinite(z)&(xyz[...,0]>xr[0])&(xyz[...,0]<xr[1])&(xyz[...,1]>yr[0])&(xyz[...,1]<yr[1])&(z>zmin)
    p=xyz[sel]
    print(name, 'n',len(p),'x',p[:,0].min().round(3),p[:,0].max().round(3),'y',p[:,1].min().round(3),p[:,1].max().round(3),'z',p[:,2].min().round(3),p[:,2].max().round(3))
    return p
# white mug rim: z>0.535
r=region('white rim', (-0.3,0.0), (-0.25,-0.03), 0.535)
# fit circle to rim xy
A=np.c_[2*r[:,0],2*r[:,1],np.ones(len(r))]; b=(r[:,0]**2+r[:,1]**2)
c=np.linalg.lstsq(A,b,rcond=None)[0]; cx,cy=c[0],c[1]; rad=np.sqrt(c[2]+cx**2+cy**2)
print('white mug rim circle center',round(cx,3),round(cy,3),'radius',round(rad,3))
r=region('red rim', (-0.3,0.0), (0.03,0.25), 0.56)
A=np.c_[2*r[:,0],2*r[:,1],np.ones(len(r))]; b=(r[:,0]**2+r[:,1]**2)
c=np.linalg.lstsq(A,b,rcond=None)[0]; cx,cy=c[0],c[1]; rad=np.sqrt(c[2]+cx**2+cy**2)
print('red mug rim circle center',round(cx,3),round(cy,3),'radius',round(rad,3))
# pudding
p=region('pudding', (-0.3,-0.12), (-0.06,0.03), 0.435)
top=p[p[:,2]>0.455]; print('pudding top face x',top[:,0].min().round(3),top[:,0].max().round(3),'y',top[:,1].min().round(3),top[:,1].max().round(3), 'n', len(top))
# plate
p=region('plate', (0.05,0.3), (-0.15,0.1), 0.435)
print('plate center est', ((p[:,0].min()+p[:,0].max())/2).round(3), ((p[:,1].min()+p[:,1].max())/2).round(3))
"


# openrua op 14
python3 -c "
import numpy as np, cv2
xyz=np.load('agentview_xyz.npy'); img=cv2.imread('agentview_rgb.png')
z=xyz[...,2]
def region(name, xr, yr, zr):
    sel=np.isfinite(z)&(xyz[...,0]>xr[0])&(xyz[...,0]<xr[1])&(xyz[...,1]>yr[0])&(xyz[...,1]<yr[1])&(z>zr[0])&(z<zr[1])
    p=xyz[sel]
    print(name, 'n',len(p),'x',p[:,0].min().round(3),p[:,0].max().round(3),'y',p[:,1].min().round(3),p[:,1].max().round(3),'z',p[:,2].min().round(3),p[:,2].max().round(3))
    return p, sel
def circ(r):
    A=np.c_[2*r[:,0],2*r[:,1],np.ones(len(r))]; b=(r[:,0]**2+r[:,1]**2)
    c=np.linalg.lstsq(A,b,rcond=None)[0]; cx,cy=c[0],c[1]; rad=np.sqrt(c[2]+cx**2+cy**2)
    return round(cx,3),round(cy,3),round(rad,3)
r,sel=region('white rim', (-0.3,0.0), (-0.25,-0.03), (0.54,0.56))
print('white rim circle', circ(r))
# pixel coords of rim
vs,us=np.where(sel); print('white rim uv range', us.min(),us.max(),vs.min(),vs.max())
r,sel=region('red rim', (-0.3,0.0), (0.03,0.25), (0.565,0.58))
print('red rim circle', circ(r))
# white mug body at mid height z 0.48-0.50
r,sel=region('white body', (-0.3,0.0), (-0.25,-0.03), (0.48,0.50)); print('white body circ', circ(r))
r,sel=region('pudding', (-0.3,-0.12), (-0.06,0.03), (0.435,0.5))
top=r[r[:,2]>0.458]; print('pudding top n',len(top),'x',top[:,0].min().round(3),top[:,0].max().round(3),'y',top[:,1].min().round(3),top[:,1].max().round(3),'center',top[:,0].mean().round(3),top[:,1].mean().round(3))
"


# openrua op 15
timeout 30 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -12; timeout 60 ros2 service call /compute_fk moveit_msgs/srv/GetPositionFK "{header: {frame_id: ''}, fk_link_names: ['panda_hand'], robot_state: {joint_state: {name: [panda_joint1,panda_joint2,panda_joint3,panda_joint4,panda_joint5,panda_joint6,panda_joint7], position: [0.0,-0.161037389,0.0,-2.44459747,0.0,2.2267522,0.7853981633974483]}}}" 2>&1 | grep -A 12 pose_stamped

# openrua op 16
mkdir -p "$(dirname /workspace/robot.py)"
cat > /workspace/robot.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Reusable controller: joint state, FK, IK, trajectory, gripper.

All poses: position of the TCP (fingertip midpoint) unless hand=True.
Orientation: quaternion (x,y,z,w) of the hand frame. TOP_DOWN points the
fingers at the table with the finger axis along world y.
"""
import math
import sys
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

TOP_DOWN = (1.0, 0.0, 0.0, 0.0)


def qmul(a, b):
    x1, y1, z1, w1 = a
    x2, y2, z2, w2 = b
    return (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)


def qz(deg):
    a = math.radians(deg) / 2
    return (0.0, 0.0, math.sin(a), math.cos(a))


def top_down_yaw(deg):
    """Top-down grasp with finger axis rotated `deg` about world z."""
    return qmul(qz(deg), TOP_DOWN)


def quat_to_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class Robot:
    def __init__(self):
        self.m = yaml.safe_load(open("/workspace/machine.yaml"))
        self.traj = next(a for a in self.m["actuators"] if a["kind"] == "joint_trajectory")
        self.grip = next(a for a in self.m["actuators"] if a["kind"] == "gripper")
        self.joints = self.traj["joints"]
        self.tcp_off = float(self.m["hand"]["tcp_offset_m"])
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("robot_ctl")
        self._js = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, self.traj["port"])
        self.gc = ActionClient(self.node, GripperCommand, self.grip["port"])
        self.ik_cli = self.node.create_client(GetPositionIK, self.m["planning"]["ik_service"])
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        assert self.fjt.wait_for_server(timeout_sec=20), "no fjt server"
        assert self.gc.wait_for_server(timeout_sec=20), "no gripper server"
        assert self.ik_cli.wait_for_service(timeout_sec=20), "no ik"
        assert self.fk_cli.wait_for_service(timeout_sec=20), "no fk"
        self.wait_js()

    def _on_js(self, msg):
        self._js = msg

    def spin(self, t=0.1):
        rclpy.spin_once(self.node, timeout_sec=t)

    def wait_js(self):
        self._js = None
        while self._js is None:
            self.spin(0.2)
        return dict(zip(self._js.name, self._js.position))

    def arm_q(self):
        js = self.wait_js()
        return [js[j] for j in self.joints]

    def fingers(self):
        js = self.wait_js()
        return js.get("panda_finger_joint1"), js.get("panda_finger_joint2")

    # ---- kinematics -------------------------------------------------
    def fk(self, q=None):
        q = q if q is not None else self.arm_q()
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = list(self.joints)
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        assert res is not None and res.error_code.val == 1, f"FK failed {res}"
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z])
        quat = (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        return pos, quat

    def tcp(self, q=None):
        pos, quat = self.fk(q)
        R = quat_to_R(quat)
        return pos + self.tcp_off * R[:, 2], quat

    def ik(self, pos, quat, seed=None, hand=False, tries=3):
        pos = np.array(pos, dtype=float)
        if not hand:
            R = quat_to_R(quat)
            pos = pos - self.tcp_off * R[:, 2]
        seed = seed if seed is not None else self.arm_q()
        for _ in range(tries):
            req = GetPositionIK.Request()
            req.ik_request.group_name = self.m["planning"]["group"]
            req.ik_request.pose_stamped.header.frame_id = ""
            p = req.ik_request.pose_stamped.pose
            p.position.x, p.position.y, p.position.z = map(float, pos)
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
            req.ik_request.robot_state.joint_state.name = list(self.joints)
            req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
            req.ik_request.timeout.sec = 2
            fut = self.ik_cli.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            res = fut.result()
            if res is not None and res.error_code.val == 1:
                sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
                return [sol[j] for j in self.joints]
            print(f"  IK attempt failed: {None if res is None else res.error_code.val}", flush=True)
        return None

    # ---- motion -----------------------------------------------------
    def move_q(self, q, seconds=3.0, waypoints=None):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(self.joints)
        pts = []
        wps = list(waypoints or []) + [(q, seconds)]
        for qq, t in wps:
            pt = JointTrajectoryPoint(positions=[float(v) for v in qq])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        assert gh is not None and gh.accepted, "fjt goal rejected"
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=900)
        code = rf.result().result.error_code
        err = np.abs(np.array(self.arm_q()) - np.array(q)).max()
        print(f"  move done code={code} max_joint_err={err:.4f}", flush=True)
        return code, err

    def move_tcp(self, pos, quat, seconds=3.0, seed=None):
        q = self.ik(pos, quat, seed=seed)
        if q is None:
            print(f"  NO IK for {np.round(pos,3)}", flush=True)
            return None
        self.move_q(q, seconds)
        tcp, _ = self.tcp()
        print(f"  tcp now {np.round(tcp,4)} target {np.round(pos,4)} err {np.linalg.norm(tcp-pos):.4f}", flush=True)
        return q

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(self.grip["max_effort"])
        fut = self.gc.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result
        f = self.fingers()
        print(f"  gripper -> {width}: reached={r.reached_goal} stalled={r.stalled} fingers={f}", flush=True)
        return f

    def open(self):
        return self.gripper(self.grip["open_m"])

    def close(self):
        return self.gripper(self.grip["closed_m"])
OPENRUA_EOF

# openrua op 17
timeout 300 python3 -u -c "
from robot import *
r=Robot()
q=r.arm_q(); print('q',np.round(q,3))
pos,quat=r.fk(q); print('hand fk',np.round(pos,4),np.round(quat,4))
tcp,_=r.tcp(q); print('tcp',np.round(tcp,4))
sol=r.ik(pos,quat,hand=True); print('ik of fk pose (hand):',None if sol is None else np.round(sol,3))
print('fingers',r.fingers())
"

# openrua op 18
timeout 300 python3 -u -c "
from robot import *
r=Robot()
sol=[0.058,-0.161,-0.058,-2.445,-0.012,2.227,0.009]
pos,quat=r.fk(sol); print('fk of ik sol',np.round(pos,4),np.round(quat,4))
# also check IK for TOP_DOWN at pregrasp above the white mug
tcp_target=[-0.104,-0.191,0.68]
q=r.ik(tcp_target, TOP_DOWN); print('ik pregrasp', None if q is None else np.round(q,3))
if q: 
    p,qq=r.tcp(q); print('fk check tcp',np.round(p,4),np.round(qq,4))
"

# openrua op 19
python3 - <<'EOF'
import re
s=open('robot.py').read()
s=s.replace('            req.ik_request.group_name = self.m["planning"]["group"]\n',
            '            req.ik_request.group_name = self.m["planning"]["group"]\n            req.ik_request.ik_link_name = "panda_hand"\n')
open('robot.py','w').write(s)
EOF
timeout 300 python3 -u -c "
from robot import *
r=Robot()
tcp_target=[-0.104,-0.191,0.68]
q=r.ik(tcp_target, TOP_DOWN); print('ik pregrasp', None if q is None else np.round(q,3))
if q:
    p,qq=r.tcp(q); print('fk check tcp',np.round(p,4),np.round(qq,4))
"

# openrua op 20
timeout 900 python3 -u -c "
from robot import *
r=Robot()
r.move_tcp([-0.104,-0.191,0.68], TOP_DOWN, 4)
" && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 21
timeout 120 python3 scene.py robot0_eye_in_hand 2>&1 | head -1 && python3 -c "
import numpy as np, cv2
xyz=np.load('robot0_eye_in_hand_xyz.npy'); z=xyz[...,2]
sel=np.isfinite(z)&(z>0.535)&(z<0.56)&(xyz[...,0]>-0.3)&(xyz[...,0]<0.1)&(xyz[...,1]>-0.3)&(xyz[...,1]<-0.03)
r=xyz[sel]; print('rim pts',len(r),'x',r[:,0].min().round(3),r[:,0].max().round(3),'y',r[:,1].min().round(3),r[:,1].max().round(3))
A=np.c_[2*r[:,0],2*r[:,1],np.ones(len(r))]; b=(r[:,0]**2+r[:,1]**2)
c=np.linalg.lstsq(A,b,rcond=None)[0]; cx,cy=c[0],c[1]; rad=np.sqrt(c[2]+cx**2+cy**2)
print('rim circle center',round(cx,4),round(cy,4),'radius',round(rad,4))
# table height check from this cam
print('table z', np.nanmedian(z[100:200,300:500]))
"

# openrua op 22
timeout 120 python3 scene.py robot0_eye_in_hand 2>&1 | grep -v XMLPARSER | head -5

# openrua op 23
python3 -c "
import numpy as np, cv2
xyz=np.load('robot0_eye_in_hand_xyz.npy'); z=xyz[...,2]
print('table z', np.nanmedian(z[100:200,300:500]))
sel=np.isfinite(z)&(z>0.535)&(z<0.56)&(xyz[...,0]>-0.3)&(xyz[...,0]<0.1)&(xyz[...,1]>-0.3)&(xyz[...,1]<-0.03)
r=xyz[sel]; print('rim pts',len(r),'x',r[:,0].min().round(3),r[:,0].max().round(3),'y',r[:,1].min().round(3),r[:,1].max().round(3))
A=np.c_[2*r[:,0],2*r[:,1],np.ones(len(r))]; b=(r[:,0]**2+r[:,1]**2)
c=np.linalg.lstsq(A,b,rcond=None)[0]; cx,cy=c[0],c[1]; rad=np.sqrt(c[2]+cx**2+cy**2)
print('rim circle center',round(cx,4),round(cy,4),'radius',round(rad,4))
# pudding from here
sel=np.isfinite(z)&(z>0.455)&(z<0.48)&(xyz[...,0]>-0.3)&(xyz[...,0]<-0.12)&(xyz[...,1]>-0.08)&(xyz[...,1]<0.04)
p=xyz[sel]; print('pudding top n',len(p),'x',p[:,0].min().round(3),p[:,0].max().round(3),'y',p[:,1].min().round(3),p[:,1].max().round(3),'z',p[:,2].min().round(3),p[:,2].max().round(3),'mean',p.mean(0).round(4))
"

# openrua op 24
timeout 900 python3 -u -c "
from robot import *
r=Robot()
print('fingers before', r.fingers())
r.move_tcp([-0.1043,-0.188,0.60], TOP_DOWN, 2.5)
r.move_tcp([-0.1043,-0.188,0.525], TOP_DOWN, 2.5)
r.close()
r.move_tcp([-0.1043,-0.188,0.70], TOP_DOWN, 3)
print('fingers after lift', r.fingers())
" 2>&1 | grep -v XMLPARSER && timeout 60 python3 tools/perception/cam_snap.py agentview 2>&1 | grep -v XMLPARSER

# openrua op 25
timeout 900 python3 -u -c "
from robot import *
r=Robot()
r.move_tcp([0.169,-0.069,0.70], TOP_DOWN, 4)
print('fingers', r.fingers())
r.move_tcp([0.169,-0.069,0.575], TOP_DOWN, 3)
r.open()
r.move_tcp([0.169,-0.069,0.70], TOP_DOWN, 3)
" 2>&1 | grep -v XMLPARSER && timeout 60 python3 tools/perception/cam_snap.py agentview 2>&1 | grep -v XMLPARSER

# openrua op 26
timeout 900 python3 -u -c "
from robot import *
r=Robot()
r.move_tcp([-0.18,-0.017,0.65], TOP_DOWN, 4)
" 2>&1 | grep -v XMLPARSER && timeout 120 python3 scene.py robot0_eye_in_hand 2>&1 | grep -v XMLPARSER | head -1 && python3 -c "
import numpy as np, cv2
xyz=np.load('robot0_eye_in_hand_xyz.npy'); z=xyz[...,2]
sel=np.isfinite(z)&(z>0.45)&(z<0.50)&(xyz[...,0]>-0.3)&(xyz[...,0]<-0.08)&(xyz[...,1]>-0.10)&(xyz[...,1]<0.06)
p=xyz[sel]; print('pudding n',len(p),'x',p[:,0].min().round(3),p[:,0].max().round(3),'y',p[:,1].min().round(3),p[:,1].max().round(3),'z',p[:,2].min().round(3),p[:,2].max().round(3))
top=p[p[:,2]>0.458]; print('top face n',len(top),'x',top[:,0].min().round(3),top[:,0].max().round(3),'y',top[:,1].min().round(3),top[:,1].max().round(3),'center',top[:,0].mean().round(4),top[:,1].mean().round(4))
# PCA for orientation of top face
c=top[:,:2]-top[:,:2].mean(0); w,v=np.linalg.eigh(c.T@c); print('axes',v.round(3),'extent std',np.sqrt(w/len(c)).round(4))
"

# openrua op 27
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/robot.py", "old_string": "    def move_tcp(self, pos, quat, seconds=3.0, seed=None):\\n        q = self.ik(pos, quat, seed=seed)\\n        if q is None:\\n            print(f\\"  NO IK for {np.round(pos,3)}\\", flush=True)\\n            return None\\n        self.move_q(q, seconds)", "new_string": "    def move_tcp(self, pos, quat, seconds=3.0, seed=None, retries=3):\\n        q = self.ik(pos, quat, seed=seed)\\n        if q is None:\\n            print(f\\"  NO IK for {np.round(pos,3)}\\", flush=True)\\n            return None\\n        for i in range(retries + 1):\\n            code, err = self.move_q(q, seconds)\\n            if code == 0 and err < 0.02:\\n                break\\n            print(f\\"  retry {i+1}: tolerance violation, resending\\", flush=True)", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 28
timeout 900 python3 -u -c "
from robot import *
r=Robot()
r.move_tcp([-0.18,-0.017,0.65], TOP_DOWN, 5)
" 2>&1 | grep -v XMLPARSER && timeout 120 python3 scene.py robot0_eye_in_hand 2>&1 | grep -v XMLPARSER | head -1 && python3 -c "
import numpy as np, cv2
xyz=np.load('robot0_eye_in_hand_xyz.npy'); z=xyz[...,2]
sel=np.isfinite(z)&(z>0.45)&(z<0.50)&(xyz[...,0]>-0.3)&(xyz[...,0]<-0.08)&(xyz[...,1]>-0.10)&(xyz[...,1]<0.06)
p=xyz[sel]; print('pudding n',len(p),'x',p[:,0].min().round(3),p[:,0].max().round(3),'y',p[:,1].min().round(3),p[:,1].max().round(3),'z',p[:,2].min().round(3),p[:,2].max().round(3))
top=p[p[:,2]>0.458]; print('top face n',len(top),'x',top[:,0].min().round(3),top[:,0].max().round(3),'y',top[:,1].min().round(3),top[:,1].max().round(3),'center',top[:,0].mean().round(4),top[:,1].mean().round(4))
c=top[:,:2]-top[:,:2].mean(0); w,v=np.linalg.eigh(c.T@c); print('axes',v.round(3),'extent std',np.sqrt(w/len(c)).round(4))
"

# openrua op 29
timeout 1200 python3 -u -c "
from robot import *
r=Robot()
r.move_tcp([-0.2008,-0.0173,0.55], TOP_DOWN, 3)
r.move_tcp([-0.2008,-0.0173,0.448], TOP_DOWN, 2.5)
r.close()
r.move_tcp([-0.2008,-0.0173,0.65], TOP_DOWN, 3)
print('fingers after lift', r.fingers())
" 2>&1 | grep -v XMLPARSER && timeout 60 python3 tools/perception/cam_snap.py agentview 2>&1 | grep -v XMLPARSER

# openrua op 30
timeout 1200 python3 -u -c "
from robot import *
r=Robot()
r.move_tcp([0.17,0.11,0.65], TOP_DOWN, 5)
print('fingers', r.fingers())
r.move_tcp([0.17,0.11,0.46], TOP_DOWN, 3)
r.open()
r.move_tcp([0.17,0.11,0.65], TOP_DOWN, 3)
r.move_tcp([0.0,0.0,0.75], TOP_DOWN, 4)
" 2>&1 | grep -v XMLPARSER && timeout 60 python3 tools/perception/cam_snap.py agentview 2>&1 | grep -v XMLPARSER

# openrua op 31
timeout 120 python3 scene.py agentview birdview 2>&1 | grep -v XMLPARSER | head -2 && python3 -c "
import numpy as np, cv2
xyz=np.load('agentview_xyz.npy'); z=xyz[...,2]
def reg(name,xr,yr,zr):
    sel=np.isfinite(z)&(xyz[...,0]>xr[0])&(xyz[...,0]<xr[1])&(xyz[...,1]>yr[0])&(xyz[...,1]<yr[1])&(z>zr[0])&(z<zr[1])
    p=xyz[sel]; print(name,'n',len(p),'x',p[:,0].min().round(3),p[:,0].max().round(3),'y',p[:,1].min().round(3),p[:,1].max().round(3),'z',p[:,2].min().round(3),p[:,2].max().round(3)); return p
def circ(r):
    A=np.c_[2*r[:,0],2*r[:,1],np.ones(len(r))]; b=(r[:,0]**2+r[:,1]**2)
    c=np.linalg.lstsq(A,b,rcond=None)[0]; return round(c[0],4),round(c[1],4),round(float(np.sqrt(c[2]+c[0]**2+c[1]**2)),4)
print('table z', np.nanmedian(z[300:330,100:200]))
rim=reg('white mug rim',(0.0,0.4),(-0.2,0.1),(0.565,0.60)); print(' rim circle (cx,cy,r)',circ(rim))
plate=reg('plate (visible)',(0.05,0.3),(-0.15,0.1),(0.44,0.462))
pud=reg('pudding',(0.05,0.35),(0.05,0.25),(0.435,0.50))
top=pud[pud[:,2]>0.458]; print(' pudding top center',top[:,0].mean().round(4),top[:,1].mean().round(4),'top z',top[:,2].mean().round(4))
"

# openrua op 32
python3 -c "
import numpy as np, cv2
xyz=np.load('agentview_xyz.npy'); z=xyz[...,2]
def reg(name,xr,yr,zr):
    sel=np.isfinite(z)&(xyz[...,0]>xr[0])&(xyz[...,0]<xr[1])&(xyz[...,1]>yr[0])&(xyz[...,1]<yr[1])&(z>zr[0])&(z<zr[1])
    p=xyz[sel]
    if len(p)==0: print(name,'EMPTY'); return p
    print(name,'n',len(p),'x',p[:,0].min().round(3),p[:,0].max().round(3),'y',p[:,1].min().round(3),p[:,1].max().round(3),'z',p[:,2].min().round(3),p[:,2].max().round(3)); return p
def circ(r):
    A=np.c_[2*r[:,0],2*r[:,1],np.ones(len(r))]; b=(r[:,0]**2+r[:,1]**2)
    c=np.linalg.lstsq(A,b,rcond=None)[0]; return round(c[0],4),round(c[1],4),round(float(np.sqrt(c[2]+c[0]**2+c[1]**2)),4)
mug=reg('white mug all',(0.0,0.4),(-0.2,0.1),(0.44,0.70))
rim=mug[mug[:,2]>mug[:,2].max()-0.012]; print(' rim z',rim[:,2].min().round(3),rim[:,2].max().round(3),' rim circle (cx,cy,r)',circ(rim))
plate=reg('plate (visible)',(0.05,0.3),(-0.15,0.1),(0.44,0.462))
pud=reg('pudding',(0.05,0.35),(0.05,0.25),(0.435,0.50))
top=pud[pud[:,2]>pud[:,2].max()-0.008]; print(' pudding top center',top[:,0].mean().round(4),top[:,1].mean().round(4),'top z',top[:,2].mean().round(4))
"
