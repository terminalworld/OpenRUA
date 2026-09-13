#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la && find . -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
cd /workspace; cat tools/action/fjt_send.py; echo ----; cat tools/action/gripper_cmd.py; echo ----; cat tools/action/ik_move.py

# openrua op 3
cd /workspace; ros2 topic list; echo ----; ros2 action list; echo ----; ros2 service list | head -50; echo ----; ros2 node list

# openrua op 4
cd /workspace; for c in agentview frontview sideview birdview robot0_robotview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c & done; wait; ls *.png; ros2 topic echo /joint_states --once

# openrua op 5
cd /workspace; timeout 20 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -12; timeout 20 ros2 run tf2_ros tf2_echo world panda_hand 2>&1 | head -12; ros2 topic echo /agentview/color/camera_info --once | head -20

# openrua op 6
mkdir -p "$(dirname /workspace/scan.py)"
cat > /workspace/scan.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Grab color+depth+info+TF for a camera, save npz with world-frame XYZ per pixel.

Usage: python3 scan.py <camera>
Writes <camera>_scan.npz with keys: rgb (HxWx3 bgr), xyz (HxWx3 world), depth.
"""
import sys
import numpy as np
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
import cv2


def grab(node, topic, T, timeout=30.0):
    got = {}
    sub = node.create_subscription(T, topic, lambda m: got.setdefault("m", m), 1)
    import time
    end = time.time() + timeout
    while "m" not in got and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    if "m" not in got:
        raise SystemExit(f"no msg on {topic}")
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
    node = rclpy.create_node("scan_" + cam)
    tfbuf = Buffer()
    TransformListener(tfbuf, node)
    br = CvBridge()
    rgb = br.imgmsg_to_cv2(grab(node, f"/{cam}/color/image_raw", Image), "bgr8")
    depth = br.imgmsg_to_cv2(grab(node, f"/{cam}/depth/image_raw", Image), "passthrough").astype(np.float32)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    frame = f"{cam}_optical_frame"
    import time
    end = time.time() + 15
    while time.time() < end and not tfbuf.can_transform("world", frame, rclpy.time.Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    R = quat_R(q.x, q.y, q.z, q.w)
    tr = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    H, W = depth.shape
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    X = (u - cx) * depth / fx
    Y = (v - cy) * depth / fy
    pc = np.stack([X, Y, depth], -1)
    xyz = pc @ R.T + tr
    np.savez(f"{cam}_scan.npz", rgb=rgb, xyz=xyz, depth=depth)
    cv2.imwrite(f"{cam}.png", rgb)
    print(f"{cam}: cam at {tr}, depth range {np.nanmin(depth):.3f}..{np.nanmax(depth):.3f}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 7
cd /workspace; for c in agentview birdview frontview; do timeout 90 python3 scan.py $c & done; wait

# openrua op 8
cd /workspace; python3 -c "
import numpy as np
d=np.load('birdview_scan.npz'); xyz=d['xyz']
def p(u,v): print((u,v), np.round(xyz[v,u],3))
print('table'); p(200,400); p(450,200)
print('bottle'); 
for u in range(322,340,2): p(u,258)
for v in range(250,268,2): p(330,v)
print('cabinet top'); p(415,300); p(440,270); p(390,340)
print('drawer'); 
for u in range(335,395,5): p(u,300)
for v in range(255,350,5): p(360,v)
"


# openrua op 9
cd /workspace; python3 -c "
import numpy as np
d=np.load('agentview_scan.npz'); xyz=d['xyz']
def p(u,v): print((u,v), np.round(xyz[v,u],3))
print('bottle column u=347');
for v in range(160,245,4): p(347,v)
print('bottle row v=215');
for u in range(330,365,2): p(u,215)
print('drawer front wall top / handle');
for v in range(240,300,4): p(400,v)
print('drawer side wall (left, near camera)');
for v in range(330,360,3): p(420,v)
print('drawer far side wall')
for v in range(230,260,3): p(400,v)
"


# openrua op 10
mkdir -p "$(dirname /workspace/rb.py)"
cat > /workspace/rb.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Robot helper: FK/IK, trajectory, gripper, joint state -- clients built once."""
import math
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import WrenchStamped
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]
LIMITS = FJT["limits_rad"]
TCP = float(M["hand"]["tcp_offset_m"])
BASE = np.array([-0.660, 0.0, 0.912])  # world -> panda_link0 (from TF)


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def R_quat(R):
    """rotation matrix -> (x,y,z,w)"""
    from scipy.spatial.transform import Rotation
    return Rotation.from_matrix(R).as_quat()


def down_quat(yaw=0.0):
    """hand pointing straight down (hand z = -world z), hand x rotated by yaw about world z."""
    from scipy.spatial.transform import Rotation
    R = Rotation.from_euler("xyz", [math.pi, 0, yaw]).as_matrix()
    return R_quat(R)


class Robot:
    def __init__(self, name="rb"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = None
        self._wr = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 1)
        self.node.create_subscription(WrenchStamped,
                                      "/franka_robot_state_broadcaster/external_wrench",
                                      self._on_wr, 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik_cli = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt.wait_for_server(10)
        self.grip.wait_for_server(10)
        self.ik_cli.wait_for_service(10)
        self.fk_cli.wait_for_service(10)

    def _on_js(self, m):
        self._js = m

    def _on_wr(self, m):
        self._wr = m

    def spin(self, t=0.05):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self, fresh=True):
        if fresh:
            self._js = None
            end = time.time() + 20
            while self._js is None and time.time() < end:
                self.spin(0.1)
        d = dict(zip(self._js.name, self._js.position))
        return d

    def arm_q(self):
        d = self.joints()
        return [d[j] for j in ARM]

    def fingers(self):
        d = self.joints()
        return d["panda_finger_joint1"], d["panda_finger_joint2"]

    def wrench(self):
        self._wr = None
        end = time.time() + 10
        while self._wr is None and time.time() < end:
            self.spin(0.1)
        if self._wr is None:
            return None
        f = self._wr.wrench.force
        t = self._wr.wrench.torque
        return np.array([f.x, f.y, f.z, t.x, t.y, t.z])

    # ---------- kinematics ----------
    def fk(self, q=None, link="panda_hand"):
        """world-frame pose (pos, quat) of link for arm config q (default current)."""
        if q is None:
            q = self.arm_q()
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {None if res is None else res.error_code.val}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE
        return pos, np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])

    def ik(self, pos, quat, seed=None, at_tcp=False, timeout=20.0):
        """IK for a world-frame hand pose. Returns arm q list or None."""
        pos = np.array(pos, dtype=float)
        if at_tcp:
            R = quat_R(*quat)
            pos = pos - TCP * R[:, 2]
        pos_b = pos - BASE
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pos_b)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=1, nanosec=0)
        if seed is None:
            seed = self.arm_q()
        req.ik_request.robot_state.joint_state.name = list(ARM)
        req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        res = fut.result()
        if res is None:
            print("IK: no answer")
            return None
        if res.error_code.val != 1:
            print(f"IK failed code={res.error_code.val}")
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        q = [sol[j] for j in ARM]
        for i, (v, (lo, hi)) in enumerate(zip(q, LIMITS)):
            if v < lo - 1e-3 or v > hi + 1e-3:
                print(f"IK solution violates limit on joint {i+1}: {v}")
                return None
        return q

    # ---------- motion ----------
    def move_q(self, q, seconds=3.0, wait=True):
        return self.move_traj([q], [seconds], wait=wait)

    def move_traj(self, qs, times, wait=True):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        for q, t in zip(qs, times):
            pt = JointTrajectoryPoint(positions=[float(v) for v in q])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        fut = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        gh = fut.result()
        if gh is None or not gh.accepted:
            raise RuntimeError("trajectory goal rejected")
        if not wait:
            return gh
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=600)
        r = rf.result()
        code = r.result.error_code if r is not None else None
        q_now = self.arm_q()
        err = max(abs(a - b) for a, b in zip(q_now, qs[-1]))
        print(f"traj done code={code} max_joint_err={err:.4f}")
        return code

    def move_pose(self, pos, quat, seconds=3.0, at_tcp=False, seed=None, via=None):
        """IK then move. via: list of intermediate (pos,quat) for one smooth trajectory."""
        seed = seed or self.arm_q()
        qs, ts = [], []
        pts = (via or []) + [(pos, quat)]
        n = len(pts)
        for i, (p, qt) in enumerate(pts):
            q = self.ik(p, qt, seed=seed, at_tcp=at_tcp)
            if q is None:
                print(f"no IK for waypoint {i}: {p}")
                return None
            qs.append(q)
            ts.append(seconds * (i + 1) / n)
            seed = q
        return self.move_traj(qs, ts)

    def gripper(self, width, wait=True):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        gh = fut.result()
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=120)
        r = rf.result().result
        f = self.fingers()
        print(f"gripper -> reached={r.reached_goal} stalled={r.stalled} fingers={f}")
        return f

    def open(self):
        return self.gripper(GRIP["open_m"])

    def close(self):
        return self.gripper(GRIP["closed_m"])

    def report(self):
        pos, quat = self.fk()
        print("hand pos", np.round(pos, 4), "quat", np.round(quat, 4), "fingers", self.fingers())
        return pos, quat
OPENRUA_EOF

# openrua op 11
cd /workspace; timeout 120 python3 -c "
from rb import *
r=Robot()
print('q',r.arm_q())
pos,quat=r.report()
print('down_quat(0)', down_quat(0))
# IK test: above bottle neck
q=r.ik([-0.138,0.046,1.30], down_quat(0))
print('ik above bottle', q)
print('fk of that', r.fk(q) if q else None)
print('wrench', r.wrench())
"

# openrua op 12
cd /workspace; timeout 120 python3 -c "
import rb
from rb import *
rb.BASE[:] = 0
r=Robot()
pos,quat=r.report()
q=r.ik(pos, quat)
print('ik at current fk pose (no base shift):', q)
q2=r.ik(pos-np.array([-0.66,0,0.912]), quat)
print('ik at current fk pose shifted by base:', q2)
"

# openrua op 13
cd /workspace; timeout 120 python3 -c "
import rb
from rb import *
rb.BASE[:] = 0
r=Robot()
for l in ['panda_link8','panda_hand','panda_hand_tcp','panda_leftfinger']:
    try: print(l, r.fk(link=l))
    except Exception as e: print(l, e)
import subprocess
print(subprocess.run('ros2 param get /move_group robot_description_semantic', shell=True, capture_output=True, text=True).stdout[:3000])
"

# openrua op 14
cd /workspace; python3 - <<'EOF'
import re
s=open('rb.py').read()
s=s.replace('BASE = np.array([-0.660, 0.0, 0.912])  # world -> panda_link0 (from TF)',
 'BASE = np.zeros(3)  # FK/IK service already answers in world coords (verified vs TF)\n# IK tip link is panda_link8 = panda_hand rotated +45deg about z\nRZ45 = np.array([[math.cos(math.pi/4), -math.sin(math.pi/4), 0], [math.sin(math.pi/4), math.cos(math.pi/4), 0], [0, 0, 1]])')
s=s.replace('''        pos_b = pos - BASE
        req = GetPositionIK.Request()''','''        pos_b = pos - BASE
        quat = R_quat(quat_R(*quat) @ RZ45)  # hand pose -> link8 pose
        req = GetPositionIK.Request()''')
open('rb.py','w').write(s)
EOF
timeout 120 python3 -c "
from rb import *
r=Robot()
pos,quat=r.report()
print('cur q', np.round(r.arm_q(),3))
print('ik at current pose:', np.round(r.ik(pos, quat),3))
q=r.ik([-0.138,0.046,1.30], down_quat(0))
print('ik above bottle', np.round(q,3) if q else None)
print('fk of that', r.fk(q) if q else None)
"

# openrua op 15
cd /workspace; timeout 600 python3 -u -c "
from rb import *
r=Robot()
r.open()
B=np.array([-0.138,0.046])
print(r.move_pose([B[0],B[1],1.30], down_quat(0), 3.0))
r.report()
print(r.move_pose([B[0],B[1],1.198], down_quat(0), 2.0))
r.report()
" 2>&1 | tail -20

# openrua op 16
cd /workspace; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih1.png; timeout 60 python3 tools/perception/cam_snap.py agentview av1.png

# openrua op 17
cd /workspace; timeout 90 python3 scan.py agentview >/dev/null; python3 -c "
import numpy as np
d=np.load('agentview_scan.npz'); xyz=d['xyz']
# find bottle top region: points with z in 1.10..1.14 near x=-0.14
m=(xyz[...,2]>1.10)&(xyz[...,2]<1.15)&(xyz[...,0]>-0.3)&(xyz[...,0]<0.0)&(np.abs(xyz[...,1])<0.2)
pts=xyz[m]; print('neck-top pts', len(pts), pts.mean(0) if len(pts) else None, pts.min(0) if len(pts) else None, pts.max(0) if len(pts) else None)
# body region z 0.95..1.03
m=(xyz[...,2]>0.95)&(xyz[...,2]<1.03)&(xyz[...,0]>-0.3)&(xyz[...,0]<-0.05)&(np.abs(xyz[...,1])<0.2)
pts=xyz[m]; print('body pts', len(pts), pts.mean(0), pts.min(0), pts.max(0))
# hand/fingers region z 1.15..1.25
m=(xyz[...,2]>1.15)&(xyz[...,2]<1.26)&(xyz[...,0]>-0.3)&(xyz[...,0]<0.1)&(np.abs(xyz[...,1])<0.2)
pts=xyz[m]; print('hand pts', len(pts), pts.mean(0), pts.min(0), pts.max(0))
"

# openrua op 18
cd /workspace; timeout 600 python3 -u -c "
from rb import *
r=Robot()
print('wrench before', np.round(r.wrench(),2))
r.close()
print('wrench after', np.round(r.wrench(),2))
print(r.move_pose([-0.138,0.046,1.36], down_quat(0), 2.5))
r.report()
print('wrench lifted', np.round(r.wrench(),2))
" 2>&1 | tail; timeout 60 python3 tools/perception/cam_snap.py agentview av2.png

# openrua op 19
cd /workspace; timeout 90 python3 scan.py agentview >/dev/null; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih2.png; python3 -c "
import numpy as np
d=np.load('agentview_scan.npz'); xyz=d['xyz']
for lo,hi in [(1.10,1.15),(1.05,1.10),(0.95,1.03)]:
    m=(xyz[...,2]>lo)&(xyz[...,2]<hi)&(xyz[...,0]>-0.3)&(xyz[...,0]<-0.05)&(xyz[...,1]>-0.05)&(xyz[...,1]<0.15)
    pts=xyz[m]; print(lo,hi,'pts', len(pts), np.round(pts.mean(0),4), np.round(pts.min(0),3), np.round(pts.max(0),3))
"

# openrua op 20
cd /workspace; python3 -c "
import numpy as np
d=np.load('agentview_scan.npz'); xyz=d['xyz']
m=(xyz[...,2]>0.93)&(xyz[...,2]<1.3)&(xyz[...,0]>-0.4)&(xyz[...,0]<0.05)&(xyz[...,1]>-0.1)&(xyz[...,1]<0.2)
pts=xyz[m]; print('pts', len(pts))
import collections
zs=np.round(pts[:,2],2)
for z in sorted(set(zs)):
    s=pts[zs==z]; print(z, len(s), np.round(s.mean(0),3), np.round(s[:,0].min(),3), np.round(s[:,1].min(),3), np.round(s[:,1].max(),3))
" | head -50

# openrua op 21
cd /workspace; python3 -c "
import numpy as np
d=np.load('agentview_scan.npz'); xyz=d['xyz']; dep=d['depth']
for v in range(196,275,4):
    print(v, [ (u, np.round(xyz[v,u],3).tolist()) for u in (343,346,349)])
"

# openrua op 22
cd /workspace; timeout 90 python3 scan.py birdview >/dev/null; python3 -c "
import numpy as np
d=np.load('birdview_scan.npz'); xyz=d['xyz']
m=(xyz[...,2]>0.93)&(xyz[...,2]<1.2)&(xyz[...,0]>-0.3)&(xyz[...,0]<-0.05)&(xyz[...,1]>-0.05)&(xyz[...,1]<0.15)
pts=xyz[m]; print(len(pts)); zs=np.round(pts[:,2],2)
for z in sorted(set(zs)):
    s=pts[zs==z]; print(z, len(s), np.round(s.mean(0),3))
"

# openrua op 23
cd /workspace; python3 -c "
import numpy as np
d=np.load('birdview_scan.npz'); xyz=d['xyz']
m=(xyz[...,2]>0.91)&(xyz[...,2]<1.5)&(xyz[...,0]>-0.3)&(xyz[...,0]<0.0)&(xyz[...,1]>-0.1)&(xyz[...,1]<0.15)
pts=xyz[m]; print(len(pts)); zs=np.round(pts[:,2],2)
for z in sorted(set(zs)):
    s=pts[zs==z]; print(z, len(s), np.round(s.mean(0),3), np.round(s.min(0),3), np.round(s.max(0),3))
"

# openrua op 24
cd /workspace; (timeout 90 python3 scan.py sideview & timeout 90 python3 scan.py frontview & wait) >/dev/null; python3 -c "
import numpy as np
for cam in ['sideview','frontview']:
    d=np.load(cam+'_scan.npz'); xyz=d['xyz']
    m=(xyz[...,2]>0.91)&(xyz[...,2]<1.25)&(xyz[...,0]>-0.25)&(xyz[...,0]<-0.05)&(xyz[...,1]>-0.05)&(xyz[...,1]<0.12)
    pts=xyz[m]; print(cam, len(pts)); zs=np.round(pts[:,2],2)
    for z in sorted(set(zs)):
        s=pts[zs==z]; print(' ',z, len(s), np.round(s.mean(0),3), np.round(s.min(0),3), np.round(s.max(0),3))
"

# openrua op 25
cd /workspace; python3 -c "
import numpy as np
d=np.load('birdview_scan.npz'); xyz=d['xyz']
# objects on table: z>0.905, excluding robot (x<-0.3) 
m=(xyz[...,2]>0.905)&(xyz[...,2]<1.2)&(xyz[...,0]>-0.36)&(xyz[...,0]<0.4)&(np.abs(xyz[...,1])<0.5)
pts=xyz[m]
# cluster by 5cm grid
g=np.floor(pts[:,:2]/0.05).astype(int)
import collections
c=collections.Counter(map(tuple,g))
for k,v in sorted(c.items()):
    if v>20: print(k[0]*0.05, k[1]*0.05, v, np.round(pts[(g==k).all(1)][:,2].max(),3))
"

# openrua op 26
cd /workspace; timeout 300 python3 -u -c "
from rb import *
r=Robot()
Rh=np.array([[0,0,1],[0,-1,0],[1,0,0]]).T  # columns x_h,y_h,z_h
print('det',np.linalg.det(Rh))
qh=R_quat(Rh); print('horiz quat',qh)
q0=r.arm_q()
for p in [(-0.363,-0.03,1.05),(-0.363,-0.03,0.935),(-0.33,-0.03,0.935),(-0.30,-0.03,0.935),(-0.363,0.0,0.935)]:
    q=r.ik(p,qh,seed=q0)
    print(p, None if q is None else np.round(q,3))
    if q is not None: print('   fk',np.round(r.fk(q)[0],3), np.round(r.fk(q)[1],3))
"

# openrua op 27
cd /workspace; timeout 300 python3 -u -c "
from rb import *
r=Robot()
Rh=np.array([[0,0,1],[0,-1,0],[1,0,0]]).T
qh=R_quat(Rh)
seeds=[[0,-0.785,0,-2.356,0,3.14,0.785],[0,0.3,0,-2.5,0,2.9,0.785],[0,0.8,0,-2.3,0,3.2,0.785],[0,1.0,0,-1.8,0,2.9,0.785]]
for s in seeds:
    print('seed',s,'fk',np.round(r.fk(s)[0],3),np.round(r.fk(s)[1],3))
for p in [(-0.363,-0.03,1.05),(-0.363,-0.03,0.935),(-0.30,-0.03,0.935),(-0.25,-0.03,0.935)]:
    for s in seeds:
        q=r.ik(p,qh,seed=s)
        print(p, None if q is None else np.round(q,3))
"

# openrua op 28
cd /workspace; timeout 300 python3 -u -c "
from rb import *
from scipy.spatial.transform import Rotation as Ro
r=Robot()
base=[0,-0.785,0,-2.356,0,1.571,0.785]
def pitch(q):
    p,qt=r.fk(q); R=quat_R(*qt); zh=R[:,2]
    return np.round(p,3), np.round(zh,3)
print('ready', pitch(base))
for i,d in [(1,0.3),(3,0.3),(5,0.3)]:
    q=list(base); q[i]+=d; print('joint',i+1,'+0.3 ->', pitch(q))
"

# openrua op 29
cd /workspace; timeout 600 python3 -u -c "
from rb import *
r=Robot()
best=[]
for a in np.arange(-0.5,2.4,0.15):
    for b in np.arange(-0.7,2.3,0.15):
        q2=-0.785+a; q4=-2.356+b; q6=3.14+a-b
        if not(-1.76<q2<1.76 and -3.07<q4<-0.07 and -0.02<q6<3.75): continue
        q=[0,q2,0,q4,0,q6,0.785]
        p,qt=r.fk(q)
        best.append((abs(p[0]+0.36)+abs(p[2]-0.94), np.round(p,3), np.round(q,3)))
best.sort(key=lambda t:t[0])
for b in best[:8]: print(b)
"

# openrua op 30
cd /workspace; timeout 600 python3 -u -c "
from rb import *
r=Robot()
def Rcols(x,y,z): return R_quat(np.array([x,y,z]).T)
q_negy=Rcols([0,0,-1],[1,0,0],[0,-1,0])   # z_h=-y
q_posy=Rcols([0,0,-1],[-1,0,0],[0,1,0])   # z_h=+y
q_posx=Rcols([0,0,1],[0,-1,0],[1,0,0])
seeds=[[0,-0.785,0,-2.356,0,1.571,0.785],[0,0.5,0,-2.0,1.57,1.57,0.785],[0,0.5,0,-2.0,-1.57,1.57,0.785],[0,0.9,0,-1.6,1.57,1.9,0],[0,0.9,0,-1.6,-1.57,1.9,0],[0.5,0.7,-0.5,-2.0,1.57,1.57,0.785],[-0.5,0.7,0.5,-2.0,-1.57,1.57,0.785]]
for name,qt in [('negy',q_negy),('posy',q_posy),('posx',q_posx)]:
    for p in [(-0.30,-0.03,0.935),(-0.25,-0.03,0.935),(-0.20,0.0,0.935)]:
        sols=set()
        for s in seeds:
            q=r.ik(p,qt,seed=s)
            if q is not None: sols.add(tuple(np.round(q,2)))
        print(name,p); [print('   ',s) for s in sols]
" 2>&1 | grep -v 'IK failed'

# openrua op 31
cd /workspace; timeout 600 python3 -u -c "
from rb import *
r=Robot()
r.open()
B=np.array([-0.136,0.047])
print(r.move_pose([B[0],B[1],1.133], down_quat(0), 2.5))
r.report()
" 2>&1 | tail -5; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih3.png; timeout 60 python3 tools/perception/cam_snap.py frontview fv3.png

# openrua op 32
cd /workspace; timeout 600 python3 -u -c "
from rb import *
r=Robot()
r.close()
import time
for i in range(3):
    r.spin(0.2); print('fingers', r.fingers())
" 2>&1 | tail -5

# openrua op 33
cd /workspace; timeout 600 python3 -u -c "
from rb import *
r=Robot()
print(r.move_pose([-0.136,0.047,1.25], down_quat(0), 2.5))
r.report()
" 2>&1 | tail -3; timeout 60 python3 tools/perception/cam_snap.py frontview fv4.png

# openrua op 34
cd /workspace; timeout 600 python3 -u -c "
from rb import *
r=Robot()
def Rcols(x,y,z): return R_quat(np.array([x,y,z]).T)
q_negy=Rcols([0,0,-1],[1,0,0],[0,-1,0])
q0=r.arm_q(); print('q0',np.round(q0,3))
wps=[((-0.136,0.047,1.35),down_quat(0)),((-0.25,0.10,1.25),q_negy),((-0.30,0.1234,1.12),q_negy),((-0.30,0.1234,0.93),q_negy)]
seed=q0; sols=[]
for p,qt in wps:
    best=None
    for s in [seed,[0.2,0.8,0.0,-2.34,-1.36,1.57,0.78],[0,-0.785,0,-2.356,0,1.571,0.785]]:
        q=r.ik(p,qt,seed=s)
        if q is None: continue
        d=max(abs(a-b) for a,b in zip(q,seed))
        if best is None or d<best[0]: best=(d,q)
    print(p, None if best is None else (round(best[0],2), np.round(best[1],3)))
    if best: seed=best[1]; sols.append(best[1])
np.save('lay_sols.npy', np.array(sols))
" 2>&1 | grep -v 'IK failed'

# openrua op 35
cd /workspace; timeout 900 python3 -u -c "
from rb import *
r=Robot()
sols=np.load('lay_sols.npy')
print(r.move_traj([sols[0],sols[1],sols[2]],[2.5,5.5,8.0]))
r.report()
" 2>&1 | tail -3; timeout 60 python3 tools/perception/cam_snap.py frontview fv5.png; timeout 60 python3 tools/perception/cam_snap.py agentview av5.png

# openrua op 36
cd /workspace; timeout 90 python3 scan.py frontview >/dev/null; python3 -c "
import numpy as np
d=np.load('frontview_scan.npz'); xyz=d['xyz']
m=(xyz[...,2]>0.905)&(xyz[...,2]<1.1)&(xyz[...,0]>-0.3)&(xyz[...,0]<-0.05)&(xyz[...,1]>-0.1)&(xyz[...,1]<0.12)
pts=xyz[m]; print(len(pts)); zs=np.round(pts[:,2],2)
for z in sorted(set(zs)):
    s=pts[zs==z]; print(' ',z, len(s), np.round(s.mean(0),3), np.round(s.min(0),3), np.round(s.max(0),3))
"

# openrua op 37
cd /workspace; ros2 param get /robot_state_publisher robot_description > urdf.txt 2>/dev/null; wc -c urdf.txt; python3 - <<'EOF'
import re
s=open('urdf.txt').read()
i=s.find('<link name="panda_hand"'); print(s[i:i+2500])
i=s.find('<link name="panda_leftfinger"'); print(s[i:i+1500])
EOF

# openrua op 38
cd /workspace; python3 - <<'EOF'
import numpy as np, struct
def stl_bounds(fn):
    d=open(fn,'rb').read()
    n=struct.unpack_from('<I',d,80)[0]
    pts=[]
    for i in range(n):
        off=84+i*50
        v=struct.unpack_from('<12f',d,off)
        pts.append(v[3:6]);pts.append(v[6:9]);pts.append(v[9:12])
    pts=np.array(pts); return pts.min(0), pts.max(0)
for f in ['hand','finger']:
    try: print(f, stl_bounds(f'/opt/ros/jazzy/share/moveit_resources_panda_description/meshes/collision/{f}.stl'))
    except Exception as e: print(f, e)
EOF

# openrua op 39
find / -name "hand*.stl" -o -name "finger*.stl" -o -name "hand.dae" 2>/dev/null | head; 

# openrua op 40
cd /workspace; timeout 900 python3 -u -c "
from rb import *
r=Robot()
r.close()
q0=r.arm_q()
# approach above then at push height
wps=[((-0.06,0.047,1.25),down_quat(0)),((-0.06,0.047,1.133),down_quat(0))]
sols=[]; seed=q0
for p,qt in wps:
    q=r.ik(p,qt,seed=seed); print(p, np.round(q,3)); sols.append(q); seed=q
print(r.move_traj(sols,[3.0,4.5]))
r.report()
# push trajectory: -x in small steps
push=[]; 
for x in np.linspace(-0.06,-0.20,8):
    q=r.ik((x,0.047,1.133),down_quat(0),seed=seed); push.append(q); seed=q
print(r.move_traj(push, list(np.linspace(0.8,6.0,8))))
r.report()
" 2>&1 | tail -8; timeout 60 python3 tools/perception/cam_snap.py frontview fv6.png; timeout 60 python3 tools/perception/cam_snap.py agentview av6.png

# openrua op 41
cd /workspace; timeout 90 python3 scan.py frontview >/dev/null; python3 -c "
import numpy as np
d=np.load('frontview_scan.npz'); xyz=d['xyz']
m=(xyz[...,2]>0.905)&(xyz[...,2]<1.1)&(xyz[...,0]>-0.3)&(xyz[...,0]<-0.05)&(xyz[...,1]>-0.1)&(xyz[...,1]<0.12)
pts=xyz[m]; print(len(pts)); zs=np.round(pts[:,2],2)
for z in sorted(set(zs)):
    s=pts[zs==z]; print(' ',z, len(s), np.round(s.mean(0),3), np.round(s.min(0),3), np.round(s.max(0),3))
"

# openrua op 42
cd /workspace; timeout 900 python3 -u -c "
from rb import *
r=Robot()
seed=r.arm_q()
q1=r.ik((-0.20,0.047,1.25),down_quat(0),seed=seed)
q2=r.ik((-0.118,0.047,1.25),down_quat(0),seed=q1)
print(r.move_traj([q1,q2],[2.0,4.0]))
r.open()
q3=r.ik((-0.118,0.047,1.133),down_quat(0),seed=q2)
print(r.move_q(q3,2.0)); r.report()
" 2>&1 | tail -5; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih7.png

# openrua op 43
cd /workspace; timeout 900 python3 -u -c "
from rb import *
r=Robot()
f=r.close()
if f[0]<0.003: raise SystemExit('missed grasp')
# pivot geometry
x0,y0=-0.118,0.047
rb_=0.027; L=1.03-0.90
piv=np.array([x0-rb_,0.90])
R=np.hypot(rb_,L); phi0=np.arctan2(rb_,L)
seed=r.arm_q(); qs=[]; ts=[]
thetas=np.deg2rad(np.arange(5,91,5))
for i,th in enumerate(thetas):
    gx=piv[0]-R*np.sin(th-phi0); gz=piv[1]+R*np.cos(th-phi0)+0.003
    q=r.ik((gx,y0,gz+TCP),down_quat(0),seed=seed)
    if q is None: raise SystemExit(f'no ik at theta {np.rad2deg(th)}')
    qs.append(q); ts.append(1.0*(i+1)); seed=q
    print(f'theta {np.rad2deg(th):.0f} grasp ({gx:.3f},{gz:.3f})')
np.save('tilt_qs.npy',np.array(qs))
# stage 1: to 30 deg
n1=6
print(r.move_traj(qs[:n1],ts[:n1])); r.report(); print('wrench',np.round(r.wrench(),2))
" 2>&1 | tail -22; timeout 60 python3 tools/perception/cam_snap.py frontview fv8.png

# openrua op 44
cd /workspace; timeout 60 python3 tools/perception/cam_snap.py sideview sv8.png

# openrua op 45
cd /workspace; timeout 900 python3 -u -c "
from rb import *
r=Robot()
seed=r.arm_q()
q1=r.ik((-0.186,0.10,1.143),down_quat(np.pi/2),seed=seed)
print(np.round(q1,3))
print(r.move_q(q1,3.0)); r.report()
" 2>&1 | tail -3; timeout 60 python3 tools/perception/cam_snap.py agentview av9.png; timeout 60 python3 tools/perception/cam_snap.py frontview fv9.png

# openrua op 46
cd /workspace; timeout 900 python3 -u -c "
from rb import *
r=Robot()
seed=r.arm_q(); print('now',np.round(seed,3))
q1=r.ik((-0.186,0.10,1.143),down_quat(np.pi/2),seed=seed)
print('target',np.round(q1,3))
print(r.move_q(q1,4.0)); r.report(); print(np.round(r.arm_q(),3))
" 2>&1 | tail -5

# openrua op 47
cd /workspace; timeout 900 python3 -u -c "
from rb import *
r=Robot()
q1=[-0.191,0.183,0.383,-2.452,-0.137,2.618,-0.48]
print(r.move_q(q1,3.0)); 
now=r.arm_q(); print('err per joint',np.round(np.array(now)-np.array(q1),3))
r.report()
" 2>&1 | tail -4

# openrua op 48
cd /workspace; timeout 90 python3 scan.py frontview >/dev/null; python3 -c "
import numpy as np
d=np.load('frontview_scan.npz'); xyz=d['xyz']
m=(xyz[...,2]>0.88)&(xyz[...,2]<1.2)&(xyz[...,0]>-0.25)&(xyz[...,0]<-0.12)&(xyz[...,1]>0.05)&(xyz[...,1]<0.15)
pts=xyz[m]; print(len(pts)); zs=np.round(pts[:,2],2)
for z in sorted(set(zs)):
    s=pts[zs==z]; print(' ',z, len(s), np.round(s.mean(0),3), np.round(s.min(0),3), np.round(s.max(0),3))
"

# openrua op 49
cd /workspace; timeout 900 python3 -u -c "
from rb import *
r=Robot()
x0,y0=-0.186,0.10
rb_=0.027; L=0.09; press=0.006
pivy=y0-rb_; R=np.hypot(rb_,L); phi0=np.arctan2(rb_,L)
seed=r.arm_q(); qs=[]; ts=[]
# first: lower to touch
q=r.ik((x0,y0,0.90+L-press+TCP),down_quat(np.pi/2),seed=seed); qs.append(q); ts.append(2.0); seed=q
thetas=np.deg2rad(np.arange(5,91,5))
for i,th in enumerate(thetas):
    gy=pivy-R*np.sin(th-phi0); gz=0.90+R*np.cos(th-phi0)-press
    q=r.ik((x0,gy,gz+TCP),down_quat(np.pi/2),seed=seed)
    if q is None: raise SystemExit(f'no ik at theta {np.rad2deg(th)}')
    qs.append(q); ts.append(2.0+1.0*(i+1)); seed=q
np.save('tilt_qs.npy',np.array(qs)); np.save('tilt_ts.npy',np.array(ts))
n1=10
print(r.move_traj(qs[:n1],ts[:n1])); r.report(); print('wrench',np.round(r.wrench(),2))
" 2>&1 | tail -4; timeout 60 python3 tools/perception/cam_snap.py frontview fv10.png

# openrua op 50
cd /workspace; timeout 900 python3 -u -c "
from rb import *
r=Robot()
qs=np.load('tilt_qs.npy'); ts=np.load('tilt_ts.npy')
rest=qs[10:]; t=np.arange(1,len(rest)+1)*1.0
print(r.move_traj(list(rest),list(t))); r.report(); print('wrench',np.round(r.wrench(),2))
" 2>&1 | tail -4; timeout 60 python3 tools/perception/cam_snap.py frontview fv11.png

# openrua op 51
cd /workspace; timeout 900 python3 -u -c "
from rb import *
r=Robot()
r.open()
seed=r.arm_q()
q=r.ik((-0.186,-0.0169,1.30),down_quat(np.pi/2),seed=seed)
print(r.move_q(q,2.5)); r.report()
" 2>&1 | tail -3; (timeout 90 python3 scan.py birdview & timeout 90 python3 scan.py agentview & timeout 90 python3 scan.py frontview & wait) > /dev/null; python3 -c "
import numpy as np
for cam in ['birdview','agentview','frontview']:
    d=np.load(cam+'_scan.npz'); xyz=d['xyz']
    m=(xyz[...,2]>0.905)&(xyz[...,2]<1.0)&(xyz[...,0]>-0.3)&(xyz[...,0]<-0.1)&(xyz[...,1]>-0.15)&(xyz[...,1]<0.12)
    pts=xyz[m]; print(cam, len(pts))
    if len(pts):
        print('  x',np.round(pts[:,0].min(),3),np.round(pts[:,0].max(),3),' y',np.round(pts[:,1].min(),3),np.round(pts[:,1].max(),3),' zmax',np.round(pts[:,2].max(),3))
        ys=np.round(pts[:,1],2)
        for y in sorted(set(ys)):
            s=pts[ys==y]; print('   y',y,len(s),'x',np.round(s[:,0].min(),3),np.round(s[:,0].max(),3),'zmax',np.round(s[:,2].max(),3))
"

# openrua op 52
cd /workspace; python3 -c "
import cv2
im=cv2.imread('birdview.png'); crop=im[200:380,220:420]; cv2.imwrite('bird_crop.png', cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_NEAREST))
im=cv2.imread('agentview.png'); crop=im[150:350,250:450]; cv2.imwrite('agent_crop.png', cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_NEAREST))
"

# openrua op 53
cd /workspace; python3 -c "
import numpy as np
d=np.load('agentview_scan.npz'); xyz=d['xyz']
print('top handle region')
m=(xyz[...,1]>0.12)&(xyz[...,1]<0.199)&(xyz[...,0]>-0.12)&(xyz[...,0]<0.13)&(xyz[...,2]>0.99)&(xyz[...,2]<1.13)
pts=xyz[m]; print(len(pts)); zs=np.round(pts[:,2],2)
for z in sorted(set(zs)):
    s=pts[zs==z]; print(' ',z,len(s),'x',np.round(s[:,0].min(),3),np.round(s[:,0].max(),3),'y',np.round(s[:,1].min(),3),np.round(s[:,1].max(),3))
print('bottom handle region')
m=(xyz[...,1]>-0.02)&(xyz[...,1]<0.055)&(xyz[...,0]>-0.12)&(xyz[...,0]<0.13)&(xyz[...,2]>0.91)&(xyz[...,2]<1.0)
pts=xyz[m]; print(len(pts)); zs=np.round(pts[:,2],2)
for z in sorted(set(zs)):
    s=pts[zs==z]; print(' ',z,len(s),'x',np.round(s[:,0].min(),3),np.round(s[:,0].max(),3),'y',np.round(s[:,1].min(),3),np.round(s[:,1].max(),3))
"

# openrua op 54
cd /workspace; timeout 20 ros2 run tf2_ros tf2_echo world robot0_eye_in_hand_optical_frame 2>&1 | grep -A2 Translation | head -3; timeout 20 ros2 run tf2_ros tf2_echo panda_hand robot0_eye_in_hand_optical_frame 2>&1 | grep -A4 Translation | head -5

# openrua op 55
cd /workspace; cat > look_drawer.py <<'EOF'
import math, numpy as np, subprocess
from rb import Robot, R_quat
r = Robot("look")
a = math.radians(40)
R = np.array([[0,1,0],[math.cos(a),0,math.sin(a)],[math.sin(a),0,-math.cos(a)]])
# columns: x_h=(0,cos,sin), y_h=(1,0,0), z_h=(0,sin,-cos)
print(np.linalg.det(R))
q = R_quat(R)
code = r.move_pose([0.0,-0.10,1.22], q, seconds=4)
if code is None: raise SystemExit
r.report()
EOF
python3 look_drawer.py 2>&1 | tail -5 && python3 scan.py robot0_eye_in_hand 2>&1 | tail -2

# openrua op 56
cd /workspace; python3 - <<'EOF'
import numpy as np
d=np.load("robot0_eye_in_hand_scan.npz"); xyz=d["xyz"]
X,Y,Z=xyz[...,0],xyz[...,1],xyz[...,2]
m=(np.abs(X)<0.08)&(Z>0.915)&(Z<0.94)&(Y>0.0)
print("floor y range", Y[m].min(), Y[m].max())
# back wall: points with x in -0.08..0.08, z 0.94..1.0, y>0.1
m2=(np.abs(X)<0.08)&(Z>0.94)&(Z<1.0)&(Y>0.1)
if m2.any():
    ys=Y[m2]; print("wall-ish y", np.percentile(ys,[5,50,95]), "z", np.percentile(Z[m2],[5,50,95]))
# histogram of y for floor points
h,e=np.histogram(Y[m],bins=np.arange(0,0.4,0.01)); print(list(zip(np.round(e[:-1],2),h)))
# what's at y between 0.19 and 0.3 near x=0 : z profile
for y0 in np.arange(0.15,0.30,0.01):
    mm=(np.abs(X)<0.05)&(Y>y0)&(Y<y0+0.01)
    if mm.any(): print(round(y0,2), "z", np.round(np.percentile(Z[mm],[5,50,95]),3), mm.sum())
EOF

# openrua op 57
cd /workspace; python3 scan.py birdview 2>&1 | tail -1; python3 scan.py agentview 2>&1 | tail -1; python3 - <<'EOF'
import numpy as np
for cam in ["birdview","agentview"]:
    d=np.load(f"{cam}_scan.npz"); xyz=d["xyz"]
    X,Y,Z=xyz[...,0],xyz[...,1],xyz[...,2]
    # bottle: points above table in x<-0.11, y -0.1..0.2, z 0.905..0.97
    m=(X<-0.115)&(X>-0.3)&(Y>-0.1)&(Y<0.2)&(Z>0.905)&(Z<0.97)
    print(cam,"bottle pts",m.sum())
    if m.sum():
        for y0 in np.arange(-0.04,0.16,0.01):
            mm=m&(Y>=y0)&(Y<y0+0.01)
            if mm.sum()>3: print(f"  y {y0:.2f}: x {X[mm].min():.3f}..{X[mm].max():.3f} ztop {Z[mm].max():.3f} n={mm.sum()}")
    # drawer -x wall: points z 0.95..0.99, x -0.13..-0.08, y 0.05..0.2
    w=(Z>0.95)&(Z<0.995)&(X>-0.14)&(X<-0.08)&(Y>0.06)&(Y<0.2)
    if w.sum(): print(cam,"wall x", np.percentile(X[w],[2,50,98]).round(3), "ztop", Z[w].max().round(3))
EOF

# openrua op 58
cd /workspace; grep -n -B3 -A12 'link name="panda_leftfinger"' urdf.txt | head -60; grep -n -A8 'name="panda_finger_joint1"' urdf.txt | head -20

# openrua op 59
cd /workspace; cat > drag_neck.py <<'EOF'
import math, numpy as np, sys
from rb import Robot, R_quat
r = Robot("drag")
b = math.radians(15)
s, c = math.sin(b), math.cos(b)
R = np.column_stack([[0,-1,0],[-c,0,-s],[s,0,-c]])  # x_h, y_h, z_h
assert abs(np.linalg.det(R)-1) < 1e-6
q = R_quat(R)
def go(p, sec=3, via=None):
    code = r.move_pose(p, q, seconds=sec, at_tcp=True, via=None if via is None else [(v, q) for v in via])
    if code is None: sys.exit("IK fail")
    return code
nx = -0.142
go([nx, 0.01, 1.02], 4)
r.report()
go([nx, 0.01, 0.915], 3)
r.report()
f = r.close()
if not (0.004 < f[0] < 0.012):
    print("grasp looks wrong", f); sys.exit(1)
# drag along -x in small steps
xs = np.arange(nx-0.025, -0.26, -0.025)
go([xs[-1], 0.01, 0.915], 5, via=[[x, 0.01, 0.915] for x in xs[:-1]])
r.report()
print("fingers after drag", r.fingers())
r.open()
go([xs[-1], 0.01, 1.02], 3)
r.report()
EOF
python3 drag_neck.py 2>&1 | grep -v "^$" | tail -20

# openrua op 60
cd /workspace; python3 scan.py birdview 2>&1 | tail -1; python3 - <<'EOF'
import numpy as np
d=np.load("birdview_scan.npz"); xyz=d["xyz"]
X,Y,Z=xyz[...,0],xyz[...,1],xyz[...,2]
m=(X<-0.115)&(X>-0.5)&(Y>-0.3)&(Y<0.3)&(Z>0.905)&(Z<0.99)
print("pts",m.sum())
for y0 in np.arange(-0.2,0.2,0.01):
    mm=m&(Y>=y0)&(Y<y0+0.01)
    if mm.sum()>3: print(f"  y {y0:.2f}: x {X[mm].min():.3f}..{X[mm].max():.3f} ztop {Z[mm].max():.3f} n={mm.sum()}")
EOF

# openrua op 61
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rb.py", "old_string": "    def move_traj(self, qs, times, wait=True):\\n        goal = FollowJointTrajectory.Goal()", "new_string": "    def move_traj(self, qs, times, wait=True, max_speed=0.3, retries=3):\\n        \\"\\"\\"Send trajectory; slow down so no joint exceeds max_speed rad/s;\\n        on tolerance-violation aborts (-5) resend the remaining goal.\\"\\"\\"\\n        q0 = self.arm_q()\\n        prev, t_prev, scale = q0, 0.0, 1.0\\n        for q, t in zip(qs, times):\\n            d = max(abs(a - b) for a, b in zip(q, prev))\\n            dt = max(t - t_prev, 1e-3)\\n            scale = max(scale, d / dt / max_speed)\\n            prev, t_prev = q, t\\n        if scale > 1.0:\\n            times = [t * scale for t in times]\\n            print(f\\"traj slowed x{scale:.2f} (total {times[-1]:.1f}s)\\")\\n        code = self._send_traj(qs, times, wait)\\n        if not wait:\\n            return code\\n        for _ in range(retries):\\n            q_now = self.arm_q()\\n            err = max(abs(a - b) for a, b in zip(q_now, qs[-1]))\\n            if code == 0 or err < 0.02:\\n                break\\n            print(f\\"  retrying final point (err {err:.3f})\\")\\n            code = self._send_traj([qs[-1]], [max(3.0, err / max_speed)], True)\\n        return code\\n\\n    def _send_traj(self, qs, times, wait=True):\\n        goal = FollowJointTrajectory.Goal()", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 62
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rb.py", "old_string": "            qs.append(q)\\n            ts.append(seconds * (i + 1) / n)\\n            seed = q\\n        return self.move_traj(qs, ts)", "new_string": "            qs.append(q)\\n            ts.append(seconds * (i + 1) / n)\\n            seed = q\\n        d = [abs(a - b) for a, b in zip(qs[0], self.arm_q())]\\n        print(\\"first waypoint joint deltas:\\", np.round(d, 2))\\n        if max(d) > max_delta:\\n            print(\\"move_pose: reconfiguration too large, refusing\\")\\n            return None\\n        return self.move_traj(qs, ts)", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 63
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rb.py", "old_string": "    def move_pose(self, pos, quat, seconds=3.0, at_tcp=False, seed=None, via=None):", "new_string": "    def move_pose(self, pos, quat, seconds=3.0, at_tcp=False, seed=None, via=None, max_delta=1.6):", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 64
cd /workspace; cat > grasp_body.py <<'EOF'
import math, numpy as np, sys
from rb import Robot, R_quat
r = Robot("gb")
print("q now", np.round(r.arm_q(),3)); r.report()
ang = math.radians(15)          # bottle axis angle from +y toward +x
yh0 = np.array([math.cos(ang), -math.sin(ang), 0.0])   # horizontal, perp to bottle
b = math.radians(20)
yh = math.cos(b)*yh0 + math.sin(b)*np.array([0,0,1.0])
zh = math.sin(b)*yh0 - math.cos(b)*np.array([0,0,1.0])
xh = np.cross(yh, zh)
R = np.column_stack([xh, yh, zh]); assert abs(np.linalg.det(R)-1)<1e-6
q = R_quat(R)
print("x_h", xh.round(3), "y_h", yh.round(3), "z_h", zh.round(3))
tcp = np.array([-0.179, 0.13, 0.92])
def go(p, sec=3, **kw):
    code = r.move_pose(p, q, seconds=sec, at_tcp=True, **kw)
    if code is None: sys.exit("move failed")
    return code
go(tcp + [0,0,0.10], 4)
r.report()
go(tcp, 3)
p,_ = r.report()
print("tcp now", (p + 0.1034*zh).round(4))
f = r.close()
if not (0.015 < f[0] < 0.032):
    print("grasp looks wrong", f); sys.exit(1)
go(tcp + [0,0,0.08], 3)
print("fingers after lift", r.fingers())
r.report()
EOF
python3 grasp_body.py 2>&1 | grep -v "^$" | tail -20

# openrua op 65
cd /workspace; cat > ik_probe.py <<'EOF'
import math, numpy as np
from rb import Robot, R_quat
r = Robot("probe")
ang = math.radians(15); yh0 = np.array([math.cos(ang), -math.sin(ang), 0.0]); b = math.radians(20)
yh = math.cos(b)*yh0 + math.sin(b)*np.array([0,0,1.0]); zh = math.sin(b)*yh0 - math.cos(b)*np.array([0,0,1.0]); xh = np.cross(yh, zh)
q = R_quat(np.column_stack([xh, yh, zh]))
tcp = np.array([-0.179, 0.13, 1.02])
cur = r.arm_q()
seeds = {"cur": cur, "ready": [0,-0.785,0,-2.356,0,1.571,0.785], "ready_j7-": [0,-0.785,0,-2.356,0,1.571,-0.8], "ready_j7+": [0,-0.785,0,-2.356,0,1.571,2.3]}
for n, s in seeds.items():
    sol = r.ik(tcp, q, seed=s, at_tcp=True)
    if sol: print(n, np.round(sol,2), "maxdelta from cur", round(max(abs(a-b) for a,b in zip(sol,cur)),2))
EOF
python3 ik_probe.py 2>&1 | grep -v "^$"

# openrua op 66
cd /workspace; cat > grasp_body.py <<'EOF'
import math, numpy as np, sys
from rb import Robot, R_quat, quat_R
r = Robot("gb")
ang = math.radians(15); yh0 = np.array([math.cos(ang), -math.sin(ang), 0.0]); b = math.radians(20)
yh = math.cos(b)*yh0 + math.sin(b)*np.array([0,0,1.0]); zh = math.sin(b)*yh0 - math.cos(b)*np.array([0,0,1.0]); xh = np.cross(yh, zh)
q = R_quat(np.column_stack([xh, yh, zh]))
tcp = np.array([-0.179, 0.13, 0.92])
def go(p, qq, sec=3, **kw):
    code = r.move_pose(p, qq, seconds=sec, at_tcp=True, **kw)
    if code is None: sys.exit("move failed")
    return code
p0, q0 = r.fk()
tcp0 = p0 + 0.1034*quat_R(*q0)[:,2]
go([tcp0[0], tcp0[1], 1.25], q0, 3)          # straight up, same orientation
r.report()
go([-0.20, 0.10, 1.25], q, 5, max_delta=2.6)   # reorient high up
r.report()
go(tcp + [0,0,0.10], 4)
r.report()
go(tcp, 3)
p,_ = r.report()
print("tcp now", (p + 0.1034*zh).round(4))
f = r.close()
if not (0.015 < f[0] < 0.032):
    print("grasp looks wrong", f); sys.exit(1)
go(tcp + [0,0,0.08], 3)
print("fingers after lift", r.fingers())
r.report()
EOF
python3 grasp_body.py 2>&1 | grep -v "^$" | tail -24

# openrua op 67
cd /workspace; sed -i 's/^go(\[tcp0\[0\], tcp0\[1\], 1.25\], q0, 3).*$/pass/' grasp_body.py; python3 grasp_body.py 2>&1 | grep -v "^$" | tail -24

# openrua op 68
cd /workspace; python3 - <<'EOF'
s=open("grasp_body.py").read()
s=s.replace("def go(p, qq, sec=3, **kw):","def go(p, sec=3, qq=None, **kw):\n    qq = q if qq is None else qq")
s=s.replace("go([-0.20, 0.10, 1.25], q, 5, max_delta=2.6)   # reorient high up\nr.report()\n","")
open("grasp_body.py","w").write(s)
EOF
python3 grasp_body.py 2>&1 | grep -v "^$" | tail -24

# openrua op 69
cd /workspace; python3 tools/perception/cam_snap.py agentview av_lift.png >/dev/null 2>&1; python3 tools/perception/cam_snap.py sideview sv_lift.png >/dev/null 2>&1; echo ok

# openrua op 70
cd /workspace; python3 scan.py agentview 2>&1 | tail -1; python3 - <<'EOF'
import numpy as np, math
d=np.load("agentview_scan.npz"); xyz=d["xyz"]
X,Y,Z=xyz[...,0],xyz[...,1],xyz[...,2]
# held bottle: z between table+0.04 and hand body bottom
m=(Z>0.95)&(Z<1.03)&(X<-0.05)&(X>-0.35)&(Y>-0.1)&(Y<0.3)
print("pts",m.sum())
P=np.stack([X[m],Y[m],Z[m]],1)
# project on bottle axis dir
a=np.array([math.sin(math.radians(15)),math.cos(math.radians(15)),0])
tcp=np.array([-0.1792,0.13,1.0008])
s=(P-tcp)@a
print("axis coord range", s.min().round(4), s.max().round(4), "length", (s.max()-s.min()).round(4))
h,e=np.histogram(s,bins=np.arange(-0.13,0.08,0.01)); print(list(zip(e[:-1].round(2),h)))
# perpendicular offset (in-plane) of points near the ends
perp=np.array([a[1],-a[0],0])
for lo,hi in [(-0.12,-0.10),(0.03,0.05)]:
    mm=(s>lo)&(s<hi)
    if mm.any(): print(lo,hi,"perp mean",((P[mm]-tcp)@perp).mean().round(4),"z mean",P[mm][:,2].mean().round(4))
EOF

# openrua op 71
cd /workspace; python3 - <<'EOF'
import numpy as np, math
d=np.load("agentview_scan.npz"); xyz=d["xyz"]; rgb=d["rgb"]
X,Y,Z=xyz[...,0],xyz[...,1],xyz[...,2]
a=np.array([math.sin(math.radians(15)),math.cos(math.radians(15)),0]); perp=np.array([a[1],-a[0],0])
tcp=np.array([-0.1792,0.13,1.0008])
m=(Z>0.95)&(Z<1.03)&(X<-0.05)&(X>-0.35)&(Y>-0.1)&(Y<0.3)
P=np.stack([X[m],Y[m],Z[m]],1); C=rgb[m]
s=(P-tcp)@a; p=(P-tcp)@perp
for lo in np.arange(-0.12,0.08,0.01):
    mm=(s>=lo)&(s<lo+0.01)
    if mm.sum()>2:
        print(f"s {lo:.2f}: n={mm.sum()} perp {p[mm].min():.3f}..{p[mm].max():.3f} z {P[mm][:,2].min():.3f}..{P[mm][:,2].max():.3f} color(bgr) {C[mm].mean(0).round(0)}")
EOF

# openrua op 72
cd /workspace; cat > place.py <<'EOF'
import math, numpy as np, sys
from rb import Robot, R_quat, quat_R
r = Robot("place")
def quat_from_xh(yaw_deg):
    xh = np.array([math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg)), 0.0])
    zh = np.array([0,0,-1.0]); yh = np.cross(zh, xh)
    return R_quat(np.column_stack([xh, yh, zh]))
def go(p, qq, sec=3, via=None, **kw):
    code = r.move_pose(p, qq, seconds=sec, at_tcp=True, via=via, **kw)
    if code is None: sys.exit("move failed")
    print("  fingers", np.round(r.fingers(),4))
    return code
p0, q0 = r.fk()
go([-0.18, 0.05, 1.15], q0, 4)
go([-0.18, 0.05, 1.15], quat_from_xh(75), 4)
for yaw in (20, -35, -90):
    go([-0.18, 0.05, 1.15], quat_from_xh(yaw), 5)
r.report()
np.save("q_place.npy", np.array(r.arm_q()))
EOF
python3 place.py 2>&1 | grep -v "^$" | tail -30

# openrua op 73
cd /workspace; python3 tools/perception/cam_snap.py agentview av_rot.png >/dev/null 2>&1; python3 tools/perception/cam_snap.py robot0_eye_in_hand eih_rot.png >/dev/null 2>&1; echo ok

# openrua op 74
cd /workspace; python3 scan.py agentview 2>&1 | tail -1; python3 - <<'EOF'
import numpy as np
d=np.load("agentview_scan.npz"); xyz=d["xyz"]; rgb=d["rgb"]
X,Y,Z=xyz[...,0],xyz[...,1],xyz[...,2]
tcp=np.array([-0.1803,0.0498,1.1497])
m=(Z>1.10)&(Z<1.19)&(X<-0.10)&(X>-0.26)&(Y>-0.15)&(Y<0.25)
print("pts",m.sum())
for lo in np.arange(-0.12,0.08,0.01):
    mm=m&(Y-tcp[1]>=lo)&(Y-tcp[1]<lo+0.01)
    if mm.sum()>2:
        print(f"dy {lo:.2f}: n={mm.sum()} x {X[mm].min()-tcp[0]:.3f}..{X[mm].max()-tcp[0]:.3f} z {Z[mm].min():.3f}..{Z[mm].max():.3f} bgr {rgb[mm].mean(0).round(0)}")
EOF

# openrua op 75
cd /workspace; python3 - <<'EOF'
import numpy as np
d=np.load("agentview_scan.npz"); xyz=d["xyz"]; rgb=d["rgb"]
X,Y,Z=xyz[...,0],xyz[...,1],xyz[...,2]
tcp=np.array([-0.1803,0.0498,1.1497])
m=(Z>1.10)&(Z<1.19)&(X<-0.10)&(X>-0.26)&(Y>-0.15)&(Y<0.25)
ys=Y[m]-tcp[1]; print("dy range", ys.min().round(4), ys.max().round(4))
for lo in np.arange(0.07,0.13,0.01):
    mm=m&(ys>=lo)&(ys<lo+0.01) if False else m&(Y-tcp[1]>=lo)&(Y-tcp[1]<lo+0.01)
    if mm.sum()>2: print(f"dy {lo:.2f}: n={mm.sum()} x {X[mm].min()-tcp[0]:.3f}..{X[mm].max()-tcp[0]:.3f} z {Z[mm].min():.3f}..{Z[mm].max():.3f} bgr {rgb[mm].mean(0).round(0)}")
EOF

# openrua op 76
cd /workspace; cat > park_bottle.py <<'EOF'
import math, numpy as np, sys
from rb import Robot, R_quat, down_quat
r = Robot("park")
q = down_quat(-math.pi/2)
def go(p, qq=q, sec=3, via=None, **kw):
    code = r.move_pose(p, qq, seconds=sec, at_tcp=True, via=via, **kw)
    if code is None: sys.exit("move failed")
    return code
go([-0.25, 0.10, 1.10], sec=4)
go([-0.25, 0.10, 0.94], sec=3)
r.open()
go([-0.25, 0.10, 1.10], sec=3)
r.report()
EOF
python3 park_bottle.py 2>&1 | grep -v "^$" | tail -12; python3 scan.py birdview 2>&1 | tail -1; python3 - <<'EOF'
import numpy as np
d=np.load("birdview_scan.npz"); xyz=d["xyz"]
X,Y,Z=xyz[...,0],xyz[...,1],xyz[...,2]
m=(X<-0.13)&(X>-0.5)&(Y>-0.1)&(Y<0.3)&(Z>0.905)&(Z<0.99)
for y0 in np.arange(0.0,0.25,0.01):
    mm=m&(Y>=y0)&(Y<y0+0.01)
    if mm.sum()>3: print(f"  y {y0:.2f}: x {X[mm].min():.3f}..{X[mm].max():.3f} ztop {Z[mm].max():.3f} n={mm.sum()}")
EOF

# openrua op 77
cd /workspace; cat > pull_drawer.py <<'EOF'
import math, numpy as np, sys
from rb import Robot, down_quat
r = Robot("pull")
q = down_quat(-math.pi/2)
def go(p, sec=3, via=None, **kw):
    code = r.move_pose(p, q, seconds=sec, at_tcp=True, via=via, **kw)
    if code is None: sys.exit("move failed")
    return code
r.close()
go([0.003, 0.025, 1.10], 4)
go([0.003, 0.025, 0.935], 3)
r.report()
print("wrench before", r.wrench().round(1))
ys = np.arange(0.015, -0.03, -0.01)
go([0.003, ys[-1], 0.935], 5, via=[[0.003, y, 0.935] for y in ys[:-1]])
print("wrench after", r.wrench().round(1))
r.report()
go([0.003, ys[-1], 1.10], 3)
EOF
python3 pull_drawer.py 2>&1 | grep -v "^$" | tail -14; python3 scan.py birdview 2>&1 | tail -1; python3 - <<'EOF'
import numpy as np
d=np.load("birdview_scan.npz"); xyz=d["xyz"]
X,Y,Z=xyz[...,0],xyz[...,1],xyz[...,2]
# drawer front panel / walls: points z 0.95..0.99 in x -0.12..0.13
m=(Z>0.95)&(Z<0.995)&(X>-0.12)&(X<0.13)&(Y>-0.1)&(Y<0.25)
print("drawer walls y range", Y[m].min().round(3), Y[m].max().round(3))
h=(Z>0.935)&(Z<0.97)&(X>-0.05)&(X<0.06)&(Y>-0.15)&(Y<0.1)
print("handle y range", Y[h].min().round(3), Y[h].max().round(3))
fl=(Z>0.915)&(Z<0.935)&(np.abs(X)<0.08)&(Y>-0.1)&(Y<0.3)
print("floor y range", Y[fl].min().round(3), Y[fl].max().round(3))
EOF

# openrua op 78
cd /workspace; cat > pull2.py <<'EOF'
import math, numpy as np, sys
from rb import Robot, down_quat
r = Robot("pull")
q = down_quat(-math.pi/2)
def go(p, sec=3, via=None, **kw):
    code = r.move_pose(p, q, seconds=sec, at_tcp=True, via=None if via is None else [(v,q) for v in via], **kw)
    if code is None: sys.exit("move failed")
    return code
ys = np.arange(0.015, -0.03, -0.01)
go([0.003, ys[-1], 0.935], 5, via=[[0.003, y, 0.935] for y in ys[:-1]])
print("wrench after", r.wrench().round(1))
r.report()
go([0.003, ys[-1], 1.15], 3)
go([-0.15, -0.10, 1.25], 3)
EOF
python3 pull2.py 2>&1 | grep -v "^$" | tail -8; python3 tools/perception/cam_snap.py agentview av_pull.png >/dev/null 2>&1; echo

# openrua op 79
cd /workspace; python3 scan.py birdview 2>&1 | tail -1; python3 - <<'EOF'
import numpy as np
d=np.load("birdview_scan.npz"); xyz=d["xyz"]
X,Y,Z=xyz[...,0],xyz[...,1],xyz[...,2]
m=(Z>0.95)&(Z<0.995)&(X>-0.12)&(X<0.13)&(Y>-0.15)&(Y<0.25)
print("drawer walls y range", Y[m].min().round(3), Y[m].max().round(3))
h=(Z>0.935)&(Z<0.97)&(X>-0.05)&(X<0.06)&(Y>-0.15)&(Y<0.04)
print("handle y range", Y[h].min().round(3), Y[h].max().round(3))
fl=(Z>0.915)&(Z<0.935)&(np.abs(X)<0.08)&(Y>-0.1)&(Y<0.3)
print("floor y range", Y[fl].min().round(3), Y[fl].max().round(3))
EOF

# openrua op 80
cd /workspace; python3 - <<'EOF'
import numpy as np
d=np.load("birdview_scan.npz"); xyz=d["xyz"]
X,Y,Z=xyz[...,0],xyz[...,1],xyz[...,2]
m=(Z>0.94)&(Z<0.995)&(X>-0.03)&(X<0.03)&(Y>-0.15)&(Y<0.1)
h,e=np.histogram(Y[m],bins=np.arange(-0.15,0.1,0.005))
for a,b in zip(e[:-1],h):
    if b: print(f"y {a:.3f}: {b}  zmean {Z[m&(Y>=a)&(Y<a+0.005)].mean():.3f}")
# side wall front end: x -0.11..-0.10 z>0.95
w=(Z>0.95)&(Z<0.995)&(X>-0.115)&(X<-0.095)&(Y>-0.1)&(Y<0.3)
print("left wall y", Y[w].min().round(3), Y[w].max().round(3))
EOF

# openrua op 81
cd /workspace; python3 - <<'EOF'
import numpy as np
d=np.load("robot0_eye_in_hand_scan.npz"); xyz=d["xyz"]
X,Y,Z=xyz[...,0],xyz[...,1],xyz[...,2]
m=(Z>0.93)&(Z<0.995)&(X>-0.06)&(X<0.07)&(Y>-0.1)&(Y<0.09)
h,e=np.histogram(Y[m],bins=np.arange(-0.1,0.09,0.005))
for a,b in zip(e[:-1],h):
    if b: 
        mm=m&(Y>=a)&(Y<a+0.005)
        print(f"y {a:.3f}: n={b} z {Z[mm].min():.3f}..{Z[mm].max():.3f} x {X[mm].min():.3f}..{X[mm].max():.3f}")
EOF

# openrua op 82
cd /workspace; cat > pull3.py <<'EOF'
import math, numpy as np, sys
from rb import Robot, down_quat
r = Robot("pull3")
q = down_quat(-math.pi/2)
def go(p, sec=3, via=None, **kw):
    code = r.move_pose(p, q, seconds=sec, at_tcp=True, via=None if via is None else [(v,q) for v in via], **kw)
    if code is None: sys.exit("move failed")
    return code
r.open()
wx = -0.105; y0 = 0.13
go([wx, y0, 1.10], 4)
go([wx, y0, 0.966], 3)
r.report()
f = r.close()
if not (0.002 < f[0] < 0.012): print("pinch on wall failed", f); sys.exit(1)
ys = np.arange(y0-0.01, y0-0.055, -0.01)
go([wx, ys[-1], 0.966], 5, via=[[wx, y, 0.966] for y in ys[:-1]])
print("fingers", r.fingers()); r.report()
r.open()
go([wx, ys[-1], 1.15], 3)
go([-0.15, -0.05, 1.25], 3)
EOF
python3 pull3.py 2>&1 | grep -v "^$" | tail -12; python3 scan.py birdview 2>&1 | tail -1; python3 - <<'EOF'
import numpy as np
d=np.load("birdview_scan.npz"); xyz=d["xyz"]
X,Y,Z=xyz[...,0],xyz[...,1],xyz[...,2]
w=(Z>0.95)&(Z<0.995)&(X>-0.115)&(X<-0.095)&(Y>-0.15)&(Y<0.3)
print("left wall y", Y[w].min().round(3), Y[w].max().round(3))
w=(Z>0.95)&(Z<0.995)&(X>0.105)&(X<0.13)&(Y>-0.15)&(Y<0.3)
print("right wall y", Y[w].min().round(3), Y[w].max().round(3))
fl=(Z>0.915)&(Z<0.935)&(np.abs(X)<0.08)&(Y>-0.15)&(Y<0.3)
print("floor y range", Y[fl].min().round(3), Y[fl].max().round(3))
EOF

# openrua op 83
cd /workspace; python3 - <<'EOF'
import numpy as np
d=np.load("robot0_eye_in_hand_scan.npz"); xyz=d["xyz"]
X,Y,Z=xyz[...,0],xyz[...,1],xyz[...,2]
m=(Y>0.17)&(Y<0.2)&(X>-0.06)&(X<0.06)&(Z>0.99)&(Z<1.13)
h,e=np.histogram(Z[m],bins=np.arange(0.99,1.13,0.005))
for a,b in zip(e[:-1],h):
    if b:
        mm=m&(Z>=a)&(Z<a+0.005); print(f"z {a:.3f}: n={b} y {Y[mm].min():.3f}..{Y[mm].max():.3f} x {X[mm].min():.3f}..{X[mm].max():.3f}")
# cabinet lip: points at y 0.2..0.215, z 0.98..1.01 
m2=(Y>0.198)&(Y<0.215)&(np.abs(X)<0.09)&(Z>0.96)&(Z<1.02)
print("lip z", np.percentile(Z[m2],[1,5,50]).round(3) if m2.any() else None)
EOF

# openrua op 84
cd /workspace; python3 - <<'EOF'
import numpy as np
d=np.load("birdview_scan.npz"); xyz=d["xyz"]
X,Y,Z=xyz[...,0],xyz[...,1],xyz[...,2]
for zlo,zhi in [(1.15,1.20),(1.20,1.26),(1.26,1.35)]:
    m=(Z>zlo)&(Z<zhi)&(np.abs(Y+0.05)<0.08)&(X>-0.4)&(X<0.1)
    if m.any(): print(f"z {zlo}-{zhi}: x {X[m].min():.3f}..{X[m].max():.3f} y {Y[m].min():.3f}..{Y[m].max():.3f} n={m.sum()}")
EOF

# openrua op 85
cd /workspace; python3 scan.py birdview 2>&1 | tail -1; python3 - <<'EOF'
import numpy as np
d=np.load("birdview_scan.npz"); xyz=d["xyz"]
X,Y,Z=xyz[...,0],xyz[...,1],xyz[...,2]
for zlo,zhi in [(1.14,1.19),(1.19,1.26),(1.26,1.40)]:
    m=(Z>zlo)&(Z<zhi)&(np.abs(Y+0.05)<0.12)&(X>-0.45)&(X<0.15)
    if m.any(): print(f"z {zlo}-{zhi}: x {X[m].min():.3f}..{X[m].max():.3f} y {Y[m].min():.3f}..{Y[m].max():.3f} n={m.sum()}")
EOF

# openrua op 86
cd /workspace; python3 scan.py sideview 2>&1 | tail -1; python3 scan.py frontview 2>&1 | tail -1; python3 - <<'EOF'
import numpy as np
for cam in ["sideview","frontview"]:
    d=np.load(f"{cam}_scan.npz"); xyz=d["xyz"]
    X,Y,Z=xyz[...,0],xyz[...,1],xyz[...,2]
    for zlo,zhi in [(1.14,1.19),(1.19,1.26)]:
        m=(Z>zlo)&(Z<zhi)&(np.abs(Y+0.05)<0.12)&(X>-0.45)&(X<0.15)
        if m.any(): print(f"{cam} z {zlo}-{zhi}: x {X[m].min():.3f}..{X[m].max():.3f} y {Y[m].min():.3f}..{Y[m].max():.3f} n={m.sum()}")
EOF

# openrua op 87
cd /workspace; python3 - <<'EOF'
import numpy as np
d=np.load("birdview_scan.npz"); xyz=d["xyz"]
X,Y,Z=xyz[...,0],xyz[...,1],xyz[...,2]
m=(X<-0.15)&(X>-0.4)&(Y>-0.05)&(Y<0.3)&(Z>0.905)&(Z<0.99)
for y0 in np.arange(0.0,0.25,0.01):
    mm=m&(Y>=y0)&(Y<y0+0.01)
    if mm.sum()>3: print(f"  y {y0:.2f}: x {X[mm].min():.3f}..{X[mm].max():.3f} ztop {Z[mm].max():.3f} n={mm.sum()}")
EOF

# openrua op 88
cd /workspace; cat > regrasp.py <<'EOF'
import math, numpy as np, sys
from rb import Robot, down_quat
r = Robot("rg")
q = down_quat(-math.pi/2)
def go(p, qq=q, sec=3, **kw):
    code = r.move_pose(p, qq, seconds=sec, at_tcp=True, **kw)
    if code is None: sys.exit("move failed")
    return code
r.open()
tcp = [-0.259, 0.093, 0.922]
go([tcp[0], tcp[1], 1.10], sec=4)
go(tcp)
f = r.close()
if not (0.014 < f[0] < 0.03): print("grasp wrong", f); sys.exit(1)
go([tcp[0], tcp[1], 1.15], sec=3)
print("fingers", r.fingers())
# rotate to yaw 0 (bottle along x)
for yaw in (-math.pi/4, 0.0):
    go([-0.20, 0.05, 1.15], down_quat(yaw), sec=4)
    print("fingers", np.round(r.fingers(),4))
r.report()
EOF
python3 regrasp.py 2>&1 | grep -v "^$" | tail -16

# openrua op 89
cd /workspace; python3 scan.py sideview 2>&1 | tail -1; python3 - <<'EOF'
import numpy as np
d=np.load("sideview_scan.npz"); xyz=d["xyz"]; rgb=d["rgb"]
X,Y,Z=xyz[...,0],xyz[...,1],xyz[...,2]
tcp=np.array([-0.2002,0.0501,1.1495])
m=(Z>1.10)&(Z<1.19)&(Y>-0.05)&(Y<0.15)&(X>-0.40)&(X<0.0)
print("pts",m.sum(), "dx range", (X[m].min()-tcp[0]).round(4), (X[m].max()-tcp[0]).round(4))
for lo in np.arange(-0.13,0.13,0.01):
    mm=m&(X-tcp[0]>=lo)&(X-tcp[0]<lo+0.01)
    if mm.sum()>2: print(f"dx {lo:.2f}: n={mm.sum()} y {Y[mm].min()-tcp[1]:.3f}..{Y[mm].max()-tcp[1]:.3f} z {Z[mm].min():.3f}..{Z[mm].max():.3f} bgr {rgb[mm].mean(0).round(0)}")
EOF

# openrua op 90
cd /workspace; cat > drop.py <<'EOF'
import math, numpy as np, sys
from rb import Robot, down_quat
r = Robot("drop")
q = down_quat(0.0)
def go(p, sec=3, **kw):
    code = r.move_pose(p, q, seconds=sec, at_tcp=True, **kw)
    if code is None: sys.exit("move failed")
    return code
go([0.042, 0.125, 1.20], 5)
print("fingers", np.round(r.fingers(),4))
go([0.042, 0.125, 0.99], 4)
p,_ = r.report()
print("wrench", r.wrench().round(1))
r.open()
go([0.042, 0.125, 1.20], 3)
go([-0.10, -0.05, 1.25], 3)
EOF
python3 drop.py 2>&1 | grep -v "^$" | tail -12; python3 tools/perception/cam_snap.py agentview av_drop.png >/dev/null 2>&1; echo

# openrua op 91
cd /workspace; python3 scan.py birdview 2>&1 | tail -1; python3 scan.py agentview 2>&1 | tail -1; python3 - <<'EOF'
import numpy as np
for cam in ["birdview","agentview"]:
    d=np.load(f"{cam}_scan.npz"); xyz=d["xyz"]; rgb=d["rgb"]
    X,Y,Z=xyz[...,0],xyz[...,1],xyz[...,2]
    # inside drawer interior region, above floor, exclude walls: x -0.095..0.105, y 0.075..0.235
    m=(X>-0.095)&(X<0.105)&(Y>0.05)&(Y<0.24)&(Z>0.93)&(Z<1.0)
    # exclude greyish wall points: keep dark/green or golden
    print(cam, "pts", m.sum())
    for x0 in np.arange(-0.10,0.11,0.01):
        mm=m&(X>=x0)&(X<x0+0.01)
        if mm.sum()>2: print(f"  x {x0:.2f}: y {Y[mm].min():.3f}..{Y[mm].max():.3f} ztop {Z[mm].max():.3f} n={mm.sum()} bgr {rgb[mm].mean(0).round(0)}")
EOF

# openrua op 92
cd /workspace; python3 - <<'EOF'
import numpy as np
for cam in ["birdview","agentview"]:
    d=np.load(f"{cam}_scan.npz"); xyz=d["xyz"]; rgb=d["rgb"].astype(int)
    X,Y,Z=xyz[...,0],xyz[...,1],xyz[...,2]
    dark=(rgb.sum(-1)<150); gold=(rgb[...,2]>rgb[...,0]+40)
    m=(X>-0.12)&(X<0.13)&(Y>0.04)&(Y<0.25)&(Z>0.92)&(Z<1.02)&(dark|gold)
    print(cam,"bottle pts",m.sum(), "x", X[m].min().round(3), X[m].max().round(3), "y", Y[m].min().round(3), Y[m].max().round(3), "ztop", Z[m].max().round(3))
    for x0 in np.arange(-0.10,0.11,0.01):
        mm=m&(X>=x0)&(X<x0+0.01)
        if mm.sum()>2: print(f"  x {x0:.2f}: y {Y[mm].min():.3f}..{Y[mm].max():.3f} ztop {Z[mm].max():.3f} n={mm.sum()} gold={gold[mm].mean():.2f}")
EOF

# openrua op 93
cd /workspace; python3 -c "
import cv2; im=cv2.imread('agentview.png'); c=im[220:380,330:500]; cv2.imwrite('drawer_crop.png', cv2.resize(c,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))"

# openrua op 94
cd /workspace; cat > close_drawer.py <<'EOF'
import math, numpy as np, sys
from rb import Robot, R_quat
r = Robot("close")
g = math.radians(30)
zh = np.array([0, math.sin(g), -math.cos(g)]); yh = np.array([-1.0, 0, 0]); xh = np.cross(yh, zh)
R = np.column_stack([xh, yh, zh]); assert abs(np.linalg.det(R)-1) < 1e-6
q = R_quat(R)
def go(p, sec=3, via=None, **kw):
    code = r.move_pose(p, q, seconds=sec, at_tcp=True, via=None if via is None else [(v, q) for v in via], **kw)
    if code is None: sys.exit("move failed")
    return code
r.close()
go([0.0, 0.02, 1.12], 5)
go([0.0, 0.02, 0.937], 3)
r.report(); print("wrench", r.wrench().round(1))
ys = np.arange(0.04, 0.18, 0.02)
code = go([0.0, ys[-1], 0.937], 8, via=[[0.0, y, 0.937] for y in ys[:-1]])
print("push code", code, "wrench", r.wrench().round(1)); r.report()
go([0.0, 0.10, 1.15], 3)
go([-0.10, -0.05, 1.25], 3)
EOF
python3 close_drawer.py 2>&1 | grep -v "^$" | tail -14; python3 tools/perception/cam_snap.py agentview av_close.png >/dev/null 2>&1; echo

# openrua op 95
cd /workspace; python3 -c "
import cv2; im=cv2.imread('av_close.png'); c=im[200:400,300:520]; cv2.imwrite('close_crop.png', cv2.resize(c,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))"

# openrua op 96
cd /workspace; python3 scan.py birdview 2>&1 | tail -1; python3 -c "
import numpy as np
d=np.load('birdview_scan.npz'); xyz=d['xyz']
# drawer front region: x -0.1..0.11, look for points with z 0.93..1.0 by y
m=(xyz[...,0]>-0.09)&(xyz[...,0]<0.10)&(xyz[...,2]>0.93)&(xyz[...,2]<1.0)
P=xyz[m]
import collections
for y in np.arange(0.0,0.26,0.01):
    s=P[(P[:,1]>=y)&(P[:,1]<y+0.01)]
    if len(s): print(f'y {y:.2f} n {len(s):4d} z {s[:,2].min():.3f}..{s[:,2].max():.3f} x {s[:,0].min():.3f}..{s[:,0].max():.3f}')
"

# openrua op 97
cd /workspace; python3 -c "
import numpy as np
d=np.load('birdview_scan.npz'); xyz=d['xyz']
z=xyz[...,2]; x=xyz[...,0]; y=xyz[...,1]
# anything above table (0.92) and below cabinet-top, outside cabinet footprint (y>0.19) and not robot (x<-0.3?)
m=np.isfinite(z)&(z>0.93)&(z<1.12)&(y<0.19)
P=xyz[m]
print('points above table outside cabinet:',len(P))
if len(P):
    for xb in np.arange(-0.6,0.8,0.1):
        s=P[(P[:,0]>=xb)&(P[:,0]<xb+0.1)]
        if len(s): print(f'x {xb:.1f}: n {len(s)} y {s[:,1].min():.2f}..{s[:,1].max():.2f} z {s[:,2].min():.3f}..{s[:,2].max():.3f}')
"; python3 tools/perception/cam_snap.py frontview fv_final.png >/dev/null 2>&1; python3 tools/perception/cam_snap.py agentview av_final.png

# openrua op 98
cd /workspace; python3 scan.py sideview 2>&1 | tail -1; python3 -c "
import numpy as np
d=np.load('sideview_scan.npz'); xyz=d['xyz']
x=xyz[...,0]; y=xyz[...,1]; z=xyz[...,2]
m=np.isfinite(z)&(x>-0.12)&(x<0.13)&(y<0.3)&(y>0.05)
for zb in np.arange(0.93,1.14,0.01):
    s=xyz[m&(z>=zb)&(z<zb+0.01)]
    if len(s):
        # exclude handle: take points with x in [-0.12,-0.06] or [0.06,0.13] (panel, away from handle)
        pan=s[(np.abs(s[:,0])>0.06)]
        print(f'z {zb:.2f} n {len(s):4d} y_all {s[:,1].min():.3f}..{s[:,1].max():.3f}  panel(y) {np.median(pan[:,1]) if len(pan) else float(\"nan\"):.3f}')
"

# openrua op 99
cd /workspace; python3 scan.py agentview 2>&1 | tail -1; python3 -c "
import numpy as np
d=np.load('agentview_scan.npz'); xyz=d['xyz']
x=xyz[...,0]; y=xyz[...,1]; z=xyz[...,2]
m=np.isfinite(z)&(x>-0.2)&(x<0.2)&(y<0.25)&(y>0.1)&(z>0.92)&(z<1.13)
for zb in np.arange(0.92,1.13,0.01):
    s=xyz[m&(z>=zb)&(z<zb+0.01)]
    if len(s):
        pan=s[(np.abs(s[:,0])>0.06)&(np.abs(s[:,0])<0.1)]
        side=s[(np.abs(s[:,0])>=0.1)]
        f=lambda a: f'{np.median(a[:,1]):.3f} (n{len(a)})' if len(a) else '-'
        print(f'z {zb:.2f} panel_y {f(pan)} side_y {f(side)} xrange {s[:,0].min():.3f}..{s[:,0].max():.3f}')
"

# openrua op 100
cd /workspace; cat > close2.py <<'EOF'
import math, numpy as np, sys
from rb import Robot, R_quat
r = Robot("close2")
g = math.radians(30)
zh = np.array([0, math.sin(g), -math.cos(g)]); yh = np.array([-1.0, 0, 0]); xh = np.cross(yh, zh)
q = R_quat(np.column_stack([xh, yh, zh]))
def go(p, sec=3, **kw):
    code = r.move_pose(p, q, seconds=sec, at_tcp=True, **kw)
    if code is None: sys.exit("move failed")
    return code
r.close()
go([0.0, 0.14, 1.12], 5)
go([0.0, 0.14, 0.937], 3)
print("wrench", r.wrench().round(1))
for y in (0.165, 0.175, 0.185):
    c = go([0.0, y, 0.937], 3)
    w = r.wrench(); print(f"y {y}: code {c} wrench {w.round(1)}")
    if c != 0 or abs(w[1]) > 15: break
r.report()
go([0.0, 0.12, 1.15], 3)
go([-0.10, -0.05, 1.25], 3)
EOF
python3 close2.py 2>&1 | grep -E "wrench|hand pos|retry|refus|IK" 

# openrua op 101
cd /workspace; python3 scan.py agentview 2>&1 | tail -1; python3 -c "
import numpy as np
d=np.load('agentview_scan.npz'); xyz=d['xyz']
x=xyz[...,0]; y=xyz[...,1]; z=xyz[...,2]
m=np.isfinite(z)&(x>-0.2)&(x<0.2)&(y<0.25)&(y>0.1)&(z>0.92)&(z<1.13)
for zb in [0.93,0.95,0.97,0.98,0.99,1.0,1.03,1.06,1.09,1.11]:
    s=xyz[m&(z>=zb)&(z<zb+0.01)]
    pan=s[(np.abs(s[:,0])>0.06)&(np.abs(s[:,0])<0.1)]
    hb=s[(np.abs(s[:,0])<0.045)]
    f=lambda a: f'{np.median(a[:,1]):.3f} (n{len(a)})' if len(a) else '-'
    print(f'z {zb:.2f} panel_y {f(pan)} centre_y(min) {hb[:,1].min() if len(hb) else float(\"nan\"):.3f}')
"; python3 scan.py birdview 2>&1|tail -1; python3 -c "
import numpy as np
d=np.load('birdview_scan.npz'); xyz=d['xyz']
m=(xyz[...,0]>-0.09)&(xyz[...,0]<0.10)&(xyz[...,2]>0.93)&(xyz[...,2]<1.0)&(xyz[...,1]<0.3)
P=xyz[m]
for y in np.arange(0.10,0.24,0.01):
    s=P[(P[:,1]>=y)&(P[:,1]<y+0.01)]
    if len(s): print(f'bird y {y:.2f} n {len(s):4d} z {s[:,2].min():.3f}..{s[:,2].max():.3f} x {s[:,0].min():.3f}..{s[:,0].max():.3f}')
"

# openrua op 102
cd /workspace; python3 tools/perception/cam_snap.py agentview av_final2.png >/dev/null 2>&1; python3 -c "
import cv2; im=cv2.imread('av_final2.png'); c=im[150:400,380:640]; cv2.imwrite('final_crop.png', cv2.resize(c,None,fx=2,fy=2,interpolation=cv2.INTER_CUBIC))"
