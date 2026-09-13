#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
cd /workspace; cat tools/action/fjt_send.py; echo ----; cat tools/action/gripper_cmd.py; echo ----; cat tools/action/ik_move.py

# openrua op 3
cd /workspace; timeout 30 ros2 topic list; echo ----; timeout 20 ros2 action list; echo ----; timeout 20 ros2 node list

# openrua op 4
cd /workspace; for c in agentview birdview frontview sideview robot0_eye_in_hand robot0_robotview; do timeout 60 python3 tools/perception/cam_snap.py $c; done; timeout 20 ros2 topic echo /joint_states --once

# openrua op 5
cd /workspace; python3 -c "
import cv2
im=cv2.imread('robot0_robotview.png')
cv2.imwrite('crop_cans.png', cv2.resize(im[20:260,140:440],None,fx=2.5,fy=2.5,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('robot0_eye_in_hand.png')
cv2.imwrite('crop_eih.png', cv2.resize(im[260:420,180:520],None,fx=2.5,fy=2.5,interpolation=cv2.INTER_CUBIC))
"

# openrua op 6
cd /workspace; python3 -c "
import cv2
im=cv2.imread('birdview.png')
cv2.imwrite('crop_bird.png', cv2.resize(im[200:330,230:420],None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))
"; timeout 60 python3 tools/perception/px2world.py birdview 383 285 2>&1 | grep -v XMLPARSER

# openrua op 7
cd /workspace; cat > px_batch.py <<'EOF'
import sys, struct, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
cam = sys.argv[1]
pts = [tuple(map(int, a.split(','))) for a in sys.argv[2:]]
rclpy.init(); node = rclpy.create_node('pxb')
tfbuf = Buffer(); TransformListener(tfbuf, node)
got = {}
node.create_subscription(Image, f'/{cam}/depth/image_raw', lambda m: got.setdefault('d', m), 1)
node.create_subscription(CameraInfo, f'/{cam}/color/camera_info', lambda m: got.setdefault('i', m), 1)
while 'd' not in got or 'i' not in got: rclpy.spin_once(node, timeout_sec=0.2)
d, info = got['d'], got['i']
depth = np.frombuffer(d.data, dtype=np.float32).reshape(d.height, d.width)
fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
frame = f'{cam}_optical_frame'
while not tfbuf.can_transform('world', frame, rclpy.time.Time()): rclpy.spin_once(node, timeout_sec=0.2)
t = tfbuf.lookup_transform('world', frame, rclpy.time.Time())
q = t.transform.rotation; x,y,z,w = q.x,q.y,q.z,q.w
R = np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
T = np.eye(4); T[:3,:3]=R; T[:3,3]=[t.transform.translation.x,t.transform.translation.y,t.transform.translation.z]
print('cam pose', T[:3,3], 'size', d.width, d.height)
for u,v in pts:
    zz = depth[v,u]
    p = T @ np.array([(u-cx)*zz/fx,(v-cy)*zz/fy,zz,1])
    print(f'({u},{v}) depth={zz:.3f} -> {p[0]:.4f} {p[1]:.4f} {p[2]:.4f}')
rclpy.shutdown()
EOF
timeout 60 python3 px_batch.py birdview 284,225 284,259 261,291 277,309 292,292 319,282 332,299 382,286 320,150 200,300 2>&1 | grep -v XMLPARSER

# openrua op 8
cd /workspace; timeout 20 ros2 topic echo /tf_static --once 2>&1 | grep -v XMLPARSER | head -80; timeout 20 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | grep -v XMLPARSER | head -20

# openrua op 9
cd /workspace; timeout 15 ros2 run tf2_ros tf2_echo world panda_hand 2>&1 | grep -v XMLPARSER | head -12; timeout 15 ros2 run tf2_ros tf2_echo panda_hand robot0_eye_in_hand_optical_frame 2>&1 | grep -v XMLPARSER | head -8

# openrua op 10
cd /workspace; timeout 60 python3 px_batch.py robot0_eye_in_hand 484,340 232,336 260,155 505,115 455,185 590,185 60,200 320,240 2>&1 | grep -v XMLPARSER

# openrua op 11
cd /workspace; timeout 60 python3 px_batch.py birdview 362,286 402,286 382,262 382,310 372,286 392,286 2>&1 | grep -v XMLPARSER; timeout 20 ros2 interface show moveit_msgs/srv/GetPositionIK | head -30

# openrua op 12
mkdir -p "$(dirname /workspace/rob.py)"
cat > /workspace/rob.py <<'OPENRUA_EOF'
"""Reusable robot helpers: one node, clients built once.

World frame -> planner frame (panda_link0): base = world - (-0.51, 0, 0.42)
(read from TF world->panda_link0 at session start).
"""
import math
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint

BASE_IN_WORLD = np.array([-0.51, 0.0, 0.42])
TCP_OFF = 0.1034
TABLE_Z = 0.425  # world

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def down_quat(yaw_deg=0.0):
    """Hand pointing straight down (hand z = world -z), fingers separated
    along world y when yaw=0; yaw rotates the finger axis about world z."""
    # q = Rz(yaw) * Rx(pi)
    h = math.radians(yaw_deg) / 2
    # Rx(pi) = (1,0,0,0); Rz(yaw) = (0,0,sin h, cos h)
    # product (Rz * Rx): w = -0, x = cos h, y = sin h, z = 0
    return (math.cos(h), math.sin(h), 0.0, 0.0)


class Robot:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("rob")
        self.js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      self._on_js, 1)
        self.tfbuf = Buffer()
        TransformListener(self.tfbuf, self.node)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK,
                                          M["planning"]["ik_service"])
        assert self.fjt.wait_for_server(timeout_sec=20)
        assert self.grip.wait_for_server(timeout_sec=20)
        assert self.ik.wait_for_service(timeout_sec=20)
        self.spin(0.5)

    def _on_js(self, msg):
        self.js = dict(zip(msg.name, msg.position))

    def spin(self, sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    # ---- sensing
    def joints(self):
        self.js = {}
        while not self.js:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return dict(self.js)

    def arm_q(self):
        j = self.joints()
        return [j[n] for n in ARM]

    def fingers(self):
        j = self.joints()
        return j.get("panda_finger_joint1"), j.get("panda_finger_joint2")

    def hand_world(self):
        """(pos, quat xyzw) of panda_hand in world via TF (fresh spin)."""
        self.spin(0.3)
        t = self.tfbuf.lookup_transform("world", "panda_hand",
                                        rclpy.time.Time())
        tr, q = t.transform.translation, t.transform.rotation
        return np.array([tr.x, tr.y, tr.z]), (q.x, q.y, q.z, q.w)

    def tcp_world(self):
        p, q = self.hand_world()
        R = quat_to_R(*q)
        return p + TCP_OFF * R[:, 2], q

    # ---- acting
    def ik_world_tcp(self, xyz, quat, seed=None):
        """IK for a TCP pose given in WORLD coords; returns arm joints."""
        R = quat_to_R(*quat)
        hand = np.array(xyz) - TCP_OFF * R[:, 2]
        base = hand - BASE_IN_WORLD
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, base)
        (p.orientation.x, p.orientation.y,
         p.orientation.z, p.orientation.w) = map(float, quat)
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=2)
        s = JointState()
        s.name = list(ARM)
        s.position = list(seed) if seed is not None else self.arm_q()
        req.ik_request.robot_state.joint_state = s
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            code = None if res is None else res.error_code.val
            raise RuntimeError(f"IK failed code={code} for tcp={xyz}")
        sol = dict(zip(res.solution.joint_state.name,
                       res.solution.joint_state.position))
        return [sol[n] for n in ARM]

    def move_q(self, q, sec=3.0, tol=0.02):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pt = JointTrajectoryPoint(positions=[float(v) for v in q])
        pt.time_from_start = Duration(sec=int(sec),
                                      nanosec=int((sec % 1) * 1e9))
        goal.trajectory.points = [pt]
        fut = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=120)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=600)
        code = rf.result().result.error_code
        self.spin(0.3)
        err = max(abs(a - b) for a, b in zip(self.arm_q(), q))
        print(f"  move: code={code} max_joint_err={err:.4f}", flush=True)
        return code, err

    def move_tcp(self, xyz, quat, sec=3.0):
        q = self.ik_world_tcp(xyz, quat)
        code, err = self.move_q(q, sec)
        tcp, _ = self.tcp_world()
        d = np.array(xyz) - tcp
        print(f"  tcp now {tcp.round(4)} target {np.round(xyz,4)} "
              f"delta {d.round(4)}", flush=True)
        return tcp

    def gripper(self, width, timeout=120):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=timeout)
        r = rf.result().result
        self.spin(0.3)
        f = self.fingers()
        print(f"  gripper({width}): reached={r.reached_goal} "
              f"stalled={r.stalled} pos={r.position:.4f} fingers={f}",
              flush=True)
        return f
OPENRUA_EOF

# openrua op 13
cd /workspace; cat > step1.py <<'EOF'
from rob import *
r = Robot()
print("fingers", r.fingers(), "tcp", r.tcp_world())
r.gripper(0.04)
q = down_quat(0)
print("pre-grasp above alphabet soup")
r.move_tcp([-0.115, -0.153, 0.62], q, 3.0)
print("fingers", r.fingers())
EOF
timeout 900 python3 -u step1.py 2>&1 | grep -v XMLPARSER

# openrua op 14
cd /workspace; cat > iktest.py <<'EOF'
from rob import *
r = Robot()
tcp, q = r.tcp_world()
tests = [
 ("current", tcp, q),
 ("target q=(1,0,0,0)", [-0.115,-0.153,0.62], (1,0,0,0)),
 ("target cur quat", [-0.115,-0.153,0.62], q),
 ("target yaw45", [-0.115,-0.153,0.62], down_quat(45)),
 ("target yaw-45", [-0.115,-0.153,0.62], down_quat(-45)),
 ("target higher 0.70", [-0.115,-0.153,0.70], (1,0,0,0)),
 ("shift only y", [-0.059,-0.153,0.674], q),
 ("shift only x", [-0.115,0.0,0.674], q),
 ("shift only z", [-0.059,0.0,0.62], q),
]
for name, p, qq in tests:
    try:
        sol = r.ik_world_tcp(p, qq)
        print(name, "OK", [round(v,3) for v in sol])
    except Exception as e:
        print(name, "FAIL", e)
EOF
timeout 600 python3 -u iktest.py 2>&1 | grep -v XMLPARSER

# openrua op 15
cd /workspace; python3 - <<'EOF'
import re
s = open('rob.py').read()
s = s.replace('        req.ik_request.avoid_collisions = False\n        req.ik_request.timeout = Duration(sec=2)\n', '')
open('rob.py','w').write(s)
EOF
timeout 600 python3 -u iktest.py 2>&1 | grep -v XMLPARSER

# openrua op 16
cd /workspace; timeout 300 python3 -u tools/action/ik_move.py 0.457 0.0 0.358 0.9996 0 -0.0284 0 2 2>&1 | grep -v XMLPARSER

# openrua op 17
cd /workspace; timeout 20 ros2 service list 2>&1 | grep -v XMLPARSER | grep -v parameter; timeout 20 ros2 param get /move_group robot_description_semantic 2>&1 | grep -v XMLPARSER | head -60

# openrua op 18
cd /workspace; cat > fktest.py <<'EOF'
from rob import *
from moveit_msgs.srv import GetPositionFK
r = Robot()
cli = r.node.create_client(GetPositionFK, '/compute_fk')
cli.wait_for_service(timeout_sec=10)
req = GetPositionFK.Request()
req.header.frame_id = ''
req.fk_link_names = ['panda_link8', 'panda_hand']
s = JointState(); s.name = list(ARM); s.position = r.arm_q()
req.robot_state.joint_state = s
fut = cli.call_async(req)
rclpy.spin_until_future_complete(r.node, fut, timeout_sec=60)
res = fut.result()
print('code', res.error_code.val)
for n, ps in zip(res.fk_link_names, res.pose_stamped):
    p, q = ps.pose.position, ps.pose.orientation
    print(n, ps.header.frame_id, round(p.x,4), round(p.y,4), round(p.z,4), '|', round(q.x,4), round(q.y,4), round(q.z,4), round(q.w,4))
print('TF hand world', r.hand_world())
EOF
timeout 300 python3 -u fktest.py 2>&1 | grep -v XMLPARSER

# openrua op 19
cd /workspace; python3 - <<'EOF'
s = open('rob.py').read()
s = s.replace('''        R = quat_to_R(*quat)
        hand = np.array(xyz) - TCP_OFF * R[:, 2]
        base = hand - BASE_IN_WORLD
''', '''        R = quat_to_R(*quat)
        hand = np.array(xyz) - TCP_OFF * R[:, 2]
        # empirically (compute_fk) the planner frame is WORLD here; the IK
        # tip link is panda_link8 = hand rotated by +45 deg about hand z
        base = hand
        quat = quat_mul(quat, (0.0, 0.0, math.sin(math.pi / 8),
                               math.cos(math.pi / 8)))
''')
s = s.replace('''def down_quat(''', '''def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def down_quat(''')
open('rob.py','w').write(s)
EOF
timeout 600 python3 -u iktest.py 2>&1 | grep -v XMLPARSER

# openrua op 20
cd /workspace; cat > step1.py <<'EOF'
from rob import *
r = Robot()
q = down_quat(0)
print("pre-grasp above alphabet soup")
r.move_tcp([-0.115, -0.153, 0.60], q, 3.0)
print("fingers", r.fingers())
EOF
timeout 900 python3 -u step1.py 2>&1 | grep -v XMLPARSER; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand 2>&1 | grep -v XMLPARSER

# openrua op 21
cd /workspace; timeout 60 python3 px_batch.py robot0_eye_in_hand 330,310 300,310 360,310 330,285 330,335 2>&1 | grep -v XMLPARSER

# openrua op 22
cd /workspace; cat > blob.py <<'EOF'
"""Centroid/extent in world of pixels whose world z is within [zlo, zhi] in an ROI."""
import sys, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from rob import quat_to_R
cam = sys.argv[1]; zlo, zhi = float(sys.argv[2]), float(sys.argv[3])
u0,v0,u1,v1 = map(int, sys.argv[4:8]) if len(sys.argv) > 4 else (0,0,640,480)
rclpy.init(); node = rclpy.create_node('blob')
tfbuf = Buffer(); TransformListener(tfbuf, node)
got = {}
node.create_subscription(Image, f'/{cam}/depth/image_raw', lambda m: got.setdefault('d', m), 1)
node.create_subscription(CameraInfo, f'/{cam}/color/camera_info', lambda m: got.setdefault('i', m), 1)
while 'd' not in got or 'i' not in got: rclpy.spin_once(node, timeout_sec=0.2)
d, info = got['d'], got['i']
depth = np.frombuffer(d.data, dtype=np.float32).reshape(d.height, d.width)
fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
frame = f'{cam}_optical_frame'
while not tfbuf.can_transform('world', frame, rclpy.time.Time()): rclpy.spin_once(node, timeout_sec=0.2)
t = tfbuf.lookup_transform('world', frame, rclpy.time.Time())
q = t.transform.rotation
T = np.eye(4); T[:3,:3] = quat_to_R(q.x,q.y,q.z,q.w); T[:3,3] = [t.transform.translation.x,t.transform.translation.y,t.transform.translation.z]
vv, uu = np.mgrid[0:d.height, 0:d.width]
Z = depth
P = np.stack([(uu-cx)*Z/fx, (vv-cy)*Z/fy, Z, np.ones_like(Z)], -1) @ T.T
W = P[..., :3]
m = (W[...,2] >= zlo) & (W[...,2] <= zhi) & np.isfinite(Z)
roi = np.zeros_like(m); roi[v0:v1, u0:u1] = True; m &= roi
pts = W[m]
print('n', len(pts))
if len(pts):
    print('centroid', pts.mean(0).round(4))
    print('min', pts.min(0).round(4), 'max', pts.max(0).round(4))
    ys, xs = np.nonzero(m); print('pixel bbox u', xs.min(), xs.max(), 'v', ys.min(), ys.max())
rclpy.shutdown()
EOF
timeout 60 python3 blob.py robot0_eye_in_hand 0.50 0.52 2>&1 | grep -v XMLPARSER

# openrua op 23
cd /workspace; timeout 60 python3 blob.py robot0_eye_in_hand 0.50 0.52 260 240 410 380 2>&1 | grep -v XMLPARSER

# openrua op 24
cd /workspace; cat > step2.py <<'EOF'
from rob import *
r = Robot()
q = down_quat(0)
print("descend to grasp")
r.move_tcp([-0.111, -0.157, 0.465], q, 2.5)
print("close")
f = r.gripper(0.0)
print("fingers after close", f)
EOF
timeout 900 python3 -u step2.py 2>&1 | grep -v XMLPARSER; timeout 60 python3 tools/perception/cam_snap.py agentview 2>&1 | grep -v XMLPARSER

# openrua op 25
cd /workspace; cat > step3.py <<'EOF'
from rob import *
r = Robot()
q = down_quat(0)
print("lift")
r.move_tcp([-0.111, -0.157, 0.72], q, 2.5)
print("fingers", r.fingers())
print("over basket")
r.move_tcp([-0.0, 0.26, 0.72], q, 3.5)
print("fingers", r.fingers())
print("lower into basket")
r.move_tcp([-0.0, 0.26, 0.61], q, 2.0)
print("open")
r.gripper(0.04)
print("retreat up")
r.move_tcp([-0.0, 0.26, 0.75], q, 2.0)
EOF
timeout 1200 python3 -u step3.py 2>&1 | grep -v XMLPARSER; timeout 60 python3 tools/perception/cam_snap.py agentview 2>&1 | grep -v XMLPARSER

# openrua op 26
cd /workspace; cat > step4.py <<'EOF'
from rob import *
r = Robot()
q = down_quat(0)
print("above butter")
r.move_tcp([0.063, 0.057, 0.60], q, 3.0)
EOF
timeout 900 python3 -u step4.py 2>&1 | grep -v XMLPARSER; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand 2>&1 | grep -v XMLPARSER; timeout 60 python3 blob.py robot0_eye_in_hand 0.44 0.47 2>&1 | grep -v XMLPARSER

# openrua op 27
cd /workspace; timeout 60 python3 blob.py robot0_eye_in_hand 0.44 0.47 270 220 380 360 2>&1 | grep -v XMLPARSER

# openrua op 28
cd /workspace; cat > step5.py <<'EOF'
from rob import *
r = Robot()
q = down_quat(0)
print("pre-grasp")
r.move_tcp([0.068, 0.058, 0.52], q, 2.0)
print("descend")
r.move_tcp([0.068, 0.058, 0.437], q, 2.0)
print("close")
r.gripper(0.0)
print("lift")
r.move_tcp([0.068, 0.058, 0.60], q, 2.5)
print("fingers", r.fingers())
EOF
timeout 1200 python3 -u step5.py 2>&1 | grep -v XMLPARSER; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand 2>&1 | grep -v XMLPARSER

# openrua op 29
cd /workspace; cat > step6.py <<'EOF'
from rob import *
r = Robot()
q = down_quat(0)
print("raise")
r.move_tcp([0.068, 0.058, 0.72], q, 2.0)
print("over basket")
r.move_tcp([0.045, 0.28, 0.72], q, 3.0)
print("fingers", r.fingers())
print("lower")
r.move_tcp([0.045, 0.28, 0.62], q, 2.0)
print("open")
r.gripper(0.04)
print("retreat")
r.move_tcp([0.045, 0.28, 0.78], q, 2.0)
print("home-ish")
r.move_tcp([-0.06, 0.0, 0.75], q, 3.0)
EOF
timeout 1200 python3 -u step6.py 2>&1 | grep -v XMLPARSER; for c in agentview robot0_robotview birdview; do timeout 60 python3 tools/perception/cam_snap.py $c 2>&1 | grep -v XMLPARSER; done

# openrua op 30
cd /workspace; python3 -c "
import cv2
im=cv2.imread('birdview.png')
cv2.imwrite('crop_bird.png', cv2.resize(im[250:330,340:420],None,fx=6,fy=6,interpolation=cv2.INTER_CUBIC))
"; timeout 60 python3 blob.py birdview 0.50 0.60 365 262 400 310 2>&1 | grep -v XMLPARSER; echo "--- old can spot:"; timeout 60 python3 blob.py birdview 0.44 0.60 270 245 300 275 2>&1 | grep -v XMLPARSER; echo "--- old butter spot:"; timeout 60 python3 blob.py birdview 0.44 0.60 322 290 345 310 2>&1 | grep -v XMLPARSER
