#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 2 -type f | head -50

# openrua op 1
cat README.md machine.yaml docs/10-machine.md docs/20-perception.md docs/30-action.md docs/40-patterns.md tools/README.md; ls -la tools .ros

# openrua op 2
ls -la tools/action tools/perception; cat tools/action/*.py tools/perception/*.py

# openrua op 3
ros2 topic list; echo ---; ros2 action list; echo ---; ros2 service list | grep -iv parameter | head -40; echo ---; ros2 node list

# openrua op 4
ros2 topic echo /joint_states --once; for c in agentview frontview birdview sideview robot0_eye_in_hand; do python3 tools/perception/cam_snap.py $c; done

# openrua op 5
timeout 20 ros2 run tf2_ros tf2_echo world birdview_optical_frame 2>&1 | head -8; timeout 20 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -8; ros2 topic echo /birdview/color/camera_info --once | grep -A12 '^k:'

# openrua op 6
mkdir -p "$(dirname /workspace/grab.py)"
cat > /workspace/grab.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Grab color+depth+intrinsics+TF for a camera; save color png, depth npy,
and a meta json with K and world<-optical 4x4 T.

Usage: python3 grab.py <camera>
"""
import json
import sys

import numpy as np
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

import cv2


def grab(node, topic, msg_type, timeout=20.0):
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
    node = rclpy.create_node("grab_" + cam)
    tfbuf = Buffer()
    TransformListener(tfbuf, node)
    br = CvBridge()
    color = br.imgmsg_to_cv2(grab(node, f"/{cam}/color/image_raw", Image), "bgr8")
    depth = br.imgmsg_to_cv2(grab(node, f"/{cam}/depth/image_raw", Image), "passthrough")
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    frame = f"{cam}_optical_frame"
    import time
    end = time.time() + 15
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
        if tfbuf.can_transform("world", frame, rclpy.time.Time()):
            break
    t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    T = np.eye(4)
    T[:3, :3] = quat_R(q.x, q.y, q.z, q.w)
    T[:3, 3] = [t.transform.translation.x, t.transform.translation.y,
                t.transform.translation.z]
    cv2.imwrite(f"{cam}.png", color)
    np.save(f"{cam}_depth.npy", depth.astype(np.float32))
    json.dump({"K": list(info.k), "T": T.tolist(),
               "w": info.width, "h": info.height},
              open(f"{cam}_meta.json", "w"))
    print(f"saved {cam}.png {cam}_depth.npy {cam}_meta.json")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 7
mkdir -p "$(dirname /workspace/p2w.py)"
cat > /workspace/p2w.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Pixel(s) -> world using saved grab.py outputs.
Usage: python3 p2w.py <camera> u v [u v ...]
"""
import json
import sys

import numpy as np

cam = sys.argv[1]
meta = json.load(open(f"{cam}_meta.json"))
depth = np.load(f"{cam}_depth.npy")
K = np.array(meta["K"]).reshape(3, 3)
T = np.array(meta["T"])
fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]


def p2w(u, v):
    z = float(depth[v, u])
    p = np.array([(u - cx) * z / fx, (v - cy) * z / fy, z, 1.0])
    return (T @ p)[:3]


vals = list(map(int, sys.argv[2:]))
for u, v in zip(vals[::2], vals[1::2]):
    w = p2w(u, v)
    print(f"({u},{v}) depth={depth[v,u]:.3f} -> world {w[0]:.3f} {w[1]:.3f} {w[2]:.3f}")
OPENRUA_EOF

# openrua op 8
python3 grab.py birdview && python3 grab.py agentview && python3 -c "
import numpy as np, cv2
d=np.load('birdview_depth.npy'); print(d.shape, np.nanmin(d), np.nanmax(d))
# table height: depth at center-ish table pixel
print('table depth at (320,330):', d[330,320], ' at (200,200):', d[200,200])
# upscale crop of birdview region for inspection
img=cv2.imread('birdview.png'); crop=img[220:320,200:440]; cv2.imwrite('bird_crop.png', cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST))
"

# openrua op 9
python3 p2w.py birdview 279 266 249 291 387 290 325 240 344 281 320 330 && python3 -c "
import numpy as np
d=np.load('birdview_depth.npy')
# print depth patch around white mug and plates
np.set_printoptions(precision=3, linewidth=250)
print(d[258:275:2, 268:292:2])
print('plate L', d[285:298:3, 240:260:3])
"

# openrua op 10
python3 -c "
import numpy as np
d=np.load('birdview_depth.npy')
vals,counts=np.unique(np.round(d,2),return_counts=True)
for v,c in zip(vals,counts):
    if c>200: print(v,c)
print('row 290:', np.round(d[290, 150:500:10],3))
"

# openrua op 11
time python3 tools/action/fjt_send.py 0.0,-0.161037389,0.0,-2.44459747,0.0,2.2267522,0.7853981633974483 1.0 && python3 grab.py birdview && python3 p2w.py birdview 279 266 249 291 387 290 325 240 320 330

# openrua op 12
mkdir -p "$(dirname /workspace/blobs.py)"
cat > /workspace/blobs.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Segment above-table blobs in a top-down grab and report world stats.
Usage: python3 blobs.py <camera> <table_depth> [min_px]
"""
import json
import sys

import numpy as np
import cv2

cam = sys.argv[1]
table = float(sys.argv[2])
min_px = int(sys.argv[3]) if len(sys.argv) > 3 else 30
meta = json.load(open(f"{cam}_meta.json"))
depth = np.load(f"{cam}_depth.npy")
color = cv2.imread(f"{cam}.png")
K = np.array(meta["K"]).reshape(3, 3)
T = np.array(meta["T"])
fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]

mask = ((depth < table - 0.01) & (depth > 0.5)).astype(np.uint8)
n, lab, stats, cents = cv2.connectedComponentsWithStats(mask)
for i in range(1, n):
    if stats[i, cv2.CC_STAT_AREA] < min_px:
        continue
    ys, xs = np.where(lab == i)
    zs = depth[ys, xs]
    # world coords of all pixels
    P = np.stack([(xs - cx) * zs / fx, (ys - cy) * zs / fy, zs, np.ones_like(zs)])
    W = (T @ P)[:3]
    bgr = color[ys, xs].mean(0)
    x0, y0, w, h = stats[i, :4]
    print(f"blob {i}: px area={stats[i,4]} bbox=({x0},{y0},{w},{h}) "
          f"centroid px=({cents[i][0]:.0f},{cents[i][1]:.0f}) "
          f"world x[{W[0].min():.3f},{W[0].max():.3f}] y[{W[1].min():.3f},{W[1].max():.3f}] "
          f"ztop={W[2].max():.3f} zmin={W[2].min():.3f} mean bgr={bgr.round(0)}")
OPENRUA_EOF

# openrua op 13
python3 blobs.py birdview 2.575 20

# openrua op 14
mkdir -p "$(dirname /workspace/rob.py)"
cat > /workspace/rob.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Small persistent control library for the Panda on this machine."""
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
TCP = M["hand"]["tcp_offset_m"]
BASE = np.array([-0.51, 0.0, 0.42])  # world position of panda_link0 (tf2_echo)


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def down_quat(yaw):
    """Hand z pointing down (world -z), fingers opening along a direction
    rotated by `yaw` about world z from world x... quaternion (x,y,z,w).
    Rotation = Rz(yaw) * Rx(pi)."""
    # Rx(pi) = (1,0,0,0); Rz(yaw) = (0,0,sin(yaw/2),cos(yaw/2))
    s, c = math.sin(yaw / 2), math.cos(yaw / 2)
    # q = qz * qx
    qx = (c * 1 + 0, s * 1 * 0 + 0, 0, 0)  # placeholder, do proper mult
    # (w1,x1,y1,z1)=(c,0,0,s) ; (w2,x2,y2,z2)=(0,1,0,0)
    w = c * 0 - 0 * 1 - 0 * 0 - s * 0
    x = c * 1 + 0 * 0 + 0 * 0 - s * 0
    y = c * 0 - 0 * 0 + 0 * 0 + s * 1
    z = c * 0 + 0 * 0 - 0 * 1 + s * 0
    return (x, y, z, w)


class Robot:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("rob_" + str(int(time.time()) % 100000))
        self.js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      self._on_js, 10)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik_cli = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.tw_pub = self.node.create_publisher(TwistStamped, TW["port"], 10)
        self.fjt.wait_for_server(10)
        self.grip.wait_for_server(10)
        self.ik_cli.wait_for_service(10)
        self.fk_cli.wait_for_service(10)
        self.wait_js()

    def _on_js(self, m):
        self.js = dict(zip(m.name, m.position))
        self.js_t = time.time()

    def spin(self, t=0.05):
        rclpy.spin_once(self.node, timeout_sec=t)

    def wait_js(self):
        self.js = {}
        while not self.js:
            self.spin(0.2)
        return self.js

    def arm_q(self):
        self.wait_js()
        return [self.js[j] for j in ARM]

    def fingers(self):
        self.wait_js()
        return self.js["panda_finger_joint1"], self.js["panda_finger_joint2"]

    # ---- FK / IK (base frame = panda_link0; world = base + BASE) ----
    def fk(self, q=None, link="panda_hand"):
        q = self.arm_q() if q is None else q
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {res and res.error_code.val}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE
        q_ = (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        return pos, q_

    def tcp(self, q=None):
        pos, quat = self.fk(q)
        R = quat_R(*quat)
        return pos + TCP * R[:, 2], quat

    def ik(self, pos_world, quat, seed=None, at_tcp=True, timeout=1.0):
        pos = np.array(pos_world, dtype=float)
        if at_tcp:
            R = quat_R(*quat)
            pos = pos - TCP * R[:, 2]
        pos = pos - BASE
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pos)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=int(timeout), nanosec=int((timeout % 1) * 1e9))
        seed = self.arm_q() if seed is None else seed
        req.ik_request.robot_state.joint_state.name = list(ARM)
        req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None:
            raise RuntimeError("IK no answer")
        if res.error_code.val != 1:
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in ARM]

    # ---- motion ----
    def move_q(self, q, seconds=3.0, via=None):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pts = []
        if via:
            n = len(via) + 1
            for i, v in enumerate(via):
                pt = JointTrajectoryPoint(positions=[float(x) for x in v])
                t = seconds * (i + 1) / n
                pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
                pts.append(pt)
        pt = JointTrajectoryPoint(positions=[float(x) for x in q])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=600)
        code = rf.result().result.error_code
        self.wait_js()
        err = max(abs(a - b) for a, b in zip(self.arm_q(), q))
        print(f"move_q: error_code={code} max_joint_err={err:.4f}")
        return code, err

    def move_tcp(self, pos_world, quat, seconds=3.0, seed=None):
        q = self.ik(pos_world, quat, seed=seed)
        if q is None:
            print(f"IK FAILED for {np.round(pos_world,3)}")
            return None
        return self.move_q(q, seconds)

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=120)
        r = rf.result().result
        self.wait_js()
        print(f"gripper({width}): reached={r.reached_goal} stalled={r.stalled} fingers={self.fingers()}")
        return r

    def servo(self, lin=(0, 0, 0), ang=(0, 0, 0), n=20, dt=0.05):
        msg = TwistStamped()
        msg.header.frame_id = TW["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, lin)
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = map(float, ang)
        for _ in range(n):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.tw_pub.publish(msg)
            self.spin(dt)
        self.wait_js()
OPENRUA_EOF

# openrua op 15
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rob.py", "old_string": "    s, c = math.sin(yaw / 2), math.cos(yaw / 2)\\n    # q = qz * qx\\n    qx = (c * 1 + 0, s * 1 * 0 + 0, 0, 0)  # placeholder, do proper mult\\n    # (w1,x1,y1,z1)=(c,0,0,s) ; (w2,x2,y2,z2)=(0,1,0,0)\\n    w = c * 0 - 0 * 1 - 0 * 0 - s * 0\\n    x = c * 1 + 0 * 0 + 0 * 0 - s * 0\\n    y = c * 0 - 0 * 0 + 0 * 0 + s * 1\\n    z = c * 0 + 0 * 0 - 0 * 1 + s * 0\\n    return (x, y, z, w)", "new_string": "    s, c = math.sin(yaw / 2), math.cos(yaw / 2)\\n    # q = qz * qx with qz=(w=c, z=s), qx=(w=0, x=1)  ->  (x=c, y=s, z=0, w=0)\\n    return (c, s, 0.0, 0.0)", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 16
python3 -c "
from rob import *
r=Robot()
print('q', np.round(r.arm_q(),3))
pos,quat=r.fk(); print('hand world', np.round(pos,3), 'quat', np.round(quat,3))
print('R', np.round(quat_R(*quat),3))
print('tcp world', np.round(r.tcp()[0],3))
print('fingers', r.fingers())
"

# openrua op 17
python3 -c "
import numpy as np
from math import cos,sin,pi
def T(a,d,al,th):
    return np.array([[cos(th),-sin(th),0,a],[sin(th)*cos(al),cos(th)*cos(al),-sin(al),-d*sin(al)],[sin(th)*sin(al),cos(th)*sin(al),cos(al),d*cos(al)],[0,0,0,1]])
a=[0,0,0,0.0825,-0.0825,0,0.088,0]; d=[0.333,0,0.316,0,0.384,0,0,0.107]; al=[0,-pi/2,pi/2,pi/2,-pi/2,pi/2,pi/2,0]
q=[0,-0.161037389,0,-2.44459747,0,2.2267522,0.7853981633974483,0]
M=np.eye(4)
for i in range(8): M=M@T(a[i],d[i],al[i],q[i])
print('flange in base', np.round(M[:3,3],3)); print(np.round(M[:3,:3],3))
"


# openrua op 18
sed -i 's/^BASE = np.array(\[-0.51, 0.0, 0.42\]).*/BASE = np.array([0.0, 0.0, 0.0])  # MoveIt FK\/IK poses are already in world coords (verified vs DH FK)/' rob.py && python3 -c "
from rob import *
r=Robot()
pos,quat=r.fk(); print('hand world', np.round(pos,3), 'quat', np.round(quat,3))
tcp,_=r.tcp(); print('tcp world', np.round(tcp,3))
q=r.ik(tcp, quat, at_tcp=True); print('ik of current tcp ->', None if q is None else np.round(q,3))
print('current q', np.round(r.arm_q(),3))
"

# openrua op 19
python3 -c "
from rob import *
r=Robot()
q0=(1.0,0.0,0.0,0.0)
res=r.move_tcp((-0.0875,-0.2155,0.70), q0, seconds=3.0)
print(res)
tcp,quat=r.tcp(); print('tcp', np.round(tcp,3), np.round(quat,3))
" && python3 grab.py birdview && python3 blobs.py birdview 2.575 20

# openrua op 20
python3 -c "
from rob import *
r=Robot()
q0=(1.0,0.0,0.0,0.0)
q=r.ik((-0.0875,-0.2155,0.70), q0)
print('ik', np.round(q,3)); print('now', np.round(r.arm_q(),3))
print(r.move_q(q, 3.0)); print('now', np.round(r.arm_q(),3))
tcp,quat=r.tcp(); print('tcp', np.round(tcp,3), np.round(quat,3))
"

# openrua op 21
python3 -c "
from rob import *
r=Robot()
for l in ['panda_link8','panda_hand','panda_hand_tcp']:
    try:
        p,q=r.fk(link=l); print(l, np.round(p,3), np.round(q,3))
    except Exception as e: print(l, 'ERR', e)
"; ros2 param get /move_group robot_description_semantic 2>/dev/null | grep -A3 -i 'group name="panda_arm"' | head

# openrua op 22
python3 - <<'EOF'
import re
s=open('rob.py').read()
s=s.replace('''def down_quat(yaw):''','''def hand_quat(psi):
    """link8 orientation quaternion for a down-pointing hand whose finger
    axis is rotated psi about world z from world y (psi=0: fingers along y,
    psi=pi/2: fingers along x). panda_hand is link8 yawed by +45 deg."""
    return down_quat(psi - math.pi / 4)


def down_quat(yaw):''')
s=s.replace('def fk(self, q=None, link="panda_hand"):','def fk(self, q=None, link="panda_link8"):')
open('rob.py','w').write(s)
EOF
python3 -c "
import numpy as np, cv2
d=np.load('birdview_depth.npy'); img=cv2.imread('birdview.png')
crop=img[200:320,220:420]; cv2.imwrite('bird_crop.png', cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST))
"

# openrua op 23
python3 - <<'EOF'
import numpy as np, json, cv2
meta=json.load(open('birdview_meta.json')); d=np.load('birdview_depth.npy')
K=np.array(meta['K']).reshape(3,3); T=np.array(meta['T']); fx,fy,cx,cy=K[0,0],K[1,1],K[0,2],K[1,2]
def world(us,vs):
    zs=d[vs,us]; P=np.stack([(us-cx)*zs/fx,(vs-cy)*zs/fy,zs,np.ones_like(zs)]); return (T@P)[:3]
for name,(u0,v0) in {'yellow':(325,242),'red':(344,279)}.items():
    win=d[v0-25:v0+25,u0-25:u0+25]
    mask=(win<2.56).astype(np.uint8)
    n,lab,st,ce=cv2.connectedComponentsWithStats(mask)
    i=lab[25,25]
    if i==0:
        # pick nearest component
        i=max(range(1,n),key=lambda k:st[k,4])
    ys,xs=np.where(lab==i); W=world(xs+u0-25, ys+v0-25)
    # rim pixels: top height
    top=W[2].max()
    rim=W[:, W[2]>top-0.015]
    print(name,'px area',len(xs),'x[%.3f,%.3f] y[%.3f,%.3f] ztop %.3f'%(W[0].min(),W[0].max(),W[1].min(),W[1].max(),top))
    print('   rim x[%.3f,%.3f] y[%.3f,%.3f] rim center (%.3f,%.3f)'%(rim[0].min(),rim[0].max(),rim[1].min(),rim[1].max(),rim[0].mean(),rim[1].mean()))
np.set_printoptions(precision=3,linewidth=250)
print(d[232:254:2, 312:340:2])
EOF

# openrua op 24
python3 - <<'EOF'
import numpy as np, json
meta=json.load(open('birdview_meta.json')); d=np.load('birdview_depth.npy')
K=np.array(meta['K']).reshape(3,3); T=np.array(meta['T']); fx,fy,cx,cy=K[0,0],K[1,1],K[0,2],K[1,2]
def world(us,vs):
    zs=d[vs,us]; P=np.stack([(us-cx)*zs/fx,(vs-cy)*zs/fy,zs,np.ones_like(zs)]); return (T@P)[:3]
for name,(u0,v0,lo,hi) in {'yellow':(325,242,2.455,2.475),'red':(344,279,2.42,2.44),'white_prev':(281,266,2.40,2.42)}.items():
    win=d[v0-16:v0+16,u0-16:u0+16]
    ys,xs=np.where((win>=lo)&(win<=hi))
    W=world(xs+u0-16, ys+v0-16)
    print(name,'rim px',len(xs),'x[%.3f,%.3f] y[%.3f,%.3f] center (%.4f,%.4f) ztop %.3f'%(W[0].min(),W[0].max(),W[1].min(),W[1].max(),(W[0].min()+W[0].max())/2,(W[1].min()+W[1].max())/2,W[2].max()))
EOF

# openrua op 25
python3 -c "
from rob import *
r=Robot()
print(r.move_tcp((-0.0875,-0.2115,0.65), hand_quat(0.0), seconds=2.5))
tcp,quat=r.tcp(); print('tcp', np.round(tcp,3), 'hand', np.round(r.fk(link='panda_hand')[1],3))
" && python3 grab.py robot0_eye_in_hand && python3 grab.py birdview

# openrua op 26
python3 -c "
from rob import *
r=Robot()
q=r.ik((-0.0875,-0.2115,0.65), hand_quat(0.0))
print('ik ', np.round(q,3)); print('now', np.round(r.arm_q(),3))
print(r.move_q(q, 2.0)); print('now', np.round(r.arm_q(),3))
tcp,quat=r.tcp(); print('tcp', np.round(tcp,3), 'hand', np.round(r.fk(link='panda_hand')[1],3))
" && python3 grab.py robot0_eye_in_hand

# openrua op 27
python3 - <<'EOF'
import numpy as np, json
meta=json.load(open('robot0_eye_in_hand_meta.json')); d=np.load('robot0_eye_in_hand_depth.npy')
T=np.array(meta['T']); print('cam T:\n', np.round(T,3))
K=np.array(meta['K']).reshape(3,3); fx,fy,cx,cy=K[0,0],K[1,1],K[0,2],K[1,2]
def world(u,v):
    z=d[v,u]; p=np.array([(u-cx)*z/fx,(v-cy)*z/fy,z,1.0]); return (T@p)[:3]
np.set_printoptions(precision=3,linewidth=200)
print('row 300 depth:', d[300,170:350:8])
for u,v in [(330,300),(183,300),(255,300),(255,240),(255,365),(420,180),(320,420)]:
    print((u,v), 'depth %.3f'%d[v,u], 'world', world(u,v))
EOF

# openrua op 28
cat > rimfit.py <<'EOF'
import numpy as np, json, sys
cam=sys.argv[1]; zlo=float(sys.argv[2]); zhi=float(sys.argv[3])
roi=list(map(int,sys.argv[4:8])) if len(sys.argv)>=8 else None  # u0 v0 u1 v1
meta=json.load(open(f'{cam}_meta.json')); d=np.load(f'{cam}_depth.npy')
T=np.array(meta['T']); K=np.array(meta['K']).reshape(3,3); fx,fy,cx,cy=K[0,0],K[1,1],K[0,2],K[1,2]
vs,us=np.mgrid[0:d.shape[0],0:d.shape[1]]
zs=d
P=np.stack([(us-cx)*zs/fx,(vs-cy)*zs/fy,zs,np.ones_like(zs)]).reshape(4,-1)
W=(T@P)[:3].reshape(3,*d.shape)
m=(W[2]>zlo)&(W[2]<zhi)&np.isfinite(d)&(d>0.01)
if roi: 
    rm=np.zeros_like(m); rm[roi[1]:roi[3],roi[0]:roi[2]]=True; m&=rm
x=W[0][m]; y=W[1][m]
print('n',m.sum(),'x[%.3f,%.3f] y[%.3f,%.3f] zmean %.3f'%(x.min(),x.max(),y.min(),y.max(),W[2][m].mean()))
# algebraic circle fit
A=np.stack([x,y,np.ones_like(x)],1); b=x**2+y**2
c=np.linalg.lstsq(A,b,rcond=None)[0]; xc,yc=c[0]/2,c[1]/2; R=np.sqrt(c[2]+xc**2+yc**2)
print('circle center (%.4f,%.4f) R=%.4f'%(xc,yc,R))
EOF
python3 rimfit.py robot0_eye_in_hand 0.53 0.56 150 200 400 400

# openrua op 29
cat >> rob.py <<'EOF'


def go(r, pos, psi, seconds=2.5, tries=3, tol=0.02):
    """IK + trajectory to TCP pos with finger axis psi; resend on lag."""
    q = r.ik(pos, hand_quat(psi))
    if q is None:
        print(f"IK FAILED for {np.round(pos, 3)} psi={psi}")
        return False
    for i in range(tries):
        code, err = r.move_q(q, seconds)
        if err < tol:
            break
    tcp, _ = r.tcp()
    print(f"go -> tcp {np.round(tcp, 4)} (target {np.round(pos, 4)}) err={np.linalg.norm(tcp - np.array(pos)):.4f}")
    return err < tol
EOF
python3 -c "
from rob import *
r=Robot()
go(r, (-0.0825,-0.213,0.52), 0.0, 2.5)
print('fingers', r.fingers())
" && python3 grab.py robot0_eye_in_hand && python3 grab.py agentview

# openrua op 30
python3 -c "
import cv2, numpy as np
a=cv2.imread('agentview.png'); e=cv2.imread('robot0_eye_in_hand.png')
cv2.imwrite('pair.png', np.hstack([a,e]))
"

# openrua op 31
python3 -c "
from rob import *
r=Robot()
r.gripper(0.0)
import time
for i in range(3):
    r.spin(0.3); print('fingers', r.fingers())
" && python3 grab.py robot0_eye_in_hand

# openrua op 32
python3 -c "
from rob import *
r=Robot()
go(r, (-0.0825,-0.213,0.72), 0.0, 2.5)
print('fingers', r.fingers())
" && python3 grab.py agentview && python3 grab.py birdview && python3 blobs.py birdview 2.575 20

# openrua op 33
python3 rimfit.py birdview 0.69 0.74 250 240 330 300; python3 rimfit.py birdview 0.70 0.735 250 240 330 300

# openrua op 34
python3 rimfit.py agentview 0.60 0.70 100 60 300 200

# openrua op 35
python3 - <<'EOF'
import numpy as np, json
cam='agentview'
meta=json.load(open(f'{cam}_meta.json')); d=np.load(f'{cam}_depth.npy')
T=np.array(meta['T']); K=np.array(meta['K']).reshape(3,3); fx,fy,cx,cy=K[0,0],K[1,1],K[0,2],K[1,2]
vs,us=np.mgrid[0:d.shape[0],0:d.shape[1]]
P=np.stack([(us-cx)*d/fx,(vs-cy)*d/fy,d,np.ones_like(d)]).reshape(4,-1)
W=(T@P)[:3].reshape(3,*d.shape)
for zlo,zhi in [(0.60,0.65),(0.65,0.70),(0.70,0.73)]:
    m=(W[2]>zlo)&(W[2]<zhi)&(W[0]>-0.15)&(W[1]<0)&(W[1]>-0.4)
    x=W[0][m]; y=W[1][m]
    print(zlo,zhi,'n',m.sum(),'x[%.3f,%.3f] y[%.3f,%.3f]'%(x.min(),x.max(),y.min(),y.max()))
    # front-most surface point (max x) around each y bin
    ys=np.round(y,3); 
    # sample: for y bins, max x
    bins=np.arange(-0.24,-0.02,0.01)
    prof=[(b, x[(y>=b)&(y<b+0.01)].max() if ((y>=b)&(y<b+0.01)).any() else None) for b in bins]
    print('  ', [(round(b,2), None if v is None else round(v,3)) for b,v in prof])
EOF

# openrua op 36
python3 - <<'EOF'
import numpy as np, json
cam='agentview'
meta=json.load(open(f'{cam}_meta.json')); d=np.load(f'{cam}_depth.npy')
T=np.array(meta['T']); K=np.array(meta['K']).reshape(3,3); fx,fy,cx,cy=K[0,0],K[1,1],K[0,2],K[1,2]
vs,us=np.mgrid[0:d.shape[0],0:d.shape[1]]
P=np.stack([(us-cx)*d/fx,(vs-cy)*d/fy,d,np.ones_like(d)]).reshape(4,-1)
W=(T@P)[:3].reshape(3,*d.shape)
m=(W[2]>0.5)&(W[2]<0.66)&(W[0]>-0.15)&(W[0]<0.0)&(W[1]<-0.1)&(W[1]>-0.25)
print('mug lowest z %.3f, highest z overall for these px'%(W[2][m].min()), W[2][m].max())
m2=(W[2]>0.73)&(W[2]<0.80)&(W[0]>-0.15)&(W[0]<0.0)&(W[1]<-0.1)&(W[1]>-0.25)
print('rim-ish z range', W[2][m2].min() if m2.any() else None, W[2][m2].max() if m2.any() else None)
EOF

# openrua op 37
python3 -c "
from rob import *
r=Robot()
go(r, (0.015,-0.349,0.72), 0.0, 2.5)
print('fingers', r.fingers())
go(r, (0.015,-0.349,0.55), 0.0, 2.5)
print('fingers', r.fingers())
" && python3 grab.py agentview && python3 grab.py robot0_eye_in_hand && python3 -c "
import cv2, numpy as np
a=cv2.imread('agentview.png'); e=cv2.imread('robot0_eye_in_hand.png')
cv2.imwrite('pair.png', np.hstack([a,e]))
"

# openrua op 38
python3 rimfit.py robot0_eye_in_hand 0.440 0.462; python3 rimfit.py robot0_eye_in_hand 0.56 0.60

# openrua op 39
python3 -c "
from rob import *
r=Robot()
r.gripper(0.04)
go(r, (0.015,-0.349,0.72), 0.0, 2.5)
" && python3 grab.py birdview && python3 blobs.py birdview 2.575 20 && python3 grab.py agentview

# openrua op 40
python3 rimfit.py birdview 0.555 0.58 200 240 300 330; python3 -c "
import cv2; img=cv2.imread('birdview.png'); cv2.imwrite('bird_crop.png', cv2.resize(img[240:330,200:300],None,fx=5,fy=5,interpolation=cv2.INTER_NEAREST))"

# openrua op 41
python3 -c "
from rob import *
r=Robot()
go(r, (-0.189,0.0655,0.72), 0.0, 3.0)
go(r, (-0.189,0.0655,0.65), 0.0, 2.0)
" && python3 grab.py robot0_eye_in_hand && python3 rimfit.py robot0_eye_in_hand 0.525 0.55 && python3 grab.py birdview

# openrua op 42
cat > rimfit2.py <<'EOF'
import numpy as np, json, sys
cam=sys.argv[1]; zlo,zhi,xlo,xhi,ylo,yhi=map(float,sys.argv[2:8])
meta=json.load(open(f'{cam}_meta.json')); d=np.load(f'{cam}_depth.npy')
T=np.array(meta['T']); K=np.array(meta['K']).reshape(3,3); fx,fy,cx,cy=K[0,0],K[1,1],K[0,2],K[1,2]
vs,us=np.mgrid[0:d.shape[0],0:d.shape[1]]
P=np.stack([(us-cx)*d/fx,(vs-cy)*d/fy,d,np.ones_like(d)]).reshape(4,-1)
W=(T@P)[:3].reshape(3,*d.shape)
m=(W[2]>zlo)&(W[2]<zhi)&(W[0]>xlo)&(W[0]<xhi)&(W[1]>ylo)&(W[1]<yhi)&np.isfinite(d)&(d>0.01)
x=W[0][m]; y=W[1][m]
print('n',m.sum(),'x[%.4f,%.4f] y[%.4f,%.4f] z[%.3f,%.3f] bbox center (%.4f,%.4f)'%(x.min(),x.max(),y.min(),y.max(),W[2][m].min(),W[2][m].max(),(x.min()+x.max())/2,(y.min()+y.max())/2))
# outermost points: convex-ish: take top-most z pixels only (rim)
top=W[2][m].max(); mm=m&(W[2]>top-0.008); x2=W[0][mm]; y2=W[1][mm]
print('rim-top n',mm.sum(),'x[%.4f,%.4f] y[%.4f,%.4f] center (%.4f,%.4f)'%(x2.min(),x2.max(),y2.min(),y2.max(),(x2.min()+x2.max())/2,(y2.min()+y2.max())/2))
EOF
python3 rimfit2.py robot0_eye_in_hand 0.50 0.56 -0.30 -0.10 -0.15 0.15

# openrua op 43
python3 -c "
from rob import *
r=Robot()
go(r, (-0.1856,0.072,0.512), 0.0, 2.5)
" && python3 grab.py robot0_eye_in_hand

# openrua op 44
python3 -c "
from rob import *
r=Robot()
r.gripper(0.0)
go(r, (-0.1856,0.072,0.72), 0.0, 2.5)
print('fingers', r.fingers())
" && python3 grab.py agentview

# openrua op 45
python3 - <<'EOF'
import numpy as np, json
cam='agentview'
meta=json.load(open(f'{cam}_meta.json')); d=np.load(f'{cam}_depth.npy')
T=np.array(meta['T']); K=np.array(meta['K']).reshape(3,3); fx,fy,cx,cy=K[0,0],K[1,1],K[0,2],K[1,2]
vs,us=np.mgrid[0:d.shape[0],0:d.shape[1]]
P=np.stack([(us-cx)*d/fx,(vs-cy)*d/fy,d,np.ones_like(d)]).reshape(4,-1)
W=(T@P)[:3].reshape(3,*d.shape)
for zlo,zhi in [(0.62,0.66),(0.66,0.70),(0.70,0.74)]:
    m=(W[2]>zlo)&(W[2]<zhi)&(W[0]>-0.30)&(W[0]<-0.05)&(W[1]<0.15)&(W[1]>-0.10)
    x=W[0][m]; y=W[1][m]
    print(zlo,zhi,'n',m.sum(),'x[%.3f,%.3f] y[%.3f,%.3f]'%(x.min(),x.max(),y.min(),y.max()))
    bins=np.arange(-0.10,0.15,0.01)
    prof=[(round(b,2), round(x[(y>=b)&(y<b+0.01)].max(),3) if ((y>=b)&(y<b+0.01)).any() else None) for b in bins]
    print('  ', prof)
m=(W[2]>0.5)&(W[2]<0.66)&(W[0]>-0.30)&(W[0]<-0.05)&(W[1]<0.15)&(W[1]>-0.10)
print('lowest z', W[2][m].min())
EOF

# openrua op 46
python3 -c "
from rob import *
r=Robot()
go(r, (0.015,0.330,0.72), 0.0, 3.0)
print('fingers', r.fingers())
go(r, (0.015,0.330,0.55), 0.0, 2.5)
print('fingers', r.fingers())
" && python3 grab.py agentview && python3 grab.py robot0_eye_in_hand && python3 -c "
import cv2, numpy as np
a=cv2.imread('agentview.png'); e=cv2.imread('robot0_eye_in_hand.png')
cv2.imwrite('pair.png', np.hstack([a,e]))
"

# openrua op 47
python3 -c "
from rob import *
r=Robot()
r.gripper(0.04)
go(r, (0.015,0.330,0.75), 0.0, 2.5)
# move arm back toward a neutral pose out of the way of the birdview plates
go(r, (-0.25,0.0,0.75), 0.0, 3.0)
" && python3 grab.py birdview && python3 grab.py agentview && python3 blobs.py birdview 2.575 20

# openrua op 48
python3 -c "
import cv2, numpy as np
a=cv2.imread('agentview.png'); b=cv2.imread('birdview.png')
crop=cv2.resize(b[240:330,200:440],None,fx=2,fy=2,interpolation=cv2.INTER_NEAREST)
cv2.imwrite('pair.png', np.hstack([a, cv2.resize(crop,(640,480))]))
"; python3 rimfit2.py birdview 0.55 0.575 -0.10 0.10 -0.40 -0.20; python3 rimfit2.py birdview 0.545 0.565 -0.10 0.10 0.20 0.40; python3 rimfit2.py birdview 0.44 0.46 -0.10 0.10 -0.40 -0.20; python3 rimfit2.py birdview 0.44 0.46 -0.10 0.10 0.20 0.40
