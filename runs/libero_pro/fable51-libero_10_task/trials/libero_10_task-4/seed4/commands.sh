#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 2 -type f | head -50

# openrua op 1
cat README.md machine.yaml docs/10-machine.md docs/20-perception.md docs/30-action.md docs/40-patterns.md tools/README.md; ls -la .ros tools

# openrua op 2
ls -la tools/action tools/perception; cat tools/action/*.py tools/perception/*.py

# openrua op 3
ros2 topic list && echo --- && ros2 action list && echo --- && ros2 service list | grep -iE "ik|fk|plan|scene" && echo --- && ros2 node list

# openrua op 4
for c in agentview birdview frontview sideview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c; done; ros2 topic echo /joint_states --once

# openrua op 5
timeout 20 ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id" | sort -u; echo ---; timeout 30 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -8; echo ---; timeout 30 ros2 run tf2_ros tf2_echo world birdview_optical_frame 2>&1 | head -8

# openrua op 6
mkdir -p "$(dirname /workspace/locate.py)"
cat > /workspace/locate.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Locate mugs/plates from the birdview camera: colour segmentation +
depth + intrinsics + TF -> world coordinates. Prints one line per blob."""
import sys
import numpy as np, rclpy, cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

CAM = sys.argv[1] if len(sys.argv) > 1 else "birdview"


def grab(node, topic, T):
    got = {}
    sub = node.create_subscription(T, topic, lambda m: got.setdefault("m", m), 1)
    while "m" not in got:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    return got["m"]


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


rclpy.init()
node = rclpy.create_node("locate")
tfbuf = Buffer(); TransformListener(tfbuf, node)
br = CvBridge()
color = br.imgmsg_to_cv2(grab(node, f"/{CAM}/color/image_raw", Image), "bgr8")
depth = br.imgmsg_to_cv2(grab(node, f"/{CAM}/depth/image_raw", Image), "passthrough").astype(float)
info = grab(node, f"/{CAM}/color/camera_info", CameraInfo)
frame = f"{CAM}_optical_frame"
while not tfbuf.can_transform("world", frame, rclpy.time.Time()):
    rclpy.spin_once(node, timeout_sec=0.2)
t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
q = t.transform.rotation
R = quat_R(q.x, q.y, q.z, q.w)
tr = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
H, W = depth.shape
vv, uu = np.mgrid[0:H, 0:W]
pc = np.stack([(uu - cx) * depth / fx, (vv - cy) * depth / fy, depth], -1)
pw = pc @ R.T + tr  # world xyz per pixel
np.save(f"{CAM}_world.npy", pw)
cv2.imwrite(f"{CAM}_color.png", color)

# table height estimate: the mode of z over the table region
zs = pw[..., 2]
finite = np.isfinite(zs)
hist, edges = np.histogram(zs[finite], bins=400)
table_z = edges[np.argmax(hist)]
print(f"table_z ~ {table_z:.3f}")

# anything above the table by >8mm and not the robot: connected blobs
above = finite & (zs > table_z + 0.008)
above = above.astype(np.uint8)
n, lab, stats, cents = cv2.connectedComponentsWithStats(above)
for i in range(1, n):
    x, y, w, h, a = stats[i]
    if a < 30:
        continue
    m = lab == i
    P = pw[m]
    c = color[m].mean(0)[::-1]  # rgb
    print(f"blob {i}: px({cents[i][0]:.0f},{cents[i][1]:.0f}) area={a} "
          f"world xy=({P[:,0].mean():.3f},{P[:,1].mean():.3f}) "
          f"zmax={P[:,2].max():.3f} zmin={P[:,2].min():.3f} rgb={c.astype(int)}")
rclpy.shutdown()
OPENRUA_EOF

# openrua op 7
timeout 120 python3 locate.py birdview

# openrua op 8
python3 -c "
import numpy as np
pw=np.load('birdview_world.npy'); z=pw[...,2]
f=np.isfinite(z)&(z>0.2)&(z<0.6)
h,e=np.histogram(z[f],bins=80); 
for hh,ee in zip(h,e):
    if hh>200: print(f'{ee:.3f} {hh}')
"

# openrua op 9
python3 - <<'EOF'
import numpy as np, cv2
pw=np.load('birdview_world.npy'); color=cv2.imread('birdview_color.png')
z=pw[...,2]; table=0.4255
above=(np.isfinite(z)&(z>table+0.006)).astype(np.uint8)
n,lab,stats,cents=cv2.connectedComponentsWithStats(above)
for i in range(1,n):
    x,y,w,h,a=stats[i]
    if a<20: continue
    m=lab==i; P=pw[m]; c=color[m].mean(0)[::-1]
    print(f"blob {i}: px({cents[i][0]:.0f},{cents[i][1]:.0f}) bbox={w}x{h} area={a} xy=({P[:,0].mean():.3f},{P[:,1].mean():.3f}) zmax={P[:,2].max():.3f} zmin={P[:,2].min():.3f} rgb={c.astype(int)}")
EOF

# openrua op 10
python3 - <<'EOF'
import numpy as np, cv2
pw=np.load('birdview_world.npy'); color=cv2.imread('birdview_color.png')
crop=color[220:320,200:440]; cv2.imwrite('bird_crop.png', cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_NEAREST))
hsv=cv2.cvtColor(color,cv2.COLOR_BGR2HSV)
z=pw[...,2]
def report(name,mask):
    mask=mask&np.isfinite(z)&(z>0.44)
    n,lab,stats,cents=cv2.connectedComponentsWithStats(mask.astype(np.uint8))
    for i in range(1,n):
        if stats[i][4]<15: continue
        m=lab==i;P=pw[m]
        print(f"{name} blob px({cents[i][0]:.0f},{cents[i][1]:.0f}) area={stats[i][4]} xy=({P[:,0].mean():.3f},{P[:,1].mean():.3f}) zmax={P[:,2].max():.3f} zmin={P[:,2].min():.3f}")
# yellow: hue ~20-35 (opencv 0-180), sat high
report('yellow',(hsv[...,0]>18)&(hsv[...,0]<38)&(hsv[...,1]>90)&(hsv[...,2]>90))
# red: hue <8 or >170, sat high
report('red',((hsv[...,0]<8)|(hsv[...,0]>170))&(hsv[...,1]>110)&(hsv[...,2]>60))
EOF

# openrua op 11
timeout 120 python3 locate.py agentview 2>&1 | head -3; python3 - <<'EOF'
import numpy as np, cv2
pw=np.load('agentview_world.npy'); color=cv2.imread('agentview_color.png')
hsv=cv2.cvtColor(color,cv2.COLOR_BGR2HSV)
z=pw[...,2]
def report(name,mask):
    mask=mask&np.isfinite(z)&(z>0.43)
    n,lab,stats,cents=cv2.connectedComponentsWithStats(mask.astype(np.uint8))
    for i in range(1,n):
        if stats[i][4]<40: continue
        m=lab==i;P=pw[m]
        print(f"{name} blob px({cents[i][0]:.0f},{cents[i][1]:.0f}) area={stats[i][4]} x[{P[:,0].min():.3f},{P[:,0].max():.3f}] y[{P[:,1].min():.3f},{P[:,1].max():.3f}] z[{P[:,2].min():.3f},{P[:,2].max():.3f}]")
report('yellow',(hsv[...,0]>18)&(hsv[...,0]<38)&(hsv[...,1]>90)&(hsv[...,2]>90))
report('red',((hsv[...,0]<8)|(hsv[...,0]>170))&(hsv[...,1]>110)&(hsv[...,2]>60))
EOF

# openrua op 12
python3 - <<'EOF'
import numpy as np
pw=np.load('birdview_world.npy')
for name,(u,v) in {'left plate ctr':(253,285),'right plate ctr':(388,288),'table below left plate':(253,312),'table left of plate':(215,285),'table center-ish':(329,330),'table far left':(160,230),'white mug top':(272,265)}.items():
    p=pw[v,u]; print(f"{name:25s} px({u},{v}) -> x={p[0]:.3f} y={p[1]:.3f} z={p[2]:.3f}")
EOF

# openrua op 13
timeout 300 python3 tools/action/gripper_cmd.py 0.04; ros2 topic echo /joint_states --once | grep -A9 position | tail -2

# openrua op 14
timeout 120 python3 locate.py birdview >/dev/null; python3 - <<'EOF'
import numpy as np
pw=np.load('birdview_world.npy')
for name,(u,v) in {'left plate ctr':(253,285),'right plate ctr':(388,288),'table':(253,312),'white mug inside':(272,265)}.items():
    p=pw[v,u]; print(f"{name:25s} -> x={p[0]:.3f} y={p[1]:.3f} z={p[2]:.3f}")
EOF

# openrua op 15
mkdir -p "$(dirname /workspace/robot.py)"
cat > /workspace/robot.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Reusable Panda helper: one node, clients built once.
Frames: MoveIt plans in panda_link0; world->panda_link0 = (-0.51,0,0.42).
"""
import math, time
import numpy as np, rclpy, yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

BASE_IN_WORLD = np.array([-0.51, 0.0, 0.42])
TCP = 0.1034


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def down_quat(yaw):
    """Hand pointing straight down (+Z hand = -Z world), fingers rotated by yaw
    about world Z. Panda's canonical top-down grasp is q=(1,0,0,0) rotated."""
    # R = Rz(yaw) @ Rx(pi): quaternion product
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    # qz(yaw) = (0,0,sy,cy); qx(pi) = (1,0,0,0); product qz*qx:
    return (cy, sy, 0.0, 0.0)  # (x,y,z,w) -> x=cy, y=sy, z=0, w=0


class Robot:
    def __init__(self):
        m = yaml.safe_load(open("/workspace/machine.yaml"))
        self.m = m
        self.traj = next(a for a in m["actuators"] if a["kind"] == "joint_trajectory")
        self.grip = next(a for a in m["actuators"] if a["kind"] == "gripper")
        self.twist = next(a for a in m["actuators"] if a["kind"] == "cartesian_twist")
        self.joints = self.traj["joints"]
        rclpy.init()
        self.node = rclpy.create_node("robot_helper")
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, self.traj["port"])
        self.gc = ActionClient(self.node, GripperCommand, self.grip["port"])
        self.ik = self.node.create_client(GetPositionIK, m["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.tw_pub = self.node.create_publisher(TwistStamped, self.twist["port"], 10)
        assert self.fjt.wait_for_server(10) and self.gc.wait_for_server(10)
        assert self.ik.wait_for_service(10) and self.fk.wait_for_service(10)
        while not self._js:
            rclpy.spin_once(self.node, timeout_sec=0.2)

    def _on_js(self, msg):
        self._js.update(zip(msg.name, msg.position))

    def spin(self, t=0.1):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joint_state(self, fresh=True):
        if fresh:
            for _ in range(3):
                self.spin(0.1)
        return dict(self._js)

    def arm_q(self):
        js = self.joint_state()
        return [js[j] for j in self.joints]

    def finger_gap(self):
        js = self.joint_state()
        return js["panda_finger_joint1"] - js["panda_finger_joint2"]

    def _seed(self, q=None):
        s = JointState()
        s.name = list(self.joints)
        s.position = [float(v) for v in (q if q is not None else self.arm_q())]
        return s

    def fk_world(self, q=None):
        """hand pose in world: (xyz, quat xyzw)."""
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state = self._seed(q)
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        r = fut.result()
        assert r is not None and r.error_code.val == 1, f"FK failed {r}"
        p = r.pose_stamped[0].pose
        xyz = np.array([p.position.x, p.position.y, p.position.z]) + BASE_IN_WORLD
        return xyz, (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)

    def ik_world(self, xyz, quat, seed=None, at_tcp=True):
        """joint solution for hand (or TCP) at world xyz with quat (xyzw)."""
        xyz = np.asarray(xyz, float)
        if at_tcp:
            R = quat_R(*quat)
            xyz = xyz - TCP * R[:, 2]
        xyz = xyz - BASE_IN_WORLD
        req = GetPositionIK.Request()
        req.ik_request.group_name = self.m["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, xyz)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.robot_state.joint_state = self._seed(seed)
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=2)
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            return None
        sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
        return [sol[j] for j in self.joints]

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
        rclpy.spin_until_future_complete(self.node, send)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res)
        code = res.result().result.error_code
        err = max(abs(a - b) for a, b in zip(self.arm_q(), q))
        return code, err

    def move_world(self, xyz, quat, seconds=3.0, seed=None, at_tcp=True):
        q = self.ik_world(xyz, quat, seed=seed, at_tcp=at_tcp)
        if q is None:
            return None
        code, err = self.move_q(q, seconds)
        return q, code, err

    def gripper(self, width):
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = float(self.grip["max_effort"])
        fut = self.gc.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=300)
        r = res.result().result
        return r.reached_goal, r.stalled, self.finger_gap()

    def servo(self, v, n_ticks, dt=0.05):
        msg = TwistStamped()
        msg.header.frame_id = self.twist["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, v)
        for _ in range(n_ticks):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.tw_pub.publish(msg)
            self.spin(dt)

    def tcp_world(self, q=None):
        xyz, quat = self.fk_world(q)
        R = quat_R(*quat)
        return xyz + TCP * R[:, 2], quat

    def shutdown(self):
        self.node.destroy_node()
        rclpy.shutdown()
OPENRUA_EOF

# openrua op 16
timeout 120 python3 -c "
from robot import *
r=Robot()
print('q', [round(v,3) for v in r.arm_q()])
xyz,quat=r.fk_world(); print('hand world', xyz.round(3), 'quat', [round(v,3) for v in quat])
t,_=r.tcp_world(); print('tcp world', t.round(3))
print('gap', round(r.finger_gap(),4))
# IK test: TCP above the yellow-mug area, straight down
for yaw in (0.0, math.pi/4, -math.pi/4, math.pi/2):
    q=r.ik_world([-0.10,0.05,0.60], down_quat(yaw)); print('ik yaw',round(yaw,2), None if q is None else [round(v,2) for v in q])
r.shutdown()
"

# openrua op 17
timeout 60 python3 tools/perception/cam_snap.py agentview agentview2.png; timeout 120 python3 -c "
from robot import *
r=Robot()
xyz,quat=r.fk_world([0,0,0,-1.5,0,1.5,0]); print('fk test pose', xyz.round(3), [round(v,3) for v in quat])
xyz,quat=r.fk_world(); print('fk current', xyz.round(3))
r.shutdown()
"

# openrua op 18
python3 - <<'EOF'
import numpy as np
pw=np.load('birdview_world.npy')
for name,(u,v) in {'robot base top':(320,140),'link near base':(320,175),'gripper body':(345,275),'gripper cam':(360,292),'red mug':(309,240)}.items():
    p=pw[v,u]; print(f"{name:20s} -> x={p[0]:.3f} y={p[1]:.3f} z={p[2]:.3f}")
EOF
timeout 60 ros2 run tf2_ros tf2_echo panda_link0 panda_hand 2>&1 | grep -A2 "At time" | head -3

# openrua op 19
timeout 120 python3 -c "
from robot import *
import robot
r=Robot()
robot.BASE_IN_WORLD=np.zeros(3)  # test raw
q0=r.arm_q()
xyz,quat=r.fk_world(); print('fk raw', xyz.round(3))
q=r.ik_world(xyz,quat,at_tcp=False); print('ik at raw fk pose:', None if q is None else [round(v,2) for v in q])
q=r.ik_world(xyz-np.array([-0.51,0,0.42]),quat,at_tcp=False); print('ik at fk pose minus base offset:', None if q is None else [round(v,2) for v in q])
q=r.ik_world([-0.10,0.05,0.65], down_quat(0), at_tcp=True); print('ik world target above yellow', None if q is None else [round(v,2) for v in q])
if q: print('fk of that', r.fk_world(q)[0].round(3), r.tcp_world(q)[0].round(3))
r.shutdown()
"

# openrua op 20
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/robot.py", "old_string": "BASE_IN_WORLD = np.array([-0.51, 0.0, 0.42])", "new_string": "# Measured: /compute_fk and /compute_ik on this machine already work in\\n# WORLD coordinates (FK of current q matches TF world->hand), so no offset.\\nBASE_IN_WORLD = np.array([0.0, 0.0, 0.0])", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 21
timeout 600 python3 -c "
from robot import *
r=Robot()
res=r.move_world([-0.35,0.0,0.95], down_quat(0), seconds=3.0)
print('move', None if res is None else (res[1], round(res[2],4)))
print('tcp now', r.tcp_world()[0].round(3))
r.shutdown()
" && timeout 120 python3 locate.py birdview

# openrua op 22
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/robot.py", "old_string": "        code = res.result().result.error_code\\n        err = max(abs(a - b) for a, b in zip(self.arm_q(), q))\\n        return code, err", "new_string": "        code = res.result().result.error_code\\n        err = max(abs(a - b) for a, b in zip(self.arm_q(), q))\\n        # controller lag shows up as -5 with a residual; resend converges\\n        if err > tol and retries > 0:\\n            print(f\\"  [move_q] code={code} residual={err:.3f}, resending\\")\\n            return self.move_q(q, seconds, retries=retries - 1, tol=tol)\\n        return code, err", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 23
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/robot.py", "old_string": "    def move_q(self, q, seconds=3.0, waypoints=None):", "new_string": "    def move_q(self, q, seconds=3.0, waypoints=None, retries=3, tol=0.02):", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 24
timeout 900 python3 -c "
from robot import *
r=Robot()
res=r.move_world([-0.35,0.0,0.95], down_quat(0), seconds=4.0)
print('move', None if res is None else (res[1], round(res[2],4)))
print('tcp now', r.tcp_world()[0].round(3))
r.shutdown()
" && timeout 120 python3 locate.py birdview

# openrua op 25
python3 - <<'EOF'
import numpy as np, cv2
pw=np.load('birdview_world.npy'); color=cv2.imread('birdview_color.png')
crop=color[220:320,200:440]; cv2.imwrite('bird_crop.png', cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_NEAREST))
z=pw[...,2]; table=0.4255
# objects: above table, but below 0.62 (excludes robot which is high now)
above=(np.isfinite(z)&(z>table+0.006)&(z<0.62)&(pw[...,0]>-0.35)).astype(np.uint8)
n,lab,stats,cents=cv2.connectedComponentsWithStats(above)
for i in range(1,n):
    x,y,w,h,a=stats[i]
    if a<20: continue
    m=lab==i; P=pw[m]; c=color[m].mean(0)[::-1]
    print(f"blob {i}: px({cents[i][0]:.0f},{cents[i][1]:.0f}) bbox={w}x{h} area={a} xy=({P[:,0].mean():.3f},{P[:,1].mean():.3f}) x[{P[:,0].min():.3f},{P[:,0].max():.3f}] y[{P[:,1].min():.3f},{P[:,1].max():.3f}] z[{P[:,2].min():.3f},{P[:,2].max():.3f}] rgb={c.astype(int)}")
EOF

# openrua op 26
python3 - <<'EOF'
import numpy as np, cv2
pw=np.load('birdview_world.npy'); color=cv2.imread('birdview_color.png')
z=pw[...,2]
def blobs(mask,name,minarea=15):
    n,lab,stats,cents=cv2.connectedComponentsWithStats(mask.astype(np.uint8))
    out=[]
    for i in range(1,n):
        if stats[i][4]<minarea: continue
        m=lab==i;P=pw[m]
        # circle fit via bounding box of xy
        cx=(P[:,0].min()+P[:,0].max())/2; cy=(P[:,1].min()+P[:,1].max())/2
        print(f"{name}: area={stats[i][4]} mean=({P[:,0].mean():.3f},{P[:,1].mean():.3f}) bboxctr=({cx:.3f},{cy:.3f}) size=({P[:,0].max()-P[:,0].min():.3f},{P[:,1].max()-P[:,1].min():.3f}) ztop={P[:,2].max():.3f}")
        out.append((cx,cy))
    return out
sel=np.isfinite(z)&(pw[...,0]>-0.35)
print("== mug rims (z>0.50)"); blobs(sel&(z>0.50)&(z<0.62),'rim')
print("== plates (0.435<z<0.47, exclude mugs)"); blobs(sel&(z>0.435)&(z<0.47),'plate',minarea=200)
EOF

# openrua op 27
timeout 600 python3 -c "
from robot import *
r=Robot()
R_=0.045
white=(-0.084,-0.186); yellow=(-0.066,0.092); lplate=(-0.011,-0.290); rplate=(0.011,0.290)
for yaw in (math.pi/2, -math.pi/2, 0):
    q=down_quat(yaw); R=quat_R(*q); print('yaw',round(yaw,2),'hand y (finger axis) in world:',R[:,1].round(2),'hand z:',R[:,2].round(2))
q0=r.arm_q()
for name,(cx,cy),ztop in [('white',white,0.549),('yellow',yellow,0.542)]:
    for yaw in (math.pi/2,-math.pi/2):
        pre=r.ik_world([cx-R_,cy,0.70],down_quat(yaw),seed=q0)
        g=r.ik_world([cx-R_,cy,ztop-0.025],down_quat(yaw),seed=pre)
        print(name,'yaw',round(yaw,2),'pre',None if pre is None else [round(v,2) for v in pre],'grasp',None if g is None else [round(v,2) for v in g])
for name,(px,py) in [('lplate',lplate),('rplate',rplate)]:
    for yaw in (math.pi/2,-math.pi/2):
        pre=r.ik_world([px-R_,py,0.70],down_quat(yaw),seed=q0)
        pl=r.ik_world([px-R_,py,0.443+0.10],down_quat(yaw),seed=pre)
        print(name,'yaw',round(yaw,2),'pre',None if pre is None else [round(v,2) for v in pre],'place',None if pl is None else [round(v,2) for v in pl])
r.shutdown()
"

# openrua op 28
mkdir -p "$(dirname /workspace/task.py)"
cat > /workspace/task.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Pick each mug by its rim and place it on its plate, verifying each step.
Usage: python3 -u task.py <white|yellow|both>
"""
import sys, math
import numpy as np, cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import rclpy
from robot import Robot, down_quat

R_RIM = 0.045          # rim radius (wall midpoint), measured ~9 cm dia
TABLE = 0.425
PLATE_TOP = 0.443
GRASP_DEPTH = 0.03     # fingertips this far below the rim
Z_TRAVEL = 0.72

# measured from birdview (world xy, rim top z)
MUGS = {
    "white":  dict(c=(-0.084, -0.186), ztop=0.549, yaw=+math.pi / 2, plate=(0.011, 0.290)),   # -> right plate
    "yellow": dict(c=(-0.066, 0.092),  ztop=0.542, yaw=-math.pi / 2, plate=(-0.011, -0.290)),  # -> left plate
}


def snap(r, cam, out):
    got = []
    sub = r.node.create_subscription(Image, f"/{cam}/color/image_raw", got.append, 1)
    while not got:
        rclpy.spin_once(r.node, timeout_sec=0.5)
    r.node.destroy_subscription(sub)
    cv2.imwrite(out, CvBridge().imgmsg_to_cv2(got[0], "bgr8"))
    print(f"  snapshot {out}")


def goto(r, xyz, quat, seconds, seed=None, label=""):
    q = r.ik_world(xyz, quat, seed=seed)
    if q is None:
        raise SystemExit(f"IK failed for {label} {np.round(xyz,3)}")
    code, err = r.move_q(q, seconds)
    tcp, _ = r.tcp_world()
    d = np.linalg.norm(tcp - np.asarray(xyz))
    print(f"  {label}: code={code} qerr={err:.4f} tcp={tcp.round(3)} target={np.round(xyz,3)} |d|={d*1000:.1f}mm")
    if d > 0.01:
        raise SystemExit(f"pose error too large at {label}")
    return q


def do_mug(r, name):
    m = MUGS[name]
    cx, cy = m["c"]
    quat = down_quat(m["yaw"])
    gx, gy = cx - R_RIM, cy               # grasp the rim on the -x (robot) side
    zg = m["ztop"] - GRASP_DEPTH
    h = m["ztop"] - TABLE                 # mug height
    print(f"== {name}: grasp at ({gx:.3f},{gy:.3f},{zg:.3f})")

    print("open gripper:", r.gripper(0.04))
    q = goto(r, [gx, gy, Z_TRAVEL], quat, 4.0, label="pre-grasp high")
    q = goto(r, [gx, gy, m["ztop"] + 0.04], quat, 2.5, seed=q, label="above rim")
    snap(r, "robot0_eye_in_hand", f"{name}_eih_above.png")
    q = goto(r, [gx, gy, zg], quat, 2.0, seed=q, label="grasp height")
    ok, stalled, gap = r.gripper(0.0)
    print(f"  close: reached={ok} stalled={stalled} gap={gap*1000:.1f}mm")
    if gap < 0.002:
        raise SystemExit("closed on air")
    q = goto(r, [gx, gy, Z_TRAVEL], quat, 3.0, seed=q, label="lift")
    gap = r.finger_gap()
    print(f"  gap after lift {gap*1000:.1f}mm")
    if gap < 0.002:
        raise SystemExit("lost the mug on lift")
    snap(r, "agentview", f"{name}_lifted.png")

    px, py = m["plate"]
    tx, ty = px - R_RIM, py               # mug centre lands on the plate centre
    z_place = PLATE_TOP + 0.012 + (h - GRASP_DEPTH)   # mug bottom ~1 cm above plate
    q = goto(r, [tx, ty, Z_TRAVEL], quat, 4.0, seed=q, label="above plate")
    q = goto(r, [tx, ty, z_place + 0.05], quat, 2.5, seed=q, label="lowering")
    q = goto(r, [tx, ty, z_place], quat, 2.0, seed=q, label="place height")
    print("  open:", r.gripper(0.04))
    q = goto(r, [tx, ty, Z_TRAVEL], quat, 3.0, seed=q, label="retreat")
    snap(r, "agentview", f"{name}_placed.png")


def main():
    which = sys.argv[1]
    r = Robot()
    try:
        for name in (["white", "yellow"] if which == "both" else [which]):
            do_mug(r, name)
        # park clear of the birdview
        goto(r, [-0.35, 0.0, 0.95], down_quat(0), 4.0, label="park")
    finally:
        r.shutdown()
    print("DONE")


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 29
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/task.py", "old_string": "    snap(r, \\"robot0_eye_in_hand\\", f\\"{name}_eih_above.png\\")\\n    q = goto(", "new_string": "    snap(r, \\"robot0_eye_in_hand\\", f\\"{name}_eih_above.png\\")\\n    if \\"check\\" in sys.argv:\\n        print(\\"CHECK STOP\\"); return\\n    q = goto(", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 30
timeout 1500 python3 -u task.py white check 2>&1 | grep -v "^\[" 

# openrua op 31
timeout 1700 python3 -u task.py white 2>&1 | grep -v "^\[" 

# openrua op 32
timeout 120 python3 locate.py birdview >/dev/null; python3 - <<'EOF'
import numpy as np, cv2
pw=np.load('birdview_world.npy'); color=cv2.imread('birdview_color.png')
z=pw[...,2]; sel=np.isfinite(z)&(pw[...,0]>-0.35)
def blobs(mask,name,minarea=15):
    n,lab,stats,cents=cv2.connectedComponentsWithStats(mask.astype(np.uint8))
    for i in range(1,n):
        if stats[i][4]<minarea: continue
        m=lab==i;P=pw[m]
        cx=(P[:,0].min()+P[:,0].max())/2; cy=(P[:,1].min()+P[:,1].max())/2
        print(f"{name}: area={stats[i][4]} mean=({P[:,0].mean():.3f},{P[:,1].mean():.3f}) bboxctr=({cx:.3f},{cy:.3f}) size=({P[:,0].max()-P[:,0].min():.3f},{P[:,1].max()-P[:,1].min():.3f}) z[{P[:,2].min():.3f},{P[:,2].max():.3f}]")
print("== rims"); blobs(sel&(z>0.50)&(z<0.62),'rim')
print("== low"); blobs(sel&(z>0.435)&(z<0.50),'low',minarea=100)
crop=color[220:320,200:440]; cv2.imwrite('bird_crop.png', cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_NEAREST))
EOF

# openrua op 33
timeout 300 python3 tools/action/gripper_cmd.py 0.04 >/dev/null; timeout 300 python3 tools/action/gripper_cmd.py 0.04 >/dev/null; timeout 60 python3 tools/perception/cam_snap.py agentview agentview3.png; timeout 60 python3 tools/perception/cam_snap.py frontview frontview3.png

# openrua op 34
python3 - <<'EOF'
import numpy as np
pw=np.load('birdview_world.npy'); z=pw[...,2]
pc=np.array([-0.011,-0.290])
r=np.hypot(pw[...,0]-pc[0],pw[...,1]-pc[1])
m=np.isfinite(z)&(r<0.09)
for lo in np.arange(0,0.09,0.01):
    s=m&(r>=lo)&(r<lo+0.01)
    if s.sum(): print(f"r {lo:.2f}-{lo+0.01:.2f}: z mean {z[s].mean():.4f} min {z[s].min():.4f} max {z[s].max():.4f} n={s.sum()}")
EOF

# openrua op 35
python3 - <<'EOF'
import numpy as np, cv2
pw=np.load('birdview_world.npy'); z=pw[...,2]; sel=np.isfinite(z)&(pw[...,0]>-0.35)
def fit_circle(P):
    # algebraic least squares circle fit
    x,y=P[:,0],P[:,1]; A=np.c_[2*x,2*y,np.ones_like(x)]; b=x**2+y**2
    c,_,_,_=np.linalg.lstsq(A,b,rcond=None); cx,cy=c[0],c[1]; r=np.sqrt(c[2]+cx**2+cy**2); return cx,cy,r
n,lab,stats,cents=cv2.connectedComponentsWithStats((sel&(z>0.50)&(z<0.62)).astype(np.uint8))
for i in range(1,n):
    if stats[i][4]<50: continue
    m=lab==i; P=pw[m]; zt=P[:,2].max()
    for dz in (0.006,0.01,0.015):
        ring=P[P[:,2]>zt-dz]
        cx,cy,r=fit_circle(ring)
        # robust: drop outliers > 8mm from circle, refit
        d=np.abs(np.hypot(ring[:,0]-cx,ring[:,1]-cy)-r); ring2=ring[d<0.008]
        cx2,cy2,r2=fit_circle(ring2)
        print(f"blob{i} ztop={zt:.3f} dz={dz}: n={len(ring)} fit=({cx:.3f},{cy:.3f}) r={r:.3f} | robust n={len(ring2)} ({cx2:.3f},{cy2:.3f}) r={r2:.3f}")
EOF

# openrua op 36
python3 -c "
import cv2; im=cv2.imread('frontview3.png'); cv2.imwrite('front_crop.png', cv2.resize(im[170:260,180:480],None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))"

# openrua op 37
for c in galleryview paperview robot0_robotview sideview; do timeout 60 python3 tools/perception/cam_snap.py $c ${c}3.png; done; python3 -c "
import cv2
ims=[cv2.imread(f'{c}3.png') for c in ['galleryview','paperview','robot0_robotview','sideview']]
top=cv2.hconcat(ims[:2]); bot=cv2.hconcat(ims[2:]); cv2.imwrite('grid.png', cv2.vconcat([top,bot]))"

# openrua op 38
python3 - <<'EOF'
import numpy as np
pw=np.load('birdview_world.npy'); z=pw[...,2]
def ring_profile(cx,cy,r,name):
    d=np.hypot(pw[...,0]-cx,pw[...,1]-cy)
    m=np.isfinite(z)&(d>r-0.012)&(d<r+0.012)&(z>0.49)
    ang=np.degrees(np.arctan2(pw[...,1][m]-cy,pw[...,0][m]-cx)); zz=z[m]
    out=[]
    for a in range(-180,180,45):
        s=(ang>=a)&(ang<a+45)
        if s.sum(): out.append(f"{a:4d}:{zz[s].max():.3f}")
    print(name, ' '.join(out))
ring_profile(0.005,0.295,0.045,'white on plate')
ring_profile(-0.067,0.096,0.047,'yellow on table')
ring_profile(-0.204,0.015,0.042,'red on table')
EOF

# openrua op 39
python3 - <<'EOF'
import re
s=open('task.py').read()
s=s.replace('"yellow": dict(c=(-0.066, 0.092),','"yellow": dict(c=(-0.067, 0.096),')
s=s.replace('    tx, ty = px - R_RIM, py               # mug centre lands on the plate centre',
            '    tx, ty = px - 0.040, py               # mug centre lands on the plate centre (0.040: measured from white run)')
s=s.replace('z_place = PLATE_TOP + 0.012 + (h - GRASP_DEPTH)   # mug bottom ~1 cm above plate',
            'z_place = PLATE_TOP + 0.006 + (h - GRASP_DEPTH)   # mug bottom ~6 mm above plate')
open('task.py','w').write(s)
EOF
grep -n "yellow\|tx, ty\|z_place =" task.py

# openrua op 40
timeout 1700 python3 -u task.py yellow 2>&1 | grep -v "^\[" 

# openrua op 41
timeout 60 python3 tools/perception/cam_snap.py agentview agentview4.png; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih4.png; python3 -c "
import cv2; a=cv2.imread('agentview4.png'); b=cv2.imread('eih4.png'); cv2.imwrite('pair.png', cv2.hconcat([a,b]))"

# openrua op 42
timeout 60 python3 tools/perception/cam_snap.py sideview side4.png; timeout 60 python3 tools/perception/cam_snap.py robot0_robotview rv4.png; timeout 30 ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once | grep -A7 wrench; python3 -c "
import cv2; a=cv2.imread('side4.png'); b=cv2.imread('rv4.png'); cv2.imwrite('pair2.png', cv2.hconcat([cv2.resize(a[300:480,200:480],None,fx=2,fy=2), cv2.resize(b[0:240,200:520],None,fx=1.5,fy=1.5)]))"

# openrua op 43
mkdir -p "$(dirname /workspace/eih_check.py)"
cat > /workspace/eih_check.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Eye-in-hand 3D check: point cloud in the camera frame; report the
fingertip blocks (bottom of image) and the nearest rim points."""
import numpy as np, rclpy, cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image

CAM = "robot0_eye_in_hand"


def grab(node, topic, T):
    got = {}
    sub = node.create_subscription(T, topic, lambda m: got.setdefault("m", m), 1)
    while "m" not in got:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    return got["m"]


rclpy.init(); node = rclpy.create_node("eih_check"); br = CvBridge()
color = br.imgmsg_to_cv2(grab(node, f"/{CAM}/color/image_raw", Image), "bgr8")
depth = br.imgmsg_to_cv2(grab(node, f"/{CAM}/depth/image_raw", Image), "passthrough").astype(float)
info = grab(node, f"/{CAM}/color/camera_info", CameraInfo)
rclpy.shutdown()
fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
H, W = depth.shape
vv, uu = np.mgrid[0:H, 0:W]
X = (uu - cx) * depth / fx; Y = (vv - cy) * depth / fy; Z = depth
np.save("eih_depth.npy", depth); cv2.imwrite("eih_color.png", color)
print(f"size {W}x{H} fx={fx:.1f} depth range {np.nanmin(Z):.3f}..{np.nanmax(Z):.3f}")
# depth histogram of near things (fingers)
near = Z < 0.20
print("near (<0.20m) pixel count", near.sum())
for lo in np.arange(0.05, 0.40, 0.025):
    s = (Z >= lo) & (Z < lo + 0.025)
    if s.sum() > 50:
        print(f"  Z {lo:.3f}-{lo+0.025:.3f}: n={s.sum():6d} X[{X[s].min():+.3f},{X[s].max():+.3f}] Y[{Y[s].min():+.3f},{Y[s].max():+.3f}]")
OPENRUA_EOF

# openrua op 44
timeout 120 python3 eih_check.py

# openrua op 45
python3 -c "
import numpy as np, cv2
d=np.load('eih_depth.npy'); v=((d-0.04)/(0.22-0.04)*255).clip(0,255).astype('uint8'); v=cv2.applyColorMap(v,cv2.COLORMAP_JET); cv2.imwrite('eih_depth.png', v)"

# openrua op 46
python3 - <<'EOF'
import numpy as np, cv2
d=np.load('eih_depth.npy'); info_fx=312.8
H,W=d.shape; vv,uu=np.mgrid[0:H,0:W]; cx,cy=W/2-0.5,H/2-0.5
X=(uu-cx)*d/info_fx; Y=(vv-cy)*d/info_fx; Z=d
def rep(name,m):
    print(f"{name}: n={m.sum()} X[{X[m].min():+.3f},{X[m].max():+.3f}] mean {X[m].mean():+.3f}  Y[{Y[m].min():+.3f},{Y[m].max():+.3f}]  Z[{Z[m].min():.3f},{Z[m].max():.3f}] mean {Z[m].mean():.3f}")
rows=(vv>=355)&(vv<=405)
rep('left finger block', rows&(uu>100)&(uu<210)&(Z<0.095))
rep('right finger block', rows&(uu>430)&(uu<500)&(Z<0.095))
rep('hand body strip (v>410)', (vv>415)&(Z<0.08))
# rim: outer edge of mug disk: pixels with Z in [0.095,0.115] and radius from disk center
# disk center approx (205,270) px, outer radius ~165 px
r=np.hypot(uu-205,vv-270)
rep('rim ring (r 150-170, Z<0.13)', (r>150)&(r<170)&(Z<0.13)&(vv<350))
rep('rim ring top segment', (r>150)&(r<170)&(Z<0.13)&(vv<150))
rep('rim ring left seg', (r>150)&(r<170)&(Z<0.13)&(uu<80))
rep('rim ring right seg', (r>150)&(r<170)&(Z<0.13)&(uu>330)&(vv<300))
rep('mug bottom (red)', (r<40))
rep('red mug region', (uu>480)&(vv>290)&(vv<380)&(Z<0.15))
EOF

# openrua op 47
timeout 900 python3 -c "
from robot import *
r=Robot()
q=r.arm_q()
res=r.move_world([-0.112,0.096,0.72], down_quat(-math.pi/2), seconds=3.0, seed=q); print('lift', res[1:], r.tcp_world()[0].round(3))
# calibration pose: fingertips at z=0.60 over free table
res=r.move_world([-0.10,-0.10,0.60], down_quat(-math.pi/2), seconds=4.0, seed=res[0]); print('calib pose', res[1:], 'tcp', r.tcp_world()[0].round(3), 'gap', round(r.finger_gap(),4))
r.shutdown()
" 2>&1 | grep -v '^\['; timeout 120 python3 locate.py birdview > /dev/null; python3 - <<'EOF'
import numpy as np
pw=np.load('birdview_world.npy'); z=pw[...,2]
m=np.isfinite(z)&(z>0.59)&(z<0.66)&(np.abs(pw[...,1]+0.10)<0.08)&(np.abs(pw[...,0]+0.10)<0.15)
P=pw[m]; print('finger pixels', m.sum(), 'x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]'%(P[:,0].min(),P[:,0].max(),P[:,1].min(),P[:,1].max(),P[:,2].min(),P[:,2].max()))
print('centroid xy', P[:,:2].mean(0).round(3), ' bbox center', ((P[:,0].min()+P[:,0].max())/2, (P[:,1].min()+P[:,1].max())/2))
# the fingertips only (lowest 1.5cm)
m2=m&(z<0.615); P2=pw[m2]; print('tips n', m2.sum(), 'x[%.3f,%.3f] y[%.3f,%.3f]'%(P2[:,0].min(),P2[:,0].max(),P2[:,1].min(),P2[:,1].max()), 'centroid', P2[:,:2].mean(0).round(3))
EOF

# openrua op 48
python3 - <<'EOF'
import numpy as np, cv2
pw=np.load('birdview_world.npy'); z=pw[...,2]; color=cv2.imread('birdview_color.png')
m=np.isfinite(z)&(z>0.55)&(z<0.90)&(pw[...,0]>-0.4)
P=pw[m]
print('n',m.sum())
for lo in np.arange(0.55,0.90,0.025):
    s=m&(z>=lo)&(z<lo+0.025)
    if s.sum(): print(f"z {lo:.3f}: n={s.sum():5d} x[{pw[...,0][s].min():+.3f},{pw[...,0][s].max():+.3f}] y[{pw[...,1][s].min():+.3f},{pw[...,1][s].max():+.3f}]")
crop=color[200:330,180:460]; cv2.imwrite('bird_crop2.png', cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_NEAREST))
EOF

# openrua op 49
timeout 300 python3 -c "
from robot import *
r=Robot()
xyz,quat=r.fk_world(); R=quat_R(*quat)
print('hand pos',xyz.round(3)); print('hand x-axis (world):',R[:,0].round(3)); print('hand y-axis (finger axis):',R[:,1].round(3)); print('hand z-axis:',R[:,2].round(3))
print('requested quat', down_quat(-math.pi/2), 'actual', np.round(quat,3))
r.shutdown()
" 2>&1 | grep -v '^\['

# openrua op 50
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/robot.py", "old_string": "class Robot:", "new_string": "def ik_down_quat(hand_yaw):\\n    \\"\\"\\"Quaternion to REQUEST from IK so that panda_hand ends up pointing down\\n    with its finger axis at `hand_yaw` (0 = fingers along world y,\\n    pi/2 = along world x). Measured: IK solves for panda_link8, which is\\n    rotated -45 deg about z from panda_hand, so request yaw - pi/4.\\"\\"\\"\\n    return down_quat(hand_yaw - math.pi / 4)\\n\\n\\nclass Robot:", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 51
timeout 300 python3 -c "
from robot import *
r=Robot()
q0=r.arm_q()
for hy in (math.pi/2, -math.pi/2, 0):
    q=r.ik_world([-0.112,0.096,0.72], ik_down_quat(hy), seed=q0)
    if q is None: print('yaw',hy,'IK none'); continue
    xyz,quat=r.fk_world(q); R=quat_R(*quat); tcp,_=r.tcp_world(q)
    print(f'hand yaw {hy:+.2f}: finger axis {R[:,1].round(3)} hand z {R[:,2].round(3)} tcp {tcp.round(3)} q={[round(v,2) for v in q]}')
r.shutdown()
" 2>&1 | grep -v '^\['

# openrua op 52
python3 - <<'EOF'
s=open('task.py').read()
s=s.replace("from robot import Robot, down_quat","from robot import Robot, down_quat, ik_down_quat\n\nHOME = [0.0, -0.16, 0.0, -2.44, 0.0, 2.23, 0.785]")
s=s.replace("""    quat = down_quat(m["yaw"])
    gx, gy = cx - R_RIM, cy               # grasp the rim on the -x (robot) side
    zg = m["ztop"] - GRASP_DEPTH
    h = m["ztop"] - TABLE                 # mug height
    print(f"== {name}: grasp at ({gx:.3f},{gy:.3f},{zg:.3f})")

    print("open gripper:", r.gripper(0.04))
    q = goto(r, [gx, gy, Z_TRAVEL], quat, 4.0, label="pre-grasp high")""",
"""    gx, gy = cx - R_RIM, cy               # grasp the rim on the -x (robot) side
    zg = m["ztop"] - GRASP_DEPTH
    h = m["ztop"] - TABLE                 # mug height
    print(f"== {name}: grasp at ({gx:.3f},{gy:.3f},{zg:.3f})")

    # fingers along world x; +-pi/2 are equivalent, keep the least contorted
    best = None
    for yaw in (math.pi / 2, -math.pi / 2):
        qq = r.ik_world([gx, gy, Z_TRAVEL], ik_down_quat(yaw), seed=HOME)
        if qq is None:
            continue
        cost = sum(abs(a - b) for a, b in zip(qq[:6], HOME[:6]))
        print(f"  yaw {yaw:+.2f}: q={[round(v,2) for v in qq]} cost={cost:.2f}")
        if best is None or cost < best[0]:
            best = (cost, yaw, qq)
    quat = ik_down_quat(best[1])
    print("open gripper:", r.gripper(0.04))
    q = goto(r, [gx, gy, Z_TRAVEL], quat, 4.0, seed=best[2], label="pre-grasp high")""")
s=s.replace('goto(r, [-0.35, 0.0, 0.95], down_quat(0), 4.0, label="park")','goto(r, [-0.35, 0.0, 0.95], ik_down_quat(0), 4.0, seed=HOME, label="park")')
open('task.py','w').write(s)
EOF
grep -n "ik_down_quat\|HOME\|best" task.py

# openrua op 53
timeout 600 python3 -c "
from robot import *
r=Robot()
HOME=[0.0,-0.16,0.0,-2.44,0.0,2.23,0.785]
res=r.move_world([-0.35,0.0,0.95], ik_down_quat(0), seconds=4.0, seed=HOME); print('park', res[1:], r.tcp_world()[0].round(3))
r.shutdown()
" 2>&1 | grep -v '^\['; timeout 120 python3 locate.py birdview >/dev/null; python3 - <<'EOF'
import numpy as np, cv2
pw=np.load('birdview_world.npy'); z=pw[...,2]; sel=np.isfinite(z)&(pw[...,0]>-0.35)
def fit_circle(P):
    x,y=P[:,0],P[:,1]; A=np.c_[2*x,2*y,np.ones_like(x)]; b=x**2+y**2
    c,_,_,_=np.linalg.lstsq(A,b,rcond=None); cx,cy=c[0],c[1]; return cx,cy,np.sqrt(c[2]+cx**2+cy**2)
n,lab,stats,cents=cv2.connectedComponentsWithStats((sel&(z>0.50)&(z<0.62)).astype(np.uint8))
for i in range(1,n):
    if stats[i][4]<50: continue
    m=lab==i; P=pw[m]; zt=P[:,2].max(); ring=P[P[:,2]>zt-0.01]
    cx,cy,r=fit_circle(ring); d=np.abs(np.hypot(ring[:,0]-cx,ring[:,1]-cy)-r); cx,cy,r=fit_circle(ring[d<0.008])
    print(f"rim: ztop={zt:.3f} center=({cx:.3f},{cy:.3f}) r={r:.3f}")
EOF

# openrua op 54
sed -i 's/"yellow": dict(c=(-0.067, 0.096),/"yellow": dict(c=(-0.064, 0.096),/' task.py && timeout 1700 python3 -u task.py yellow 2>&1 | grep -v "^\[" 

# openrua op 55
timeout 300 python3 tools/action/gripper_cmd.py 0.04 >/dev/null; timeout 60 python3 tools/perception/cam_snap.py agentview final_agent.png; timeout 60 python3 tools/perception/cam_snap.py frontview final_front.png; timeout 120 python3 locate.py birdview >/dev/null; python3 - <<'EOF'
import numpy as np, cv2
pw=np.load('birdview_world.npy'); z=pw[...,2]; color=cv2.imread('birdview_color.png'); sel=np.isfinite(z)&(pw[...,0]>-0.35)
def fit_circle(P):
    x,y=P[:,0],P[:,1]; A=np.c_[2*x,2*y,np.ones_like(x)]; b=x**2+y**2
    c,_,_,_=np.linalg.lstsq(A,b,rcond=None); cx,cy=c[0],c[1]; return cx,cy,np.sqrt(c[2]+cx**2+cy**2)
plates={'left':(-0.011,-0.290),'right':(0.011,0.290)}
n,lab,stats,cents=cv2.connectedComponentsWithStats((sel&(z>0.50)&(z<0.62)).astype(np.uint8))
for i in range(1,n):
    if stats[i][4]<50: continue
    m=lab==i; P=pw[m]; zt=P[:,2].max(); ring=P[P[:,2]>zt-0.01]
    cx,cy,r=fit_circle(ring); d=np.abs(np.hypot(ring[:,0]-cx,ring[:,1]-cy)-r); cx,cy,r=fit_circle(ring[d<0.008])
    hsv=cv2.cvtColor(color[m].reshape(-1,1,3),cv2.COLOR_BGR2HSV).reshape(-1,3)
    yellow=((hsv[:,0]>18)&(hsv[:,0]<38)&(hsv[:,1]>90)).mean(); red=(((hsv[:,0]<8)|(hsv[:,0]>170))&(hsv[:,1]>110)).mean()
    tag='yellow/white mug' if yellow>0.1 else ('red mug' if red>0.1 else 'white mug')
    # level check: rim z by octant
    dd=np.hypot(pw[...,0]-cx,pw[...,1]-cy); rm=np.isfinite(z)&(dd>r-0.012)&(dd<r+0.012)&(z>0.49)
    ang=np.degrees(np.arctan2(pw[...,1][rm]-cy,pw[...,0][rm]-cx)); zz=z[rm]
    oct=[zz[(ang>=a)&(ang<a+45)].max() for a in range(-180,180,45) if ((ang>=a)&(ang<a+45)).any()]
    near=min(plates,key=lambda k:np.hypot(cx-plates[k][0],cy-plates[k][1])); dist=np.hypot(cx-plates[near][0],cy-plates[near][1])
    print(f"{tag:17s} center=({cx:+.3f},{cy:+.3f}) r={r:.3f} ztop={zt:.3f} rim z spread={max(oct)-min(oct):.3f}  nearest plate={near} ({dist*100:.1f} cm off centre)")
crop=color[220:320,200:440]; cv2.imwrite('final_bird_crop.png', cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_NEAREST))
EOF

# openrua op 56
python3 -c "
import cv2; a=cv2.imread('final_agent.png'); b=cv2.imread('final_bird_crop.png'); b=cv2.resize(b,(640,int(640*b.shape[0]/b.shape[1]))); cv2.imwrite('final_pair.png', cv2.vconcat([a,b]))"

# openrua op 57
mkdir -p "$(dirname /claude-config/projects/-workspace/memory/panda-workstation-quirks.md)"
cat > /claude-config/projects/-workspace/memory/panda-workstation-quirks.md <<'OPENRUA_EOF'
---
name: panda-workstation-quirks
description: Measured facts about the Franka Panda sim workstation in /workspace that contradict or extend the starter docs (frames, IK tip link, gripper limits, settling)
metadata:
  type: project
---

Measured on the /workspace Panda workstation (2026-09-12), verified by FK vs TF vs birdview depth:

- `/compute_ik` and `/compute_fk` work directly in **world** coordinates (FK of the current q equals TF world->panda_hand). The doc's "planner frame is the arm base" is wrong here; do NOT add the world->panda_link0 offset (-0.51, 0, 0.42).
- IK solves for **panda_link8**, which is rotated -45° about z from panda_hand. To get the hand's finger axis at yaw θ, request quat for yaw θ − π/4 (see `robot.py: ik_down_quat`). Otherwise fingers come out diagonal.
- Gripper max gap ≈ 7.8–8.0 cm; the mugs are ~9 cm across, so grasp by the **rim** (one finger inside, one outside), fingertips ~3 cm below the rim. Gap after closing on a rim: ~12–18 mm.
- Scene is unsettled at session start (objects float ~6 cm). Run any command (e.g. gripper open) before localising.
- FollowJointTrajectory often returns -5 with a residual on long moves; resending the same goal converges (`robot.py: move_q` retries).
- Table top z=0.425, plate tops z=0.443 (flat only to r≈4 cm, then slopes up). Birdview camera at (-0.2, 0, 3) gives ~4 mm/px; circle-fit the rim ring for mug centres.

**Why:** these took several probing rounds to discover; the docs are generic and partly wrong for this machine.
**How to apply:** reuse `/workspace/robot.py` and `/workspace/task.py` patterns; verify with FK + birdview rim fit rather than oblique cameras (they exaggerate tilt).
OPENRUA_EOF

# openrua op 58
cd /claude-config/projects/-workspace/memory && (test -f MEMORY.md && cat MEMORY.md || echo "(no index yet)")

# openrua op 59
mkdir -p "$(dirname /claude-config/projects/-workspace/memory/MEMORY.md)"
cat > /claude-config/projects/-workspace/memory/MEMORY.md <<'OPENRUA_EOF'
- [Panda workstation quirks](panda-workstation-quirks.md) — IK/FK are world-frame, IK tip is link8 (45° off hand), 8 cm gripper → rim grasps, scene needs a command to settle
OPENRUA_EOF
