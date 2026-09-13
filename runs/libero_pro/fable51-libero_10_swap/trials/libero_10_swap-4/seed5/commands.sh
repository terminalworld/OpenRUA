#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
cat /workspace/tools/action/fjt_send.py; echo ----; cat /workspace/tools/action/gripper_cmd.py; echo ----; cat /workspace/tools/action/ik_move.py

# openrua op 3
timeout 30 ros2 topic list; echo ---; timeout 20 ros2 action list; echo ---; timeout 20 ros2 node list

# openrua op 4
for c in agentview birdview frontview sideview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c; done; timeout 20 ros2 topic echo /joint_states --once

# openrua op 5
timeout 20 ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id|translation|rotation|x:|y:|z:|w:" | head -80; echo ---; timeout 15 ros2 topic echo /birdview/color/camera_info --once

# openrua op 6
mkdir -p "$(dirname /workspace/scene.py)"
cat > /workspace/scene.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Scene helper: dump TF frames and convert pixels of a camera to world.

Usage: python3 scene.py <camera> u,v [u,v ...]
"""
import struct
import sys

import numpy as np
import rclpy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener


def grab(node, topic, T, timeout=20.0):
    got = {}
    sub = node.create_subscription(T, topic, lambda m: got.setdefault("m", m), 1)
    import time
    end = time.time() + timeout
    while "m" not in got and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    return got.get("m")


def quat_R(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main():
    cam = sys.argv[1]
    pix = [tuple(int(a) for a in p.split(",")) for p in sys.argv[2:]]
    rclpy.init()
    node = rclpy.create_node("scene")
    buf = Buffer()
    TransformListener(buf, node)
    import time
    t0 = time.time()
    while time.time() - t0 < 3:
        rclpy.spin_once(node, timeout_sec=0.2)
    print(buf.all_frames_as_string())
    for tgt in ["panda_link0", "panda_hand", f"{cam}_optical_frame"]:
        try:
            t = buf.lookup_transform("world", tgt, Time())
            tr, q = t.transform.translation, t.transform.rotation
            print(f"world->{tgt}: t=({tr.x:.4f},{tr.y:.4f},{tr.z:.4f}) q=({q.x:.4f},{q.y:.4f},{q.z:.4f},{q.w:.4f})")
        except Exception as e:
            print(f"world->{tgt}: FAIL {e}")
    if not pix:
        return
    depth = grab(node, f"/{cam}/depth/image_raw", Image)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    t = buf.lookup_transform("world", f"{cam}_optical_frame", Time())
    T = np.eye(4)
    T[:3, :3] = quat_R(t.transform.rotation)
    T[:3, 3] = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
    d = np.frombuffer(depth.data, dtype=np.float32).reshape(depth.height, depth.width)
    for u, v in pix:
        z = float(d[v, u])
        p = T @ np.array([(u - cx) * z / fx, (v - cy) * z / fy, z, 1.0])
        print(f"px({u},{v}) depth={z:.4f} -> world ({p[0]:.4f}, {p[1]:.4f}, {p[2]:.4f})")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 7
timeout 60 python3 scene.py birdview 253,288 385,285 278,268 320,236 318,285 320,160

# openrua op 8
timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png && python3 -c "
import numpy as np, cv2
d=np.load('birdview_depth.npy')
# camera at z=3.0 looking down: height = 3.0 - depth (approx, pinhole so only exact at center; compute properly)
fx=579.4112549695428; cx=320; cy=240
H,W=d.shape
v,u=np.mgrid[0:H,0:W]
# ray direction z component: depth is z along optical axis, so world z = 3.0 - depth
z = 3.0 - d
wx = -0.2 + (v-cy)*d/fx   # image down -> +x (verified)
wy = (u-cx)*d/fx           # image right -> +y
table = np.median(z[(z>0.45)&(z<0.5)])
print('table z', table)
mask = (z>table+0.02)&(z<table+0.2)
# find connected components of elevated pixels
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask.astype(np.uint8))
for i in range(1,n):
    if stats[i,4]<30: continue
    m=lab==i
    print(i,'area',stats[i,4],'centroid px',cent[i].round(1),'world xy',(wx[m].mean().round(3),wy[m].mean().round(3)),'zmax',z[m].max().round(3),'zmed',np.median(z[m]).round(3))
img=cv2.imread('birdview.png')
crop=img[200:330,200:440]
cv2.imwrite('bird_crop.png',cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_NEAREST))
"


# openrua op 9
timeout 20 ros2 service list | grep -iE "fk|ik|state|scene"; timeout 10 ros2 interface show moveit_msgs/srv/GetPositionFK | head -30

# openrua op 10
mkdir -p "$(dirname /workspace/robot.py)"
cat > /workspace/robot.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Reusable robot control library for the Panda (one node, clients built once).

Frames: IK/FK operate in panda_link0 (arm base). world->panda_link0 is
translation BASE_T (from TF). Helpers accept world coordinates and
convert.
"""
import math
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
TRAJ = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
TWIST = next(a for a in M["actuators"] if a["kind"] == "cartesian_twist")
ARM = TRAJ["joints"]
BASE_T = np.array([-0.51, 0.0, 0.42])  # world -> panda_link0 (TF)
TCP = float(M["hand"]["tcp_offset_m"])

# hand pointing straight down, fingers along world Y  (qx,qy,qz,qw)
Q_DOWN = (1.0, 0.0, 0.0, 0.0)


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def quat_yaw_down(yaw):
    """Hand pointing down (hand +Z = world -Z), rotated about world Z by yaw.
    yaw=0 -> same as Q_DOWN (rotation of pi about X)."""
    # q = qz(yaw) * qx(pi)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    # qx(pi) = (1,0,0,0); qz = (0,0,sy,cy)
    # product (qz * qx): w = cy*0 - sy*0 = 0 ... compute generally
    a = (0.0, 0.0, sy, cy)
    b = (1.0, 0.0, 0.0, 0.0)
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


class Robot:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("robot_lib")
        self.js = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, TRAJ["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.ik_cli = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.twist_pub = self.node.create_publisher(TwistStamped, TWIST["port"], 10)
        self.fjt.wait_for_server(10)
        self.grip.wait_for_server(10)
        self.fk_cli.wait_for_service(10)
        self.ik_cli.wait_for_service(10)
        self.wait_js()

    def _on_js(self, m):
        self.js = m

    def spin(self, t=0.1):
        rclpy.spin_once(self.node, timeout_sec=t)

    def wait_js(self):
        self.js = None
        t0 = time.time()
        while self.js is None and time.time() - t0 < 10:
            self.spin(0.2)
        return self.js

    def joints(self):
        self.wait_js()
        d = dict(zip(self.js.name, self.js.position))
        return [d[j] for j in ARM]

    def fingers(self):
        self.wait_js()
        d = dict(zip(self.js.name, self.js.position))
        return d["panda_finger_joint1"], d["panda_finger_joint2"]

    # ---------- kinematics (base frame) ----------
    def fk(self, q=None, link="panda_hand"):
        q = self.joints() if q is None else q
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            raise RuntimeError(f"FK failed: {r and r.error_code.val}")
        p = r.pose_stamped[0].pose
        return (np.array([p.position.x, p.position.y, p.position.z]),
                (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w))

    def hand_world(self):
        p, q = self.fk()
        return p + BASE_T, q

    def tcp_world(self, q=None):
        p, quat = self.fk(q)
        R = quat_R(*quat)
        return p + BASE_T + TCP * R[:, 2]

    def ik(self, pos_base, quat, seed=None, tries=5):
        seed = self.joints() if seed is None else seed
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pos_base)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.robot_state.joint_state.name = list(ARM)
        req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
        req.ik_request.timeout.sec = 1
        req.ik_request.avoid_collisions = False
        for _ in range(tries):
            fut = self.ik_cli.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            r = fut.result()
            if r is not None and r.error_code.val == 1:
                sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
                return [sol[j] for j in ARM]
        return None

    def ik_world(self, pos_world, quat, seed=None, at_tcp=True):
        pos = np.array(pos_world, dtype=float) - BASE_T
        if at_tcp:
            R = quat_R(*quat)
            pos = pos - TCP * R[:, 2]
        return self.ik(pos, quat, seed)

    # ---------- motion ----------
    def move_joints(self, q, seconds=3.0, waypoints=None):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pts = []
        wps = list(waypoints or []) + [(q, seconds)]
        for pos, t in wps:
            pt = JointTrajectoryPoint(positions=[float(v) for v in pos])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        h = send.result()
        res = h.get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        err = np.abs(np.array(self.joints()) - np.array(q)).max()
        print(f"  move_joints: code={code} max_err={err:.4f}", flush=True)
        return code, err

    def move_tcp_world(self, pos_world, quat, seconds=3.0, seed=None):
        q = self.ik_world(pos_world, quat, seed)
        if q is None:
            print(f"  IK FAILED for {pos_world}", flush=True)
            return None
        self.move_joints(q, seconds)
        tcp = self.tcp_world()
        print(f"  tcp now {tcp.round(4)} target {np.array(pos_world).round(4)}", flush=True)
        return q

    def servo(self, vx=0.0, vy=0.0, vz=0.0, n=20, dt=0.05):
        msg = TwistStamped()
        msg.header.frame_id = TWIST["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = vx, vy, vz
        for _ in range(n):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.twist_pub.publish(msg)
            self.spin(dt)

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=120)
        r = res.result().result
        f = self.fingers()
        print(f"  gripper({width}): reached={r.reached_goal} stalled={r.stalled} fingers={f}", flush=True)
        return f


if __name__ == "__main__":
    r = Robot()
    print("joints", np.array(r.joints()).round(4))
    print("fingers", r.fingers())
    p, q = r.hand_world()
    print("hand world", p.round(4), np.array(q).round(4))
    print("tcp world", r.tcp_world().round(4))
    R = quat_R(*q)
    print("hand axes (cols x,y,z):\n", R.round(3))
OPENRUA_EOF

# openrua op 11
timeout 120 python3 robot.py

# openrua op 12
timeout 60 python3 -c "
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
rclpy.init(); n=rclpy.create_node('rd')
got=[]
qos=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
n.create_subscription(String,'/robot_description',got.append,qos)
import time; t=time.time()
while not got and time.time()-t<10: rclpy.spin_once(n,timeout_sec=0.2)
open('robot.urdf','w').write(got[0].data); print(len(got[0].data))
"; grep -E "<joint name=|<origin|<parent|<child|<axis" robot.urdf | head -80

# openrua op 13
python3 -c "
import numpy as np
def rx(a):
    c,s=np.cos(a),np.sin(a); return np.array([[1,0,0],[0,c,-s],[0,s,c]])
def rz(a):
    c,s=np.cos(a),np.sin(a); return np.array([[c,-s,0],[s,c,0],[0,0,1]])
def T(R,p):
    M=np.eye(4); M[:3,:3]=R; M[:3,3]=p; return M
# (rpy roll about x only, xyz)
chain=[(0,[0,0,0.333]),(-np.pi/2,[0,0,0]),(np.pi/2,[0,-0.316,0]),(np.pi/2,[0.0825,0,0]),(-np.pi/2,[-0.0825,0.384,0]),(np.pi/2,[0,0,0]),(np.pi/2,[0.088,0,0])]
def fk(q):
    M=np.eye(4)
    for (r,p),qi in zip(chain,q):
        M=M@T(rx(r),p)@T(rz(qi),[0,0,0])
    M=M@T(np.eye(3),[0,0,0.107])@T(rz(-np.pi/4),[0,0,0])
    return M
q=[0,-0.161037389,0,-2.44459747,0,2.2267522,0.7853981633974483]
M=fk(q); print('hand base-frame', M[:3,3].round(4)); print(M[:3,:3].round(3))
print('hand world', (M[:3,3]+[-0.51,0,0.42]).round(4))
"


# openrua op 14
timeout 120 python3 -c "
from robot import *
r=Robot()
q0=r.joints()
p,quat=r.fk()
print('FK raw pos', p.round(4))
# test IK in 'world' interpretation (raw) and base interpretation
for label,pos in [('raw(as returned)',p),('minus BASE_T',p-BASE_T),('plus BASE_T',p+BASE_T)]:
    sol=r.ik(pos,quat,seed=q0,tries=2)
    if sol is None: print(label,'IK none'); continue
    print(label,'sol',np.array(sol).round(3),'diff from current',np.abs(np.array(sol)-np.array(q0)).max().round(4))
"


# openrua op 15
timeout 120 python3 -c "
from robot import *
r=Robot()
q0=r.joints()
p,quat=r.fk()
sol=[0.058,-0.161,-0.058,-2.445,-0.012,2.227,0.009]
p2,quat2=r.fk(sol)
print(p.round(4),np.array(quat).round(3)); print(p2.round(4),np.array(quat2).round(3))
"


# openrua op 16
python3 - <<'EOF'
import re
s=open('robot.py').read()
s=s.replace('BASE_T = np.array([-0.51, 0.0, 0.42])  # world -> panda_link0 (TF)',
 'BASE_T = np.array([0.0, 0.0, 0.0])  # MoveIt model frame IS world here (verified: FK of current pose matches birdview)')
s=s.replace('        req.ik_request.pose_stamped.header.frame_id = ""\n',
 '        req.ik_request.pose_stamped.header.frame_id = ""\n        req.ik_request.ik_link_name = "panda_hand"\n')
open('robot.py','w').write(s)
EOF
timeout 120 python3 -c "
from robot import *
r=Robot()
q0=r.joints()
p,quat=r.fk()
sol=r.ik(p,quat,seed=q0,tries=2)
print('sol',np.array(sol).round(3),'diff',np.abs(np.array(sol)-np.array(q0)).max().round(4))
print('tcp world', r.tcp_world().round(4))
"

# openrua op 17
timeout 600 python3 -c "
from robot import *
r=Robot()
for tgt in [(-0.45,0,1.0),(-0.42,0,0.95),(-0.40,0,0.9)]:
    q=r.ik_world(tgt,Q_DOWN)
    print(tgt,'->',None if q is None else np.array(q).round(3))
    if q is not None:
        r.move_joints(q,4.0)
        print('tcp',r.tcp_world().round(4)); break
"


# openrua op 18
timeout 600 python3 -c "
from robot import *
r=Robot()
q=[0.,-1.314,0.,-2.238,0.,0.924,0.785]
print('before',np.array(r.joints()).round(3))
r.move_joints(q,4.0)
print('after',np.array(r.joints()).round(3)); print('tcp',r.tcp_world().round(4))
"


# openrua op 19
timeout 120 python3 tools/perception/cam_snap.py birdview && timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png && timeout 60 python3 tools/perception/cam_snap.py agentview && python3 -c "
import numpy as np, cv2
d=np.load('birdview_depth.npy')
fx=579.4112549695428; cx=320; cy=240
H,W=d.shape
v,u=np.mgrid[0:H,0:W]
z = 3.0 - d
wx = -0.2 + (v-cy)*d/fx
wy = (u-cx)*d/fx
tab=(z>0.47)&(z<0.5)
print('table z median', np.median(z[tab]).round(4), 'p95', np.percentile(z[tab],95).round(4))
table=0.488
mask = (z>table+0.02)&(z<table+0.3)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask.astype(np.uint8))
for i in range(1,n):
    if stats[i,4]<30: continue
    m=lab==i
    xs,ys=wx[m],wy[m]
    print(i,'area',stats[i,4],'px',cent[i].round(1),'xy',(xs.mean().round(3),ys.mean().round(3)),'xrange',(xs.min().round(3),xs.max().round(3)),'yrange',(ys.min().round(3),ys.max().round(3)),'zmax',z[m].max().round(3))
# plates: slightly above table
pm=(z>table+0.004)&(z<table+0.02)
n,lab,stats,cent=cv2.connectedComponentsWithStats(pm.astype(np.uint8))
for i in range(1,n):
    if stats[i,4]<100: continue
    m=lab==i
    print('plate?',i,'area',stats[i,4],'px',cent[i].round(1),'xy',(wx[m].mean().round(3),wy[m].mean().round(3)),'zmax',z[m].max().round(3))
img=cv2.imread('birdview.png')
crop=img[200:330,200:440]
cv2.imwrite('bird_crop.png',cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_NEAREST))
"


# openrua op 20
python3 -c "
import numpy as np
d=np.load('birdview_depth.npy'); z=3.0-d
np.set_printoptions(linewidth=200, precision=4, suppress=True)
print('row 288 (through plates), cols 200..440 step 8'); print(z[288,200:440:8])
print('row 268 (white mug), cols 250..320 step 3'); print(z[268,250:320:3])
print('col 282 (white mug), rows 240..300 step 3'); print(z[240:300:3,282])
print('row 236 (yellow mug), cols 290..360 step 3'); print(z[236,290:360:3])
print('row 272 (red mug), cols 310..380 step 3'); print(z[272,310:380:3])
print('table far corners', z[100,100], z[400,550], z[240,320], z[350,320])
"


# openrua op 21
python3 -c "
import numpy as np, cv2
d=np.load('birdview_depth.npy'); z=3.0-d
fx=579.4112549695428; cx=320; cy=240
H,W=d.shape
v,u=np.mgrid[0:H,0:W]
wx = -0.2 + (v-cy)*d/fx; wy = (u-cx)*d/fx
def comps(mask,minarea,label):
    n,lab,stats,cent=cv2.connectedComponentsWithStats(mask.astype(np.uint8))
    for i in range(1,n):
        if stats[i,4]<minarea: continue
        m=lab==i
        print(label,'area',stats[i,4],'px',cent[i].round(1),'xy',(wx[m].mean().round(4),wy[m].mean().round(4)),'z min/med/max',z[m].min().round(3),np.median(z[m]).round(3),z[m].max().round(3))
# plates and mug interiors: 0.435..0.47 above table region (x in -0.4..0.3)
region=(wx>-0.4)&(wx<0.3)&(np.abs(wy)<0.5)
comps((z>0.435)&(z<0.47)&region,80,'plate/interior')
comps((z>0.5)&(z<0.6)&region,20,'rim')
"


# openrua op 22
mkdir -p "$(dirname /workspace/pick_place.py)"
cat > /workspace/pick_place.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Rim-grasp pick and place of a mug onto a plate.

Usage: python3 pick_place.py <phase> ...
  pick  mx my rim_z side(+1|-1)   grasp the mug wall on the +y/-y side
  place px py                     put the held mug centred on the plate
Coordinates are world; hand always points straight down (Q_DOWN).
"""
import sys
import time

import numpy as np

from robot import Q_DOWN, Robot

WALL_R = 0.036      # mug wall radius (centre of the wall)
Z_TRAVEL = 0.68     # safe travel height for the TCP
GRASP_BELOW_RIM = 0.03
MUG_H = 0.11        # mug height (bottom -> rim)
PLATE_TOP = 0.447


def goto(r, pos, secs=3.0, tol=0.01):
    """IK + trajectory with one retry; returns final TCP."""
    for attempt in range(3):
        q = r.ik_world(pos, Q_DOWN)
        if q is None:
            raise SystemExit(f"IK failed for {pos}")
        code, err = r.move_joints(q, secs)
        tcp = r.tcp_world()
        d = np.linalg.norm(tcp - np.array(pos))
        print(f"  goto {np.array(pos).round(3)} -> tcp {tcp.round(4)} (|d|={d:.4f}) code={code}", flush=True)
        if code == 0 and d < tol:
            return tcp
        print("  retrying goal", flush=True)
    return tcp


def pick(r, mx, my, rim_z, side):
    gy = my + side * WALL_R
    print(f"PICK mug at ({mx},{my}) rim {rim_z}, wall grasp at y={gy:.3f}", flush=True)
    r.gripper(0.04)
    goto(r, (mx, gy, Z_TRAVEL), 4.0)
    goto(r, (mx, gy, rim_z + 0.02), 3.0)        # just above the rim
    goto(r, (mx, gy, rim_z - GRASP_BELOW_RIM), 2.0)  # fingers straddle wall
    f = r.gripper(0.0)
    gap = f[0] - f[1] if f[1] < 0 else f[0] + f[1]
    print(f"  finger gap after close: {gap:.4f}", flush=True)
    goto(r, (mx, gy, Z_TRAVEL), 3.0)
    f = r.fingers()
    print(f"  fingers after lift: {f}", flush=True)
    return gy


def place(r, px, py, side):
    gy = py + side * WALL_R
    print(f"PLACE on plate ({px},{py}), tcp y={gy:.3f}", flush=True)
    goto(r, (px, gy, Z_TRAVEL), 4.0)
    z_set = PLATE_TOP + (MUG_H - GRASP_BELOW_RIM) + 0.008
    goto(r, (px, gy, z_set + 0.04), 3.0)
    goto(r, (px, gy, z_set), 2.0)
    r.gripper(0.04)
    goto(r, (px, gy, Z_TRAVEL), 3.0)


if __name__ == "__main__":
    r = Robot()
    a = sys.argv[1:]
    t0 = time.time()
    if a[0] == "pick":
        pick(r, float(a[1]), float(a[2]), float(a[3]), float(a[4]))
    elif a[0] == "place":
        place(r, float(a[1]), float(a[2]), float(a[3]))
    elif a[0] == "goto":
        goto(r, tuple(map(float, a[1:4])))
    print(f"done in {time.time()-t0:.0f}s wall", flush=True)
OPENRUA_EOF

# openrua op 23
timeout 1700 python3 -u pick_place.py pick -0.083 -0.170 0.535 -1 2>&1 | tee pick1.log

# openrua op 24
timeout 120 python3 tools/perception/cam_snap.py agentview && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 25
timeout 1700 python3 -u pick_place.py place 0.011 -0.297 -1 2>&1 | tee place1.log

# openrua op 26
timeout 120 python3 tools/perception/cam_snap.py agentview && timeout 60 python3 tools/perception/cam_snap.py birdview && python3 -c "
import cv2; img=cv2.imread('birdview.png'); cv2.imwrite('bird_crop.png',cv2.resize(img[200:330,200:440],None,fx=3,fy=3,interpolation=cv2.INTER_NEAREST))"

# openrua op 27
timeout 1700 python3 -u pick_place.py pick -0.220 0.018 0.53 1 2>&1 | tee pick2.log

# openrua op 28
timeout 120 python3 tools/perception/cam_snap.py agentview

# openrua op 29
timeout 1700 python3 -u pick_place.py place -0.007 0.291 1 2>&1 | tee place2.log

# openrua op 30
timeout 900 python3 -u pick_place.py goto -0.45 0 1.0 2>&1 | tail -3; for c in agentview birdview frontview; do timeout 60 python3 tools/perception/cam_snap.py $c; done; timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png; python3 -c "
import numpy as np, cv2
d=np.load('birdview_depth.npy'); z=3.0-d
fx=579.4112549695428; cx=320; cy=240
H,W=d.shape; v,u=np.mgrid[0:H,0:W]
wx=-0.2+(v-cy)*d/fx; wy=(u-cx)*d/fx
region=(wx>-0.4)&(wx<0.3)&(np.abs(wy)<0.5)
def comps(mask,minarea,label):
    n,lab,stats,cent=cv2.connectedComponentsWithStats(mask.astype(np.uint8))
    for i in range(1,n):
        if stats[i,4]<minarea: continue
        m=lab==i
        print(label,'area',stats[i,4],'xy',(wx[m].mean().round(4),wy[m].mean().round(4)),'z med/max',np.median(z[m]).round(3),z[m].max().round(3))
comps((z>0.435)&(z<0.5)&region,80,'plate/interior')
comps((z>0.5)&(z<0.6)&region,20,'rim')
img=cv2.imread('birdview.png'); cv2.imwrite('bird_crop.png',cv2.resize(img[200:330,200:440],None,fx=3,fy=3,interpolation=cv2.INTER_NEAREST))"
