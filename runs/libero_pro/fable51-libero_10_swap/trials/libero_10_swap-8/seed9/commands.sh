#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 2 -type f | head -50

# openrua op 1
cat README.md machine.yaml docs/10-machine.md docs/20-perception.md docs/30-action.md docs/40-patterns.md tools/README.md; ls -la .ros tools

# openrua op 2
ls -la tools/action tools/perception; cat tools/perception/*.py tools/action/*.py

# openrua op 3
timeout 30 ros2 topic list; echo ---; timeout 20 ros2 action list; echo ---; timeout 20 ros2 service list | grep -v parameter; echo ---; timeout 20 ros2 node list

# openrua op 4
for c in agentview frontview birdview sideview robot0_robotview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c & done; wait; timeout 20 ros2 topic echo /joint_states --once

# openrua op 5
timeout 20 ros2 topic echo /tf_static --once --qos-durability transient_local 2>&1 | grep -E "frame_id|x:|y:|z:|w:" | head -80; echo ---; timeout 10 ros2 topic echo /tf --once 2>&1 | grep -E "frame_id" | head

# openrua op 6
cat > /workspace/tfdump.py <<'EOF'
import rclpy
from tf2_msgs.msg import TFMessage
rclpy.init(); n = rclpy.create_node("tfdump")
got = []
n.create_subscription(TFMessage, "/tf", got.append, 10)
import time
end = time.time()+5
while time.time() < end:
    rclpy.spin_once(n, timeout_sec=0.2)
seen = {}
for m in got:
    for t in m.transforms:
        seen[(t.header.frame_id, t.child_frame_id)] = t.transform
for k, tr in seen.items():
    p, q = tr.translation, tr.rotation
    print(k, f"t=({p.x:.4f},{p.y:.4f},{p.z:.4f}) q=({q.x:.4f},{q.y:.4f},{q.z:.4f},{q.w:.4f})")
EOF
timeout 30 python3 /workspace/tfdump.py

# openrua op 7
timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png && timeout 20 ros2 topic echo /birdview/color/camera_info --once | grep -A12 "^k:"

# openrua op 8
cat > /workspace/bird.py <<'EOF'
import numpy as np, cv2
d = np.load("birdview_depth.npy")
fx=fy=579.4112549695428; cx=320; cy=240
# birdview optical: t=(-0.2,0,3.0) q=(0.7071,0.7071,0,0) -> R = rot about (1,1,0)/sqrt2 by 180deg
# R = [[0,1,0],[1,0,0],[0,0,-1]]
T = np.array([-0.2,0,3.0])
def w(u,v):
    z = d[v,u]
    pc = np.array([(u-cx)*z/fx,(v-cy)*z/fy,z])
    return np.array([pc[1], pc[0], -pc[2]]) + T
print("table at (320,420):", w(320,420))
print("floor at (100,50):", w(100,50))
# heights map: world z
img = cv2.imread("birdview.png")
Z = np.zeros_like(d)
for v in range(0,480):
    for u in range(0,640):
        Z[v,u] = w(u,v)[2]
np.save("birdZ.npy", Z)
tab = np.median(Z[400:470, 200:440])
print("table z median", tab)
mask = (Z > tab + 0.02).astype(np.uint8)
mask[:200,:] = 0  # cut the robot area roughly
n, lab, stats, cent = cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    x,y,ww,hh,a = stats[i]
    if a < 30: continue
    ys,xs = np.where(lab==i)
    zmax = Z[ys,xs].max()
    c = cent[i]
    print(f"blob {i}: px center ({c[0]:.0f},{c[1]:.0f}) bbox {x},{y},{ww},{hh} area {a} zmax {zmax:.3f} world_xy {w(int(c[0]),int(c[1]))[:2]}")
EOF
python3 /workspace/bird.py

# openrua op 9
cat > /workspace/bird2.py <<'EOF'
import numpy as np, cv2
Z = np.load("birdZ.npy"); d=np.load("birdview_depth.npy")
fx=579.4112549695428; cx=320; cy=240; T=np.array([-0.2,0,3.0])
def w(u,v):
    z=d[v,u]; pc=np.array([(u-cx)*z/fx,(v-cy)*z/fy if False else (v-cy)*z/fx,z]); return np.array([pc[1],pc[0],-pc[2]])+T
print("Z hist table region:", np.percentile(Z[300:450,180:460],[5,25,50,75,95]))
mask = ((Z > 0.92) & (Z < 1.3)).astype(np.uint8)
mask[:150,:]=0
n, lab, stats, cent = cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    x,y,ww,hh,a = stats[i]
    if a < 30: continue
    ys,xs = np.where(lab==i)
    c = cent[i]
    print(f"blob {i}: px ({c[0]:.0f},{c[1]:.0f}) bbox x{x} y{y} w{ww} h{hh} area {a} zmax {Z[ys,xs].max():.3f} zmin {Z[ys,xs].min():.3f} world {w(int(c[0]),int(c[1]))}")
EOF
python3 /workspace/bird2.py

# openrua op 10
cat > /workspace/bird3.py <<'EOF'
import numpy as np
Z = np.load("birdZ.npy"); d=np.load("birdview_depth.npy")
fx=579.4112549695428; cx=320; cy=240; T=np.array([-0.2,0,3.0])
def w(u,v):
    z=d[v,u]; pc=np.array([(u-cx)*z/fx,(v-cy)*z/fx,z]); return np.array([pc[1],pc[0],-pc[2]])+T
np.set_printoptions(linewidth=250, precision=2)
for name,(x,y,ww,hh) in {"potA":(237,229,42,23),"potB":(365,275,43,25)}.items():
    sub = Z[y:y+hh, x:x+ww]
    print(name); print(((sub-0.9)*100).astype(int))
    ys,xs = np.where(sub > 1.04)
    cu, cv = x+xs.mean(), y+ys.mean()
    print(" top-region center px", cu, cv, "world", w(int(round(cu)),int(round(cv))))
    ys,xs = np.where(sub > 0.95)
    print(" body >5cm center px", x+xs.mean(), y+ys.mean(), "extent", xs.min(),xs.max(), ys.min(), ys.max())
print("between knob and plate", [round(float(Z[v,330]),3) for v in range(296,335)])
print("stove plate corners:", w(305,328), w(357,381))
EOF
python3 /workspace/bird3.py

# openrua op 11
timeout 60 python3 tools/perception/cam_snap.py /frontview/depth/image_raw frontview_depth.png && timeout 60 python3 tools/perception/cam_snap.py /sideview/depth/image_raw sideview_depth.png && timeout 20 ros2 topic echo /frontview/color/camera_info --once | grep -A3 "^k:" && timeout 20 ros2 topic echo /sideview/color/camera_info --once | grep -A3 "^k:"

# openrua op 12
cat > /workspace/cloud.py <<'EOF'
import numpy as np
fx=579.4112549695428; cx=320; cy=240
def R(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
cams = {"frontview":((1.0,0,1.48),(0.5608,0.5608,-0.4306,-0.4306)),
        "sideview":((-0.0565,1.2761,1.4880),(0.0099,0.8064,-0.5912,-0.0069))}
def cloud(name):
    d=np.load(f"{name}_depth.npy"); t,q=cams[name]; Rm=R(*q)
    v,u=np.mgrid[0:480,0:640]
    pc=np.stack([(u-cx)*d/fx,(v-cy)*d/fx,d],-1).reshape(-1,3)
    return pc@Rm.T+np.array(t)
for name in cams:
    P=cloud(name)
    for pot,(px,py) in {"A":(-0.2,-0.2),"B":(-0.037,0.24)}.items():
        m=(np.abs(P[:,0]-px)<0.08)&(np.abs(P[:,1]-py)<0.08)&(P[:,2]>0.905)
        Q=P[m]
        print(name,pot,"n",len(Q),"zmax",Q[:,2].max() if len(Q) else None)
        for z0 in np.arange(0.91,1.06,0.01):
            s=Q[np.abs(Q[:,2]-z0)<0.005]
            if len(s)<3: continue
            print(f"  z={z0:.2f} n={len(s):3d} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]")
EOF
python3 /workspace/cloud.py

# openrua op 13
timeout 10 ros2 interface show moveit_msgs/srv/GetPositionIK | head -40; timeout 10 ros2 interface show moveit_msgs/srv/GetPositionFK | head -30

# openrua op 14
mkdir -p "$(dirname /workspace/rob.py)"
cat > /workspace/rob.py <<'OPENRUA_EOF'
"""Reusable helpers for this Panda: joint state, FK/IK, trajectory, gripper,
servo, camera snapshots.  World <-> base offset from TF (world->panda_link0).
"""
import time
import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState, Image
from trajectory_msgs.msg import JointTrajectoryPoint
from cv_bridge import CvBridge
import cv2

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
TW = next(a for a in M["actuators"] if a["kind"] == "cartesian_twist")
JOINTS = FJT["joints"]
BASE_IN_WORLD = np.array([-0.66, 0.0, 0.912])   # from /tf_static world->panda_link0
TCP = M["hand"]["tcp_offset_m"]


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def R_to_quat(R):
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1) * 2
        return np.array([(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s,
                         (R[1, 0] - R[0, 1]) / s, 0.25 * s])
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = np.sqrt(1 + R[i, i] - R[j, j] - R[k, k]) * 2
    q = np.zeros(4)
    q[i] = 0.25 * s
    q[j] = (R[j, i] + R[i, j]) / s
    q[k] = (R[k, i] + R[i, k]) / s
    q[3] = (R[k, j] - R[j, k]) / s
    return q


class Robot:
    def __init__(self, name="rob"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self.js = {}
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.tw_pub = self.node.create_publisher(TwistStamped, TW["port"], 10)
        self.bridge = CvBridge()
        self.fjt.wait_for_server(10)
        self.grip.wait_for_server(10)
        self.ik.wait_for_service(10)
        self.fk.wait_for_service(10)
        self.wait_js()

    def _on_js(self, msg):
        self.js = dict(zip(msg.name, msg.position))

    def spin(self, t=0.05):
        rclpy.spin_once(self.node, timeout_sec=t)

    def wait_js(self):
        self.js = {}
        while not self.js:
            self.spin(0.2)
        return dict(self.js)

    def arm_q(self):
        js = self.wait_js()
        return np.array([js[j] for j in JOINTS])

    def fingers(self):
        js = self.wait_js()
        return js.get("panda_finger_joint1"), js.get("panda_finger_joint2")

    # ---- kinematics (base frame) ----
    def _seed(self, q):
        s = JointState()
        s.name = list(JOINTS)
        s.position = [float(v) for v in q]
        return s

    def fk_base(self, q=None, link="panda_hand"):
        q = self.arm_q() if q is None else q
        req = GetPositionFK.Request()
        req.fk_link_names = [link]
        req.robot_state.joint_state = self._seed(q)
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {None if res is None else res.error_code.val}")
        p = res.pose_stamped[0].pose
        return (np.array([p.position.x, p.position.y, p.position.z]),
                np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]))

    def tcp_world(self, q=None):
        p, quat = self.fk_base(q)
        R = quat_to_R(*quat)
        return p + BASE_IN_WORLD + R[:, 2] * TCP, quat

    def ik_base(self, pos, quat, seed=None, timeout=20.0):
        """pos: hand-frame position in base frame. Returns joint array or None."""
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pos)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.robot_state.joint_state = self._seed(self.arm_q() if seed is None else seed)
        req.ik_request.timeout = Duration(sec=int(timeout), nanosec=int((timeout % 1) * 1e9))
        req.ik_request.avoid_collisions = False
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return np.array([sol[j] for j in JOINTS])

    def ik_tcp_world(self, tcp_world, quat, seed=None):
        """IK for a TCP position given in WORLD frame with hand quaternion quat."""
        R = quat_to_R(*quat)
        hand_world = np.asarray(tcp_world) - R[:, 2] * TCP
        return self.ik_base(hand_world - BASE_IN_WORLD, quat, seed)

    # ---- motion ----
    def move_q(self, q, seconds=3.0, via=None):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        pts = []
        if via is not None:
            for i, v in enumerate(via):
                pt = JointTrajectoryPoint(positions=[float(x) for x in v])
                t = seconds * (i + 1) / (len(via) + 1)
                pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
                pts.append(pt)
        pt = JointTrajectoryPoint(positions=[float(x) for x in q])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res)
        code = res.result().result.error_code
        err = np.abs(self.arm_q() - np.asarray(q)).max()
        print(f"  move_q: error_code={code} max joint err={err:.4f}", flush=True)
        return code, err

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP.get("max_effort", 30.0))
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=120)
        r = rf.result().result
        f = self.fingers()
        print(f"  gripper({width}): reached={r.reached_goal} stalled={r.stalled} fingers={f}", flush=True)
        return f

    def servo(self, lin, ang=(0, 0, 0), ticks=20, dt=0.05):
        msg = TwistStamped()
        msg.header.frame_id = TW["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, lin)
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = map(float, ang)
        for _ in range(ticks):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.tw_pub.publish(msg)
            self.spin(dt)

    # ---- vision ----
    def snap(self, cam, out=None):
        topic = cam if cam.startswith("/") else f"/{cam}/color/image_raw"
        out = out or f"/workspace/{cam.strip('/').split('/')[0]}.png"
        got = []
        sub = self.node.create_subscription(Image, topic, got.append, 1)
        while not got:
            self.spin(0.2)
        self.node.destroy_subscription(sub)
        msg = got[0]
        if "FC" in msg.encoding or "16UC" in msg.encoding:
            d = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            np.save(out.rsplit(".", 1)[0] + ".npy", d)
            return d
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        cv2.imwrite(out, img)
        return img

    def depth(self, cam):
        return self.snap(f"/{cam}/depth/image_raw", f"/workspace/{cam}_depth.png")
OPENRUA_EOF

# openrua op 15
cat > iktest.py <<'EOF'
import numpy as np, sys
from rob import *
r = Robot("iktest")
q0 = r.arm_q(); print("q0", q0)
p, quat = r.fk_base(); print("hand base", p, quat, "tcp world", r.tcp_world()[0])
# candidate downward orientations: fingers along world x
cands = {"fx+": (0.7071,0.7071,0,0), "fx-": (0.7071,-0.7071,0,0), "fy": (1,0,0,0), "fy2": (0,1,0,0)}
for name,(x,y,z) in {"potA":(-0.196,-0.200,0.97),"potB":(-0.035,0.238,0.97),"stove1":(0.165,-0.01,1.0),"stove2":(0.165,0.09,1.0),"stoveC":(0.21,0.04,1.0)}.items():
    for cn, quat in cands.items():
        sol = r.ik_tcp_world((x,y,z), quat, seed=q0)
        ok = sol is not None
        chk = ""
        if ok:
            tw, _ = r.tcp_world(sol); chk = f"fk-check tcp={np.round(tw,3)} q={np.round(sol,2)}"
        print(name, cn, "OK" if ok else "FAIL", chk, flush=True)
EOF
timeout 600 python3 -u iktest.py 2>&1 | tail -40

# openrua op 16
cat > fktest.py <<'EOF'
import numpy as np
from rob import *
import rclpy
from moveit_msgs.srv import GetPositionFK
r = Robot("fktest")
q0 = r.arm_q()
for fid in ["", "world", "panda_link0"]:
    req = GetPositionFK.Request(); req.header.frame_id = fid
    req.fk_link_names = ["panda_link0","panda_hand"]
    req.robot_state.joint_state = r._seed(q0)
    fut = r.fk.call_async(req); rclpy.spin_until_future_complete(r.node, fut, timeout_sec=30)
    res = fut.result()
    print(repr(fid), res.error_code.val, [(ps.header.frame_id, np.round([ps.pose.position.x,ps.pose.position.y,ps.pose.position.z],3)) for ps in res.pose_stamped])
# IK in world coords for pot A above
for quat in [(0.7071,0.7071,0,0),(0.7071,-0.7071,0,0),(1,0,0,0)]:
    R = quat_to_R(*quat)
    hand = np.array([-0.196,-0.200,1.05]) - R[:,2]*TCP
    sol = r.ik_base(hand, quat, seed=q0)
    print(quat, sol if sol is None else np.round(sol,3))
EOF
timeout 300 python3 -u fktest.py 2>&1 | tail

# openrua op 17
python3 - <<'EOF'
s = open("rob.py").read()
s = s.replace('''    def tcp_world(self, q=None):
        p, quat = self.fk_base(q)
        R = quat_to_R(*quat)
        return p + BASE_IN_WORLD + R[:, 2] * TCP, quat''','''    def tcp_world(self, q=None):
        # NOTE: measured fact: /compute_fk and /compute_ik on this machine work
        # in the WORLD frame (empty frame_id == "world"), not panda_link0.
        p, quat = self.fk_base(q)
        R = quat_to_R(*quat)
        return p + R[:, 2] * TCP, quat''')
s = s.replace('''        return self.ik_base(hand_world - BASE_IN_WORLD, quat, seed)''','''        return self.ik_base(hand_world, quat, seed)''')
s = s.replace('"""pos: hand-frame position in base frame.', '"""pos: hand-frame position in WORLD frame (model frame).')
open("rob.py","w").write(s)
EOF
sed -i 's/^p, quat = r.fk_base(); print("hand base"/p, quat = r.fk_base(); print("hand world"/' iktest.py
timeout 900 python3 -u iktest.py 2>&1 | tail -22

# openrua op 18
cat > iksweep.py <<'EOF'
import numpy as np
from rob import *
r = Robot("iksweep")
q0 = r.arm_q()
def rot_y(th):
    c,s=np.cos(th),np.sin(th); return np.array([[c,0,s],[0,1,0],[-s,0,c]])
def rot_x(th):
    c,s=np.cos(th),np.sin(th); return np.array([[1,0,0],[0,c,-s],[0,s,c]])
Rfx = quat_to_R(0.7071,-0.7071,0,0)   # fingers along x, z down
Rfy = quat_to_R(1,0,0,0)             # fingers along y, z down
seed = np.array([0.0, 0.4, 0.0, -1.8, 0.0, 2.2, 0.8])
for z in [0.96, 1.0]:
  for pitch in [0, 15, 30, 45]:
    for name,Rb in [("fx",Rfx),("fy",Rfy)]:
        R = rot_y(np.radians(pitch)) @ Rb   # tilt hand z toward +x
        q = R_to_quat(R)
        reach = None
        for x in [0.10,0.13,0.16,0.19,0.22,0.25]:
            sol = r.ik_tcp_world((x,0.04,z), q, seed=seed)
            if sol is not None: reach = x; last=sol
        print(f"z={z} pitch={pitch} {name}: max x reachable = {reach}", "" if reach is None else np.round(last,2), flush=True)
EOF
timeout 1500 python3 -u iksweep.py 2>&1 | tail -20

# openrua op 19
sleep 120; cat /tmp/claude-1000/-workspace/1209017d-0a7d-4dcf-a575-e66083b58ce3/tasks/b660mu3s5.output

# openrua op 20
cat > iksweep.py <<'EOF'
import numpy as np, time, sys
from rob import *
r = Robot("iksweep")
def rot_y(th):
    c,s=np.cos(th),np.sin(th); return np.array([[c,0,s],[0,1,0],[-s,0,c]])
Rfx = quat_to_R(0.7071,-0.7071,0,0)   # fingers along x, z down
Rfy = quat_to_R(1,0,0,0)             # fingers along y, z down
seed = np.array([0.0, 0.4, 0.0, -1.8, 0.0, 2.2, 0.8])
z = 1.0
for pitch in [0, 20, 35, 50]:
    for name,Rb in [("fx",Rfx),("fy",Rfy)]:
        R = rot_y(np.radians(pitch)) @ Rb
        q = R_to_quat(R)
        reach = None
        for x in [0.10,0.14,0.17,0.20,0.24]:
            t=time.time()
            sol = r.ik_tcp_world((x,0.04,z), q, seed=seed)
            print(f"   x={x} {'ok' if sol is not None else 'fail'} {time.time()-t:.1f}s", flush=True)
            if sol is None: break
            reach = x; last=sol
        print(f"z={z} pitch={pitch} {name}: max x reachable = {reach}", "" if reach is None else np.round(last,2), flush=True)
EOF
sed -i 's/timeout=20.0/timeout=1.5/' rob.py
timeout 900 python3 -u iksweep.py > iksweep.log 2>&1 &
sleep 90; cat iksweep.log

# openrua op 21
sed -i 's/for pitch in \[0, 20, 35, 50\]/for pitch in [-20, -35, -50]/' iksweep.py && timeout 900 python3 -u iksweep.py > iksweep.log 2>&1; cat iksweep.log

# openrua op 22
cat > iksweep2.py <<'EOF'
import numpy as np, time, sys
from rob import *
r = Robot("iksweep2")
def rot_y(th):
    c,s=np.cos(th),np.sin(th); return np.array([[c,0,s],[0,1,0],[-s,0,c]])
Rfx = quat_to_R(0.7071,-0.7071,0,0)   # fingers along x, z down
seeds = [np.array([0.08,1.23,-0.07,-0.72,0.08,1.94,1.6]), np.array([0.0,0.9,0.0,-1.0,0.0,1.9,0.8]), np.array([0.0,1.4,0.0,-0.4,0.0,1.8,0.8])]
for pitch in [0, -10]:
    R = rot_y(np.radians(pitch)) @ Rfx; q = R_to_quat(R)
    for y in [-0.01, 0.09]:
        for x in [0.15,0.16,0.17,0.18,0.19]:
            got=None
            for s in seeds:
                sol = r.ik_tcp_world((x,y,1.0), q, seed=s, timeout=4.0)
                if sol is not None: got=sol; break
            print(f"pitch={pitch} y={y} x={x}: {'OK '+str(np.round(got,2)) if got is not None else 'fail'}", flush=True)
            if got is None: break
EOF
timeout 900 python3 -u iksweep2.py > iksweep2.log 2>&1; cat iksweep2.log

# openrua op 23
python3 - <<'EOF'
s=open("rob.py").read()
s=s.replace('''    def ik_tcp_world(self, tcp_world, quat, seed=None):
        """IK for a TCP position given in WORLD frame with hand quaternion quat."""
        R = quat_to_R(*quat)
        hand_world = np.asarray(tcp_world) - R[:, 2] * TCP
        return self.ik_base(hand_world, quat, seed)''','''    def ik_tcp_world(self, tcp_world, quat, seed=None, timeout=1.5):
        """IK for a TCP position given in WORLD frame with hand quaternion quat."""
        R = quat_to_R(*quat)
        hand_world = np.asarray(tcp_world) - R[:, 2] * TCP
        return self.ik_base(hand_world, quat, seed, timeout)''')
open("rob.py","w").write(s)
EOF
timeout 900 python3 -u iksweep2.py > iksweep2.log 2>&1; cat iksweep2.log

# openrua op 24
cat > stage1.py <<'EOF'
import numpy as np, time
from rob import *
r = Robot("stage1")
print("fingers before:", r.fingers())
r.gripper(0.04)
POT_A = np.array([-0.196, -0.200])
Q_FX = (0.7071, -0.7071, 0, 0)   # fingers along world x, hand z down
q0 = r.arm_q()
pre = r.ik_tcp_world((POT_A[0], POT_A[1], 1.15), Q_FX, seed=q0, timeout=5)
print("pre-grasp q:", None if pre is None else np.round(pre, 3))
if pre is None: raise SystemExit("no IK")
r.move_q(pre, 4.0)
tcp, quat = r.tcp_world(); print("tcp now", np.round(tcp, 4), np.round(quat, 3))
r.snap("robot0_eye_in_hand"); r.depth("robot0_eye_in_hand")
r.snap("agentview", "/workspace/agentview_s1.png")
EOF
timeout 900 python3 -u stage1.py 2>&1 | tail -20

# openrua op 25
cat > oritest.py <<'EOF'
import numpy as np
from rob import *
r = Robot("oritest")
q0 = r.arm_q()
pos = (-0.196, -0.200, 1.15)
for quat in [(1,0,0,0),(0.7071,-0.7071,0,0),(0.7071,0.7071,0,0),(0,1,0,0),(0.9239,-0.3827,0,0),(0.3827,-0.9239,0,0)]:
    sol = r.ik_tcp_world(pos, quat, seed=q0, timeout=5)
    if sol is None: print(quat, "fail"); continue
    tcp, fq = r.tcp_world(sol)
    R = quat_to_R(*fq)
    print("req", quat, "-> fk quat", np.round(fq,3), "hand y (finger axis) in world", np.round(R[:,1],3), "j7", round(sol[6],3))
EOF
timeout 600 python3 -u oritest.py 2>&1 | tail

# openrua op 26
python3 - <<'EOF'
s=open("rob.py").read()
s=s.replace('''    def ik_tcp_world(self, tcp_world, quat, seed=None, timeout=1.5):
        """IK for a TCP position given in WORLD frame with hand quaternion quat."""
        R = quat_to_R(*quat)
        hand_world = np.asarray(tcp_world) - R[:, 2] * TCP
        return self.ik_base(hand_world, quat, seed, timeout)''','''    def ik_tcp_world(self, tcp_world, quat, seed=None, timeout=1.5):
        """IK for a TCP position given in WORLD frame with desired panda_hand
        quaternion quat.  Measured fact: /compute_ik solves for a tip frame
        rotated -45 deg about z from panda_hand (panda_link8), so the request
        orientation is R_hand @ Rz(+45deg).  Position is shared."""
        R = quat_to_R(*quat)
        hand_world = np.asarray(tcp_world) - R[:, 2] * TCP
        c, s_ = np.cos(np.pi / 4), np.sin(np.pi / 4)
        Rz45 = np.array([[c, -s_, 0], [s_, c, 0], [0, 0, 1]])
        q_req = R_to_quat(R @ Rz45)
        return self.ik_base(hand_world, q_req, seed, timeout)''')
open("rob.py","w").write(s)
EOF
cat > oritest2.py <<'EOF'
import numpy as np
from rob import *
r = Robot("oritest2")
q0 = r.arm_q()
pos = (-0.196, -0.200, 1.15)
def rot_y(th):
    c,s=np.cos(th),np.sin(th); return np.array([[c,0,s],[0,1,0],[-s,0,c]])
tests = [(1,0,0,0),(0.7071,-0.7071,0,0),(0.7071,0.7071,0,0), tuple(R_to_quat(rot_y(np.radians(-10))@quat_to_R(1,0,0,0)))]
for quat in tests:
    sol = r.ik_tcp_world(pos, quat, seed=q0, timeout=5)
    if sol is None: print(quat, "fail"); continue
    tcp, fq = r.tcp_world(sol)
    R = quat_to_R(*fq)
    print("req", np.round(quat,3), "-> fk quat", np.round(fq,3), "tcp", np.round(tcp,3), "finger axis", np.round(R[:,1],3), "hand z", np.round(R[:,2],3), "j7", round(sol[6],3))
EOF
timeout 600 python3 -u oritest2.py 2>&1 | tail

# openrua op 27
cat > iktest3.py <<'EOF'
import numpy as np
from rob import *
r = Robot("iktest3")
def rot_y(th):
    c,s=np.cos(th),np.sin(th); return np.array([[c,0,s],[0,1,0],[-s,0,c]])
seedA = np.array([0.03,1.13,-0.08,-0.95,0.09,2.25,1.52])
for base in [(1,0,0,0),(0,1,0,0)]:
    for pitch in [10,12,15]:
        q = R_to_quat(rot_y(np.radians(-pitch)) @ quat_to_R(*base))
        for y in [-0.012, 0.092]:
            for z in [1.055, 1.135]:
                for x in [0.15,0.16,0.17]:
                    sol = r.ik_tcp_world((x,y,z), q, seed=seedA, timeout=3)
                    ok = sol is not None
                    extra = ""
                    if ok:
                        tcp, fq = r.tcp_world(sol); R = quat_to_R(*fq)
                        extra = f"j={np.round(sol,2)} hz={np.round(R[:,2],2)}"
                    print(f"base={base} pitch={pitch} y={y} z={z} x={x}: {'OK' if ok else 'fail'} {extra}", flush=True)
EOF
timeout 1200 python3 -u iktest3.py > iktest3.log 2>&1; grep -c OK iktest3.log; grep fail iktest3.log | head -30

# openrua op 28
grep "pitch=12" iktest3.log

# openrua op 29
timeout 20 ros2 topic echo /robot0_eye_in_hand/color/camera_info --once | grep -A9 "^k:" | tr -d ' \n'; echo; cat >> rob.py <<'EOF'


def tf_lookup(node, child, parent="world", secs=3.0):
    """Read the latest transform parent->child straight off /tf (no buffer staleness)."""
    from tf2_msgs.msg import TFMessage
    got = {}
    def cb(m):
        for t in m.transforms:
            if t.header.frame_id == parent and t.child_frame_id == child:
                got["t"] = t.transform
    sub = node.create_subscription(TFMessage, "/tf", cb, 50)
    end = time.time() + secs
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
        if "t" in got and time.time() > end - secs + 1.0:
            break
    node.destroy_subscription(sub)
    if "t" not in got:
        return None
    tr = got["t"]
    R = quat_to_R(tr.rotation.x, tr.rotation.y, tr.rotation.z, tr.rotation.w)
    p = np.array([tr.translation.x, tr.translation.y, tr.translation.z])
    return p, R


def cloud_from_depth(depth, K, p, R):
    """Depth image (m) + intrinsics + camera pose -> Nx3 world points."""
    fx, fy, cx, cy = K[0], K[4], K[2], K[5]
    v, u = np.mgrid[0:depth.shape[0], 0:depth.shape[1]]
    pc = np.stack([(u - cx) * depth / fx, (v - cy) * depth / fy, depth], -1).reshape(-1, 3)
    return pc @ R.T + p
EOF
echo ok

# openrua op 30
cat > stage2.py <<'EOF'
import numpy as np, sys
from rob import *
r = Robot("stage2")
POT = np.array([-0.196, -0.200])
Q_G = (0.7071, -0.7071, 0, 0)   # fingers along world x, hand z down, hand x = world -y
q0 = r.arm_q()
pre = r.ik_tcp_world((POT[0], POT[1], 1.15), Q_G, seed=q0, timeout=5)
print("pre q", np.round(pre,3)); r.move_q(pre, 3.0)
tcp, fq = r.tcp_world(); print("tcp", np.round(tcp,4), "quat", np.round(fq,3))
# eye-in-hand depth -> refine pot center
d = r.depth("robot0_eye_in_hand"); r.snap("robot0_eye_in_hand")
res = tf_lookup(r.node, "robot0_eye_in_hand_optical_frame")
print("cam pose", None if res is None else (np.round(res[0],4), np.round(res[1],3)))
K = [-312.77408948188935,0,320,0,-312.77408948188935,240,0,0,1]
for sign in [1,-1]:
    Kt = list(K); Kt[0]*=sign; Kt[4]*=sign
    P = cloud_from_depth(d, Kt, *res)
    tab = P[(P[:,2]>0.85)&(P[:,2]<0.95)]
    top = P[(P[:,2]>1.0)&(P[:,2]<1.045)&(np.abs(P[:,0]-POT[0])<0.08)&(np.abs(P[:,1]-POT[1])<0.08)]
    print(f"sign {sign}: table pts {len(tab)} z med {np.median(tab[:,2]) if len(tab) else None}; top pts {len(top)}",
          "center", np.round(top[:,:2].mean(0),4) if len(top) else None,
          "x-range", (np.round(top[:,0].min(),3), np.round(top[:,0].max(),3)) if len(top) else None,
          "y-range", (np.round(top[:,1].min(),3), np.round(top[:,1].max(),3)) if len(top) else None)
    # body-only slice (exclude handle/spout): z 0.99..1.02
    body = P[(P[:,2]>0.985)&(P[:,2]<1.025)&(np.abs(P[:,0]-POT[0])<0.08)&(np.abs(P[:,1]-POT[1])<0.08)]
    if len(body): print("   body slice center", np.round(body[:,:2].mean(0),4), "x-range", np.round([body[:,0].min(),body[:,0].max()],3), "y-range", np.round([body[:,1].min(),body[:,1].max()],3))
EOF
timeout 600 python3 -u stage2.py 2>&1 | tail -12

# openrua op 31
cat > measure.py <<'EOF'
import numpy as np
from rob import *
import rclpy
rclpy.init(); node = rclpy.create_node("m")
d = np.load("/workspace/robot0_eye_in_hand_depth.npy")
res = tf_lookup(node, "robot0_eye_in_hand_optical_frame")
K = [312.77408948188935,0,320,0,312.77408948188935,240,0,0,1]
P = cloud_from_depth(d, K, *res)
POT=(-0.196,-0.200)
Q = P[(np.abs(P[:,0]-POT[0])<0.09)&(np.abs(P[:,1]-POT[1])<0.09)]
for z0 in np.arange(0.90,1.07,0.005):
    s = Q[(Q[:,2]>=z0)&(Q[:,2]<z0+0.005)]
    if len(s)<5: continue
    print(f"z {z0:.3f}-{z0+0.005:.3f} n={len(s):5d} x[{s[:,0].min():.4f},{s[:,0].max():.4f}] w={s[:,0].max()-s[:,0].min():.4f}  y[{s[:,1].min():.4f},{s[:,1].max():.4f}] w={s[:,1].max()-s[:,1].min():.4f}")
# the pot top seen as a mask: pixels with z>1.02 : the x-extent along rows through the center
top = (P[:,2]>1.02).reshape(d.shape)
ys,xs = np.where(top)
print("top mask px bbox", xs.min(), xs.max(), ys.min(), ys.max(), "cam height above table", res[0][2]-0.90)
EOF
timeout 60 python3 measure.py

# openrua op 32
cat > stage3.py <<'EOF'
import numpy as np, sys
from rob import *
r = Robot("stage3")
C = np.array([-0.1967, -0.1995])
Q_G = (0.7071, -0.7071, 0, 0)
def go(tcp, seconds=3.0, seed=None, via_n=0):
    seed = r.arm_q() if seed is None else seed
    q = r.ik_tcp_world(tcp, Q_G, seed=seed, timeout=5)
    if q is None: raise SystemExit(f"IK fail {tcp}")
    via = []
    if via_n:
        cur, _ = r.tcp_world()
        s = seed
        for i in range(1, via_n+1):
            p = cur + (np.asarray(tcp)-cur)*i/(via_n+1)
            s = r.ik_tcp_world(p, Q_G, seed=s, timeout=5)
            if s is None: raise SystemExit(f"IK fail via {p}")
            via.append(s)
    code, err = r.move_q(q, seconds, via=via or None)
    if err > 0.01:
        print("  resending"); code, err = r.move_q(q, seconds)
    tcp_now, fq = r.tcp_world(); print("  tcp", np.round(tcp_now,4), "quat", np.round(fq,3), flush=True)
    return q
print("fingers", r.fingers())
go((C[0], C[1], 1.15), 3.0)
go((C[0], C[1], 1.048), 3.0, via_n=2)
r.snap("robot0_eye_in_hand", "/workspace/eih_pregrasp.png")
f = r.gripper(0.0)
r.snap("agentview", "/workspace/agentview_grasp.png")
go((C[0], C[1], 1.25), 3.0, via_n=1)
print("fingers after lift", r.fingers())
r.snap("agentview", "/workspace/agentview_lift.png"); r.snap("sideview", "/workspace/sideview_lift.png")
EOF
timeout 900 python3 -u stage3.py 2>&1 | tail -20

# openrua op 33
cat > stage4.py <<'EOF'
import numpy as np, sys
from rob import *
r = Robot("stage4")
def rot_y(th):
    c,s=np.cos(th),np.sin(th); return np.array([[c,0,s],[0,1,0],[-s,0,c]])
PITCH = np.radians(12)
Q_P = tuple(R_to_quat(rot_y(-PITCH) @ quat_to_R(1,0,0,0)))   # fingers along y, hand z tilted toward +x
HANG = 0.148    # TCP -> pot bottom (knob grasp), pot on table: TCP 1.048, bottom 0.90
PLATE = 0.932
Y_PLACE = float(sys.argv[1]) if len(sys.argv) > 1 else -0.008
X_BOTTOM = 0.185
Z_MARGIN = float(sys.argv[2]) if len(sys.argv) > 2 else 0.008
hz = np.array([np.sin(PITCH), 0, -np.cos(PITCH)])
tcp_place = np.array([X_BOTTOM, Y_PLACE, PLATE + Z_MARGIN + 0.037*np.sin(PITCH)]) - HANG*hz
print("place TCP", np.round(tcp_place,4), "expected bottom center", np.round(tcp_place + HANG*hz,4))

def go(tcp, quat, seconds=3.0, via=None):
    seed = r.arm_q()
    q = r.ik_tcp_world(tcp, quat, seed=seed, timeout=5)
    if q is None: raise SystemExit(f"IK fail {tcp}")
    vias = []
    if via:
        s = seed
        for p in via:
            s = r.ik_tcp_world(p, quat, seed=s, timeout=5)
            if s is None: raise SystemExit(f"IK fail via {p}")
            vias.append(s)
    code, err = r.move_q(q, seconds, via=vias or None)
    if err > 0.01:
        print("  resending"); code, err = r.move_q(q, seconds)
    tcp_now, fq = r.tcp_world(); R = quat_to_R(*fq)
    print("  tcp", np.round(tcp_now,4), "hz", np.round(R[:,2],3), "fingers", np.round(r.fingers(),4), flush=True)
    return q

cur, _ = r.tcp_world(); print("start tcp", np.round(cur,4))
pre = tcp_place + np.array([0, 0, 0.10])
mid = (cur + pre)/2; mid[2] = 1.28
go(pre, Q_P, 5.0, via=[mid])
r.snap("agentview", "/workspace/agentview_preplace.png")
go(tcp_place, Q_P, 3.0, via=[tcp_place + np.array([0,0,0.05])])
r.snap("sideview", "/workspace/sideview_place.png")
r.gripper(0.04)
go(tcp_place + np.array([0,0,0.12]), Q_P, 3.0)
r.snap("agentview", "/workspace/agentview_placed.png"); r.snap("sideview", "/workspace/sideview_placed.png"); r.snap("birdview", "/workspace/birdview_placed.png")
EOF
timeout 900 python3 -u stage4.py 2>&1 | tail -20

# openrua op 34
cat > stage5.py <<'EOF'
import numpy as np, sys
from rob import *
r = Robot("stage5")
C = np.array([-0.035, 0.2385])
Q_G = (0.7071, -0.7071, 0, 0)
def go(tcp, quat, seconds=3.0, via=None):
    seed = r.arm_q()
    q = r.ik_tcp_world(tcp, quat, seed=seed, timeout=5)
    if q is None: raise SystemExit(f"IK fail {tcp}")
    vias = []
    if via:
        s = seed
        for p in via:
            s = r.ik_tcp_world(p, quat, seed=s, timeout=5)
            if s is None: raise SystemExit(f"IK fail via {p}")
            vias.append(s)
    code, err = r.move_q(q, seconds, via=vias or None)
    if err > 0.01:
        print("  resending"); code, err = r.move_q(q, seconds)
    tcp_now, fq = r.tcp_world(); R = quat_to_R(*fq)
    print("  tcp", np.round(tcp_now,4), "hz", np.round(R[:,2],3), "fingers", np.round(r.fingers(),4), flush=True)
    return q
cur,_ = r.tcp_world()
mid = (cur + np.array([C[0],C[1],1.25]))/2; mid[2]=1.30
go((C[0], C[1], 1.25), Q_G, 5.0, via=[mid])
# birdview check of pot A on the stove
d = r.depth("birdview"); r.snap("birdview", "/workspace/birdview_s5.png")
fx=579.4112549695428; T=np.array([-0.2,0,3.0])
v,u = np.mgrid[0:480,0:640]
pc = np.stack([(u-320)*d/fx,(v-240)*d/fx,d],-1).reshape(-1,3)
P = np.stack([pc[:,1], pc[:,0], -pc[:,2]],-1) + T
stove = P[(P[:,0]>0.10)&(P[:,0]<0.31)&(P[:,1]>-0.06)&(P[:,1]<0.14)]
top = stove[(stove[:,2]>1.05)&(stove[:,2]<1.08)]
knob = stove[stove[:,2]>1.08]
print("pot A on stove: lid pts", len(top), "lid center", np.round(top[:,:2].mean(0),4) if len(top) else None, "lid z", np.round(np.median(top[:,2]),4) if len(top) else None,
      "knob center", np.round(knob[:,:2].mean(0),4) if len(knob) else None, "zmax", np.round(stove[:,2].max(),4))
# eye-in-hand refine pot B knob
dh = r.depth("robot0_eye_in_hand"); r.snap("robot0_eye_in_hand", "/workspace/eih_B.png")
res = tf_lookup(r.node, "robot0_eye_in_hand_optical_frame")
K = [312.77408948188935,0,320,0,312.77408948188935,240,0,0,1]
Ph = cloud_from_depth(dh, K, *res)
Q = Ph[(np.abs(Ph[:,0]-C[0])<0.09)&(np.abs(Ph[:,1]-C[1])<0.09)]
for z0 in [1.025,1.030,1.035,1.040,1.045,1.050,1.055]:
    s = Q[(Q[:,2]>=z0)&(Q[:,2]<z0+0.005)]
    if len(s)<5: continue
    print(f"z {z0:.3f} n={len(s):5d} x[{s[:,0].min():.4f},{s[:,0].max():.4f}] y[{s[:,1].min():.4f},{s[:,1].max():.4f}] cx={s[:,0].mean():.4f} cy={s[:,1].mean():.4f}")
knobB = Q[Q[:,2]>1.045]
print("knob B center", np.round(knobB[:,:2].mean(0),4), "zmax", np.round(Q[:,2].max(),4))
np.save("/workspace/knobB.npy", knobB[:,:2].mean(0))
EOF
timeout 900 python3 -u stage5.py 2>&1 | tail -20

# openrua op 35
cat > stage6.py <<'EOF'
import numpy as np, sys
from rob import *
r = Robot("stage6")
C = np.array([-0.0351, 0.2385])
Q_G = (0.7071, -0.7071, 0, 0)
def go(tcp, quat, seconds=3.0, via=None):
    seed = r.arm_q()
    q = r.ik_tcp_world(tcp, quat, seed=seed, timeout=5)
    if q is None: raise SystemExit(f"IK fail {tcp}")
    vias = []
    if via:
        s = seed
        for p in via:
            s = r.ik_tcp_world(p, quat, seed=s, timeout=5)
            if s is None: raise SystemExit(f"IK fail via {p}")
            vias.append(s)
    code, err = r.move_q(q, seconds, via=vias or None)
    tries = 0
    while err > 0.01 and tries < 3:
        print("  resending"); code, err = r.move_q(q, seconds); tries += 1
    tcp_now, fq = r.tcp_world(); R = quat_to_R(*fq)
    print("  tcp", np.round(tcp_now,4), "hz", np.round(R[:,2],3), "finger axis", np.round(R[:,1],3), "fingers", np.round(r.fingers(),4), flush=True)
    return q
print("q now", np.round(r.arm_q(),3))
go((C[0], C[1], 1.15), Q_G, 3.0)
go((C[0], C[1], 1.048), Q_G, 3.0, via=[(C[0], C[1], 1.10)])
r.snap("robot0_eye_in_hand", "/workspace/eih_B_pregrasp.png")
r.gripper(0.0)
go((C[0], C[1], 1.25), Q_G, 3.0, via=[(C[0], C[1], 1.15)])
print("fingers after lift", r.fingers())
r.snap("sideview", "/workspace/sideview_liftB.png")
EOF
timeout 900 python3 -u stage6.py 2>&1 | tail -20

# openrua op 36
timeout 900 python3 -u stage4.py 0.09 0.008 2>&1 | tail -20

# openrua op 37
cat > verify.py <<'EOF'
import numpy as np
from rob import *
r = Robot("verify")
# retreat to a neutral pose that clears the birdview of the stove
home = np.array([0.0, -0.3, 0.0, -2.2, 0.0, 1.9, 0.785])
r.move_q(home, 4.0)
for c in ["birdview","agentview","frontview","sideview"]:
    r.snap(c, f"/workspace/{c}_final.png")
d = r.depth("birdview")
fx=579.4112549695428; T=np.array([-0.2,0,3.0])
v,u = np.mgrid[0:480,0:640]
pc = np.stack([(u-320)*d/fx,(v-240)*d/fx,d],-1).reshape(-1,3)
P = np.stack([pc[:,1], pc[:,0], -pc[:,2]],-1) + T
tab = P[(P[:,0]>-0.4)&(P[:,0]<0.45)&(P[:,1]>-0.4)&(P[:,1]<0.4)]
plate = tab[(tab[:,2]>0.92)&(tab[:,2]<0.94)]
print("stove plate footprint x[%.3f,%.3f] y[%.3f,%.3f]" % (plate[:,0].min(), plate[:,0].max(), plate[:,1].min(), plate[:,1].max()))
import cv2
Z = P[:,2].reshape(480,640)
mask = ((Z>1.04)&(Z<1.10)).astype(np.uint8)
n, lab, stats, cent = cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i][4] < 50: continue
    ys,xs = np.where(lab==i); pts = P.reshape(480,640,3)[ys,xs]
    lid = pts[(pts[:,2]>1.055)&(pts[:,2]<1.07)]
    print(f"pot lid blob: center xy {np.round(pts[:,:2].mean(0),4)} z lid median {np.median(pts[:,2]):.4f} zmax {pts[:,2].max():.4f} n={len(pts)}")
# anything left at table height above 0.95 outside the stove region? (pots still on table?)
rest = tab[(tab[:,2]>0.95)&~((tab[:,0]>0.0)&(tab[:,0]<0.31)&(tab[:,1]>-0.07)&(tab[:,1]<0.15))]
print("objects >5cm above table outside stove region:", len(rest), "pts", np.round(rest[:,:2].mean(0),3) if len(rest) else "")
EOF
timeout 600 python3 -u verify.py 2>&1 | tail

# openrua op 38
cat > settle.py <<'EOF'
import numpy as np
from rob import *
r = Robot("settle")
home = np.array([0.0, -0.3, 0.0, -2.2, 0.0, 1.9, 0.785])
for _ in range(3):
    code, err = r.move_q(home, 4.0)
    if err < 0.01: break
print("final q", np.round(r.arm_q(),3), "fingers", np.round(r.fingers(),4))
d = r.depth("birdview")
fx=579.4112549695428; T=np.array([-0.2,0,3.0])
v,u = np.mgrid[0:480,0:640]
pc = np.stack([(u-320)*d/fx,(v-240)*d/fx,d],-1).reshape(-1,3)
P = np.stack([pc[:,1], pc[:,0], -pc[:,2]],-1) + T
Z = P[:,2].reshape(480,640)
import cv2
mask = ((Z>1.04)&(Z<1.10)).astype(np.uint8); mask[:150,:]=0
n, lab, stats, cent = cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i][4] < 50: continue
    ys,xs = np.where(lab==i); pts = P.reshape(480,640,3)[ys,xs]
    print(f"pot lid: center xy {np.round(pts[:,:2].mean(0),4)} lid z {np.median(pts[:,2]):.4f}")
tab = P[(P[:,0]>-0.45)&(P[:,0]<0.0)&(P[:,1]>-0.4)&(P[:,1]<0.4)&(P[:,2]>0.95)&(P[:,2]<1.2)]
print("pts 5-30cm above table in the original pot area (x<0):", len(tab))
r.snap("agentview", "/workspace/agentview_final.png")
EOF
timeout 600 python3 -u settle.py 2>&1 | tail
