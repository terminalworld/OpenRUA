#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/action/fjt_send.py; echo ----; cat /workspace/tools/action/ik_move.py; echo ----; cat /workspace/tools/action/gripper_cmd.py; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
timeout 30 ros2 topic list; echo ---ACTIONS; timeout 30 ros2 action list; echo ---SERVICES; timeout 30 ros2 service list | head -80; echo ---NODES; timeout 30 ros2 node list

# openrua op 3
timeout 20 ros2 topic echo /joint_states --once; echo ---; timeout 20 ros2 topic info /pipeline_state -v | head -30; timeout 10 ros2 topic echo /pipeline_state --once

# openrua op 4
mkdir -p snaps && for c in agentview frontview sideview birdview robot0_eye_in_hand robot0_robotview; do timeout 60 python3 tools/perception/cam_snap.py $c snaps/$c.png; done; ls -la snaps

# openrua op 5
timeout 20 ros2 topic echo /tf_static --once 2>/dev/null | grep -E "frame_id|child_frame_id" | head -60; echo ---; timeout 15 ros2 topic echo /agentview/color/camera_info --once | head -30

# openrua op 6
cat > tfdump.py <<'EOF'
import rclpy, sys
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
rclpy.init(); node = rclpy.create_node("tfdump")
seen = {}
def cb(msg):
    for t in msg.transforms:
        seen[(t.header.frame_id, t.child_frame_id)] = t.transform
qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
node.create_subscription(TFMessage, "/tf_static", cb, qos)
node.create_subscription(TFMessage, "/tf", cb, 100)
import time
t0=time.time()
while time.time()-t0 < 4: rclpy.spin_once(node, timeout_sec=0.2)
for (p,c),tr in sorted(seen.items()):
    print(f"{p} -> {c}: t=({tr.translation.x:.4f},{tr.translation.y:.4f},{tr.translation.z:.4f}) q=({tr.rotation.x:.4f},{tr.rotation.y:.4f},{tr.rotation.z:.4f},{tr.rotation.w:.4f})")
EOF
timeout 30 python3 tfdump.py

# openrua op 7
timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw snaps/birdview_depth.png && cat > bird.py <<'EOF'
import numpy as np, yaml
d = np.load("snaps/birdview_depth.npy")
print(d.shape, np.nanmin(d), np.nanmax(d))
# birdview: cam at (-0.2,0,3.0), optical z down. q=(0.7071,0.7071,0,0): R = rotation... compute
fx=fy=579.4112549695428; cx=320; cy=240
x,y,z,w = 0.7071,0.7071,0,0
R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
t = np.array([-0.2,0,3.0])
def px2w(u,v):
    Z = d[v,u]; p = np.array([(u-cx)*Z/fx,(v-cy)*Z/fy,Z]); return R@p+t
# table height: center region
for (u,v) in [(320,240),(200,350),(500,350),(240,280),(330,310),(377,283),(377,240),(300,240),(320,200)]:
    print((u,v), d[v,u], px2w(u,v).round(4))
# height map
H = np.zeros_like(d)
for v in range(0,480,1):
    for u in range(0,640,1):
        H[v,u] = px2w(u,v)[2]
np.save("snaps/bird_H.npy", H)
tab = np.median(H[300:400, 200:450])
print("table z ~", tab)
mask = H > tab+0.01
import cv2
cv2.imwrite("snaps/bird_mask.png", (mask*255).astype(np.uint8))
EOF
python3 bird.py

# openrua op 8
cat > blobs.py <<'EOF'
import numpy as np, cv2
H = np.load("snaps/bird_H.npy")
fx=579.4112549695428; cx=320; cy=240
def px2w(u,v):
    # birdview straight down: from previous mapping
    Z = 3.0 - H[v,u]
    return np.array([-0.2 + (v-cy)*Z/fx, (u-cx)*Z/fx*-1, H[v,u]])
# verify mapping sign using earlier sample: (200,350)->(0.1987,-0.4349)
print(px2w(200,350))
tab=0.9001
mask = ((H > tab+0.008) & (H < 1.2)).astype(np.uint8)
mask[:, :] = mask
n, lab, stats, cents = cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    a = stats[i, cv2.CC_STAT_AREA]
    if a < 30: continue
    ys,xs = np.where(lab==i)
    hs = H[ys,xs]
    u,v = cents[i]
    w = px2w(int(u),int(v))
    # world extents
    W = np.array([px2w(x,y) for x,y in zip(xs[::5],ys[::5])])
    print(f"blob{i} area={a} px_c=({u:.0f},{v:.0f}) bbox={stats[i][:4]} zmax={hs.max():.3f} zmed={np.median(hs):.3f} x[{W[:,0].min():.3f},{W[:,0].max():.3f}] y[{W[:,1].min():.3f},{W[:,1].max():.3f}]")
EOF
python3 blobs.py

# openrua op 9
cat > crops.py <<'EOF'
import numpy as np, cv2
img = cv2.imread("snaps/birdview.png"); H = np.load("snaps/bird_H.npy")
def crop(name, u0,v0,u1,v1, s=6):
    c = img[v0:v1, u0:u1]; c = cv2.resize(c, None, fx=s, fy=s, interpolation=cv2.INTER_NEAREST)
    cv2.imwrite(f"snaps/crop_{name}.png", c)
crop("knob", 355,220,400,262, 10)
crop("pan", 205,245,325,320, 5)
crop("stove", 345,250,410,315, 8)
np.set_printoptions(linewidth=250, precision=3, suppress=True)
print("knob heights (rows 228..254, cols 364..390):")
print(H[228:255:2, 364:391:2].round(3))
EOF
python3 crops.py

# openrua op 10
cat > pan.py <<'EOF'
import numpy as np
H = np.load("snaps/bird_H.npy")
np.set_printoptions(linewidth=300, precision=3, suppress=True)
fx=579.4112549695428; cx=320; cy=240
def px2w(u,v):
    Z = 3.0 - H[v,u]; return np.array([-0.2 + (v-cy)*Z/fx, (u-cx)*Z/fx, H[v,u]])
print("pan rows 250..312 step 3, cols 212..320 step 4")
print(H[250:313:3, 212:321:4].round(3))
# pan body: circular region cols 214..280, find rim center via pixels with H>0.95 within cols<285
ys,xs = np.where((H[245:320, 205:290] > 0.95))
ys+=245; xs+=205
print("rim px center", xs.mean(), ys.mean(), "px extents", xs.min(), xs.max(), ys.min(), ys.max())
print("rim world center", px2w(int(xs.mean()), int(ys.mean())).round(3))
# handle: cols 285..320
ys,xs = np.where((H[245:320, 283:325] > 0.91))
ys+=245; xs+=283
print("handle px", xs.min(), xs.max(), ys.min(), ys.max(), "heights", H[ys,xs].min(), H[ys,xs].max())
for u in range(285,322,4):
    col = H[260:300,u]; v = 260+np.argmax(col)
    print(u, v, px2w(u,v).round(3))
EOF
python3 pan.py

# openrua op 11
mkdir -p "$(dirname /workspace/rob.py)"
cat > /workspace/rob.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Small helper library: one node, reusable clients.

    from rob import R
    r = R(); r.joints(); r.ik(x,y,z,qx,qy,qz,qw, at_tcp=True); r.move(q, secs)
    r.grip(width); r.fk()
World<->base: base = world - (-0.66, 0, 0.912) (no rotation, from TF).
"""
import sys, time
import numpy as np
import rclpy, yaml
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK, GetPositionFK

BASE = np.array([-0.66, 0.0, 0.912])
M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]
TCP = float(M["hand"]["tcp_offset_m"])


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class R:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("rob_" + str(int(time.time() * 1000) % 100000))
        self._js = None
        self.node.create_subscription(JointState, "/joint_states", self._cb, 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.gr = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ikc = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fkc = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt.wait_for_server(10); self.gr.wait_for_server(10)
        self.ikc.wait_for_service(10); self.fkc.wait_for_service(10)

    def _cb(self, m):
        self._js = m

    def spin(self, t=0.05):
        rclpy.spin_once(self.node, timeout_sec=t)

    def js(self):
        self._js = None
        while self._js is None:
            self.spin(0.2)
        return dict(zip(self._js.name, self._js.position))

    def joints(self):
        j = self.js()
        return [j[n] for n in ARM]

    def fingers(self):
        j = self.js()
        return j["panda_finger_joint1"], j["panda_finger_joint2"]

    def _seed(self):
        s = JointState()
        j = self.js()
        for n in ARM:
            s.name.append(n); s.position.append(j[n])
        return s

    def fk(self):
        """hand pose in WORLD (pos, quat)."""
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state = self._seed()
        fut = self.fkc.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE
        q = (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        return pos, q

    def tcp(self):
        pos, q = self.fk()
        return pos + TCP * quat_R(*q)[:, 2], q

    def ik(self, x, y, z, qx, qy, qz, qw, at_tcp=True, seed=None):
        """world pose -> arm joints (list) or None."""
        p = np.array([x, y, z], float)
        if at_tcp:
            p = p - TCP * quat_R(qx, qy, qz, qw)[:, 2]
        p = p - BASE
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        ps = req.ik_request.pose_stamped.pose
        ps.position.x, ps.position.y, ps.position.z = map(float, p)
        ps.orientation.x, ps.orientation.y, ps.orientation.z, ps.orientation.w = qx, qy, qz, qw
        if seed is None:
            req.ik_request.robot_state.joint_state = self._seed()
        else:
            s = JointState(); s.name = list(ARM); s.position = [float(v) for v in seed]
            req.ik_request.robot_state.joint_state = s
        req.ik_request.timeout.sec = 2
        fut = self.ikc.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            print("IK failed", None if res is None else res.error_code.val, file=sys.stderr)
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in ARM]

    def move(self, q, secs=3.0, wait=True):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pt = JointTrajectoryPoint(positions=[float(v) for v in q])
        pt.time_from_start = Duration(sec=int(secs), nanosec=int((secs % 1) * 1e9))
        goal.trajectory.points = [pt]
        fut = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=120)
        h = fut.result()
        rf = h.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=600)
        code = rf.result().result.error_code
        err = np.abs(np.array(self.joints()) - np.array(q)).max()
        print(f"move done code={code} max_err={err:.4f}", flush=True)
        return code

    def move_path(self, qs, secs_each=2.0):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        t = 0.0
        for q in qs:
            t += secs_each
            pt = JointTrajectoryPoint(positions=[float(v) for v in q])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        fut = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=120)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=600)
        code = rf.result().result.error_code
        err = np.abs(np.array(self.joints()) - np.array(qs[-1])).max()
        print(f"path done code={code} max_err={err:.4f}", flush=True)
        return code

    def grip(self, width):
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = float(GRIP["max_effort"])
        fut = self.gr.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result
        print(f"grip -> pos={r.position:.4f} reached={r.reached_goal} stalled={r.stalled} fingers={self.fingers()}", flush=True)
        return r

    def goto(self, x, y, z, q, secs=3.0, at_tcp=True):
        j = self.ik(x, y, z, *q, at_tcp=at_tcp)
        if j is None:
            return None
        code = self.move(j, secs)
        p, _ = self.tcp()
        print(f"tcp now {p.round(4)} target {np.array([x,y,z]).round(4)}", flush=True)
        return code
OPENRUA_EOF

# openrua op 12
cat > s0.py <<'EOF'
from rob import *
r = R()
print("joints", np.round(r.joints(),3))
print("fk hand", r.fk())
print("tcp", r.tcp())
print("fingers", r.fingers())
# test IK for key poses
Q_DOWN_Y = (1.0, 0.0, 0.0, 0.0)          # fingers along world y
Q_DOWN_X = (0.7071068, 0.7071068, 0.0, 0.0)  # fingers along world x
for name,(x,y,z,q) in {"knob_pre":(-0.197,0.201,1.05,Q_DOWN_Y),
                       "knob":(-0.197,0.201,0.95,Q_DOWN_Y),
                       "pan_pre":(-0.066,-0.085,1.15,Q_DOWN_X),
                       "pan":(-0.066,-0.085,1.005,Q_DOWN_X),
                       "stove_place":(-0.044,0.383,1.04,Q_DOWN_X),
                       "stove_place_alt":(-0.224,0.203,1.04,Q_DOWN_Y)}.items():
    j = r.ik(x,y,z,*q)
    print(name, None if j is None else np.round(j,3))
EOF
timeout 300 python3 s0.py

# openrua op 13
cat > s1.py <<'EOF'
from rob import *
r = R()
req = GetPositionFK.Request()
req.header.frame_id = ""
req.fk_link_names = ["panda_hand", "panda_link0", "panda_link8"]
req.robot_state.joint_state = r._seed()
print(req.robot_state.joint_state)
fut = r.fkc.call_async(req)
rclpy.spin_until_future_complete(r.node, fut, timeout_sec=30)
res = fut.result()
print(res.error_code, res.fk_link_names)
for ps in res.pose_stamped: print(ps.header.frame_id, ps.pose.position)
# IK without timeout, like ik_move.py
req = GetPositionIK.Request()
req.ik_request.group_name = "panda_arm"
req.ik_request.pose_stamped.header.frame_id = ""
p = req.ik_request.pose_stamped.pose
p.position.x, p.position.y, p.position.z = 0.4, 0.0, 0.4
p.orientation.x = 1.0
req.ik_request.robot_state.joint_state = r._seed()
fut = r.ikc.call_async(req)
rclpy.spin_until_future_complete(r.node, fut, timeout_sec=60)
res = fut.result(); print("ik", res.error_code, res.solution.joint_state.name, np.round(res.solution.joint_state.position,3))
EOF
timeout 300 python3 s1.py

# openrua op 14
python3 - <<'EOF'
s = open("rob.py").read()
s = s.replace("        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE\n", "        pos = np.array([p.position.x, p.position.y, p.position.z])  # FK already reports world\n")
s = s.replace("        p = p - BASE\n", "        # IK model frame == world here (FK reports panda_link0 at BASE)\n")
open("rob.py","w").write(s)
EOF
timeout 300 python3 s0.py

# openrua op 15
cat > s2.py <<'EOF'
from rob import *
import subprocess
r = R()
Q_DOWN_Y = (1.0, 0.0, 0.0, 0.0)
r.grip(0.04)
print("goto knob_pre", r.goto(-0.197, 0.201, 1.05, Q_DOWN_Y, 4.0))
print("joints", np.round(r.joints(),3))
EOF
timeout 900 python3 -u s2.py > s2.log 2>&1; cat s2.log

# openrua op 16
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih1.png && timeout 60 python3 tools/perception/cam_snap.py sideview snaps/side1.png

# openrua op 17
cat > s3.py <<'EOF'
import rclpy, numpy as np, time
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy
from rob import quat_R
rclpy.init(); node = rclpy.create_node("tfchk")
seen = {}
def cb(msg):
    for t in msg.transforms: seen[(t.header.frame_id, t.child_frame_id)] = t.transform
node.create_subscription(TFMessage, "/tf", cb, 100)
node.create_subscription(TFMessage, "/tf_static", cb, QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))
t0=time.time()
while time.time()-t0<3: rclpy.spin_once(node, timeout_sec=0.2)
tr = seen[("world","robot0_eye_in_hand_optical_frame")]
q = tr.rotation; R = quat_R(q.x,q.y,q.z,q.w)  # cam->world
print("cam pos", tr.translation)
# image directions: cam x = image right, cam y = image down, cam z = optical
for name, wv in [("world x", [1,0,0]), ("world y",[0,1,0]), ("world z",[0,0,1])]:
    cv = R.T @ np.array(wv)
    print(name, "-> cam (right,down,fwd) =", cv.round(3))
EOF
timeout 60 python3 s3.py

# openrua op 18
cat > s4.py <<'EOF'
from rob import *
r = R()
pos,q = r.fk(); Rm = quat_R(*q)
print("hand pos", pos.round(4), "q", np.round(q,4))
print("hand x in world", Rm[:,0].round(3)); print("hand y (finger axis) in world", Rm[:,1].round(3)); print("hand z", Rm[:,2].round(3))
# also FK of the fingers
req = GetPositionFK.Request(); req.header.frame_id=""
req.fk_link_names = ["panda_leftfinger","panda_rightfinger","panda_link8"]
req.robot_state.joint_state = r._seed()
fut = r.fkc.call_async(req); rclpy.spin_until_future_complete(r.node, fut, timeout_sec=30)
for n,ps in zip(fut.result().fk_link_names, fut.result().pose_stamped): print(n, np.round([ps.pose.position.x,ps.pose.position.y,ps.pose.position.z],4))
EOF
timeout 120 python3 s4.py

# openrua op 19
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rob.py", "old_string": "        req.ik_request.group_name = M[\\"planning\\"][\\"group\\"]\\n        req.ik_request.pose_stamped.header.frame_id = \\"\\"", "new_string": "        req.ik_request.group_name = M[\\"planning\\"][\\"group\\"]\\n        req.ik_request.ik_link_name = \\"panda_hand\\"  # default tip is link8 (45deg off the hand)\\n        req.ik_request.pose_stamped.header.frame_id = \\"\\"", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 20
cat > s5.py <<'EOF'
from rob import *
r = R()
Q_DOWN_Y = (1.0, 0.0, 0.0, 0.0)
print("goto knob_pre", r.goto(-0.197, 0.201, 1.05, Q_DOWN_Y, 3.0))
pos,q = r.fk(); Rm = quat_R(*q)
print("hand q", np.round(q,4), "finger axis", Rm[:,1].round(3))
print("joints", np.round(r.joints(),3))
EOF
timeout 900 python3 -u s5.py > s5.log 2>&1; cat s5.log; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih2.png

# openrua op 21
cat > s6.py <<'EOF'
from rob import *
r = R()
Q_DOWN_Y = (1.0, 0.0, 0.0, 0.0)
j = r.ik(-0.197, 0.201, 1.05, *Q_DOWN_Y)
print("target", np.round(j,3))
for i in range(3):
    code = r.move(j, 3.0)
    if code == 0: break
pos,q = r.fk(); Rm = quat_R(*q)
print("hand q", np.round(q,4), "finger axis", Rm[:,1].round(3), "tcp", r.tcp()[0].round(4))
print("joints", np.round(r.joints(),3))
EOF
timeout 900 python3 -u s6.py > s6.log 2>&1; cat s6.log

# openrua op 22
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih3.png

# openrua op 23
cat > s7.py <<'EOF'
from rob import *
r = R()
Q = (1.0, 0.0, 0.0, 0.0)
for z in (0.98, 0.945):
    j = r.ik(-0.197, 0.201, z, *Q)
    for i in range(3):
        if r.move(j, 2.0) == 0: break
    print("tcp", r.tcp()[0].round(4))
r.grip(0.0)
print("fingers", r.fingers())
EOF
timeout 900 python3 -u s7.py > s7.log 2>&1; cat s7.log; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih4.png; timeout 60 python3 tools/perception/cam_snap.py sideview snaps/side2.png

# openrua op 24
cat > s8.py <<'EOF'
from rob import *
import sys
r = R()
d = float(sys.argv[1])
j = r.joints(); j[6] += d
print("target q7", j[6])
for i in range(3):
    if r.move(j, 3.0) == 0: break
print("joints", np.round(r.joints(),3), "fingers", r.fingers())
EOF
timeout 900 python3 -u s8.py -1.57 > s8.log 2>&1; cat s8.log; timeout 60 python3 tools/perception/cam_snap.py agentview snaps/agent2.png; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih5.png

# openrua op 25
cat > s9.py <<'EOF'
from rob import *
r = R()
r.grip(0.04)
j = r.ik(-0.197, 0.201, 1.08, 1.0,0,0,0)  # straight up, hand yaw back to nominal
for i in range(3):
    if r.move(j, 3.0) == 0: break
print("tcp", r.tcp()[0].round(4), "joints", np.round(r.joints(),3))
EOF
timeout 900 python3 -u s9.py > s9.log 2>&1; cat s9.log; timeout 60 python3 tools/perception/cam_snap.py agentview snaps/agent3.png

# openrua op 26
python3 - <<'EOF'
s = open("rob.py").read()
s = s.replace("ps.orientation.x, ps.orientation.y, ps.orientation.z, ps.orientation.w = qx, qy, qz, qw", "ps.orientation.x, ps.orientation.y, ps.orientation.z, ps.orientation.w = map(float, (qx, qy, qz, qw))")
open("rob.py","w").write(s)
EOF
cat > s9.py <<'EOF'
from rob import *
r = R()
j = r.ik(-0.197, 0.201, 1.08, 1.0,0,0,0)
for i in range(3):
    if r.move(j, 3.0) == 0: break
print("tcp", r.tcp()[0].round(4), "joints", np.round(r.joints(),3), "fingers", r.fingers())
EOF
timeout 900 python3 -u s9.py > s9.log 2>&1; cat s9.log; timeout 60 python3 tools/perception/cam_snap.py agentview snaps/agent3.png

# openrua op 27
cat > s10.py <<'EOF'
from rob import *
r = R()
QX = (0.7071068, 0.7071068, 0.0, 0.0)
j = r.ik(-0.063, -0.075, 1.15, *QX)
print("target", np.round(j,3))
for i in range(4):
    if r.move(j, 4.0) == 0: break
pos,q = r.fk(); Rm = quat_R(*q)
print("tcp", r.tcp()[0].round(4), "finger axis", Rm[:,1].round(3), "joints", np.round(r.joints(),3))
EOF
timeout 900 python3 -u s10.py > s10.log 2>&1; cat s10.log; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih6.png

# openrua op 28
cat > s11.py <<'EOF'
from rob import *
r = R()
QX = (0.7071068, 0.7071068, 0.0, 0.0)
j = r.ik(-0.063, -0.075, 1.035, *QX)
for i in range(4):
    if r.move(j, 3.0) == 0: break
print("tcp", r.tcp()[0].round(4), "joints", np.round(r.joints(),3))
EOF
timeout 900 python3 -u s11.py > s11.log 2>&1; cat s11.log; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih7.png; timeout 60 python3 tools/perception/cam_snap.py frontview snaps/front2.png

# openrua op 29
cat > s12.py <<'EOF'
from rob import *
r = R()
QX = (0.7071068, 0.7071068, 0.0, 0.0)
j = r.ik(-0.063, -0.075, 1.008, *QX)
for i in range(4):
    if r.move(j, 2.0) == 0: break
print("tcp", r.tcp()[0].round(4))
r.grip(0.0)
print("fingers", r.fingers())
EOF
timeout 900 python3 -u s12.py > s12.log 2>&1; cat s12.log; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih8.png; timeout 60 python3 tools/perception/cam_snap.py sideview snaps/side3.png

# openrua op 30
cat > s13.py <<'EOF'
from rob import *
r = R()
QX = (0.7071068, 0.7071068, 0.0, 0.0)
j = r.ik(-0.063, -0.075, 1.25, *QX)
for i in range(4):
    if r.move(j, 3.0) == 0: break
print("tcp", r.tcp()[0].round(4), "fingers", r.fingers())
EOF
timeout 900 python3 -u s13.py > s13.log 2>&1; cat s13.log; timeout 60 python3 tools/perception/cam_snap.py frontview snaps/front3.png

# openrua op 31
timeout 60 python3 tools/perception/cam_snap.py /frontview/depth/image_raw snaps/front_depth.png && cat > handle.py <<'EOF'
import numpy as np
from rob import quat_R
d = np.load("snaps/front_depth.npy")
fx=579.4112549695428; cx=320; cy=240
# frontview optical pose
t = np.array([1.0,0.0,1.48]); R = quat_R(0.5608,0.5608,-0.4306,-0.4306)
def px2w(u,v):
    Z=d[v,u]; return R@np.array([(u-cx)*Z/fx,(v-cy)*Z/fx,Z])+t
np.set_printoptions(linewidth=250, precision=3, suppress=True)
# scan columns across the handle; for each column print world points where x is near -0.06 (handle) 
for u in range(236, 306, 6):
    pts=[]
    for v in range(330, 380):
        w = px2w(u,v)
        pts.append((v, w.round(3)))
    # handle pixels: world x between -0.12 and 0.0 and z>0.95
    hp = [(v,w) for v,w in pts if -0.13 < w[0] < 0.0 and w[2] > 0.95]
    if hp:
        print(f"u={u}: rows {hp[0][0]}..{hp[-1][0]} top {hp[0][1]} bottom {hp[-1][1]}")
    else:
        print(f"u={u}: none; sample", pts[10][1], pts[30][1])
EOF
python3 handle.py

# openrua op 32
timeout 60 python3 tools/perception/cam_snap.py /sideview/depth/image_raw snaps/side_depth.png && cat > xcheck.py <<'EOF'
import numpy as np
from rob import quat_R
fx=579.4112549695428; cx=320; cy=240
cams = {"front": ("snaps/front_depth.npy", np.array([1.0,0.0,1.48]), quat_R(0.5608,0.5608,-0.4306,-0.4306)),
        "side": ("snaps/side_depth.npy", np.array([-0.0565,1.2761,1.488]), quat_R(0.0099,0.8064,-0.5912,-0.0069)),
        "bird": ("snaps/birdview_depth.npy", np.array([-0.2,0,3.0]), quat_R(0.7071,0.7071,0,0))}
for name,(f,t,R) in cams.items():
    d = np.load(f)
    P = np.zeros((480,640,3))
    for v in range(480):
        Z = d[v,:]
        u = np.arange(640)
        pc = np.stack([(u-cx)*Z/fx, (v-cy)*Z/fx, Z], 1)
        P[v] = pc@R.T + t
    np.save(f"snaps/P_{name}.npy", P)
    # table height
    tab = np.median(P[...,2][(np.abs(P[...,0]+0.3)<0.1)&(np.abs(P[...,1]-0.4)<0.1)])
    # pan region: within 0.14 of (-0.064,-0.266) in xy
    dpan = np.hypot(P[...,0]+0.064, P[...,1]+0.266)
    pan = P[...,2][(dpan<0.14)]
    # knob region within 0.045 of (-0.197,0.201)
    dk = np.hypot(P[...,0]+0.197, P[...,1]-0.201); knob = P[...,2][dk<0.045]
    # stove region within 0.08 of (-0.044,0.203) 
    ds = np.hypot(P[...,0]+0.044, P[...,1]-0.203); stove = P[...,2][ds<0.08]
    # moka
    dm = np.hypot(P[...,0]-0.03, P[...,1]-0.01); moka = P[...,2][dm<0.05]
    # handle: x in [-0.09,-0.04], y in [-0.12,-0.03]
    hm = (P[...,0]>-0.09)&(P[...,0]<-0.04)&(P[...,1]>-0.12)&(P[...,1]<-0.03)
    handle = P[...,2][hm]
    f2 = lambda a: f"n={a.size} max={a.max():.3f} p95={np.percentile(a,95):.3f}" if a.size else "none"
    print(f"{name}: table={tab:.3f} pan[{f2(pan)}] knob[{f2(knob)}] stove[{f2(stove)}] moka[{f2(moka)}] handle[{f2(handle)}]")
EOF
python3 xcheck.py

# openrua op 33
cat > handle2.py <<'EOF'
import numpy as np
np.set_printoptions(linewidth=250, precision=3, suppress=True)
P = np.load("snaps/P_front.npy")
for u in (272, 278, 284, 302):
    print("u",u)
    for v in range(352, 368): print(v, P[v,u].round(3))
S = np.load("snaps/P_side.npy")
# side: handle seen end-on; find pixels with y in [-0.12,-0.03], x in [-0.09,-0.04], z<1.2
m = (S[...,0]>-0.10)&(S[...,0]<-0.03)&(S[...,1]>-0.13)&(S[...,1]<-0.02)&(S[...,2]<1.1)&(S[...,2]>0.905)
pts = S[m]; print("side handle pts", pts.shape, "z range", pts[:,2].min(), pts[:,2].max(), "x range", pts[:,0].min(), pts[:,0].max())
import collections
hist = np.histogram(pts[:,2], bins=np.arange(0.90,0.98,0.005)); print(hist)
EOF
python3 handle2.py

# openrua op 34
cat > s14.py <<'EOF'
from rob import *
r = R()
QX = (0.7071068, 0.7071068, 0.0, 0.0)
r.grip(0.04)
for z,secs in ((1.05,3.0),(0.948,2.5)):
    j = r.ik(-0.058, -0.075, z, *QX)
    for i in range(4):
        if r.move(j, secs) == 0: break
    print("tcp", r.tcp()[0].round(4))
r.grip(0.0)
# settle ticks: tiny hold trajectory to let the sim advance
j = r.joints(); r.move(j, 1.0)
print("fingers after settle", r.fingers())
EOF
timeout 900 python3 -u s14.py > s14.log 2>&1; cat s14.log; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih9.png

# openrua op 35
timeout 60 python3 tools/perception/cam_snap.py frontview snaps/front4.png; timeout 60 python3 tools/perception/cam_snap.py sideview snaps/side4.png

# openrua op 36
cat > s15.py <<'EOF'
from rob import *
r = R()
QX = (0.7071068, 0.7071068, 0.0, 0.0)
r.grip(0.04)
j = r.ik(-0.20, -0.075, 1.20, *QX)
for i in range(4):
    if r.move(j, 3.0) == 0: break
print("tcp", r.tcp()[0].round(4), "fingers", r.fingers())
EOF
timeout 900 python3 -u s15.py > s15.log 2>&1; cat s15.log
for c in frontview sideview; do timeout 60 python3 tools/perception/cam_snap.py /$c/depth/image_raw snaps/${c}_depth2.png; timeout 60 python3 tools/perception/cam_snap.py $c snaps/${c}_2.png; done

# openrua op 37
cat > pan2.py <<'EOF'
import numpy as np
from rob import quat_R
np.set_printoptions(linewidth=250, precision=3, suppress=True)
fx=579.4112549695428; cx=320; cy=240
cams = {"front": ("snaps/frontview_depth2.npy", np.array([1.0,0.0,1.48]), quat_R(0.5608,0.5608,-0.4306,-0.4306)),
        "side": ("snaps/sideview_depth2.npy", np.array([-0.0565,1.2761,1.488]), quat_R(0.0099,0.8064,-0.5912,-0.0069))}
allpts = []
for name,(f,t,R) in cams.items():
    d = np.load(f)
    P = np.zeros((480,640,3))
    for v in range(480):
        Z = d[v,:]; u = np.arange(640)
        P[v] = np.stack([(u-cx)*Z/fx, (v-cy)*Z/fx, Z], 1)@R.T + t
    np.save(f"snaps/P2_{name}.npy", P)
    pts = P.reshape(-1,3)
    # pan region: above table, within a box
    m = (pts[:,2]>0.905)&(pts[:,2]<1.0)&(pts[:,0]>-0.25)&(pts[:,0]<0.1)&(pts[:,1]>-0.45)&(pts[:,1]<0.0)
    p = pts[m]; allpts.append(p)
    print(name, "pan pts", p.shape, "z max", p[:,2].max().round(3))
    # rim: points with z>0.935 and y<-0.14
    rim = p[(p[:,2]>0.935)&(p[:,1]<-0.14)]
    print("  rim x range", rim[:,0].min().round(3), rim[:,0].max().round(3), "y range", rim[:,1].min().round(3), rim[:,1].max().round(3), "center", ((rim[:,0].min()+rim[:,0].max())/2).round(3), ((rim[:,1].min()+rim[:,1].max())/2).round(3))
    # handle: y > -0.135
    h = p[(p[:,1]>-0.13)&(p[:,2]>0.92)]
    if h.size:
        print("  handle x range", h[:,0].min().round(3), h[:,0].max().round(3), "y range", h[:,1].min().round(3), h[:,1].max().round(3), "z range", h[:,2].min().round(3), h[:,2].max().round(3))
        for y0 in np.arange(-0.12,-0.02,0.02):
            s = h[(h[:,1]>=y0)&(h[:,1]<y0+0.02)]
            if s.size: print(f"   y[{y0:.2f},{y0+0.02:.2f}] x[{s[:,0].min():.3f},{s[:,0].max():.3f}] z[{s[:,2].min():.3f},{s[:,2].max():.3f}] n={len(s)}")
EOF
python3 pan2.py

# openrua op 38
cat > s16.py <<'EOF'
from rob import *
r = R()
QX = (0.7071068, 0.7071068, 0.0, 0.0)
X, Y = -0.0585, -0.075
j0 = r.ik(X, Y, 1.05, *QX)
for i in range(4):
    if r.move(j0, 3.0) == 0: break
print("pre tcp", r.tcp()[0].round(4))
zs = np.arange(1.03, 0.947, -0.02).tolist() + [0.948]
qs = []; prev = r.joints()
for z in zs:
    j = r.ik(X, Y, z, *QX, seed=prev)
    if j is None: raise SystemExit("ik fail at z=%s" % z)
    print(f"z={z:.3f} jump={np.abs(np.array(j)-np.array(prev)).max():.3f} q={np.round(j,3)}")
    qs.append(j); prev = j
code = r.move_path(qs, 0.8)
print("tcp", r.tcp()[0].round(4), "joints", np.round(r.joints(),3))
EOF
timeout 900 python3 -u s16.py > s16.log 2>&1; cat s16.log; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih10.png

# openrua op 39
cat > s17.py <<'EOF'
from rob import *
r = R()
QX = (0.7071068, 0.7071068, 0.0, 0.0)
X, Y = -0.0585, -0.108
j0 = r.ik(X, Y, 1.05, *QX)
for i in range(4):
    if r.move(j0, 3.0) == 0: break
print("pre tcp", r.tcp()[0].round(4))
zs = np.arange(1.03, 0.947, -0.02).tolist() + [0.948]
qs = []; prev = r.joints()
for z in zs:
    j = r.ik(X, Y, z, *QX, seed=prev)
    if j is None: raise SystemExit("ik fail at z=%s" % z)
    qs.append(j); prev = j
code = r.move_path(qs, 0.8)
print("tcp", r.tcp()[0].round(4), "joints", np.round(r.joints(),3))
EOF
timeout 900 python3 -u s17.py > s17.log 2>&1; cat s17.log; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih11.png; timeout 60 python3 tools/perception/cam_snap.py frontview snaps/front5.png

# openrua op 40
cat > s18.py <<'EOF'
from rob import *
r = R()
QX = (0.7071068, 0.7071068, 0.0, 0.0)
r.grip(0.0)
j = r.joints(); r.move(j, 1.0)
print("fingers after settle", r.fingers())
# lift slowly straight up via waypoints
X, Y = -0.0585, -0.108
qs=[]; prev=r.joints()
for z in (0.97, 1.0, 1.05, 1.10, 1.15):
    jj = r.ik(X, Y, z, *QX, seed=prev); qs.append(jj); prev=jj
r.move_path(qs, 0.8)
print("tcp", r.tcp()[0].round(4), "fingers", r.fingers())
EOF
timeout 900 python3 -u s18.py > s18.log 2>&1; cat s18.log; timeout 60 python3 tools/perception/cam_snap.py frontview snaps/front6.png

# openrua op 41
cat > s19.py <<'EOF'
from rob import *
r = R()
QX = (0.7071068, 0.7071068, 0.0, 0.0)
X, Y = -0.0585, -0.108
r.grip(0.04)
qs=[]; prev=r.joints()
for z in (1.10, 1.05, 1.0, 0.97, 0.95, 0.93):
    jj = r.ik(X, Y, z, *QX, seed=prev); qs.append(jj); prev=jj
r.move_path(qs, 0.8)
print("tcp", r.tcp()[0].round(4), "fingers", r.fingers())
r.grip(0.0)
j = r.joints(); r.move(j, 1.0)
print("fingers after settle", r.fingers())
qs=[]; prev=r.joints()
for z in (0.95, 0.98, 1.02, 1.07, 1.12):
    jj = r.ik(X, Y, z, *QX, seed=prev); qs.append(jj); prev=jj
r.move_path(qs, 0.8)
print("tcp", r.tcp()[0].round(4), "fingers", r.fingers())
EOF
timeout 900 python3 -u s19.py > s19.log 2>&1; cat s19.log; timeout 60 python3 tools/perception/cam_snap.py frontview snaps/front7.png

# openrua op 42
cat > s20.py <<'EOF'
from rob import *
r = R()
QX = (0.7071068, 0.7071068, 0.0, 0.0)
X0, Y0 = -0.0585, -0.108
XT, YT = -0.0425, 0.348
# check target IK first
jt = r.ik(XT, YT, 0.965, *QX)
print("target ik", None if jt is None else np.round(jt,3))
if jt is None: raise SystemExit("unreachable")
qs=[]; prev=r.joints()
# rise
for z in (1.16, 1.20):
    jj = r.ik(X0, Y0, z, *QX, seed=prev); qs.append(jj); prev=jj
# translate in y (and x) at z=1.20
n = 10
for k in range(1, n+1):
    x = X0 + (XT-X0)*k/n; y = Y0 + (YT-Y0)*k/n
    jj = r.ik(x, y, 1.20, *QX, seed=prev)
    if jj is None: raise SystemExit(f"ik fail at {x},{y}")
    print(f"wp {k} jump={np.abs(np.array(jj)-np.array(prev)).max():.3f}")
    qs.append(jj); prev=jj
r.move_path(qs, 0.7)
print("tcp", r.tcp()[0].round(4), "fingers", r.fingers())
EOF
timeout 900 python3 -u s20.py > s20.log 2>&1; cat s20.log; timeout 60 python3 tools/perception/cam_snap.py agentview snaps/agent4.png; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih12.png

# openrua op 43
cat > s21.py <<'EOF'
from rob import *
r = R()
QX = (0.7071068, 0.7071068, 0.0, 0.0)
XT, YT = -0.0425, 0.348
qs=[]; prev=r.joints()
for z in (1.15, 1.10, 1.05, 1.01):
    jj = r.ik(XT, YT, z, *QX, seed=prev); qs.append(jj); prev=jj
r.move_path(qs, 0.8)
print("tcp", r.tcp()[0].round(4), "fingers", r.fingers())
EOF
timeout 900 python3 -u s21.py > s21.log 2>&1; cat s21.log; timeout 60 python3 tools/perception/cam_snap.py birdview snaps/bird2.png

# openrua op 44
python3 -c "
import cv2; img=cv2.imread('snaps/bird2.png'); c=img[230:340,320:440]; cv2.imwrite('snaps/bird2_crop.png', cv2.resize(c,None,fx=5,fy=5,interpolation=cv2.INTER_NEAREST))"

# openrua op 45
cat > s22.py <<'EOF'
from rob import *
r = R()
QX = (0.7071068, 0.7071068, 0.0, 0.0)
XT, YT = -0.0425, 0.348
qs=[]; prev=r.joints()
for z in (0.99, 0.975, 0.965):
    jj = r.ik(XT, YT, z, *QX, seed=prev); qs.append(jj); prev=jj
r.move_path(qs, 0.8)
print("tcp", r.tcp()[0].round(4), "fingers", r.fingers())
r.grip(0.04)
j = r.joints(); r.move(j, 1.0)
print("fingers", r.fingers())
qs=[]; prev=r.joints()
for z in (1.0, 1.05, 1.12):
    jj = r.ik(XT, YT, z, *QX, seed=prev); qs.append(jj); prev=jj
r.move_path(qs, 0.8)
print("tcp", r.tcp()[0].round(4))
EOF
timeout 900 python3 -u s22.py > s22.log 2>&1; cat s22.log; timeout 60 python3 tools/perception/cam_snap.py agentview snaps/agent5.png; timeout 60 python3 tools/perception/cam_snap.py frontview snaps/front8.png

# openrua op 46
cat > s23.py <<'EOF'
from rob import *
r = R()
# retreat toward home-ish pose above table, away from objects
home = [0.0, -0.161, 0.0, -2.445, 0.0, 2.227, 0.785]
for i in range(4):
    if r.move(home, 4.0) == 0: break
print("joints", np.round(r.joints(),3), "tcp", r.tcp()[0].round(3))
EOF
timeout 900 python3 -u s23.py > s23.log 2>&1; cat s23.log
for c in frontview sideview birdview; do timeout 60 python3 tools/perception/cam_snap.py /$c/depth/image_raw snaps/${c}_depth3.png; timeout 60 python3 tools/perception/cam_snap.py $c snaps/${c}_3.png; done
cat > verify.py <<'EOF'
import numpy as np
from rob import quat_R
fx=579.4112549695428; cx=320; cy=240
cams = {"front": ("snaps/frontview_depth3.npy", np.array([1.0,0.0,1.48]), quat_R(0.5608,0.5608,-0.4306,-0.4306)),
        "side": ("snaps/sideview_depth3.npy", np.array([-0.0565,1.2761,1.488]), quat_R(0.0099,0.8064,-0.5912,-0.0069)),
        "bird": ("snaps/birdview_depth3.npy", np.array([-0.2,0,3.0]), quat_R(0.7071,0.7071,0,0))}
for name,(f,t,R) in cams.items():
    d = np.load(f); P = np.zeros((480,640,3))
    for v in range(480):
        Z = d[v,:]; u = np.arange(640)
        P[v] = np.stack([(u-cx)*Z/fx, (v-cy)*Z/fx, Z], 1)@R.T + t
    pts = P.reshape(-1,3)
    # stove plate region
    m = (pts[:,0]>-0.20)&(pts[:,0]<0.12)&(pts[:,1]>0.05)&(pts[:,1]<0.36)&(pts[:,2]>0.905)&(pts[:,2]<1.1)
    p = pts[m]
    rim = p[p[:,2]>0.96]
    rim = rim[rim[:,1]<0.30]
    if rim.size:
        print(f"{name}: pan rim z max {rim[:,2].max():.3f}; rim x[{rim[:,0].min():.3f},{rim[:,0].max():.3f}] y[{rim[:,1].min():.3f},{rim[:,1].max():.3f}] center ({(rim[:,0].min()+rim[:,0].max())/2:.3f},{(rim[:,1].min()+rim[:,1].max())/2:.3f})")
    # anything on table between pan and knob? knob top
    k = pts[(np.hypot(pts[:,0]+0.197, pts[:,1]-0.201)<0.045)&(pts[:,2]>0.905)&(pts[:,2]<1.0)]
    print(f"   knob z max {k[:,2].max():.3f}")
EOF
python3 verify.py

# openrua op 47
cat > verify2.py <<'EOF'
import numpy as np
from rob import quat_R
fx=579.4112549695428; cx=320; cy=240
cams = {"front": ("snaps/frontview_depth3.npy", np.array([1.0,0.0,1.48]), quat_R(0.5608,0.5608,-0.4306,-0.4306)),
        "side": ("snaps/sideview_depth3.npy", np.array([-0.0565,1.2761,1.488]), quat_R(0.0099,0.8064,-0.5912,-0.0069)),
        "bird": ("snaps/birdview_depth3.npy", np.array([-0.2,0,3.0]), quat_R(0.7071,0.7071,0,0))}
for name,(f,t,R) in cams.items():
    d = np.load(f); P = np.zeros((480,640,3))
    for v in range(480):
        Z = d[v,:]; u = np.arange(640)
        P[v] = np.stack([(u-cx)*Z/fx, (v-cy)*Z/fx, Z], 1)@R.T + t
    pts = P.reshape(-1,3)
    box = (pts[:,0]>-0.18)&(pts[:,0]<0.10)&(pts[:,1]>0.09)&(pts[:,1]<0.45)&(pts[:,2]>0.905)&(pts[:,2]<1.02)
    p = pts[box]
    hist = np.histogram(p[:,2], bins=np.arange(0.90,1.02,0.01))
    print(name, "z hist", list(zip(hist[1][:-1].round(2), hist[0])))
    body = p[(p[:,2]>0.95)&(p[:,1]<0.31)]  # pan body above the stove plate level
    print(f"   pan body(z>0.95,y<0.31): x[{body[:,0].min():.3f},{body[:,0].max():.3f}] y[{body[:,1].min():.3f},{body[:,1].max():.3f}] zmax {body[:,2].max():.3f} center ({(body[:,0].min()+body[:,0].max())/2:.3f},{(body[:,1].min()+body[:,1].max())/2:.3f})")
    handle = p[(p[:,1]>0.31)&(p[:,2]>0.94)]
    if handle.size: print(f"   handle: y[{handle[:,1].min():.3f},{handle[:,1].max():.3f}] z[{handle[:,2].min():.3f},{handle[:,2].max():.3f}]")
EOF
python3 verify2.py
