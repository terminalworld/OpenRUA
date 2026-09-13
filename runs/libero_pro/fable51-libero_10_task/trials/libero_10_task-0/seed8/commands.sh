#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 2 -type f | head -50

# openrua op 1
ls -laR /workspace/tools /workspace/.ros; echo ---; ros2 topic list; echo ---; ros2 action list; echo ---; ros2 service list | head -40; echo ---; ros2 node list

# openrua op 2
ros2 topic echo /joint_states --once 2>&1 | head -40; for c in agentview birdview frontview sideview robot0_robotview robot0_eye_in_hand; do python3 tools/perception/cam_snap.py $c; done

# openrua op 3
python3 -c "
import cv2
im=cv2.imread('agentview.png')
cv2.imwrite('crop_left.png', cv2.resize(im[240:430,90:280], None, fx=3, fy=3, interpolation=cv2.INTER_CUBIC))
cv2.imwrite('crop_right.png', cv2.resize(im[160:400,300:400], None, fx=3, fy=3, interpolation=cv2.INTER_CUBIC))
"

# openrua op 4
python3 -c "
import cv2
im=cv2.imread('birdview.png')
c=im[200:340,230:420]
c=cv2.resize(c,None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC)
# draw grid every 10 px original
for x in range(0,190,10):
    cv2.line(c,(x*4,0),(x*4,c.shape[0]),(0,255,0) if (x+230)%50==0 else (80,80,80),1)
for y in range(0,140,10):
    cv2.line(c,(0,y*4),(c.shape[1],y*4),(0,255,0) if (y+200)%50==0 else (80,80,80),1)
cv2.imwrite('bird_crop.png',c)
"; ros2 run tf2_ros tf2_echo world birdview_optical_frame 2>&1 | head -8 &
sleep 3; kill %1 2>/dev/null; ros2 topic echo /birdview/color/camera_info --once 2>&1 | grep -A12 '^k:'

# openrua op 5
for p in "272 310" "319 285" "381 286" "292 300" "265 291" "326 297" "284 259" "284 229" "340 330"; do echo -n "$p -> "; timeout 60 python3 tools/perception/px2world.py birdview $p 2>&1 | tail -1; done

# openrua op 6
for p in "352 232" "352 255" "345 240" "360 240" "140 385" "520 250" "300 400"; do echo -n "agentview $p -> "; timeout 60 python3 tools/perception/px2world.py agentview $p 2>&1 | tail -1; done; echo; timeout 20 python3 -c "
import rclpy
from tf2_ros import Buffer, TransformListener
rclpy.init(); n=rclpy.create_node('tfq'); b=Buffer(); TransformListener(b,n)
import time
for _ in range(30):
    rclpy.spin_once(n,timeout_sec=0.2)
for f in ['panda_link0','panda_hand','birdview_optical_frame','agentview_optical_frame']:
    try:
        t=b.lookup_transform('world',f,rclpy.time.Time()); tr=t.transform.translation; r=t.transform.rotation
        print(f, round(tr.x,4),round(tr.y,4),round(tr.z,4), '|', round(r.x,4),round(r.y,4),round(r.z,4),round(r.w,4))
    except Exception as e: print(f,'ERR',e)
print(b.all_frames_as_string())
"

# openrua op 7
mkdir -p "$(dirname /workspace/rb.py)"
cat > /workspace/rb.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Session helper: one node, reusable clients, world-frame poses.

World->base (panda_link0) is a pure translation on this machine (read
from TF at start).  All public functions take WORLD coordinates.
"""
import struct
import sys
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import CameraInfo, Image, JointState
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint
from scipy.spatial.transform import Rotation as Rot

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]
TCP = float(M["hand"]["tcp_offset_m"])

# hand pointing straight down, fingers along world Y
Q_DOWN = Rot.from_quat([1.0, 0.0, 0.0, 0.0])


def q_down_yaw(yaw_deg):
    """Downward hand, rotated yaw_deg about world Z (0 -> fingers along Y)."""
    return (Rot.from_euler("z", yaw_deg, degrees=True) * Q_DOWN).as_quat()


class RB:
    def __init__(self, name="rb"):
        rclpy.init()
        self.n = rclpy.create_node(name)
        self.js = {}
        self.n.create_subscription(JointState, "/joint_states",
                                   lambda m: self.js.__setitem__("m", m), 1)
        self.tf = Buffer()
        TransformListener(self.tf, self.n)
        self.ik = self.n.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fjt = ActionClient(self.n, FollowJointTrajectory, FJT["port"])
        self.gr = ActionClient(self.n, GripperCommand, GRIP["port"])
        self.ik.wait_for_service(10)
        self.fjt.wait_for_server(10)
        self.gr.wait_for_server(10)
        self.spin(0.5)
        t = self.lookup("world", "panda_link0")
        self.base = np.array(t[0])
        self.log(f"base in world: {self.base}")

    def log(self, *a):
        print(time.strftime("%H:%M:%S"), *a, flush=True)

    def spin(self, sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(self.n, timeout_sec=0.05)

    # ---------------- sensing ----------------
    def joints(self):
        self.js.pop("m", None)
        while "m" not in self.js:
            rclpy.spin_once(self.n, timeout_sec=0.1)
        m = self.js["m"]
        return dict(zip(m.name, m.position))

    def fingers(self):
        j = self.joints()
        return j["panda_finger_joint1"], j["panda_finger_joint2"]

    def lookup(self, a, b):
        end = time.time() + 10
        while time.time() < end:
            rclpy.spin_once(self.n, timeout_sec=0.1)
            if self.tf.can_transform(a, b, rclpy.time.Time()):
                break
        t = self.tf.lookup_transform(a, b, rclpy.time.Time())
        tr, r = t.transform.translation, t.transform.rotation
        return (tr.x, tr.y, tr.z), (r.x, r.y, r.z, r.w)

    def hand_pose(self):
        """Hand frame + TCP point in world (from TF, after a fresh spin)."""
        self.spin(0.3)
        p, q = self.lookup("world", "panda_hand")
        R = Rot.from_quat(q)
        tcp = np.array(p) + TCP * R.as_matrix()[:, 2]
        return np.array(p), np.array(q), tcp

    def grab(self, topic, typ, timeout=30):
        got = {}
        sub = self.n.create_subscription(typ, topic,
                                         lambda m: got.setdefault("m", m), 1)
        end = time.time() + timeout
        while "m" not in got and time.time() < end:
            rclpy.spin_once(self.n, timeout_sec=0.1)
        self.n.destroy_subscription(sub)
        if "m" not in got:
            raise RuntimeError(f"no msg on {topic}")
        return got["m"]

    def snap(self, cam, out=None):
        import cv2
        from cv_bridge import CvBridge
        m = self.grab(f"/{cam}/color/image_raw", Image)
        img = CvBridge().imgmsg_to_cv2(m, "bgr8")
        out = out or f"/workspace/{cam}.png"
        cv2.imwrite(out, img)
        return img

    def cloud(self, cam):
        """World-frame point cloud (H,W,3) of the camera's current depth."""
        d = self.grab(f"/{cam}/depth/image_raw", Image)
        info = self.grab(f"/{cam}/color/camera_info", CameraInfo)
        z = np.frombuffer(d.data, dtype="<f4").reshape(d.height, d.width)
        fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
        v, u = np.mgrid[0:d.height, 0:d.width]
        pc = np.stack([(u - cx) * z / fx, (v - cy) * z / fy, z], -1)
        p, q = self.lookup("world", f"{cam}_optical_frame")
        R = Rot.from_quat(q).as_matrix()
        return pc @ R.T + np.array(p)

    # ---------------- acting ----------------
    def ik_world(self, xyz, quat, at_tcp=True, seed=None):
        xyz = np.array(xyz, float)
        R = Rot.from_quat(quat).as_matrix()
        if at_tcp:
            xyz = xyz - TCP * R[:, 2]
        b = xyz - self.base
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, b)
        (p.orientation.x, p.orientation.y,
         p.orientation.z, p.orientation.w) = map(float, quat)
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=2)
        s = JointState()
        cur = seed or self.joints()
        for j in ARM:
            s.name.append(j)
            s.position.append(float(cur[j]))
        req.ik_request.robot_state.joint_state = s
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.n, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"IK failed: {None if res is None else res.error_code.val}")
        sol = dict(zip(res.solution.joint_state.name,
                       res.solution.joint_state.position))
        return [sol[j] for j in ARM]

    def traj(self, points, seconds):
        """points: list of joint vectors; seconds: total time (spread evenly)."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM
        n = len(points)
        for i, q in enumerate(points):
            t = seconds * (i + 1) / n
            pt = JointTrajectoryPoint(positions=[float(x) for x in q])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        fut = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.n, fut)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.n, rf)
        code = rf.result().result.error_code
        cur = self.joints()
        err = max(abs(cur[j] - points[-1][i]) for i, j in enumerate(ARM))
        self.log(f"traj done code={code} max_joint_err={err:.4f}")
        return code, err

    def goto(self, xyz, quat, seconds=3.0, at_tcp=True, via=None):
        """IK to world pose (TCP by default) then one trajectory."""
        q = self.ik_world(xyz, quat, at_tcp)
        pts = ([via] if via is not None else []) + [q]
        code, err = self.traj(pts, seconds)
        p, hq, tcp = self.hand_pose()
        self.log(f"tcp now {np.round(tcp, 4)} (target {np.round(xyz, 4)})")
        return code, err, tcp

    def gripper(self, width):
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = float(GRIP["max_effort"])
        fut = self.gr.send_goal_async(g)
        rclpy.spin_until_future_complete(self.n, fut, timeout_sec=30)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.n, rf, timeout_sec=120)
        r = rf.result().result
        self.spin(0.3)
        f = self.fingers()
        self.log(f"gripper({width}) reached={r.reached_goal} stalled={r.stalled} fingers={f}")
        return f

    def close(self):
        rclpy.shutdown()
OPENRUA_EOF

# openrua op 8
mkdir -p "$(dirname /workspace/p1_survey.py)"
cat > /workspace/p1_survey.py <<'OPENRUA_EOF'
import numpy as np
from rb import RB, q_down_yaw

r = RB("p1")
r.log("joints", {k: round(v, 3) for k, v in r.joints().items()})
r.gripper(0.04)

# park hand high over the far side (behind the objects, toward -x) so the
# overhead camera sees the object region
q = q_down_yaw(0)
r.goto([-0.35, -0.25, 0.85], q, seconds=3.0)

r.snap("birdview", "/workspace/bird2.png")
pc = r.cloud("birdview")
np.save("/workspace/bird_pc.npy", pc)
r.snap("robot0_eye_in_hand", "/workspace/eih.png")
r.close()
OPENRUA_EOF

# openrua op 9
timeout 900 python3 -u p1_survey.py 2>&1 | tail -20

# openrua op 10
timeout 60 python3 tools/perception/cam_snap.py agentview av2.png >/dev/null && echo ok

# openrua op 11
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rb.py", "old_string": "        q = self.ik_world(xyz, quat, at_tcp)\\n        pts = ([via] if via is not None else []) + [q]\\n        code, err = self.traj(pts, seconds)", "new_string": "        q = self.ik_world(xyz, quat, at_tcp)\\n        pts = ([via] if via is not None else []) + [q]\\n        code, err = self.traj(pts, seconds)\\n        for _ in range(3):\\n            if code == 0 and err < 0.02:\\n                break\\n            # machine fact: -5 on long goals is usually controller lag;\\n            # resending the same goal converges\\n            self.log(\\"retrying goal\\")\\n            code, err = self.traj([q], max(seconds, 3.0))", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 12
mkdir -p "$(dirname /workspace/p1_survey.py)"
cat > /workspace/p1_survey.py <<'OPENRUA_EOF'
import numpy as np
from rb import RB, q_down_yaw

r = RB("p1")
r.log("joints", {k: round(v, 3) for k, v in r.joints().items()})

# park TCP off to the -y side so the overhead camera sees the objects
q = q_down_yaw(0)
r.goto([-0.25, -0.45, 0.75], q, seconds=5.0)

r.snap("birdview", "/workspace/bird2.png")
pc = r.cloud("birdview")
np.save("/workspace/bird_pc.npy", pc)
r.snap("agentview", "/workspace/av3.png")
r.close()
OPENRUA_EOF

# openrua op 13
timeout 900 python3 -u p1_survey.py 2>&1 | tail -20

# openrua op 14
mkdir -p "$(dirname /workspace/p1_survey.py)"
cat > /workspace/p1_survey.py <<'OPENRUA_EOF'
import numpy as np
from rb import RB, q_down_yaw

r = RB("p1")
r.log("joints", {k: round(v, 3) for k, v in r.joints().items()})

# park TCP off to the -y side so the overhead camera sees the objects
target = [-0.25, -0.45, 0.75]
for yaw in (0, 45, -45, 90, -90, 135):
    try:
        q = q_down_yaw(yaw)
        r.ik_world(target, q)
        r.log("yaw ok", yaw)
        break
    except RuntimeError as e:
        r.log("yaw", yaw, e)
r.goto(target, q, seconds=5.0)

r.snap("birdview", "/workspace/bird2.png")
pc = r.cloud("birdview")
np.save("/workspace/bird_pc.npy", pc)
r.snap("agentview", "/workspace/av3.png")
r.close()
OPENRUA_EOF

# openrua op 15
timeout 900 python3 -u p1_survey.py 2>&1 | tail -20

# openrua op 16
cat > /workspace/ik_probe.py <<'EOF'
import numpy as np
from rb import RB, q_down_yaw
r = RB("probe")
p, q, tcp = r.hand_pose()
r.log("hand", np.round(p,3), np.round(q,3), "tcp", np.round(tcp,3))
tests = [("current", tcp, q),
         ("cc pregrasp", [0.108,-0.211,0.55], q_down_yaw(0)),
         ("tomato pregrasp", [-0.12,0.047,0.60], q_down_yaw(0)),
         ("basket", [0.0,0.264,0.72], q_down_yaw(0)),
         ("park1", [-0.25,-0.45,0.75], q_down_yaw(0)),
         ("park2", [-0.1,-0.4,0.70], q_down_yaw(0)),
         ("park3", [0.0,-0.35,0.65], q_down_yaw(0)),
         ("home-ish", [-0.05,0.0,0.68], q_down_yaw(0)),
]
for name, xyz, qq in tests:
    try:
        sol = r.ik_world(xyz, qq)
        r.log(name, "OK", np.round(sol,3))
    except RuntimeError as e:
        r.log(name, e)
r.close()
EOF
timeout 600 python3 -u ik_probe.py 2>&1 | tail -20

# openrua op 17
cat > /workspace/ik_probe2.py <<'EOF'
import numpy as np, rclpy
from rb import RB, M, ARM, TCP
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
from scipy.spatial.transform import Rotation as Rot
r = RB("probe2")
p, q, tcp = r.hand_pose()
b = np.array(p) - r.base
def call(variant, seed_joints, with_extras, frame=""):
    req = GetPositionIK.Request()
    req.ik_request.group_name = M["planning"]["group"]
    req.ik_request.pose_stamped.header.frame_id = frame
    pp = req.ik_request.pose_stamped.pose
    pp.position.x, pp.position.y, pp.position.z = map(float, b)
    pp.orientation.x, pp.orientation.y, pp.orientation.z, pp.orientation.w = map(float, q)
    if with_extras:
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=2)
    s = JointState()
    for j, v in seed_joints.items():
        s.name.append(j); s.position.append(float(v))
    req.ik_request.robot_state.joint_state = s
    fut = r.ik.call_async(req)
    rclpy.spin_until_future_complete(r.n, fut, timeout_sec=60)
    res = fut.result()
    r.log(variant, None if res is None else res.error_code.val,
          None if res is None or res.error_code.val != 1 else np.round(res.solution.joint_state.position, 3))
cur = r.joints()
arm_cur = {j: cur[j] for j in ARM}
home = dict(zip(ARM, [0.0, -0.161, 0.0, -2.445, 0.0, 2.227, 0.785]))
zeros = dict(zip(ARM, [0.0, 0.0, 0.0, -1.5, 0.0, 1.5, 0.0]))
call("cur seed, extras", arm_cur, True)
call("cur seed, no extras", arm_cur, False)
call("home seed, no extras", home, False)
call("zeros seed, no extras", zeros, False)
call("empty seed, no extras", {}, False)
call("cur seed, frame panda_link0", arm_cur, False, "panda_link0")
call("cur seed, frame world", arm_cur, False, "world")
r.close()
EOF
timeout 600 python3 -u ik_probe2.py 2>&1 | tail -20

# openrua op 18
cat > /workspace/ik_probe3.py <<'EOF'
import numpy as np, rclpy
from rb import RB, M, ARM, TCP, q_down_yaw
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
r = RB("probe3")
p, q, tcp = r.hand_pose()
def call(label, xyz, quat, frame):
    req = GetPositionIK.Request()
    req.ik_request.group_name = M["planning"]["group"]
    req.ik_request.pose_stamped.header.frame_id = frame
    pp = req.ik_request.pose_stamped.pose
    pp.position.x, pp.position.y, pp.position.z = map(float, xyz)
    pp.orientation.x, pp.orientation.y, pp.orientation.z, pp.orientation.w = map(float, quat)
    s = JointState(); cur = r.joints()
    for j in ARM: s.name.append(j); s.position.append(float(cur[j]))
    req.ik_request.robot_state.joint_state = s
    fut = r.ik.call_async(req)
    rclpy.spin_until_future_complete(r.n, fut, timeout_sec=60)
    res = fut.result()
    r.log(label, frame or "''", None if res is None else res.error_code.val,
          None if res is None or res.error_code.val != 1 else np.round(res.solution.joint_state.position[:7], 3))
call("hand world coords", p, q, "")
call("hand world coords", p, q, "world")
call("hand base coords", np.array(p)-r.base, q, "panda_link0")
for name, xyz in [("cc pregrasp", [0.108,-0.211,0.55]), ("tomato pregrasp", [-0.12,0.047,0.60]),
                  ("basket", [0.0,0.264,0.72]), ("park2", [-0.1,-0.4,0.70]), ("home-ish", [-0.05,0.0,0.68])]:
    hand = np.array(xyz) + [0,0,TCP]   # hand above tcp for downward hand
    call(name, hand - r.base, q_down_yaw(0), "panda_link0")
r.close()
EOF
timeout 600 python3 -u ik_probe3.py 2>&1 | tail -20

# openrua op 19
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rb.py", "old_string": "        b = xyz - self.base\\n        req = GetPositionIK.Request()\\n        req.ik_request.group_name = M[\\"planning\\"][\\"group\\"]\\n        req.ik_request.pose_stamped.header.frame_id = \\"\\"\\n        p = req.ik_request.pose_stamped.pose\\n        p.position.x, p.position.y, p.position.z = map(float, b)", "new_string": "        # machine fact (verified): the planner\'s model frame here is\\n        # `world`; an empty frame_id means world, \\"panda_link0\\" also works\\n        req = GetPositionIK.Request()\\n        req.ik_request.group_name = M[\\"planning\\"][\\"group\\"]\\n        req.ik_request.pose_stamped.header.frame_id = \\"world\\"\\n        p = req.ik_request.pose_stamped.pose\\n        p.position.x, p.position.y, p.position.z = map(float, xyz)", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 20
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rb.py", "old_string": "        req.ik_request.avoid_collisions = False\\n        req.ik_request.timeout = Duration(sec=2)\\n        s = JointState()", "new_string": "        s = JointState()", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 21
timeout 900 python3 -u p1_survey.py 2>&1 | tail -20

# openrua op 22
cat > /workspace/seg.py <<'EOF'
import numpy as np, cv2
pc = np.load("/workspace/bird_pc.npy")
img = cv2.imread("/workspace/bird2.png")
z = pc[...,2]
# table plane height: mode of z in the table region
tbl = z[(np.abs(pc[...,0])<0.35)&(np.abs(pc[...,1])<0.5)&np.isfinite(z)]
h,_ = np.histogram(tbl, bins=np.arange(0.3,0.7,0.002)); table_z = 0.3+0.002*(np.argmax(h)+0.5)
print("table z ~", round(table_z,4))
mask = (z > table_z+0.012) & (z < table_z+0.35) & np.isfinite(z) & (np.abs(pc[...,1])<0.5) & (pc[...,0]>-0.4) & (pc[...,0]<0.35)
n, lab = cv2.connectedComponents(mask.astype(np.uint8))
for i in range(1,n):
    m = lab==i
    if m.sum()<15: continue
    P = pc[m]
    ys,xs = np.nonzero(m)
    print(f"blob {i}: px({xs.mean():.0f},{ys.mean():.0f}) n={m.sum()} "
          f"center=({P[:,0].mean():.3f},{P[:,1].mean():.3f}) x[{P[:,0].min():.3f},{P[:,0].max():.3f}] "
          f"y[{P[:,1].min():.3f},{P[:,1].max():.3f}] ztop={P[:,2].max():.3f} zmed={np.median(P[:,2]):.3f}")
cv2.imwrite("/workspace/bird_mask.png", (mask*255).astype(np.uint8))
EOF
python3 seg.py

# openrua op 23
cat >> /workspace/seg.py <<'EOF'
print("--- PCA")
for i in (4,9):
    m = lab==i; P = pc[m][:,:2]; c = P.mean(0)
    w,v = np.linalg.eigh(np.cov((P-c).T)); major = v[:,1]
    ang = np.degrees(np.arctan2(major[1], major[0]))
    print(f"blob {i}: major axis angle from +x = {ang:.1f} deg; extents along major/minor: "
          f"{np.ptp((P-c)@v[:,1]):.3f} {np.ptp((P-c)@v[:,0]):.3f}")
EOF
python3 seg.py | tail -3

# openrua op 24
timeout 60 python3 - <<'EOF' 2>&1 | head -40
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
rclpy.init(); n=rclpy.create_node('urdfq')
got=[]
qos=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
n.create_subscription(String,'/robot_description',lambda m: got.append(m.data),qos)
import time; t=time.time()
while not got and time.time()-t<15: rclpy.spin_once(n,timeout_sec=0.2)
open('robot.urdf','w').write(got[0]) if got else print("none")
import re
u=got[0]
for name in ['panda_hand','panda_leftfinger','panda_link7']:
    i=u.find(f'<link name="{name}"'); j=u.find('</link>',i)
    print(name); print(u[i:j][:1500]); print('-----')
EOF

# openrua op 25
timeout 60 python3 - <<'EOF'
import numpy as np, struct
def stl_bounds(p):
    d=open(p,'rb').read()
    if d[:5]==b'solid' and b'facet' in d[:300]:
        v=[list(map(float,l.split()[1:4])) for l in d.decode().splitlines() if l.strip().startswith('vertex')]
        v=np.array(v)
    else:
        n=struct.unpack_from('<I',d,80)[0]
        arr=np.frombuffer(d[84:84+n*50],dtype=np.dtype([('n','<3f4'),('v','<9f4'),('a','<u2')]))
        v=arr['v'].reshape(-1,3)
    return v.min(0), v.max(0)
for f in ['hand','finger']:
    lo,hi=stl_bounds(f'/opt/ros/jazzy/share/moveit_resources_panda_description/meshes/collision/{f}.stl')
    print(f, 'min',np.round(lo,4),'max',np.round(hi,4))
EOF
grep -o '<joint name="panda_finger_joint1"[^/]*/>.\{0,400\}' robot.urdf | head -c 700; echo; grep -o '<joint name="panda_hand_joint".\{0,300\}' robot.urdf | head -c 400

# openrua op 26
mkdir -p "$(dirname /workspace/p2_pick.py)"
cat > /workspace/p2_pick.py <<'OPENRUA_EOF'
"""Pick one object and drop it in the basket.  Usage: p2_pick.py cc|tomato"""
import sys
import numpy as np
from rb import RB, q_down_yaw

OBJ = {
    # name: (center xy, grasp tcp z, yaw deg, pregrasp z)
    "cc":     ((0.108, -0.209), 0.440, 15.0, 0.60),
    "tomato": ((-0.127, 0.046), 0.478, 0.0, 0.65),
}
BASKET = (-0.01, 0.25)
DROP_Z = 0.75

(cx, cy), gz, yaw, pz = OBJ[sys.argv[1]]
r = RB("p2")
q = q_down_yaw(yaw)

f = r.fingers()
if f[0] < 0.035:
    r.gripper(0.04)

r.log("== pregrasp")
r.goto([cx, cy, pz], q, seconds=5.0)
r.log("== descend")
code, err, tcp = r.goto([cx, cy, gz], q, seconds=3.0)
if np.linalg.norm(tcp[:2] - [cx, cy]) > 0.008 or abs(tcp[2] - gz) > 0.008:
    r.log("!! descend inaccurate, retrying")
    r.goto([cx, cy, gz], q, seconds=3.0)
r.log("== close")
f = r.gripper(0.0)
gap = f[0] - f[1]
r.log(f"finger gap = {gap:.4f} m")
if gap < 0.01:
    r.log("!! closed on air; aborting")
    r.gripper(0.04)
    r.goto([cx, cy, pz], q, seconds=3.0)
    r.close()
    sys.exit(2)
r.log("== lift")
r.goto([cx, cy, DROP_Z], q, seconds=3.0)
r.snap("agentview", f"/workspace/{sys.argv[1]}_lifted.png")
f = r.fingers()
r.log(f"gap after lift = {f[0]-f[1]:.4f}")
r.log("== to basket")
r.goto([BASKET[0], BASKET[1], DROP_Z], q, seconds=5.0)
f = r.fingers()
r.log(f"gap over basket = {f[0]-f[1]:.4f}")
r.log("== release")
r.gripper(0.04)
r.spin(0.5)
r.log("== retreat up")
r.goto([BASKET[0], BASKET[1], 0.85], q, seconds=3.0)
r.snap("agentview", f"/workspace/{sys.argv[1]}_dropped.png")
r.close()
OPENRUA_EOF

# openrua op 27
timeout 1500 python3 -u p2_pick.py cc 2>&1 | tee p2_cc.log | tail -40

# openrua op 28
timeout 120 python3 tools/perception/cam_snap.py agentview av4.png >/dev/null; timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih.png >/dev/null; python3 -c "
import cv2; im=cv2.imread('av4.png'); cv2.imwrite('av4_crop.png', cv2.resize(im[250:450,60:300],None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))"

# openrua op 29
timeout 60 python3 - <<'EOF'
import rclpy, numpy as np
from tf2_ros import Buffer, TransformListener
from scipy.spatial.transform import Rotation as Rot
rclpy.init(); n=rclpy.create_node('tfq'); b=Buffer(); TransformListener(b,n)
for _ in range(20): rclpy.spin_once(n,timeout_sec=0.1)
for f in ['panda_link8','panda_hand','panda_leftfinger','panda_rightfinger']:
    t=b.lookup_transform('world',f,rclpy.time.Time()); tr=t.transform.translation; r=t.transform.rotation
    R=Rot.from_quat([r.x,r.y,r.z,r.w]).as_matrix()
    print(f, np.round([tr.x,tr.y,tr.z],4), 'quat', np.round([r.x,r.y,r.z,r.w],3), '\n   hand axes in world: x',np.round(R[:,0],2),'y',np.round(R[:,1],2),'z',np.round(R[:,2],2))
EOF

# openrua op 30
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rb.py", "old_string": "def q_down_yaw(yaw_deg):\\n    \\"\\"\\"Downward hand, rotated yaw_deg about world Z (0 -> fingers along Y).\\"\\"\\"\\n    return (Rot.from_euler(\\"z\\", yaw_deg, degrees=True) * Q_DOWN).as_quat()", "new_string": "def q_down_yaw(yaw_deg):\\n    \\"\\"\\"Downward hand, finger axis at (90 + yaw_deg) deg from world +X\\n    (0 -> fingers along Y).\\n\\n    Machine fact (verified via TF): the IK group\'s tip is panda_link8 and\\n    panda_hand is yawed -45 deg from it, so the finger axis (hand Y) ends\\n    up at link8_yaw - 45 deg.  Compensate here.\\n    \\"\\"\\"\\n    return (Rot.from_euler(\\"z\\", yaw_deg - 45.0, degrees=True) * Q_DOWN).as_quat()", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 31
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/p2_pick.py", "old_string": "    \\"cc\\":     ((0.108, -0.209), 0.440, 15.0, 0.60),", "new_string": "    # cc: box spans x[0.068,0.148]; grasp toward +x end and yaw -10 so the\\n    # palm\'s +y end stays clear of the milk carton at (0.06,-0.11)\\n    \\"cc\\":     ((0.125, -0.209), 0.440, -10.0, 0.60),", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 32
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/p2_pick.py", "old_string": "r.goto([cx, cy, pz], q, seconds=5.0)\\nr.log(\\"== descend\\")", "new_string": "r.goto([cx, cy, pz], q, seconds=5.0)\\nfrom scipy.spatial.transform import Rotation as Rot\\n_, hq, _ = r.hand_pose()\\nfy = Rot.from_quat(hq).as_matrix()[:, 1]\\nfang = np.degrees(np.arctan2(fy[1], fy[0])) % 180\\nr.log(f\\"finger axis angle from +x: {fang:.1f} deg (want {(90 + yaw) % 180:.1f})\\")\\nif abs(((fang - (90 + yaw)) + 90) % 180 - 90) > 5:\\n    r.log(\\"!! finger axis wrong; aborting\\")\\n    r.close(); sys.exit(3)\\nr.log(\\"== descend\\")", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 33
timeout 1500 python3 -u p2_pick.py cc 2>&1 | tee p2_cc.log | tail -40

# openrua op 34
timeout 1500 python3 -u p2_pick.py tomato 2>&1 | tee p2_tomato.log | tail -40

# openrua op 35
timeout 120 python3 tools/perception/cam_snap.py agentview av5.png >/dev/null; timeout 120 python3 tools/perception/cam_snap.py sideview sv5.png >/dev/null; echo ok

# openrua op 36
mkdir -p "$(dirname /workspace/p3_resurvey.py)"
cat > /workspace/p3_resurvey.py <<'OPENRUA_EOF'
import numpy as np
from rb import RB, q_down_yaw

r = RB("p3")
r.goto([-0.25, -0.45, 0.75], q_down_yaw(45), seconds=5.0)
r.snap("birdview", "/workspace/bird3.png")
np.save("/workspace/bird_pc3.npy", r.cloud("birdview"))
r.snap("agentview", "/workspace/av6.png")
r.close()
OPENRUA_EOF

# openrua op 37
timeout 900 python3 -u p3_resurvey.py 2>&1 | tail -5 && sed -i 's#bird_pc.npy#bird_pc3.npy#; s#bird2.png#bird3.png#' seg.py && python3 seg.py

# openrua op 38
cat > /workspace/seg2.py <<'EOF'
import numpy as np, cv2
pc0 = np.load("/workspace/bird_pc.npy"); pc = np.load("/workspace/bird_pc3.npy")
z = pc[...,2]
# lying can blob
m = (z>0.437)&(z<0.56)&(pc[...,0]>-0.14)&(pc[...,0]<-0.02)&(pc[...,1]>0.03)&(pc[...,1]<0.15)
P = pc[m]; c = P[:,:2].mean(0)
w,v = np.linalg.eigh(np.cov((P[:,:2]-c).T)); major=v[:,1]
ang = np.degrees(np.arctan2(major[1],major[0]))
print(f"can: n={m.sum()} center={np.round(c,4)} major angle={ang:.1f} extents major={np.ptp((P[:,:2]-c)@v[:,1]):.3f} minor={np.ptp((P[:,:2]-c)@v[:,0]):.3f} ztop={P[:,2].max():.3f}")
# height profile along minor axis (should be a semicircle for a cylinder)
proj = (P[:,:2]-c)@v[:,0]
for lo in np.arange(-0.04,0.04,0.01):
    s=(proj>=lo)&(proj<lo+0.01)
    if s.any(): print(f"  minor {lo:+.2f}..{lo+0.01:+.2f}: zmax={P[s,2].max():.3f}")
# ridge top position (highest points) -> axis line
top = P[P[:,2]>P[:,2].max()-0.01]
print("ridge center", np.round(top[:,:2].mean(0),4), "n", len(top))
# basket interior diff
bi = (pc[...,0]>-0.06)&(pc[...,0]<0.045)&(pc[...,1]>0.19)&(pc[...,1]<0.32)
d = pc[...,2]-pc0[...,2]
print("basket interior: before zmed", np.round(np.median(pc0[bi][:,2]),3), "after zmed", np.round(np.median(pc[bi][:,2]),3),
      "max after", np.round(pc[bi][:,2].max(),3), "pixels raised >1.5cm:", int((d[bi]>0.015).sum()))
raised = bi & (d>0.015)
if raised.any():
    R = pc[raised]; print("raised region center", np.round(R[:,:2].mean(0),3), "z range", np.round(R[:,2].min(),3), np.round(R[:,2].max(),3),
                          "x", np.round(R[:,0].min(),3), np.round(R[:,0].max(),3), "y", np.round(R[:,1].min(),3), np.round(R[:,1].max(),3))
EOF
python3 seg2.py; python3 -c "
import cv2; im=cv2.imread('bird3.png'); cv2.imwrite('bird3_crop.png', cv2.resize(im[240:320,320:420],None,fx=5,fy=5,interpolation=cv2.INTER_CUBIC))"

# openrua op 39
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rb.py", "old_string": "    def gripper(self, width):", "new_string": "    def line(self, a, b, quat, step=0.02, seconds=None, at_tcp=True):\\n        \\"\\"\\"Straight Cartesian TCP path a->b as one multi-point trajectory\\n        (IK per waypoint, each seeded with the previous solution).\\"\\"\\"\\n        a, b = np.array(a, float), np.array(b, float)\\n        n = max(1, int(np.ceil(np.linalg.norm(b - a) / step)))\\n        seed = self.joints()\\n        pts = []\\n        for i in range(1, n + 1):\\n            q = self.ik_world(a + (b - a) * i / n, quat, at_tcp, seed=seed)\\n            seed = dict(zip(ARM, q))\\n            pts.append(q)\\n        seconds = seconds or max(2.0, n * 0.6)\\n        code, err = self.traj(pts, seconds)\\n        if code != 0 or err > 0.02:\\n            self.log(\\"retrying final point\\")\\n            code, err = self.traj([pts[-1]], 2.0)\\n        p, hq, tcp = self.hand_pose()\\n        self.log(f\\"line done tcp {np.round(tcp, 4)} (target {np.round(b, 4)})\\")\\n        return code, err, tcp\\n\\n    def gripper(self, width):", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 40
mkdir -p "$(dirname /workspace/p4_can.py)"
cat > /workspace/p4_can.py <<'OPENRUA_EOF'
"""Grasp the lying tomato can perpendicular to its axis, drop in basket.
Usage: p4_can.py <cx> <cy> <axis_deg> <ztop>"""
import sys
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from rb import RB, q_down_yaw

cx, cy, axis, ztop = map(float, sys.argv[1:5])
yaw = axis            # finger axis at 90+yaw = axis+90 -> perpendicular to can
gz = ztop - 0.039     # pads around the can's centre height, palm clear of top
pz = ztop + 0.08
DROP = (0.0, 0.26, 0.72)

r = RB("p4")
q = q_down_yaw(yaw)
if r.fingers()[0] < 0.035:
    r.gripper(0.04)

r.log("== pregrasp")
r.goto([cx, cy, pz], q, seconds=5.0)
_, hq, _ = r.hand_pose()
fy = Rot.from_quat(hq).as_matrix()[:, 1]
fang = np.degrees(np.arctan2(fy[1], fy[0])) % 180
r.log(f"finger axis {fang:.1f} deg, want {(90 + yaw) % 180:.1f}")
if abs(((fang - (90 + yaw)) + 90) % 180 - 90) > 5:
    r.log("!! finger axis wrong"); r.close(); sys.exit(3)

r.log("== straight descent")
code, err, tcp = r.line([cx, cy, pz], [cx, cy, gz], q, step=0.02)
if np.linalg.norm(tcp[:2] - [cx, cy]) > 0.006 or abs(tcp[2] - gz) > 0.006:
    r.log("!! descent off; backing out")
    r.line([cx, cy, tcp[2]], [cx, cy, pz], q)
    r.close(); sys.exit(2)

r.log("== close")
f = r.gripper(0.0)
gap = f[0] - f[1]
r.log(f"gap = {gap:.4f} (fingers {f[0]:.4f} {f[1]:.4f}); expect ~0.065")
if not (0.055 < gap < 0.075):
    r.log("!! bad grasp; releasing and backing out")
    r.gripper(0.04)
    r.line([cx, cy, gz], [cx, cy, pz], q)
    r.close(); sys.exit(2)

r.log("== lift straight")
r.line([cx, cy, gz], [cx, cy, 0.72], q, step=0.04)
f = r.fingers(); r.log(f"gap after lift {f[0]-f[1]:.4f}")
r.snap("agentview", "/workspace/can_lifted.png")
if f[0] - f[1] < 0.05:
    r.log("!! lost the can during lift"); r.close(); sys.exit(2)

r.log("== to basket")
r.goto(DROP, q, seconds=5.0)
f = r.fingers(); r.log(f"gap over basket {f[0]-f[1]:.4f}")
if f[0] - f[1] < 0.05:
    r.log("!! lost the can in transit"); r.close(); sys.exit(2)
r.log("== release")
r.gripper(0.04)
r.spin(0.5)
r.goto([DROP[0], DROP[1], 0.85], q, seconds=3.0)
r.snap("agentview", "/workspace/can_dropped.png")
r.close()
OPENRUA_EOF

# openrua op 41
timeout 1700 python3 -u p4_can.py -0.0822 0.0894 52.0 0.501 2>&1 | tee p4.log | tail -40

# openrua op 42
timeout 900 python3 -u p3_resurvey.py 2>&1 | tail -2 && python3 - <<'EOF'
import numpy as np, cv2
pc0 = np.load("/workspace/bird_pc.npy"); pc = np.load("/workspace/bird_pc3.npy")
z = pc[...,2]
bi = (pc[...,0]>-0.06)&(pc[...,0]<0.045)&(pc[...,1]>0.19)&(pc[...,1]<0.32)
d = z - pc0[...,2]
print("basket interior raised >1.5cm px:", int((d[bi]>0.015).sum()), "zmax", np.round(z[bi].max(),3))
# anything left on the table in the old can area / anywhere outside basket above table?
tbl = (z>0.437)&(z<0.6)&(pc[...,0]>-0.4)&(pc[...,0]<0.25)&(np.abs(pc[...,1])<0.45)
n, lab = cv2.connectedComponents(tbl.astype(np.uint8))
for i in range(1,n):
    m = lab==i
    if m.sum()<15: continue
    P=pc[m]; print(f"blob center=({P[:,0].mean():.3f},{P[:,1].mean():.3f}) n={m.sum()} ztop={P[:,2].max():.3f}")
im=cv2.imread('bird3.png'); cv2.imwrite('bird4_crop.png', cv2.resize(im[240:320,320:420],None,fx=5,fy=5,interpolation=cv2.INTER_CUBIC))
EOF
timeout 120 python3 tools/perception/cam_snap.py frontview fv_final.png >/dev/null

# openrua op 43
mkdir -p "$(dirname /claude-config/projects/-workspace/memory/panda-ik-frame-and-hand-yaw.md)"
cat > /claude-config/projects/-workspace/memory/panda-ik-frame-and-hand-yaw.md <<'OPENRUA_EOF'
---
name: panda-ik-frame-and-hand-yaw
description: Verified quirks of the /workspace Franka Panda sim that contradict docs/30-action.md — IK frame_id semantics and the 45° link8/hand yaw offset
metadata:
  type: project
---

On the Franka Panda workstation in /workspace (MoveIt `/compute_ik`, group `panda_arm`), two facts differ from what `docs/30-action.md` "Planning facts" claims (verified 2026-09-12 via TF and IK probes):

1. An empty `frame_id` in a GetPositionIK request is interpreted in the **`world`** frame (TF has world→panda_link0 at (-0.51, 0, 0.42)), NOT the arm base. `frame_id: "panda_link0"` is also accepted and transformed correctly (no -21 error). Sending base-frame coords with empty frame_id moves the arm to the wrong place.
2. The IK group's tip link is `panda_link8`; `panda_hand` is yawed -45° from it (`panda_hand_joint rpy 0 0 -0.785`). So the finger axis (hand Y) ends up at `link8_yaw - 45°`. To get fingers along world Y, command link8 with yaw -45° on top of the flip-down quaternion. `/workspace/rb.py::q_down_yaw` already compensates.

Also useful: gripper max opening measures ~7.2 cm (nominal 8); joint-space descents bow by ~1 cm over 15 cm, so tight grasps (can ⌀6.5 cm) need a multi-waypoint straight-line descent (`rb.line`). Trajectory error -5 on long goals is lag; resending converges.

**Why:** the generic docs say the opposite for (1) and don't mention (2); both cost a failed grasp before being diagnosed.
**How to apply:** trust TF/probe results over docs; reuse `/workspace/rb.py` if it still exists. See also [[panda-workspace-object-layout]].
OPENRUA_EOF

# openrua op 44
cd /claude-config/projects/-workspace/memory && ls && (test -f MEMORY.md && cat MEMORY.md || true)

# openrua op 45
mkdir -p "$(dirname /claude-config/projects/-workspace/memory/MEMORY.md)"
cat > /claude-config/projects/-workspace/memory/MEMORY.md <<'OPENRUA_EOF'
- [Panda IK frame & hand yaw quirks](panda-ik-frame-and-hand-yaw.md) — empty IK frame_id = world (not base); fingers sit at link8_yaw − 45°; docs say otherwise
OPENRUA_EOF
