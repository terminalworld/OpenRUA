#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace; echo ---; find /workspace -maxdepth 2 -type f | head -50

# openrua op 1
cat /workspace/tools/README.md; echo ---; find /workspace/tools /workspace/.ros -type f | head; echo ---; ls -la /workspace/.ros

# openrua op 2
timeout 30 ros2 topic list; echo "--- actions"; timeout 30 ros2 action list; echo "--- services"; timeout 30 ros2 service list | grep -v -E "parameter|describe_|get_type" ; echo "--- nodes"; timeout 30 ros2 node list

# openrua op 3
mkdir -p "$(dirname /workspace/tools/cam_snap.py)"
cat > /workspace/tools/cam_snap.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Snapshot color (and optionally depth) frames from one or more cameras.

usage: cam_snap.py <cam> [<cam> ...] [--depth]
writes <cam>.png (color) and, with --depth, <cam>_depth.npy (float32 metres).
"""
import sys
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

args = [a for a in sys.argv[1:] if not a.startswith("--")]
want_depth = "--depth" in sys.argv
rclpy.init()
node = rclpy.create_node("cam_snap")
bridge = CvBridge()
got = {}
subs = []
for cam in args:
    topics = {f"{cam}.png": f"/{cam}/color/image_raw"}
    if want_depth:
        topics[f"{cam}_depth.npy"] = f"/{cam}/depth/image_raw"
    for out, topic in topics.items():
        def cb(msg, out=out):
            got.setdefault(out, msg)
        subs.append(node.create_subscription(Image, topic, cb, qos_profile_sensor_data))
        subs.append(node.create_subscription(Image, topic, cb, 1))
need = len(args) * (2 if want_depth else 1)
import time
t0 = time.time()
while len(got) < need and time.time() - t0 < 60:
    rclpy.spin_once(node, timeout_sec=0.2)
for out, msg in got.items():
    if out.endswith(".png"):
        img = bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imwrite(out, img)
        print(out, img.shape)
    else:
        d = bridge.imgmsg_to_cv2(msg, "passthrough").astype(np.float32)
        np.save(out, d)
        print(out, d.shape, "min", np.nanmin(d), "max", np.nanmax(d))
missing = need - len(got)
if missing:
    print("MISSING", missing, "frames", file=sys.stderr)
node.destroy_node()
rclpy.shutdown()
OPENRUA_EOF

# openrua op 4
mkdir -p snaps && cd snaps && timeout 120 python3 ../tools/cam_snap.py agentview birdview frontview sideview galleryview paperview robot0_robotview robot0_eye_in_hand 2>&1 | tail -20; echo ---; timeout 20 ros2 topic echo /joint_states --once

# openrua op 5
cd /workspace && timeout 20 ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id|child_frame_id|x:|y:|z:|w:" | head -80; echo ---; timeout 15 ros2 topic echo /tf --once 2>&1 | grep -E "frame_id" | sort | uniq -c

# openrua op 6
timeout 20 ros2 topic echo /tf --once 2>&1 | grep -B2 -A20 "child_frame_id: panda_link1" | head -30; echo ---; for c in agentview birdview frontview sideview; do echo "== $c"; timeout 15 ros2 topic echo /$c/color/camera_info --once 2>&1 | grep -E "frame_id|^k:|^- " | head -12; done

# openrua op 7
timeout 20 ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id" ; echo ---; timeout 20 ros2 topic echo /robot_description --once --qos-durability transient_local --qos-reliability reliable 2>/dev/null > robot_description.txt; wc -c robot_description.txt; grep -oE 'link name="[^"]+"' robot_description.txt | sort -u; grep -iE "camera|optical" robot_description.txt | head

# openrua op 8
cat robot_description.txt; echo; timeout 30 ros2 service call /get_urdf std_srvs/srv/Trigger 2>&1 | head -c 300; echo; timeout 10 ros2 service type /get_urdf

# openrua op 9
timeout 60 python3 -c "
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
rclpy.init(); n=rclpy.create_node('rd')
qos=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
got=[]
n.create_subscription(String,'/robot_description',got.append,qos)
import time; t=time.time()
while not got and time.time()-t<30: rclpy.spin_once(n,timeout_sec=0.2)
open('robot.urdf','w').write(got[0].data if got else '')
print(len(got[0].data) if got else 'none')
"; grep -oE '<link name="[^"]+"' robot.urdf | sort -u; grep -iE "camera|optical" robot.urdf | head; grep -oE '<joint name="[^"]+" type="[^"]+"' robot.urdf

# openrua op 10
mkdir -p "$(dirname /workspace/tools/robot.py)"
cat > /workspace/tools/robot.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Shared helpers: joint state, FK, IK, trajectory, gripper. Import or run.

  python3 tools/robot.py state          # joints + hand pose (base frame)
  python3 tools/robot.py move j1,...,j7 [sec]
  python3 tools/robot.py grip open|close
  python3 tools/robot.py ik x y z qx qy qz qw [sec]   # plan via IK and move (TCP pose)
"""
import sys, time, math
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from moveit_msgs.msg import RobotState
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped, Pose
from builtin_interfaces.msg import Duration

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
JOINTS = FJT["joints"]
LIMITS = FJT["limits_rad"]
TCP = M["hand"]["tcp_offset_m"]


def quat_to_mat(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def mat_to_quat(R):
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    else:
        i = np.argmax(np.diag(R))
        if i == 0:
            s = math.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            w = (R[2, 1] - R[1, 2]) / s; x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s; z = (R[0, 2] + R[2, 0]) / s
        elif i == 1:
            s = math.sqrt(1 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            w = (R[0, 2] - R[2, 0]) / s; x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s; z = (R[1, 2] + R[2, 1]) / s
        else:
            s = math.sqrt(1 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
            w = (R[1, 0] - R[0, 1]) / s; x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s; z = 0.25 * s
    return np.array([x, y, z, w])


class Robot(Node):
    def __init__(self):
        super().__init__("robot_helper")
        self._js = None
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.fk_cli = self.create_client(GetPositionFK, M["planning"]["ik_service"].replace("ik", "fk"))
        self.ik_cli = self.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.traj_cli = ActionClient(self, FollowJointTrajectory, FJT["port"])
        self.grip_cli = ActionClient(self, GripperCommand, GRIP["port"])

    def _on_js(self, msg):
        self._js = msg

    # ---- sensing
    def joints(self, fresh=True):
        if fresh:
            self._js = None
        while self._js is None:
            rclpy.spin_once(self, timeout_sec=0.2)
        d = dict(zip(self._js.name, self._js.position))
        return d

    def arm_q(self):
        d = self.joints()
        return np.array([d[j] for j in JOINTS])

    def finger_gap(self):
        d = self.joints()
        return d["panda_finger_joint1"] - d["panda_finger_joint2"]

    def _call(self, cli, req, timeout=60):
        while not cli.wait_for_service(timeout_sec=1.0):
            pass
        fut = cli.call_async(req)
        t0 = time.time()
        while not fut.done() and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return fut.result()

    def fk(self, q=None, link="panda_hand"):
        """Return (pos, quat[xyzw], R) of link in panda_link0 frame."""
        if q is None:
            q = self.arm_q()
        req = GetPositionFK.Request()
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = list(JOINTS)
        req.robot_state.joint_state.position = [float(v) for v in q]
        res = self._call(self.fk_cli, req)
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {res and res.error_code.val}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z])
        quat = np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        return pos, quat, quat_to_mat(quat)

    def tcp(self, q=None):
        pos, quat, R = self.fk(q)
        return pos + R @ np.array([0, 0, TCP]), quat, R

    def ik(self, pos, quat, seed=None, at_tcp=True, attempts=5):
        """pos/quat of the TCP (or panda_hand if at_tcp False) in panda_link0. Returns q or None."""
        pos = np.asarray(pos, float)
        R = quat_to_mat(quat)
        if at_tcp:
            pos = pos - R @ np.array([0, 0, TCP])
        if seed is None:
            seed = self.arm_q()
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.ik_link_name = "panda_hand"
        req.ik_request.robot_state.joint_state.name = list(JOINTS)
        req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
        req.ik_request.avoid_collisions = False
        ps = PoseStamped()
        ps.header.frame_id = ""
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = map(float, pos)
        ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = map(float, quat)
        req.ik_request.pose_stamped = ps
        req.ik_request.timeout.sec = 2
        for _ in range(attempts):
            res = self._call(self.ik_cli, req)
            if res is not None and res.error_code.val == 1:
                d = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
                return np.array([d[j] for j in JOINTS])
            # perturb seed and retry
            req.ik_request.robot_state.joint_state.position = [
                float(v + np.random.uniform(-0.3, 0.3)) for v in seed]
        return None

    # ---- acting
    def move(self, q_list, seconds, wait=True):
        """q_list: one q (7,) or list of waypoints; seconds: total duration (evenly spaced)."""
        q_list = np.atleast_2d(np.asarray(q_list, float))
        for q in q_list:
            for i, (v, (lo, hi)) in enumerate(zip(q, LIMITS)):
                if v < lo - 1e-6 or v > hi + 1e-6:
                    raise ValueError(f"joint {i+1} value {v:.3f} outside [{lo},{hi}]")
        while not self.traj_cli.wait_for_server(timeout_sec=1.0):
            pass
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        n = len(q_list)
        for i, q in enumerate(q_list):
            pt = JointTrajectoryPoint()
            pt.positions = [float(v) for v in q]
            pt.velocities = [0.0] * 7 if (i == n - 1) else []
            t = seconds * (i + 1) / n
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t - int(t)) * 1e9))
            goal.trajectory.points.append(pt)
        fut = self.traj_cli.send_goal_async(goal)
        while not fut.done():
            rclpy.spin_once(self, timeout_sec=0.1)
        gh = fut.result()
        if not gh.accepted:
            raise RuntimeError("trajectory goal rejected")
        if not wait:
            return gh
        rf = gh.get_result_async()
        while not rf.done():
            rclpy.spin_once(self, timeout_sec=0.1)
        res = rf.result().result
        err = np.abs(self.arm_q() - q_list[-1]).max()
        print(f"traj done: error_code={res.error_code} max_joint_err={err:.4f}", flush=True)
        return res.error_code, err

    def grip(self, open_=True, wait=True):
        while not self.grip_cli.wait_for_server(timeout_sec=1.0):
            pass
        goal = GripperCommand.Goal()
        goal.command.position = float(GRIP["open_m"] if open_ else GRIP["closed_m"])
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip_cli.send_goal_async(goal)
        while not fut.done():
            rclpy.spin_once(self, timeout_sec=0.1)
        gh = fut.result()
        rf = gh.get_result_async()
        while not rf.done():
            rclpy.spin_once(self, timeout_sec=0.1)
        r = rf.result().result
        print(f"grip {'open' if open_ else 'close'}: pos={r.position:.4f} reached={r.reached_goal} stalled={r.stalled} gap={self.finger_gap():.4f}", flush=True)
        return r


def main():
    rclpy.init()
    r = Robot()
    cmd = sys.argv[1] if len(sys.argv) > 1 else "state"
    if cmd == "state":
        q = r.arm_q()
        print("q =", np.round(q, 4).tolist())
        print("fingers gap =", round(r.finger_gap(), 4))
        pos, quat, R = r.fk(q)
        print("hand pos =", np.round(pos, 4).tolist(), "quat xyzw =", np.round(quat, 4).tolist())
        tpos, _, _ = r.tcp(q)
        print("tcp  pos =", np.round(tpos, 4).tolist())
        print("hand R =\n", np.round(R, 3))
    elif cmd == "move":
        q = [float(v) for v in sys.argv[2].split(",")]
        sec = float(sys.argv[3]) if len(sys.argv) > 3 else 3.0
        r.move(q, sec)
        print("now q =", np.round(r.arm_q(), 4).tolist())
    elif cmd == "grip":
        r.grip(sys.argv[2] == "open")
    elif cmd == "ik":
        v = [float(x) for x in sys.argv[2:9]]
        sec = float(sys.argv[9]) if len(sys.argv) > 9 else 3.0
        q = r.ik(v[:3], v[3:])
        if q is None:
            print("IK FAILED"); sys.exit(1)
        print("ik q =", np.round(q, 4).tolist())
        r.move(q, sec)
        tpos, _, _ = r.tcp()
        print("tcp now =", np.round(tpos, 4).tolist())
    r.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 11
timeout 120 python3 tools/robot.py state

# openrua op 12
cd snaps && timeout 60 python3 ../tools/cam_snap.py robot0_eye_in_hand agentview birdview --depth 2>&1; python3 -c "
import numpy as np
d=np.load('robot0_eye_in_hand_depth.npy'); print('eih center', d[240,320], 'table corner', d[200,100], 'mug', d[30,290], 'fingers', d[390,230])
d=np.load('agentview_depth.npy'); print('agent table', d[400,320], 'mug', d[260,330])
"

# openrua op 13
cd /workspace && timeout 20 ros2 topic echo /robot0_eye_in_hand/color/camera_info --once 2>&1 | grep -A9 "^k:" | tr '\n' ' '; echo; timeout 300 python3 tools/robot.py ik -0.03 0.0 1.25 1 0 0 0 3

# openrua op 14
timeout 300 python3 tools/robot.py move 0.0,0.242,0.0,-1.7238,0.0,1.9658,0.7854 3 && timeout 60 python3 tools/robot.py state | head -4

# openrua op 15
cd snaps && timeout 60 python3 ../tools/cam_snap.py robot0_eye_in_hand agentview --depth 2>&1 | grep -v png

# openrua op 16
mkdir -p "$(dirname /workspace/tools/eih.py)"
cat > /workspace/tools/eih.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Eye-in-hand camera geometry. Camera sits at (0.05,0,0) in panda_hand,
looking along hand +z; image-up = hand +x, image-right = hand +y.

  python3 tools/eih.py snap            # save eih.png / eih_depth.npy in cwd
  python3 tools/eih.py px u v          # world point for pixel (uses live FK + depth)
"""
import sys
import numpy as np

F = 312.77408948188935
CX, CY = 320.0, 240.0
CAM_IN_HAND = np.array([0.05, 0.0, 0.0])


def px_to_hand(u, v, d):
    X = (u - CX) / F * d   # image right  -> hand +y
    Y = (v - CY) / F * d   # image down   -> hand -x
    return CAM_IN_HAND + np.array([-Y, X, d])


def px_to_world(u, v, d, hand_pos, hand_R):
    return hand_pos + hand_R @ px_to_hand(u, v, d)


def cloud(depth, hand_pos, hand_R, step=1):
    """Full depth image -> (H,W,3) world points."""
    H, W = depth.shape
    us, vs = np.meshgrid(np.arange(W), np.arange(H))
    X = (us - CX) / F * depth
    Y = (vs - CY) / F * depth
    P_hand = np.stack([CAM_IN_HAND[0] - Y, CAM_IN_HAND[1] + X, CAM_IN_HAND[2] + depth], -1)
    return hand_pos + P_hand @ hand_R.T


if __name__ == "__main__":
    import rclpy
    sys.path.insert(0, "/workspace/tools")
    from robot import Robot
    import subprocess
    rclpy.init()
    r = Robot()
    pos, quat, R = r.fk()
    if sys.argv[1] == "px":
        u, v = int(sys.argv[2]), int(sys.argv[3])
        d = np.load("robot0_eye_in_hand_depth.npy")[v, u]
        print("depth", d, "world", np.round(px_to_world(u, v, d, pos, R), 4))
    r.destroy_node(); rclpy.shutdown()
OPENRUA_EOF

# openrua op 17
timeout 120 python3 -c "
import sys; sys.path.insert(0,'/workspace/tools')
import numpy as np, cv2, rclpy
from robot import Robot
from eih import cloud
rclpy.init(); r=Robot(); pos,quat,R=r.fk(); print('hand',pos)
d=np.load('robot0_eye_in_hand_depth.npy'); img=cv2.imread('robot0_eye_in_hand.png')
P=cloud(d,pos,R)
print('table z sample', P[100,100], P[50,600])
# mask: everything above table by >2cm and not the gripper (bottom rows)
above = (P[:,:,2] > 0.93)
above[360:,:] = False
# connected components
n,lab,stats,cent = cv2.connectedComponentsWithStats(above.astype(np.uint8))
for i in range(1,n):
    if stats[i,4] < 200: continue
    m = lab==i
    pts=P[m]
    bgr = img[m].mean(0)
    print(f'comp {i}: px area {stats[i,4]} centroid px {cent[i].round(0)} world x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] zmax {pts[:,2].max():.3f} color bgr {bgr.round(0)}')
r.destroy_node(); rclpy.shutdown()
"

# openrua op 18
python3 -c "
import numpy as np, cv2
d=np.load('birdview_depth.npy'); img=cv2.imread('birdview.png')
print('table', d[380,320], 'box top', d[260,250], 'door', d[207,190], 'yellow mug', d[298,328], 'gray mug', d[300,425], 'robot base', d[60,320], 'floor', d[450,100])
# objects above table: depth < table-0.02
tab = d[380,320]
m = d < tab-0.03
m[:190,:]=False  # ignore robot
n,lab,stats,cent=cv2.connectedComponentsWithStats(m.astype(np.uint8))
for i in range(1,n):
    if stats[i,4]<50: continue
    x,y,w,h,a=stats[i]; print(i,'bbox',x,y,w,h,'area',a,'min depth',d[lab==i].min().round(3), 'median', np.median(d[lab==i]).round(3))
"


# openrua op 19
python3 -c "
import cv2
img=cv2.imread('birdview.png'); crop=img[170:340,120:320]; cv2.imwrite('bird_crop.png', cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST))
img=cv2.imread('agentview.png'); crop=img[90:360,0:280]; cv2.imwrite('agent_crop.png', cv2.resize(crop,None,fx=2.5,fy=2.5,interpolation=cv2.INTER_NEAREST))
"

# openrua op 20
timeout 120 python3 -c "
import sys; sys.path.insert(0,'/workspace/tools')
import numpy as np, rclpy
from robot import Robot
rclpy.init(); r=Robot()
for l in ['panda_link0','panda_link1','panda_link4','panda_link8','panda_hand','panda_leftfinger']:
    p,q,R=r.fk(link=l); print(l, np.round(p,3), np.round(q,3))
r.destroy_node(); rclpy.shutdown()
"

# openrua op 21
timeout 300 python3 -c "
import sys; sys.path.insert(0,'/workspace/tools')
import numpy as np, rclpy
from robot import Robot, mat_to_quat
rclpy.init(); r=Robot()
R=np.array([[0,0,1],[0,-1,0],[1,0,0]],float)  # hand z=+x world, hand x=+z world
q_=mat_to_quat(R); print('quat',q_)
for tcp in [(-0.47,-0.21,1.02),(-0.45,-0.20,1.05),(-0.50,-0.18,1.05)]:
    q=r.ik(tcp,q_)
    print(tcp, None if q is None else np.round(q,3))
    if q is not None:
        r.move(q,4); print('tcp now', np.round(r.tcp()[0],3)); break
r.destroy_node(); rclpy.shutdown()
" && cd snaps && timeout 60 python3 ../tools/cam_snap.py robot0_eye_in_hand --depth

# openrua op 22
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/tools/robot.py", "old_string": "            if res is not None and res.error_code.val == 1:\\n                d = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))\\n                return np.array([d[j] for j in JOINTS])", "new_string": "            if res is not None and res.error_code.val == 1:\\n                d = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))\\n                q = np.array([d[j] for j in JOINTS])\\n                if all(lo + 0.05 <= v <= hi - 0.05 for v, (lo, hi) in zip(q, LIMITS)):\\n                    return q\\n                print(\\"ik: solution near joint limit, retrying\\", np.round(q, 2), flush=True)", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 23
timeout 400 python3 -c "
import sys; sys.path.insert(0,'/workspace/tools')
import numpy as np, rclpy
from robot import Robot, mat_to_quat
rclpy.init(); r=Robot()
R=np.array([[0,0,1],[0,-1,0],[1,0,0]],float)
q_=mat_to_quat(R)
for tcp in [(-0.47,-0.21,1.02),(-0.45,-0.20,1.05),(-0.50,-0.18,1.05),(-0.45,-0.15,1.10)]:
    q=r.ik(tcp,q_,attempts=8)
    print(tcp, None if q is None else np.round(q,3))
    if q is not None:
        r.move(q,4); print('tcp now', np.round(r.tcp()[0],3)); break
r.destroy_node(); rclpy.shutdown()
" && cd snaps && timeout 60 python3 ../tools/cam_snap.py robot0_eye_in_hand --depth

# openrua op 24
timeout 60 python3 ../tools/robot.py state | head -5; timeout 60 python3 ../tools/cam_snap.py agentview robot0_eye_in_hand

# openrua op 25
timeout 400 python3 -c "
import sys; sys.path.insert(0,'/workspace/tools')
import numpy as np, rclpy
from robot import Robot
rclpy.init(); r=Robot()
pos,quat,R=r.tcp()
q=r.ik(pos+np.array([0,0.05,0.12]),quat)
print('lift q', np.round(q,3), 'delta', np.round(q-r.arm_q(),2))
r.move(q,3); print('tcp now', np.round(r.tcp()[0],3))
home=[0.0,0.2415,0.0,-1.7249,0.0,1.9659,0.7854]
r.move(home,4); print('tcp now', np.round(r.tcp()[0],3))
r.destroy_node(); rclpy.shutdown()
"

# openrua op 26
timeout 400 python3 ../tools/robot.py move 0.0,0.2415,0.0,-1.7249,0.0,1.9659,0.7854 5

# openrua op 27
timeout 60 ros2 service call /query_planner_interface moveit_msgs/srv/QueryPlannerInterfaces 2>&1 | grep -oE "pipeline_id='[^']*'|planner_ids=\[[^]]{0,300}" | head; timeout 20 ros2 interface show moveit_msgs/srv/GetMotionPlan | head -5; timeout 20 ros2 interface show moveit_msgs/msg/MotionPlanRequest | grep -vE "^\s*#" | grep -v "^$" | head -40

# openrua op 28
timeout 120 python3 -c "
import rclpy
from moveit_msgs.srv import GetPlanningScene
from moveit_msgs.msg import PlanningSceneComponents
rclpy.init(); n=rclpy.create_node('ps')
c=n.create_client(GetPlanningScene,'/get_planning_scene'); c.wait_for_service()
req=GetPlanningScene.Request(); req.components.components=PlanningSceneComponents.SCENE_SETTINGS|PlanningSceneComponents.TRANSFORMS|PlanningSceneComponents.WORLD_OBJECT_NAMES|PlanningSceneComponents.ROBOT_STATE
f=c.call_async(req)
while not f.done(): rclpy.spin_once(n,timeout_sec=0.1)
s=f.result().scene
print('model', s.robot_model_name, 'fixed frames', [(t.header.frame_id,t.child_frame_id) for t in s.fixed_frame_transforms])
print('world objs', [o.id for o in s.world.collision_objects])
print('multi dof', s.robot_state.multi_dof_joint_state.joint_names, s.robot_state.multi_dof_joint_state.header.frame_id, s.robot_state.joint_state.header.frame_id)
print('attached', [a.object.id for a in s.robot_state.attached_collision_objects])
"

# openrua op 29
timeout 120 python3 -c "
import rclpy, json, numpy as np
from moveit_msgs.srv import GetPlanningScene
from moveit_msgs.msg import PlanningSceneComponents
rclpy.init(); n=rclpy.create_node('ps')
c=n.create_client(GetPlanningScene,'/get_planning_scene'); c.wait_for_service()
req=GetPlanningScene.Request(); req.components.components=PlanningSceneComponents.TRANSFORMS
f=c.call_async(req)
while not f.done(): rclpy.spin_once(n,timeout_sec=0.1)
out={}
for t in f.result().scene.fixed_frame_transforms:
    tr=t.transform.translation; q=t.transform.rotation
    out[t.header.frame_id]=dict(t=[tr.x,tr.y,tr.z],q=[q.x,q.y,q.z,q.w])
    print(t.header.frame_id,'->',t.child_frame_id, np.round([tr.x,tr.y,tr.z],3), np.round([q.x,q.y,q.z,q.w],3))
json.dump(out,open('/workspace/cam_extrinsics.json','w'),indent=1)
"

# openrua op 30
mkdir -p "$(dirname /workspace/tools/cams.py)"
cat > /workspace/tools/cams.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Fixed-camera geometry from /workspace/cam_extrinsics.json (pose of the
optical frame in world; optical: x right, y down, z forward)."""
import json
import numpy as np
import sys
sys.path.insert(0, "/workspace/tools")
from robot import quat_to_mat

EXT = json.load(open("/workspace/cam_extrinsics.json"))
F = {"default": 579.4112549695428, "robot0_eye_in_hand": 312.77408948188935}
CX, CY = 320.0, 240.0


def cam_pose(name):
    e = EXT[f"{name}_optical_frame"]
    return np.array(e["t"]), quat_to_mat(e["q"])


def cloud(name, depth, pos=None, R=None):
    """(H,W) depth -> (H,W,3) world points."""
    if pos is None:
        pos, R = cam_pose(name)
    f = F.get(name, F["default"])
    H, W = depth.shape
    us, vs = np.meshgrid(np.arange(W), np.arange(H))
    X = (us - CX) / f * depth
    Y = (vs - CY) / f * depth
    P = np.stack([X, Y, depth], -1)
    return pos + P @ R.T


def project(name, pts):
    """world points (N,3) -> pixel (N,2) and depth (N,)"""
    pos, R = cam_pose(name)
    f = F.get(name, F["default"])
    Pc = (np.asarray(pts) - pos) @ R
    u = Pc[:, 0] / Pc[:, 2] * f + CX
    v = Pc[:, 1] / Pc[:, 2] * f + CY
    return np.stack([u, v], -1), Pc[:, 2]
OPENRUA_EOF

# openrua op 31
timeout 60 python3 ../tools/cam_snap.py birdview agentview frontview sideview --depth >/dev/null 2>&1; timeout 60 python3 -c "
import sys; sys.path.insert(0,'/workspace/tools')
import numpy as np, cv2
from cams import cloud
d=np.load('birdview_depth.npy'); P=cloud('birdview',d); img=cv2.imread('birdview.png')
print('table z', P[380,320], 'floor', P[450,100,2])
z=P[:,:,2]
m=(z>0.93)&(z<1.5); m[:185,:]=False
n,lab,stats,cent=cv2.connectedComponentsWithStats(m.astype(np.uint8))
for i in range(1,n):
    if stats[i,4]<40: continue
    pts=P[lab==i]; print(i,'area',stats[i,4],'x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]'%(pts[:,0].min(),pts[:,0].max(),pts[:,1].min(),pts[:,1].max(),pts[:,2].min(),pts[:,2].max()), 'bgr', img[lab==i].mean(0).round())
# door bar details: rows 195-220 cols 140-235
sub=P[195:220,140:235]; zs=sub[:,:,2]; msk=zs>1.0
print('door: x[%.3f,%.3f] y[%.3f,%.3f] ztop %.3f'%(sub[msk][:,0].min(),sub[msk][:,0].max(),sub[msk][:,1].min(),sub[msk][:,1].max(),zs[msk].max()))
# box body: rows 215-320 cols 224-290 where z>1.09
sub=P[205:325,215:295]; msk=sub[:,:,2]>1.09
print('boxtop: x[%.3f,%.3f] y[%.3f,%.3f]'%(sub[msk][:,0].min(),sub[msk][:,0].max(),sub[msk][:,1].min(),sub[msk][:,1].max()))
"

# openrua op 32
timeout 60 python3 -c "
import sys; sys.path.insert(0,'/workspace/tools')
import numpy as np, cv2
from cams import cloud
for cam in ['agentview','frontview','sideview','birdview']:
    d=np.load(cam+'_depth.npy'); P=cloud(cam,d)
    # microwave region: y<-0.08, x<0.1, z>0.92
    m=(P[:,:,1]<-0.08)&(P[:,:,1]>-0.6)&(P[:,:,0]<0.15)&(P[:,:,0]>-0.5)&(P[:,:,2]>0.905)&(P[:,:,2]<1.2)
    pts=P[m]
    print(cam, 'microwave+door pts', len(pts), 'x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]'%(pts[:,0].min(),pts[:,0].max(),pts[:,1].min(),pts[:,1].max(),pts[:,2].min(),pts[:,2].max()))
    # box only: y>-0.27
    b=pts[pts[:,1]>-0.27]
    print('   box(y>-0.27): x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]'%(b[:,0].min(),b[:,0].max(),b[:,1].min(),b[:,1].max(),b[:,2].min(),b[:,2].max()))
    # table top z
    t=P[(np.abs(P[:,:,2]-0.9)<0.01)]; print('   table z mean %.4f n=%d'%(t[:,2].mean(),len(t)))
    # yellow mug region
    m=(np.abs(P[:,:,0]-0.0)<0.08)&(np.abs(P[:,:,1]-0.03)<0.09)&(P[:,:,2]>0.91)&(P[:,:,2]<1.05)
    pts=P[m]
    if len(pts): print('   mug pts',len(pts),'x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]'%(pts[:,0].min(),pts[:,0].max(),pts[:,1].min(),pts[:,1].max(),pts[:,2].min(),pts[:,2].max()))
"

# openrua op 33
timeout 60 python3 -c "
import sys; sys.path.insert(0,'/workspace/tools')
import numpy as np, cv2
from cams import cloud
for cam in ['agentview','birdview','frontview']:
    d=np.load(cam+'_depth.npy'); P=cloud(cam,d)
    m=(P[:,:,1]<-0.29)&(P[:,:,1]>-0.65)&(P[:,:,0]<-0.2)&(P[:,:,0]>-0.5)&(P[:,:,2]>0.905)&(P[:,:,2]<1.2)
    pts=P[m]
    print(cam, len(pts))
    # histogram of x for door points binned by y
    for ylo in np.arange(-0.58,-0.28,0.05):
        s=pts[(pts[:,1]>=ylo)&(pts[:,1]<ylo+0.05)]
        if len(s): print('  y[%.2f,%.2f] n=%4d x[%.3f,%.3f] z[%.3f,%.3f]'%(ylo,ylo+0.05,len(s),s[:,0].min(),s[:,0].max(),s[:,2].min(),s[:,2].max()))
    # box front face: points near x=-0.295 with y in [-0.3,-0.11]
    m=(P[:,:,1]<-0.11)&(P[:,:,1]>-0.32)&(P[:,:,0]<-0.25)&(P[:,:,0]>-0.36)&(P[:,:,2]>0.905)&(P[:,:,2]<1.2)
    pts=P[m]
    if len(pts):
        for ylo in np.arange(-0.32,-0.11,0.03):
            s=pts[(pts[:,1]>=ylo)&(pts[:,1]<ylo+0.03)]
            if len(s): print('  frontface y[%.2f,%.2f] n=%4d x[%.3f,%.3f] z[%.3f,%.3f]'%(ylo,ylo+0.03,len(s),s[:,0].min(),s[:,0].max(),s[:,2].min(),s[:,2].max()))
"

# openrua op 34
mkdir -p "$(dirname /workspace/tools/planner.py)"
cat > /workspace/tools/planner.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Collision-aware planning on top of robot.Robot.

  python3 tools/planner.py scene            # publish the world model (table, microwave, door, mugs)
  python3 tools/planner.py goto x y z qx qy qz qw   # plan (OMPL) to TCP pose and execute
"""
import sys, time
import numpy as np
import rclpy
from moveit_msgs.srv import GetMotionPlan, ApplyPlanningScene, GetCartesianPath
from moveit_msgs.msg import (PlanningScene, CollisionObject, Constraints, JointConstraint,
                             RobotState, AttachedCollisionObject, PositionConstraint, OrientationConstraint)
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose, PoseStamped
from control_msgs.action import FollowJointTrajectory

sys.path.insert(0, "/workspace/tools")
from robot import Robot, JOINTS, TCP, quat_to_mat, mat_to_quat, M

TABLE_Z = 0.90


def box(id_, lo, hi, frame="world"):
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    co = CollisionObject()
    co.header.frame_id = frame
    co.id = id_
    p = SolidPrimitive(); p.type = SolidPrimitive.BOX
    p.dimensions = list(hi - lo)
    pose = Pose()
    c = (lo + hi) / 2
    pose.position.x, pose.position.y, pose.position.z = map(float, c)
    pose.orientation.w = 1.0
    co.primitives.append(p); co.primitive_poses.append(pose)
    co.operation = CollisionObject.ADD
    return co


def cylinder(id_, cx, cy, r, z0, z1, frame="world"):
    co = CollisionObject()
    co.header.frame_id = frame
    co.id = id_
    p = SolidPrimitive(); p.type = SolidPrimitive.CYLINDER
    p.dimensions = [float(z1 - z0), float(r)]
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = float(cx), float(cy), float((z0 + z1) / 2)
    pose.orientation.w = 1.0
    co.primitives.append(p); co.primitive_poses.append(pose)
    co.operation = CollisionObject.ADD
    return co


def remove(id_):
    co = CollisionObject(); co.id = id_; co.header.frame_id = "world"
    co.operation = CollisionObject.REMOVE
    return co


# ---- world model (measured from camera clouds)
MW_LO = np.array([-0.295, -0.315, TABLE_Z])
MW_HI = np.array([0.050, -0.115, 1.110])
DOOR_LO = np.array([-0.320, -0.590, TABLE_Z])
DOOR_HI = np.array([-0.292, -0.290, 1.110])
HANDLE_LO = np.array([-0.360, -0.585, TABLE_Z + 0.02])
HANDLE_HI = np.array([-0.320, -0.525, 1.090])
GRAY_MUG = (0.0, 0.345, 0.065)
YELLOW_MUG = (0.0, 0.03, 0.055)


def scene_objects(microwave="solid", yellow=True, door=True):
    objs = [box("table", [-0.55, -0.65, TABLE_Z - 0.06], [0.50, 0.65, TABLE_Z])]
    if microwave == "solid":
        objs.append(box("mw", MW_LO, MW_HI))
    elif microwave == "walls":
        # hollow: opening on the -y face. wall thickness ~2 cm
        t = 0.02
        objs.append(box("mw_top", [MW_LO[0], MW_LO[1], MW_HI[2] - t], MW_HI))
        objs.append(box("mw_bottom", MW_LO, [MW_HI[0], MW_HI[1], MW_LO[2] + t]))
        objs.append(box("mw_back", [MW_LO[0], MW_HI[1] - t, MW_LO[2]], MW_HI))          # +y wall
        objs.append(box("mw_left", MW_LO, [MW_LO[0] + t, MW_HI[1], MW_HI[2]]))            # -x wall
        objs.append(box("mw_right", [MW_HI[0] - 0.08, MW_LO[1], MW_LO[2]], MW_HI))      # +x wall incl. control panel
    if door:
        objs.append(box("door", DOOR_LO, DOOR_HI))
        objs.append(box("handle", HANDLE_LO, HANDLE_HI))
    objs.append(cylinder("gray_mug", GRAY_MUG[0], GRAY_MUG[1], GRAY_MUG[2], TABLE_Z, 1.02))
    if yellow:
        objs.append(cylinder("yellow_mug", YELLOW_MUG[0], YELLOW_MUG[1], YELLOW_MUG[2], TABLE_Z, 1.01))
    return objs


ALL_IDS = ["table", "mw", "mw_top", "mw_bottom", "mw_back", "mw_left", "mw_right", "door", "handle",
           "gray_mug", "yellow_mug"]


class Planner(Robot):
    def __init__(self):
        super().__init__()
        self.plan_cli = self.create_client(GetMotionPlan, "/plan_kinematic_path")
        self.scene_cli = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.cart_cli = self.create_client(GetCartesianPath, "/compute_cartesian_path")

    def apply_scene(self, objs, remove_ids=(), attached=None):
        ps = PlanningScene(); ps.is_diff = True
        for i in remove_ids:
            ps.world.collision_objects.append(remove(i))
        ps.world.collision_objects.extend(objs)
        if attached is not None:
            ps.robot_state.is_diff = True
            ps.robot_state.attached_collision_objects.extend(attached)
        req = ApplyPlanningScene.Request(); req.scene = ps
        res = self._call(self.scene_cli, req)
        return res.success

    def set_scene(self, **kw):
        return self.apply_scene(scene_objects(**kw), remove_ids=ALL_IDS)

    def attach_mug(self, r=0.055, h=0.105, z_off=0.0):
        """Attach a cylinder (the mug) to panda_hand at the fingers."""
        aco = AttachedCollisionObject()
        aco.link_name = "panda_hand"
        aco.touch_links = ["panda_hand", "panda_leftfinger", "panda_rightfinger"]
        co = CollisionObject(); co.header.frame_id = "panda_hand"; co.id = "held_mug"
        p = SolidPrimitive(); p.type = SolidPrimitive.CYLINDER; p.dimensions = [h, r]
        pose = Pose(); pose.position.z = TCP + z_off
        pose.orientation.w = 1.0
        co.primitives.append(p); co.primitive_poses.append(pose); co.operation = CollisionObject.ADD
        aco.object = co
        return self.apply_scene([], attached=[aco])

    def detach_mug(self):
        aco = AttachedCollisionObject(); aco.link_name = "panda_hand"
        aco.object.id = "held_mug"; aco.object.operation = CollisionObject.REMOVE
        return self.apply_scene([remove("held_mug")], attached=[aco])

    def _start_state(self):
        rs = RobotState()
        rs.joint_state.name = list(JOINTS)
        rs.joint_state.position = [float(v) for v in self.arm_q()]
        return rs

    def plan_joint(self, q_goal, pipeline="ompl", planner_id="RRTConnect", tries=3, time_s=5.0):
        req = GetMotionPlan.Request()
        r = req.motion_plan_request
        r.group_name = M["planning"]["group"]
        r.pipeline_id = pipeline
        r.planner_id = planner_id
        r.num_planning_attempts = 4
        r.allowed_planning_time = time_s
        r.max_velocity_scaling_factor = 0.4
        r.max_acceleration_scaling_factor = 0.4
        r.start_state = self._start_state()
        c = Constraints()
        for n, v in zip(JOINTS, q_goal):
            jc = JointConstraint(); jc.joint_name = n; jc.position = float(v)
            jc.tolerance_above = jc.tolerance_below = 0.005; jc.weight = 1.0
            c.joint_constraints.append(jc)
        r.goal_constraints.append(c)
        for _ in range(tries):
            res = self._call(self.plan_cli, req, timeout=time_s + 30)
            if res is not None and res.motion_plan_response.error_code.val == 1:
                return res.motion_plan_response.trajectory.joint_trajectory
            print("plan failed:", res and res.motion_plan_response.error_code.val, flush=True)
        return None

    def cartesian(self, waypoints_tcp, quat, step=0.01, jump=5.0, avoid=True):
        """Straight-line Cartesian path through TCP waypoints (world). Returns (traj, fraction)."""
        R = quat_to_mat(quat)
        req = GetCartesianPath.Request()
        req.header.frame_id = "world"
        req.start_state = self._start_state()
        req.group_name = M["planning"]["group"]
        req.link_name = "panda_hand"
        req.max_step = step
        req.jump_threshold = jump
        req.avoid_collisions = avoid
        req.max_velocity_scaling_factor = 0.3
        req.max_acceleration_scaling_factor = 0.3
        for w in waypoints_tcp:
            p = Pose()
            hp = np.asarray(w, float) - R @ np.array([0, 0, TCP])
            p.position.x, p.position.y, p.position.z = map(float, hp)
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
            req.waypoints.append(p)
        res = self._call(self.cart_cli, req, timeout=60)
        if res is None or res.error_code.val != 1:
            print("cartesian failed:", res and res.error_code.val, flush=True)
            return None, 0.0
        return res.solution.joint_trajectory, res.fraction

    def execute(self, jt, time_scale=1.5, min_time=1.5):
        """Send a planned JointTrajectory to the controller (positions only, rescaled time)."""
        names = list(jt.joint_names)
        idx = [names.index(j) for j in JOINTS]
        pts = []
        for p in jt.points:
            t = (p.time_from_start.sec + p.time_from_start.nanosec * 1e-9) * time_scale
            pts.append((t, [p.positions[i] for i in idx]))
        total = max(pts[-1][0], min_time)
        if pts[-1][0] < min_time and pts[-1][0] > 0:
            k = min_time / pts[-1][0]
            pts = [(t * k, q) for t, q in pts]
        # drop the zero-time first point if it equals the current state
        qs = [q for _, q in pts]
        # robot.move spaces points evenly; build the goal ourselves for exact timing
        from trajectory_msgs.msg import JointTrajectoryPoint
        from builtin_interfaces.msg import Duration
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        for i, (t, q) in enumerate(pts):
            if i == 0 and t == 0.0:
                continue
            pt = JointTrajectoryPoint(); pt.positions = [float(v) for v in q]
            if i == len(pts) - 1:
                pt.velocities = [0.0] * 7
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t - int(t)) * 1e9))
            goal.trajectory.points.append(pt)
        while not self.traj_cli.wait_for_server(timeout_sec=1.0):
            pass
        fut = self.traj_cli.send_goal_async(goal)
        while not fut.done():
            rclpy.spin_once(self, timeout_sec=0.1)
        gh = fut.result()
        rf = gh.get_result_async()
        while not rf.done():
            rclpy.spin_once(self, timeout_sec=0.1)
        res = rf.result().result
        err = np.abs(self.arm_q() - np.array(qs[-1])).max()
        print(f"exec done: error_code={res.error_code} pts={len(goal.trajectory.points)} T={total:.1f}s max_joint_err={err:.4f}", flush=True)
        if err > 0.02:  # converge with a short resend of the final point
            self.move(qs[-1], 2.0)
            err = np.abs(self.arm_q() - np.array(qs[-1])).max()
        return res.error_code, err

    def goto_q(self, q, **kw):
        jt = self.plan_joint(q, **kw)
        if jt is None:
            return False
        code, err = self.execute(jt)
        return err < 0.02

    def goto_tcp(self, pos, quat, seed=None, **kw):
        q = self.ik(pos, quat, seed=seed, attempts=10)
        if q is None:
            print("IK failed for", pos, flush=True)
            return False
        ok = self.goto_q(q, **kw)
        tp = self.tcp()[0]
        print("tcp now", np.round(tp, 4), "target", np.round(pos, 4), "ok", ok, flush=True)
        return ok

    def line(self, pos_to, quat=None, step=0.01, avoid=True, min_fraction=0.95, time_scale=1.5):
        """Straight-line TCP move from the current pose to pos_to keeping orientation."""
        p0, q0, _ = self.tcp()
        if quat is None:
            quat = q0
        jt, frac = self.cartesian([pos_to], quat, step=step, avoid=avoid)
        if jt is None or frac < min_fraction:
            print(f"line: fraction {frac:.2f} < {min_fraction}", flush=True)
            return False
        self.execute(jt, time_scale=time_scale)
        tp = self.tcp()[0]
        print("tcp now", np.round(tp, 4), "target", np.round(pos_to, 4), flush=True)
        return np.linalg.norm(tp - pos_to) < 0.01


def main():
    rclpy.init()
    p = Planner()
    cmd = sys.argv[1]
    if cmd == "scene":
        print("scene ok:", p.set_scene())
    elif cmd == "goto":
        v = [float(x) for x in sys.argv[2:9]]
        p.goto_tcp(v[:3], v[3:])
    elif cmd == "line":
        v = [float(x) for x in sys.argv[2:5]]
        p.line(np.array(v))
    p.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 35
cd /workspace && timeout 120 python3 tools/planner.py scene && timeout 60 python3 -c "
import rclpy
from moveit_msgs.srv import GetPlanningScene
from moveit_msgs.msg import PlanningSceneComponents
rclpy.init(); n=rclpy.create_node('ps')
c=n.create_client(GetPlanningScene,'/get_planning_scene'); c.wait_for_service()
req=GetPlanningScene.Request(); req.components.components=PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
f=c.call_async(req)
while not f.done(): rclpy.spin_once(n,timeout_sec=0.1)
for o in f.result().scene.world.collision_objects: print(o.id, o.header.frame_id, [round(v,3) for v in o.primitives[0].dimensions], round(o.pose.position.x,3), round(o.pose.position.y,3), round(o.pose.position.z,3), '| prim pose', round(o.primitive_poses[0].position.x,3), round(o.primitive_poses[0].position.y,3), round(o.primitive_poses[0].position.z,3))
"

# openrua op 36
timeout 600 python3 -c "
import sys; sys.path.insert(0,'/workspace/tools')
import numpy as np, rclpy
from planner import Planner
from robot import mat_to_quat, TCP
rclpy.init(); p=Planner()
def look_pose(cam_pos, pitch_deg):
    th=np.radians(pitch_deg)
    z=np.array([0,np.cos(th),-np.sin(th)])   # view dir: +y, pitched down
    x=np.array([0,np.sin(th),np.cos(th)])    # image up
    y=np.cross(z,x)
    R=np.stack([x,y,z],1)
    hand=np.asarray(cam_pos)-0.05*x
    return hand+R@np.array([0,0,TCP]), mat_to_quat(R)
for cam,pitch in [((-0.12,-0.58,1.18),35),((-0.12,-0.55,1.22),40),((-0.15,-0.62,1.15),30)]:
    tcp,q=look_pose(cam,pitch); print('try',cam,pitch,'tcp',np.round(tcp,3))
    if p.goto_tcp(tcp,q): break
p.destroy_node(); rclpy.shutdown()
" && cd snaps && timeout 60 python3 ../tools/cam_snap.py robot0_eye_in_hand agentview --depth

# openrua op 37
timeout 600 python3 -c "
import sys; sys.path.insert(0,'/workspace/tools')
import numpy as np, rclpy
from planner import Planner
from robot import mat_to_quat, TCP
rclpy.init(); p=Planner()
def look_pose(cam_pos, pitch_deg, roll=0):
    th=np.radians(pitch_deg)
    z=np.array([0,np.cos(th),-np.sin(th)])
    x=np.array([0,np.sin(th),np.cos(th)])
    if roll: x=-x
    y=np.cross(z,x)
    R=np.stack([x,y,z],1)
    hand=np.asarray(cam_pos)-0.05*x
    return hand+R@np.array([0,0,TCP]), mat_to_quat(R)
sols=[]
for roll in [0,1]:
  for cam in [(-0.12,-0.55,1.2),(-0.2,-0.5,1.25),(-0.05,-0.5,1.25),(-0.12,-0.45,1.3)]:
    for pitch in [30,45,60]:
        tcp,q=look_pose(cam,pitch,roll)
        s=p.ik(tcp,q,attempts=6)
        print(roll,cam,pitch,'->', None if s is None else np.round(s,2))
        if s is not None: sols.append((cam,pitch,roll,tcp,q,s))
        if len(sols)>=1: break
    if sols: break
  if sols: break
if sols:
    cam,pitch,roll,tcp,q,s=sols[0]
    ok=p.goto_q(s); print('goto ok',ok, 'tcp', np.round(p.tcp()[0],3))
p.destroy_node(); rclpy.shutdown()
" && cd snaps && timeout 60 python3 ../tools/cam_snap.py robot0_eye_in_hand agentview --depth

# openrua op 38
timeout 60 python3 ../tools/cam_snap.py robot0_eye_in_hand agentview sideview >/dev/null && timeout 60 python3 ../tools/robot.py state | head -3

# openrua op 39
timeout 60 python3 ../tools/cam_snap.py robot0_eye_in_hand --depth >/dev/null; timeout 120 python3 -c "
import sys; sys.path.insert(0,'/workspace/tools')
import numpy as np, rclpy, cv2
from robot import Robot
from eih import cloud
rclpy.init(); r=Robot(); pos,quat,R=r.fk()
d=np.load('robot0_eye_in_hand_depth.npy'); P=cloud(d,pos,R)
# points inside the microwave footprint, at z between 0.9 and 1.11
m=(P[:,:,0]>-0.30)&(P[:,:,0]<0.06)&(P[:,:,2]>0.895)&(P[:,:,2]<1.12)&(P[:,:,1]>-0.36)&(P[:,:,1]<0.0)&(d>0.05)
pts=P[m]; print('n',len(pts))
# face plane: y histogram
h,e=np.histogram(pts[:,1],bins=np.arange(-0.36,0.0,0.01)); 
for c,lo in zip(h,e[:-1]): 
    if c>50: print('  y[%.2f,%.2f] %d'%(lo,lo+0.01,c))
# recessed points (y>-0.29): extents by x,z
rec=pts[pts[:,1]>-0.29]
print('cavity pts', len(rec), 'x[%.3f,%.3f] z[%.3f,%.3f] ymax %.3f'%(rec[:,0].min(),rec[:,0].max(),rec[:,2].min(),rec[:,2].max(),rec[:,1].max()))
# cavity interior: percentiles
for q in [1,5,50,95,99]: print('  pct',q,'x %.3f z %.3f y %.3f'%tuple(np.percentile(rec,q,axis=0)))
# face points (y in [-0.33,-0.30])
face=pts[(pts[:,1]>-0.335)&(pts[:,1]<-0.30)]
print('face pts x[%.3f,%.3f] z[%.3f,%.3f]'%(face[:,0].min(),face[:,0].max(),face[:,2].min(),face[:,2].max()))
# save mask image
vis=cv2.imread('robot0_eye_in_hand.png'); vis[m & (P[:,:,1]>-0.29)]=(0,255,0); cv2.imwrite('eih_cavity.png',vis)
r.destroy_node(); rclpy.shutdown()
"

# openrua op 40
timeout 120 python3 -c "
import sys; sys.path.insert(0,'/workspace/tools')
import numpy as np, rclpy
from robot import Robot
from eih import cloud
rclpy.init(); r=Robot(); pos,quat,R=r.fk()
d=np.load('robot0_eye_in_hand_depth.npy'); P=cloud(d,pos,R)
m=(P[:,:,0]>-0.30)&(P[:,:,0]<0.06)&(P[:,:,2]>0.895)&(P[:,:,2]<1.095)&(P[:,:,1]>-0.29)&(P[:,:,1]<-0.10)&(d>0.05)
pts=P[m]; print('interior pts',len(pts))
for q in [0.5,2,50,98,99.5]: print('  pct',q,'x %.3f y %.3f z %.3f'%tuple(np.percentile(pts,q,axis=0)))
# back wall
bw=pts[pts[:,1]>-0.16]; print('back wall x[%.3f,%.3f] z[%.3f,%.3f] y median %.3f'%(bw[:,0].min(),bw[:,0].max(),bw[:,2].min(),bw[:,2].max(),np.median(bw[:,1])))
# floor: points with z<0.96
fl=pts[pts[:,2]<0.96]; print('floor z pct', np.percentile(fl[:,2],[2,50,98]).round(3), 'n',len(fl))
# ceiling
ce=pts[pts[:,2]>1.07]; print('ceiling z pct', np.percentile(ce[:,2],[2,50,98]).round(3), 'n',len(ce))
# side walls
lw=pts[pts[:,0]<-0.26]; print('left wall x pct', np.percentile(lw[:,0],[2,50,98]).round(3))
rw=pts[pts[:,0]>-0.03]; print('right wall x pct', np.percentile(rw[:,0],[2,50,98]).round(3), 'n', len(rw))
# opening bezel: face points at y in [-0.33,-0.30]: find the hole by x,z grid
face=P[(P[:,:,1]>-0.335)&(P[:,:,1]<-0.295)&(P[:,:,2]>0.9)&(P[:,:,2]<1.1)&(P[:,:,0]>-0.3)&(P[:,:,0]<0.06)]
import collections
xs=np.arange(-0.30,0.06,0.02)
for x0 in xs:
    s=face[(face[:,0]>=x0)&(face[:,0]<x0+0.02)]
    if len(s): 
        h,_=np.histogram(s[:,2],bins=np.arange(0.9,1.12,0.02)); print('x %.2f'%x0, ''.join('#' if c>20 else '.' for c in h))
r.destroy_node(); rclpy.shutdown()
"

# openrua op 41
timeout 60 python3 ../tools/cam_snap.py agentview frontview sideview robot0_robotview --depth >/dev/null; timeout 60 python3 -c "
import sys; sys.path.insert(0,'/workspace/tools')
import numpy as np
from cams import cloud
for cam in ['agentview','frontview','robot0_robotview','sideview']:
    d=np.load(cam+'_depth.npy'); P=cloud(cam,d)
    m=(np.abs(P[:,:,0])<0.1)&(P[:,:,1]>-0.09)&(P[:,:,1]<0.12)&(P[:,:,2]>0.905)&(P[:,:,2]<1.03)
    pts=P[m]; print(cam,len(pts))
    for z0 in np.arange(0.905,1.01,0.01):
        s=pts[(pts[:,2]>=z0)&(pts[:,2]<z0+0.01)]
        if len(s)>5: print('  z %.3f n=%4d x[%.3f,%.3f] y[%.3f,%.3f]'%(z0,len(s),s[:,0].min(),s[:,0].max(),s[:,1].min(),s[:,1].max()))
"

# openrua op 42
timeout 120 python3 -c "
import sys; sys.path.insert(0,'/workspace/tools')
import numpy as np, rclpy
from robot import Robot
from eih import cloud
from cams import cloud as fcloud
# handle from robotview + agentview
for cam in ['robot0_robotview','agentview','frontview']:
    d=np.load(cam+'_depth.npy'); P=fcloud(cam,d)
    m=(np.abs(P[:,:,0])<0.1)&(P[:,:,1]>-0.07)&(P[:,:,1]<-0.028)&(P[:,:,2]>0.91)&(P[:,:,2]<1.0)
    pts=P[m]
    if len(pts): print(cam,'handle pts',len(pts),'x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]'%(pts[:,0].min(),pts[:,0].max(),pts[:,1].min(),pts[:,1].max(),pts[:,2].min(),pts[:,2].max()), 'x pct', np.percentile(pts[:,0],[5,50,95]).round(3))
# opening bezel from eih
rclpy.init(); r=Robot(); pos,quat,R=r.fk()
d=np.load('robot0_eye_in_hand_depth.npy'); P=cloud(d,pos,R)
face=P[(P[:,:,1]>-0.335)&(P[:,:,1]<-0.295)&(P[:,:,2]>0.9)&(P[:,:,2]<1.1)&(P[:,:,0]>-0.3)&(P[:,:,0]<0.06)]
print('face y pct', np.percentile(face[:,1],[5,50,95]).round(3))
for x0 in np.arange(-0.30,0.06,0.02):
    s=face[(face[:,0]>=x0)&(face[:,0]<x0+0.02)]
    h,_=np.histogram(s[:,2],bins=np.arange(0.9,1.12,0.01)); print('x %.2f'%x0, ''.join('#' if c>10 else '.' for c in h), len(s))
print('z bins from 0.90 step 0.01')
# opening edges: for interior points near the opening plane (y in [-0.29,-0.25]) extents
inner=P[(P[:,:,1]>-0.29)&(P[:,:,1]<-0.25)&(P[:,:,2]>0.9)&(P[:,:,2]<1.095)&(P[:,:,0]>-0.3)&(P[:,:,0]<0.06)]
print('near-opening interior x[%.3f,%.3f] z[%.3f,%.3f]'%(inner[:,0].min(),inner[:,0].max(),inner[:,2].min(),inner[:,2].max()))
r.destroy_node(); rclpy.shutdown()
"

# openrua op 43
cd /workspace && python3 - <<'EOF'
import re
s=open('tools/planner.py').read()
s=s.replace('print(f"exec done: error_code={res.error_code} pts={len(goal.trajectory.points)} T={total:.1f}s max_joint_err={err:.4f}", flush=True)',
 'print(f"exec done: error_code={res.error_code} ({res.error_string}) pts={len(goal.trajectory.points)} T={total:.1f}s max_joint_err={err:.4f}", flush=True)')
open('tools/planner.py','w').write(s)
EOF
timeout 900 python3 -c "
import sys; sys.path.insert(0,'/workspace/tools')
import numpy as np, rclpy, time
from planner import Planner
rclpy.init(); p=Planner()
home=[0.0,0.2415,0.0,-1.7249,0.0,1.9659,0.7854]
jt=p.plan_joint(home)
print('planned pts', len(jt.points), 'T', jt.points[-1].time_from_start.sec + jt.points[-1].time_from_start.nanosec*1e-9)
t0=time.time(); p.execute(jt, time_scale=3.0); print('wall', round(time.time()-t0,1))
print('q', np.round(p.arm_q(),3))
p.destroy_node(); rclpy.shutdown()
"

# openrua op 44
python3 - <<'EOF'
s=open('tools/planner.py').read()
old="""        while not self.traj_cli.wait_for_server(timeout_sec=1.0):
            pass
        fut = self.traj_cli.send_goal_async(goal)"""
new="""        # hold point: give the lagging controller time to settle before the goal is judged
        last = goal.trajectory.points[-1]
        hold = JointTrajectoryPoint(); hold.positions = list(last.positions); hold.velocities = [0.0] * 7
        th = pts[-1][0] + 1.5
        hold.time_from_start = Duration(sec=int(th), nanosec=int((th - int(th)) * 1e9))
        goal.trajectory.points.append(hold)
        while not self.traj_cli.wait_for_server(timeout_sec=1.0):
            pass
        fut = self.traj_cli.send_goal_async(goal)"""
assert old in s; s=s.replace(old,new); open('tools/planner.py','w').write(s)
r=open('tools/robot.py').read()
old="""            goal.trajectory.points.append(pt)
        fut = self.traj_cli.send_goal_async(goal)"""
new="""            goal.trajectory.points.append(pt)
        hold = JointTrajectoryPoint(); hold.positions = [float(v) for v in q_list[-1]]; hold.velocities = [0.0] * 7
        th = seconds + 1.5
        hold.time_from_start = Duration(sec=int(th), nanosec=int((th - int(th)) * 1e9))
        goal.trajectory.points.append(hold)
        fut = self.traj_cli.send_goal_async(goal)"""
assert old in r; r=r.replace(old,new); open('tools/robot.py','w').write(r)
print('patched')
EOF

# openrua op 45
mkdir -p "$(dirname /workspace/tools/task.py)"
cat > /workspace/tools/task.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Staged task: put the yellow/white mug in the microwave, close the door.
usage: python3 tools/task.py <stage>   (pregrasp | grasp | lift | preinsert | insert | release | retract)
"""
import sys
import numpy as np
import rclpy
sys.path.insert(0, "/workspace/tools")
from planner import Planner, remove, cylinder, TABLE_Z
from robot import mat_to_quat, quat_to_mat, TCP
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

# grasp orientation: hand z -> world +y (approach), hand x -> world -z, hand y -> world -x
R_G = np.array([[0, -1, 0],
                [0, 0, 1],
                [-1, 0, 0]], float)
Q_G = mat_to_quat(R_G)
# alternative roll (hand x -> +z)
R_G2 = np.array([[0, 1, 0],
                 [0, 0, 1],
                 [1, 0, 0]], float)
Q_G2 = mat_to_quat(R_G2)

HANDLE = np.array([-0.002, -0.050, 0.958])      # grip point on the mug handle (world)
PREGRASP = HANDLE + np.array([0, -0.12, 0])
LIFT_DZ = 0.056                                  # TCP z after lift: mug bottom ~1.2 cm above cavity floor (0.944)
MUG_X = -0.16                                    # cavity centre x
INSERT_Y = -0.295                                # TCP y at final insertion (mug centre ~ -0.215)
PREINSERT = np.array([MUG_X, -0.42, HANDLE[2] + LIFT_DZ])


def attach_mug(p, R):
    """Attach the mug (held by the handle) as a vertical cylinder ~8 cm in front of the fingertips."""
    aco = AttachedCollisionObject()
    aco.link_name = "panda_hand"
    aco.touch_links = ["panda_hand", "panda_leftfinger", "panda_rightfinger"]
    co = CollisionObject(); co.header.frame_id = "panda_hand"; co.id = "held_mug"
    prim = SolidPrimitive(); prim.type = SolidPrimitive.CYLINDER; prim.dimensions = [0.105, 0.05]
    pose = Pose()
    # mug centre in hand frame: 0.08 along z (approach), slight offset along x (down) for its centre height
    c_world_rel = np.array([0.0, 0.08, 0.9525 - HANDLE[2]])  # relative to TCP in world
    c_hand = R.T @ c_world_rel + np.array([0, 0, TCP])
    pose.position.x, pose.position.y, pose.position.z = map(float, c_hand)
    # cylinder axis (its z) must be world z: hand-frame direction of world z
    zh = R.T @ np.array([0, 0, 1.0])
    # rotation taking (0,0,1) to zh
    v = np.cross([0, 0, 1.0], zh); s = np.linalg.norm(v); c = np.dot([0, 0, 1.0], zh)
    if s < 1e-6:
        q = [0, 0, 0, 1] if c > 0 else [1, 0, 0, 0]
    else:
        ax = v / s; ang = np.arctan2(s, c)
        q = [*(ax * np.sin(ang / 2)), np.cos(ang / 2)]
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = map(float, q)
    co.primitives.append(prim); co.primitive_poses.append(pose); co.operation = CollisionObject.ADD
    aco.object = co
    return p.apply_scene([], attached=[aco])


def main():
    stage = sys.argv[1]
    rclpy.init()
    p = Planner()
    print("start tcp", np.round(p.tcp()[0], 4), "gap", round(p.finger_gap(), 4), flush=True)

    if stage == "pregrasp":
        p.set_scene(microwave="solid", yellow=True)
        p.grip(True)
        for q in (Q_G, Q_G2):
            print("trying orientation", np.round(q, 3), flush=True)
            if p.goto_tcp(PREGRASP, q):
                break
    elif stage == "grasp":
        # take the mug out of the collision world for the final approach
        p.apply_scene([], remove_ids=["yellow_mug"])
        ok = p.line(HANDLE, step=0.005, avoid=True)
        print("approach ok", ok, flush=True)
        p.grip(False)
        print("gap after close", round(p.finger_gap(), 4), flush=True)
    elif stage == "lift":
        pos, quat, R = p.tcp()
        attach_mug(p, R)
        ok = p.line(pos + np.array([0, 0, LIFT_DZ]), step=0.005, avoid=False)
        print("lift ok", ok, "gap", round(p.finger_gap(), 4), flush=True)
    elif stage == "preinsert":
        p.set_scene(microwave="walls", yellow=False)
        pos, quat, R = p.tcp()
        ok = p.goto_tcp(PREINSERT, quat)
        print("preinsert ok", ok, "gap", round(p.finger_gap(), 4), flush=True)
    elif stage == "insert":
        pos, quat, R = p.tcp()
        target = np.array([MUG_X, INSERT_Y, pos[2]])
        ok = p.line(target, step=0.005, avoid=False, time_scale=2.5)
        print("insert ok", ok, "gap", round(p.finger_gap(), 4), flush=True)
    elif stage == "release":
        p.grip(True)
        p.detach_mug()
        print("gap", round(p.finger_gap(), 4), flush=True)
    elif stage == "retract":
        pos, quat, R = p.tcp()
        ok = p.line(np.array([pos[0], -0.45, pos[2]]), step=0.005, avoid=False)
        print("retract ok", ok, flush=True)
    print("end tcp", np.round(p.tcp()[0], 4), "q", np.round(p.arm_q(), 3).tolist(), flush=True)
    p.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 46
timeout 900 python3 -u tools/task.py pregrasp 2>&1 | grep -v "^$"

# openrua op 47
timeout 900 python3 -c "
import sys; sys.path.insert(0,'/workspace/tools')
import numpy as np, rclpy
from robot import Robot, LIMITS, mat_to_quat
rclpy.init(); r=Robot()
def R_yawpitch(yaw_deg, pitch_deg, flip=False):
    yw=np.radians(yaw_deg); pt=np.radians(pitch_deg)
    z=np.array([-np.sin(yw)*np.cos(pt), np.cos(yw)*np.cos(pt), -np.sin(pt)])  # approach, base +y
    up=np.array([0,0,1.0]); x=-(up-np.dot(up,z)*z); x/=np.linalg.norm(x)   # hand x = down-ish
    if flip: x=-x
    y=np.cross(z,x); return np.stack([x,y,z],1)
lo=np.array([l[0] for l in LIMITS]); hi=np.array([l[1] for l in LIMITS])
targets=[(-0.002,-0.17,0.958)]
for yaw in [0,15,-15,30]:
  for pitch in [0,10]:
    for flip in [False,True]:
        R=R_yawpitch(yaw,pitch,flip); q=mat_to_quat(R)
        found=None
        for k in range(12):
            seed=np.random.uniform(lo+0.1,hi-0.1) if k>0 else None
            s=r.ik(targets[0],q,seed=seed,attempts=1)
            if s is not None: found=s; break
        print('yaw',yaw,'pitch',pitch,'flip',flip,'->',None if found is None else np.round(found,2), flush=True)
        if found is not None: break
    else: continue
    break
r.destroy_node(); rclpy.shutdown()
"

# openrua op 48
timeout 1500 python3 -c "
import sys; sys.path.insert(0,'/workspace/tools')
import numpy as np, rclpy
from robot import Robot, LIMITS, mat_to_quat
rclpy.init(); r=Robot()
def R_yaw(yaw_deg, flip=False):
    yw=np.radians(yaw_deg)
    z=np.array([-np.sin(yw), np.cos(yw), 0.0])
    x=np.array([0,0,-1.0]) if not flip else np.array([0,0,1.0])
    y=np.cross(z,x); return np.stack([x,y,z],1)
lo=np.array([l[0] for l in LIMITS]); hi=np.array([l[1] for l in LIMITS])
for (x,y,z) in [(-0.16,-0.45,1.014),(-0.16,-0.35,1.014),(-0.16,-0.295,1.014),(-0.16,-0.25,1.014),(-0.002,-0.17,0.958),(-0.002,-0.05,0.958),(-0.1,-0.2,0.96)]:
    for yaw in [0,-15,15]:
        for flip in [False,True]:
            q=mat_to_quat(R_yaw(yaw,flip)); found=None
            for k in range(25):
                seed=np.random.uniform(lo+0.1,hi-0.1) if k>0 else None
                s=r.ik((x,y,z),q,seed=seed,attempts=1)
                if s is not None: found=s; break
            print((x,y,z),'yaw',yaw,'flip',flip,'->',None if found is None else np.round(found,2), flush=True)
r.destroy_node(); rclpy.shutdown()
" 2>&1 | grep -v XMLPARSER

# openrua op 49
cat /tmp/claude-1000/-workspace/7ff900f3-ebfd-4964-89f8-b93583aa0267/tasks/blh14w5qq.output | grep -v XMLPARSER

# openrua op 50
timeout 1700 python3 -c "
import sys; sys.path.insert(0,'/workspace/tools')
import numpy as np, rclpy
from robot import Robot, LIMITS, mat_to_quat
rclpy.init(); r=Robot()
def R_yaw(yaw_deg, flip=False):
    yw=np.radians(yaw_deg); z=np.array([-np.sin(yw), np.cos(yw), 0.0])
    x=np.array([0,0,-1.0]) if not flip else np.array([0,0,1.0]); y=np.cross(z,x); return np.stack([x,y,z],1)
lo=np.array([l[0] for l in LIMITS]); hi=np.array([l[1] for l in LIMITS])
for yaw in [-15,-25,-35]:
    a=np.array([-np.sin(np.radians(yaw)), np.cos(np.radians(yaw)),0])
    for mug_y in [-0.30,-0.33,-0.36]:
        mug=np.array([-0.15,mug_y,0]); tcp=mug-0.08*a; tcp[2]=1.014
        for flip in [False,True]:
            q=mat_to_quat(R_yaw(yaw,flip)); found=None
            for k in range(30):
                seed=np.random.uniform(lo+0.1,hi-0.1) if k>0 else None
                s=r.ik(tcp,q,seed=seed,attempts=1)
                if s is not None: found=s; break
            print('yaw',yaw,'mug_y',mug_y,'tcp',np.round(tcp,3),'flip',flip,'->',None if found is None else np.round(found,2), flush=True)
r.destroy_node(); rclpy.shutdown()
" 2>&1 | grep -v XMLPARSER

# openrua op 51
grep -A12 '<link name="panda_hand"' robot.urdf | head -30; grep -A8 '<link name="panda_leftfinger"' robot.urdf | head -12; grep -oE 'filename="[^"]+"' robot.urdf | sort -u | head

# openrua op 52
python3 - <<'EOF'
import numpy as np, struct
def load_stl(fn):
    d=open(fn,'rb').read()
    if d[:5]==b'solid' and b'facet' in d[:300]:
        v=[list(map(float,l.split()[1:])) for l in d.decode().splitlines() if l.strip().startswith('vertex')]
        return np.array(v)
    n=struct.unpack('<I',d[80:84])[0]
    a=np.frombuffer(d[84:84+n*50],dtype=np.dtype([('n','<3f4'),('v','<9f4'),('a','<u2')]))
    return a['v'].reshape(-1,3).astype(float)
for f in ['hand','finger']:
    v=load_stl(f'/opt/ros/jazzy/share/moveit_resources_panda_description/meshes/collision/{f}.stl')
    print(f, 'min',v.min(0).round(4),'max',v.max(0).round(4))
    # hand: width in y at various z
    if f=='hand':
        for z0 in np.arange(0,0.07,0.01):
            m=(v[:,2]>=z0)&(v[:,2]<z0+0.01)
            if m.any(): print(f'  z {z0:.2f}: x[{v[m,0].min():.3f},{v[m,0].max():.3f}] y[{v[m,1].min():.3f},{v[m,1].max():.3f}]')
EOF

# openrua op 53
find / -iname 'hand*.stl' -o -iname 'hand*.obj' -o -iname 'finger*.stl' 2>/dev/null | grep -v proc | head; ls /opt/ros/jazzy/share 2>/dev/null | head -3

# openrua op 54
mkdir -p "$(dirname /workspace/tools/insert_geom.py)"
cat > /workspace/tools/insert_geom.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""2D (top view) footprint check of hand + held mug against the microwave/door
for the insertion path.  Hand pose: TCP at p, hand z = a(yaw) = (-sin, cos, 0),
hand x = -world z, hand y = (-cos, -sin, 0).  Mug centre = TCP + 0.0785*a."""
import numpy as np

GRIP_TO_CENTRE = 0.0785
MUG_R = 0.05
# obstacles as axis-aligned boxes (xlo, xhi, ylo, yhi)
OBS = [
    (-0.322, -0.290, -0.590, -0.285),    # open door panel
    (-0.300, -0.260, -0.320, -0.290),    # front wall, -x of opening
    (-0.060, 0.055, -0.320, -0.290),     # front wall, +x of opening (incl. control panel)
    (-0.300, -0.266, -0.320, -0.110),    # left side wall
    (-0.057, 0.055, -0.320, -0.110),     # right side wall + panel
    (-0.300, 0.055, -0.144, -0.110),     # back wall
]


def frame(yaw):
    a = np.array([-np.sin(yaw), np.cos(yaw)])
    yh = np.array([-np.cos(yaw), -np.sin(yaw)])
    return a, yh


def footprint(tcp, yaw, n=9):
    """Return list of (points, label) for hand body, fingers, mug body, handle."""
    a, yh = frame(yaw)
    tcp = np.asarray(tcp[:2], float)
    pts = {}
    # hand body: s in +-0.103, t in [-0.1034, -0.0374]
    s = np.linspace(-0.103, 0.103, 11); t = np.linspace(-0.1034, -0.0374, 5)
    S, T = np.meshgrid(s, t)
    pts["hand"] = tcp + S.reshape(-1, 1) * yh + T.reshape(-1, 1) * a
    s = np.linspace(-0.05, 0.05, 6); t = np.linspace(-0.045, 0.012, 4)
    S, T = np.meshgrid(s, t)
    pts["fingers"] = tcp + S.reshape(-1, 1) * yh + T.reshape(-1, 1) * a
    c = tcp + GRIP_TO_CENTRE * a
    th = np.linspace(0, 2 * np.pi, 48)
    pts["mug"] = c + MUG_R * np.stack([np.cos(th), np.sin(th)], -1)
    pts["handle"] = tcp + np.linspace(-0.012, 0.04, 6).reshape(-1, 1) * a
    return pts, c


def collide(tcp, yaw, margin=0.0):
    pts, c = footprint(tcp, yaw)
    hits = []
    for name, P in pts.items():
        for (xlo, xhi, ylo, yhi) in OBS:
            m = (P[:, 0] > xlo - margin) & (P[:, 0] < xhi + margin) & (P[:, 1] > ylo - margin) & (P[:, 1] < yhi + margin)
            if m.any():
                hits.append((name, (xlo, xhi, ylo, yhi)))
                break
    return hits


def tcp_for_mug(mug_c, yaw):
    a, _ = frame(yaw)
    return np.asarray(mug_c, float) - GRIP_TO_CENTRE * a


def check_path(waypoints, n=40, margin=0.0):
    """waypoints: list of (mug_x, mug_y, yaw_deg). Linear interpolation. Returns list of problems."""
    W = np.array(waypoints, float)
    probs = []
    for i in range(len(W) - 1):
        for u in np.linspace(0, 1, n, endpoint=(i == len(W) - 2)):
            w = W[i] * (1 - u) + W[i + 1] * u
            yaw = np.deg2rad(w[2])
            tcp = tcp_for_mug(w[:2], yaw)
            h = collide(tcp, yaw, margin)
            if h:
                probs.append((np.round(w, 3).tolist(), np.round(tcp, 3).tolist(), h))
    return probs


if __name__ == "__main__":
    import sys
    for yaw in (0, -15, -25, -35):
        for my in (-0.40, -0.37, -0.34, -0.31, -0.28, -0.25, -0.22):
            for mx in (-0.13, -0.15, -0.17, -0.19, -0.21):
                tcp = tcp_for_mug((mx, my), np.deg2rad(yaw))
                h = collide(tcp, np.deg2rad(yaw))
                print(f"yaw {yaw:4d} mug ({mx:.2f},{my:.2f}) tcp ({tcp[0]:.3f},{tcp[1]:.3f}) ->", "ok" if not h else [x[0] for x in h])
OPENRUA_EOF

# openrua op 55
python3 tools/insert_geom.py | grep -v "\['hand', 'fingers', 'mug'" | awk '{print}' | column -c 200 | head -80

# openrua op 56
python3 tools/insert_geom.py | grep ok

# openrua op 57
mkdir -p "$(dirname /workspace/tools/reach_probe.py)"
cat > /workspace/tools/reach_probe.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Continuation IK probe: march the TCP outward along -y from a feasible pose,
seeding each step with the previous solution. Reports the last feasible y.
usage: reach_probe.py yaw_deg pitch_deg x z y_start y_end"""
import sys
import numpy as np
import rclpy
sys.path.insert(0, "/workspace/tools")
from robot import Robot, mat_to_quat, LIMITS


def R_of(yaw, pitch):
    """hand z = approach a (yaw about world z from +y, pitch down), hand x ~ -world z."""
    cy, sy, cp, sp = np.cos(yaw), np.sin(yaw), np.cos(pitch), np.sin(pitch)
    a = np.array([-sy * cp, cy * cp, -sp])
    xh = np.array([-sy * sp, cy * sp, cp]) * -1.0        # ~ -z, tilted with pitch
    yh = np.cross(a, xh)
    return np.stack([xh, yh, a], 1)


def main():
    yaw, pitch, x, z, y0, y1 = map(float, sys.argv[1:7])
    rclpy.init(); r = Robot()
    R = R_of(np.deg2rad(yaw), np.deg2rad(pitch)); q_ = mat_to_quat(R)
    seed = None
    last = None
    for y in np.arange(y0, y1 - 1e-9, -0.01 if y1 < y0 else 0.01):
        q = r.ik([x, y, z], q_, seed=seed, attempts=8)
        if q is None:
            print(f"yaw {yaw:+.0f} pitch {pitch:+.0f}  y={y:.3f}: FAIL", flush=True)
            break
        last = (y, q)
        seed = q
        marg = min(min(v - lo, hi - v) for v, (lo, hi) in zip(q, LIMITS))
        print(f"yaw {yaw:+.0f} pitch {pitch:+.0f}  y={y:.3f}: ok  q={np.round(q, 2).tolist()} limit-margin {marg:.2f}", flush=True)
    if last:
        print("LAST_OK", round(last[0], 3), np.round(last[1], 4).tolist())
    r.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 58
(python3 tools/reach_probe.py 0 0 -0.17 1.014 -0.30 -0.50 2>&1 | tail -4; echo ---; python3 tools/reach_probe.py 0 15 -0.17 1.014 -0.30 -0.50 2>&1 | tail -4; echo ---; python3 tools/reach_probe.py 0 30 -0.17 1.014 -0.30 -0.50 2>&1 | tail -4; echo ---; python3 tools/reach_probe.py -15 0 -0.15 1.014 -0.30 -0.50 2>&1 | tail -4)

# openrua op 59
(python3 tools/reach_probe.py 0 40 -0.16 1.014 -0.36 -0.50 2>&1 | tail -3; echo ---; python3 tools/reach_probe.py 0 50 -0.16 1.014 -0.36 -0.50 2>&1 | tail -3; echo ---; python3 tools/reach_probe.py -10 40 -0.165 1.014 -0.36 -0.50 2>&1 | tail -3; echo ---; python3 tools/reach_probe.py -10 30 -0.165 1.014 -0.36 -0.50 2>&1 | tail -3)

# openrua op 60
grep -n "def \|^[A-Z_]* = " tools/planner.py

# openrua op 61
mkdir -p "$(dirname /workspace/tools/task.py)"
cat > /workspace/tools/task.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Staged task: put the yellow/white mug in the microwave, close the door.
usage: python3 tools/task.py <stage>
stages: pregrasp | grasp | lift | preinsert | insert | release | retract | park

Grasp geometry (from IK reach probes + 2D footprint check, see insert_geom.py):
hand approach a = yaw -10 deg from +y, pitched 40 deg down; fingers close along x on the
mug handle. Pitch raises the wrist (reach), small yaw keeps the hand body clear of the door.
"""
import sys
import numpy as np
import rclpy
sys.path.insert(0, "/workspace/tools")
from planner import Planner
from robot import mat_to_quat, TCP
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

YAW, PITCH = np.deg2rad(-10.0), np.deg2rad(40.0)


def R_of(yaw, pitch):
    cy, sy, cp, sp = np.cos(yaw), np.sin(yaw), np.cos(pitch), np.sin(pitch)
    a = np.array([-sy * cp, cy * cp, -sp])            # hand z (approach)
    xh = -np.array([-sy * sp, cy * sp, cp])           # hand x (~ -world z)
    yh = np.cross(a, xh)                              # hand y (finger axis, ~ -world x)
    return np.stack([xh, yh, a], 1)


R_G = R_of(YAW, PITCH)
Q_G = mat_to_quat(R_G)
A_XY = R_G[:2, 2]                                     # horizontal approach direction
GRIP_TO_CENTRE = 0.0785                               # handle grip point -> mug axis

HANDLE = np.array([-0.002, -0.050, 0.958])            # grip point on the handle (world)
PREGRASP = HANDLE - np.array([*(0.12 * A_XY / np.linalg.norm(A_XY)), 0.0])
LIFT_DZ = 0.056                                       # mug bottom ends ~1.2 cm above cavity floor
MUG_ENTRY = np.array([-0.160, -0.390])                # mug axis just outside the opening
MUG_FINAL = np.array([-0.160, -0.225])                # mug axis inside (handle end ~2.5 cm behind face)
Z_CARRY = HANDLE[2] + LIFT_DZ


def tcp_for_mug(mug_xy):
    return np.array([*(np.asarray(mug_xy) - GRIP_TO_CENTRE * A_XY), Z_CARRY])


PREINSERT = tcp_for_mug(MUG_ENTRY)
INSERT = tcp_for_mug(MUG_FINAL)
RETRACT = np.array([PREINSERT[0], -0.44, Z_CARRY])
HOME_Q = [-0.002, 0.244, 0.003, -1.725, -0.001, 1.961, 0.778]


def attach_mug(p):
    """Attach the held mug as a vertical cylinder in the hand frame."""
    aco = AttachedCollisionObject()
    aco.link_name = "panda_hand"
    aco.touch_links = ["panda_hand", "panda_leftfinger", "panda_rightfinger"]
    co = CollisionObject(); co.header.frame_id = "panda_hand"; co.id = "held_mug"
    prim = SolidPrimitive(); prim.type = SolidPrimitive.CYLINDER; prim.dimensions = [0.105, 0.05]
    pose = Pose()
    c_world_rel = np.array([*(GRIP_TO_CENTRE * A_XY), 0.9525 - HANDLE[2]])   # mug centre rel. TCP
    c_hand = R_G.T @ c_world_rel + np.array([0, 0, TCP])
    pose.position.x, pose.position.y, pose.position.z = map(float, c_hand)
    zh = R_G.T @ np.array([0, 0, 1.0])                # world z in hand frame = cylinder axis
    v = np.cross([0, 0, 1.0], zh); s = np.linalg.norm(v); c = np.dot([0, 0, 1.0], zh)
    if s < 1e-6:
        q = [0, 0, 0, 1] if c > 0 else [1, 0, 0, 0]
    else:
        ax = v / s; ang = np.arctan2(s, c)
        q = [*(ax * np.sin(ang / 2)), np.cos(ang / 2)]
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = map(float, q)
    co.primitives.append(prim); co.primitive_poses.append(pose); co.operation = CollisionObject.ADD
    aco.object = co
    return p.apply_scene([], attached=[aco])


def main():
    stage = sys.argv[1]
    if stage == "info":
        print("R_G=\n", np.round(R_G, 3)); print("Q_G", np.round(Q_G, 4))
        print("PREGRASP", np.round(PREGRASP, 4)); print("HANDLE", HANDLE)
        print("PREINSERT", np.round(PREINSERT, 4)); print("INSERT", np.round(INSERT, 4)); print("RETRACT", RETRACT)
        return
    rclpy.init()
    p = Planner()
    print("start tcp", np.round(p.tcp()[0], 4), "gap", round(p.finger_gap(), 4), flush=True)

    if stage == "pregrasp":
        p.set_scene(microwave="solid", yellow=True)
        p.grip(True)
        ok = p.goto_tcp(PREGRASP, Q_G)
        print("pregrasp ok", ok, flush=True)
    elif stage == "grasp":
        p.apply_scene([], remove_ids=["yellow_mug"])   # mug leaves the collision world for the approach
        ok = p.line(HANDLE, quat=Q_G, step=0.005, avoid=True)
        print("approach ok", ok, flush=True)
        p.grip(False)
        print("gap after close", round(p.finger_gap(), 4), flush=True)
    elif stage == "lift":
        attach_mug(p)
        pos = p.tcp()[0]
        ok = p.line(pos + np.array([0, 0, LIFT_DZ]), quat=Q_G, step=0.005, avoid=False)
        print("lift ok", ok, "gap", round(p.finger_gap(), 4), flush=True)
    elif stage == "preinsert":
        p.set_scene(microwave="walls", yellow=False)
        ok = p.goto_tcp(PREINSERT, Q_G, time_s=10.0)
        print("preinsert ok", ok, "gap", round(p.finger_gap(), 4), flush=True)
    elif stage == "insert":
        ok = p.line(INSERT, quat=Q_G, step=0.005, avoid=False, time_scale=2.5)
        print("insert ok", ok, "gap", round(p.finger_gap(), 4), flush=True)
    elif stage == "release":
        p.grip(True)
        p.detach_mug()
        print("gap", round(p.finger_gap(), 4), flush=True)
    elif stage == "retract":
        ok = p.line(RETRACT, quat=Q_G, step=0.005, avoid=False)
        print("retract ok", ok, flush=True)
    elif stage == "park":
        p.set_scene(microwave="solid", yellow=False)
        ok = p.goto_q(HOME_Q, time_s=10.0)
        print("park ok", ok, flush=True)
    print("end tcp", np.round(p.tcp()[0], 4), "q", np.round(p.arm_q(), 3).tolist(), flush=True)
    p.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 62
python3 tools/task.py info && python3 - <<'EOF'
import sys; sys.path.insert(0,'/workspace/tools')
import numpy as np
import insert_geom as g
# check the insertion straight line with yaw -10 (footprint check ignores pitch: conservative for body extent)
yaw=-10
for my in np.linspace(-0.39,-0.225,12):
    tcp=g.tcp_for_mug((-0.16,my),np.deg2rad(yaw))
    print(round(my,3), np.round(tcp,3), g.collide(tcp,np.deg2rad(yaw)) or 'ok')
EOF

# openrua op 63
python3 - <<'EOF'
import numpy as np, sys
sys.path.insert(0,'/workspace/tools')
from task import R_of
def margins(yaw_d, pitch_d, mug_x, mug_y_final=-0.225):
    R=R_of(np.deg2rad(yaw_d),np.deg2rad(pitch_d)); a=R[:,2]; xh=R[:,0]; yh=R[:,1]
    axy=a[:2]
    tcp=np.array([*(np.array([mug_x,mug_y_final])-0.0785*axy),1.014])
    # body corners: s in ±0.103, t in [-0.1034,-0.0374], u in ±0.032
    best={}
    door=-1; bez=-1; worst_in=None
    for s in (-0.103,0.103):
        for t in (-0.1034,-0.0374):
            for u in (-0.032,0.032):
                c=tcp+s*yh+t*a+u*xh
                door=max(door,c[0])  # want < -0.296 ... actually need c[0] > -0.296
    # -x corner (s=+0.103): min x
    cs=[tcp+s*yh+t*a+u*xh for s in (-0.103,0.103) for t in (-0.1034,-0.0374) for u in (-0.032,0.032)]
    cs=np.array(cs)
    minx=cs[:,0].min(); door_margin=minx-(-0.296)
    inside=cs[cs[:,1]>-0.32]  # points past/at the face plane
    bez_margin=(-0.06-inside[:,0].max()) if len(inside) else 9
    bez_marginL=(inside[:,0].min()-(-0.26)) if len(inside) else 9
    ztop=inside[:,2].max() if len(inside) else 0
    mug_margin_R=-0.057-(mug_x+0.05); mug_margin_L=(mug_x-0.05)-(-0.266)
    print(f"yaw {yaw_d} mugx {mug_x:.3f}: tcp {np.round(tcp,3)} door {door_margin*100:.1f}cm  bezel+x {bez_margin*100:.1f}cm bezel-x {bez_marginL*100:.1f}cm  ztop {ztop:.3f}  mug R {mug_margin_R*100:.1f} L {mug_margin_L*100:.1f}")
for yaw in (-8,-10,-12,-15):
    for mx in (-0.145,-0.15,-0.155,-0.16):
        margins(yaw,40,mx)
EOF

# openrua op 64
python3 - <<'EOF'
import numpy as np, sys
sys.path.insert(0,'/workspace/tools')
from task import R_of
FACE=-0.315; MARG=0.008
def analyze(yaw_d,pitch_d):
    R=R_of(np.deg2rad(yaw_d),np.deg2rad(pitch_d)); a=R[:,2]; xh=R[:,0]; yh=R[:,1]
    best=None
    for mug_x in np.arange(-0.20,-0.09,0.0025):
        for mug_y in np.arange(-0.30,-0.19,0.0025):
            tcp=np.array([*(np.array([mug_x,mug_y])-0.0785*a[:2]),1.014])
            cs=np.array([tcp+s*yh+t*a+u*xh for s in (-0.103,0.103) for t in (-0.1034,-0.0374) for u in (-0.032,0.032)])
            ok=True
            if cs[:,0].min() < -0.296+MARG: ok=False                      # door
            inside=cs[cs[:,1]>FACE-MARG]
            if len(inside):
                if inside[:,0].min()< -0.26+MARG or inside[:,0].max()> -0.06-MARG: ok=False
                if inside[:,2].max()>1.075-MARG: ok=False
            if mug_x+0.05 > -0.057-MARG or mug_x-0.05 < -0.266+MARG: ok=False
            if mug_y+0.05 > -0.144-MARG: ok=False
            if not ok: continue
            handle_end=tcp[1]-0.012*a[1]
            depth=handle_end-FACE
            if best is None or depth>best[0]: best=(depth,mug_x,mug_y,tcp,handle_end)
    if best: print(f"yaw {yaw_d} pitch {pitch_d}: handle-end depth {best[0]*100:.1f}cm at mug ({best[1]:.3f},{best[2]:.3f}) tcp {np.round(best[3],3)}")
    else: print(f"yaw {yaw_d} pitch {pitch_d}: none")
for yaw in (0,-5,-8,-10,-12,-15,-20):
    for pitch in (30,35,40,45):
        analyze(yaw,pitch)
EOF

# openrua op 65
(python3 tools/reach_probe.py -12 35 -0.14 1.014 -0.40 -0.50 2>&1 | tail -2; echo ---; python3 tools/reach_probe.py -12 40 -0.14 1.014 -0.40 -0.50 2>&1 | tail -2; echo ---; python3 tools/reach_probe.py -15 35 -0.14 1.014 -0.40 -0.50 2>&1 | tail -2)

# openrua op 66
python3 - <<'EOF'
import re
s=open('tools/task.py').read()
s=s.replace("YAW, PITCH = np.deg2rad(-10.0), np.deg2rad(40.0)","YAW, PITCH = np.deg2rad(-15.0), np.deg2rad(35.0)")
s=s.replace("MUG_ENTRY = np.array([-0.160, -0.390])","MUG_ENTRY = np.array([-0.122, -0.385])")
s=s.replace("MUG_FINAL = np.array([-0.160, -0.225])","MUG_FINAL = np.array([-0.122, -0.225])")
s=s.replace("""    if stage == "pregrasp":""","""    if stage == "ikcheck":
        for name, pos in (("PREGRASP", PREGRASP), ("HANDLE", HANDLE), ("PREINSERT", PREINSERT), ("INSERT", INSERT)):
            q = p.ik(pos, Q_G, attempts=10)
            print(name, np.round(pos, 3), None if q is None else np.round(q, 3).tolist(), flush=True)
    elif stage == "pregrasp":""")
open('tools/task.py','w').write(s)
EOF
python3 tools/task.py info && python3 tools/task.py ikcheck 2>&1 | grep -v "^ik:"

# openrua op 67
python3 -u tools/task.py pregrasp 2>&1 | grep -v "^ik:" | tail -8

# openrua op 68
cd /workspace/snaps && ls -t *.png | head; python3 /workspace/tools/cam_snap.py birdview agentview --depth 2>&1 | tail -2

# openrua op 69
python3 - <<'EOF'
import numpy as np, sys
sys.path.insert(0,'/workspace/tools')
import cams
d=np.load('agentview_depth.npy'); P=cams.cloud('agentview',d)
pts=P.reshape(-1,3)
m=(np.abs(pts[:,0])<0.03)&(pts[:,1]>-0.075)&(pts[:,1]<-0.015)&(pts[:,2]>0.905)
h=pts[m]
print("handle pts",len(h))
for z0 in np.arange(0.92,1.0,0.01):
    s=h[(h[:,2]>=z0)&(h[:,2]<z0+0.01)]
    if len(s): print(f"z {z0:.2f}: n={len(s):3d} y[{s[:,1].min():.3f},{s[:,1].max():.3f}] x[{s[:,0].min():.3f},{s[:,0].max():.3f}]")
# where is the camera relative to the mug (which side of the handle is visible)?
print("cam pos", cams.cam_pose('agentview')[0].round(3))
EOF

# openrua op 70
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/tools/task.py", "old_string": "    if stage == \\"ikcheck\\":", "new_string": "    if stage == \\"td_grasp\\":\\n        # top-down grasp of the handle bar: hand z down, fingers along x\\n        p.set_scene(microwave=\\"solid\\", yellow=False)\\n        p.grip(True)\\n        ok = p.goto_tcp(TD_GRASP + np.array([0, 0, 0.10]), Q_TD, time_s=10.0)\\n        print(\\"above ok\\", ok, flush=True)\\n        if ok:\\n            ok = p.line(TD_GRASP, quat=Q_TD, step=0.005, avoid=False, time_scale=2.0)\\n            print(\\"descend ok\\", ok, flush=True)\\n            p.grip(False)\\n            print(\\"gap after close\\", round(p.finger_gap(), 4), flush=True)\\n    elif stage == \\"drag\\":\\n        # drag the mug along the table (+y) so a frontal grasp has room behind it\\n        pos = p.tcp()[0]\\n        ok = p.line(pos + np.array([0, DRAG_DY, 0]), quat=Q_TD, step=0.005, avoid=False, time_scale=3.0)\\n        print(\\"drag ok\\", ok, \\"gap\\", round(p.finger_gap(), 4), flush=True)\\n        p.grip(True)\\n        pos = p.tcp()[0]\\n        ok = p.line(pos + np.array([0, 0, 0.10]), quat=Q_TD, step=0.005, avoid=False)\\n        print(\\"raise ok\\", ok, flush=True)\\n    elif stage == \\"ikcheck\\":", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 71
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/tools/task.py", "old_string": "HOME_Q = [-0.002, 0.244, 0.003, -1.725, -0.001, 1.961, 0.778]", "new_string": "HOME_Q = [-0.002, 0.244, 0.003, -1.725, -0.001, 1.961, 0.778]\\n\\n# top-down relocation grasp (the mug stands right behind the microwave; no room for a frontal grasp)\\nR_TD = np.array([[0, -1, 0],\\n                 [-1, 0, 0],\\n                 [0, 0, -1]], float)                  # hand x -> -y, hand y -> -x (fingers along x), hand z down\\nQ_TD = mat_to_quat(R_TD)\\nTD_GRASP = np.array([-0.001, -0.053, 0.970])          # pads on the handle bar (z 0.96-0.98)\\nDRAG_DY = 0.16                                        # slide the mug to y ~ 0.19", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 72
cd /workspace && python3 -u tools/task.py td_grasp 2>&1 | grep -v "^ik:" | tail -12

# openrua op 73
python3 - <<'EOF'
s=open('tools/planner.py').read()
s=s.replace("def execute(self, jt, time_scale=1.5, min_time=1.5):","def execute(self, jt, time_scale=3.0, min_time=3.0):")
s=s.replace("        th = pts[-1][0] + 1.5\n","        th = pts[-1][0] + 3.0\n")
s=s.replace("""        if err > 0.02:  # converge with a short resend of the final point
            self.move(qs[-1], 2.0)
            err = np.abs(self.arm_q() - np.array(qs[-1])).max()""","""        for _ in range(2):
            if err <= 0.02:
                break
            self.move(qs[-1], 3.0)  # converge with a resend of the final point
            err = np.abs(self.arm_q() - np.array(qs[-1])).max()""")
open('tools/planner.py','w').write(s)
s=open('tools/robot.py').read()
s=s.replace("        th = seconds + 1.5\n","        th = seconds + 3.0\n")
open('tools/robot.py','w').write(s)
EOF
grep -n "time_scale=" tools/planner.py tools/task.py | head

# openrua op 74
python3 - <<'EOF'
s=open('tools/task.py').read()
s=s.replace("""    elif stage == "drag":""","""    elif stage == "td_descend":
        ok = p.line(TD_GRASP, quat=Q_TD, step=0.005, avoid=False, time_scale=2.0)
        print("descend ok", ok, flush=True)
        p.grip(False)
        print("gap after close", round(p.finger_gap(), 4), flush=True)
    elif stage == "drag":""")
open('tools/task.py','w').write(s)
EOF
python3 -u tools/task.py td_descend 2>&1 | grep -v "^ik:" | tail -8

# openrua op 75
python3 -u tools/task.py drag 2>&1 | grep -v "^ik:" | tail -8 && cd snaps && python3 /workspace/tools/cam_snap.py agentview birdview --depth | tail -1

# openrua op 76
python3 - <<'EOF'
import numpy as np, sys
sys.path.insert(0,'/workspace/tools')
import cams
d=np.load('agentview_depth.npy'); P=cams.cloud('agentview',d).reshape(-1,3)
m=(P[:,2]>0.905)&(P[:,2]<1.02)&(P[:,1]>0.0)&(P[:,1]<0.30)&(P[:,0]>-0.15)&(P[:,0]<0.15)
q=P[m]
print("pts",len(q), "z max", q[:,2].max().round(3))
rim=q[q[:,2]>0.995]
print("rim pts",len(rim),"x[%.3f,%.3f] y[%.3f,%.3f]"%(rim[:,0].min(),rim[:,0].max(),rim[:,1].min(),rim[:,1].max()))
# fit circle to rim points (xy)
A=np.c_[2*rim[:,0],2*rim[:,1],np.ones(len(rim))]; b=(rim[:,0]**2+rim[:,1]**2)
c=np.linalg.lstsq(A,b,rcond=None)[0]; cx,cy=c[0],c[1]; r=np.sqrt(c[2]+cx**2+cy**2)
print("rim circle centre (%.4f,%.4f) r %.4f"%(cx,cy,r))
# handle: points at z 0.93-0.98 outside r+0.01 from the centre
body=q[(q[:,2]>0.925)&(q[:,2]<0.985)]
dist=np.hypot(body[:,0]-cx,body[:,1]-cy)
h=body[dist>0.055]
print("handle pts",len(h),"x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]"%(h[:,0].min(),h[:,0].max(),h[:,1].min(),h[:,1].max(),h[:,2].min(),h[:,2].max()))
print("handle centroid", h.mean(0).round(4), "dir from centre", np.arctan2(h[:,1].mean()-cy,h[:,0].mean()-cx)*180/np.pi)
EOF

# openrua op 77
python3 - <<'EOF'
import numpy as np, sys
sys.path.insert(0,'/workspace/tools')
import cams
d=np.load('agentview_depth.npy'); P=cams.cloud('agentview',d).reshape(-1,3)
m=(P[:,2]>0.905)&(P[:,2]<1.02)&(P[:,1]>0.08)&(P[:,1]<0.265)&(P[:,0]>-0.12)&(P[:,0]<0.12)
q=P[m]
rim=q[q[:,2]>0.998]
A=np.c_[2*rim[:,0],2*rim[:,1],np.ones(len(rim))]; b=(rim[:,0]**2+rim[:,1]**2)
c=np.linalg.lstsq(A,b,rcond=None)[0]; cx,cy=c[0],c[1]; r=np.sqrt(c[2]+cx**2+cy**2)
print("rim pts",len(rim),"circle centre (%.4f,%.4f) r %.4f  rim z max %.3f"%(cx,cy,r,q[:,2].max()))
for z0 in np.arange(0.92,1.0,0.01):
    s=q[(q[:,2]>=z0)&(q[:,2]<z0+0.01)&(q[:,1]<cy-0.045)]
    if len(s): print(f"z {z0:.2f}: n={len(s):3d} y[{s[:,1].min():.3f},{s[:,1].max():.3f}] x[{s[:,0].min():.3f},{s[:,0].max():.3f}]")
EOF

# openrua op 78
python3 - <<'EOF'
s=open('tools/task.py').read()
s=s.replace('HANDLE = np.array([-0.002, -0.050, 0.958])            # grip point on the handle (world)',
            'MUG_XY = np.array([-0.0006, 0.1851])                  # mug axis after relocation (agentview fit)\nHANDLE = np.array([MUG_XY[0], MUG_XY[1] - 0.0785, 0.958])   # grip point on the handle bar')
s=s.replace('from planner import Planner','from planner import Planner, cylinder')
s=s.replace("""    elif stage == "ikcheck":""","""    elif stage == "fgrasp":
        # frontal (pitched) handle grasp at the relocated mug, approached from above
        p.set_scene(microwave="solid", yellow=False)
        p.apply_scene([cylinder("yellow_mug", MUG_XY[0], MUG_XY[1], 0.055, 0.90, 1.006)])
        p.grip(True)
        ok = p.goto_tcp(HANDLE + np.array([0, 0, 0.08]), Q_G, time_s=10.0)
        print("above ok", ok, flush=True)
        if ok:
            p.apply_scene([], remove_ids=["yellow_mug"])
            ok = p.line(HANDLE, quat=Q_G, step=0.005, avoid=False, time_scale=2.0)
            print("descend ok", ok, flush=True)
            p.grip(False)
            print("gap after close", round(p.finger_gap(), 4), flush=True)
    elif stage == "ikcheck":""")
open('tools/task.py','w').write(s)
EOF
grep -n "def cylinder" -A3 tools/planner.py | head -5; python3 tools/task.py info | tail -5

# openrua op 79
cd /workspace && python3 - <<'EOF'
s=open('/workspace/tools/task.py').read()
s=s.replace('HANDLE = np.array([-0.002, -0.050, 0.958])            # grip point on the handle (world)',
            'MUG_XY = np.array([-0.0006, 0.1851])                  # mug axis after relocation (agentview fit)\nHANDLE = np.array([MUG_XY[0], MUG_XY[1] - 0.0785, 0.958])   # grip point on the handle bar')
s=s.replace('from planner import Planner','from planner import Planner, cylinder')
s=s.replace("""    elif stage == "ikcheck":""","""    elif stage == "fgrasp":
        # frontal (pitched) handle grasp at the relocated mug, approached from above
        p.set_scene(microwave="solid", yellow=False)
        p.apply_scene([cylinder("yellow_mug", MUG_XY[0], MUG_XY[1], 0.055, 0.90, 1.006)])
        p.grip(True)
        ok = p.goto_tcp(HANDLE + np.array([0, 0, 0.08]), Q_G, time_s=10.0)
        print("above ok", ok, flush=True)
        if ok:
            p.apply_scene([], remove_ids=["yellow_mug"])
            ok = p.line(HANDLE, quat=Q_G, step=0.005, avoid=False, time_scale=2.0)
            print("descend ok", ok, flush=True)
            p.grip(False)
            print("gap after close", round(p.finger_gap(), 4), flush=True)
    elif stage == "ikcheck":""")
open('/workspace/tools/task.py','w').write(s)
EOF
grep -n "def cylinder" -A3 /workspace/tools/planner.py | head -5; python3 /workspace/tools/task.py info | tail -5

# openrua op 80
python3 -u tools/task.py fgrasp 2>&1 | grep -v "^ik:" | tail -10

# openrua op 81
cd /workspace/snaps && python3 /workspace/tools/cam_snap.py agentview robot0_eye_in_hand | tail -1

# openrua op 82
cd /workspace && python3 - <<'EOF'
s=open('/workspace/tools/task.py').read()
s=s.replace("""    elif stage == "ikcheck":""","""    elif stage == "fgrasp2":
        # re-do the descent slowly (controller lag deflected the first attempt)
        p.grip(True)
        pos = p.tcp()[0]
        p.line(pos + np.array([0, 0, 0.05]), quat=Q_G, step=0.005, avoid=False, time_scale=3.0)
        p.line(HANDLE + np.array([0, 0, 0.05]), quat=Q_G, step=0.005, avoid=False, time_scale=3.0)
        ok = p.line(HANDLE, quat=Q_G, step=0.0025, avoid=False, time_scale=6.0)
        print("descend ok", ok, flush=True)
        p.grip(False)
        print("gap after close", round(p.finger_gap(), 4), flush=True)
    elif stage == "ikcheck":""")
open('/workspace/tools/task.py','w').write(s)
EOF
python3 -u tools/task.py fgrasp2 2>&1 | grep -v "^ik:" | tail -12

# openrua op 83
python3 - <<'EOF'
s=open('/workspace/tools/task.py').read()
s=s.replace("""    elif stage == "ikcheck":""","""    elif stage == "backoff":
        p.grip(True)
        pos = p.tcp()[0]
        p.line(pos + np.array([0, 0, 0.10]), quat=Q_G, step=0.005, avoid=False, time_scale=3.0)
    elif stage == "ikcheck":""")
open('/workspace/tools/task.py','w').write(s)
EOF
python3 -u tools/task.py backoff 2>&1 | grep -v "^ik:" | tail -3; cd snaps && python3 /workspace/tools/cam_snap.py agentview birdview --depth | tail -1

# openrua op 84
python3 - <<'EOF'
import numpy as np, sys
sys.path.insert(0,'/workspace/tools')
import cams
d=np.load('agentview_depth.npy'); P=cams.cloud('agentview',d).reshape(-1,3)
m=(P[:,2]>0.905)&(P[:,2]<1.02)&(P[:,1]>0.05)&(P[:,1]<0.265)&(P[:,0]>-0.12)&(P[:,0]<0.12)
q=P[m]
rim=q[q[:,2]>0.998]
A=np.c_[2*rim[:,0],2*rim[:,1],np.ones(len(rim))]; b=(rim[:,0]**2+rim[:,1]**2)
c=np.linalg.lstsq(A,b,rcond=None)[0]; cx,cy=c[0],c[1]; r=np.sqrt(c[2]+cx**2+cy**2)
print("rim pts",len(rim),"circle centre (%.4f,%.4f) r %.4f  z max %.3f"%(cx,cy,r,q[:,2].max()))
for z0 in np.arange(0.91,1.0,0.01):
    s=q[(q[:,2]>=z0)&(q[:,2]<z0+0.01)&(q[:,1]<cy-0.045)]
    if len(s): print(f"z {z0:.2f}: n={len(s):3d} y[{s[:,1].min():.3f},{s[:,1].max():.3f}] x[{s[:,0].min():.3f},{s[:,0].max():.3f}]")
EOF

# openrua op 85
python3 - <<'EOF'
s=open('/workspace/tools/task.py').read()
s=s.replace("""def attach_mug(p):""","""def grasp_from_measure(centre, bar):
    \"\"\"Handle grasp aligned with the measured handle: returns (tcp, R, quat).\"\"\"
    centre, bar = np.asarray(centre, float), np.asarray(bar, float)
    d = centre - bar; d /= np.linalg.norm(d)                 # horizontal approach direction (hand -> mug)
    yaw = np.arctan2(-d[0], d[1])
    R = R_of(yaw, PITCH)
    tcp = np.array([*(centre - GRIP_TO_CENTRE * d), HANDLE[2]])
    print(f"measured grasp: yaw {np.rad2deg(yaw):.1f} deg, tcp {np.round(tcp, 4)}", flush=True)
    return tcp, R, mat_to_quat(R)


MEAS_CENTRE = (-0.0023, 0.1788)                       # latest agentview fit
MEAS_BAR = (0.0185, 0.098)


def attach_mug(p):""")
s=s.replace("""    elif stage == "backoff":""","""    elif stage == "fgrasp3":
        tcp, R, q = grasp_from_measure(MEAS_CENTRE, MEAS_BAR)
        p.grip(True)
        p.set_scene(microwave="solid", yellow=False)
        ok = p.goto_tcp(tcp + np.array([0, 0, 0.06]), q, time_s=10.0)
        print("above ok", ok, flush=True)
        if ok:
            ok = p.line(tcp, quat=q, step=0.0025, avoid=False, time_scale=6.0)
            print("descend ok", ok, flush=True)
            p.grip(False)
            print("gap after close", round(p.finger_gap(), 4), flush=True)
    elif stage == "backoff":""")
open('/workspace/tools/task.py','w').write(s)
EOF
cd /workspace && python3 -u tools/task.py fgrasp3 2>&1 | grep -v "^ik:" | tail -10

# openrua op 86
cd /workspace/snaps && python3 /workspace/tools/cam_snap.py robot0_eye_in_hand agentview --depth 2>&1 | tail -3

# openrua op 87
cd /workspace && python3 - <<'EOF' 2>&1 | grep -v "^ik:"
import sys, numpy as np, rclpy
sys.path.insert(0,"/workspace/tools")
from robot import Robot, LIMITS
from task import grasp_from_measure, MEAS_CENTRE, MEAS_BAR
rclpy.init(); r=Robot()
tcp,R,qq=grasp_from_measure(MEAS_CENTRE, MEAS_BAR)
q=r.arm_q(); print("q now", np.round(q,3).tolist())
for name,t in [("target",tcp),("now-pose",[0.0308,0.1106,0.9843])]:
    s=r.ik(t,qq,seed=q,attempts=6)
    print(name, None if s is None else np.round(s,3).tolist(), None if s is None else "diff "+str(np.round(s-q,3).tolist()))
print("limit margins", [round(min(v-lo,hi-v),2) for v,(lo,hi) in zip(q,LIMITS)])
print("fingers", {k:round(v,4) for k,v in r.joints().items() if "finger" in k})
EOF

# openrua op 88
python3 - <<'EOF'
s=open('/workspace/tools/task.py').read()
s=s.replace("""def grasp_from_measure(centre, bar):""","""def servo_tcp(p, target, quat, iters=4, tol=0.004, seconds=3.0):
    \"\"\"Closed-loop TCP positioning: re-command target + measured error until |err| < tol.
    Compensates the steady-state sag of the position controller at stretched poses.\"\"\"
    target = np.asarray(target, float)
    cmd = target.copy()
    for i in range(iters):
        cur, _, _ = p.tcp()
        err = target - cur
        print(f"servo {i}: tcp {np.round(cur, 4)} err {np.round(err, 4)} |err| {np.linalg.norm(err):.4f}", flush=True)
        if np.linalg.norm(err) < tol:
            return True
        cmd = cmd + err
        q = p.ik(cmd, quat, seed=p.arm_q(), attempts=6)
        if q is None:
            print("servo: ik failed for", np.round(cmd, 4), flush=True)
            return False
        p.move(q, seconds)
    cur, _, _ = p.tcp()
    print(f"servo end: tcp {np.round(cur, 4)} err {np.round(target - cur, 4)}", flush=True)
    return np.linalg.norm(target - cur) < tol


def grasp_from_measure(centre, bar):""")
s=s.replace("""    elif stage == "backoff":""","""    elif stage == "servo_test":
        tcp, R, q = grasp_from_measure(MEAS_CENTRE, MEAS_BAR)
        p.grip(True)
        ok = servo_tcp(p, tcp, q)
        print("servo ok", ok, flush=True)
        if ok:
            p.grip(False)
            print("gap after close", round(p.finger_gap(), 4), flush=True)
    elif stage == "backoff":""")
open('/workspace/tools/task.py','w').write(s)
EOF
python3 -u tools/task.py servo_test 2>&1 | grep -v "^ik:" | tail -14

# openrua op 89
cd /workspace/snaps && python3 /workspace/tools/cam_snap.py robot0_eye_in_hand agentview birdview --depth 2>&1 | tail -1

# openrua op 90
cd /workspace/tools && grep -n "^def \|^    def \|^[A-Z_]* =" eih.py cams.py | head -40

# openrua op 91
cd /workspace && python3 - <<'EOF' 2>&1 | tail -30
import sys, numpy as np, rclpy
sys.path.insert(0,"/workspace/tools")
from robot import Robot, TCP
import eih
rclpy.init(); r=Robot()
pos,quat,R=r.fk()
d=np.load("/workspace/snaps/robot0_eye_in_hand_depth.npy")
H,W=d.shape
u,v=np.meshgrid(np.arange(W),np.arange(H))
ph=eih.px_to_hand(u.ravel(),v.ravel(),d.ravel())   # (N,3) in hand frame?
ph=np.asarray(ph).reshape(-1,3) if np.asarray(ph).shape[0]!=3 else np.asarray(ph).T
pw=(R@ph.T).T+pos
# points near the fingertip plane: hand z in [0.06,0.12], |hand y|<0.06, |hand x|<0.05
m=(ph[:,2]>0.05)&(ph[:,2]<0.13)&(np.abs(ph[:,1])<0.07)&(np.abs(ph[:,0])<0.06)
print("near pts",m.sum())
sel=ph[m]; selw=pw[m]
# histogram by hand y (finger closing axis) and hand z
for ylo in np.arange(-0.06,0.06,0.01):
    mm=(sel[:,1]>=ylo)&(sel[:,1]<ylo+0.01)
    if mm.sum(): print(f"hand y [{ylo:+.2f},{ylo+0.01:+.2f}] n={mm.sum():4d} hand z {sel[mm,2].min():.3f}-{sel[mm,2].max():.3f} hand x {sel[mm,0].min():+.3f}..{sel[mm,0].max():+.3f} world z {selw[mm,2].min():.3f}-{selw[mm,2].max():.3f} world x {selw[mm,0].min():+.3f}..{selw[mm,0].max():+.3f} y {selw[mm,1].min():+.3f}..{selw[mm,1].max():+.3f}")
print("tcp world", np.round(pos+R@[0,0,TCP],4))
EOF

# openrua op 92
python3 - <<'EOF' 2>&1 | tail -30
import sys, numpy as np, rclpy
sys.path.insert(0,"/workspace/tools")
from robot import Robot, TCP
import eih
rclpy.init(); r=Robot()
pos,quat,R=r.fk()
d=np.load("/workspace/snaps/robot0_eye_in_hand_depth.npy")
H,W=d.shape
u,v=np.meshgrid(np.arange(W),np.arange(H))
X=(u.ravel()-320)/eih.F*d.ravel(); Y=(v.ravel()-240)/eih.F*d.ravel()
ph=np.stack([-Y,X,d.ravel()],1)+eih.CAM_IN_HAND
pw=(R@ph.T).T+pos
m=(ph[:,2]>0.05)&(ph[:,2]<0.13)&(np.abs(ph[:,1])<0.07)&(np.abs(ph[:,0])<0.06)
print("near pts",m.sum())
sel=ph[m]; selw=pw[m]
for ylo in np.arange(-0.06,0.06,0.01):
    mm=(sel[:,1]>=ylo)&(sel[:,1]<ylo+0.01)
    if mm.sum(): print(f"hand y [{ylo:+.2f},{ylo+0.01:+.2f}] n={mm.sum():4d} hand z {sel[mm,2].min():.3f}-{sel[mm,2].max():.3f} hand x {sel[mm,0].min():+.3f}..{sel[mm,0].max():+.3f} world z {selw[mm,2].min():.3f}-{selw[mm,2].max():.3f} world x {selw[mm,0].min():+.3f}..{selw[mm,0].max():+.3f} y {selw[mm,1].min():+.3f}..{selw[mm,1].max():+.3f}")
print("tcp world", np.round(pos+R@[0,0,TCP],4))
EOF

# openrua op 93
python3 - <<'EOF' 2>&1 | tail -12
import sys, numpy as np, rclpy
sys.path.insert(0,"/workspace/tools")
from robot import Robot
rclpy.init(); r=Robot()
q=r.arm_q()
for L in ["panda_link1","panda_link2","panda_link3","panda_link4","panda_link5","panda_link6","panda_link7","panda_link8","panda_hand","panda_leftfinger","panda_rightfinger"]:
    p,_,_=r.fk(q,L); print(L, np.round(p,3))
EOF

# openrua op 94
python3 -u tools/task.py backoff 2>&1 | grep -v "^ik:" | tail -4 && cd /workspace/snaps && python3 /workspace/tools/cam_snap.py agentview birdview --depth 2>&1 | tail -1

# openrua op 95
cat > /workspace/tools/measure_mug.py <<'EOF'
#!/usr/bin/env python3
"""Fit the yellow mug rim circle and handle bar from the agentview depth cloud.
usage: measure_mug.py [x_lo x_hi y_lo y_hi]  (search window, world). Prints CENTRE/BAR."""
import sys, numpy as np
sys.path.insert(0, "/workspace/tools")
import cams

def fit_circle(P):
    x, y = P[:, 0], P[:, 1]
    A = np.stack([x, y, np.ones_like(x)], 1); b = x * x + y * y
    c = np.linalg.lstsq(A, b, rcond=None)[0]
    cx, cy = c[0] / 2, c[1] / 2
    return cx, cy, np.sqrt(c[2] + cx * cx + cy * cy)

def main():
    win = list(map(float, sys.argv[1:5])) if len(sys.argv) >= 5 else [-0.12, 0.12, 0.0, 0.30]
    d = np.load("/workspace/snaps/agentview_depth.npy")
    P = cams.cloud("agentview", d).reshape(-1, 3)
    m = (P[:, 0] > win[0]) & (P[:, 0] < win[1]) & (P[:, 1] > win[2]) & (P[:, 1] < win[3]) & (P[:, 2] > 0.905) & (P[:, 2] < 1.02)
    P = P[m]
    rim = P[P[:, 2] > 0.995]
    cx, cy, r = fit_circle(rim[:, :2])
    print(f"rim pts {len(rim)} circle centre ({cx:.4f},{cy:.4f}) r {r:.4f} z max {P[:,2].max():.3f}")
    # handle = points outside the body cylinder (r>0.06 from the centre), z 0.93-0.975
    dist = np.hypot(P[:, 0] - cx, P[:, 1] - cy)
    H = P[(dist > 0.062) & (dist < 0.11) & (P[:, 2] > 0.93) & (P[:, 2] < 0.975)]
    if len(H):
        ang = np.degrees(np.arctan2(H[:, 1] - cy, H[:, 0] - cx))
        print(f"handle pts {len(H)} x[{H[:,0].min():.3f},{H[:,0].max():.3f}] y[{H[:,1].min():.3f},{H[:,1].max():.3f}] "
              f"dist[{dist[(dist>0.062)&(dist<0.11)&(P[:,2]>0.93)&(P[:,2]<0.975)].min():.3f},...] angle median {np.median(ang):.1f} deg")
        bx, by = np.median(H[:, 0]), np.median(H[:, 1])
        print(f"BAR ({bx:.4f},{by:.4f})  CENTRE ({cx:.4f},{cy:.4f})")
    else:
        print("no handle points found")

if __name__ == "__main__":
    main()
EOF
cd /workspace && python3 tools/measure_mug.py

# openrua op 96
python3 tools/measure_mug.py -0.08 0.08 0.05 0.26

# openrua op 97
cd /workspace/tools && grep -n "def attach_mug" -A 12 task.py && grep -n 'stage == "td_grasp"' -A 22 task.py && grep -n "def attach_mug\|def detach_mug\|def apply_scene\|def set_scene\|def goto_tcp\|def line\|def execute\|def plan_joint\|def cartesian" -A 3 planner.py | head -60

# openrua op 98
sed -n 100,125p task.py

# openrua op 99
python3 - <<'EOF'
s=open('/workspace/tools/task.py').read()
s=s.replace("""def attach_mug(p):
    \"\"\"Attach the held mug as a vertical cylinder in the hand frame.\"\"\"""","""def attach_mug(p, R=None, c_world_rel=None):
    \"\"\"Attach the held mug as a vertical cylinder in the hand frame.
    R: hand rotation at grasp; c_world_rel: mug centre relative to the TCP (world axes).\"\"\"
    if R is None:
        R = R_G
    if c_world_rel is None:
        c_world_rel = np.array([*(GRIP_TO_CENTRE * A_XY), 0.9525 - HANDLE[2]])""")
s=s.replace("""    c_world_rel = np.array([*(GRIP_TO_CENTRE * A_XY), 0.9525 - HANDLE[2]])   # mug centre rel. TCP
    c_hand = R_G.T @ c_world_rel + np.array([0, 0, TCP])""","""    c_hand = R.T @ np.asarray(c_world_rel, float) + np.array([0, 0, TCP])""")
s=s.replace("""    zh = R_G.T @ np.array([0, 0, 1.0])                # world z in hand frame = cylinder axis""","""    zh = R.T @ np.array([0, 0, 1.0])                  # world z in hand frame = cylinder axis""")
# relocation constants + stage
s=s.replace("""MEAS_CENTRE = (-0.0023, 0.1788)                       # latest agentview fit
MEAS_BAR = (0.0185, 0.098)""","""MEAS_CENTRE = (-0.0003, 0.1749)                       # latest agentview fit
MEAS_BAR = (0.028, 0.101)                             # bar centre (visible +x face minus ~7 mm)
RELOC_CENTRE = np.array([-0.03, 0.24])                # where to set the mug down for the pitched grasp
TD_Z = 0.965                                          # TCP height for the top-down handle grasp


def Rz(t):
    c, s_ = np.cos(t), np.sin(t)
    return np.array([[c, -s_, 0], [s_, c, 0], [0, 0, 1.0]])


def td_frames(centre, bar):
    \"\"\"Top-down grasp of the handle bar with the finger axis normal to the handle plane,
    plus the place pose that leaves the handle pointing along -A_XY (insertion approach).\"\"\"
    centre, bar = np.asarray(centre, float), np.asarray(bar, float)
    d_g = bar - centre; d_g /= np.linalg.norm(d_g)               # handle direction now
    d_p = -A_XY / np.linalg.norm(A_XY)                             # handle direction wanted
    ang_g = np.arctan2(d_g[1], d_g[0]); ang_p = np.arctan2(d_p[1], d_p[0])
    R_grasp = Rz(ang_g + np.pi / 2) @ R_TD                          # R_TD closes along x for a handle along -y
    R_place = Rz(ang_p + np.pi / 2) @ R_TD
    tcp_g = np.array([*(centre + GRIP_TO_CENTRE * d_g), TD_Z])
    tcp_p = np.array([*(RELOC_CENTRE + GRIP_TO_CENTRE * d_p), TD_Z + 0.004])
    print(f"td grasp: handle dir {np.round(d_g,3)} -> {np.round(d_p,3)}, rotate {np.degrees(ang_p-ang_g):.1f} deg; "
          f"tcp_g {np.round(tcp_g,4)} tcp_p {np.round(tcp_p,4)}", flush=True)
    return tcp_g, R_grasp, tcp_p, R_place, d_g""")
s=s.replace("""    elif stage == "servo_test":""","""    elif stage == "relocate":
        tcp_g, R_g, tcp_p, R_p, d_g = td_frames(MEAS_CENTRE, MEAS_BAR)
        q_g, q_p = mat_to_quat(R_g), mat_to_quat(R_p)
        p.set_scene(microwave="solid", yellow=False)
        p.grip(True)
        ok = p.goto_tcp(tcp_g + np.array([0, 0, 0.08]), q_g, time_s=10.0)
        print("above ok", ok, flush=True)
        if not ok: return
        ok = p.line(tcp_g, quat=q_g, step=0.004, avoid=False, time_scale=3.0)
        print("descend ok", ok, flush=True)
        p.grip(False)
        gap = p.finger_gap(); print("gap after close", round(gap, 4), flush=True)
        if not (0.010 < gap < 0.025):
            print("unexpected gap, stopping", flush=True); return
        c_rel = np.array([*(GRIP_TO_CENTRE * d_g), 0.9525 - TD_Z])
        attach_mug(p, R=R_g, c_world_rel=c_rel)
        ok = p.line(tcp_g + np.array([0, 0, 0.04]), quat=q_g, step=0.004, avoid=False, time_scale=3.0)
        print("lift ok", ok, "gap", round(p.finger_gap(), 4), flush=True)
        ok = p.goto_tcp(tcp_p + np.array([0, 0, 0.04]), q_p, time_s=10.0)
        print("carry ok", ok, "gap", round(p.finger_gap(), 4), flush=True)
        if not ok: return
        ok = p.line(tcp_p, quat=q_p, step=0.004, avoid=False, time_scale=3.0)
        print("lower ok", ok, flush=True)
        p.grip(True)
        p.detach_mug()
        ok = p.line(tcp_p + np.array([0, 0, 0.10]), quat=q_p, step=0.005, avoid=False, time_scale=2.0)
        print("raise ok", ok, flush=True)
    elif stage == "servo_test":""")
open('/workspace/tools/task.py','w').write(s)
EOF
cd /workspace && python3 -u tools/task.py relocate 2>&1 | grep -v "^ik:" | grep -v "^traj done" | tail -16

# openrua op 100
cd /workspace/snaps && python3 /workspace/tools/cam_snap.py agentview birdview --depth 2>&1 | tail -1 && python3 tools/measure_mug.py -0.12 0.06 0.10 0.30

# openrua op 101
python3 /workspace/tools/measure_mug.py -0.12 0.06 0.10 0.30

# openrua op 102
python3 /workspace/tools/measure_mug.py -0.12 0.05 0.12 0.285

# openrua op 103
sed -i 's/^MEAS_CENTRE = .*/MEAS_CENTRE = (-0.0308, 0.2342)                       # latest agentview fit/; s/^MEAS_BAR = .*/MEAS_BAR = (-0.047, 0.157)                            # bar centre from centre + 0.0785*dir/' tools/task.py && python3 -u tools/task.py fgrasp3 2>&1 | grep -v "^ik:" | grep -v "^traj done" | tail -8

# openrua op 104
python3 - <<'EOF'
s=open('/workspace/tools/task.py').read()
s=s.replace("""    elif stage == "servo_test":""","""    elif stage == "lift2":
        tcp, R, q = grasp_from_measure(MEAS_CENTRE, MEAS_BAR)
        d = R[:2, 2] / np.linalg.norm(R[:2, 2])
        c_rel = np.array([*(GRIP_TO_CENTRE * d), 0.9525 - HANDLE[2]])
        attach_mug(p, R=R, c_world_rel=c_rel)
        pos = p.tcp()[0]
        ok = p.line(pos + np.array([0, 0, LIFT_DZ]), quat=q, step=0.004, avoid=False, time_scale=3.0)
        print("lift ok", ok, "gap", round(p.finger_gap(), 4), flush=True)
    elif stage == "servo_test":""")
open('/workspace/tools/task.py','w').write(s)
EOF
python3 -u tools/task.py lift2 2>&1 | grep -v "^ik:" | grep -v "^traj done" | tail -4 && cd /workspace/snaps && python3 /workspace/tools/cam_snap.py agentview --depth 2>&1 | tail -1

# openrua op 105
cd /workspace && python3 - <<'EOF'
import sys, numpy as np
sys.path.insert(0,"/workspace/tools"); import cams
d=np.load("/workspace/snaps/agentview_depth.npy"); P=cams.cloud("agentview",d).reshape(-1,3)
m=(P[:,0]>-0.12)&(P[:,0]<0.05)&(P[:,1]>0.12)&(P[:,1]<0.29)&(P[:,2]>0.905)&(P[:,2]<1.09)
Q=P[m]
print("points in mug region", len(Q), "z range", Q[:,2].min().round(3), Q[:,2].max().round(3))
for zlo in np.arange(0.90,1.09,0.02):
    mm=(Q[:,2]>=zlo)&(Q[:,2]<zlo+0.02)
    if mm.sum()>20: print(f"z[{zlo:.2f},{zlo+0.02:.2f}] n={mm.sum():4d} x[{Q[mm,0].min():+.3f},{Q[mm,0].max():+.3f}] y[{Q[mm,1].min():+.3f},{Q[mm,1].max():+.3f}]")
EOF

# openrua op 106
grep -n 'stage == "preinsert"' -A 8 tools/task.py && python3 -u tools/task.py preinsert 2>&1 | grep -v "^ik:" | grep -v "^traj done" | tail -6

# openrua op 107
cd /workspace/snaps && python3 /workspace/tools/cam_snap.py agentview birdview --depth 2>&1 | tail -1

# openrua op 108
cd /workspace && python3 - <<'EOF'
import sys, numpy as np
sys.path.insert(0,"/workspace/tools"); import cams
from measure_mug import fit_circle
d=np.load("/workspace/snaps/agentview_depth.npy"); P=cams.cloud("agentview",d).reshape(-1,3)
m=(P[:,0]>-0.29)&(P[:,0]<0.0)&(P[:,1]>-0.60)&(P[:,1]<-0.33)&(P[:,2]>0.905)&(P[:,2]<1.03)
Q=P[m]; print("pts",len(Q))
for zlo in np.arange(0.90,1.03,0.02):
    mm=(Q[:,2]>=zlo)&(Q[:,2]<zlo+0.02)
    if mm.sum()>10: print(f"z[{zlo:.2f},{zlo+0.02:.2f}] n={mm.sum():4d} x[{Q[mm,0].min():+.3f},{Q[mm,0].max():+.3f}] y[{Q[mm,1].min():+.3f},{Q[mm,1].max():+.3f}]")
rim=Q[Q[:,2]>0.995]
if len(rim)>30:
    cx,cy,r=fit_circle(rim[:,:2]); print(f"rim fit centre ({cx:.4f},{cy:.4f}) r {r:.4f} n {len(rim)}")
EOF

# openrua op 109
python3 - <<'EOF' 2>&1 | grep -v "^ik:\|^traj done" | tail -3
import sys, numpy as np, rclpy
sys.path.insert(0,"/workspace/tools")
from planner import Planner
from task import Q_G
rclpy.init(); p=Planner()
p.detach_mug()
pos=p.tcp()[0]
print("raise", p.line(pos+np.array([0.05,0,0.15]), quat=Q_G, step=0.01, avoid=False, time_scale=2.0))
p.destroy_node(); rclpy.shutdown()
EOF
cd /workspace/snaps && python3 /workspace/tools/cam_snap.py agentview birdview --depth 2>&1 | tail -1

# openrua op 110
python3 - <<'EOF' 2>&1 | grep -v "^ik:\|^traj done" | tail -3
import sys, numpy as np, rclpy
sys.path.insert(0,"/workspace/tools")
from planner import Planner
from task import Q_G, HOME_Q
rclpy.init(); p=Planner()
p.set_scene(microwave="solid", yellow=False)
print("park", p.goto_q(HOME_Q, time_s=10.0))
p.destroy_node(); rclpy.shutdown()
EOF
python3 /workspace/tools/cam_snap.py agentview birdview --depth 2>&1 | tail -1

# openrua op 111
cd /workspace && python3 - <<'EOF'
import sys, numpy as np
sys.path.insert(0,"/workspace/tools"); import cams
from measure_mug import fit_circle
d=np.load("/workspace/snaps/agentview_depth.npy"); P=cams.cloud("agentview",d).reshape(-1,3)
m=(P[:,0]>-0.29)&(P[:,0]<0.0)&(P[:,1]>-0.60)&(P[:,1]<-0.33)&(P[:,2]>0.905)&(P[:,2]<1.10)
Q=P[m]; print("pts",len(Q))
for zlo in np.arange(0.90,1.10,0.01):
    mm=(Q[:,2]>=zlo)&(Q[:,2]<zlo+0.01)
    if mm.sum()>10: print(f"z[{zlo:.2f},{zlo+0.01:.2f}] n={mm.sum():4d} x[{Q[mm,0].min():+.3f},{Q[mm,0].max():+.3f}] y[{Q[mm,1].min():+.3f},{Q[mm,1].max():+.3f}]")
EOF

# openrua op 112
python3 - <<'EOF'
import sys, numpy as np
sys.path.insert(0,"/workspace/tools"); import cams
d=np.load("/workspace/snaps/birdview_depth.npy"); P=cams.cloud("birdview",d).reshape(-1,3)
m=(P[:,0]>-0.26)&(P[:,0]<-0.04)&(P[:,1]>-0.56)&(P[:,1]<-0.33)&(P[:,2]>0.905)
Q=P[m]; print("pts",len(Q), "zmax", Q[:,2].max().round(3))
# height map on a 1cm grid
xs=np.arange(-0.26,-0.04,0.01); ys=np.arange(-0.56,-0.33,0.01)
print("      y:"+"".join(f"{y:6.2f}" for y in ys))
for x in xs:
    row=[]
    for y in ys:
        mm=(Q[:,0]>=x)&(Q[:,0]<x+0.01)&(Q[:,1]>=y)&(Q[:,1]<y+0.01)
        row.append(f"{Q[mm,2].max():6.3f}" if mm.any() else "     .")
    print(f"x {x:5.2f}:"+"".join(row))
EOF

# openrua op 113
cd /workspace/tools && grep -n "def goto_q" -A 8 planner.py && grep -n "def box\|def cylinder\|def scene_objects\|ALL_IDS" planner.py | head && grep -n "def scene_objects" -A 25 planner.py | head -40

# openrua op 114
python3 - <<'EOF'
s=open('robot.py').read()
s=s.replace("def ik(self, pos, quat, seed=None, at_tcp=True, attempts=5):","def ik(self, pos, quat, seed=None, at_tcp=True, attempts=5, avoid=False):")
s=s.replace("req.ik_request.avoid_collisions = False","req.ik_request.avoid_collisions = bool(avoid)")
open('robot.py','w').write(s)
EOF
cat > /workspace/tools/right_mug.py <<'EOF'
#!/usr/bin/env python3
"""Recover the yellow mug lying on its side in front of the microwave.
Stages: pinch (rim-wall pinch, one finger inside the mouth), upright (rotate & set down).
usage: right_mug.py pinch|upright|info"""
import sys
import numpy as np
import rclpy
sys.path.insert(0, "/workspace/tools")
from robot import mat_to_quat, TCP
from planner import Planner, box, ALL_IDS, scene_objects
from task import attach_mug, Rz

# measured lying pose (birdview/agentview depth)
AX_X, AX_Z = -0.168, 0.952          # mug axis (along world y) at the rim end
RIM_Y = -0.437                      # rim plane
MUG_LEN, RIM_R = 0.105, 0.0485
PITCH = np.deg2rad(50.0)
ANG = np.deg2rad(25.0)              # pinch point angle above the +x side of the rim
INSIDE = 0.016                      # pad centre this far inside the mouth (along y)


def grasp_frame():
    a = np.array([0.0, np.cos(PITCH), -np.sin(PITCH)])                     # approach: +y, pitched down
    yh = np.array([np.cos(ANG), np.sin(ANG) * np.sin(PITCH), np.sin(ANG) * np.cos(PITCH)])  # closing dir (radial-ish)
    yh -= yh.dot(a) * a; yh /= np.linalg.norm(yh)
    xh = np.cross(yh, a)
    R = np.stack([xh, yh, a], 1)
    assert np.linalg.det(R) > 0.99
    rim_pt = np.array([AX_X + (RIM_R - 0.002) * np.cos(ANG), RIM_Y, AX_Z + (RIM_R - 0.002) * np.sin(ANG)])
    tcp = rim_pt + np.array([0, INSIDE, 0])
    # mug centre relative to the TCP (world axes) and cylinder axis (world y)
    c_rel = np.array([AX_X, RIM_Y + MUG_LEN / 2, AX_Z]) - tcp
    return tcp, R, a, c_rel


def attach_lying(p, R, c_rel):
    """Attach the lying mug (axis along world y) to the hand."""
    from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
    from shape_msgs.msg import SolidPrimitive
    from geometry_msgs.msg import Pose
    aco = AttachedCollisionObject(); aco.link_name = "panda_hand"
    aco.touch_links = ["panda_hand", "panda_leftfinger", "panda_rightfinger"]
    co = CollisionObject(); co.header.frame_id = "panda_hand"; co.id = "held_mug"
    prim = SolidPrimitive(); prim.type = SolidPrimitive.CYLINDER; prim.dimensions = [MUG_LEN, 0.05]
    pose = Pose()
    c_hand = R.T @ c_rel + np.array([0, 0, TCP])
    pose.position.x, pose.position.y, pose.position.z = map(float, c_hand)
    ax = R.T @ np.array([0, 1.0, 0])                     # world y in hand frame = cylinder axis
    v = np.cross([0, 0, 1.0], ax); s = np.linalg.norm(v); c = np.dot([0, 0, 1.0], ax)
    axn = v / s; ang = np.arctan2(s, c)
    q = [*(axn * np.sin(ang / 2)), np.cos(ang / 2)]
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = map(float, q)
    co.primitives.append(prim); co.primitive_poses.append(pose); co.operation = CollisionObject.ADD
    aco.object = co
    return p.apply_scene([], attached=[aco])


def main():
    stage = sys.argv[1]
    rclpy.init(); p = Planner()
    tcp, R, a, c_rel = grasp_frame()
    q = mat_to_quat(R)
    print("pinch tcp", np.round(tcp, 4), "a", np.round(a, 3), "closing", np.round(R[:, 1], 3), flush=True)
    if stage == "info":
        s = p.ik(tcp, q, attempts=8); print("ik pinch", None if s is None else np.round(s, 3).tolist())
        s = p.ik(tcp - 0.05 * a, q, attempts=8); print("ik pre", None if s is None else np.round(s, 3).tolist())
    elif stage == "pinch":
        objs = scene_objects(microwave="solid", yellow=False)
        objs.append(box("yellow_mug", [-0.225, -0.435, 0.90], [-0.07, -0.33, 1.006]))
        p.apply_scene(objs, remove_ids=ALL_IDS)
        p.grip(True)
        ok = p.goto_tcp(tcp - 0.05 * a, q, time_s=10.0)
        print("pre ok", ok, flush=True)
        if not ok: return
        p.apply_scene([], remove_ids=["yellow_mug"])
        ok = p.line(tcp, quat=q, step=0.0025, avoid=False, time_scale=5.0)
        print("enter ok", ok, flush=True)
        p.grip(False)
        print("gap after close", round(p.finger_gap(), 4), flush=True)
    elif stage == "upright":
        gap = p.finger_gap(); print("gap", round(gap, 4))
        if gap < 0.002:
            print("nothing in the gripper"); return
        attach_lying(p, R, c_rel)
        pos = p.tcp()[0]
        ok = p.line(pos + np.array([0, 0, 0.03]), quat=q, step=0.004, avoid=False, time_scale=4.0)
        print("lift ok", ok, "gap", round(p.finger_gap(), 4), flush=True)
        # candidate upright poses: R_final = Rz(phi) @ Rx(-90deg) @ R ; mug bottom 6 mm above the table
        Rx = np.array([[1, 0, 0], [0, 0, 1.0], [0, -1.0, 0]])            # rotation about x by -90deg: +y -> -z
        assert np.allclose(Rx @ np.array([0, 1.0, 0]), [0, 0, -1])
        best = None
        for phi_deg in [0, 20, 40, 60, 80, 100, 120, 140, 160, 180, -20, -40, -60, -80, -100, -120, -140, -160]:
            Rf = Rz(np.deg2rad(phi_deg)) @ Rx @ R
            c_rel_f = Rz(np.deg2rad(phi_deg)) @ Rx @ c_rel            # mug centre rel. TCP after rotation
            for cx, cy in [(-0.15, -0.42), (-0.12, -0.42), (-0.18, -0.45), (-0.10, -0.46)]:
                tcp_f = np.array([cx, cy, 0.906 + MUG_LEN / 2]) - c_rel_f
                s = p.ik(tcp_f, mat_to_quat(Rf), attempts=4, avoid=True)
                if s is not None:
                    best = (phi_deg, (cx, cy), tcp_f, Rf, s); break
            if best: break
        if best is None:
            print("no collision-free upright pose found"); return
        phi_deg, c, tcp_f, Rf, s = best
        print(f"upright pose: phi {phi_deg} mug centre {c} tcp {np.round(tcp_f,4)} q {np.round(s,3).tolist()}", flush=True)
        ok = p.goto_q(s, time_s=10.0)
        print("rotate ok", ok, "gap", round(p.finger_gap(), 4), "tcp", np.round(p.tcp()[0], 4), flush=True)
        if not ok: return
        # lower 6 mm to set the base on the table, release, retreat along -a (up and away)
        qf = mat_to_quat(Rf)
        ok = p.line(tcp_f - np.array([0, 0, 0.005]), quat=qf, step=0.0025, avoid=False, time_scale=4.0)
        print("set down ok", ok, flush=True)
        p.grip(True)
        p.detach_mug()
        af = Rf[:, 2]
        ok = p.line(p.tcp()[0] - 0.06 * af + np.array([0, 0, 0.03]), quat=qf, step=0.004, avoid=False, time_scale=3.0)
        print("retreat ok", ok, flush=True)
    print("end tcp", np.round(p.tcp()[0], 4), "q", np.round(p.arm_q(), 3).tolist())
    p.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
EOF
cd /workspace && python3 -u tools/right_mug.py info 2>&1 | grep -v "^ik:" | tail -5

# openrua op 115
python3 -u tools/right_mug.py pinch 2>&1 | grep -v "^ik:\|^traj done" | tail -8

# openrua op 116
cd /workspace/snaps && python3 /workspace/tools/cam_snap.py robot0_eye_in_hand agentview --depth 2>&1 | tail -1

# openrua op 117
python3 -u tools/right_mug.py upright 2>&1 | grep -v "^ik:\|^traj done" | tail -10

# openrua op 118
cd /workspace && python3 -u tools/right_mug.py upright 2>&1 | grep -v "^ik:\|^traj done" | tail -10

# openrua op 119
cd /workspace/snaps && python3 /workspace/tools/cam_snap.py agentview birdview --depth 2>&1 | tail -1

# openrua op 120
cd /workspace && python3 - <<'EOF'
import sys, numpy as np
sys.path.insert(0,"/workspace/tools"); import cams
from measure_mug import fit_circle
d=np.load("/workspace/snaps/birdview_depth.npy"); P=cams.cloud("birdview",d).reshape(-1,3)
m=(P[:,0]>-0.30)&(P[:,0]<-0.04)&(P[:,1]>-0.60)&(P[:,1]<-0.33)&(P[:,2]>0.905)&(P[:,2]<1.05)
Q=P[m]; print("pts",len(Q), "zmax", Q[:,2].max().round(3))
xs=np.arange(-0.30,-0.04,0.01); ys=np.arange(-0.60,-0.33,0.01)
print("      y:"+"".join(f"{y:6.2f}" for y in ys))
for x in xs:
    row=[]
    for y in ys:
        mm=(Q[:,0]>=x)&(Q[:,0]<x+0.01)&(Q[:,1]>=y)&(Q[:,1]<y+0.01)
        row.append(f"{Q[mm,2].max():6.3f}" if mm.any() else "     .")
    print(f"x {x:5.2f}:"+"".join(row))
rim=Q[Q[:,2]>0.99]
if len(rim)>30:
    cx,cy,r=fit_circle(rim[:,:2]); print(f"rim fit centre ({cx:.4f},{cy:.4f}) r {r:.4f} n {len(rim)}")
EOF

# openrua op 121
python3 - <<'EOF' 2>&1 | grep -v "^ik:"
import sys, numpy as np, rclpy
sys.path.insert(0,"/workspace/tools")
from robot import Robot, mat_to_quat
rclpy.init(); r=Robot()
AX_X, AX_Z, Y_R, Y_B = -0.165, 0.953, -0.52, -0.415
C = np.array([Y_B, 0.90])                    # pivot (y,z): base lowest edge
for pitch_deg in (50, 55, 60, 65):
    p=np.deg2rad(pitch_deg)
    a=np.array([0,np.cos(p),-np.sin(p)]); yh=np.array([1.0,0,0]); xh=np.cross(yh,a)
    R=np.stack([xh,yh,a],1); q=mat_to_quat(R)
    d = min(0.02, (0.0345)/np.tan(p))          # insertion depth with TCP at axis height
    tcp0=np.array([AX_X, Y_R+d, AX_Z])
    # arc: rotate (tcp0 - C) about x by -90deg in steps
    v0=np.array([tcp0[1]-C[0], tcp0[2]-C[1]]); ang0=np.arctan2(v0[1],v0[0]); rad=np.linalg.norm(v0)
    seed=None; oks=[]
    for th in np.linspace(0, np.pi/2, 7):
        ang=ang0-th
        tcp=np.array([AX_X, C[0]+rad*np.cos(ang), C[1]+rad*np.sin(ang)])
        s=r.ik(tcp,q,seed=seed,attempts=4)
        oks.append("ok" if s is not None else "FAIL")
        if s is not None: seed=s
    print(f"pitch {pitch_deg}: depth {d*100:.1f}cm start {np.round(tcp0,3)} radius {rad:.3f} arc:", oks)
EOF

# openrua op 122
mkdir -p "$(dirname /workspace/tools/scoop.py)"
cat > /workspace/tools/scoop.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Right the lying mug by 'scooping': closed fingers into the mouth just below
the axis height, then move the TCP along an arc about the base edge on the table
so the rim end lifts and the mug tips onto its base.
  python3 tools/scoop.py info|run [pitch_deg]
Mug lying along y, rim at -y end (Y_R), base end at Y_B, axis at (AX_X, AX_Z)."""
import sys
import numpy as np
import rclpy
sys.path.insert(0, "/workspace/tools")
from robot import mat_to_quat
from planner import Planner

AX_X, AX_Z = -0.165, 0.953
Y_R, Y_B = -0.52, -0.415
PITCH = np.deg2rad(float(sys.argv[2]) if len(sys.argv) > 2 else 60.0)
DEPTH = min(0.02, 0.0345 / np.tan(PITCH))
C = np.array([Y_B, 0.90])                       # pivot (y, z)


def frame():
    a = np.array([0.0, np.cos(PITCH), -np.sin(PITCH)])
    yh = np.array([1.0, 0.0, 0.0])              # closing axis along x (irrelevant, fingers closed)
    xh = np.cross(yh, a)
    R = np.stack([xh, yh, a], 1)
    return a, R, mat_to_quat(R)


def arc_points(n):
    tcp0 = np.array([AX_X, Y_R + DEPTH, AX_Z])
    v0 = np.array([tcp0[1] - C[0], tcp0[2] - C[1]])
    ang0, rad = np.arctan2(v0[1], v0[0]), np.linalg.norm(v0)
    pts = []
    for th in np.linspace(0, np.pi / 2, n + 1)[1:]:
        ang = ang0 - th
        pts.append(np.array([AX_X, C[0] + rad * np.cos(ang), C[1] + rad * np.sin(ang)]))
    return tcp0, pts


def main():
    stage = sys.argv[1]
    rclpy.init(); p = Planner()
    a, R, quat = frame()
    tcp0, pts = arc_points(6)
    print("tcp0", np.round(tcp0, 4), "depth", round(DEPTH, 4))
    if stage == "info":
        seed = None
        for q_ in [tcp0 - 0.06 * a, tcp0] + pts:
            s = p.ik(q_, quat, seed=seed, attempts=4)
            print(np.round(q_, 3), "ok" if s is not None else "FAIL")
            if s is not None:
                seed = s
    elif stage == "run":
        p.set_scene(microwave="solid", yellow=False)
        # clear the mug, close fingers
        tcp, _, Rc = p.tcp()
        p.line(tcp + np.array([0, 0, 0.05]), mat_to_quat(Rc), step=0.01, avoid=False, min_fraction=0.5)
        p.grip(False)
        pre = tcp0 - 0.06 * a
        ok = p.goto_tcp(pre, quat, time_s=8)
        print("pre ok", ok, "tcp", np.round(p.tcp()[0], 4))
        ok = p.line(tcp0, quat, step=0.005, avoid=False, time_scale=4)
        print("insert ok", ok, "tcp", np.round(p.tcp()[0], 4))
        for i, pt in enumerate(pts):
            ok = p.line(pt, quat, step=0.005, avoid=False, min_fraction=0.9, time_scale=5)
            print(f"arc {i} target {np.round(pt, 3)} ok {ok} tcp {np.round(p.tcp()[0], 4)}", flush=True)
            if not ok:
                break
        # retreat: up, then back along -y
        tcp = p.tcp()[0]
        ok = p.line(tcp + np.array([0, 0, 0.08]), quat, step=0.01, avoid=False, min_fraction=0.9, time_scale=3)
        print("retreat ok", ok, "tcp", np.round(p.tcp()[0], 4))
    p.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 123
grep -n "def line\|def goto_tcp\|def execute\|def set_scene" tools/planner.py && python3 -u tools/scoop.py info 2>&1 | grep -v "^ik:"

# openrua op 124
python3 - <<'EOF' 2>&1 | grep -v "^ik:"
import sys, numpy as np, rclpy
sys.path.insert(0,"/workspace/tools"); sys.argv=["x","info","60"]
import scoop
from robot import Robot
rclpy.init(); r=Robot()
a,R,quat=scoop.frame(); tcp0,_=scoop.arc_points(6)
for d in (0.02,0.03,0.04,0.05):
    for dz in (0.0, 0.02):
        pre=tcp0-d*a+np.array([0,0,dz])
        s=r.ik(pre,quat,attempts=4)
        print(d,dz,np.round(pre,3),"ok" if s is not None else "FAIL")
EOF

# openrua op 125
sed -i 's/tcp0 = np.array(\[AX_X, Y_R + DEPTH, AX_Z\])/tcp0 = np.array([AX_X, Y_R + DEPTH, AX_Z - 0.01])/; s/tcp0 - 0.06 \* a/tcp0 - 0.04 * a/g' tools/scoop.py && grep -n "tcp0 = np.array\|0.04 \* a" tools/scoop.py && python3 -u tools/scoop.py run 60 2>&1 | grep -v "^ik:"

# openrua op 126
sed -i 's/time_s=8)/time_s=8.0)/' tools/scoop.py && python3 -u tools/scoop.py run 60 2>&1 | grep -v "^ik:"

# openrua op 127
cd /workspace/snaps && python3 /workspace/tools/cam_snap.py agentview birdview --depth >/dev/null 2>&1; python3 - <<'EOF'
import sys, numpy as np
sys.path.insert(0,"/workspace/tools"); import cams
d=np.load("/workspace/snaps/birdview_depth.npy"); P=cams.cloud("birdview", d)
m=(P[:,0]>-0.26)&(P[:,0]<-0.06)&(P[:,1]>-0.62)&(P[:,1]<-0.33)&(P[:,2]>0.905)&(P[:,2]<1.09)
Q=P[m]; print("pts",len(Q))
xs=np.arange(-0.26,-0.06,0.01); ys=np.arange(-0.62,-0.33,0.01)
print("      "+" ".join(f"{y:6.2f}" for y in ys))
for x in xs:
    row=[]
    for y in ys:
        s=Q[(Q[:,0]>=x)&(Q[:,0]<x+0.01)&(Q[:,1]>=y)&(Q[:,1]<y+0.01)]
        row.append(f"{s[:,2].max():6.3f}" if len(s) else "     .")
    print(f"{x:6.2f} "+" ".join(row))
EOF

# openrua op 128
cd /workspace/snaps && python3 - <<'EOF'
import sys, numpy as np
sys.path.insert(0,"/workspace/tools"); import cams
d=np.load("/workspace/snaps/birdview_depth.npy"); P=cams.cloud("birdview", d).reshape(-1,3)
m=(P[:,0]>-0.26)&(P[:,0]<-0.06)&(P[:,1]>-0.62)&(P[:,1]<-0.33)&(P[:,2]>0.905)&(P[:,2]<1.09)
Q=P[m]; print("pts",len(Q))
xs=np.arange(-0.26,-0.06,0.01); ys=np.arange(-0.62,-0.33,0.01)
print("      "+" ".join(f"{y:6.2f}" for y in ys))
for x in xs:
    row=[]
    for y in ys:
        s=Q[(Q[:,0]>=x)&(Q[:,0]<x+0.01)&(Q[:,1]>=y)&(Q[:,1]<y+0.01)]
        row.append(f"{s[:,2].max():6.3f}" if len(s) else "     .")
    print(f"{x:6.2f} "+" ".join(row))
EOF

# openrua op 129
python3 - <<'EOF'
import sys, numpy as np
sys.path.insert(0,"/workspace/tools"); import cams
d=np.load("/workspace/snaps/agentview_depth.npy"); P=cams.cloud("agentview", d).reshape(-1,3)
m=(P[:,0]>-0.30)&(P[:,0]<-0.02)&(P[:,1]>-0.66)&(P[:,1]<-0.40)&(P[:,2]>0.905)&(P[:,2]<1.06)
Q=P[m]; print("pts",len(Q))
print("x range",Q[:,0].min(),Q[:,0].max()," y range",Q[:,1].min(),Q[:,1].max()," z range",Q[:,2].min(),Q[:,2].max())
# height map in (y,z) of the +x-most surface: for each (y,z) cell, max x
ys=np.arange(-0.64,-0.40,0.01); zs=np.arange(0.905,1.06,0.01)
print("       "+" ".join(f"{y:6.2f}" for y in ys))
for z in zs[::-1]:
    row=[]
    for y in ys:
        s=Q[(Q[:,1]>=y)&(Q[:,1]<y+0.01)&(Q[:,2]>=z)&(Q[:,2]<z+0.01)]
        row.append(f"{s[:,0].max():6.3f}" if len(s) else "     .")
    print(f"{z:6.3f} "+" ".join(row))
EOF

# openrua op 130
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/tools/planner.py", "old_string": "        total = max(pts[-1][0], min_time)\\n        if pts[-1][0] < min_time and pts[-1][0] > 0:\\n            k = min_time / pts[-1][0]\\n            pts = [(t * k, q) for t, q in pts]", "new_string": "        # the controller tracks at most ~0.3 rad/s: stretch so no joint averages > MAX_VEL\\n        Q = np.array([q for _, q in pts])\\n        travel = np.abs(np.diff(Q, axis=0)).sum(0).max() if len(Q) > 1 else 0.0\\n        need = max(min_time, travel / self.MAX_VEL)\\n        if pts[-1][0] < need and pts[-1][0] > 0:\\n            k = need / pts[-1][0]\\n            pts = [(t * k, q) for t, q in pts]\\n        total = pts[-1][0]", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 131
cd /workspace && grep -n "^class Planner" tools/planner.py && sed -i 's/^class Planner(Robot):/class Planner(Robot):\n    MAX_VEL = 0.15   # rad\/s average per joint over a trajectory/' tools/planner.py && sed -n '/^class Planner/,/+3p' tools/planner.py | head -5 && python3 -c "import sys; sys.path.insert(0,'tools'); import planner; print(planner.Planner.MAX_VEL)"

# openrua op 132
mkdir -p "$(dirname /workspace/tools/right2.py)"
cat > /workspace/tools/right2.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Second righting attempt. Mug lies along x (rim at +x end), axis (y=AX_Y, z=AX_Z),
handle up-and--y at HANDLE_ANG deg from +y (measured in the y-z plane).
  python3 tools/right2.py roll      # push the handle over the top so it points up (+y roll)
  python3 tools/right2.py pinch     # top-down pinch of the handle bar
  python3 tools/right2.py yaw       # lift, rotate so the mouth faces -x, move to SET_X, set down
  python3 tools/right2.py scoop     # closed fingers into the mouth from -x, arc to tip it upright
"""
import sys
import numpy as np
import rclpy
sys.path.insert(0, "/workspace/tools")
from robot import mat_to_quat
from planner import Planner

AX_Y, AX_Z = -0.50, 0.953
BAR_X = -0.17                    # mid of the handle bar along x
HANDLE_ANG = np.deg2rad(130.0)   # bar direction from axis, angle from +y toward +z
R_MUG, R_BAR = 0.0485, 0.0785
R_TD = np.array([[-1.0, 0, 0], [0, 1.0, 0], [0, 0, -1.0]])   # hand down, fingers close along y
Q_TD = mat_to_quat(R_TD)


def Rz(t):
    c, s = np.cos(t), np.sin(t)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def roll_path():
    pts = []
    for th_deg in np.arange(135, 84, -10):
        th = np.deg2rad(th_deg)
        ax_y = AX_Y + R_MUG * (HANDLE_ANG - th)          # mug rolls toward +y
        bar = np.array([BAR_X, ax_y + R_BAR * np.cos(th), AX_Z + R_BAR * np.sin(th)])
        pts.append(bar + np.array([0, -0.0175, 0.0]))
    return pts


def main():
    stage = sys.argv[1]
    rclpy.init(); p = Planner()
    if stage == "roll":
        p.set_scene(microwave="solid", yellow=False)
        p.grip(False)
        pts = roll_path()
        pre = pts[0] + np.array([0, -0.01, 0.06])
        ok = p.goto_tcp(pre, Q_TD)
        print("pre ok", ok)
        ok = p.line(pts[0] + np.array([0, -0.01, 0]), Q_TD, step=0.005, avoid=False, time_scale=3)
        print("down ok", ok)
        for i, pt in enumerate(pts):
            ok = p.line(pt, Q_TD, step=0.004, avoid=False, min_fraction=0.9, time_scale=5)
            print(f"roll {i} target {np.round(pt, 3)} ok {ok} tcp {np.round(p.tcp()[0], 4)}", flush=True)
            if not ok:
                break
        tcp = p.tcp()[0]
        p.line(tcp + np.array([0, -0.01, 0.06]), Q_TD, step=0.01, avoid=False, min_fraction=0.9)
    elif stage == "pinch":
        bar = np.array([float(v) for v in sys.argv[2:5]]) if len(sys.argv) > 4 else None
        p.set_scene(microwave="solid", yellow=False)
        p.grip(True)
        pre = bar + np.array([0, 0, 0.06])
        ok = p.goto_tcp(pre, Q_TD)
        print("pre ok", ok)
        ok = p.line(bar, Q_TD, step=0.004, avoid=False, time_scale=4)
        print("descend ok", ok, "tcp", np.round(p.tcp()[0], 4))
        p.grip(False)
        print("gap", round(p.finger_gap(), 4))
    elif stage == "yaw":
        # args: mug axis direction base->rim angle (deg, from +x), set-down x
        ang_now = np.deg2rad(float(sys.argv[2])); set_x = float(sys.argv[3])
        tcp, _, R = p.tcp()
        p.line(tcp + np.array([0, 0, 0.04]), mat_to_quat(R), step=0.005, avoid=False, time_scale=3)
        print("lift gap", round(p.finger_gap(), 4))
        # rotate about z so base->rim points to -x (angle pi)
        dth = (np.pi - ang_now + np.pi) % (2 * np.pi) - np.pi
        print("rotate by deg", np.rad2deg(dth))
        R2 = Rz(dth) @ R
        target = np.array([set_x, tcp[1], tcp[2] + 0.04])
        ok = p.goto_tcp(target, mat_to_quat(R2))
        print("rotate/move ok", ok, "gap", round(p.finger_gap(), 4))
        ok = p.line(np.array([set_x, tcp[1], tcp[2] + 0.004]), mat_to_quat(R2), step=0.005, avoid=False, time_scale=3)
        print("lower ok", ok)
        p.grip(True)
        t2 = p.tcp()[0]
        p.line(t2 + np.array([0, 0, 0.08]), mat_to_quat(R2), step=0.01, avoid=False, min_fraction=0.9)
    elif stage == "scoop":
        # args: rim x, base x, axis y, axis z
        x_r, x_b, ay, az = map(float, sys.argv[2:6])
        pitch = np.deg2rad(60.0)
        a = np.array([np.cos(pitch), 0, -np.sin(pitch)])
        xh = np.array([0, 1.0, 0]); yh = np.cross(a, xh)
        R = np.stack([xh, yh, a], 1); quat = mat_to_quat(R)
        depth = min(0.02, 0.0345 / np.tan(pitch))
        tcp0 = np.array([x_r + depth, ay, az - 0.01])
        C = np.array([x_b, 0.90])
        v0 = np.array([tcp0[0] - C[0], tcp0[2] - C[1]]); ang0 = np.arctan2(v0[1], v0[0]); rad = np.linalg.norm(v0)
        pts = []
        for th in np.linspace(0, np.pi / 2, 7)[1:]:
            ang = ang0 + th          # -x end rises toward +z: angle increases toward 90deg
            pts.append(np.array([C[0] + rad * np.cos(ang), ay, C[1] + rad * np.sin(ang)]))
        print("tcp0", np.round(tcp0, 4), "arc", [np.round(q, 3).tolist() for q in pts])
        p.set_scene(microwave="solid", yellow=False)
        p.grip(False)
        pre = tcp0 - 0.05 * a
        ok = p.goto_tcp(pre, quat)
        print("pre ok", ok, "tcp", np.round(p.tcp()[0], 4))
        ok = p.line(tcp0, quat, step=0.005, avoid=False, time_scale=4)
        print("insert ok", ok, "tcp", np.round(p.tcp()[0], 4))
        for i, pt in enumerate(pts):
            ok = p.line(pt, quat, step=0.005, avoid=False, min_fraction=0.9, time_scale=5)
            print(f"arc {i} target {np.round(pt, 3)} ok {ok} tcp {np.round(p.tcp()[0], 4)}", flush=True)
            if not ok:
                break
        tcp = p.tcp()[0]
        ok = p.line(tcp + np.array([0, 0, 0.08]), quat, step=0.01, avoid=False, min_fraction=0.9, time_scale=3)
        print("retreat ok", ok, "tcp", np.round(p.tcp()[0], 4))
    p.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 133
python3 -u tools/right2.py roll 2>&1 | grep -v "^ik:"

# openrua op 134
cd /workspace/snaps && python3 /workspace/tools/cam_snap.py agentview birdview --depth >/dev/null 2>&1; python3 - <<'EOF'
import sys, numpy as np
sys.path.insert(0,"/workspace/tools"); import cams
for cam,axes in (("birdview",(0,1,2)),("agentview",(1,2,0))):
    d=np.load(f"/workspace/snaps/{cam}_depth.npy"); P=cams.cloud(cam, d).reshape(-1,3)
    m=(P[:,0]>-0.30)&(P[:,0]<-0.02)&(P[:,1]>-0.66)&(P[:,1]<-0.36)&(P[:,2]>0.905)&(P[:,2]<1.06)
    Q=P[m]; print(cam,"pts",len(Q), "x",Q[:,0].min().round(3),Q[:,0].max().round(3),"y",Q[:,1].min().round(3),Q[:,1].max().round(3),"z",Q[:,2].min().round(3),Q[:,2].max().round(3))
    if cam=="birdview":
        xs=np.arange(-0.26,-0.04,0.01); ys=np.arange(-0.60,-0.38,0.01)
        print("      "+" ".join(f"{y:6.2f}" for y in ys))
        for x in xs:
            row=[]
            for y in ys:
                s=Q[(Q[:,0]>=x)&(Q[:,0]<x+0.01)&(Q[:,1]>=y)&(Q[:,1]<y+0.01)]
                row.append(f"{s[:,2].max():6.3f}" if len(s) else "     .")
            print(f"{x:6.2f} "+" ".join(row))
    else:
        ys=np.arange(-0.60,-0.38,0.01); zs=np.arange(0.905,1.06,0.01)
        print("       "+" ".join(f"{y:6.2f}" for y in ys))
        for z in zs[::-1]:
            row=[]
            for y in ys:
                s=Q[(Q[:,1]>=y)&(Q[:,1]<y+0.01)&(Q[:,2]>=z)&(Q[:,2]<z+0.01)]
                row.append(f"{s[:,0].max():6.3f}" if len(s) else "     .")
            print(f"{z:6.3f} "+" ".join(row))
EOF

# openrua op 135
python3 -u tools/right2.py pinch -0.16 -0.465 1.026 2>&1 | grep -v "^ik:"

# openrua op 136
cd /workspace && python3 -u tools/right2.py pinch -0.16 -0.465 1.026 2>&1 | grep -v "^ik:"

# openrua op 137
python3 - <<'EOF' 2>&1 | grep -v "^ik:"
import sys, numpy as np, rclpy
sys.path.insert(0,"/workspace/tools")
from robot import mat_to_quat
from planner import Planner
rclpy.init(); p=Planner(); p.set_scene(microwave="solid", yellow=False)
pitch=np.deg2rad(60); a=np.array([np.cos(pitch),0,-np.sin(pitch)]); xh=np.array([0,1.0,0]); yh=np.cross(a,xh)
quat=mat_to_quat(np.stack([xh,yh,a],1)); depth=0.02
for set_x in (-0.20,-0.18,-0.16,-0.14):
    x_r, x_b = set_x-0.05, set_x+0.055
    ay, az = -0.40, 0.953
    tcp0=np.array([x_r+depth, ay, az-0.01]); C=np.array([x_b,0.90])
    v0=np.array([tcp0[0]-C[0], tcp0[2]-C[1]]); ang0=np.arctan2(v0[1],v0[0]); rad=np.linalg.norm(v0)
    pts=[tcp0-0.05*a, tcp0]+[np.array([C[0]+rad*np.cos(ang0+th), ay, C[1]+rad*np.sin(ang0+th)]) for th in np.linspace(0,np.pi/2,7)[1:]]
    seed=None; res=[]
    for q_ in pts:
        s=p.ik(q_,quat,seed=seed,attempts=4,avoid=True); res.append("ok" if s is not None else "X")
        if s is not None: seed=s
    print(f"set_x {set_x}: mug stands at x {x_b+0.039:.3f};", res)
EOF

# openrua op 138
python3 - <<'EOF' 2>&1 | grep -v "^ik:"
import sys, numpy as np, rclpy
sys.path.insert(0,"/workspace/tools")
from robot import mat_to_quat
from planner import Planner
rclpy.init(); p=Planner(); p.set_scene(microwave="solid", yellow=False)
pitch=np.deg2rad(60); a=np.array([np.cos(pitch),0,-np.sin(pitch)]); xh=np.array([0,1.0,0]); yh=np.cross(a,xh)
quat=mat_to_quat(np.stack([xh,yh,a],1))
# sanity: a clearly free pose, with/without avoid
free=np.array([-0.15,-0.40,1.05])
print("free pose avoid:", p.ik(free,quat,attempts=3,avoid=True) is not None, " noavoid:", p.ik(free,quat,attempts=3) is not None)
for set_x in (-0.18,-0.14):
    x_r, x_b = set_x-0.05, set_x+0.055; ay, az = -0.40, 0.953
    tcp0=np.array([x_r+0.02, ay, az-0.01]); C=np.array([x_b,0.90])
    v0=np.array([tcp0[0]-C[0], tcp0[2]-C[1]]); ang0=np.arctan2(v0[1],v0[0]); rad=np.linalg.norm(v0)
    pts=[tcp0-0.05*a, tcp0]+[np.array([C[0]+rad*np.cos(ang0+th), ay, C[1]+rad*np.sin(ang0+th)]) for th in np.linspace(0,np.pi/2,7)[1:]]
    seed=None; res=[]
    for q_ in pts:
        s=p.ik(q_,quat,seed=seed,attempts=4); res.append("ok" if s is not None else "X")
        if s is not None: seed=s
    print(f"set_x {set_x} noavoid:", res)
EOF

# openrua op 139
python3 - <<'EOF' 2>&1 | grep -v "^ik:"
import sys, numpy as np, rclpy
sys.path.insert(0,"/workspace/tools")
from robot import mat_to_quat
from planner import Planner
rclpy.init(); p=Planner()
pitch=np.deg2rad(60); a=np.array([np.cos(pitch),0,-np.sin(pitch)]); xh=np.array([0,1.0,0]); yh=np.cross(a,xh)
quat=mat_to_quat(np.stack([xh,yh,a],1))
for set_x in (-0.20,-0.18,-0.16,-0.14):
    x_r, x_b = set_x-0.05, set_x+0.055; ay, az = -0.40, 0.953
    tcp0=np.array([x_r+0.02, ay, az-0.01]); C=np.array([x_b,0.90])
    v0=np.array([tcp0[0]-C[0], tcp0[2]-C[1]]); ang0=np.arctan2(v0[1],v0[0]); rad=np.linalg.norm(v0)
    pts=[tcp0-0.05*a, tcp0]+[np.array([C[0]+rad*np.cos(ang0+th), ay, C[1]+rad*np.sin(ang0+th)]) for th in np.linspace(0,np.pi/2,7)[1:]]
    seed=None; worst=9
    for q_ in pts:
        s=p.ik(q_,quat,seed=seed,attempts=4)
        if s is None: print("IK fail"); break
        seed=s
        for link in ("panda_link6","panda_link7","panda_hand"):
            pos,_,_=p.fk(s,link)
            # clearance to door box x<=-0.296, y in [-0.59,-0.29], z<=1.11 (+ link radius ~0.05)
            dx=pos[0]-(-0.296); dz=pos[2]-1.11
            if -0.59-0.05<pos[1]<-0.29+0.05:
                worst=min(worst, max(dx,dz))
    print(f"set_x {set_x}: worst clearance (centre to door box, want >0.05): {worst:.3f}")
EOF

# openrua op 140
python3 - <<'EOF' 2>&1 | grep -v "^ik:"
import sys, numpy as np, rclpy
sys.path.insert(0,"/workspace/tools")
from robot import mat_to_quat
from planner import Planner
rclpy.init(); p=Planner()
for pitch_deg in (60,70):
  pitch=np.deg2rad(pitch_deg); a=np.array([np.cos(pitch),0,-np.sin(pitch)]); xh=np.array([0,1.0,0]); yh=np.cross(a,xh)
  quat=mat_to_quat(np.stack([xh,yh,a],1))
  set_x=-0.14; x_r, x_b = set_x-0.05, set_x+0.055; ay, az = -0.40, 0.953
  tcp0=np.array([x_r+0.02, ay, az-0.01]); C=np.array([x_b,0.90])
  v0=np.array([tcp0[0]-C[0], tcp0[2]-C[1]]); ang0=np.arctan2(v0[1],v0[0]); rad=np.linalg.norm(v0)
  pts=[tcp0-0.05*a, tcp0]+[np.array([C[0]+rad*np.cos(ang0+th), ay, C[1]+rad*np.sin(ang0+th)]) for th in np.linspace(0,np.pi/2,7)[1:]]
  seed=None
  for q_ in pts:
    s=p.ik(q_,quat,seed=seed,attempts=4); seed=s
    row=[]
    for link in ("panda_link4","panda_link5","panda_link6","panda_link7"):
        pos,_,_=p.fk(s,link); row.append(f"{link[-1]}:({pos[0]:.2f},{pos[1]:.2f},{pos[2]:.2f})")
    print(pitch_deg, np.round(q_,3), " ".join(row))
EOF

# openrua op 141
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/tools/planner.py", "old_string": "    def execute(self, jt, time_scale=3.0, min_time=3.0):", "new_string": "    def cartesian_poses(self, poses, step=0.005, jump=5.0, avoid=False):\\n        \\"\\"\\"Cartesian path through (tcp_pos, quat) keyframes; orientation is slerped between them.\\"\\"\\"\\n        req = GetCartesianPath.Request()\\n        req.header.frame_id = \\"world\\"\\n        req.start_state = self._start_state()\\n        req.group_name = M[\\"planning\\"][\\"group\\"]\\n        req.link_name = \\"panda_hand\\"\\n        req.max_step = step\\n        req.jump_threshold = jump\\n        req.avoid_collisions = avoid\\n        req.max_velocity_scaling_factor = 0.3\\n        req.max_acceleration_scaling_factor = 0.3\\n        for pos, quat in poses:\\n            R = quat_to_mat(quat)\\n            p = Pose()\\n            hp = np.asarray(pos, float) - R @ np.array([0, 0, TCP])\\n            p.position.x, p.position.y, p.position.z = map(float, hp)\\n            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)\\n            req.waypoints.append(p)\\n        res = self._call(self.cart_cli, req, timeout=60)\\n        if res is None or res.error_code.val != 1:\\n            print(\\"cartesian failed:\\", res and res.error_code.val, flush=True)\\n            return None, 0.0\\n        return res.solution.joint_trajectory, res.fraction\\n\\n    def execute(self, jt, time_scale=3.0, min_time=3.0):", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 142
cat >> /workspace/tools/right2.py <<'EOF'


def Ry(t):
    c, s = np.cos(t), np.sin(t)
    return np.array([[c, 0, s], [0, 1.0, 0], [-s, 0, c]])


def tilt_frames(phi_deg, tilt_deg, p0, p1, n=8):
    """Keyframes rotating the TD pinch about world y by -tilt (rim +x -> up) and yawing by phi,
    while the TCP slides from p0 to p1."""
    out = []
    for k in range(1, n + 1):
        f = k / n
        R = Rz(np.deg2rad(phi_deg) * f) @ Ry(-np.deg2rad(tilt_deg) * f) @ R_TD
        out.append((p0 + (p1 - p0) * f, mat_to_quat(R)))
    return out


def tilt_main(stage):
    rclpy.init(); p = Planner()
    phi, tilt = float(sys.argv[2]), float(sys.argv[3])
    p1 = np.array([float(v) for v in sys.argv[4:7]])        # TCP at end of rotation (z high)
    z_set = float(sys.argv[7])
    tcp, _, R = p.tcp()
    p0 = np.array([tcp[0], tcp[1], p1[2]])
    frames = tilt_frames(phi, tilt, p0, p1)
    h = Rz(np.deg2rad(phi)) @ Ry(-np.deg2rad(tilt)) @ np.array([0, 0, 1.0])
    print("handle dir at end", np.round(h, 3), "mug centre offset", np.round(-0.0785 * h, 4))
    if stage == "tiltinfo":
        seed = None
        for pos, q in frames + [(np.array([p1[0], p1[1], z_set]), frames[-1][1])]:
            s = p.ik(pos, q, seed=seed, attempts=4)
            print(np.round(pos, 3), "ok" if s is not None else "FAIL")
            if s is not None:
                seed = s
        return
    ok = p.line(p0, mat_to_quat(R), step=0.005, avoid=False, time_scale=3)
    print("lift ok", ok, "gap", round(p.finger_gap(), 4))
    jt, frac = p.cartesian_poses(frames, step=0.005)
    print("cartesian fraction", frac)
    if jt is None or frac < 0.95:
        return
    p.execute(jt, time_scale=6)
    print("rotated; gap", round(p.finger_gap(), 4), "tcp", np.round(p.tcp()[0], 4))
    q_end = frames[-1][1]
    ok = p.line(np.array([p1[0], p1[1], z_set]), q_end, step=0.005, avoid=False, time_scale=4)
    print("lowered ok", ok, "gap", round(p.finger_gap(), 4))
    p.grip(True)
    a = Rz(np.deg2rad(phi)) @ Ry(-np.deg2rad(tilt)) @ np.array([0, 0, -1.0])
    t2 = p.tcp()[0]
    p.line(t2 - 0.05 * a + np.array([0, 0, 0.05]), q_end, step=0.01, avoid=False, min_fraction=0.9)
    print("retreated; tcp", np.round(p.tcp()[0], 4))
    p.destroy_node(); rclpy.shutdown()


if __name__ == "__main__" and sys.argv[1] in ("tilt", "tiltinfo"):
    tilt_main(sys.argv[1])
EOF
sed -i 's/^if __name__ == "__main__":\n    main()//' tools/right2.py && python3 - <<'EOF'
import re
s=open('/workspace/tools/right2.py').read()
s=s.replace('if __name__ == "__main__":\n    main()\n', 'if __name__ == "__main__" and sys.argv[1] not in ("tilt", "tiltinfo"):\n    main()\n',1)
open('/workspace/tools/right2.py','w').write(s)
EOF
python3 -u tools/right2.py tiltinfo 60 65 -0.143 -0.43 1.10 0.997 2>&1 | grep -v "^ik:"; python3 -u tools/right2.py tiltinfo 75 65 -0.140 -0.43 1.10 0.997 2>&1 | grep -v "^ik:"

# openrua op 143
python3 -u tools/right2.py tiltinfo 60 65 -0.15 -0.45 1.10 0.997 2>&1 | grep -v "^ik:"

# openrua op 144
python3 -u tools/right2.py tilt 60 65 -0.15 -0.45 1.10 0.997 2>&1 | grep -v "^ik:"

# openrua op 145
python3 - <<'EOF' 2>&1 | grep -v "^ik:"
import sys, numpy as np, rclpy
sys.path.insert(0,"/workspace/tools")
from planner import Planner
from right2 import tilt_frames, Rz, Ry, R_TD
rclpy.init(); p=Planner()
q=p.arm_q(); print("q now", np.round(q,3))
tcp,_,R=p.tcp(); print("tcp", np.round(tcp,4)); print(np.round(R,3))
p0=np.array([tcp[0],tcp[1],1.10]); p1=np.array([-0.15,-0.45,1.10])
for phi in (60,-60):
    print("phi",phi)
    seed=q
    for pos,qq in tilt_frames(phi,65,p0,p1):
        s=p.ik(pos,qq,seed=seed,attempts=3)
        print(np.round(pos,3), None if s is None else np.round(s,2))
        if s is not None: seed=s
EOF

# openrua op 146
python3 - <<'EOF' 2>&1 | grep -v "^ik:"
import sys, numpy as np, rclpy
sys.path.insert(0,"/workspace/tools")
from planner import Planner
from robot import mat_to_quat
from right2 import Rz, Ry, R_TD
rclpy.init(); p=Planner()
q=p.arm_q()
tcp,_,R=p.tcp()
p0=np.array([tcp[0],tcp[1],1.10]); p1=np.array([-0.15,-0.45,1.10])
R_F = R_TD @ Rz(np.pi)
print("flipped grasp pose q:", np.round(p.ik(tcp, mat_to_quat(R_F), seed=q),2))
seed=None
for k in range(0,9):
    f=k/8
    Rk = Rz(np.deg2rad(60)*f) @ Ry(-np.deg2rad(65)*f) @ R_F
    pos = p0+(p1-p0)*f
    s=p.ik(pos,mat_to_quat(Rk),seed=seed,attempts=3)
    print(k, np.round(pos,3), None if s is None else np.round(s,2))
    if s is not None: seed=s
s=p.ik(np.array([-0.15,-0.45,0.997]), mat_to_quat(Rk), seed=seed); print("low", None if s is None else np.round(s,2))
EOF

# openrua op 147
python3 - <<'EOF' 2>&1 | grep -v "^ik:"
import sys, numpy as np, rclpy
sys.path.insert(0,"/workspace/tools")
from planner import Planner
from robot import mat_to_quat
from right2 import Rz, Ry, R_TD
rclpy.init(); p=Planner()
q=p.arm_q()
tcp,_,R=p.tcp()
p0=np.array([tcp[0],tcp[1],1.10]); p1=np.array([-0.15,-0.45,1.10])
R_F = R_TD @ Rz(np.pi)
seed = q.copy(); seed[6] += np.pi
print("fk check", np.round(p.tcp(seed)[0],4)); print(np.round(p.tcp(seed)[2],2))
for k in range(0,9):
    f=k/8
    Rk = Rz(np.deg2rad(60)*f) @ Ry(-np.deg2rad(65)*f) @ R_F
    pos = p0+(p1-p0)*f
    s=p.ik(pos,mat_to_quat(Rk),seed=seed,attempts=3)
    print(k, np.round(pos,3), None if s is None else np.round(s,2))
    if s is not None: seed=s
s=p.ik(np.array([-0.15,-0.45,0.997]), mat_to_quat(Rk), seed=seed); print("low", None if s is None else np.round(s,2))
EOF

# openrua op 148
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/tools/right2.py", "old_string": "def tilt_frames(phi_deg, tilt_deg, p0, p1, n=8):\\n    \\"\\"\\"Keyframes rotating the TD pinch about world y by -tilt (rim +x -> up) and yawing by phi,\\n    while the TCP slides from p0 to p1.\\"\\"\\"\\n    out = []\\n    for k in range(1, n + 1):\\n        f = k / n\\n        R = Rz(np.deg2rad(phi_deg) * f) @ Ry(-np.deg2rad(tilt_deg) * f) @ R_TD\\n        out.append((p0 + (p1 - p0) * f, mat_to_quat(R)))\\n    return out\\n\\n\\ndef tilt_main(stage):\\n    rclpy.init(); p = Planner()\\n    phi, tilt = float(sys.argv[2]), float(sys.argv[3])\\n    p1 = np.array([float(v) for v in sys.argv[4:7]])        # TCP at end of rotation (z high)\\n    z_set = float(sys.argv[7])\\n    tcp, _, R = p.tcp()\\n    p0 = np.array([tcp[0], tcp[1], p1[2]])\\n    frames = tilt_frames(phi, tilt, p0, p1)", "new_string": "def tilt_frames(phi_deg, tilt_deg, p0, p1, n=8, R0=R_TD):\\n    \\"\\"\\"Keyframes rotating the TD pinch about world y by -tilt (rim +x -> up) and yawing by phi,\\n    while the TCP slides from p0 to p1.\\"\\"\\"\\n    out = []\\n    for k in range(1, n + 1):\\n        f = k / n\\n        R = Rz(np.deg2rad(phi_deg) * f) @ Ry(-np.deg2rad(tilt_deg) * f) @ R0\\n        out.append((p0 + (p1 - p0) * f, mat_to_quat(R)))\\n    return out\\n\\n\\ndef tilt_main(stage):\\n    rclpy.init(); p = Planner()\\n    if stage == \\"flip\\":\\n        # release, lift, rotate the hand by pi about its z (joint 7), re-pinch at the same TCP\\n        tcp, quat, R = p.tcp()\\n        p.grip(True)\\n        p.line(tcp + np.array([0, 0, 0.06]), quat, step=0.01, avoid=False, min_fraction=0.9)\\n        q = p.arm_q(); q[6] += np.pi if q[6] < 0 else -np.pi\\n        p.move(q, 25.0)\\n        tcp2, quat2, R2 = p.tcp()\\n        print(\\"flipped R\\\\n\\", np.round(R2, 3))\\n        ok = p.line(tcp, quat2, step=0.004, avoid=False, time_scale=4)\\n        print(\\"descend ok\\", ok, \\"tcp\\", np.round(p.tcp()[0], 4))\\n        p.grip(False)\\n        print(\\"gap\\", round(p.finger_gap(), 4))\\n        p.destroy_node(); rclpy.shutdown(); return\\n    phi, tilt = float(sys.argv[2]), float(sys.argv[3])\\n    p1 = np.array([float(v) for v in sys.argv[4:7]])        # TCP at end of rotation (z high)\\n    z_set = float(sys.argv[7])\\n    tcp, _, R = p.tcp()\\n    p0 = np.array([tcp[0], tcp[1], p1[2]])\\n    frames = tilt_frames(phi, tilt, p0, p1, R0=R)", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 149
sed -i 's/if __name__ == "__main__" and sys.argv\[1\] not in ("tilt", "tiltinfo"):/if __name__ == "__main__" and sys.argv[1] not in ("tilt", "tiltinfo", "flip"):/; s/if __name__ == "__main__" and sys.argv\[1\] in ("tilt", "tiltinfo"):/if __name__ == "__main__" and sys.argv[1] in ("tilt", "tiltinfo", "flip"):/' tools/right2.py && grep -n '__main__' tools/right2.py && python3 -u tools/right2.py flip 2>&1 | grep -v "^ik:"

# openrua op 150
python3 - <<'EOF' 2>&1 | grep -v "^ik:"
import sys, numpy as np, rclpy
sys.path.insert(0,"/workspace/tools")
from planner import Planner
rclpy.init(); p=Planner()
p.set_scene(microwave="solid", yellow=False)
p.grip(True)
tcp,quat,R=p.tcp()
ok=p.line(np.array([-0.1602,-0.4646,1.0262]), quat, step=0.004, avoid=False, time_scale=4)
print("descend ok", ok, "tcp", np.round(p.tcp()[0],4))
p.grip(False); print("gap", round(p.finger_gap(),4))
EOF

# openrua op 151
python3 -u tools/right2.py tilt 60 65 -0.15 -0.45 1.10 0.997 2>&1 | grep -v "^ik:"

# openrua op 152
cd /workspace/snaps && python3 /workspace/tools/cam_snap.py agentview birdview --depth 2>&1 | tail -2

# openrua op 153
cd /workspace && python3 - <<'EOF'
import sys, numpy as np, json
sys.path.insert(0,"/workspace/tools")
import cams
d=np.load("snaps/birdview_depth.npy")
P=cams.cloud("birdview", d).reshape(-1,3)
m=(P[:,0]>-0.30)&(P[:,0]<0.0)&(P[:,1]>-0.55)&(P[:,1]<-0.32)&(P[:,2]>0.905)&(P[:,2]<1.09)
Q=P[m]
print("n",len(Q))
# 1cm grid of max height
xs=np.arange(-0.30,0.0,0.01); ys=np.arange(-0.55,-0.32,0.01)
H=np.full((len(xs),len(ys)),np.nan)
for q in Q:
    i=int((q[0]+0.30)/0.01); j=int((q[1]+0.55)/0.01)
    if 0<=i<len(xs) and 0<=j<len(ys):
        H[i,j]=np.nanmax([H[i,j],q[2]])
np.set_printoptions(linewidth=250)
print("cols y from -0.55 to -0.33")
for i,x in enumerate(xs):
    print(f"{x:6.2f} "+" ".join("  ." if np.isnan(v) else f"{int((v-0.9)*100):3d}" for v in H[i]))
EOF

# openrua op 154
head -30 tools/eih.py && ls snaps | head -30

# openrua op 155
python3 -c "import json;d=json.load(open('/workspace/cam_extrinsics.json'));[print(k, {kk:(v[kk] if kk!='R' else None) for kk in v}) for k,v in d.items()]" 2>&1 | head; cd /workspace/snaps && python3 /workspace/tools/cam_snap.py frontview sideview --depth 2>&1 | tail -2

# openrua op 156
cd /workspace && python3 - <<'EOF' 2>&1 | grep -v "^ik:"
import sys, numpy as np, rclpy
sys.path.insert(0,"/workspace/tools")
from planner import Planner
from robot import mat_to_quat
rclpy.init(); p=Planner()
p.set_scene(microwave="solid", yellow=False)
R=np.array([[1.0,0,0],[0,-1.0,0],[0,0,-1.0]])
ok=p.goto_tcp(np.array([-0.10,-0.41,1.33]), mat_to_quat(R))
print("ok",ok,"tcp",np.round(p.tcp()[0],4))
EOF
python3 /workspace/tools/eih.py snap 2>&1 | tail -1

# openrua op 157
python3 tools/eih.py snap; ls -la eih.png snaps/eih.png 2>&1

# openrua op 158
sed -n 30,80p tools/eih.py

# openrua op 159
cd /workspace/snaps && python3 /workspace/tools/cam_snap.py robot0_eye_in_hand --depth 2>&1 | tail -1

# openrua op 160
cd /workspace && python3 - <<'EOF' 2>&1 | grep -v "^ik:"
import sys, numpy as np, rclpy
sys.path.insert(0,"/workspace/tools")
import eih
from robot import Robot
rclpy.init(); r=Robot()
pos,quat,R=r.fk()
d=np.load("snaps/robot0_eye_in_hand_depth.npy")
P=eih.cloud(d,pos,R).reshape(-1,3)
img=np.array(__import__("PIL.Image",fromlist=["Image"]).open("snaps/robot0_eye_in_hand.png")).reshape(-1,3)
yellow=(img[:,0]>150)&(img[:,1]>110)&(img[:,2]<120)
m=yellow&(P[:,2]>0.905)&(P[:,2]<1.10)&(P[:,1]<-0.32)&(P[:,0]>-0.3)
Q=P[m]; print("n",len(Q), "bbox", np.round(Q.min(0),3), np.round(Q.max(0),3))
c=Q[:,:2].mean(0); U,S,Vt=np.linalg.svd(Q[:,:2]-c,full_matrices=False)
u=Vt[0]; print("centre xy", np.round(c,4), "axis", np.round(u,3), "ang deg", np.degrees(np.arctan2(u[1],u[0])))
t=(Q[:,:2]-c)@u
print("extent along axis", round(t.min(),3), round(t.max(),3))
# heights along axis
for lo in np.arange(t.min(), t.max(), 0.01):
    s=(t>=lo)&(t<lo+0.01)
    if s.sum(): print(f"t {lo:6.3f} zmax {Q[s,2].max():.3f} n {s.sum()}")
np.save("snaps/mugpts.npy", Q)
EOF

# openrua op 161
python3 - <<'EOF' 2>&1 | grep -v "^ik:"
import sys, numpy as np, rclpy
sys.path.insert(0,"/workspace/tools")
import eih
from robot import Robot
from PIL import Image
rclpy.init(); r=Robot()
pos,quat,R=r.fk()
d=np.load("snaps/robot0_eye_in_hand_depth.npy")
P=eih.cloud(d,pos,R).reshape(-1,3)
img=np.array(Image.open("snaps/robot0_eye_in_hand.png")).reshape(-1,3).astype(int)
white=(img.min(1)>170)
near=(P[:,0]>-0.20)&(P[:,0]<0.0)&(P[:,1]>-0.50)&(P[:,1]<-0.30)&(P[:,2]>0.91)&(P[:,2]<1.02)
W=P[white&near]; print("white n",len(W), "bbox", np.round(W.min(0),3), np.round(W.max(0),3))
print("rim centroid", np.round(W.mean(0),4))
# fit plane normal via SVD on white points
c=W.mean(0); U,S,Vt=np.linalg.svd(W-c); n=Vt[2]; print("rim plane normal", np.round(n,3), "sv", np.round(S,3))
yellow=(img[:,0]>150)&(img[:,1]>110)&(img[:,2]<120)
Y=P[yellow&near&(P[:,2]<0.995)]   # body only, below handle
cy=Y.mean(0); print("body pts n",len(Y),"centroid",np.round(cy,4))
EOF

# openrua op 162
python3 - <<'EOF'
import numpy as np
Q=np.load("snaps/mugpts.npy")
B=Q[Q[:,2]<0.995]
best=None
for th in np.arange(0,180,2):
    u=np.array([np.cos(np.deg2rad(th)),np.sin(np.deg2rad(th))]); v=np.array([-u[1],u[0]])
    lat=(B[:,:2])@v; w=np.percentile(lat,97)-np.percentile(lat,3)
    if best is None or w<best[1]: best=(th,w)
    if th%10==0: print(th, round(w,4))
print("best",best)
th=best[0]; u=np.array([np.cos(np.deg2rad(th)),np.sin(np.deg2rad(th))]); v=np.array([-u[1],u[0]])
lat=B[:,:2]@v; t=B[:,:2]@u
c_lat=(np.percentile(lat,97)+np.percentile(lat,3))/2
print("axis line: lat offset",round(c_lat,4),"t range",round(t.min(),4),round(t.max(),4))
for tt in np.arange(t.min(),t.max(),0.01):
    s=(t>=tt)&(t<tt+0.01)
    if s.sum()>5: print(f"t {tt:.3f} lat width {np.percentile(lat[s],97)-np.percentile(lat[s],3):.3f} zmax {B[s,2].max():.3f}")
# a point on the axis
pt=c_lat*v+u*np.median(t); print("axis passes", np.round(pt,4), "dir", np.round(u,3))
EOF

# openrua op 163
mkdir -p "$(dirname /workspace/tools/right3.py)"
cat > /workspace/tools/right3.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Third righting attempt.
  python3 tools/right3.py pinchinfo bx by bz bar_ang_deg      # IK/joint-7 check for TD pinch + 120deg yaw
  python3 tools/right3.py pinch bx by bz bar_ang_deg yaw_deg  # TD pinch of the handle top bar
  python3 tools/right3.py carry dyaw_deg x y                  # lift, yaw the lying mug, set it down (bar centre at x,y)
  python3 tools/right3.py scoopinfo x_rim y_axis z_axis [pitch]
  python3 tools/right3.py scoop x_rim y_axis z_axis [pitch]   # closed fingers into the mouth from -x, arc about base edge
"""
import sys
import numpy as np
import rclpy
sys.path.insert(0, "/workspace/tools")
from robot import mat_to_quat
from planner import Planner

R_TD = np.array([[-1.0, 0, 0], [0, 1.0, 0], [0, 0, -1.0]])   # hand down, fingers close along y
MUG_LEN, R_RIM = 0.105, 0.0485
BAR_Z = 1.026                                                  # TCP height for pinching the top bar


def Rz(t):
    c, s = np.cos(t), np.sin(t)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def td_R(bar_ang_deg, flip=False):
    """TD hand with the closing axis perpendicular to a bar at bar_ang (deg from +x)."""
    yaw = np.deg2rad(bar_ang_deg + 90.0 - 90.0 + (180.0 if flip else 0.0))
    # R_TD has hand y = +Y; rotate so hand y = perpendicular of the bar = bar_ang + 90
    return Rz(np.deg2rad(bar_ang_deg) + (np.pi if flip else 0.0)) @ R_TD


def scoop_frame(pitch_deg):
    pth = np.deg2rad(pitch_deg)
    a = np.array([np.cos(pth), 0, -np.sin(pth)])      # approach: toward +x and down
    yh = np.array([0, 1.0, 0])
    xh = np.cross(yh, a)
    return a, mat_to_quat(np.stack([xh, yh, a], 1))


def scoop_arc(x_rim, y_ax, z_ax, pitch_deg, n=9):
    depth = min(0.02, 0.0345 / np.tan(np.deg2rad(pitch_deg)))
    tcp0 = np.array([x_rim + depth, y_ax, z_ax - 0.01])
    C = np.array([x_rim + MUG_LEN, 0.90])               # base edge on the table (x, z)
    v0 = np.array([tcp0[0] - C[0], tcp0[2] - C[1]])
    ang0, rad = np.arctan2(v0[1], v0[0]), np.linalg.norm(v0)
    pts = []
    for th in np.linspace(0, np.pi / 2, n + 1)[1:]:
        ang = ang0 - th                                  # rim end rises and moves toward +x
        pts.append(np.array([C[0] + rad * np.cos(ang), y_ax, C[1] + rad * np.sin(ang)]))
    return tcp0, pts


def main():
    stage = sys.argv[1]
    rclpy.init(); p = Planner()
    p.set_scene(microwave="solid", yellow=False)
    if stage == "pinchinfo":
        bx, by, bz, ang = map(float, sys.argv[2:6])
        for flip in (False, True):
            R0 = td_R(ang, flip)
            seed = None
            print("flip", flip)
            for k in range(0, 7):
                dy = np.deg2rad(120.0) * k / 6
                pos = np.array([bx, by, bz + (0.04 if k else 0)])
                s = p.ik(pos, mat_to_quat(Rz(dy) @ R0), seed=seed, attempts=3)
                print(f"  yaw+{np.rad2deg(dy):5.1f}", None if s is None else np.round(s, 2))
                if s is not None:
                    seed = s
    elif stage == "pinch":
        bx, by, bz, ang = map(float, sys.argv[2:6]); flip = len(sys.argv) > 6 and sys.argv[6] == "flip"
        quat = mat_to_quat(td_R(ang, flip))
        p.grip(True)
        bar = np.array([bx, by, bz])
        ok = p.goto_tcp(bar + np.array([0, 0, 0.06]), quat)
        print("pre ok", ok, "q", np.round(p.arm_q(), 2))
        ok = p.line(bar, quat, step=0.004, avoid=False, time_scale=4)
        print("descend ok", ok, "tcp", np.round(p.tcp()[0], 4))
        p.grip(False)
        print("gap", round(p.finger_gap(), 4))
    elif stage == "carry":
        dyaw = np.deg2rad(float(sys.argv[2])); tx, ty = float(sys.argv[3]), float(sys.argv[4])
        tcp, quat, R = p.tcp()
        ok = p.line(tcp + np.array([0, 0, 0.04]), quat, step=0.005, avoid=False, time_scale=3)
        print("lift ok", ok, "gap", round(p.finger_gap(), 4))
        # yaw + translate in keyframes
        frames = []
        n = 8
        p0 = tcp + np.array([0, 0, 0.04]); p1 = np.array([tx, ty, p0[2]])
        for k in range(1, n + 1):
            f = k / n
            frames.append((p0 + (p1 - p0) * f, mat_to_quat(Rz(dyaw * f) @ R)))
        jt, frac = p.cartesian_poses(frames, step=0.005)
        print("cartesian fraction", frac)
        if jt is None or frac < 0.95:
            p.destroy_node(); rclpy.shutdown(); return
        p.execute(jt, time_scale=6)
        q_end = frames[-1][1]
        print("yawed; gap", round(p.finger_gap(), 4), "tcp", np.round(p.tcp()[0], 4))
        ok = p.line(np.array([tx, ty, tcp[2]]), q_end, step=0.005, avoid=False, time_scale=4)
        print("lowered ok", ok, "gap", round(p.finger_gap(), 4), "tcp", np.round(p.tcp()[0], 4))
        p.grip(True)
        t2 = p.tcp()[0]
        p.line(t2 + np.array([0, 0, 0.08]), q_end, step=0.01, avoid=False, min_fraction=0.9)
        print("retreated; tcp", np.round(p.tcp()[0], 4))
    elif stage in ("scoopinfo", "scoop"):
        x_r, y_a, z_a = map(float, sys.argv[2:5])
        pitch = float(sys.argv[5]) if len(sys.argv) > 5 else 60.0
        a, quat = scoop_frame(pitch)
        tcp0, pts = scoop_arc(x_r, y_a, z_a, pitch)
        pre = tcp0 - 0.05 * a
        print("pre", np.round(pre, 4), "tcp0", np.round(tcp0, 4))
        if stage == "scoopinfo":
            seed = None
            for q_ in [pre, tcp0] + pts:
                s = p.ik(q_, quat, seed=seed, attempts=4)
                print(np.round(q_, 3), "FAIL" if s is None else np.round(s, 2))
                if s is not None:
                    seed = s
        else:
            p.grip(False)
            ok = p.goto_tcp(pre, quat, time_s=8.0)
            print("pre ok", ok, "tcp", np.round(p.tcp()[0], 4))
            if not ok:
                p.destroy_node(); rclpy.shutdown(); return
            ok = p.line(tcp0, quat, step=0.005, avoid=False, time_scale=4)
            print("insert ok", ok, "tcp", np.round(p.tcp()[0], 4))
            for i, pt in enumerate(pts):
                ok = p.line(pt, quat, step=0.005, avoid=False, min_fraction=0.9, time_scale=5)
                print(f"arc {i} target {np.round(pt, 3)} ok {ok} tcp {np.round(p.tcp()[0], 4)}", flush=True)
                if not ok:
                    break
            tcp = p.tcp()[0]
            ok = p.line(tcp + np.array([0, 0, 0.10]), quat, step=0.01, avoid=False, min_fraction=0.9, time_scale=3)
            print("retreat ok", ok, "tcp", np.round(p.tcp()[0], 4))
    p.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 164
python3 - <<'EOF'
import re
s=open("tools/right3.py").read()
s=s.replace('''    yaw = np.deg2rad(bar_ang_deg + 90.0 - 90.0 + (180.0 if flip else 0.0))
    # R_TD has hand y = +Y; rotate so hand y = perpendicular of the bar = bar_ang + 90
''','''    # R_TD has hand y = +Y (closing axis); yaw so the closing axis is perpendicular to the bar
''')
open("tools/right3.py","w").write(s)
EOF
python3 -u tools/right3.py pinchinfo -0.101 -0.406 1.026 60 2>&1 | grep -v "^ik:"

# openrua op 165
python3 -u tools/right3.py pinch -0.101 -0.406 1.026 60 2>&1 | grep -v "^ik:"

# openrua op 166
python3 -u tools/right3.py carry 120 -0.1075 -0.42 2>&1 | grep -v "^ik:"

# openrua op 167
python3 - <<'EOF' 2>&1 | grep -v "^ik:"
import sys, numpy as np, rclpy
sys.path.insert(0,"/workspace/tools")
from planner import Planner
from robot import mat_to_quat
rclpy.init(); p=Planner()
p.set_scene(microwave="solid", yellow=False)
R=np.array([[1.0,0,0],[0,-1.0,0],[0,0,-1.0]])
ok=p.goto_tcp(np.array([-0.10,-0.42,1.33]), mat_to_quat(R))
print("ok",ok,"tcp",np.round(p.tcp()[0],4))
EOF
cd /workspace/snaps && python3 /workspace/tools/cam_snap.py robot0_eye_in_hand --depth 2>&1 | tail -1

# openrua op 168
cd /workspace && python3 - <<'EOF' 2>&1 | grep -v "^ik:"
import sys, numpy as np, rclpy
sys.path.insert(0,"/workspace/tools")
import eih
from robot import Robot
from PIL import Image
rclpy.init(); r=Robot()
pos,quat,R=r.fk()
d=np.load("snaps/robot0_eye_in_hand_depth.npy")
P=eih.cloud(d,pos,R).reshape(-1,3)
img=np.array(Image.open("snaps/robot0_eye_in_hand.png")).reshape(-1,3).astype(int)
yellow=(img[:,0]>150)&(img[:,1]>110)&(img[:,2]<120)
near=(P[:,0]>-0.30)&(P[:,0]<0.05)&(P[:,1]>-0.55)&(P[:,1]<-0.32)&(P[:,2]>0.905)&(P[:,2]<1.10)
Q=P[yellow&near]; print("n",len(Q),"bbox",np.round(Q.min(0),3),np.round(Q.max(0),3))
B=Q[Q[:,2]<0.995]
best=None
for th in np.arange(0,180,1):
    u=np.array([np.cos(np.deg2rad(th)),np.sin(np.deg2rad(th))]); v=np.array([-u[1],u[0]])
    lat=B[:,:2]@v; w=np.percentile(lat,97)-np.percentile(lat,3)
    if best is None or w<best[1]: best=(th,w)
th=best[0]; print("axis angle",th,"width",round(best[1],4))
u=np.array([np.cos(np.deg2rad(th)),np.sin(np.deg2rad(th))]); v=np.array([-u[1],u[0]])
lat=B[:,:2]@v; t=B[:,:2]@u; c_lat=(np.percentile(lat,97)+np.percentile(lat,3))/2
for tt in np.arange(t.min(),t.max(),0.01):
    s=(t>=tt)&(t<tt+0.01)
    if s.sum()>5: print(f"t {tt:.3f} lat width {np.percentile(lat[s],97)-np.percentile(lat[s],3):.3f} zmax {B[s,2].max():.3f} n {s.sum()}")
print("t range", round(t.min(),4), round(t.max(),4), "axis point", np.round(c_lat*v,4), "u", np.round(u,3))
# handle
Hh=Q[Q[:,2]>1.02]; print("handle top pts n",len(Hh),"xy mean",np.round(Hh[:,:2].mean(0),4),"zmax",round(Hh[:,2].max(),4))
EOF

# openrua op 169
python3 -u tools/right3.py scoopinfo -0.16 -0.422 0.95 60 2>&1 | grep -v "^ik:"

# openrua op 170
python3 -u tools/right3.py pinch -0.1015 -0.426 1.026 0 2>&1 | grep -v "^ik:" && python3 -u tools/right3.py carry 0 -0.1675 -0.424 2>&1 | grep -v "^ik:"

# openrua op 171
python3 - <<'EOF'
s=open("tools/right3.py").read()
s=s.replace("def scoop_arc(x_rim, y_ax, z_ax, pitch_deg, n=9):","def scoop_arc(x_rim, y_ax, z_ax, pitch_deg, n=9, dz=-0.01):")
s=s.replace("tcp0 = np.array([x_rim + depth, y_ax, z_ax - 0.01])","tcp0 = np.array([x_rim + depth, y_ax, z_ax + dz])")
s=s.replace("""        pitch = float(sys.argv[5]) if len(sys.argv) > 5 else 60.0
        a, quat = scoop_frame(pitch)
        tcp0, pts = scoop_arc(x_r, y_a, z_a, pitch)""","""        pitch = float(sys.argv[5]) if len(sys.argv) > 5 else 60.0
        dz = float(sys.argv[6]) if len(sys.argv) > 6 else -0.01
        a, quat = scoop_frame(pitch)
        tcp0, pts = scoop_arc(x_r, y_a, z_a, pitch, dz=dz)""")
open("tools/right3.py","w").write(s)
EOF
python3 - <<'EOF' 2>&1 | grep -v "^ik:"
import sys, numpy as np, rclpy
sys.path.insert(0,"/workspace/tools")
from planner import Planner
from robot import mat_to_quat
rclpy.init(); p=Planner()
p.set_scene(microwave="solid", yellow=False)
R=np.array([[1.0,0,0],[0,-1.0,0],[0,0,-1.0]])
ok=p.goto_tcp(np.array([-0.165,-0.42,1.33]), mat_to_quat(R))
print("ok",ok,"tcp",np.round(p.tcp()[0],4))
EOF
cd /workspace/snaps && python3 /workspace/tools/cam_snap.py robot0_eye_in_hand --depth 2>&1 | tail -1
python3 - <<'EOF' 2>&1 | grep -v "^ik:"
import sys, numpy as np, rclpy
sys.path.insert(0,"/workspace/tools")
import eih
from robot import Robot
from PIL import Image
rclpy.init(); r=Robot()
pos,quat,R=r.fk()
d=np.load("snaps/robot0_eye_in_hand_depth.npy")
P=eih.cloud(d,pos,R).reshape(-1,3)
img=np.array(Image.open("snaps/robot0_eye_in_hand.png")).reshape(-1,3).astype(int)
yellow=(img[:,0]>150)&(img[:,1]>110)&(img[:,2]<120)
white=(img.min(1)>170)
near=(P[:,0]>-0.35)&(P[:,0]<0.05)&(P[:,1]>-0.55)&(P[:,1]<-0.33)&(P[:,2]>0.905)&(P[:,2]<1.10)
Q=P[yellow&near]; print("yellow n",len(Q),"bbox",np.round(Q.min(0),3),np.round(Q.max(0),3))
W=P[white&near]; print("white n",len(W),"bbox",np.round(W.min(0),3),np.round(W.max(0),3))
B=Q[Q[:,2]<0.995]
for x in np.arange(B[:,0].min(),B[:,0].max(),0.01):
    s=(B[:,0]>=x)&(B[:,0]<x+0.01)
    if s.sum()>5: print(f"x {x:.3f} y range {np.percentile(B[s,1],3):.3f}..{np.percentile(B[s,1],97):.3f} mid {(np.percentile(B[s,1],3)+np.percentile(B[s,1],97))/2:.4f} zmax {B[s,2].max():.3f}")
Hh=Q[Q[:,2]>1.02]; print("handle top n",len(Hh),"xy",np.round(Hh[:,:2].mean(0),4),"zmax",round(Hh[:,2].max(),4))
EOF

# openrua op 172
cd /workspace && python3 - <<'EOF' 2>&1 | grep -v "^ik:"
import sys, numpy as np, rclpy
sys.path.insert(0,"/workspace/tools")
import eih
from robot import Robot
from PIL import Image
rclpy.init(); r=Robot()
pos,quat,R=r.fk()
d=np.load("snaps/robot0_eye_in_hand_depth.npy")
P=eih.cloud(d,pos,R).reshape(-1,3)
img=np.array(Image.open("snaps/robot0_eye_in_hand.png")).reshape(-1,3).astype(int)
yellow=(img[:,0]>150)&(img[:,1]>110)&(img[:,2]<120)
white=(img.min(1)>170)
near=(P[:,0]>-0.35)&(P[:,0]<0.05)&(P[:,1]>-0.55)&(P[:,1]<-0.33)&(P[:,2]>0.905)&(P[:,2]<1.10)
Q=P[yellow&near]; print("yellow n",len(Q),"bbox",np.round(Q.min(0),3),np.round(Q.max(0),3))
W=P[white&near]; print("white n",len(W),"bbox",np.round(W.min(0),3),np.round(W.max(0),3))
B=Q[Q[:,2]<0.995]
for x in np.arange(B[:,0].min(),B[:,0].max(),0.01):
    s=(B[:,0]>=x)&(B[:,0]<x+0.01)
    if s.sum()>5: print(f"x {x:.3f} y range {np.percentile(B[s,1],3):.3f}..{np.percentile(B[s,1],97):.3f} mid {(np.percentile(B[s,1],3)+np.percentile(B[s,1],97))/2:.4f} zmax {B[s,2].max():.3f}")
Hh=Q[Q[:,2]>1.02]; print("handle top n",len(Hh),"xy",np.round(Hh[:,:2].mean(0),4),"zmax",round(Hh[:,2].max(),4))
EOF

# openrua op 173
python3 - <<'EOF' 2>&1 | grep -v "^ik:"
import sys, numpy as np, rclpy
sys.path.insert(0,"/workspace/tools")
import eih
from robot import Robot
from PIL import Image
rclpy.init(); r=Robot()
pos,quat,R=r.fk()
d=np.load("snaps/robot0_eye_in_hand_depth.npy")
P=eih.cloud(d,pos,R).reshape(-1,3)
img=np.array(Image.open("snaps/robot0_eye_in_hand.png")).reshape(-1,3).astype(int)
yellow=(img[:,0]>150)&(img[:,1]>110)&(img[:,2]<120)
near=(P[:,0]>-0.35)&(P[:,0]<0.05)&(P[:,1]>-0.55)&(P[:,1]<-0.33)&(P[:,2]>0.905)&(P[:,2]<1.10)
Q=P[yellow&near]
B=Q[Q[:,2]<0.995]
for x in np.arange(B[:,0].min(),B[:,0].max(),0.01):
    s=(B[:,0]>=x)&(B[:,0]<x+0.01)
    if s.sum()>5: print(f"x {x:.3f} y range {np.percentile(B[s,1],3):.3f}..{np.percentile(B[s,1],97):.3f} mid {(np.percentile(B[s,1],3)+np.percentile(B[s,1],97))/2:.4f} zmax {B[s,2].max():.3f}")
Hh=Q[Q[:,2]>1.02]; print("handle top n",len(Hh),"xy",np.round(Hh[:,:2].mean(0),4),"zmax",round(Hh[:,2].max(),4))
# any non-table points near the rim end (x<-0.21)
E=P[near&(P[:,0]<-0.21)&(P[:,2]>0.91)]; print("pts beyond x<-0.21:",len(E), "xmin", E[:,0].min() if len(E) else None)
EOF

# openrua op 174
python3 -u tools/right3.py scoopinfo -0.222 -0.426 0.95 75 -0.01 2>&1 | grep -v "^ik:"

# openrua op 175
python3 - <<'EOF' 2>&1 | grep -v "^ik:"
import sys, numpy as np, rclpy
sys.path.insert(0,"/workspace/tools")
from robot import Robot
rclpy.init(); r=Robot()
for name,q in [("tcp0",[-2.03,1.71,1.38,-1.94,-1.75,1.96,-2.43]),("mid",[-1.85,1.66,1.49,-1.8,-1.67,1.89,-2.4]),("end",[-1.62,1.68,1.51,-1.56,-1.66,1.89,-2.41])]:
    print(name)
    for l in ["panda_link2","panda_link3","panda_link4","panda_link5","panda_link6","panda_link7","panda_link8","panda_hand"]:
        pos=r.fk(q,l)[0]; print("  ",l,np.round(pos,3))
EOF

# openrua op 176
python3 - <<'EOF' 2>&1 | grep -v "^ik:"
import sys, numpy as np, rclpy
sys.path.insert(0,"/workspace/tools")
from planner import Planner
from right3 import scoop_frame, scoop_arc
rclpy.init(); p=Planner()
a,quat=scoop_frame(75)
tcp0,pts=scoop_arc(-0.222,-0.426,0.95,75,dz=-0.01)
rng=np.random.default_rng(0)
LIM=np.array(p.LIMITS) if hasattr(p,'LIMITS') else None
from robot import LIMITS
def links(q):
    return {l:p.fk(q,l)[0] for l in ["panda_link4","panda_link5","panda_link7"]}
found=[]
for i in range(25):
    seed=np.array([rng.uniform(lo+0.3,hi-0.3) for lo,hi in LIMITS])
    s=p.ik(tcp0,quat,seed=seed,attempts=1)
    if s is None: continue
    L=links(s)
    ok = L["panda_link5"][2]>1.22 and L["panda_link4"][2]>1.22
    print(np.round(s,2), "l4",np.round(L["panda_link4"],3),"l5",np.round(L["panda_link5"],3), "OK" if ok else "")
    if ok: found.append(s)
np.save("snaps/scoop_seeds.npy", np.array(found))
print("found",len(found))
EOF

# openrua op 177
python3 - <<'EOF'
s=open("tools/right3.py").read()
s=s.replace("""def scoop_frame(pitch_deg):
    pth = np.deg2rad(pitch_deg)
    a = np.array([np.cos(pth), 0, -np.sin(pth)])      # approach: toward +x and down
    yh = np.array([0, 1.0, 0])""","""def scoop_frame(pitch_deg, flip=True):
    pth = np.deg2rad(pitch_deg)
    a = np.array([np.cos(pth), 0, -np.sin(pth)])      # approach: toward +x and down
    yh = np.array([0, -1.0 if flip else 1.0, 0])      # flip: wrist offset toward +x (away from the door)""")
open("tools/right3.py","w").write(s)
EOF
python3 - <<'EOF' 2>&1 | grep -v "^ik:"
import sys, numpy as np, rclpy
sys.path.insert(0,"/workspace/tools")
from planner import Planner
from robot import LIMITS
from right3 import scoop_frame, scoop_arc
rclpy.init(); p=Planner()
a,quat=scoop_frame(75)
tcp0,pts=scoop_arc(-0.222,-0.426,0.95,75,dz=-0.01)
rng=np.random.default_rng(1)
def links(q):
    return {l:p.fk(q,l)[0] for l in ["panda_link3","panda_link4","panda_link5","panda_link7"]}
for i in range(12):
    seed=np.array([rng.uniform(lo+0.3,hi-0.3) for lo,hi in LIMITS])
    s=p.ik(tcp0,quat,seed=seed,attempts=1)
    if s is None: continue
    L=links(s)
    print(np.round(s,2), "l3",np.round(L["panda_link3"],3),"l4",np.round(L["panda_link4"],3),"l5",np.round(L["panda_link5"],3),"l7",np.round(L["panda_link7"],3))
EOF

# openrua op 178
python3 - <<'EOF' 2>&1 | grep -v "^ik:"
import sys, numpy as np, rclpy
sys.path.insert(0,"/workspace/tools")
from planner import Planner
from robot import LIMITS
from right3 import scoop_frame, scoop_arc
rclpy.init(); p=Planner()
tcp0,pts=scoop_arc(-0.222,-0.50,0.95,75,dz=-0.01)
rng=np.random.default_rng(2)
for flip in (True,False):
    a,quat=scoop_frame(75,flip)
    seen=[]
    for i in range(40):
        seed=np.array([rng.uniform(lo+0.3,hi-0.3) for lo,hi in LIMITS])
        s=p.ik(tcp0,quat,seed=seed,attempts=1)
        if s is None: continue
        if any(np.abs(s-t).max()<0.2 for t in seen): continue
        seen.append(s)
        l5=p.fk(s,"panda_link5")[0]; l7=p.fk(s,"panda_link7")[0]; l4=p.fk(s,"panda_link4")[0]
        print("flip",flip,np.round(s,2),"d",np.round((l5-l7)/0.088,2),"l4",np.round(l4,3),"l5",np.round(l5,3))
EOF

# openrua op 179
python3 -u tools/right3.py pinch -0.166 -0.43 1.026 0 2>&1 | grep -v "^ik:" | grep -E "gap|ok" && python3 -u tools/right3.py carry 0 -0.124 -0.474 2>&1 | grep -v "^ik:" | grep -E "gap|ok|fraction"

# openrua op 180
cat > tools/measure_lying.py <<'EOF'
#!/usr/bin/env python3
"""Move the wrist camera above (x, y) and fit the lying mug (axis along x): prints rim x, axis y, handle bar centre."""
import sys, subprocess, numpy as np, rclpy
sys.path.insert(0, "/workspace/tools")
import eih
from planner import Planner
from robot import mat_to_quat
from PIL import Image
x, y = float(sys.argv[1]), float(sys.argv[2])
rclpy.init(); p = Planner()
p.set_scene(microwave="solid", yellow=False)
R = np.array([[1.0, 0, 0], [0, -1.0, 0], [0, 0, -1.0]])
ok = p.goto_tcp(np.array([x, y, 1.33]), mat_to_quat(R))
subprocess.run("cd /workspace/snaps && python3 /workspace/tools/cam_snap.py robot0_eye_in_hand --depth >/dev/null", shell=True)
pos, quat, Rh = p.fk()
d = np.load("/workspace/snaps/robot0_eye_in_hand_depth.npy")
P = eih.cloud(d, pos, Rh).reshape(-1, 3)
img = np.array(Image.open("/workspace/snaps/robot0_eye_in_hand.png")).reshape(-1, 3).astype(int)
yellow = (img[:, 0] > 150) & (img[:, 1] > 110) & (img[:, 2] < 120)
near = (P[:, 0] > x - 0.15) & (P[:, 0] < x + 0.15) & (P[:, 1] > y - 0.12) & (P[:, 1] < y + 0.12) & (P[:, 2] > 0.905) & (P[:, 2] < 1.10)
Q = P[yellow & near]
print("yellow n", len(Q), "bbox", np.round(Q.min(0), 4), np.round(Q.max(0), 4))
B = Q[Q[:, 2] < 0.995]
for xx in np.arange(B[:, 0].min(), B[:, 0].max(), 0.01):
    s = (B[:, 0] >= xx) & (B[:, 0] < xx + 0.01)
    if s.sum() > 5:
        lo, hi = np.percentile(B[s, 1], 3), np.percentile(B[s, 1], 97)
        print(f"x {xx:.3f} y {lo:.3f}..{hi:.3f} mid {(lo+hi)/2:.4f} w {hi-lo:.3f} zmax {B[s, 2].max():.3f}")
Hh = Q[Q[:, 2] > 1.02]
print("handle top n", len(Hh), "xy", np.round(Hh[:, :2].mean(0), 4), "zmax", round(Hh[:, 2].max(), 4))
p.destroy_node(); rclpy.shutdown()
EOF
python3 -u tools/measure_lying.py -0.125 -0.47 2>&1 | grep -v "^ik:" | grep -v "exec done"

# openrua op 181
python3 - <<'EOF'
s=open("tools/right3.py").read()
s=s.replace("""        else:
            p.grip(False)
            ok = p.goto_tcp(pre, quat, time_s=8.0)
            print("pre ok", ok, "tcp", np.round(p.tcp()[0], 4))
            if not ok:
                p.destroy_node(); rclpy.shutdown(); return
""","""        else:
            p.grip(False)
            q_pre = pick_branch(p, pre, quat)
            if q_pre is None:
                print("no suitable IK branch"); p.destroy_node(); rclpy.shutdown(); return
            ok = p.goto_q(q_pre, time_s=8.0)
            print("pre ok", ok, "tcp", np.round(p.tcp()[0], 4), "q", np.round(p.arm_q(), 2))
            if not ok:
                p.destroy_node(); rclpy.shutdown(); return
""")
s=s.replace("""def main():
    stage = sys.argv[1]""","""def pick_branch(p, pos, quat, tries=40):
    \"\"\"IK solution whose wrist (link5) sits on the +y side of the hand and elbow (link4) is high.\"\"\"
    from robot import LIMITS
    rng = np.random.default_rng(0)
    best = None
    for _ in range(tries):
        seed = np.array([rng.uniform(lo + 0.3, hi - 0.3) for lo, hi in LIMITS])
        s = p.ik(pos, quat, seed=seed, attempts=1)
        if s is None:
            continue
        l4 = p.fk(s, "panda_link4")[0]; l5 = p.fk(s, "panda_link5")[0]; l7 = p.fk(s, "panda_link7")[0]
        d = (l5 - l7) / 0.088
        if d[1] > 0.9 and l4[2] > 1.30:
            print("branch", np.round(s, 2), "d", np.round(d, 2), "l4", np.round(l4, 3), "l5", np.round(l5, 3))
            if best is None or l4[2] > best[1]:
                best = (s, l4[2])
    return None if best is None else best[0]


def main():
    stage = sys.argv[1]""")
open("tools/right3.py","w").write(s)
EOF
grep -n "def goto_q" -A6 tools/planner.py | head -12

# openrua op 182
python3 -u tools/right3.py scoopinfo -0.179 -0.4754 0.95 75 -0.01 2>&1 | grep -v "^ik:"

# openrua op 183
python3 - <<'EOF'
s=open("tools/right3.py").read()
s=s.replace('''    elif stage in ("scoopinfo", "scoop"):''','''    elif stage == "hinge":
        # TD pinch of the top bar acts as a friction hinge (weak axis = world y): move the TCP along
        # an arc about the base edge on the table so the mug pivots upright.  args: x_pivot sign(+1/-1)
        x_piv, sgn = float(sys.argv[2]), float(sys.argv[3])       # sgn -1: rim at +x, mug tips toward -x
        tcp, quat, R = p.tcp()
        C = np.array([x_piv, 0.90])
        v0 = np.array([tcp[0] - C[0], tcp[2] - C[1]]); ang0 = np.arctan2(v0[1], v0[0]); rad = np.linalg.norm(v0)
        print("arc radius", round(rad, 4), "ang0", round(np.rad2deg(ang0), 1))
        for k in range(1, 10):
            ang = ang0 - sgn * np.deg2rad(10.0 * k)
            pt = np.array([C[0] + rad * np.cos(ang), tcp[1], C[1] + rad * np.sin(ang)])
            ok = p.line(pt, quat, step=0.004, avoid=False, min_fraction=0.9, time_scale=5)
            print(f"arc {k} target {np.round(pt, 3)} ok {ok} tcp {np.round(p.tcp()[0], 4)} gap {p.finger_gap():.4f}", flush=True)
            if not ok:
                break
        p.grip(True)
        t2 = p.tcp()[0]
        p.line(t2 + np.array([0, 0, 0.10]), quat, step=0.01, avoid=False, min_fraction=0.9)
        print("retreated; tcp", np.round(p.tcp()[0], 4))
    elif stage in ("scoopinfo", "scoop"):''')
open("tools/right3.py","w").write(s)
EOF
python3 -u tools/right3.py pinch -0.1222 -0.481 1.026 0 2>&1 | grep -v "^ik:" | grep -E "gap|ok"

# openrua op 184
python3 -u tools/right3.py carry -180 -0.06 -0.478 2>&1 | grep -v "^ik:" | grep -E "gap|ok|fraction"

# openrua op 185
python3 -u tools/measure_lying.py -0.06 -0.478 2>&1 | grep -v "^ik:" | grep -v "exec done"

# openrua op 186
python3 -u tools/right3.py pinch -0.060 -0.4675 1.026 0 2>&1 | grep -v "^ik:" | grep -E "gap|ok" && python3 -u tools/right3.py hinge -0.110 -1 2>&1 | grep -v "^ik:" | grep -vE "^exec done|^tcp now"

# openrua op 187
cd /workspace/snaps && python3 /workspace/tools/cam_snap.py birdview robot0_eye_in_hand --depth 2>&1 | tail -1; python3 - <<'EOF'
import sys, numpy as np
sys.path.insert(0,"/workspace/tools")
import cams
d=np.load("snaps/birdview_depth.npy")
P=cams.cloud("birdview", d).reshape(-1,3)
m=(P[:,0]>-0.30)&(P[:,0]<0.02)&(P[:,1]>-0.56)&(P[:,1]<-0.38)&(P[:,2]>0.905)&(P[:,2]<1.09)
Q=P[m]
xs=np.arange(-0.30,0.02,0.01); ys=np.arange(-0.56,-0.38,0.01)
H=np.full((len(xs),len(ys)),np.nan)
for q in Q:
    i=int((q[0]+0.30)/0.01); j=int((q[1]+0.56)/0.01)
    if 0<=i<len(xs) and 0<=j<len(ys): H[i,j]=np.nanmax([H[i,j],q[2]])
print("cols y from -0.56 to -0.39")
for i,x in enumerate(xs):
    print(f"{x:6.2f} "+" ".join("  ." if np.isnan(v) else f"{int((v-0.9)*100):3d}" for v in H[i]))
EOF

# openrua op 188
python3 - <<'EOF'
import sys, numpy as np
sys.path.insert(0,"/workspace/tools")
import cams
d=np.load("snaps/birdview_depth.npy")
P=cams.cloud("birdview", d).reshape(-1,3)
m=(P[:,0]>-0.30)&(P[:,0]<0.02)&(P[:,1]>-0.56)&(P[:,1]<-0.38)&(P[:,2]>0.905)&(P[:,2]<1.09)
Q=P[m]
xs=np.arange(-0.30,0.02,0.01); ys=np.arange(-0.56,-0.38,0.01)
H=np.full((len(xs),len(ys)),np.nan)
for q in Q:
    i=int((q[0]+0.30)/0.01); j=int((q[1]+0.56)/0.01)
    if 0<=i<len(xs) and 0<=j<len(ys): H[i,j]=np.nanmax([H[i,j],q[2]])
print("cols y from -0.56 to -0.39")
for i,x in enumerate(xs):
    print(f"{x:6.2f} "+" ".join("  ." if np.isnan(v) else f"{int((v-0.9)*100):3d}" for v in H[i]))
EOF

# openrua op 189
timeout 300 python3 -u tools/measure_lying.py -0.16 -0.47 2>&1 | tail -40

# openrua op 190
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/tools/right3.py", "old_string": "    elif stage in (\\"scoopinfo\\", \\"scoop\\"):", "new_string": "    elif stage == \\"hinge2\\":\\n        # like hinge, but the commanded radius is shortened by delta so the hand presses the mug\\n        # toward the pivot (base edge into the table) instead of dragging it.  args: x_pivot sgn delta\\n        x_piv, sgn, delta = float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])\\n        step_deg = float(sys.argv[5]) if len(sys.argv) > 5 else 10.0\\n        tcp, quat, R = p.tcp()\\n        C = np.array([x_piv, 0.90])\\n        v0 = np.array([tcp[0] - C[0], tcp[2] - C[1]]); ang0 = np.arctan2(v0[1], v0[0]); rad = np.linalg.norm(v0)\\n        print(\\"arc radius\\", round(rad, 4), \\"ang0\\", round(np.rad2deg(ang0), 1), \\"delta\\", delta)\\n        lag = 0\\n        k = 1\\n        while np.rad2deg(ang0) - sgn * step_deg * k <= 180.0 + 1e-6 and k * step_deg <= 90.0 + 1e-6:\\n            ang = ang0 - sgn * np.deg2rad(step_deg * k)\\n            pt = np.array([C[0] + (rad - delta) * np.cos(ang), tcp[1], C[1] + (rad - delta) * np.sin(ang)])\\n            ok = p.line(pt, quat, step=0.004, avoid=False, min_fraction=0.9, time_scale=5)\\n            now = p.tcp()[0]\\n            err = np.linalg.norm(now - pt)\\n            print(f\\"arc {k} target {np.round(pt, 3)} ok {ok} tcp {np.round(now, 4)} err {err:.4f} gap {p.finger_gap():.4f}\\", flush=True)\\n            lag = lag + 1 if err > 0.015 else 0\\n            if lag >= 2:\\n                print(\\"stalled\\"); break\\n            k += 1\\n        p.grip(True)\\n        t2 = p.tcp()[0]\\n        p.line(t2 + np.array([0, 0, 0.10]), quat, step=0.01, avoid=False, min_fraction=0.9)\\n        print(\\"retreated; tcp\\", np.round(p.tcp()[0], 4))\\n    elif stage in (\\"scoopinfo\\", \\"scoop\\"):", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 191
timeout 600 python3 -u tools/right3.py pinch -0.158 -0.457 1.026 0 2>&1 | grep -v "^exec\|^traj\|^ik:" | tail -8

# openrua op 192
cd /workspace/snaps && timeout 120 python3 /workspace/tools/cam_snap.py robot0_eye_in_hand birdview --depth 2>&1 | tail -2

# openrua op 193
cd /workspace && timeout 600 python3 -u - <<'EOF' 2>&1 | grep -v "^exec\|^traj\|^ik:"
import sys, numpy as np, rclpy
sys.path.insert(0,"/workspace/tools")
from planner import Planner
from task import HOME_Q
rclpy.init(); p=Planner(); p.set_scene(microwave="solid", yellow=False)
print("q", np.round(p.arm_q(),2)); 
for l in ["panda_link3","panda_link4","panda_link5","panda_link6","panda_link7"]:
    print(l, np.round(p.fk(None,l)[0],3))
p.grip(True)
tcp,quat,R=p.tcp()
ok=p.line(tcp+np.array([0,0,0.12]), quat, step=0.01, avoid=False, min_fraction=0.9, time_scale=3)
print("up ok", ok, np.round(p.tcp()[0],4))
ok=p.goto_q(np.array(HOME_Q), time_s=12.0)
print("home ok", ok, np.round(p.tcp()[0],4))
p.destroy_node(); rclpy.shutdown()
EOF

# openrua op 194
mkdir -p "$(dirname /workspace/tools/heightmap.py)"
cat > /workspace/tools/heightmap.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Birdview height map (cm above the table) of a region.
  python3 tools/heightmap.py x0 x1 y0 y1 [cell_m]"""
import sys, subprocess, os
import numpy as np
sys.path.insert(0, "/workspace/tools")
import cams

x0, x1, y0, y1 = map(float, sys.argv[1:5])
cell = float(sys.argv[5]) if len(sys.argv) > 5 else 0.01
os.makedirs("/workspace/snaps", exist_ok=True)
subprocess.run(["python3", "/workspace/tools/cam_snap.py", "birdview", "--depth"], cwd="/workspace/snaps",
               check=True, stdout=subprocess.DEVNULL)
depth = np.load("/workspace/snaps/birdview_depth.npy")
pts = cams.cloud("birdview", depth).reshape(-1, 3)
m = (pts[:, 0] >= x0) & (pts[:, 0] < x1) & (pts[:, 1] >= y0) & (pts[:, 1] < y1) & (pts[:, 2] < 1.3)
pts = pts[m]
nx, ny = int(round((x1 - x0) / cell)), int(round((y1 - y0) / cell))
H = np.full((nx, ny), -1.0)
ix = ((pts[:, 0] - x0) / cell).astype(int); iy = ((pts[:, 1] - y0) / cell).astype(int)
for i, j, z in zip(ix, iy, pts[:, 2]):
    H[i, j] = max(H[i, j], z)
print("rows x from %.3f, cols y from %.3f, cell %.3f; values cm above table" % (x0, y0, cell))
print("      " + " ".join("%3d" % round((y0 + (j + 0.5) * cell) * 100) for j in range(ny)))
for i in range(nx):
    row = " ".join("%3d" % round((H[i, j] - 0.90) * 100) if H[i, j] > 0 else "  ." for j in range(ny))
    print("%6.3f " % (x0 + (i + 0.5) * cell) + row)
OPENRUA_EOF

# openrua op 195
timeout 120 python3 tools/heightmap.py -0.33 -0.05 -0.62 -0.38 0.01 2>&1 | tail -32

# openrua op 196
timeout 120 python3 tools/heightmap.py -0.24 -0.10 -0.74 -0.50 0.01 2>&1 | tail -16

# openrua op 197
cd /workspace/snaps && timeout 120 python3 /workspace/tools/cam_snap.py frontview galleryview paperview sideview 2>&1 | tail -4

# openrua op 198
mkdir -p "$(dirname /workspace/tools/look.py)"
cat > /workspace/tools/look.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Move the wrist camera to (x, y, z) looking straight down, snap, and print a height map of
all non-table points in the view (cm above table) plus colour tags (Y yellow / W white / . other).
  python3 tools/look.py x y z [x0 x1 y0 y1] [cell]"""
import sys, subprocess, numpy as np, rclpy
sys.path.insert(0, "/workspace/tools")
import eih
from planner import Planner
from robot import mat_to_quat
from PIL import Image

x, y, z = map(float, sys.argv[1:4])
rclpy.init(); p = Planner()
p.set_scene(microwave="solid", yellow=False)
R = np.array([[1.0, 0, 0], [0, -1.0, 0], [0, 0, -1.0]])
ok = p.goto_tcp(np.array([x, y, z]), mat_to_quat(R))
print("camera move ok", ok, "tcp", np.round(p.tcp()[0], 4))
subprocess.run("cd /workspace/snaps && python3 /workspace/tools/cam_snap.py robot0_eye_in_hand --depth >/dev/null", shell=True)
pos, quat, Rh = p.fk()
d = np.load("/workspace/snaps/robot0_eye_in_hand_depth.npy")
P = eih.cloud(d, pos, Rh).reshape(-1, 3)
img = np.array(Image.open("/workspace/snaps/robot0_eye_in_hand.png")).reshape(-1, 3).astype(int)
yellow = (img[:, 0] > 150) & (img[:, 1] > 110) & (img[:, 2] < 120)
white = (img[:, 0] > 150) & (img[:, 1] > 150) & (img[:, 2] > 150)
if len(sys.argv) > 7:
    x0, x1, y0, y1 = map(float, sys.argv[4:8])
else:
    x0, x1, y0, y1 = x - 0.15, x + 0.15, y - 0.15, y + 0.15
cell = float(sys.argv[8]) if len(sys.argv) > 8 else 0.01
m = (P[:, 0] >= x0) & (P[:, 0] < x1) & (P[:, 1] >= y0) & (P[:, 1] < y1) & (P[:, 2] > 0.905) & (P[:, 2] < 1.10)
nx, ny = int(round((x1 - x0) / cell)), int(round((y1 - y0) / cell))
H = np.full((nx, ny), -1.0); T = np.full((nx, ny), ".", dtype=object)
for (px, py, pz), yl, wh in zip(P[m], yellow[m], white[m]):
    i, j = int((px - x0) / cell), int((py - y0) / cell)
    if pz > H[i, j]:
        H[i, j] = pz; T[i, j] = "Y" if yl else ("W" if wh else "o")
print("rows x from %.3f, cols y from %.3f; height cm + tag" % (x0, y0))
print("       " + " ".join("%4d" % round((y0 + (j + 0.5) * cell) * 100) for j in range(ny)))
for i in range(nx):
    print("%6.3f " % (x0 + (i + 0.5) * cell) + " ".join(
        ("%3d%s" % (round((H[i, j] - 0.90) * 100), T[i, j])) if H[i, j] > 0 else "   ." for j in range(ny)))
Q = P[m & yellow]
if len(Q):
    print("yellow bbox", np.round(Q.min(0), 4), np.round(Q.max(0), 4), "n", len(Q))
Q = P[m & white]
if len(Q):
    print("white bbox", np.round(Q.min(0), 4), np.round(Q.max(0), 4), "n", len(Q))
p.destroy_node(); rclpy.shutdown()
OPENRUA_EOF

# openrua op 199
timeout 300 python3 -u tools/look.py -0.14 -0.55 1.17 -0.26 -0.08 -0.70 -0.50 0.01 2>&1 | grep -v "^exec\|^traj\|^ik:" | tail -28

# openrua op 200
cd /workspace && timeout 300 python3 -u tools/look.py -0.14 -0.55 1.17 -0.26 -0.08 -0.70 -0.50 0.01 2>&1 | grep -v "^exec\|^traj\|^ik:" | tail -28
