#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
cat /workspace/tools/action/fjt_send.py; echo ----; cat /workspace/tools/action/gripper_cmd.py; echo ----; cat /workspace/tools/action/ik_move.py

# openrua op 3
ros2 topic list; echo ----; ros2 action list; echo ----; ros2 service list | head -40; echo ----; ros2 node list

# openrua op 4
mkdir -p snaps && for c in agentview birdview frontview sideview galleryview paperview robot0_robotview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c snaps/$c.png & done; wait; ls -la snaps; ros2 topic echo /joint_states --once

# openrua op 5
python3 -c "
import cv2
im = cv2.imread('snaps/agentview.png')
crop = im[190:420, 100:400]
cv2.imwrite('snaps/agent_crop.png', cv2.resize(crop, None, fx=3, fy=3, interpolation=cv2.INTER_CUBIC))
"

# openrua op 6
timeout 20 ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id|child_frame_id" ; echo ---; timeout 20 ros2 topic echo /tf --once 2>&1 | grep -E "frame_id|child_frame_id" | sort | uniq -c

# openrua op 7
timeout 20 ros2 topic echo /tf --once 2>&1 | grep -B2 -A12 "child_frame_id: panda_link0\|child_frame_id: birdview_optical_frame\|child_frame_id: agentview_optical_frame"; ros2 topic echo /birdview/color/camera_info --once | grep -A10 "^k:"

# openrua op 8
timeout 30 ros2 topic echo /tf --once 2>&1 > /tmp/tf.txt; grep -c transforms /tmp/tf.txt; python3 - <<'EOF'
import yaml
d = yaml.safe_load(open('/tmp/tf.txt').read().split('---')[0])
for t in d['transforms']:
    tr = t['transform']['translation']; r = t['transform']['rotation']
    print(f"{t['child_frame_id']:35s} t=({tr['x']:.4f},{tr['y']:.4f},{tr['z']:.4f}) q=({r['x']:.4f},{r['y']:.4f},{r['z']:.4f},{r['w']:.4f})")
EOF

# openrua op 9
mkdir -p "$(dirname /workspace/locate.py)"
cat > /workspace/locate.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Segment objects above the table from the birdview depth+color and print
world-frame centroids, heights and footprints."""
import sys
import numpy as np
import rclpy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage

CAM = sys.argv[1] if len(sys.argv) > 1 else "birdview"


def grab(node, topic, typ):
    got = {}
    sub = node.create_subscription(typ, topic, lambda m: got.setdefault("m", m), 1)
    while "m" not in got:
        rclpy.spin_once(node, timeout_sec=0.5)
    node.destroy_subscription(sub)
    return got["m"]


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main():
    rclpy.init()
    node = rclpy.create_node("locate")
    depth = CvBridge().imgmsg_to_cv2(grab(node, f"/{CAM}/depth/image_raw", Image), "passthrough").astype(np.float64)
    color = CvBridge().imgmsg_to_cv2(grab(node, f"/{CAM}/color/image_raw", Image), "bgr8")
    info = grab(node, f"/{CAM}/color/camera_info", CameraInfo)
    tf = None
    while tf is None:
        m = grab(node, "/tf", TFMessage)
        for t in m.transforms:
            if t.child_frame_id == f"{CAM}_optical_frame":
                tf = t
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    H, W = depth.shape
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    X = (u - cx) * depth / fx
    Y = (v - cy) * depth / fy
    P = np.stack([X, Y, depth], -1).reshape(-1, 3)
    q = tf.transform.rotation
    R = quat_R(q.x, q.y, q.z, q.w)
    tr = np.array([tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z])
    Pw = (R @ P.T).T + tr
    Pw = Pw.reshape(H, W, 3)
    np.save(f"snaps/{CAM}_world.npy", Pw)
    z = Pw[..., 2]
    # table height estimate: mode of z over central region
    zz = z[np.isfinite(z)]
    hist, edges = np.histogram(zz, bins=400, range=(0, 2))
    table_z = edges[np.argmax(hist)]
    print(f"table_z ~ {table_z:.3f}")
    mask = (z > table_z + 0.01) & (z < table_z + 0.5) & np.isfinite(z)
    # exclude robot: anything near base x<-0.2 ... print all blobs anyway
    n, lab, stats, cents = cv2.connectedComponentsWithStats(mask.astype(np.uint8), 8)
    for i in range(1, n):
        if stats[i, cv2.CC_STAT_AREA] < 15:
            continue
        sel = lab == i
        pts = Pw[sel]
        col = color[sel].mean(0)
        x0, y0, x1, y1 = pts[:, 0].min(), pts[:, 1].min(), pts[:, 0].max(), pts[:, 1].max()
        print(f"blob {i}: px({cents[i][0]:.0f},{cents[i][1]:.0f}) area={stats[i, cv2.CC_STAT_AREA]} "
              f"centroid=({pts[:, 0].mean():.3f},{pts[:, 1].mean():.3f}) top_z={pts[:, 2].max():.3f} "
              f"xrange=[{x0:.3f},{x1:.3f}] yrange=[{y0:.3f},{y1:.3f}] bgr={col.astype(int)}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 10
timeout 120 python3 locate.py birdview

# openrua op 11
python3 - <<'EOF'
import numpy as np, cv2
Pw = np.load('snaps/birdview_world.npy'); z = Pw[...,2]
zz = z[np.isfinite(z)]
hist, edges = np.histogram(zz, bins=100, range=(0.3, 0.8))
for h,e in zip(hist,edges):
    if h>20: print(f"{e:.3f} {h}")
EOF

# openrua op 12
python3 - <<'EOF'
import numpy as np, cv2
Pw = np.load('snaps/birdview_world.npy'); z = Pw[...,2]
color = cv2.imread('snaps/birdview.png')
table_z = 0.42
mask = (z > table_z + 0.012) & (z < table_z + 0.35) & np.isfinite(z)
n, lab, stats, cents = cv2.connectedComponentsWithStats(mask.astype(np.uint8), 8)
for i in range(1, n):
    if stats[i, cv2.CC_STAT_AREA] < 6: continue
    sel = lab == i; pts = Pw[sel]; col = color[sel].mean(0)
    print(f"blob {i}: px({cents[i][0]:.0f},{cents[i][1]:.0f}) area={stats[i,4]} centroid=({pts[:,0].mean():.3f},{pts[:,1].mean():.3f}) top_z={pts[:,2].max():.3f} x=[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y=[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] bgr={col.astype(int)}")
EOF

# openrua op 13
timeout 120 python3 locate.py agentview > /dev/null; python3 - <<'EOF'
import numpy as np, cv2
Pw = np.load('snaps/agentview_world.npy')
for name,(u,v) in {"blue can":(215,240),"orange can":(343,245),"ketchup":(258,330),"cream cheese":(145,378),"basket rim":(540,262)}.items():
    p = Pw[v,u]; print(f"{name:14s} px({u},{v}) -> world ({p[0]:.3f},{p[1]:.3f},{p[2]:.3f})")
EOF

# openrua op 14
mkdir -p "$(dirname /workspace/robot.py)"
cat > /workspace/robot.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Small helper library for this Panda: joint state, FK, IK, trajectory,
gripper. Clients are built once per Robot instance.

World <-> base: panda_link0 sits at world (-0.51, 0, 0.42), identity rotation.
"""
import time
import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from trajectory_msgs.msg import JointTrajectoryPoint

BASE_IN_WORLD = np.array([-0.51, 0.0, 0.42])
TCP_OFF = 0.1034


def w2b(p):
    return np.asarray(p, float) - BASE_IN_WORLD


def b2w(p):
    return np.asarray(p, float) + BASE_IN_WORLD


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class Robot:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("robot_helper")
        M = yaml.safe_load(open("/workspace/machine.yaml"))
        self.fjt_entry = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
        self.grip_entry = next(a for a in M["actuators"] if a["kind"] == "gripper")
        self.joints = self.fjt_entry["joints"]
        self._js = None
        self._wr = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 1)
        self.node.create_subscription(WrenchStamped, "/franka_robot_state_broadcaster/external_wrench", self._on_wr, 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, self.fjt_entry["port"])
        self.grip = ActionClient(self.node, GripperCommand, self.grip_entry["port"])
        self.ik = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        assert self.fjt.wait_for_server(10), "no fjt server"
        assert self.grip.wait_for_server(10), "no gripper server"
        assert self.ik.wait_for_service(10), "no ik"
        assert self.fk.wait_for_service(10), "no fk"

    def _on_js(self, m):
        self._js = m

    def _on_wr(self, m):
        self._wr = m

    def spin(self, t=0.1):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joint_state(self, fresh=True):
        if fresh:
            self._js = None
        while self._js is None:
            self.spin(0.2)
        return dict(zip(self._js.name, self._js.position))

    def arm_q(self):
        js = self.joint_state()
        return [js[j] for j in self.joints]

    def fingers(self):
        js = self.joint_state()
        return js["panda_finger_joint1"], js["panda_finger_joint2"]

    def wrench(self):
        self._wr = None
        while self._wr is None:
            self.spin(0.2)
        f = self._wr.wrench.force
        return np.array([f.x, f.y, f.z])

    def _seed(self, q=None):
        q = q if q is not None else self.arm_q()
        s = JointState()
        s.name = list(self.joints)
        s.position = [float(v) for v in q]
        return s

    def fk_hand(self, q=None):
        """Hand pose in BASE frame: (pos[3], quat[4])."""
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state = self._seed(q)
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        r = fut.result()
        assert r is not None and r.error_code.val == 1, f"FK failed {r}"
        p = r.pose_stamped[0].pose
        return (np.array([p.position.x, p.position.y, p.position.z]),
                np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]))

    def tcp_world(self, q=None):
        p, quat = self.fk_hand(q)
        R = quat_R(*quat)
        return b2w(p + TCP_OFF * R[:, 2])

    def ik_tcp_world(self, pw, quat, seed=None, tries=5):
        """IK for TCP at world pos pw with hand orientation quat. Returns joint list or None."""
        R = quat_R(*quat)
        pb = w2b(pw) - TCP_OFF * R[:, 2]
        for k in range(tries):
            req = GetPositionIK.Request()
            req.ik_request.group_name = "panda_arm"
            req.ik_request.pose_stamped.header.frame_id = ""
            po = req.ik_request.pose_stamped.pose
            po.position.x, po.position.y, po.position.z = map(float, pb)
            po.orientation.x, po.orientation.y, po.orientation.z, po.orientation.w = map(float, quat)
            req.ik_request.robot_state.joint_state = self._seed(seed)
            req.ik_request.avoid_collisions = False
            req.ik_request.timeout.sec = 1
            fut = self.ik.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            r = fut.result()
            if r is not None and r.error_code.val == 1:
                sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
                return [sol[j] for j in self.joints]
            print(f"  ik try {k} failed: {None if r is None else r.error_code.val}")
        return None

    def move_joints(self, q, seconds=3.0, via=None):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(self.joints)
        pts = []
        if via:
            n = len(via) + 1
            for i, v in enumerate(via):
                t = seconds * (i + 1) / n
                p = JointTrajectoryPoint(positions=[float(x) for x in v])
                p.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
                pts.append(p)
        p = JointTrajectoryPoint(positions=[float(x) for x in q])
        p.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        pts.append(p)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res)
        code = res.result().result.error_code
        err = np.abs(np.array(self.arm_q()) - np.array(q)).max()
        print(f"  move done code={code} max_joint_err={err:.4f}")
        return code, err

    def move_tcp(self, pw, quat, seconds=3.0, seed=None):
        q = self.ik_tcp_world(pw, quat, seed=seed)
        if q is None:
            print(f"  IK FAILED for {pw}")
            return None
        self.move_joints(q, seconds)
        got = self.tcp_world()
        print(f"  tcp now world={np.round(got, 4)} target={np.round(pw, 4)} err={np.linalg.norm(got - pw):.4f}")
        return q

    def gripper(self, width):
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = float(self.grip_entry.get("max_effort", 30.0))
        fut = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=120)
        r = rf.result().result
        f = self.fingers()
        print(f"  gripper -> reached={r.reached_goal} stalled={r.stalled} fingers={f[0]:.4f},{f[1]:.4f}")
        return f


# Hand orientations (quaternions x,y,z,w) in base frame, hand Z pointing down.
Q_DOWN_Y = np.array([1.0, 0.0, 0.0, 0.0])          # fingers open along base Y
Q_DOWN_X = np.array([0.7071068, 0.7071068, 0.0, 0.0])  # fingers open along base X
OPENRUA_EOF

# openrua op 15
timeout 120 python3 -c "
from robot import *
r = Robot()
p,q = r.fk_hand()
print('hand base pos', p.round(4), 'quat', q.round(4))
R = quat_R(*q); print('hand axes in base:\n', R.round(3))
print('tcp world', r.tcp_world().round(4))
print('fingers', r.fingers())
print('wrench', r.wrench())
"

# openrua op 16
timeout 120 python3 -c "
from robot import *
r = Robot()
for name,q in [('current', r.arm_q()), ('zeros',[0]*7), ('ready',[0,-0.785,0,-2.356,0,1.571,0.785])]:
    p,quat = r.fk_hand(q); print(name, p.round(4), quat.round(4))
"

# openrua op 17
timeout 120 python3 -c "
from robot import *
import numpy as np
r = Robot()
q0 = np.array(r.arm_q())
p,quat = r.fk_hand()
# IK using raw pose as returned by FK (assume same frame)
r.BASE = None
req_p = p
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
def ik_raw(pb, quat):
    req = GetPositionIK.Request()
    req.ik_request.group_name='panda_arm'; req.ik_request.pose_stamped.header.frame_id=''
    po = req.ik_request.pose_stamped.pose
    po.position.x,po.position.y,po.position.z = map(float,pb)
    po.orientation.x,po.orientation.y,po.orientation.z,po.orientation.w = map(float,quat)
    req.ik_request.robot_state.joint_state = r._seed(); req.ik_request.timeout.sec=1
    fut = r.ik.call_async(req); rclpy.spin_until_future_complete(r.node, fut, timeout_sec=60)
    res = fut.result(); 
    if res.error_code.val!=1: return None
    sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position)); return np.array([sol[j] for j in r.joints])
s1 = ik_raw(p, quat); print('IK(world pose) ->', None if s1 is None else np.round(s1-q0,4))
s2 = ik_raw(p - BASE_IN_WORLD, quat); print('IK(base pose) ->', None if s2 is None else np.round(s2-q0,4))
"

# openrua op 18
python3 - <<'EOF'
import re
s = open('robot.py').read()
s = s.replace('''        R = quat_R(*quat)
        pb = w2b(pw) - TCP_OFF * R[:, 2]
        for k in range(tries):
            req = GetPositionIK.Request()
            req.ik_request.group_name = "panda_arm"
            req.ik_request.pose_stamped.header.frame_id = ""''',
'''        # machine fact (measured): FK/IK model frame is WORLD here, and the
        # default IK tip is panda_link8, so name the hand link explicitly.
        R = quat_R(*quat)
        pb = np.asarray(pw, float) - TCP_OFF * R[:, 2]
        for k in range(tries):
            req = GetPositionIK.Request()
            req.ik_request.group_name = "panda_arm"
            req.ik_request.ik_link_name = "panda_hand"
            req.ik_request.pose_stamped.header.frame_id = ""''')
s = s.replace('''    def fk_hand(self, q=None):
        """Hand pose in BASE frame: (pos[3], quat[4])."""''','''    def fk_hand(self, q=None):
        """Hand pose in WORLD frame (measured: FK output frame is world): (pos[3], quat[4])."""''')
s = s.replace('''        R = quat_R(*quat)
        return b2w(p + TCP_OFF * R[:, 2])''','''        R = quat_R(*quat)
        return p + TCP_OFF * R[:, 2]''')
open('robot.py','w').write(s)
EOF
timeout 120 python3 -c "
from robot import *
import numpy as np
r = Robot()
q0 = np.array(r.arm_q()); p,quat = r.fk_hand()
tcp = r.tcp_world(); print('tcp world', tcp.round(4))
s = r.ik_tcp_world(tcp, quat); print('roundtrip dq', np.round(np.array(s)-q0,4))
# test target poses
for name,pw,Q in [('above soup',(-0.186,-0.165,0.60),Q_DOWN_Y),('soup grasp',(-0.186,-0.165,0.465),Q_DOWN_Y),('above cheese',(0.097,-0.209,0.60),Q_DOWN_Y),('cheese grasp',(0.097,-0.209,0.435),Q_DOWN_Y),('above basket',(0.0,0.25,0.75),Q_DOWN_Y)]:
    s = r.ik_tcp_world(np.array(pw), Q)
    if s is None: print(name,'IK FAIL'); continue
    print(name, 'q=',np.round(s,3), 'fk tcp=', r.tcp_world(s).round(4))
"

# openrua op 19
mkdir -p "$(dirname /workspace/pick_place.py)"
cat > /workspace/pick_place.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Pick one object (top-down grasp) and drop it into the basket.
Usage: python3 -u pick_place.py <x> <y> <grasp_z> <hover_z> [xfingers]
"""
import sys
import numpy as np
from robot import Robot, Q_DOWN_Y, Q_DOWN_X

x, y, gz, hz = map(float, sys.argv[1:5])
Q = Q_DOWN_X if (len(sys.argv) > 5 and sys.argv[5] == "xfingers") else Q_DOWN_Y
BASKET = np.array([0.0, 0.25])
DROP_Z = 0.70
TRANSIT_Z = 0.75

r = Robot()
print("start tcp", r.tcp_world().round(4), "fingers", r.fingers())

print("[1] open gripper")
r.gripper(0.04)

print("[2] hover above object")
if r.move_tcp(np.array([x, y, hz]), Q, 4.0) is None:
    sys.exit("hover IK failed")

print("[3] descend to grasp height")
if r.move_tcp(np.array([x, y, gz]), Q, 2.5) is None:
    sys.exit("grasp IK failed")
w0 = r.wrench()

print("[4] close gripper")
f = r.gripper(0.0)
gap = f[0] - f[1]
print(f"  finger gap = {gap:.4f} m (0 => closed on air)")
if gap < 0.005:
    sys.exit("GRASP FAILED: fingers closed fully")

print("[5] lift")
r.move_tcp(np.array([x, y, TRANSIT_Z]), Q, 2.5)
f = r.fingers()
print(f"  after lift finger gap = {f[0]-f[1]:.4f}", "wrench", r.wrench().round(2), "was", w0.round(2))
if f[0] - f[1] < 0.005:
    sys.exit("object lost during lift")

print("[6] move over basket")
if r.move_tcp(np.array([BASKET[0], BASKET[1], TRANSIT_Z]), Q, 4.0) is None:
    sys.exit("basket IK failed")
r.move_tcp(np.array([BASKET[0], BASKET[1], DROP_Z]), Q, 2.0)
f = r.fingers()
print(f"  over basket finger gap = {f[0]-f[1]:.4f}")

print("[7] release")
r.gripper(0.04)

print("[8] retreat up")
r.move_tcp(np.array([BASKET[0], BASKET[1], TRANSIT_Z + 0.05]), Q, 2.0)
print("DONE")
OPENRUA_EOF

# openrua op 20
nohup python3 -u pick_place.py -0.186 -0.165 0.455 0.62 > soup.log 2>&1 &
sleep 45; cat /workspace/soup.log

# openrua op 21
sleep 40; cat /workspace/soup.log; echo; pgrep -f pick_place.py || echo "process ended"

# openrua op 22
sleep 3; pgrep -f pick_place.py || echo ended; for c in agentview birdview; do timeout 60 python3 tools/perception/cam_snap.py $c snaps/${c}_1.png & done; wait

# openrua op 23
nohup python3 -u pick_place.py 0.097 -0.209 0.432 0.58 > cheese.log 2>&1 &
sleep 60; cat /workspace/cheese.log

# openrua op 24
sleep 30; cat /workspace/cheese.log | tail -12; echo; pgrep -f pick_place.py || echo "process ended"

# openrua op 25
sleep 3; for c in agentview birdview sideview; do timeout 60 python3 tools/perception/cam_snap.py $c snaps/${c}_2.png & done; wait; python3 -c "
import cv2
im = cv2.imread('snaps/agentview_2.png'); crop = im[180:380, 430:640]
cv2.imwrite('snaps/basket_crop.png', cv2.resize(crop, None, fx=3, fy=3, interpolation=cv2.INTER_CUBIC))
im = cv2.imread('snaps/birdview_2.png'); crop = im[240:330, 330:430]
cv2.imwrite('snaps/basket_bird.png', cv2.resize(crop, None, fx=5, fy=5, interpolation=cv2.INTER_CUBIC))"

# openrua op 26
timeout 120 python3 locate.py birdview > /dev/null; python3 - <<'EOF'
import numpy as np, cv2
Pw = np.load('snaps/birdview_world.npy'); z = Pw[...,2]
color = cv2.imread('snaps/birdview_2.png')
mask = (z > 0.432) & (z < 0.77) & np.isfinite(z)
n, lab, stats, cents = cv2.connectedComponentsWithStats(mask.astype(np.uint8), 8)
print("blobs above table (excluding arm at z~0.77+):")
for i in range(1, n):
    if stats[i,4] < 6: continue
    pts = Pw[lab==i]
    print(f"  centroid=({pts[:,0].mean():.3f},{pts[:,1].mean():.3f}) top_z={pts[:,2].max():.3f} x=[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y=[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] area={stats[i,4]}")
# check original spots
for name,(x,y) in {"soup origin":(-0.186,-0.165),"cheese origin":(0.097,-0.209)}.items():
    sel = (np.abs(Pw[...,0]-x)<0.04)&(np.abs(Pw[...,1]-y)<0.03)&np.isfinite(z)
    print(f"{name}: max z in 8x6cm patch = {z[sel].max():.3f} (table=0.420)")
# inside basket: interior region x[-0.06,0.06] y[0.19,0.32]
sel = (Pw[...,0]>-0.06)&(Pw[...,0]<0.06)&(Pw[...,1]>0.19)&(Pw[...,1]<0.32)&np.isfinite(z)
zi = z[sel]; print(f"basket interior z: min={zi.min():.3f} max={zi.max():.3f}; pixels with 0.44<z<0.60 (objects inside, below rim 0.627): {((zi>0.44)&(zi<0.60)).sum()}")
EOF

# openrua op 27
timeout 200 python3 -u -c "
from robot import *
r = Robot()
home = [0.0, -0.161037389, 0.0, -2.44459747, 0.0, 2.2267522, 0.7853981633974483]
code, err = r.move_joints(home, 4.0)
if err > 0.05: code, err = r.move_joints(home, 3.0)
print('tcp', r.tcp_world().round(3))
" && for c in agentview birdview; do timeout 60 python3 tools/perception/cam_snap.py $c snaps/${c}_3.png & done; wait; python3 -c "
import cv2
im = cv2.imread('snaps/birdview_3.png'); crop = im[230:330, 320:440]
cv2.imwrite('snaps/basket_bird3.png', cv2.resize(crop, None, fx=5, fy=5, interpolation=cv2.INTER_CUBIC))"
