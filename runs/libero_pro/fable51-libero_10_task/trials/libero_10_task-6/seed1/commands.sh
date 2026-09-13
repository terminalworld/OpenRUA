#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
cd /workspace; cat tools/action/fjt_send.py; echo ----; cat tools/action/gripper_cmd.py; echo ----; cat tools/action/ik_move.py

# openrua op 3
cd /workspace; ros2 topic list; echo ----; ros2 action list; echo ----; ros2 service list | grep -iv parameter | head -40; echo ----; ros2 node list

# openrua op 4
cd /workspace; ros2 topic echo /joint_states --once; for c in agentview birdview frontview sideview robot0_eye_in_hand; do python3 tools/perception/cam_snap.py $c; done

# openrua op 5
cd /workspace; ros2 topic echo /birdview/color/camera_info --once | head -30; timeout 10 ros2 run tf2_ros tf2_echo world birdview_optical_frame 2>&1 | head -12; ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id|child" | head -60

# openrua op 6
cd /workspace; cat > geo.py <<'EOF'
#!/usr/bin/env python3
"""Grab depth+color+info+TF for a camera; save world xyz map to <cam>_xyz.npy and color to <cam>.png.
Usage: python3 geo.py <cam>"""
import sys, numpy as np, rclpy, cv2
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener

def grab(node, topic, T, timeout=20.0):
    got = {}
    sub = node.create_subscription(T, topic, lambda m: got.setdefault("m", m), 1)
    import time; end = time.time() + timeout
    while "m" not in got and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    return got["m"]

def quat_R(x, y, z, w):
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])

def main():
    cam = sys.argv[1]
    rclpy.init(); node = rclpy.create_node("geo")
    tfbuf = Buffer(); TransformListener(tfbuf, node)
    br = CvBridge()
    depth = br.imgmsg_to_cv2(grab(node, f"/{cam}/depth/image_raw", Image), "passthrough").astype(np.float32)
    color = br.imgmsg_to_cv2(grab(node, f"/{cam}/color/image_raw", Image), "bgr8")
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    frame = f"{cam}_optical_frame"
    import time; end = time.time() + 10
    while time.time() < end and not tfbuf.can_transform("world", frame, rclpy.time.Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation; R = quat_R(q.x, q.y, q.z, q.w)
    p0 = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    h, w = depth.shape
    u, v = np.meshgrid(np.arange(w), np.arange(h))
    pc = np.stack([(u-cx)*depth/fx, (v-cy)*depth/fy, depth], -1)
    xyz = pc @ R.T + p0
    np.save(f"{cam}_xyz.npy", xyz); cv2.imwrite(f"{cam}.png", color)
    print("saved", cam, "cam pos", p0, "depth range", np.nanmin(depth), np.nanmax(depth))
    rclpy.shutdown()

main()
EOF
python3 geo.py birdview && python3 geo.py agentview

# openrua op 7
cd /workspace; python3 - <<'EOF'
import numpy as np, cv2
xyz = np.load("birdview_xyz.npy"); img = cv2.imread("birdview.png")
z = xyz[...,2]
# table height: mode of z in center region
zc = z[150:350, 200:450]
hist, edges = np.histogram(zc, bins=200)
print("table z mode:", edges[np.argmax(hist)])
# pixel -> world checks
for (u,v) in [(318,320),(297,268),(275,262),(310,242),(320,150),(100,240),(540,240),(320,50)]:
    print((u,v), xyz[v,u].round(3), img[v,u])
EOF

# openrua op 8
cd /workspace; python3 - <<'EOF'
import numpy as np, cv2
xyz = np.load("birdview_xyz.npy"); img = cv2.imread("birdview.png")
z = xyz[...,2]
print("plain table px:", xyz[300,400].round(3), xyz[200,250].round(3))
table = 0.425
mask = ((z > table+0.008) & (z < 0.75)).astype(np.uint8)
# exclude robot: robot is roughly x < -0.15?  Let's just cluster all
n, lab, stats, cents = cv2.connectedComponentsWithStats(mask)
for i in range(1, n):
    if stats[i, cv2.CC_STAT_AREA] < 15: continue
    m = lab == i
    pts = xyz[m]; col = img[m].mean(0)
    print(f"comp {i}: area={stats[i,4]} centroid_px={cents[i].round(0)} x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] z[{pts[:,2].min():.3f},{pts[:,2].max():.3f}] bgr={col.round(0)}")
EOF

# openrua op 9
cd /workspace; python3 - <<'EOF'
import cv2, numpy as np
img = cv2.imread("birdview.png")
crop = img[200:350, 230:400]
cv2.imwrite("bird_crop.png", cv2.resize(crop, None, fx=4, fy=4, interpolation=cv2.INTER_NEAREST))
xyz = np.load("birdview_xyz.npy")
z = xyz[200:350,230:400,2]
np.set_printoptions(linewidth=250, precision=2)
print(z[60:80:2, 55:80])
EOF

# openrua op 10
cd /workspace; python3 - <<'EOF'
import numpy as np, cv2
xyz = np.load("agentview_xyz.npy"); img = cv2.imread("agentview.png")
z = xyz[...,2]
print("table px:", xyz[300,100].round(3), xyz[450,500].round(3))
mask = ((z > 0.44) & (z < 0.75) & (xyz[...,0] > -0.35)).astype(np.uint8)
n, lab, stats, cents = cv2.connectedComponentsWithStats(mask)
for i in range(1, n):
    if stats[i, cv2.CC_STAT_AREA] < 30: continue
    m = lab == i
    pts = xyz[m]; col = img[m].mean(0)
    print(f"comp {i}: area={stats[i,4]} px={cents[i].round(0)} x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] z[{pts[:,2].min():.3f},{pts[:,2].max():.3f}] bgr={col.round(0)}")
# pudding pixel probes
for (u,v) in [(378,278),(370,270),(390,285),(295,385),(322,200),(190,225)]:
    print((u,v), xyz[v,u].round(3), img[v,u])
EOF

# openrua op 11
cd /workspace; timeout 8 ros2 run tf2_ros tf2_echo world panda_hand 2>&1 | head -8; timeout 8 ros2 run tf2_ros tf2_echo panda_link0 panda_hand 2>&1 | head -8; timeout 8 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -8

# openrua op 12
cd /workspace; python3 tools/action/gripper_cmd.py 0.04; ros2 topic echo /joint_states --once | grep -A9 position | tail -2; python3 geo.py agentview; python3 - <<'EOF'
import numpy as np, cv2
xyz = np.load("agentview_xyz.npy"); img = cv2.imread("agentview.png")
z = xyz[...,2]
print("table px:", xyz[300,100].round(3))
mask = ((z > 0.44) & (z < 0.75) & (xyz[...,0] > -0.35)).astype(np.uint8)
n, lab, stats, cents = cv2.connectedComponentsWithStats(mask)
for i in range(1, n):
    if stats[i, cv2.CC_STAT_AREA] < 30: continue
    m = lab == i; pts = xyz[m]
    print(f"comp {i}: area={stats[i,4]} px={cents[i].round(0)} x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] z[{pts[:,2].min():.3f},{pts[:,2].max():.3f}]")
EOF

# openrua op 13
cd /workspace; python3 - <<'EOF'
import numpy as np, cv2
xyz = np.load("agentview_xyz.npy"); img = cv2.imread("agentview.png")
z = xyz[...,2]
# red mug region
m = (z > 0.44) & (z < 0.75) & (xyz[...,0] > -0.26) & (xyz[...,0] < -0.14) & (np.abs(xyz[...,1]) < 0.08)
pts = xyz[m]; cols = img[m]
print("n", len(pts))
for zlo in np.arange(0.44, 0.58, 0.02):
    s = pts[(pts[:,2]>=zlo)&(pts[:,2]<zlo+0.02)]
    if len(s): print(f"z {zlo:.2f}: n={len(s)} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]")
# rim: points with z>0.565
rim = pts[pts[:,2]>0.565]
print("rim x", rim[:,0].min(), rim[:,0].max(), "y", rim[:,1].min(), rim[:,1].max())
# fit circle to rim xy
A = np.c_[2*rim[:,0], 2*rim[:,1], np.ones(len(rim))]
b = (rim[:,0]**2 + rim[:,1]**2)
c = np.linalg.lstsq(A, b, rcond=None)[0]
cx, cy = c[0], c[1]; r = np.sqrt(c[2] + cx**2 + cy**2)
print("rim circle center", cx, cy, "r", r)
# save crop
ys, xs = np.where(m)
crop = img[ys.min()-10:ys.max()+10, xs.min()-10:xs.max()+10]
cv2.imwrite("mug_crop.png", cv2.resize(crop, None, fx=4, fy=4, interpolation=cv2.INTER_NEAREST))
EOF

# openrua op 14
cd /; (find / -xdev \( -iname "*.bddl" -o -iname "*libero*" -o -iname "*pudding*" \) 2>/dev/null | grep -v proc | head -20); pip list 2>/dev/null | grep -iE "libero|robosuite|mujoco" ; ls /workspace/.ros/log | head; grep -il "region\|right" /workspace/.ros/log/* 2>/dev/null | head

# openrua op 15
cd /workspace; head -5 .ros/log/python3_113_1789227533989.log; ros2 node info /robot_bridge 2>&1 | head -50; ros2 param list /robot_bridge 2>&1 | head

# openrua op 16
mkdir -p "$(dirname /workspace/ctl.py)"
cat > /workspace/ctl.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Sequenced TCP moves for the Panda, world-frame in, verified out.

Usage: python3 ctl.py CMD [CMD ...]
  move:x,y,z[,yaw_deg],secs   IK (base frame) -> trajectory; TCP target in WORLD
  grip:open|close             gripper to 0.04 / 0.0 per finger
  pose                        print TCP world pose + finger gap
Hand points straight down; yaw=0 => fingers open along world Y.
"""
import math
import sys
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

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]
TCP = float(M["hand"]["tcp_offset_m"])
W2B = np.array([-0.51, 0.0, 0.42])  # world -> panda_link0 translation (TF, identity rotation)


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def down_quat(yaw_deg):
    """Hand Z down, hand X rotated by yaw about world Z. q = Rz(yaw) * Rx(pi)."""
    h = math.radians(yaw_deg) / 2
    # Rz(yaw) = (0,0,sin h,cos h); Rx(pi) = (1,0,0,0); product (Hamilton):
    return (math.cos(h), math.sin(h), 0.0, 0.0)  # (x, y, z, w)


class Ctl:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("ctl")
        self.js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self.js.update(zip(m.name, m.position)), 1)
        self.tfbuf = Buffer()
        TransformListener(self.tfbuf, self.node)
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.gr = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik.wait_for_service(10)
        self.fjt.wait_for_server(10)
        self.gr.wait_for_server(10)
        while not self.js:
            rclpy.spin_once(self.node, timeout_sec=0.2)

    def spin(self, n=5):
        for _ in range(n):
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def pose(self):
        self.spin(10)
        end = time.time() + 10
        while time.time() < end and not self.tfbuf.can_transform("world", "panda_hand", rclpy.time.Time()):
            rclpy.spin_once(self.node, timeout_sec=0.2)
        t = self.tfbuf.lookup_transform("world", "panda_hand", rclpy.time.Time())
        q = t.transform.rotation
        R = quat_R(q.x, q.y, q.z, q.w)
        hand = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
        tcp = hand + TCP * R[:, 2]
        gap = self.js.get("panda_finger_joint1", float("nan")) - self.js.get("panda_finger_joint2", float("nan"))
        print(f"TCP world {tcp.round(4)} hand z-axis {R[:,2].round(3)} hand y-axis {R[:,1].round(3)} "
              f"finger gap {gap:.4f}")
        return tcp

    def move(self, x, y, z, yaw, secs):
        qx, qy, qz, qw = down_quat(yaw)
        R = quat_R(qx, qy, qz, qw)
        hand_w = np.array([x, y, z]) - TCP * R[:, 2]
        hand_b = hand_w - W2B
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, hand_b)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = qx, qy, qz, qw
        seed = JointState()
        self.spin(5)
        for j in ARM:
            seed.name.append(j)
            seed.position.append(float(self.js[j]))
        req.ik_request.robot_state.joint_state = seed
        req.ik_request.avoid_collisions = False
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            print(f"IK FAILED for TCP ({x},{y},{z}) yaw {yaw}: "
                  f"{None if res is None else res.error_code.val}")
            return False
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        target = [sol[j] for j in ARM]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM
        pt = JointTrajectoryPoint(positions=target)
        pt.time_from_start = Duration(sec=int(secs), nanosec=int((secs % 1) * 1e9))
        goal.trajectory.points = [pt]
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        rf = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf)
        code = rf.result().result.error_code
        self.spin(10)
        err = max(abs(self.js[j] - t) for j, t in zip(ARM, target))
        print(f"move -> TCP ({x},{y},{z}) yaw {yaw}: fjt code {code}, max joint err {err:.4f}")
        self.pose()
        return code == 0

    def grip(self, what):
        goal = GripperCommand.Goal()
        goal.command.position = GRIP["open_m"] if what == "open" else GRIP["closed_m"]
        goal.command.max_effort = float(GRIP["max_effort"])
        send = self.gr.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=30)
        rf = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=120)
        r = rf.result().result
        self.spin(10)
        gap = self.js["panda_finger_joint1"] - self.js["panda_finger_joint2"]
        print(f"grip {what}: reached={r.reached_goal} stalled={r.stalled} gap {gap:.4f}")


def main():
    c = Ctl()
    for cmd in sys.argv[1:]:
        if cmd == "pose":
            c.pose()
        elif cmd.startswith("grip:"):
            c.grip(cmd.split(":")[1])
        elif cmd.startswith("move:"):
            v = [float(s) for s in cmd.split(":")[1].split(",")]
            if len(v) == 4:
                x, y, z, secs = v; yaw = 0.0
            else:
                x, y, z, yaw, secs = v
            if not c.move(x, y, z, yaw, secs):
                print("ABORTING sequence")
                break
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 17
cd /workspace; python3 -u ctl.py pose 2>&1 | grep -v "^\[" 

# openrua op 18
cd /workspace; python3 -u ctl.py move:-0.194,0.052,0.70,3 2>&1 | grep -v "^\["

# openrua op 19
cd /workspace; python3 - <<'EOF' 2>&1 | grep -v "^\["
import sys; sys.argv=["x"]
import ctl, numpy as np, rclpy
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
c = ctl.Ctl()
def try_ik(x,y,z,yaw=0):
    qx,qy,qz,qw = ctl.down_quat(yaw); R = ctl.quat_R(qx,qy,qz,qw)
    hand_b = np.array([x,y,z]) - ctl.TCP*R[:,2] - ctl.W2B
    req = GetPositionIK.Request(); req.ik_request.group_name="panda_arm"
    p = req.ik_request.pose_stamped.pose
    p.position.x,p.position.y,p.position.z = map(float,hand_b)
    p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w = qx,qy,qz,qw
    seed = JointState(); c.spin(3)
    for j in ctl.ARM: seed.name.append(j); seed.position.append(float(c.js[j]))
    req.ik_request.robot_state.joint_state = seed
    req.ik_request.timeout.sec = 2
    fut = c.ik.call_async(req); rclpy.spin_until_future_complete(c.node, fut, timeout_sec=30)
    r = fut.result()
    print((x,y,z,yaw), "->", None if r is None else r.error_code.val)
try_ik(-0.0589, 0, 0.6744)
try_ik(-0.194, 0.052, 0.70)
try_ik(-0.194, 0.052, 0.65)
try_ik(-0.194, 0.052, 0.60)
try_ik(-0.194, 0.0, 0.70)
try_ik(-0.15, 0.05, 0.70)
try_ik(-0.10, 0.05, 0.70)
try_ik(0.153, 0.02, 0.60)
try_ik(0.153, 0.11, 0.55)
try_ik(-0.061, 0.08, 0.55)
EOF

# openrua op 20
cd /workspace; python3 - <<'EOF' 2>&1 | grep -v "^\["
import sys; sys.argv=["x"]
import ctl, rclpy
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState
c = ctl.Ctl()
cli = c.node.create_client(GetPositionFK, "/compute_fk"); cli.wait_for_service(10)
req = GetPositionFK.Request()
req.fk_link_names = ["panda_link8", "panda_hand", "panda_hand_tcp", "panda_link0"]
req.header.frame_id = ""
js = JointState(); c.spin(3)
for j in ctl.ARM: js.name.append(j); js.position.append(float(c.js[j]))
req.robot_state.joint_state = js
fut = cli.call_async(req); rclpy.spin_until_future_complete(c.node, fut, timeout_sec=30)
r = fut.result()
print("code", r.error_code.val, r.fk_link_names)
for n, ps in zip(r.fk_link_names, r.pose_stamped):
    p, q = ps.pose.position, ps.pose.orientation
    print(n, ps.header.frame_id, round(p.x,4), round(p.y,4), round(p.z,4), "|", round(q.x,3), round(q.y,3), round(q.z,3), round(q.w,3))
EOF
ros2 param get /move_group robot_description_semantic 2>/dev/null | grep -iE "group name|chain|tip|end_effector" | head

# openrua op 21
cd /workspace; python3 - <<'EOF' 2>&1 | grep -v "^\["
import sys; sys.argv=["x"]
import ctl, numpy as np, rclpy, math
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
c = ctl.Ctl()
def qmul(a, b):  # (x,y,z,w)
    ax,ay,az,aw = a; bx,by,bz,bw = b
    return (aw*bx+ax*bw+ay*bz-az*by, aw*by-ax*bz+ay*bw+az*bx, aw*bz+ax*by-ay*bx+az*bw, aw*bw-ax*bx-ay*by-az*bz)
def try_ik(x,y,z,yaw=0, base=False):
    qh = ctl.down_quat(yaw); R = ctl.quat_R(*qh)
    q8 = qmul(qh, (0,0,math.sin(math.pi/8), math.cos(math.pi/8)))
    hand = np.array([x,y,z]) - ctl.TCP*R[:,2]
    if base: hand = hand - ctl.W2B
    req = GetPositionIK.Request(); req.ik_request.group_name="panda_arm"
    p = req.ik_request.pose_stamped.pose
    p.position.x,p.position.y,p.position.z = map(float,hand)
    p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w = q8
    seed = JointState(); c.spin(3)
    for j in ctl.ARM: seed.name.append(j); seed.position.append(float(c.js[j]))
    req.ik_request.robot_state.joint_state = seed
    req.ik_request.timeout.sec = 2
    fut = c.ik.call_async(req); rclpy.spin_until_future_complete(c.node, fut, timeout_sec=30)
    r = fut.result()
    code = None if r is None else r.error_code.val
    sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position)) if code == 1 else {}
    print((x,y,z,yaw), "base" if base else "world", "->", code, [round(sol[j],3) for j in ctl.ARM] if sol else "")
try_ik(-0.0589, 0, 0.6744, base=False)
try_ik(-0.0589, 0, 0.6744, base=True)
try_ik(-0.194, 0.052, 0.70)
try_ik(-0.194, 0.052, 0.56)
try_ik(0.153, 0.02, 0.60)
try_ik(0.153, 0.11, 0.46)
try_ik(-0.061, 0.08, 0.455)
EOF

# openrua op 22
cd /workspace; python3 - <<'EOF'
import re
s = open("ctl.py").read()
s = s.replace('''W2B = np.array([-0.51, 0.0, 0.42])  # world -> panda_link0 translation (TF, identity rotation)
''', '''# MoveIt's model frame here is WORLD (FK puts panda_link0 at -0.51,0,0.42);
# the panda_arm group's tip link is panda_link8 = panda_hand rotated -45deg about Z.
Q_HAND_TO_L8 = (0.0, 0.0, math.sin(math.pi / 8), math.cos(math.pi / 8))


def qmul(a, b):  # (x, y, z, w) Hamilton product a*b
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)
''')
s = s.replace('''        hand_w = np.array([x, y, z]) - TCP * R[:, 2]
        hand_b = hand_w - W2B
''', '''        hand_b = np.array([x, y, z]) - TCP * R[:, 2]  # link8 origin == hand origin
        qx, qy, qz, qw = qmul((qx, qy, qz, qw), Q_HAND_TO_L8)
''')
s = s.replace('''        req.ik_request.avoid_collisions = False
''', '''        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = 2
''')
open("ctl.py", "w").write(s)
EOF
python3 -u ctl.py move:-0.194,0.052,0.70,3 2>&1 | grep -v "^\["

# openrua op 23
cd /workspace; python3 tools/perception/cam_snap.py robot0_eye_in_hand >/dev/null; python3 geo.py birdview > /dev/null; python3 - <<'EOF'
import numpy as np
xyz = np.load("birdview_xyz.npy"); z = xyz[...,2]
# red mug rim from birdview now (hand raised above); rim ~ z 0.56-0.58
m = (z>0.555)&(z<0.59)&(xyz[...,0]>-0.3)&(xyz[...,0]<-0.1)&(np.abs(xyz[...,1])<0.1)
pts = xyz[m]; print("rim pts", len(pts), "x", pts[:,0].min().round(3), pts[:,0].max().round(3), "y", pts[:,1].min().round(3), pts[:,1].max().round(3))
A = np.c_[2*pts[:,0], 2*pts[:,1], np.ones(len(pts))]; b = pts[:,0]**2+pts[:,1]**2
c = np.linalg.lstsq(A,b,rcond=None)[0]; print("circle", c[0].round(4), c[1].round(4), "r", np.sqrt(c[2]+c[0]**2+c[1]**2).round(4))
EOF

# openrua op 24
cd /workspace; python3 geo.py robot0_eye_in_hand && python3 - <<'EOF'
import numpy as np, cv2
xyz = np.load("robot0_eye_in_hand_xyz.npy"); z = xyz[...,2]
print("table sample z", np.nanmedian(z[100:200, 50:150]).round(3))
m = (z>0.55)&(z<0.60)&(xyz[...,0]>-0.3)&(xyz[...,0]<-0.1)&(np.abs(xyz[...,1])<0.1)
pts = xyz[m]; print("rim pts", len(pts), "x", pts[:,0].min().round(3), pts[:,0].max().round(3), "y", pts[:,1].min().round(3), pts[:,1].max().round(3))
A = np.c_[2*pts[:,0], 2*pts[:,1], np.ones(len(pts))]; b = pts[:,0]**2+pts[:,1]**2
c = np.linalg.lstsq(A,b,rcond=None)[0]; print("rim circle", c[0].round(4), c[1].round(4), "r", np.sqrt(c[2]+c[0]**2+c[1]**2).round(4))
# also pudding
m2 = (z>0.45)&(z<0.48)&(xyz[...,0]>-0.15)&(xyz[...,0]<0.0)&(xyz[...,1]>0.02)&(xyz[...,1]<0.15)
p2 = xyz[m2]; print("pudding top pts", len(p2), "x", p2[:,0].min().round(3), p2[:,0].max().round(3), "y", p2[:,1].min().round(3), p2[:,1].max().round(3), "center", p2[:,:2].mean(0).round(3))
EOF

# openrua op 25
cd /workspace; python3 - <<'EOF'
import numpy as np
xyz = np.load("robot0_eye_in_hand_xyz.npy"); z = xyz[...,2]
for zlo in [0.555, 0.565, 0.57]:
    m = (z>zlo)&(z<0.60)&(xyz[...,0]>-0.3)&(xyz[...,0]<-0.1)&(np.abs(xyz[...,1])<0.1)
    pts = xyz[m][:, :2]
    for it in range(5):
        A = np.c_[2*pts[:,0], 2*pts[:,1], np.ones(len(pts))]; b = (pts**2).sum(1)
        c = np.linalg.lstsq(A,b,rcond=None)[0]; cx, cy = c[0], c[1]; r = np.sqrt(c[2]+cx**2+cy**2)
        d = np.abs(np.hypot(pts[:,0]-cx, pts[:,1]-cy) - r)
        keep = d < max(0.004, np.percentile(d, 80))
        pts = pts[keep]
    print(f"zlo {zlo}: n={len(pts)} center ({cx:.4f}, {cy:.4f}) r {r:.4f}  y-extent {pts[:,1].min():.3f}..{pts[:,1].max():.3f} x-extent {pts[:,0].min():.3f}..{pts[:,0].max():.3f}")
EOF

# openrua op 26
cd /workspace; python3 -u ctl.py move:-0.195,0.056,0.60,2 move:-0.195,0.056,0.555,2 grip:close move:-0.195,0.056,0.70,2 2>&1 | grep -v "^\["

# openrua op 27
cd /workspace; python3 tools/perception/cam_snap.py agentview >/dev/null; python3 tools/perception/cam_snap.py sideview > /dev/null; echo ok

# openrua op 28
cd /workspace; python3 -u ctl.py move:-0.10,0.03,0.72,3 move:0.153,0.019,0.72,4 move:0.153,0.019,0.60,3 2>&1 | grep -v "^\["

# openrua op 29
cd /workspace; python3 -u ctl.py move:0.153,0.019,0.72,4 2>&1 | grep -v "^\["

# openrua op 30
cd /workspace; python3 -u ctl.py move:0.153,0.019,0.62,3 move:0.153,0.019,0.582,3 2>&1 | grep -v "^\["; python3 tools/perception/cam_snap.py agentview >/dev/null

# openrua op 31
cd /workspace; python3 -u ctl.py grip:open move:0.153,0.019,0.70,3 2>&1 | grep -v "^\["; python3 tools/perception/cam_snap.py agentview >/dev/null; python3 geo.py birdview >/dev/null; python3 - <<'EOF'
import numpy as np
xyz = np.load("birdview_xyz.npy"); z = xyz[...,2]
# red mug rim near plate
m = (z>0.56)&(z<0.62)&(xyz[...,0]>0.0)&(xyz[...,0]<0.3)&(np.abs(xyz[...,1])<0.15)
pts = xyz[m][:, :2]
for it in range(5):
    A = np.c_[2*pts[:,0], 2*pts[:,1], np.ones(len(pts))]; b = (pts**2).sum(1)
    c = np.linalg.lstsq(A,b,rcond=None)[0]; cx, cy = c[0], c[1]; r = np.sqrt(c[2]+cx**2+cy**2)
    d = np.abs(np.hypot(pts[:,0]-cx, pts[:,1]-cy) - r); pts = pts[d < max(0.004, np.percentile(d, 80))]
print(f"mug rim center ({cx:.3f},{cy:.3f}) r {r:.3f}; rim z max {xyz[m][:,2].max():.3f}")
# plate
mp = (z>0.445)&(z<0.47)&(xyz[...,0]>0.0)&(xyz[...,0]<0.3)&(np.abs(xyz[...,1])<0.15)
pp = xyz[mp]; print("plate pts", len(pp), "x", pp[:,0].min().round(3), pp[:,0].max().round(3), "y", pp[:,1].min().round(3), pp[:,1].max().round(3))
EOF

# openrua op 32
cd /workspace; python3 -u ctl.py move:-0.062,0.08,0.62,4 move:-0.062,0.08,0.50,2 move:-0.062,0.08,0.447,2 2>&1 | grep -v "^\["; python3 tools/perception/cam_snap.py robot0_eye_in_hand >/dev/null

# openrua op 33
cd /workspace; python3 -u ctl.py move:-0.062,0.08,0.62,4 move:-0.062,0.08,0.50,3 move:-0.062,0.08,0.447,3 2>&1 | grep -v "^\["; python3 tools/perception/cam_snap.py robot0_eye_in_hand >/dev/null

# openrua op 34
cd /workspace; python3 -u ctl.py grip:close move:-0.062,0.08,0.60,3 2>&1 | grep -v "^\["; python3 tools/perception/cam_snap.py agentview >/dev/null

# openrua op 35
cd /workspace; python3 -u ctl.py move:0.153,0.13,0.65,4 2>&1 | grep -v "^\["

# openrua op 36
cd /workspace; python3 -u ctl.py move:0.153,0.13,0.65,4 move:0.153,0.13,0.52,3 move:0.153,0.13,0.453,3 2>&1 | grep -v "^\["

# openrua op 37
cd /workspace; python3 -u ctl.py grip:open move:0.153,0.13,0.60,3 move:-0.10,0.0,0.75,4 2>&1 | grep -v "^\["

# openrua op 38
cd /workspace; python3 -u ctl.py move:-0.10,0.0,0.75,4 2>&1 | grep -v "^\["; for c in agentview birdview frontview; do python3 tools/perception/cam_snap.py $c >/dev/null; done; python3 geo.py birdview >/dev/null

# openrua op 39
cd /workspace; python3 - <<'EOF'
import numpy as np
xyz = np.load("birdview_xyz.npy"); z = xyz[...,2]
roi = (xyz[...,0]>0.0)&(xyz[...,0]<0.3)&(np.abs(xyz[...,1])<0.25)
# mug rim (highest ring)
m = roi&(z>0.56)&(z<0.62); pts = xyz[m][:,:2]
for _ in range(5):
    A = np.c_[2*pts[:,0],2*pts[:,1],np.ones(len(pts))]; b=(pts**2).sum(1)
    c = np.linalg.lstsq(A,b,rcond=None)[0]; cx,cy=c[0],c[1]; r=np.sqrt(c[2]+cx**2+cy**2)
    d=np.abs(np.hypot(pts[:,0]-cx,pts[:,1]-cy)-r); pts=pts[d<max(0.004,np.percentile(d,80))]
print(f"red mug: rim center ({cx:.3f},{cy:.3f}) r {r:.3f}, rim top z {xyz[m][:,2].max():.3f} (was 0.575 on table -> now +{xyz[m][:,2].max()-0.575:.3f})")
# plate: ring z 0.445-0.47 excluding mug body footprint
mp = roi&(z>0.445)&(z<0.47)&(xyz[...,1]<0.06); pp=xyz[mp]
print(f"plate: x [{pp[:,0].min():.3f},{pp[:,0].max():.3f}] y [{pp[:,1].min():.3f},{pp[:,1].max():.3f}] -> center ({(pp[:,0].min()+pp[:,0].max())/2:.3f},{(pp[:,1].min()+pp[:,1].max())/2:.3f})")
# pudding
mq = roi&(z>0.455)&(z<0.48)&(xyz[...,1]>0.07); pq=xyz[mq]
print(f"pudding: n={len(pq)} x [{pq[:,0].min():.3f},{pq[:,0].max():.3f}] y [{pq[:,1].min():.3f},{pq[:,1].max():.3f}] top z {pq[:,2].max():.3f} center ({pq[:,0].mean():.3f},{pq[:,1].mean():.3f})")
EOF
