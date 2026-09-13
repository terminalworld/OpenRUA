#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la && find . -maxdepth 3 -type f | head -100

# openrua op 1
cat tools/README.md && echo ---- && cat tools/perception/cam_snap.py && echo ---- && cat tools/perception/px2world.py

# openrua op 2
cat tools/action/fjt_send.py && echo ---- && cat tools/action/gripper_cmd.py && echo ---- && cat tools/action/ik_move.py

# openrua op 3
ros2 topic list 2>&1; echo ---; ros2 action list 2>&1; echo ---; ros2 service list 2>&1 | grep -v -E "parameter|describe|get_type" ; echo ---; ros2 node list 2>&1

# openrua op 4
for c in agentview frontview birdview sideview robot0_eye_in_hand; do python3 tools/perception/cam_snap.py $c; done; ros2 topic echo /joint_states --once 2>&1 | head -40

# openrua op 5
python3 -c "
import cv2
im=cv2.imread('agentview.png')
crop=im[200:420,60:420]
cv2.imwrite('crop1.png',cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
"

# openrua op 6
timeout 20 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -12; echo ---; timeout 20 ros2 run tf2_ros tf2_echo world agentview_optical_frame 2>&1 | head -12; echo ---; ros2 topic echo /agentview/color/camera_info --once 2>&1 | grep -A12 "^k:"

# openrua op 7
mkdir -p "$(dirname /workspace/pxw.py)"
cat > /workspace/pxw.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Batch pixel->world for one camera: python3 pxw.py <cam> u,v [u,v ...]
Also supports a window median: u,v,r -> median depth in (2r+1)^2 window.
"""
import sys
import numpy as np
import rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge


def grab(node, topic, typ, timeout=15.0):
    got = {}
    sub = node.create_subscription(typ, topic, lambda m: got.setdefault("m", m), 1)
    import time
    end = time.time() + timeout
    while "m" not in got and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    return got.get("m")


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main():
    cam = sys.argv[1]
    rclpy.init()
    node = rclpy.create_node("pxw")
    buf = Buffer(); TransformListener(buf, node)
    depth = CvBridge().imgmsg_to_cv2(grab(node, f"/{cam}/depth/image_raw", Image), "passthrough")
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    frame = f"{cam}_optical_frame"
    import time
    end = time.time() + 10
    while time.time() < end and not buf.can_transform("world", frame, rclpy.time.Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    t = buf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    T = np.eye(4); T[:3, :3] = quat_R(q.x, q.y, q.z, q.w)
    T[:3, 3] = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
    for arg in sys.argv[2:]:
        parts = [int(a) for a in arg.split(",")]
        u, v = parts[0], parts[1]
        r = parts[2] if len(parts) > 2 else 0
        win = depth[max(0, v - r):v + r + 1, max(0, u - r):u + r + 1]
        win = win[np.isfinite(win) & (win > 0)]
        if win.size == 0:
            print(f"({u},{v}) no depth"); continue
        z = float(np.median(win))
        p = T @ np.array([(u - cx) * z / fx, (v - cy) * z / fy, z, 1.0])
        print(f"({u},{v}) depth={z:.3f} world=({p[0]:.4f}, {p[1]:.4f}, {p[2]:.4f})")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 8
python3 pxw.py agentview 215,255 213,270 363,360 363,345 530,270 530,320 300,420 100,420; echo ---; python3 pxw.py birdview 283,263 333,296 383,288 383,270 320,340 250,350 330,250

# openrua op 9
python3 -c "
import cv2
im=cv2.imread('birdview.png')
crop=im[240:330,230:420]
cv2.imwrite('crop2.png',cv2.resize(crop,None,fx=6,fy=6,interpolation=cv2.INTER_CUBIC))
"

# openrua op 10
python3 - <<'EOF'
import numpy as np, rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
rclpy.init(); n=rclpy.create_node('d')
got={}
n.create_subscription(Image,'/birdview/depth/image_raw',lambda m:got.setdefault('m',m),1)
while 'm' not in got: rclpy.spin_once(n,timeout_sec=0.2)
d=CvBridge().imgmsg_to_cv2(got['m'],'passthrough')
np.save('bird_depth.npy',d)
# camera at z=? depth of table ~2.575; objects closer
for name,(u,v) in {'soup':(284,265),'butter':(332,299),'basket':(383,288)}.items():
    win=d[v-25:v+26,u-25:u+26]
    print(name, 'center depth', d[v,u])
    mask=win < d[v,u]+0.02
    ys,xs=np.where(mask)
    print('  extent px: u',xs.min()-25,xs.max()-25,' v',ys.min()-25,ys.max()-25, ' count',mask.sum())
# butter mask precise
u,v=332,299
win=d[v-15:v+16,u-15:u+16]
print((win<2.56).astype(int))
EOF

# openrua op 11
python3 - <<'EOF'
import numpy as np
d=np.load('bird_depth.npy')
u,v=284,265
win=d[v-15:v+16,u-15:u+16]
m=(win<2.53)
print(m.astype(int))
ys,xs=np.where(m)
print('soup center px', u+xs.mean()-15, v+ys.mean()-15, 'w',xs.max()-xs.min()+1,'h',ys.max()-ys.min()+1)
EOF
python3 pxw.py birdview 284,265 332,298 383,288

# openrua op 12
mkdir -p "$(dirname /workspace/rb.py)"
cat > /workspace/rb.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Small robot helper library for this Panda workstation (see machine.yaml)."""
import sys, time
import numpy as np
import rclpy, yaml
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState, Image, CameraInfo
from geometry_msgs.msg import WrenchStamped, TwistStamped
from moveit_msgs.srv import GetPositionIK, GetPositionFK

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]
BASE_IN_WORLD = np.array([-0.510, 0.000, 0.420])  # from tf world->panda_link0
TCP_OFF = float(M["hand"]["tcp_offset_m"])
TOPDOWN = (1.0, 0.0, 0.0, 0.0)  # hand z down, fingers open along base y


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


def yaw_quat(yaw):
    """Top-down grasp rotated by yaw about world z (fingers along y at yaw=0)."""
    # q = Rz(yaw) * (1,0,0,0)
    c, s = np.cos(yaw / 2), np.sin(yaw / 2)
    # Rz = (0,0,s,c); (1,0,0,0) ; product (w1w2 - v1.v2, w1v2 + w2v1 + v1xv2)
    # q1 = (x=0,y=0,z=s,w=c), q2 = (1,0,0,0)
    x = c * 1 + 0
    y = s * 1  # z1*x2 -> v1 x v2 = (0,0,s)x(1,0,0) = (0, s, 0)
    z = 0.0
    w = 0.0
    return (x, y, z, w)


class Robot:
    def __init__(self, name="rb"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self.js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self.js.__setitem__("m", m), 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik_cli = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.twist_pub = self.node.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)

    def spin(self, t=0.1):
        rclpy.spin_once(self.node, timeout_sec=t)

    # ---------- sensing ----------
    def joints(self, fresh=True):
        if fresh:
            self.js.pop("m", None)
        end = time.time() + 15
        while "m" not in self.js and time.time() < end:
            self.spin(0.2)
        m = self.js["m"]
        return dict(zip(m.name, m.position))

    def arm_q(self):
        j = self.joints()
        return [j[n] for n in ARM]

    def fingers(self):
        j = self.joints()
        return j.get("panda_finger_joint1"), j.get("panda_finger_joint2")

    def wrench(self):
        got = {}
        sub = self.node.create_subscription(WrenchStamped, M["sensors"][1]["port"],
                                            lambda m: got.setdefault("m", m), 1)
        end = time.time() + 10
        while "m" not in got and time.time() < end:
            self.spin(0.2)
        self.node.destroy_subscription(sub)
        if "m" not in got:
            return None
        f = got["m"].wrench.force; t = got["m"].wrench.torque
        return np.array([f.x, f.y, f.z, t.x, t.y, t.z])

    def fk(self, q=None, link="panda_hand"):
        """Hand pose in base frame -> (pos[3], quat[4]) ; also returns world pos."""
        if q is None:
            q = self.arm_q()
        self.fk_cli.wait_for_service(timeout_sec=10)
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = [float(x) for x in q]
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {res and res.error_code.val}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z])
        quat = np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        return pos, quat

    def tcp_world(self, q=None):
        pos, quat = self.fk(q)
        R = quat_R(*quat)
        tcp_b = pos + TCP_OFF * R[:, 2]
        return b2w(tcp_b), quat

    # ---------- IK ----------
    def ik(self, pos_base, quat, seed=None, at_tcp=True, timeout=60):
        pos_base = np.asarray(pos_base, float)
        if at_tcp:
            R = quat_R(*quat)
            pos_base = pos_base - TCP_OFF * R[:, 2]
        self.ik_cli.wait_for_service(timeout_sec=10)
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pos_base)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        seed = seed if seed is not None else self.arm_q()
        req.ik_request.robot_state.joint_state.name = list(ARM)
        req.ik_request.robot_state.joint_state.position = [float(x) for x in seed]
        req.ik_request.avoid_collisions = False
        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        res = fut.result()
        if res is None:
            raise RuntimeError("IK timeout")
        if res.error_code.val != 1:
            raise RuntimeError(f"IK failed code={res.error_code.val}")
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in ARM]

    def ik_world(self, pos_world, quat=TOPDOWN, **kw):
        return self.ik(w2b(pos_world), quat, **kw)

    # ---------- acting ----------
    def move(self, q, seconds=3.0, wait=True):
        self.fjt.wait_for_server(timeout_sec=10)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pt = JointTrajectoryPoint(positions=[float(x) for x in q])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        goal.trajectory.points = [pt]
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        if gh is None or not gh.accepted:
            raise RuntimeError("FJT goal rejected")
        if not wait:
            return gh
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=600)
        r = rf.result()
        code = r.result.error_code if r else None
        now = self.arm_q()
        err = max(abs(a - b) for a, b in zip(now, q))
        print(f"move: error_code={code} max_joint_err={err:.4f}")
        return code, err

    def move_world(self, pos_world, quat=TOPDOWN, seconds=3.0, seed=None):
        q = self.ik_world(pos_world, quat, seed=seed)
        r = self.move(q, seconds)
        tcp, _ = self.tcp_world()
        print(f"  tcp now world={np.round(tcp,4)} target={np.round(pos_world,4)}")
        return r

    def gripper(self, width, wait=True):
        self.grip.wait_for_server(timeout_sec=10)
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP.get("max_effort", 30.0))
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        gh = fut.result()
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        res = rf.result().result
        f = self.fingers()
        print(f"gripper: reached={res.reached_goal} stalled={res.stalled} fingers={f}")
        return res, f

    def snap(self, cam, out=None):
        import cv2
        from cv_bridge import CvBridge
        got = {}
        sub = self.node.create_subscription(Image, f"/{cam}/color/image_raw",
                                            lambda m: got.setdefault("m", m), 1)
        end = time.time() + 15
        while "m" not in got and time.time() < end:
            self.spin(0.2)
        self.node.destroy_subscription(sub)
        img = CvBridge().imgmsg_to_cv2(got["m"], "bgr8")
        out = out or f"/workspace/{cam}.png"
        cv2.imwrite(out, img)
        return out
OPENRUA_EOF

# openrua op 13
python3 - <<'EOF'
from rb import *
r = Robot()
q = r.arm_q(); print("q", np.round(q,3))
pos, quat = r.fk(q); print("hand base", np.round(pos,4), np.round(quat,4))
tcp, _ = r.tcp_world(q); print("tcp world", np.round(tcp,4))
print("fingers", r.fingers())
print("wrench", r.wrench())
# test IK: above soup
sol = r.ik_world([-0.095,-0.156,0.60], TOPDOWN)
print("ik above soup", np.round(sol,3))
tcp2,_ = r.tcp_world(sol); print("fk of sol tcp world", np.round(tcp2,4))
EOF

# openrua op 14
timeout 15 ros2 run tf2_ros tf2_echo world robot0_eye_in_hand_optical_frame 2>&1 | grep -A1 Translation | head -3; timeout 15 ros2 run tf2_ros tf2_echo panda_link0 panda_hand 2>&1 | grep -A1 Translation | head -3
python3 - <<'EOF'
from rb import *
r = Robot()
# try IK with pose expressed in WORLD (no base offset)
try:
    sol = r.ik([-0.095,-0.156,0.60], TOPDOWN)
    print("ik(world coords) ok", np.round(sol,3))
    pos,quat = r.fk(sol); print("fk of sol", np.round(pos,4), np.round(quat,4))
except Exception as e: print(e)
EOF

# openrua op 15
python3 - <<'EOF'
from rb import *
r = Robot()
sol = r.ik([-0.095,-0.156,0.60], TOPDOWN)
for link in ["panda_link8","panda_hand"]:
    pos,quat = r.fk(sol, link); print(link, np.round(pos,4), np.round(quat,4))
# try requesting yaw -45 deg so panda_hand ends at (1,0,0,0)?
for yaw in [np.pi/4, -np.pi/4]:
    qd = yaw_quat(yaw)
    sol = r.ik([-0.095,-0.156,0.60], qd)
    pos,quat = r.fk(sol, "panda_hand")
    R = quat_R(*quat)
    print("req yaw", yaw, "->hand quat", np.round(quat,4), "hand y-axis in world", np.round(R[:,1],3), "z-axis", np.round(R[:,2],3))
EOF

# openrua op 16
python3 - <<'EOF'
import re
s = open('rb.py').read()
s = s.replace('''def w2b(p):
    return np.asarray(p, float) - BASE_IN_WORLD


def b2w(p):
    return np.asarray(p, float) + BASE_IN_WORLD
''', '''# NOTE (measured): MoveIt's model frame on this machine is WORLD, not the
# arm base (FK of panda_hand at home = (-0.053, 0, 0.778) = tf world->hand).
# IK tip link is panda_link8; panda_hand is yawed -45 deg from it.
def w2b(p):
    return np.asarray(p, float)


def b2w(p):
    return np.asarray(p, float)


def grasp_quat(finger_yaw=0.0):
    """link8 quaternion for a top-down grasp whose finger-opening axis is
    world +y rotated by finger_yaw about world z (0 -> fingers along y,
    pi/2 -> fingers along x)."""
    return yaw_quat(finger_yaw - np.pi / 4)
''')
open('rb.py','w').write(s)
EOF
python3 - <<'EOF'
from rb import *
r = Robot()
tcp,_ = r.tcp_world(); print("tcp world now", np.round(tcp,4))
for fy in [0, np.pi/2]:
    sol = r.ik([-0.095,-0.156,0.60], grasp_quat(fy))
    pos,quat = r.fk(sol,"panda_hand"); R=quat_R(*quat)
    tcp,_ = r.tcp_world(sol)
    print("finger_yaw",fy,"hand y-axis",np.round(R[:,1],3),"tcp",np.round(tcp,4), "q", np.round(sol,3))
EOF

# openrua op 17
python3 -u - <<'EOF' 2>&1 | tee step_open.log
from rb import *
r = Robot()
print("fingers before", r.fingers())
r.gripper(0.04)
print("fingers after", r.fingers())
EOF

# openrua op 18
python3 - <<'EOF'
import numpy as np, rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
rclpy.init(); n=rclpy.create_node('d')
got={}
n.create_subscription(Image,'/agentview/depth/image_raw',lambda m:got.setdefault('m',m),1)
while 'm' not in got: rclpy.spin_once(n,timeout_sec=0.2)
d=CvBridge().imgmsg_to_cv2(got['m'],'passthrough'); np.save('agent_depth.npy',d)
fx=579.41
# can: around (215,255)-(213,290); for rows, find contiguous run around u=215 where depth < table-ish
for v in range(245,300,5):
    row=d[v]
    # find run containing u=215 with depth within 0.05 of d[v,215]
    c=row[215]; u0=215; u1=215
    while u0>0 and abs(row[u0-1]-c)<0.04: u0-=1; c=row[u0]
    c=row[215]
    while u1<639 and abs(row[u1+1]-c)<0.04: u1+=1; c=row[u1]
    z=row[215]
    print(v, 'depth',round(float(z),3),'run',u0,u1,'width px',u1-u0+1,'metres',round((u1-u0+1)*z/fx,4))
EOF

# openrua op 19
python3 - <<'EOF'
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
rclpy.init(); n=rclpy.create_node('u')
got={}
qos=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
n.create_subscription(String,'/robot_description',lambda m:got.setdefault('m',m),qos)
import time; t=time.time()
while 'm' not in got and time.time()-t<10: rclpy.spin_once(n,timeout_sec=0.2)
open('robot.urdf','w').write(got['m'].data if 'm' in got else '')
print(len(got.get('m',String()).data))
EOF
grep -n -A12 'name="panda_hand_joint"\|name="panda_finger_joint1"\|link name="panda_leftfinger"\|link name="panda_hand"' robot.urdf | grep -E "origin|mesh|box|cylinder|link name|joint name" | head -40

# openrua op 20
python3 -u - <<'EOF' 2>&1 | tee step1.log
from rb import *
r = Robot()
r.move_world([-0.095,-0.156,0.60], grasp_quat(0.0), seconds=4)
print(r.snap("robot0_eye_in_hand"))
print(r.snap("agentview"))
EOF

# openrua op 21
python3 - <<'EOF'
import numpy as np, rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
rclpy.init(); n=rclpy.create_node('d')
got={}
n.create_subscription(Image,'/robot0_eye_in_hand/depth/image_raw',lambda m:got.setdefault('m',m),1)
while 'm' not in got: rclpy.spin_once(n,timeout_sec=0.2)
d=CvBridge().imgmsg_to_cv2(got['m'],'passthrough'); np.save('eih_depth.npy',d)
print('center depth', d[310,318], 'table depth', d[200,318], d[240,100])
m = d < d[310,318]+0.03
# restrict to region around the can
sub = np.zeros_like(m); sub[220:400, 200:440] = m[220:400,200:440]
ys,xs=np.where(sub)
print('can px center', xs.mean(), ys.mean(), 'w', xs.max()-xs.min()+1, 'h', ys.max()-ys.min()+1, 'n', len(xs))
EOF
python3 pxw.py robot0_eye_in_hand 318,310,3

# openrua op 22
timeout 15 ros2 run tf2_ros tf2_echo world robot0_eye_in_hand_optical_frame 2>&1 | grep -A8 "Translation" | head -9; timeout 15 ros2 run tf2_ros tf2_echo panda_hand robot0_eye_in_hand_optical_frame 2>&1 | grep -A8 "Translation" | head -9

# openrua op 23
python3 pxw.py agentview 215,255,2 213,262,2 300,420,2; python3 pxw.py birdview 284,264,1 320,340,1 332,298,1; python3 pxw.py robot0_eye_in_hand 318,310,3 318,200,3 100,240,3

# openrua op 24
python3 - <<'EOF'
import numpy as np
d=np.load('eih_depth.npy')
m=(d>0.17)&(d<0.225)
m[380:,:]=False
ys,xs=np.where(m)
print('can px center', xs.mean(), ys.mean(), 'w', xs.max()-xs.min()+1, 'h', ys.max()-ys.min()+1, 'n', len(xs))
# diameter in metres at depth 0.196: fx=?
EOF
ros2 topic echo /robot0_eye_in_hand/color/camera_info --once | grep -A3 "^k:"

# openrua op 25
python3 - <<'EOF'
import numpy as np
d=np.load('eih_depth.npy')
m=(d>0.17)&(d<0.225)
sub=np.zeros_like(m); sub[220:380, 230:410]=m[220:380,230:410]
ys,xs=np.where(sub)
print('can px center', xs.mean(), ys.mean(), 'w', xs.max()-xs.min()+1, 'h', ys.max()-ys.min()+1, 'n', len(xs))
fx=312.77; z=0.196
print('diam m', (xs.max()-xs.min()+1)*z/fx, (ys.max()-ys.min()+1)*z/fx, 'area-based', 2*np.sqrt(len(xs)/np.pi)*z/fx)
EOF
python3 pxw.py robot0_eye_in_hand 318,313,2

# openrua op 26
python3 -u - <<'EOF' 2>&1 | tee step2.log
from rb import *
r = Robot()
w0 = r.wrench(); print("wrench before", np.round(w0,3))
r.move_world([-0.0895,-0.154,0.535], grasp_quat(0.0), seconds=3)
w1 = r.wrench(); print("wrench at 0.535", np.round(w1,3))
print(r.snap("robot0_eye_in_hand"))
EOF
python3 - <<'EOF'
import numpy as np, rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
rclpy.init(); n=rclpy.create_node('d')
got={}
n.create_subscription(Image,'/robot0_eye_in_hand/depth/image_raw',lambda m:got.setdefault('m',m),1)
while 'm' not in got: rclpy.spin_once(n,timeout_sec=0.2)
d=CvBridge().imgmsg_to_cv2(got['m'],'passthrough'); np.save('eih_depth2.npy',d)
print('center depth', d[311,318])
m=(d>0.10)&(d<d[311,318]+0.03)
sub=np.zeros_like(m); sub[150:420, 150:490]=m[150:420,150:490]
ys,xs=np.where(sub)
print('can px center', xs.mean(), ys.mean(), 'w', xs.max()-xs.min()+1, 'h', ys.max()-ys.min()+1, 'n', len(xs))
EOF

# openrua op 27
python3 - <<'EOF'
import numpy as np
d=np.load('eih_depth2.npy')
m=(d>0.10)&(d<0.16)
sub=np.zeros_like(m); sub[260:400, 220:420]=m[260:400,220:420]
ys,xs=np.where(sub)
print('can px center', xs.mean(), ys.mean(), 'w', xs.max()-xs.min()+1, 'h', ys.max()-ys.min()+1, 'n', len(xs))
print('cols extent', xs.min(), xs.max())
# per-row width at row 340
row=sub[340]; c=np.where(row)[0]; print('row340', c.min(), c.max(), (c.max()-c.min()+1)*0.13/312.77)
EOF
python3 pxw.py robot0_eye_in_hand 320,340,2 320,300,2

# openrua op 28
python3 - <<'EOF'
import numpy as np
d=np.load('eih_depth2.npy')
m=(d>0.10)&(d<0.136)
sub=np.zeros_like(m); sub[260:400, 220:420]=m[260:400,220:420]
ys,xs=np.where(sub)
z=0.1306; fx=312.77
print('top px center', xs.mean(), ys.mean(), 'w', xs.max()-xs.min()+1, 'h', ys.max()-ys.min()+1, 'n', len(xs))
print('diam from w', (xs.max()-xs.min()+1)*z/fx, 'from h', (ys.max()-ys.min()+1)*z/fx, 'area', 2*np.sqrt(len(xs)/np.pi)*z/fx)
cu, cv = xs.mean(), ys.mean()
# world offset of the can center relative to the optical axis
print('offset from image center (m): du', (cu-320)*z/fx, 'dv', (cv-240)*z/fx)
EOF

# openrua op 29
python3 -u - <<'EOF' 2>&1 | tee step3.log
from rb import *
r = Robot()
w0 = r.wrench(); print("wrench before", np.round(w0,3))
r.move_world([-0.0895,-0.154,0.495], grasp_quat(0.0), seconds=2)
w1 = r.wrench(); print("wrench at 0.495", np.round(w1,3))
r.move_world([-0.0895,-0.154,0.462], grasp_quat(0.0), seconds=2)
w2 = r.wrench(); print("wrench at 0.462", np.round(w2,3))
print("fingers", r.fingers())
print(r.snap("agentview"))
EOF

# openrua op 30
python3 -u - <<'EOF' 2>&1 | tee step4.log
from rb import *
r = Robot()
r.gripper(0.0)
print("wrench after close", np.round(r.wrench(),3))
tcp,_ = r.tcp_world(); print("tcp", np.round(tcp,4))
EOF

# openrua op 31
python3 -u - <<'EOF' 2>&1 | tee step5.log
from rb import *
r = Robot()
r.move_world([-0.0895,-0.154,0.70], grasp_quat(0.0), seconds=3)
print("fingers", r.fingers())
print("wrench", np.round(r.wrench(),3))
print(r.snap("agentview"))
EOF

# openrua op 32
python3 -u - <<'EOF' 2>&1 | tee step6.log
from rb import *
r = Robot()
r.move_world([-0.04,0.06,0.76], grasp_quat(0.0), seconds=3)
print("fingers", r.fingers())
r.move_world([0.008,0.272,0.74], grasp_quat(0.0), seconds=3)
print("fingers", r.fingers())
print(r.snap("agentview")); print(r.snap("robot0_eye_in_hand"))
EOF

# openrua op 33
python3 -u - <<'EOF' 2>&1 | tee step7.log
from rb import *
r = Robot()
r.move_world([0.008,0.272,0.64], grasp_quat(0.0), seconds=2.5)
print("wrench", np.round(r.wrench(),3))
r.gripper(0.04)
r.move_world([0.008,0.272,0.76], grasp_quat(0.0), seconds=2.5)
print(r.snap("agentview")); print(r.snap("robot0_eye_in_hand"))
EOF

# openrua op 34
python3 pxw.py birdview 332,298,1 320,340,1; python3 -u - <<'EOF' 2>&1 | tee step8.log
from rb import *
r = Robot()
r.move_world([0.055,0.053,0.60], grasp_quat(0.0), seconds=3)
print(r.snap("robot0_eye_in_hand"))
EOF
python3 - <<'EOF'
import numpy as np, rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
rclpy.init(); n=rclpy.create_node('d')
got={}
n.create_subscription(Image,'/robot0_eye_in_hand/depth/image_raw',lambda m:got.setdefault('m',m),1)
while 'm' not in got: rclpy.spin_once(n,timeout_sec=0.2)
d=CvBridge().imgmsg_to_cv2(got['m'],'passthrough'); np.save('eih_depth3.npy',d)
tab=np.median(d[100:200,100:540]); print('table depth', tab)
m=(d<tab-0.015)&(d>0.1)
sub=np.zeros_like(m); sub[200:400,150:490]=m[200:400,150:490]
ys,xs=np.where(sub)
z=float(np.median(d[sub]))
print('butter px center', xs.mean(), ys.mean(), 'w', xs.max()-xs.min()+1, 'h', ys.max()-ys.min()+1, 'n', len(xs), 'depth', z)
fx=312.77
print('size m', (xs.max()-xs.min()+1)*z/fx, (ys.max()-ys.min()+1)*z/fx)
print('offset from optical axis (m): du', (xs.mean()-320)*z/fx, 'dv', (ys.mean()-240)*z/fx)
EOF

# openrua op 35
python3 - <<'EOF'
import numpy as np
d=np.load('eih_depth3.npy')
tab=0.2793
m=(d<tab-0.012)&(d>0.1)
sub=np.zeros_like(m); sub[220:360,270:370]=m[220:360,270:370]
ys,xs=np.where(sub)
z=float(np.median(d[sub])); fx=312.77
print('butter px center', xs.mean(), ys.mean(), 'w', xs.max()-xs.min()+1, 'h', ys.max()-ys.min()+1, 'n', len(xs), 'depth', z, 'top z', 0.6004+0.1034-z)
print('size m', (xs.max()-xs.min()+1)*z/fx, (ys.max()-ys.min()+1)*z/fx)
print('offset from optical axis (m): du', (xs.mean()-320)*z/fx, 'dv', (ys.mean()-240)*z/fx)
EOF
python3 pxw.py robot0_eye_in_hand 319,292,2

# openrua op 36
python3 -u - <<'EOF' 2>&1 | tee step9.log
from rb import *
r = Robot()
print("wrench before", np.round(r.wrench(),3))
r.move_world([0.059,0.0537,0.50], grasp_quat(0.0), seconds=2.5)
print("wrench 0.50", np.round(r.wrench(),3))
r.move_world([0.059,0.0537,0.442], grasp_quat(0.0), seconds=2)
print("wrench 0.442", np.round(r.wrench(),3))
print("fingers", r.fingers())
print(r.snap("robot0_eye_in_hand")); print(r.snap("agentview"))
EOF

# openrua op 37
python3 -u - <<'EOF' 2>&1 | tee step10.log
from rb import *
r = Robot()
r.gripper(0.0)
print("wrench", np.round(r.wrench(),3))
r.move_world([0.059,0.0537,0.60], grasp_quat(0.0), seconds=2.5)
print("fingers", r.fingers())
print(r.snap("agentview"))
EOF

# openrua op 38
python3 -u - <<'EOF' 2>&1 | tee step11.log
from rb import *
r = Robot()
r.move_world([0.03,0.20,0.74], grasp_quat(0.0), seconds=3)
r.move_world([0.04,0.29,0.72], grasp_quat(0.0), seconds=2.5)
print("fingers", r.fingers())
r.move_world([0.04,0.29,0.64], grasp_quat(0.0), seconds=2)
print("wrench", np.round(r.wrench(),3))
r.gripper(0.04)
r.move_world([0.04,0.29,0.78], grasp_quat(0.0), seconds=2.5)
print(r.snap("agentview")); print(r.snap("robot0_eye_in_hand"))
EOF

# openrua op 39
python3 -u - <<'EOF' 2>&1 | tee step12.log
from rb import *
r = Robot()
# retreat toward home-ish pose, away from the basket
r.move_world([-0.10,0.0,0.75], grasp_quat(0.0), seconds=3)
print(r.snap("birdview")); print(r.snap("agentview"))
EOF
python3 - <<'EOF'
import numpy as np, rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
rclpy.init(); n=rclpy.create_node('d')
got={}
n.create_subscription(Image,'/birdview/depth/image_raw',lambda m:got.setdefault('m',m),1)
while 'm' not in got: rclpy.spin_once(n,timeout_sec=0.2)
d=CvBridge().imgmsg_to_cv2(got['m'],'passthrough')
cam_z = 2.575+0.425
def z(u,v): return round(float(cam_z - d[v,u]),3)
print("original soup spot (284,264): z =", z(284,264), "(table=0.425)")
print("original butter spot (332,298): z =", z(332,298), "(table=0.425)")
# basket interior region ~ u 366..400, v 272..304 ; report top surface heights
sub = d[272:305, 366:401]; hz = cam_z - sub
print("basket interior: min z", round(float(hz.min()),3), "max z", round(float(hz.max()),3))
# objects above the basket floor (~0.49) but below rim (0.62): heights of tallest points
ys,xs = np.where((hz>0.50)&(hz<0.60))
print("pixels inside basket with 0.50<z<0.60:", len(xs), " z range", round(float(hz[ys,xs].min()),3), round(float(hz[ys,xs].max()),3))
EOF

# openrua op 40
python3 -u - <<'EOF' 2>&1 | tee step13.log
from rb import *
r = Robot()
q = r.ik_world([-0.10,0.0,0.75], grasp_quat(0.0))
r.move(q, seconds=4)
print("fingers", r.fingers())
print(r.snap("agentview")); print(r.snap("birdview"))
EOF

# openrua op 41
python3 -c "
import cv2
a=cv2.imread('agentview.png'); b=cv2.imread('birdview.png')
crop=b[250:320,350:420]; crop=cv2.resize(crop,None,fx=5,fy=5,interpolation=cv2.INTER_CUBIC)
cv2.imwrite('final_basket_bird.png',crop)
cv2.imwrite('final_basket_agent.png',cv2.resize(a[190:380,430:640],None,fx=2,fy=2,interpolation=cv2.INTER_CUBIC))
"
