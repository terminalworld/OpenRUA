#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 2 -type f | head -50

# openrua op 1
cat /workspace/tools/README.md; ls -R /workspace/tools /workspace/.ros

# openrua op 2
cat tools/action/ik_move.py tools/action/fjt_send.py tools/action/gripper_cmd.py

# openrua op 3
cat tools/perception/cam_snap.py tools/perception/px2world.py

# openrua op 4
timeout 30 ros2 topic list; echo ---; timeout 30 ros2 action list; echo ---; timeout 30 ros2 service list | grep -iv parameter | head -40; echo ---; timeout 30 ros2 node list

# openrua op 5
timeout 60 ros2 topic echo /joint_states --once; for c in agentview frontview birdview sideview robot0_robotview; do timeout 60 python3 tools/perception/cam_snap.py $c; done

# openrua op 6
timeout 60 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -12; timeout 60 ros2 run tf2_ros tf2_echo world panda_hand 2>&1 | head -12; timeout 60 ros2 run tf2_ros tf2_echo world birdview_optical_frame 2>&1 | head -8

# openrua op 7
for p in "335 318" "395 285" "265 282" "265 245" "320 240" "200 350"; do echo -n "$p -> "; timeout 60 python3 tools/perception/px2world.py birdview $p; done

# openrua op 8
cat > analyze.py <<'EOF'
import numpy as np, rclpy, struct, sys
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge
rclpy.init(); node = rclpy.create_node("an")
def grab(topic, T):
    got={}
    s=node.create_subscription(T, topic, lambda m: got.setdefault("m",m), 1)
    while "m" not in got: rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(s); return got["m"]
cam = sys.argv[1] if len(sys.argv)>1 else "birdview"
d = CvBridge().imgmsg_to_cv2(grab(f"/{cam}/depth/image_raw", Image), "passthrough").astype(float)
info = grab(f"/{cam}/color/camera_info", CameraInfo)
fx,fy,cx,cy = info.k[0],info.k[4],info.k[2],info.k[5]
print("intrinsics", fx,fy,cx,cy, d.shape)
np.save(f"{cam}_depth.npy", d)
# birdview: cam at (-0.2,0,3), optical z down, image x -> world +y? check via known transform: q=(0.707,0.707,0,0)
# R = rotx(180)*... simply: world_x = -0.2 + (v-cy)*z/fy ; world_y = (u-cx)*z/fx ; world_z = 3 - z  (verified against px2world)
H,W=d.shape
vs,us=np.mgrid[0:H,0:W]
Z=3-d; X=-0.2+(vs-cy)*d/fy; Y=(us-cx)*d/fx
np.save("bird_xyz.npy", np.stack([X,Y,Z],-1))
# objects above table: Z>0.905
mask = Z>0.905
import scipy.ndimage as ndi
lab,n=ndi.label(mask)
for i in range(1,n+1):
    m=lab==i
    if m.sum()<20: continue
    print(f"blob {i}: n={m.sum()} X[{X[m].min():.3f},{X[m].max():.3f}] Y[{Y[m].min():.3f},{Y[m].max():.3f}] Zmax={Z[m].max():.3f} centroid=({X[m].mean():.3f},{Y[m].mean():.3f}) px=({us[m].mean():.0f},{vs[m].mean():.0f})")
EOF
timeout 120 python3 analyze.py

# openrua op 9
cat > pot.py <<'EOF'
import numpy as np
P=np.load("bird_xyz.npy"); X,Y,Z=P[...,0],P[...,1],P[...,2]
m=(Z>0.905)&(X>0.0)&(X<0.15)&(Y>-0.05)&(Y<0.15)
for zlo in [0.91,0.95,1.0,1.03,1.05]:
    mm=m&(Z>zlo)
    if mm.sum(): print(f"Z>{zlo}: n={mm.sum()} X[{X[mm].min():.3f},{X[mm].max():.3f}] Y[{Y[mm].min():.3f},{Y[mm].max():.3f}] c=({X[mm].mean():.3f},{Y[mm].mean():.3f})")
# ascii of heights
vs,us=np.where(m)
for v in range(vs.min(),vs.max()+1):
    row=""
    for u in range(us.min(),us.max()+1):
        z=Z[v,u]
        row+= "." if z<0.905 else str(min(9,int((z-0.9)*60)))
    print(f"{v:3d} {row}")
print("u range",us.min(),us.max())
EOF
python3 pot.py

# openrua op 10
cat > pot2.py <<'EOF'
import numpy as np
P=np.load("bird_xyz.npy"); X,Y,Z=P[...,0],P[...,1],P[...,2]
box=(X>0.0)&(X<0.15)&(Y>-0.05)&(Y<0.15)
for zlo,zhi in [(0.905,0.95),(0.95,1.0),(1.0,1.03),(1.03,1.045),(1.045,1.1)]:
    mm=box&(Z>zlo)&(Z<=zhi)
    if mm.sum(): print(f"Z in ({zlo},{zhi}]: n={mm.sum()} X[{X[mm].min():.3f},{X[mm].max():.3f}] Y[{Y[mm].min():.3f},{Y[mm].max():.3f}]")
# top lid excluding handle/spout: restrict Y to central body
top=box&(Z>1.03)
# per-row (X) Y extents
vs=np.unique(np.where(top)[0])
for v in vs:
    m=top[v]
    ys=Y[v][m]; print(f"X={X[v][m].mean():.3f} Y[{ys.min():.3f},{ys.max():.3f}] w={ys.max()-ys.min():.3f} zmax={Z[v][m].max():.3f}")
EOF
python3 pot2.py

# openrua op 11
cat > cam_xyz.py <<'EOF'
"""Save world-frame XYZ per pixel for a camera: <cam>_xyz.npy"""
import numpy as np, rclpy, sys
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener
cam=sys.argv[1]
rclpy.init(); node=rclpy.create_node("cx"); buf=Buffer(); TransformListener(buf,node)
def grab(topic,T):
    got={}; s=node.create_subscription(T,topic,lambda m:got.setdefault("m",m),1)
    while "m" not in got: rclpy.spin_once(node,timeout_sec=0.2)
    node.destroy_subscription(s); return got["m"]
d=CvBridge().imgmsg_to_cv2(grab(f"/{cam}/depth/image_raw",Image),"passthrough").astype(float)
info=grab(f"/{cam}/color/camera_info",CameraInfo)
fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
frame=f"{cam}_optical_frame"
while not buf.can_transform("world",frame,rclpy.time.Time()): rclpy.spin_once(node,timeout_sec=0.2)
t=buf.lookup_transform("world",frame,rclpy.time.Time()); q=t.transform.rotation
x,y,z,w=q.x,q.y,q.z,q.w
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
tr=np.array([t.transform.translation.x,t.transform.translation.y,t.transform.translation.z])
H,W=d.shape; vs,us=np.mgrid[0:H,0:W]
pc=np.stack([(us-cx)*d/fx,(vs-cy)*d/fy,d],-1)
P=pc@R.T+tr
np.save(f"{cam}_xyz.npy",P); print(cam,"cam at",tr, "saved")
EOF
timeout 120 python3 cam_xyz.py sideview && timeout 120 python3 cam_xyz.py frontview
cat > profile.py <<'EOF'
import numpy as np
P=np.load("sideview_xyz.npy"); X,Y,Z=P[...,0],P[...,1],P[...,2]
# target pot region
m=(X>0.0)&(X<0.14)&(Y>-0.05)&(Y<0.13)&(Z>0.905)&(Z<1.07)
for zlo in np.arange(0.905,1.06,0.01):
    mm=m&(Z>=zlo)&(Z<zlo+0.01)
    if mm.sum(): print(f"z {zlo:.3f}: n={mm.sum():3d} X[{X[mm].min():.3f},{X[mm].max():.3f}] w={X[mm].max()-X[mm].min():.3f}  Y[{Y[mm].min():.3f},{Y[mm].max():.3f}]")
EOF
python3 profile.py

# openrua op 12
python3 -c "
import cv2
im=cv2.imread('frontview.png'); c=im[300:440,280:420]; cv2.imwrite('crop_front.png', cv2.resize(c,None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('agentview.png'); c=im[250:390,280:420]; cv2.imwrite('crop_agent.png', cv2.resize(c,None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('sideview.png'); c=im[240:330,200:300]; cv2.imwrite('crop_side.png', cv2.resize(c,None,fx=5,fy=5,interpolation=cv2.INTER_CUBIC))
"

# openrua op 13
cat > handle.py <<'EOF'
import numpy as np
P=np.load("frontview_xyz.npy"); X,Y,Z=P[...,0],P[...,1],P[...,2]
m=(X>0.0)&(X<0.15)&(Y>-0.06)&(Y<0.03)&(Z>0.905)&(Z<1.07)
print("handle pts",m.sum())
for zlo in np.arange(0.905,1.06,0.005):
    mm=m&(Z>=zlo)&(Z<zlo+0.005)
    if mm.sum(): print(f"z {zlo:.3f}: n={mm.sum():3d} X[{X[mm].min():.3f},{X[mm].max():.3f}] Y[{Y[mm].min():.3f},{Y[mm].max():.3f}]")
# whole pot from frontview: Y extents by z (body)
print("--- body Y extents by z (frontview)")
m=(X>0.0)&(X<0.15)&(Y>-0.06)&(Y<0.15)&(Z>0.905)&(Z<1.07)
for zlo in np.arange(0.905,1.06,0.01):
    mm=m&(Z>=zlo)&(Z<zlo+0.01)
    if mm.sum(): print(f"z {zlo:.3f}: n={mm.sum():3d} Y[{Y[mm].min():.3f},{Y[mm].max():.3f}] w={Y[mm].max()-Y[mm].min():.3f} Xmin={X[mm].min():.3f}")
EOF
python3 handle.py

# openrua op 14
python3 -c "
import numpy as np
P=np.load('frontview_xyz.npy'); Z=P[...,2]
for v in [300,350,400,450]:
    print(v, [round(Z[v,u],3) for u in [40,120,200,320,440,520,600]])
P=np.load('birdview_xyz.npy') if False else np.load('bird_xyz.npy'); Z=P[...,2]
print('bird table Z', [round(Z[v,u],3) for v,u in [(400,320),(400,100),(400,540),(200,160),(450,470)]])
P=np.load('sideview_xyz.npy'); Z=P[...,2]
print('side table Z', [round(Z[v,u],3) for v,u in [(400,320),(380,100),(420,500),(300,150),(450,300)]])
"

# openrua op 15
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand && timeout 60 ros2 run tf2_ros tf2_echo panda_hand robot0_eye_in_hand_optical_frame 2>&1 | grep -A2 Translation | head -3

# openrua op 16
cat > urdf.py <<'EOF'
import rclpy
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
rclpy.init(); n=rclpy.create_node("u")
qos=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
got=[]
n.create_subscription(String,"/robot_description",got.append,qos)
while not got: rclpy.spin_once(n,timeout_sec=0.5)
open("robot.urdf","w").write(got[0].data); print(len(got[0].data))
EOF
timeout 60 python3 urdf.py && grep -n -i -A12 'link name="panda_\(hand\|leftfinger\|rightfinger\)"' robot.urdf | grep -i -E 'link name|geometry|mesh|box|origin|cylinder' | head -40; grep -n -B2 -A8 'joint name="panda_finger_joint1"' robot.urdf | head -30

# openrua op 17
timeout 300 python3 tools/action/gripper_cmd.py 0.04; timeout 60 ros2 topic echo /joint_states --once | grep -A9 '^position' | tail -2

# openrua op 18
mkdir -p "$(dirname /workspace/arm.py)"
cat > /workspace/arm.py <<'OPENRUA_EOF'
"""Reusable arm helpers: joint state, FK, IK, trajectory, gripper, servo."""
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
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
TW = next(a for a in M["actuators"] if a["kind"] == "cartesian_twist")
JOINTS = FJT["joints"]
TCP = M["hand"]["tcp_offset_m"]
BASE = np.array([-0.660, 0.0, 0.912])  # world -> panda_link0 (from TF)


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class Arm:
    def __init__(self, name="arm_helper"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = None
        self.node.create_subscription(JointState, "/joint_states",
                                      self._on_js, 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik_cli = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.tw_pub = self.node.create_publisher(TwistStamped, TW["port"], 10)
        self.fjt.wait_for_server(10)
        self.grip.wait_for_server(10)
        self.ik_cli.wait_for_service(10)
        self.fk_cli.wait_for_service(10)

    def _on_js(self, msg):
        self._js = msg

    def spin(self, t=0.05):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self, fresh=True):
        """dict name->position (arm + fingers)."""
        if fresh:
            self._js = None
        while self._js is None:
            self.spin(0.2)
        return dict(zip(self._js.name, self._js.position))

    def arm_q(self):
        j = self.joints()
        return [j[n] for n in JOINTS]

    def fingers(self):
        j = self.joints()
        return j["panda_finger_joint1"], j["panda_finger_joint2"]

    # ---- kinematics (planner frame = panda_link0) ----
    def fk(self, q=None, link="panda_hand"):
        """hand pose in WORLD: (xyz, quat xyzw)."""
        q = q or self.arm_q()
        req = GetPositionFK.Request()
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = list(JOINTS)
        req.robot_state.joint_state.position = list(q)
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {res and res.error_code.val}")
        p = res.pose_stamped[0].pose
        xyz = np.array([p.position.x, p.position.y, p.position.z]) + BASE
        return xyz, (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)

    def tcp(self, q=None):
        xyz, quat = self.fk(q)
        R = quat_to_R(*quat)
        return xyz + TCP * R[:, 2], quat

    def ik(self, xyz_world, quat, at_tcp=True, seed=None, timeout=60):
        """joint list for hand (or TCP) pose in world; None on failure."""
        R = quat_to_R(*quat)
        xyz = np.array(xyz_world, float)
        if at_tcp:
            xyz = xyz - TCP * R[:, 2]
        xyz = xyz - BASE  # into planner frame
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = xyz
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = quat
        req.ik_request.robot_state.joint_state.name = list(JOINTS)
        req.ik_request.robot_state.joint_state.position = list(seed or self.arm_q())
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=1)
        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        res = fut.result()
        if res is None:
            print("IK: no answer", file=sys.stderr)
            return None
        if res.error_code.val != 1:
            print(f"IK failed code={res.error_code.val}", file=sys.stderr)
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in JOINTS]

    # ---- motion ----
    def move(self, q_list, secs, verbose=True):
        """one trajectory; q_list = list of joint vectors (waypoints) or one vector."""
        if not isinstance(q_list[0], (list, tuple, np.ndarray)):
            q_list = [q_list]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        n = len(q_list)
        for i, q in enumerate(q_list):
            t = secs * (i + 1) / n
            pt = JointTrajectoryPoint(positions=[float(v) for v in q])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        if gh is None or not gh.accepted:
            raise RuntimeError("trajectory goal rejected")
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code if res.result() else None
        cur = self.arm_q()
        err = max(abs(a - b) for a, b in zip(cur, q_list[-1]))
        if verbose:
            print(f"move: error_code={code} max_joint_err={err:.4f}")
        return code, err

    def move_tcp(self, xyz, quat, secs=3.0, seed=None):
        q = self.ik(xyz, quat, at_tcp=True, seed=seed)
        if q is None:
            raise RuntimeError(f"IK failed for {xyz}")
        code, err = self.move(q, secs)
        got, _ = self.tcp()
        print(f"  tcp now {np.round(got, 4)} (target {np.round(xyz, 4)})")
        return q

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=300)
        r = res.result().result
        f = self.fingers()
        print(f"gripper({width}): reached={r.reached_goal} stalled={r.stalled} fingers={f[0]:.4f},{f[1]:.4f}")
        return f

    def servo(self, vx=0.0, vy=0.0, vz=0.0, n=20, dt=0.05):
        """stream n twist messages (base frame)."""
        msg = TwistStamped()
        msg.header.frame_id = TW["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = vx, vy, vz
        for _ in range(n):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.tw_pub.publish(msg)
            self.spin(dt)
OPENRUA_EOF

# openrua op 19
cat > step1.py <<'EOF'
import numpy as np
from arm import Arm
a = Arm()
print("q", np.round(a.arm_q(),3))
xyz,quat = a.fk(); print("hand world", np.round(xyz,4), np.round(quat,3))
t,_ = a.tcp(); print("tcp world", np.round(t,4))
# test IK for the hover pose above the handle (fingers along world X)
Q = (0.7071068, 0.7071068, 0.0, 0.0)
for cand in [Q, (0.7071068,-0.7071068,0,0)]:
    q = a.ik([0.069, -0.010, 1.20], cand)
    print("IK hover", cand, None if q is None else np.round(q,3))
EOF
timeout 300 python3 step1.py

# openrua op 20
sed -i 's/p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = quat/p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = [float(v) for v in quat]/' arm.py && sed -i 's/p.position.x, p.position.y, p.position.z = xyz$/p.position.x, p.position.y, p.position.z = [float(v) for v in xyz]/' arm.py && timeout 300 python3 -u step1.py 2>&1 | grep -v XMLPARSER

# openrua op 21
sed -i 's/^BASE = np.array(\[-0.660, 0.0, 0.912\]).*/BASE = np.array([0.0, 0.0, 0.0])  # FK\/IK poses come back in world coords on this machine (verified)/' arm.py
cat > step1.py <<'EOF'
import numpy as np
from arm import Arm
a = Arm()
q0 = a.arm_q(); print("q", np.round(q0,3))
xyz,quat = a.fk(); print("hand world", np.round(xyz,4), np.round(quat,3))
q = a.ik(xyz, quat, at_tcp=False); print("IK of current hand pose ->", None if q is None else np.round(q,3))
for cand in [(0.7071068, 0.7071068, 0.0, 0.0), (0.7071068,-0.7071068,0.0,0.0)]:
    q = a.ik([0.069, -0.010, 1.20], cand)
    print("IK hover", cand, None if q is None else np.round(q,3))
    if q is not None: print("   fk check tcp", np.round(a.tcp(q)[0],4), np.round(a.fk(q)[1],3))
EOF
timeout 300 python3 -u step1.py 2>&1 | grep -v XMLPARSER

# openrua op 22
python3 - <<'EOF'
import re
s=open('arm.py').read()
s=s.replace('''def quat_to_R(x, y, z, w):''','''def quat_mul(a, b):
    """hamilton product, xyzw."""
    x1, y1, z1, w1 = a
    x2, y2, z2, w2 = b
    return (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)


# the IK group's tip is panda_link8; panda_hand = link8 rotated -45deg about z
Q_Z45 = (0.0, 0.0, math.sin(math.pi / 8), math.cos(math.pi / 8))


def hand_to_link8(quat):
    return quat_mul(quat, Q_Z45)


def quat_to_R(x, y, z, w):''')
s=s.replace('''        """joint list for hand (or TCP) pose in world; None on failure."""
        R = quat_to_R(*quat)''','''        """joint list for HAND (or TCP) pose in world; None on failure."""
        R = quat_to_R(*quat)
        quat8 = hand_to_link8(quat)''')
s=s.replace('''        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = [float(v) for v in quat]''','''        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = [float(v) for v in quat8]''')
open('arm.py','w').write(s)
EOF
timeout 300 python3 -u step1.py 2>&1 | grep -v XMLPARSER

# openrua op 23
cat > step2.py <<'EOF'
import numpy as np
from arm import Arm
a = Arm()
Q = (0.7071068, 0.7071068, 0.0, 0.0)
a.move_tcp([0.069, -0.010, 1.20], Q, secs=4.0)
print("fingers", a.fingers())
EOF
timeout 600 python3 -u step2.py 2>&1 | grep -v XMLPARSER && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 24
cat > step2.py <<'EOF'
import numpy as np
from arm import Arm
a = Arm()
Q = (0.7071068, 0.7071068, 0.0, 0.0)
q = a.ik([0.069, -0.010, 1.20], Q)
print("target q", np.round(q,3), "current", np.round(a.arm_q(),3))
for i in range(3):
    code, err = a.move(q, 6.0)
    if err < 0.01: break
print("tcp", np.round(a.tcp()[0],4), np.round(a.fk()[1],3))
EOF
timeout 900 python3 -u step2.py 2>&1 | grep -v XMLPARSER

# openrua op 25
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand && timeout 120 python3 cam_xyz.py robot0_eye_in_hand 2>&1 | grep -v XMLPARSER

# openrua op 26
cat > handle2.py <<'EOF'
import numpy as np
P=np.load("robot0_eye_in_hand_xyz.npy"); X,Y,Z=P[...,0],P[...,1],P[...,2]
ok=np.isfinite(Z)
m=ok&(X>0.0)&(X<0.15)&(Y>-0.06)&(Y<0.15)&(Z>0.905)&(Z<1.07)
print("pot pts",m.sum())
# lid extents
lid=m&(Z>1.03)&(Y>0.02)
print(f"lid X[{X[lid].min():.4f},{X[lid].max():.4f}] Y[{Y[lid].min():.4f},{Y[lid].max():.4f}] zmax {Z[lid].max():.4f}")
knob=m&(Z>1.046)
print(f"knob X[{X[knob].min():.4f},{X[knob].max():.4f}] Y[{Y[knob].min():.4f},{Y[knob].max():.4f}] c=({X[knob].mean():.4f},{Y[knob].mean():.4f})")
h=m&(Y<0.018)
print("handle region pts",h.sum())
for ylo in np.arange(-0.035,0.02,0.005):
    mm=h&(Y>=ylo)&(Y<ylo+0.005)
    if mm.sum(): print(f"Y {ylo:.3f}: n={mm.sum():3d} X[{X[mm].min():.4f},{X[mm].max():.4f}] w={X[mm].max()-X[mm].min():.4f} Z[{Z[mm].min():.4f},{Z[mm].max():.4f}]")
EOF
python3 handle2.py

# openrua op 27
cat > step3.py <<'EOF'
import numpy as np
from arm import Arm
a = Arm()
Q = (0.7071068, 0.7071068, 0.0, 0.0)
CX, CY = 0.0719, 0.0582
q = a.ik([CX, CY, 1.12], Q)
for i in range(3):
    code, err = a.move(q, 4.0)
    if err < 0.005: break
print("tcp", np.round(a.tcp()[0],4), np.round(a.fk()[1],3), "fingers", a.fingers())
EOF
timeout 900 python3 -u step3.py 2>&1 | grep -v XMLPARSER && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand && timeout 120 python3 cam_xyz.py robot0_eye_in_hand 2>&1 | grep -v XMLPARSER

# openrua op 28
cat > check.py <<'EOF'
import numpy as np
P=np.load("robot0_eye_in_hand_xyz.npy"); X,Y,Z=P[...,0],P[...,1],P[...,2]
ok=np.isfinite(Z)&(Z>0.5)
lid=ok&(Z>1.028)&(Z<1.046)&(Y>0.02)&(Y<0.10)&(X>0.0)&(X<0.15)
print(f"lid rim X[{X[lid].min():.4f},{X[lid].max():.4f}] center {(X[lid].min()+X[lid].max())/2:.4f}  Y[{Y[lid].min():.4f},{Y[lid].max():.4f}]")
knob=ok&(Z>1.046)&(Z<1.07)&(X>0.0)&(X<0.15)
print(f"knob c=({X[knob].mean():.4f},{Y[knob].mean():.4f}) ztop={Z[knob].max():.4f}")
# fingers: points at Z between 1.10 and 1.20 (fingertips at TCP 1.12 up to hand)
f=ok&(Z>1.10)&(Z<1.19)
print("finger pts",f.sum())
for side,mm in [("left(-X)",f&(X<0.07)),("right(+X)",f&(X>0.07))]:
    if mm.sum(): print(f"{side}: X[{X[mm].min():.4f},{X[mm].max():.4f}] Y[{Y[mm].min():.4f},{Y[mm].max():.4f}] Z[{Z[mm].min():.4f},{Z[mm].max():.4f}]")
# body profile below lid, by Z
body=ok&(X>0.0)&(X<0.15)&(Y>0.02)&(Y<0.10)&(Z>0.95)&(Z<1.05)
for zlo in np.arange(0.95,1.05,0.01):
    mm=body&(Z>=zlo)&(Z<zlo+0.01)
    if mm.sum(): print(f"z {zlo:.2f}: n={mm.sum():4d} X[{X[mm].min():.4f},{X[mm].max():.4f}] w={X[mm].max()-X[mm].min():.4f}")
EOF
python3 check.py

# openrua op 29
cat > step4.py <<'EOF'
import numpy as np
from arm import Arm
a = Arm()
Q = (0.7071068, 0.7071068, 0.0, 0.0)
HX, HY = 0.0717, 0.0
q = a.ik([HX, HY, 1.10], Q)
for i in range(3):
    code, err = a.move(q, 3.0)
    if err < 0.005: break
print("tcp", np.round(a.tcp()[0],4), np.round(a.fk()[1],3))
EOF
timeout 900 python3 -u step4.py 2>&1 | grep -v XMLPARSER && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand && timeout 120 python3 cam_xyz.py robot0_eye_in_hand 2>&1 | grep -v XMLPARSER

# openrua op 30
cat > check2.py <<'EOF'
import numpy as np
P=np.load("robot0_eye_in_hand_xyz.npy"); X,Y,Z=P[...,0],P[...,1],P[...,2]
ok=np.isfinite(Z)&(Z>0.5)
h=ok&(X>0.03)&(X<0.12)&(Y>-0.04)&(Y<0.02)&(Z>0.95)&(Z<1.05)
for ylo in np.arange(-0.03,0.02,0.005):
    mm=h&(Y>=ylo)&(Y<ylo+0.005)
    if mm.sum(): print(f"Y {ylo:+.3f}: n={mm.sum():3d} X[{X[mm].min():.4f},{X[mm].max():.4f}] c={(X[mm].min()+X[mm].max())/2:.4f} Z[{Z[mm].min():.4f},{Z[mm].max():.4f}]")
# finger pads: Z near TCP 1.10..1.12
f=ok&(Z>1.098)&(Z<1.125)&(Y>-0.05)&(Y<0.05)
for side,mm in [("left(-X)",f&(X<0.07)),("right(+X)",f&(X>0.07))]:
    if mm.sum(): print(f"finger {side}: X[{X[mm].min():.4f},{X[mm].max():.4f}] Y[{Y[mm].min():.4f},{Y[mm].max():.4f}] Z[{Z[mm].min():.4f},{Z[mm].max():.4f}]")
EOF
python3 check2.py

# openrua op 31
cat > step5.py <<'EOF'
import numpy as np
from arm import Arm
a = Arm()
Q = (0.7071068, 0.7071068, 0.0, 0.0)
HX, HY = 0.0717, 0.0
q1 = a.ik([HX, HY, 1.05], Q)
q2 = a.ik([HX, HY, 1.012], Q, seed=q1)
for i in range(3):
    code, err = a.move([q1, q2], 3.0)
    if err < 0.005: break
print("tcp", np.round(a.tcp()[0],4), np.round(a.fk()[1],3))
EOF
timeout 900 python3 -u step5.py 2>&1 | grep -v XMLPARSER && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand && timeout 60 python3 tools/perception/cam_snap.py frontview

# openrua op 32
python3 -c "
import cv2
im=cv2.imread('frontview.png'); c=im[280:440,260:440]; cv2.imwrite('crop_front.png', cv2.resize(c,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))"

# openrua op 33
cat > step6.py <<'EOF'
import numpy as np
from arm import Arm
a = Arm()
f = a.gripper(0.0)
for i in range(3):
    a.spin(0.2)
print("fingers settled", a.fingers())
EOF
timeout 600 python3 -u step6.py 2>&1 | grep -v XMLPARSER

# openrua op 34
cat > step7.py <<'EOF'
import numpy as np
from arm import Arm
a = Arm()
Q = (0.7071068, 0.7071068, 0.0, 0.0)
HX, HY = 0.0717, 0.0
q1 = a.ik([HX, HY, 1.05], Q)
q2 = a.ik([HX, HY, 1.10], Q, seed=q1)
for i in range(3):
    code, err = a.move([q1, q2], 3.0)
    if err < 0.005: break
print("tcp", np.round(a.tcp()[0],4), "fingers", a.fingers())
EOF
timeout 900 python3 -u step7.py 2>&1 | grep -v XMLPARSER && timeout 60 python3 tools/perception/cam_snap.py frontview && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 35
timeout 120 python3 cam_xyz.py birdview 2>&1 | grep -v XMLPARSER; python3 - <<'EOF'
import numpy as np
P=np.load("birdview_xyz.npy"); X,Y,Z=P[...,0],P[...,1],P[...,2]
m=(Z>0.905)&(X>-0.02)&(X<0.16)&(Y>-0.06)&(Y<0.14)
import scipy.ndimage as ndi
lab,n=ndi.label(m)
for i in range(1,n+1):
    mm=lab==i
    if mm.sum()<20: continue
    print(f"blob n={mm.sum()} X[{X[mm].min():.3f},{X[mm].max():.3f}] Y[{Y[mm].min():.3f},{Y[mm].max():.3f}] Z[{Z[mm].min():.3f},{Z[mm].max():.3f}]")
# is there anything at pot's old footprint at table level+? points with Z in (0.905,1.0) near old pot center
old=(X>0.04)&(X<0.10)&(Y>0.03)&(Y<0.09)
print("old footprint Z stats:", np.round(np.percentile(Z[old],[0,25,50,75,100]),3))
EOF

# openrua op 36
timeout 120 python3 cam_xyz.py sideview 2>&1 | grep -v XMLPARSER; timeout 60 python3 tools/perception/cam_snap.py sideview; python3 - <<'EOF'
import numpy as np
P=np.load("sideview_xyz.npy"); X,Y,Z=P[...,0],P[...,1],P[...,2]
m=np.isfinite(Z)&(X>0.0)&(X<0.15)&(Y>-0.05)&(Y<0.13)&(Z>0.905)&(Z<1.25)
print("pts",m.sum())
for zlo in np.arange(0.90,1.25,0.02):
    mm=m&(Z>=zlo)&(Z<zlo+0.02)
    if mm.sum(): print(f"z {zlo:.2f}: n={mm.sum():4d} X[{X[mm].min():.3f},{X[mm].max():.3f}] Y[{Y[mm].min():.3f},{Y[mm].max():.3f}]")
EOF

# openrua op 37
cat > step8.py <<'EOF'
import numpy as np
from arm import Arm
a = Arm()
Q = (0.7071068, 0.7071068, 0.0, 0.0)
TX, TY = -0.055, -0.254
q0 = a.arm_q()
qa = a.ik([0.01, -0.13, 1.10], Q, seed=q0)
qb = a.ik([TX, TY, 1.10], Q, seed=qa)
for i in range(3):
    code, err = a.move([qa, qb], 5.0)
    if err < 0.005: break
print("tcp", np.round(a.tcp()[0],4), "fingers", a.fingers())
EOF
timeout 900 python3 -u step8.py 2>&1 | grep -v XMLPARSER && timeout 60 python3 tools/perception/cam_snap.py sideview && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 38
cat > step9.py <<'EOF'
import numpy as np, rclpy
from arm import Arm
from geometry_msgs.msg import WrenchStamped
a = Arm()
w = {}
a.node.create_subscription(WrenchStamped, "/franka_robot_state_broadcaster/external_wrench", lambda m: w.__setitem__("m", m), 1)
def wrench():
    w.pop("m", None)
    while "m" not in w: a.spin(0.2)
    f = w["m"].wrench.force; return np.round([f.x, f.y, f.z], 2)
Q = (0.7071068, 0.7071068, 0.0, 0.0)
TX, TY = -0.055, -0.254
print("wrench before", wrench())
q1 = a.ik([TX, TY, 1.065], Q)
q2 = a.ik([TX, TY, 1.050], Q, seed=q1)
for i in range(3):
    code, err = a.move([q1, q2], 3.0)
    if err < 0.005: break
print("tcp", np.round(a.tcp()[0],4), "fingers", a.fingers(), "wrench", wrench())
a.gripper(0.04)
q3 = a.ik([TX, TY, 1.15], Q, seed=q2)
for i in range(3):
    code, err = a.move(q3, 3.0)
    if err < 0.005: break
print("tcp", np.round(a.tcp()[0],4), "fingers", a.fingers())
EOF
timeout 900 python3 -u step9.py 2>&1 | grep -v XMLPARSER && timeout 60 python3 tools/perception/cam_snap.py agentview && timeout 60 python3 tools/perception/cam_snap.py sideview

# openrua op 39
timeout 120 python3 cam_xyz.py frontview 2>&1 | grep -v XMLPARSER; timeout 120 python3 cam_xyz.py sideview 2>&1 | grep -v XMLPARSER; python3 - <<'EOF'
import numpy as np
for cam in ["frontview","sideview"]:
    P=np.load(f"{cam}_xyz.npy"); X,Y,Z=P[...,0],P[...,1],P[...,2]
    # stove plate region, above the plate top 0.93, below the retreated hand (~1.10)
    m=np.isfinite(Z)&(X>-0.16)&(X<0.04)&(Y>-0.31)&(Y<-0.09)&(Z>0.935)&(Z<1.09)
    print(cam, "pot-on-stove pts", m.sum())
    for zlo in np.arange(0.93,1.09,0.02):
        mm=m&(Z>=zlo)&(Z<zlo+0.02)
        if mm.sum(): print(f"  z {zlo:.2f}: n={mm.sum():4d} X[{X[mm].min():.3f},{X[mm].max():.3f}] Y[{Y[mm].min():.3f},{Y[mm].max():.3f}]")
    knob=m&(Z>1.06)
    if knob.sum(): print(f"  knob/top c=({X[knob].mean():.3f},{Y[knob].mean():.3f}) ztop={Z[knob].max():.3f}")
    # anything left at the original pot location?
    o=np.isfinite(Z)&(X>0.02)&(X<0.12)&(Y>0.0)&(Y<0.12)&(Z>0.905)&(Z<1.2)
    print("  pts at original pot spot:", o.sum())
EOF

# openrua op 40
cat > step10.py <<'EOF'
import numpy as np
from arm import Arm
a = Arm()
Q = (0.7071068, 0.7071068, 0.0, 0.0)
q1 = a.ik([-0.055, -0.254, 1.30], Q)
home = [0.0, -0.161037389, 0.0, -2.44459747, 0.0, 2.2267522, 0.7853981633974483]
for i in range(3):
    code, err = a.move([q1, home], 6.0)
    if err < 0.01: break
print("tcp", np.round(a.tcp()[0],4))
EOF
timeout 900 python3 -u step10.py 2>&1 | grep -v XMLPARSER && timeout 60 python3 tools/perception/cam_snap.py birdview && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 41
timeout 60 ros2 topic echo /joint_states --once | grep -A9 '^position'; timeout 60 ros2 topic echo /joint_states --once | grep -A9 '^velocity' | tail -9 | tr -d ' -' | paste -sd' '

# openrua op 42
cat > step11.py <<'EOF'
import numpy as np
from arm import Arm
a = Arm()
home = [0.0, -0.161037389, 0.0, -2.44459747, 0.0, 2.2267522, 0.7853981633974483]
for i in range(4):
    code, err = a.move(home, 5.0)
    if err < 0.01: break
print("q", np.round(a.arm_q(),3), "tcp", np.round(a.tcp()[0],4))
EOF
timeout 900 python3 -u step11.py 2>&1 | grep -v XMLPARSER; timeout 60 ros2 topic echo /joint_states --once | grep -A7 '^velocity' | tail -7 | tr -d ' -' | paste -sd' '

# openrua op 43
timeout 60 python3 tools/perception/cam_snap.py agentview && rm -f *_xyz.npy bird_xyz.npy *_depth.npy

# openrua op 44
mkdir -p "$(dirname /claude-config/projects/-workspace/memory/panda-kitchen-sim-quirks.md)"
cat > /claude-config/projects/-workspace/memory/panda-kitchen-sim-quirks.md <<'OPENRUA_EOF'
---
name: panda-kitchen-sim-quirks
description: Verified quirks of the Franka Panda kitchen-sim workstation (/workspace, machine.yaml) that differ from its generic docs — IK frame, IK tip yaw offset, gripper gap, moka-pot geometry
metadata:
  type: project
---

Verified 2026-09-12 on the Panda kitchen-sim workstation (LIBERO-style scene, moka pots + stove):

- `/compute_fk` and `/compute_ik` poses are in **world** coordinates (FK of `panda_hand` matched TF `world→panda_hand`), even though docs/30-action.md says the planner frame is the arm base. Leaving `frame_id` empty is still required.
- The IK group's tip is **`panda_link8`**, not `panda_hand`: request `q_hand ⊗ Rz(+45°)` or the hand yaw comes out 45° off. `/workspace/arm.py` (my helper) does this conversion.
- Gripper max gap measured ≈ 7.75–8.0 cm (finger joint 0.0387–0.040). The moka pot's lid lip is 7.8 cm wide (its top only 7.0), so the body can't be straddled from above; the handle mount (1.9 cm) grips cleanly and the pot hangs upright.
- Long trajectories often return error_code -5 (tolerance) far from target; resending the same goal converges (as docs/30-action.md says). Joint error < 0.005 rad after a clean 0.
- Wrist camera `robot0_eye_in_hand` is offset 5 cm along hand +x; its depth at ~25 cm range gives ~0.5 mm precision — use it for final alignment instead of the 3 m birdview (3.4 mm/px).

**Why:** each of these cost several turns of debugging; the docs are generic and partly wrong for this machine.
**How to apply:** if machine.yaml shows model "Franka Emika Panda" with `/compute_ik`, `/move_action`, cameras agentview/birdview/robot0_eye_in_hand, reuse `/workspace/arm.py` if still present and trust these facts over the docs.
OPENRUA_EOF

# openrua op 45
cd /claude-config/projects/-workspace/memory && (test -f MEMORY.md && cat MEMORY.md || echo "(no index yet)"); echo "- [Panda kitchen-sim quirks](panda-kitchen-sim-quirks.md) — IK is in world frame, tip is link8 (45° yaw), gripper gap 7.75 cm, moka pot grasp by handle" >> MEMORY.md
