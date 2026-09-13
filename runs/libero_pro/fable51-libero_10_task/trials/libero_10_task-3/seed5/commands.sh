#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la && find . -maxdepth 3 -type f | head -100

# openrua op 1
cat tools/action/fjt_send.py tools/action/ik_move.py tools/action/gripper_cmd.py tools/perception/cam_snap.py tools/perception/px2world.py

# openrua op 2
ros2 topic list; echo ---; ros2 action list; echo ---; ros2 service list | grep -v parameter; echo ---; ros2 node list

# openrua op 3
ros2 topic echo /joint_states --once; for c in agentview frontview sideview birdview robot0_eye_in_hand robot0_robotview galleryview paperview; do timeout 60 python3 tools/perception/cam_snap.py $c; done

# openrua op 4
timeout 20 ros2 run tf2_ros tf2_echo world panda_hand 2>&1 | head -12; echo ---; timeout 20 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -12; echo ---; timeout 20 ros2 run tf2_ros tf2_echo world birdview_optical_frame 2>&1 | head -12

# openrua op 5
mkdir -p "$(dirname /workspace/scene.py)"
cat > /workspace/scene.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Grab depth+info+TF for a camera once; print world coords for many pixels.
Usage: python3 scene.py <camera> u,v [u,v ...]
       python3 scene.py <camera> --dump   (saves <camera>_xyz.npy HxWx3 world)
"""
import struct, sys
import numpy as np
import rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener


def grab(node, topic, T, timeout=20.0):
    got = {}
    sub = node.create_subscription(T, topic, lambda m: got.setdefault("m", m), 1)
    end = node.get_clock().now().nanoseconds / 1e9 + timeout
    while "m" not in got and node.get_clock().now().nanoseconds / 1e9 < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    return got["m"]


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def main():
    cam = sys.argv[1]
    rclpy.init()
    node = rclpy.create_node("scene")
    tfbuf = Buffer(); TransformListener(tfbuf, node)
    depth = grab(node, f"/{cam}/depth/image_raw", Image)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    D = np.frombuffer(depth.data, dtype=np.float32).reshape(depth.height, depth.width)
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    frame = f"{cam}_optical_frame"
    end = node.get_clock().now().nanoseconds / 1e9 + 10
    while node.get_clock().now().nanoseconds / 1e9 < end:
        rclpy.spin_once(node, timeout_sec=0.2)
        if tfbuf.can_transform("world", frame, rclpy.time.Time()):
            break
    t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    R = quat_R(q.x, q.y, q.z, q.w)
    p0 = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    if "--dump" in sys.argv:
        vs, us = np.mgrid[0:depth.height, 0:depth.width]
        P = np.stack([(us - cx) * D / fx, (vs - cy) * D / fy, D], -1)
        W = P @ R.T + p0
        np.save(f"{cam}_xyz.npy", W)
        print("saved", f"{cam}_xyz.npy", W.shape)
    for a in sys.argv[2:]:
        if a.startswith("--"):
            continue
        u, v = map(int, a.split(","))
        z = D[v, u]
        p = R @ np.array([(u - cx) * z / fx, (v - cy) * z / fy, z]) + p0
        print(f"px({u},{v}) depth={z:.3f} -> world {p[0]:.4f} {p[1]:.4f} {p[2]:.4f}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 6
timeout 120 python3 scene.py birdview --dump 330,250 300,300 200,400 360,300 410,300 380,270 220,270 2>&1 | tail -20

# openrua op 7
python3 -c "
import numpy as np, cv2
W=np.load('birdview_xyz.npy'); img=cv2.imread('birdview.png')
Z=W[:,:,2]
# table region: z between 0.905 and 1.5 excluding robot (x<-0.3)
X=W[:,:,0];Y=W[:,:,1]
mask=(Z>0.905)&(X>-0.35)&(X<0.6)&(np.abs(Y)<0.7)
vs,us=np.nonzero(mask)
print('objects above table: n=',mask.sum())
# cluster by y bands
for name,(y0,y1) in {'left(board)':(-0.7,-0.15),'mid(bowl/bottle)':(-0.15,0.1),'right(cabinet)':(0.1,0.7)}.items():
    m=mask&(Y>=y0)&(Y<y1)
    if m.sum()==0: continue
    print(name, 'x[%.3f,%.3f] y[%.3f,%.3f] zmax %.3f'%(X[m].min(),X[m].max(),Y[m].min(),Y[m].max(),Z[m].max()))
"


# openrua op 8
python3 -c "
import numpy as np, cv2
W=np.load('birdview_xyz.npy'); img=cv2.imread('birdview.png')
X,Y,Z=W[:,:,0],W[:,:,1],W[:,:,2]
# height histogram in cabinet region (y in 0.1..0.45, x -0.35..0.15) 
m=(Y>0.1)&(Y<0.45)&(X>-0.15)&(X<0.15)&(Z>0.905)
zs=Z[m]
h,e=np.histogram(zs,bins=30)
for c,lo in zip(h,e[:-1]): print('%.3f %d'%(lo,c))
"


# openrua op 9
python3 -c "
import numpy as np, cv2
W=np.load('birdview_xyz.npy')
X,Y,Z=W[:,:,0],W[:,:,1],W[:,:,2]
def box(m,name):
    print('%-18s n=%5d x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]'%(name,m.sum(),X[m].min(),X[m].max(),Y[m].min(),Y[m].max(),Z[m].min(),Z[m].max()))
reg=(Y>0.05)&(Y<0.5)&(X>-0.4)&(X<0.3)
box(reg&(Z>1.11)&(Z<1.13),'cabinet top')
box(reg&(Z>0.915)&(Z<0.93),'drawer floor')
box(reg&(Z>1.07)&(Z<1.11),'drawer walls?')
box(reg&(Z>1.0)&(Z<1.06),'1.0-1.06')
box(reg&(Z>0.96)&(Z<1.0),'0.96-1.0')
box(reg&(Z>1.13),'above cabinet')
# bottle: mid region, tall
mid=(Y>-0.15)&(Y<0.1)&(X>-0.35)&(X<0.15)&(Z>0.905)
box(mid&(Z>1.15)&(Z<1.3),'bottle top?')
box(mid&(Z>0.95)&(Z<1.0),'bowl?')
# drawer front (x max of drawer floor region) at various heights
"


# openrua op 10
python3 -c "
import numpy as np
W=np.load('birdview_xyz.npy')
X,Y,Z=W[:,:,0],W[:,:,1],W[:,:,2]
reg=(X>-0.2)&(X<0.2)&(Z>0.905)
print('max z per y-bin (x in -0.2..0.2):')
for y0 in np.arange(0.0,0.45,0.01):
    m=reg&(Y>=y0)&(Y<y0+0.01)
    if m.sum(): print('y=%.2f n=%4d zmax=%.3f zmed=%.3f xrange[%.3f,%.3f]'%(y0,m.sum(),Z[m].max(),np.median(Z[m]),X[m].min(),X[m].max()))
"


# openrua op 11
timeout 120 python3 scene.py robot0_eye_in_hand --dump 210,205 260,210 100,100 2>&1 | tail -5; python3 -c "
import numpy as np
W=np.load('robot0_eye_in_hand_xyz.npy')
X,Y,Z=W[:,:,0],W[:,:,1],W[:,:,2]
ok=np.isfinite(Z)
print('z range',Z[ok].min(),Z[ok].max())
m=ok&(Z>1.1)&(Z<1.4)&(X>-0.4)&(X<0.2)&(np.abs(Y)<0.15)
print('bottle top region n',m.sum())
if m.sum():
  print('x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]'%(X[m].min(),X[m].max(),Y[m].min(),Y[m].max(),Z[m].min(),Z[m].max()))
  print('center',X[m].mean(),Y[m].mean())
  h,e=np.histogram(Z[m],bins=10)
  for c,lo in zip(h,e[:-1]): print('%.3f %d'%(lo,c))
"


# openrua op 12
timeout 120 python3 scene.py agentview --dump 2>&1 | tail -1; python3 -c "
import numpy as np
W=np.load('agentview_xyz.npy')
X,Y,Z=W[:,:,0],W[:,:,1],W[:,:,2]
ok=np.isfinite(Z)
m=ok&(X>-0.3)&(X<-0.05)&(np.abs(Y)<0.08)&(Z>0.91)&(Z<1.16)
print('bottle body n',m.sum())
for z0 in np.arange(0.91,1.16,0.02):
    mm=m&(Z>=z0)&(Z<z0+0.02)
    if mm.sum(): print('z=%.2f n=%4d x[%.3f,%.3f] y[%.3f,%.3f] cx=%.3f cy=%.3f'%(z0,mm.sum(),X[mm].min(),X[mm].max(),Y[mm].min(),Y[mm].max(),X[mm].mean(),Y[mm].mean()))
m2=ok&(X>-0.3)&(X<-0.05)&(np.abs(Y)<0.08)&(Z>1.16)&(Z<1.4)
for z0 in np.arange(1.16,1.4,0.02):
    mm=m2&(Z>=z0)&(Z<z0+0.02)
    if mm.sum(): print('z=%.2f n=%4d x[%.3f,%.3f] y[%.3f,%.3f]'%(z0,mm.sum(),X[mm].min(),X[mm].max(),Y[mm].min(),Y[mm].max()))
"


# openrua op 13
python3 -c "
import numpy as np
W=np.load('robot0_eye_in_hand_xyz.npy')
X,Y,Z=W[:,:,0],W[:,:,1],W[:,:,2]
ok=np.isfinite(Z)&(X>-0.3)&(X<0.0)&(Y>-0.1)&(Y<0.15)
for z0,z1 in [(0.91,0.95),(0.95,1.0),(1.0,1.05),(1.05,1.09),(1.09,1.12),(1.12,1.14)]:
    m=ok&(Z>=z0)&(Z<z1)
    if m.sum(): print('z[%.2f,%.2f] n=%5d x[%.3f,%.3f] y[%.3f,%.3f] cx=%.3f cy=%.3f'%(z0,z1,m.sum(),X[m].min(),X[m].max(),Y[m].min(),Y[m].max(),X[m].mean(),Y[m].mean()))
"


# openrua op 14
python3 -c "
import numpy as np
W=np.load('birdview_xyz.npy')
X,Y,Z=W[:,:,0],W[:,:,1],W[:,:,2]
m=(Z>0.912)&(Z<0.93)&(Y>0.0)&(Y<0.5)&(X>-0.3)&(X<0.3)
for y0 in np.arange(0.05,0.42,0.02):
    mm=m&(Y>=y0)&(Y<y0+0.02)
    if mm.sum(): print('y=%.2f n=%4d x[%.3f,%.3f]'%(y0,mm.sum(),X[mm].min(),X[mm].max()))
print('--- table level near cabinet')
m=(Z>0.895)&(Z<0.912)&(Y>0.0)&(Y<0.5)&(X>-0.3)&(X<0.3)
for y0 in np.arange(0.05,0.42,0.04):
    mm=m&(Y>=y0)&(Y<y0+0.04)
    if mm.sum(): print('y=%.2f n=%4d x[%.3f,%.3f]'%(y0,mm.sum(),X[mm].min(),X[mm].max()))
"


# openrua op 15
python3 -c "
import numpy as np
for cam in ['robot0_eye_in_hand','agentview','birdview']:
    W=np.load(cam+'_xyz.npy')
    X,Y,Z=W[:,:,0],W[:,:,1],W[:,:,2]
    ok=np.isfinite(Z)&(X>-0.25)&(X<-0.10)&(Y>-0.01)&(Y<0.10)
    print(cam)
    for z0,z1 in [(0.905,0.93),(0.93,0.97),(0.97,1.0),(1.0,1.03),(1.03,1.06),(1.06,1.09),(1.09,1.14)]:
        m=ok&(Z>=z0)&(Z<z1)
        if m.sum(): print('  z[%.3f,%.2f] n=%5d x[%.3f,%.3f] y[%.3f,%.3f]'%(z0,z1,m.sum(),X[m].min(),X[m].max(),Y[m].min(),Y[m].max()))
"


# openrua op 16
python3 -c "
import cv2
img=cv2.imread('agentview.png'); crop=img[150:400,300:620]; cv2.imwrite('crop_drawer.png', cv2.resize(crop,None,fx=2.5,fy=2.5,interpolation=cv2.INTER_CUBIC))
img=cv2.imread('robot0_robotview.png'); crop=img[60:420,330:640]; cv2.imwrite('crop_drawer2.png', cv2.resize(crop,None,fx=2,fy=2,interpolation=cv2.INTER_CUBIC))
"

# openrua op 17
ros2 node info /robot_bridge 2>&1 | head -60; ros2 param list /robot_bridge 2>&1 | head -30

# openrua op 18
timeout 30 ros2 topic echo /tf --once 2>&1 | grep -E "frame_id|child" | sort | uniq -c; echo ---; timeout 30 ros2 topic echo /tf_static --once 2>&1 | grep -E "child_frame_id" 

# openrua op 19
timeout 60 python3 -c "
import rclpy
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
rclpy.init(); n=rclpy.create_node('tfs')
qos=QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
got=[]
n.create_subscription(TFMessage,'/tf_static',got.append,qos)
import time
for _ in range(20): rclpy.spin_once(n,timeout_sec=0.3)
for m in got:
  for t in m.transforms:
    tr=t.transform.translation; q=t.transform.rotation
    print(t.header.frame_id,'->',t.child_frame_id,'%.3f %.3f %.3f | %.3f %.3f %.3f %.3f'%(tr.x,tr.y,tr.z,q.x,q.y,q.z,q.w))
"

# openrua op 20
timeout 20 ros2 run tf2_ros tf2_echo panda_hand robot0_eye_in_hand_optical_frame 2>&1 | grep -A3 "At time" | head -8; timeout 20 ros2 topic echo /robot0_eye_in_hand/color/camera_info --once 2>&1 | grep -A12 "^k:"

# openrua op 21
python3 -c "
import numpy as np
W=np.load('birdview_xyz.npy')
X,Y,Z=W[:,:,0],W[:,:,1],W[:,:,2]
m=(Z>0.93)&(Z<1.01)&(Y>-0.2)&(Y<0.06)&(X>-0.15)&(X<0.2)
print('bowl rim n',m.sum(),'x[%.3f,%.3f] y[%.3f,%.3f] cx=%.3f cy=%.3f zmax=%.3f'%(X[m].min(),X[m].max(),Y[m].min(),Y[m].max(),X[m].mean(),Y[m].mean(),Z[m].max()))
# handle of open drawer
m=(Z>0.93)&(Z<1.0)&(Y>0.03)&(Y<0.075)&(X>-0.1)&(X<0.12)
print('handle n',m.sum(),'x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]'%(X[m].min(),X[m].max(),Y[m].min(),Y[m].max(),Z[m].min(),Z[m].max()))
# drawer front panel top
m=(Z>0.95)&(Z<1.0)&(Y>0.06)&(Y<0.1)&(X>-0.12)&(X<0.12)
print('front panel n',m.sum(),'x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]'%(X[m].min(),X[m].max(),Y[m].min(),Y[m].max(),Z[m].min(),Z[m].max()))
# wall tops
m=(Z>0.95)&(Z<1.0)&(Y>0.1)&(Y<0.2)&(X>-0.13)&(X<0.13)
print('walls n',m.sum())
for x0 in np.arange(-0.13,0.13,0.01):
    mm=m&(X>=x0)&(X<x0+0.01)
    if mm.sum(): print('  x=%.2f n=%d z[%.3f,%.3f]'%(x0,mm.sum(),Z[mm].min(),Z[mm].max()))
"


# openrua op 22
timeout 60 ros2 service call /compute_fk moveit_msgs/srv/GetPositionFK "{header: {frame_id: ''}, fk_link_names: ['panda_hand'], robot_state: {joint_state: {name: [panda_joint1,panda_joint2,panda_joint3,panda_joint4,panda_joint5,panda_joint6,panda_joint7], position: [0.0,-0.161037389,0.0,-2.44459747,0.0,2.2267522,0.7853981633974483]}}}" 2>&1 | grep -A12 "pose:" | head -20

# openrua op 23
timeout 60 ros2 service call /compute_fk moveit_msgs/srv/GetPositionFK "{header: {frame_id: ''}, fk_link_names: ['panda_hand'], robot_state: {joint_state: {name: [panda_joint1,panda_joint2,panda_joint3,panda_joint4,panda_joint5,panda_joint6,panda_joint7], position: [0.0,-0.161037389,0.0,-2.44459747,0.0,2.2267522,0.7853981633974483]}}}" 2>&1 | tail -30

# openrua op 24
mkdir -p "$(dirname /workspace/rob.py)"
cat > /workspace/rob.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Reusable robot helpers: joint state, FK, IK, trajectory, gripper, TF, camera."""
import math, struct, sys, time
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import JointState, Image, CameraInfo
from geometry_msgs.msg import WrenchStamped, TwistStamped
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from tf2_ros import Buffer, TransformListener

ARM = [f"panda_joint{i}" for i in range(1, 8)]
FJT = "/panda_arm_controller/follow_joint_trajectory"
GRIP = "/franka_gripper/gripper_action"
LIMITS = [(-2.9, 2.9), (-1.76, 1.76), (-2.9, 2.9), (-3.07, -0.07), (-2.9, 2.9), (-0.02, 3.75), (-2.9, 2.9)]


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def R_quat(R):
    """rotation matrix -> quaternion (x,y,z,w)"""
    m = R
    t = np.trace(m)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return np.array([(m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, 0.25 * s])
    i = np.argmax(np.diag(m))
    if i == 0:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        return np.array([0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s, (m[2, 1] - m[1, 2]) / s])
    if i == 1:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        return np.array([(m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s, (m[0, 2] - m[2, 0]) / s])
    s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
    return np.array([(m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s, (m[1, 0] - m[0, 1]) / s])


def frame_quat(z_axis, y_axis):
    """hand orientation from desired hand-z (approach) and hand-y (finger opening) axes in world"""
    z = np.array(z_axis, float); z /= np.linalg.norm(z)
    y = np.array(y_axis, float); y -= y.dot(z) * z; y /= np.linalg.norm(y)
    x = np.cross(y, z)
    R = np.column_stack([x, y, z])
    return R_quat(R)


class Robot:
    def __init__(self, name="rob"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = None
        self._wr = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 1)
        self.node.create_subscription(WrenchStamped, "/franka_robot_state_broadcaster/external_wrench", self._on_wr, 1)
        self.tfbuf = Buffer(); TransformListener(self.tfbuf, self.node)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT)
        self.grip = ActionClient(self.node, GripperCommand, GRIP)
        self.ik_cli = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.twist_pub = self.node.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)
        self.fjt.wait_for_server(10); self.grip.wait_for_server(10)
        self.ik_cli.wait_for_service(10); self.fk_cli.wait_for_service(10)

    def _on_js(self, m): self._js = m
    def _on_wr(self, m): self._wr = m

    def spin(self, t=0.1):
        end = time.time() + t
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def joints(self, fresh=True):
        if fresh:
            self._js = None
            while self._js is None:
                rclpy.spin_once(self.node, timeout_sec=0.2)
        d = dict(zip(self._js.name, self._js.position))
        return d

    def arm_q(self):
        d = self.joints()
        return [d[j] for j in ARM]

    def fingers(self):
        d = self.joints()
        return d["panda_finger_joint1"], d["panda_finger_joint2"]

    def wrench(self):
        self._wr = None
        end = time.time() + 3
        while self._wr is None and time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        if self._wr is None:
            return None
        f = self._wr.wrench.force; t = self._wr.wrench.torque
        return np.array([f.x, f.y, f.z, t.x, t.y, t.z])

    def fk(self, q=None, link="panda_hand"):
        if q is None:
            q = self.arm_q()
        req = GetPositionFK.Request()
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = ARM
        req.robot_state.joint_state.position = list(map(float, q))
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        r = fut.result()
        p = r.pose_stamped[0].pose
        return np.array([p.position.x, p.position.y, p.position.z]), np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])

    def ik(self, pos, quat, seed=None, timeout=20.0, attempts=1):
        req = GetPositionIK.Request()
        req.ik_request.group_name = "panda_arm"
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pos)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.robot_state.joint_state.name = ARM
        req.ik_request.robot_state.joint_state.position = list(map(float, seed if seed is not None else self.arm_q()))
        req.ik_request.timeout = Duration(sec=int(timeout), nanosec=int((timeout % 1) * 1e9))
        req.ik_request.avoid_collisions = False
        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=90)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            return None
        sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
        return [sol[j] for j in ARM]

    def move_q(self, q, seconds=3.0, via=None):
        """send trajectory; via = list of (q, t) intermediate points"""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM
        pts = []
        if via:
            for vq, vt in via:
                pt = JointTrajectoryPoint(positions=list(map(float, vq)))
                pt.time_from_start = Duration(sec=int(vt), nanosec=int((vt % 1) * 1e9))
                pts.append(pt)
        pt = JointTrajectoryPoint(positions=list(map(float, q)))
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        qn = self.arm_q()
        err = max(abs(a - b) for a, b in zip(qn, q))
        print(f"  move done code={code} max_joint_err={err:.4f}", flush=True)
        return code, err

    def move_pose(self, pos, quat, seconds=3.0, seed=None):
        q = self.ik(pos, quat, seed=seed)
        if q is None:
            print("  IK FAILED for", pos, quat, flush=True)
            return None
        self.move_q(q, seconds)
        p, _ = self.fk()
        print(f"  hand now at {p.round(4)} (target {np.round(pos,4)})", flush=True)
        return q

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = 30.0
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=120)
        r = res.result().result
        f = self.fingers()
        print(f"  gripper -> reached={r.reached_goal} stalled={r.stalled} fingers={f}", flush=True)
        return f

    def servo(self, lin, ang=(0, 0, 0), n=20, frame="panda_link0"):
        msg = TwistStamped(); msg.header.frame_id = frame
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, lin)
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = map(float, ang)
        for _ in range(n):
            self.twist_pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def snap(self, cam, out=None):
        """grab color + depth, save png, return (bgr, xyz world HxWx3)"""
        import cv2
        from cv_bridge import CvBridge
        got = {}
        subs = [self.node.create_subscription(Image, f"/{cam}/color/image_raw", lambda m: got.setdefault("c", m), 1),
                self.node.create_subscription(Image, f"/{cam}/depth/image_raw", lambda m: got.setdefault("d", m), 1),
                self.node.create_subscription(CameraInfo, f"/{cam}/color/camera_info", lambda m: got.setdefault("i", m), 1)]
        end = time.time() + 30
        while len(got) < 3 and time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        for s in subs:
            self.node.destroy_subscription(s)
        img = CvBridge().imgmsg_to_cv2(got["c"], "bgr8")
        d = got["d"]
        D = np.frombuffer(d.data, dtype=np.float32).reshape(d.height, d.width)
        k = got["i"].k
        fx, fy, cx, cy = k[0], k[4], k[2], k[5]
        frame = f"{cam}_optical_frame"
        end = time.time() + 10
        while time.time() < end and not self.tfbuf.can_transform("world", frame, rclpy.time.Time()):
            rclpy.spin_once(self.node, timeout_sec=0.2)
        t = self.tfbuf.lookup_transform("world", frame, rclpy.time.Time())
        q = t.transform.rotation
        R = quat_R(q.x, q.y, q.z, q.w)
        p0 = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
        vs, us = np.mgrid[0:d.height, 0:d.width]
        P = np.stack([(us - cx) * D / fx, (vs - cy) * D / fy, D], -1)
        W = P @ R.T + p0
        if out:
            cv2.imwrite(out, img)
            np.save(out.rsplit(".", 1)[0] + "_xyz.npy", W)
        return img, W
OPENRUA_EOF

# openrua op 25
timeout 300 python3 -u -c "
from rob import *
r=Robot()
q0=r.arm_q(); print('q0',np.round(q0,4))
p,qt=r.fk(); print('fk',p.round(4),qt.round(4))
# IK for the same pose, world coords
sol=r.ik(p,qt); print('ik world',None if sol is None else np.round(sol,4))
# IK for base-frame coords
sol2=r.ik(p-np.array([-0.66,0,0.912]),qt); print('ik base-shifted',None if sol2 is None else np.round(sol2,4))
print('fingers',r.fingers()); print('wrench',r.wrench())
"

# openrua op 26
timeout 300 python3 -u -c "
from rob import *
r=Robot()
sol=[0.0585,-0.1613,-0.0579,-2.4446,-0.0116,2.2267,0.0089]
p,qt=r.fk(sol); print('fk of ik sol',p.round(4),qt.round(4))
print(quat_R(*qt).round(3))
p,qt=r.fk(); print('fk current',p.round(4),qt.round(4)); print(quat_R(*qt).round(3))
"

# openrua op 27
timeout 600 python3 -u -c "
from rob import *
r=Robot()
p,qt=r.fk()
tests={'down, y_h=+y':frame_quat([0,0,-1],[0,1,0]),'down, y_h=+x':frame_quat([0,0,-1],[1,0,0]),'down, y_h=(1,1,0)':frame_quat([0,0,-1],[1,1,0]),'look +y tilt30, y_h=+x':frame_quat([0,0.866,-0.5],[1,0,0])}
for name,q in tests.items():
    for pos in [p, np.array([0.0,-0.03,1.08])]:
        sol=r.ik(pos,q)
        if sol is None: print(name,pos,'-> IK None'); continue
        pp,qq=r.fk(sol)
        Rw=quat_R(*q); Rs=quat_R(*qq)
        ang=np.degrees(np.arccos(np.clip((np.trace(Rw.T@Rs)-1)/2,-1,1)))
        print(f'{name} pos={pos.round(3)} -> poserr={np.linalg.norm(pp-pos):.4f} angerr={ang:.2f}deg q={np.round(sol,3)}')
"

# openrua op 28
timeout 600 python3 -u -c "
from rob import *
import rob
r=Robot()
p,qt=r.fk()
q=frame_quat([0,0,-1],[0,1,0])
# test with explicit ik_link_name
req = GetPositionIK.Request()
req.ik_request.group_name='panda_arm'; req.ik_request.ik_link_name='panda_hand'
pp=req.ik_request.pose_stamped.pose
pp.position.x,pp.position.y,pp.position.z=map(float,p)
pp.orientation.x,pp.orientation.y,pp.orientation.z,pp.orientation.w=map(float,q)
req.ik_request.robot_state.joint_state.name=ARM; req.ik_request.robot_state.joint_state.position=r.arm_q()
req.ik_request.timeout=Duration(sec=10)
fut=r.ik_cli.call_async(req); rclpy.spin_until_future_complete(r.node,fut,timeout_sec=60)
res=fut.result(); print('code',res.error_code.val)
if res.error_code.val==1:
    sol=dict(zip(res.solution.joint_state.name,res.solution.joint_state.position)); sol=[sol[j] for j in ARM]
    p2,q2=r.fk(sol); Rw=quat_R(*q); Rs=quat_R(*q2)
    print('angerr',np.degrees(np.arccos(np.clip((np.trace(Rw.T@Rs)-1)/2,-1,1))),'poserr',np.linalg.norm(p2-p))
"

# openrua op 29
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rob.py", "old_string": "        req.ik_request.group_name = \\"panda_arm\\"\\n        req.ik_request.pose_stamped.header.frame_id = \\"\\"", "new_string": "        req.ik_request.group_name = \\"panda_arm\\"\\n        req.ik_request.ik_link_name = \\"panda_hand\\"  # default tip is link8 (45deg off)\\n        req.ik_request.pose_stamped.header.frame_id = \\"\\"", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 30
timeout 60 python3 -c "
import rclpy
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy
rclpy.init(); n=rclpy.create_node('rd')
got=[]
n.create_subscription(String,'/robot_description',got.append,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
for _ in range(30):
    rclpy.spin_once(n,timeout_sec=0.3)
    if got: break
open('robot.urdf','w').write(got[0].data); print(len(got[0].data))
"; grep -n -A12 'link name="panda_hand"' robot.urdf | head -40; grep -n -B2 -A10 'link name="panda_leftfinger"' robot.urdf | head -30

# openrua op 31
cat >> /workspace/rob.py <<'EOF'


def slerp(q0, q1, t):
    q0 = np.array(q0, float); q1 = np.array(q1, float)
    d = q0.dot(q1)
    if d < 0:
        q1 = -q1; d = -d
    if d > 0.9995:
        q = q0 + t * (q1 - q0); return q / np.linalg.norm(q)
    th = math.acos(d)
    return (math.sin((1 - t) * th) * q0 + math.sin(t * th) * q1) / math.sin(th)


def move_cart(r, pos, quat, steps=6, seconds=4.0, max_jump=1.2):
    """IK along a straight line from the current hand pose; one multi-point trajectory.
    Returns final joints or None (no motion) if any IK fails / branch jumps."""
    p0, q0 = r.fk()
    seed = r.arm_q()
    pts = []
    for i in range(1, steps + 1):
        t = i / steps
        p = p0 + (np.array(pos) - p0) * t
        q = slerp(q0, quat, t)
        sol = r.ik(p, q, seed=seed)
        if sol is None:
            print(f"  IK failed at step {i}/{steps} p={p.round(3)}", flush=True)
            return None
        jump = max(abs(a - b) for a, b in zip(sol, seed))
        if jump > max_jump:
            print(f"  branch jump {jump:.2f} at step {i}; abort", flush=True)
            return None
        pts.append((sol, seconds * t))
        seed = sol
    via = pts[:-1]
    code, err = r.move_q(pts[-1][0], seconds, via=via)
    p, qn = r.fk()
    print(f"  hand at {p.round(4)} target {np.round(pos,4)} poserr={np.linalg.norm(p-np.array(pos)):.4f}", flush=True)
    return pts[-1][0]
EOF
echo ok

# openrua op 32
timeout 900 python3 -u -c "
from rob import *
r=Robot()
r.gripper(0.04)
p0,q0=r.fk(); print('start',p0.round(3))
# 1. lift straight up
move_cart(r, [p0[0],p0[1],1.36], q0, steps=3, seconds=3)
# 2. go above inspection point with view orientation (look +y, 30deg down, opening axis along x)
qv=frame_quat([0,math.cos(math.radians(30)),-math.sin(math.radians(30))],[1,0,0])
move_cart(r, [0.0,-0.06,1.30], qv, steps=6, seconds=5)
# 3. descend
move_cart(r, [0.0,-0.06,1.10], qv, steps=4, seconds=3)
print('q',np.round(r.arm_q(),3))
img,W=r.snap('robot0_eye_in_hand','inspect1.png')
print('saved')
" 2>&1 | grep -v "^\[" 

# openrua op 33
timeout 900 python3 -u -c "
from rob import *
r=Robot()
qv=frame_quat([0,math.cos(math.radians(30)),-math.sin(math.radians(30))],[1,0,0])
q=r.ik([0.0,-0.06,1.10],qv)
print('target q',np.round(q,3),'current',np.round(r.arm_q(),3))
for i in range(3):
    code,err=r.move_q(q,4.0)
    if err<0.02: break
p,qq=r.fk(); print('hand',p.round(4),qq.round(3),'want',qv.round(3))
img,W=r.snap('robot0_eye_in_hand','inspect2.png')
" 2>&1 | grep -v "^\["

# openrua op 34
python3 -c "
import numpy as np
W=np.load('inspect2_xyz.npy')
for v in range(230,300,4):
    p=W[v,320]; print(v, np.round(p,3))
print('--- column 250')
for v in range(240,300,4):
    p=W[v,250]; print(v, np.round(p,3))
"

# openrua op 35
timeout 120 python3 scene.py frontview --dump 2>&1 | tail -1; python3 -c "
import numpy as np
W=np.load('frontview_xyz.npy')
X,Y,Z=W[:,:,0],W[:,:,1],W[:,:,2]
ok=np.isfinite(Z)&(X>-0.25)&(X<-0.10)&(Y>0.0)&(Y<0.10)&(Z>0.89)
print('n',ok.sum())
for z0 in np.arange(0.89,1.16,0.01):
    m=ok&(Z>=z0)&(Z<z0+0.01)
    if m.sum(): print('z=%.2f n=%4d x[%.3f,%.3f] y[%.3f,%.3f]'%(z0,m.sum(),X[m].min(),X[m].max(),Y[m].min(),Y[m].max()))
"

# openrua op 36
timeout 120 python3 scene.py frontview --dump 2>&1 | tail -1; python3 -c "
import numpy as np
W=np.load('frontview_xyz.npy')
X,Y,Z=W[:,:,0],W[:,:,1],W[:,:,2]
ok=np.isfinite(Z)&(X>-0.25)&(X<-0.10)&(Y>0.0)&(Y<0.10)&(Z>0.89)
print('n',ok.sum())
for z0 in np.arange(0.89,1.16,0.01):
    m=ok&(Z>=z0)&(Z<z0+0.01)
    if m.sum(): print('z=%.2f n=%4d x[%.3f,%.3f] y[%.3f,%.3f]'%(z0,m.sum(),X[m].min(),X[m].max(),Y[m].min(),Y[m].max()))
"

# openrua op 37
timeout 120 python3 scene.py birdview --dump 2>&1 | tail -1; cp birdview.png birdview_old.png; timeout 60 python3 tools/perception/cam_snap.py birdview >/dev/null; python3 -c "
import numpy as np
W=np.load('birdview_xyz.npy')
X,Y,Z=W[:,:,0],W[:,:,1],W[:,:,2]
ok=np.isfinite(Z)&(X>-0.25)&(X<-0.10)&(Y>0.0)&(Y<0.10)&(Z>0.905)
print('n',ok.sum())
for z0 in np.arange(0.9,1.2,0.02):
    m=ok&(Z>=z0)&(Z<z0+0.02)
    if m.sum(): print('z=%.2f n=%4d x[%.3f,%.3f] y[%.3f,%.3f]'%(z0,m.sum(),X[m].min(),X[m].max(),Y[m].min(),Y[m].max()))
"

# openrua op 38
python3 -c "
import numpy as np
for cam in ['agentview','robot0_eye_in_hand','frontview']:
    W=np.load(cam+'_xyz.npy'); X,Y,Z=W[:,:,0],W[:,:,1],W[:,:,2]
    ok=np.isfinite(Z)
    m=ok&(X>-0.1)&(X<0.1)&(Y>0.25)&(Y<0.38)&(Z>1.05)
    print(cam,'cabinet top z: n=%d z[%.3f,%.3f] med=%.3f'%(m.sum(),Z[m].min() if m.sum() else 0,Z[m].max() if m.sum() else 0,np.median(Z[m]) if m.sum() else 0))
    m=ok&(X>-0.09)&(X<0.1)&(Y>0.10)&(Y<0.2)&(Z>0.905)&(Z<0.95)
    print(cam,'drawer floor: n=%d med z=%.3f'%(m.sum(),np.median(Z[m]) if m.sum() else 0))
    m=ok&(X>0.10)&(X<0.13)&(Y>0.10)&(Y<0.2)&(Z>0.95)&(Z<1.05)
    print(cam,'drawer right wall top: n=%d zmax=%.3f'%(m.sum(),Z[m].max() if m.sum() else 0))
    m=ok&(X>-0.12)&(X<0.12)&(Y>-0.15)&(Y<0.04)&(Z>0.95)&(Z<1.1)
    print(cam,'bowl rim: n=%d zmax=%.3f'%(m.sum(),Z[m].max() if m.sum() else 0))
"

# openrua op 39
python3 -c "
import numpy as np, struct
def stl_bbox(fn):
    d=open(fn,'rb').read()
    if d[:5]==b'solid':
        import re
        v=np.array([list(map(float,m)) for m in re.findall(rb'vertex\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)',d)])
    else:
        n=struct.unpack('<I',d[80:84])[0]
        arr=np.frombuffer(d[84:84+n*50],dtype=np.dtype([('n','<f4',3),('v','<f4',(3,3)),('a','<u2')]))
        v=arr['v'].reshape(-1,3)
    return v.min(0),v.max(0)
for f in ['hand','finger']:
    lo,hi=stl_bbox(f'/opt/ros/jazzy/share/moveit_resources_panda_description/meshes/collision/{f}.stl')
    print(f,'min',lo.round(4),'max',hi.round(4))
"

# openrua op 40
find / -iname "*hand*.stl" -o -iname "*hand*.dae" -o -iname "*finger*.stl" 2>/dev/null | grep -i -E "panda|franka" | head; find / -path /proc -prune -o -iname "*panda*" -print 2>/dev/null | grep -v -E "^/proc|\.pyc" | head -20

# openrua op 41
python3 -c "
import numpy as np
W=np.load('birdview_xyz.npy')
X,Y,Z=W[:,:,0],W[:,:,1],W[:,:,2]
m=(Z>1.03)&(Y>-0.15)&(Y<0.06)&(X>-0.2)&(X<0.2)
print('hand region n',m.sum(),'x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]'%(X[m].min(),X[m].max(),Y[m].min(),Y[m].max(),Z[m].min(),Z[m].max()))
for y0 in np.arange(-0.15,0.06,0.01):
    mm=m&(Y>=y0)&(Y<y0+0.01)
    if mm.sum(): print('y=%.2f n=%3d x[%.3f,%.3f] zmax=%.3f'%(y0,mm.sum(),X[mm].min(),X[mm].max(),Z[mm].max()))
"

# openrua op 42
python3 -c "
import numpy as np
W=np.load('birdview_xyz.npy')   # current birdview (hand away from drawer)
X,Y,Z=W[:,:,0],W[:,:,1],W[:,:,2]
for xc in [0.05,-0.05]:
    print('x=%.2f column profile:'%xc)
    m=(np.abs(X-xc)<0.004)&(Y>0.0)&(Y<0.26)
    ys=Y[m]; zs=Z[m]; o=np.argsort(ys)
    for y,z in zip(ys[o],zs[o]): print('  y=%.3f z=%.3f'%(y,z))
" | awk 'NR%2==1' | head -90

# openrua op 43
python3 -c "
import numpy as np
W=np.load('birdview_xyz.npy')
X,Y,Z=W[:,:,0],W[:,:,1],W[:,:,2]
# bowl: region x in [-0.15,0.15], y in [-0.2,0.03], z>0.905, exclude hand (z>1.03)
m=(Z>0.905)&(Z<1.03)&(X>-0.15)&(X<0.15)&(Y>-0.2)&(Y<0.03)
print('bowl n',m.sum(),'x[%.3f,%.3f] y[%.3f,%.3f] zmax %.3f'%(X[m].min(),X[m].max(),Y[m].min(),Y[m].max(),Z[m].max()))
rim=m&(Z>0.99)
print('rim n',rim.sum(),'x[%.3f,%.3f] y[%.3f,%.3f] center (%.3f,%.3f)'%(X[rim].min(),X[rim].max(),Y[rim].min(),Y[rim].max(),(X[rim].min()+X[rim].max())/2,(Y[rim].min()+Y[rim].max())/2))
# handle: y in [0.02,0.056], z>0.92
h=(Z>0.92)&(Z<0.99)&(Y>0.02)&(Y<0.056)&(X>-0.12)&(X<0.12)
print('handle n',h.sum(),'x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]'%(X[h].min(),X[h].max(),Y[h].min(),Y[h].max(),Z[h].min(),Z[h].max()))
# front panel top
p=(Z>0.975)&(Z<0.99)&(Y>0.05)&(Y<0.08)&(X>-0.13)&(X<0.13)
print('panel top n',p.sum(),'x[%.3f,%.3f] y[%.3f,%.3f]'%(X[p].min(),X[p].max(),Y[p].min(),Y[p].max()))
# side walls tops
for name,(x0,x1) in {'left wall':(-0.13,-0.09),'right wall':(0.09,0.13)}.items():
    w=(Z>0.975)&(Z<0.99)&(Y>0.08)&(Y<0.21)&(X>x0)&(X<x1)
    print(name,'n',w.sum(),'x[%.3f,%.3f] y[%.3f,%.3f]'%(X[w].min(),X[w].max(),Y[w].min(),Y[w].max()))
# cabinet footprint
c=(Z>1.1)&(X>-0.3)&(X<0.3)&(Y>0.1)
print('cabinet top x[%.3f,%.3f] y[%.3f,%.3f]'%(X[c].min(),X[c].max(),Y[c].min(),Y[c].max()))
"

# openrua op 44
python3 -c "
import numpy as np
W=np.load('birdview_xyz.npy')
X,Y,Z=W[:,:,0],W[:,:,1],W[:,:,2]
m=(Z>0.905)&(Z<1.03)&(X>-0.2)&(X<0.2)&(Y>-0.16)&(Y<0.03)
print('bowl n',m.sum(),'x[%.3f,%.3f] y[%.3f,%.3f] zmax %.3f'%(X[m].min(),X[m].max(),Y[m].min(),Y[m].max(),Z[m].max()))
for z0 in [0.91,0.95,0.98,1.0,1.01]:
    r=m&(Z>z0)
    if r.sum(): print(' z>%.2f n%d x[%.3f,%.3f] y[%.3f,%.3f]'%(z0,r.sum(),X[r].min(),X[r].max(),Y[r].min(),Y[r].max()))
# bottle
b=(Z>0.905)&(X>-0.2)&(X<-0.1)&(Y>0.0)&(Y<0.1)
print('bottle n',b.sum(),'x[%.3f,%.3f] y[%.3f,%.3f] zmax %.3f'%(X[b].min(),X[b].max(),Y[b].min(),Y[b].max(),Z[b].max()))
for z0 in [0.91,0.95,0.97,1.0,1.03,1.05]:
    r=b&(Z>z0)
    if r.sum(): print(' z>%.2f n%d x[%.3f,%.3f] y[%.3f,%.3f] c(%.3f,%.3f)'%(z0,r.sum(),X[r].min(),X[r].max(),Y[r].min(),Y[r].max(),X[r].mean(),Y[r].mean()))
"

# openrua op 45
ls -la --time-style=+%H:%M:%S *.npy *.png | head -30 && python3 -c "
import rob
r=rob.Robot()
img,W=r.snap('birdview','bv_now')
img,W=r.snap('agentview','av_now')
print(r.arm_q(), r.fingers(), r.fk())
"

# openrua op 46
python3 -c "
import rob
r=rob.Robot()
img,W=r.snap('birdview','bv_now.png')
img,W=r.snap('agentview','av_now.png')
print(r.arm_q(), r.fingers(), r.fk())
"

# openrua op 47
python3 -c "
import numpy as np
for f in ['agentview_xyz.npy','frontview_xyz.npy']:
    W=np.load(f); X,Y,Z=W[:,:,0],W[:,:,1],W[:,:,2]
    m=(Z>0.905)&(Z<1.03)&(X>-0.2)&(X<0.2)&(Y>-0.2)&(Y<0.045)
    print(f,'n',m.sum())
    for z0,z1 in [(0.905,0.93),(0.93,0.96),(0.96,0.99),(0.99,1.01),(1.01,1.03)]:
        r=m&(Z>z0)&(Z<=z1)
        if r.sum()>5: print('  z(%.3f,%.3f] n%4d x[%.3f,%.3f] y[%.3f,%.3f]'%(z0,z1,r.sum(),X[r].min(),X[r].max(),Y[r].min(),Y[r].max()))
    # upper handles: y in [0.15,0.214], z>1.0
    h=(Y>0.14)&(Y<0.212)&(Z>1.0)&(Z<1.13)&(X>-0.15)&(X<0.15)
    if h.sum(): print('  upper handles n%d x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]'%(h.sum(),X[h].min(),X[h].max(),Y[h].min(),Y[h].max(),Z[h].min(),Z[h].max()))
"

# openrua op 48
python3 -c "
import numpy as np
W=np.load('agentview_xyz.npy'); X,Y,Z=W[:,:,0],W[:,:,1],W[:,:,2]
m=(Z>0.905)&(Z<1.04)&(X>-0.1)&(X<0.2)&(Y>-0.16)&(Y<0.05)
print('n',m.sum())
for z0 in np.arange(0.905,1.04,0.01):
    r=m&(Z>z0)&(Z<=z0+0.01)
    if r.sum()>3: print('  z(%.3f] n%4d x[%.3f,%.3f] y[%.3f,%.3f]'%(z0,r.sum(),X[r].min(),X[r].max(),Y[r].min(),Y[r].max()))
"

# openrua op 49
grep -n "def \|return" /workspace/rob.py | head -60

# openrua op 50
cat >> /workspace/rob.py <<'EOF'


def hand_from_tcp(tcp, quat):
    """hand-origin position for a desired TCP position and hand orientation"""
    R = quat_R(*quat)
    return np.array(tcp, float) - 0.1034 * R[:, 2]


def tcp_now(r):
    p, q = r.fk()
    return p + 0.1034 * quat_R(*q)[:, 2]


def settle(r, q, tries=3, seconds=2.0, tol=0.01):
    """re-send a joint target until the arm has actually converged"""
    err = max(abs(a - b) for a, b in zip(r.arm_q(), q))
    for _ in range(tries):
        if err < tol:
            break
        _, err = r.move_q(q, seconds)
    return err


def go_tcp(r, tcp, quat, steps=6, seconds=4.0, max_jump=1.2):
    """straight-line move of the TCP, then settle"""
    q = move_cart(r, hand_from_tcp(tcp, quat), quat, steps=steps, seconds=seconds, max_jump=max_jump)
    if q is None:
        return None
    settle(r, q)
    print(f"  TCP now {tcp_now(r).round(4)} target {np.round(tcp,4)}", flush=True)
    return q
EOF
cat > /workspace/s1_grasp.py <<'EOF'
import numpy as np, rob, sys
r = rob.Robot()
print("start q", np.round(r.arm_q(),3), "fingers", r.fingers(), flush=True)
p0, q0 = r.fk()
# 1. lift straight up above the bowl
print("== lift", flush=True)
if rob.move_cart(r, p0 + [0, 0, 0.14], q0, steps=4, seconds=3.0) is None: sys.exit("lift failed")
# 2. pre-pre-grasp: approach +x, fingers along y (hand y = -y), well behind the bottle
Q = rob.frame_quat([1, 0, 0], [0, -1, 0])
print("== to staging", flush=True)
q = r.move_pose(rob.hand_from_tcp([-0.32, 0.046, 1.15], Q), Q, seconds=6.0)
if q is None: sys.exit("staging IK failed")
rob.settle(r, q)
print("  TCP", rob.tcp_now(r).round(4), flush=True)
r.gripper(0.04)
# 3. pre-grasp
print("== pre-grasp", flush=True)
if rob.go_tcp(r, [-0.25, 0.046, 1.03], Q, steps=4, seconds=4.0) is None: sys.exit("pregrasp failed")
# 4. advance to the neck
print("== advance", flush=True)
if rob.go_tcp(r, [-0.150, 0.046, 1.03], Q, steps=4, seconds=4.0) is None: sys.exit("advance failed")
r.snap("robot0_eye_in_hand", "eih_pregrasp.png")
print("wrench before", r.wrench(), flush=True)
print("== close", flush=True)
f = r.gripper(0.0)
print("wrench after", r.wrench(), flush=True)
print("fingers", f, "q", np.round(r.arm_q(),3), flush=True)
print("DONE", flush=True)
EOF
nohup python3 -u s1_grasp.py > s1.log 2>&1 &
echo started

# openrua op 51
sleep 45; cat s1.log

# openrua op 52
sleep 8; cat s1.log; 

# openrua op 53
pkill -f s1_grasp.py; sleep 1; cat s1.log | tail -5; python3 -c "
import rob,numpy as np
r=rob.Robot()
print('q',np.round(r.arm_q(),3)); p,q=r.fk(); print('hand',p.round(3), 'R\n',rob.quat_R(*q).round(2)); print('tcp',rob.tcp_now(r).round(3)); print('fingers',r.fingers())
"

# openrua op 54
cat s1.log | tail -5; python3 -c "
import rob,numpy as np
r=rob.Robot()
print('q',np.round(r.arm_q(),3)); p,q=r.fk(); print('hand',p.round(3), 'R\n',rob.quat_R(*q).round(2)); print('tcp',rob.tcp_now(r).round(3)); print('fingers',r.fingers())
"

# openrua op 55
python3 -c "
import rob
r=rob.Robot()
r.snap('birdview','bv2.png'); r.snap('agentview','av2.png')
"

# openrua op 56
python3 -c "
import rob
r=rob.Robot()
r.snap('birdview','bv2.png'); r.snap('agentview','av2.png')
"

# openrua op 57
cat >> /workspace/rob.py <<'EOF'


def ik_near(r, pos, quat, seed=None, max_jump=1.0, tries=8, timeout=3.0):
    """IK solution close (in joint space) to the seed; perturbs the seed on retries"""
    seed = list(r.arm_q() if seed is None else seed)
    rng = np.random.default_rng(0)
    best = None
    for i in range(tries):
        s = seed if i == 0 else list(np.array(seed) + rng.normal(0, 0.15, 7))
        sol = r.ik(pos, quat, seed=s, timeout=timeout)
        if sol is None:
            continue
        jump = max(abs(a - b) for a, b in zip(sol, seed))
        if best is None or jump < best[0]:
            best = (jump, sol)
        if jump <= max_jump:
            return sol
    print(f"  ik_near: no solution within {max_jump} (best jump {best[0] if best else None})", flush=True)
    return None
EOF
python3 -c "
import rob,numpy as np
r=rob.Robot()
q=r.arm_q(); print('q',np.round(q,3))
for l in ['panda_link2','panda_link3','panda_link4','panda_link5','panda_link6','panda_link7','panda_hand']:
    p,_=r.fk(q,l); print(l,p.round(3))
"

# openrua op 58
python3 -c "
import rob,numpy as np
r=rob.Robot()
q=r.arm_q()
def show(tag,q):
    ps=[r.fk(q,l)[0] for l in ['panda_link4','panda_link6','panda_hand']]
    tcp=r.fk(q)[0]+0.1034*rob.quat_R(*r.fk(q)[1])[:,2]
    print(tag,np.round(q,2),'elbow',ps[0].round(2),'wrist',ps[1].round(2),'hand',ps[2].round(2),'tcp',tcp.round(2))
show('now',q)
qa=list(q); qa[1]=0.6; show('A j2=0.6',qa)
qb=list(qa); qb[3]=-2.2; show('B j4=-2.2',qb)
qc=list(qb); qc[0]=0.0; show('C j1=0',qc)
seed=[0,-0.3,0,-2.0,0,1.7,0.785]
show('seed',seed)
Qd=rob.frame_quat([0,0,-1],[0,1,0])
sol=rob.ik_near(r,rob.hand_from_tcp([-0.3,0.0,1.2],Qd),Qd,seed=seed,max_jump=1.5)
if sol: show('target',sol)
"

# openrua op 59
cat >> /workspace/rob.py <<'EOF'


LINKS = ["panda_link3", "panda_link4", "panda_link5", "panda_link6", "panda_link7", "panda_hand"]


def path_check(r, q0, q1, n=10, zmin=1.0, boxes=()):
    """FK-sample the joint-space interpolation q0->q1; report min z of links/tcp and box hits.
    boxes: list of (name, xmin,xmax,ymin,ymax,zmin,zmax) forbidden for tcp/hand/link7"""
    worst = 9
    hits = []
    for i in range(n + 1):
        q = [a + (b - a) * i / n for a, b in zip(q0, q1)]
        pts = {}
        for l in LINKS:
            pts[l] = r.fk(q, l)[0]
        p, qq = r.fk(q)
        pts["tcp"] = p + 0.1034 * quat_R(*qq)[:, 2]
        pts["palm"] = p + 0.058 * quat_R(*qq)[:, 2]
        for k, v in pts.items():
            worst = min(worst, v[2])
            for b in boxes:
                if b[1] <= v[0] <= b[2] and b[3] <= v[1] <= b[4] and b[5] <= v[2] <= b[6]:
                    hits.append((i, k, b[0]))
    print(f"  path_check: min z {worst:.3f} hits {hits[:6]}", flush=True)
    return worst, hits


CABINET = ("cabinet", -0.14, 0.15, 0.03, 0.42, 0.85, 1.14)
SHELF = ("shelf", -0.35, -0.05, -0.62, -0.16, 0.85, 1.20)
EOF
python3 -c "
import rob,numpy as np
r=rob.Robot()
q=r.arm_q()
Qd=rob.frame_quat([0,0,-1],[0,1,0])
for seed in ([0,0.3,0,-2.2,0,2.5,0.785],[0,0.5,0,-2.4,0,2.9,0.785],[0.2,0.8,-0.3,-2.5,-0.3,3.0,0.5]):
    sol=rob.ik_near(r,rob.hand_from_tcp([-0.3,0.0,1.15],Qd),Qd,seed=seed,max_jump=1.5,tries=4)
    if sol:
        print('sol',np.round(sol,2),'jump from now',round(max(abs(a-b) for a,b in zip(sol,q)),2))
        rob.path_check(r,q,sol,boxes=[rob.CABINET,rob.SHELF])
"

# openrua op 60
python3 -c "
import rob,numpy as np
r=rob.Robot()
Qd=rob.frame_quat([0,0,-1],[0,1,0])
for seed in ([0,0.3,0,-2.2,0,2.5,0.785],[0,0.5,0,-2.4,0,2.9,0.785],[0.2,0.8,-0.3,-2.5,-0.3,3.0,0.5],r.arm_q()):
    for k in range(2):
        sol=r.ik(rob.hand_from_tcp([-0.3,0.0,1.15],Qd),Qd,seed=seed,timeout=3)
        print('seed',np.round(seed,2),'->',None if sol is None else np.round(sol,2))
"

# openrua op 61
python3 -c "
import rob,numpy as np
r=rob.Robot()
q=r.arm_q()
Qd=rob.frame_quat([0,0,-1],[0,-1,0])
for tcp in ([-0.2,0.0,1.15],[-0.25,0.0,1.2]):
  for seed in ([0,0.3,0,-2.2,0,2.5,0.785],[0,-0.3,0,-2.0,0,1.7,0.785],[-0.3,0.5,0.3,-2.0,0,2.5,0.785]):
    sol=r.ik(rob.hand_from_tcp(tcp,Qd),Qd,seed=seed,timeout=3)
    print(tcp,'seed',np.round(seed,2),'->',None if sol is None else np.round(sol,2), 'jump from now', None if sol is None else round(max(abs(a-b) for a,b in zip(sol,q)),2))
"

# openrua op 62
python3 -c "
import rob,numpy as np
r=rob.Robot()
q=r.arm_q()
home=[0,-0.13,0,-2.48,0,2.34,0.79]
boxes=[rob.CABINET,rob.SHELF,('bottle',-0.25,-0.12,0.03,0.25,0.85,1.0)]
print('direct'); rob.path_check(r,q,home,n=12,boxes=boxes)
# staged: first lift shoulder (j2) and fold, then rotate j1, then rest
s1=list(q); s1[1]=0.3; s1[3]=-2.7; s1[5]=2.0
print('stage1'); rob.path_check(r,q,s1,n=8,boxes=boxes)
s2=list(s1); s2[0]=0.0; s2[2]=0.0; s2[4]=0.0
print('stage2'); rob.path_check(r,s1,s2,n=8,boxes=boxes)
print('stage3'); rob.path_check(r,s2,home,n=8,boxes=boxes)
for tag,qq in [('s1',s1),('s2',s2),('home',home)]:
    p,qo=r.fk(qq); print(tag,'hand',p.round(2),'tcp',(p+0.1034*rob.quat_R(*qo)[:,2]).round(2))
"

# openrua op 63
python3 -c "
import rob,numpy as np
r=rob.Robot()
q=r.arm_q()
home=[0,-0.13,0,-2.48,0,2.34,0.79]
boxes=[rob.CABINET,rob.SHELF,('bottle',-0.25,-0.12,0.03,0.25,0.85,1.0)]
for j6,j4 in [(1.6,-2.7),(1.2,-2.9),(2.0,-2.9)]:
    s1=list(q); s1[1]=0.3; s1[3]=j4; s1[5]=j6
    s2=list(s1); s2[0]=0.0; s2[2]=0.0; s2[4]=0.0
    print('j6',j6,'j4',j4)
    a=rob.path_check(r,q,s1,n=8,boxes=boxes)[0]; b=rob.path_check(r,s1,s2,n=8,boxes=boxes)[0]; c=rob.path_check(r,s2,home,n=8,boxes=boxes)[0]
"

# openrua op 64
cat > /workspace/s2_recover.py <<'EOF'
import numpy as np, rob, sys
r = rob.Robot()
q = r.arm_q(); print("start", np.round(q,3), flush=True)
home = [0, -0.13, 0, -2.48, 0, 2.34, 0.79]
s1 = list(q); s1[1] = 0.3; s1[3] = -2.9; s1[5] = 1.2
s2 = list(s1); s2[0] = 0.0; s2[2] = 0.0; s2[4] = 0.0
for tag, qq in [("s1", s1), ("s2", s2), ("home", home)]:
    print("==", tag, flush=True)
    r.move_q(qq, 6.0)
    err = rob.settle(r, qq, seconds=3.0)
    print("  settled err", round(err,4), "tcp", rob.tcp_now(r).round(3), flush=True)
r.snap("birdview", "bv3.png"); r.snap("agentview", "av3.png")
print("DONE", flush=True)
EOF
nohup python3 -u s2_recover.py > s2.log 2>&1 &
sleep 50; cat s2.log

# openrua op 65
sleep 40; cat s2.log

# openrua op 66
sleep 30; cat s2.log | tail -8

# openrua op 67
python3 -c "
import numpy as np
W=np.load('bv3_xyz.npy'); X,Y,Z=W[:,:,0],W[:,:,1],W[:,:,2]
b=(Z>0.905)&(Z<1.0)&(X>-0.3)&(X<-0.13)&(Y>0.03)&(Y<0.3)
print('bottle n',b.sum(),'x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]'%(X[b].min(),X[b].max(),Y[b].min(),Y[b].max(),Z[b].min(),Z[b].max()))
for y0 in np.arange(0.05,0.24,0.01):
    s=b&(Y>=y0)&(Y<y0+0.01)
    if s.sum(): print(' y%.2f n%3d x[%.3f,%.3f] xc %.3f zmax %.3f'%(y0,s.sum(),X[s].min(),X[s].max(),(X[s].min()+X[s].max())/2,Z[s].max()))
"

# openrua op 68
python3 - <<'EOF'
s=open('rob.py').read()
s=s.replace('''    def move_pose(self, pos, quat, seconds=3.0, seed=None):
        q = self.ik(pos, quat, seed=seed)''','''    def move_pose(self, pos, quat, seconds=3.0, seed=None, max_jump=1.0):
        q = ik_near(self, pos, quat, seed=seed, max_jump=max_jump)''')
s=s.replace('''        sol = r.ik(p, q, seed=seed)
        if sol is None:
            print(f"  IK failed at step {i}/{steps} p={p.round(3)}", flush=True)
            return None''','''        sol = ik_near(r, p, q, seed=seed, max_jump=max_jump)
        if sol is None:
            print(f"  IK failed at step {i}/{steps} p={p.round(3)}", flush=True)
            return None''')
open('rob.py','w').write(s)
EOF
python3 -c "
import rob,numpy as np
r=rob.Robot()
q=r.arm_q(); print('q',np.round(q,3),'tcp',rob.tcp_now(r).round(3))
# push pose test
b=np.radians(30)
Qp=rob.frame_quat([np.sin(b),0,-np.cos(b)],[0,1,0])
print('R push\n',rob.quat_R(*Qp).round(2))
for tcp in ([-0.135,0.10,1.05],[-0.135,0.10,0.925],[-0.20,0.10,0.925]):
    sol=rob.ik_near(r,rob.hand_from_tcp(tcp,Qp),Qp,seed=q,max_jump=1.5)
    print(tcp,None if sol is None else np.round(sol,2))
    if sol: q=sol
"

# openrua op 69
python3 -c "
import rob,numpy as np
r=rob.Robot()
q=r.arm_q()
b=np.radians(30)
Qp=rob.frame_quat([np.sin(b),0,-np.cos(b)],[0,-1,0])
print('R push\n',rob.quat_R(*Qp).round(2))
for tcp in ([-0.135,0.10,1.05],[-0.135,0.10,0.925],[-0.20,0.10,0.925]):
    sol=rob.ik_near(r,rob.hand_from_tcp(tcp,Qp),Qp,seed=q,max_jump=1.5)
    print(tcp,None if sol is None else np.round(sol,2))
    if sol: q=sol
# grasp pose after roll: fingers along x (y_h=+x -> x_h=+y), hand down
Qg=rob.frame_quat([0,0,-1],[1,0,0]); print('R grasp\n',rob.quat_R(*Qg).round(2))
sol=rob.ik_near(r,rob.hand_from_tcp([-0.24,0.185,0.916],Qg),Qg,seed=q,max_jump=2.0); print('grasp',None if sol is None else np.round(sol,2))
Qg2=rob.frame_quat([0,0,-1],[-1,0,0]); print('R grasp2\n',rob.quat_R(*Qg2).round(2))
sol=rob.ik_near(r,rob.hand_from_tcp([-0.24,0.185,0.916],Qg2),Qg2,seed=q,max_jump=2.0); print('grasp2',None if sol is None else np.round(sol,2))
"

# openrua op 70
python3 -c "
import numpy as np
W=np.load('bv3_xyz.npy'); X,Y,Z=W[:,:,0],W[:,:,1],W[:,:,2]
for z0,z1 in [(1.15,1.2),(1.2,1.25),(1.25,1.3),(1.3,1.35),(1.35,1.42)]:
    m=(Z>z0)&(Z<z1)&(X>-0.4)&(X<0.0)&(abs(Y)<0.2)
    if m.sum(): print('z(%.2f,%.2f) n%4d x[%.3f,%.3f] y[%.3f,%.3f]'%(z0,z1,m.sum(),X[m].min(),X[m].max(),Y[m].min(),Y[m].max()))
# fingers region z 1.146..1.2
m=(Z>1.13)&(Z<1.19)&(X>-0.3)&(X<-0.1)&(abs(Y)<0.15)
print('fingers n',m.sum(),'x[%.3f,%.3f] y[%.3f,%.3f]'%(X[m].min(),X[m].max(),Y[m].min(),Y[m].max()))
"

# openrua op 71
cat > /workspace/s3_push.py <<'EOF'
import numpy as np, rob, sys
r = rob.Robot()
print("start q", np.round(r.arm_q(),3), "tcp", rob.tcp_now(r).round(3), flush=True)
r.gripper(0.0)
b = np.radians(15)
Qp = rob.frame_quat([np.sin(b), 0, -np.cos(b)], [0, -1, 0])
print("== above neck", flush=True)
q = r.move_pose(rob.hand_from_tcp([-0.155, 0.185, 1.05], Qp), Qp, seconds=6.0, max_jump=1.5)
if q is None: sys.exit("fail")
rob.settle(r, q); print("  tcp", rob.tcp_now(r).round(4), "q", np.round(r.arm_q(),2), flush=True)
print("== descend", flush=True)
if rob.go_tcp(r, [-0.155, 0.185, 0.921], Qp, steps=4, seconds=4.0, max_jump=0.8) is None: sys.exit("fail")
print("== push -x", flush=True)
if rob.go_tcp(r, [-0.225, 0.185, 0.921], Qp, steps=4, seconds=5.0, max_jump=0.8) is None: sys.exit("fail")
print("== retreat up", flush=True)
if rob.go_tcp(r, [-0.225, 0.185, 1.08], Qp, steps=3, seconds=3.0, max_jump=0.8) is None: sys.exit("fail")
r.snap("birdview", "bv4.png")
print("DONE", flush=True)
EOF
nohup python3 -u s3_push.py > s3.log 2>&1 &
sleep 55; cat s3.log

# openrua op 72
sleep 25; cat s3.log | tail -8

# openrua op 73
python3 -c "
import numpy as np
W=np.load('bv4_xyz.npy'); X,Y,Z=W[:,:,0],W[:,:,1],W[:,:,2]
b=(Z>0.905)&(Z<1.0)&(X>-0.4)&(X<-0.13)&(Y>0.0)&(Y<0.3)
print('bottle n',b.sum(),'x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]'%(X[b].min(),X[b].max(),Y[b].min(),Y[b].max(),Z[b].min(),Z[b].max()))
for y0 in np.arange(0.03,0.26,0.01):
    s=b&(Y>=y0)&(Y<y0+0.01)
    if s.sum(): print(' y%.2f n%3d x[%.3f,%.3f] xc %.3f zmax %.3f'%(y0,s.sum(),X[s].min(),X[s].max(),(X[s].min()+X[s].max())/2,Z[s].max()))
"

# openrua op 74
cat > /workspace/s4_look.py <<'EOF'
import numpy as np, rob, sys
r = rob.Robot()
Q = rob.frame_quat([0, 0, -1], [0, -1, 0])
q = r.move_pose(rob.hand_from_tcp([-0.30, -0.02, 1.15], Q), Q, seconds=5.0, max_jump=1.5)
rob.settle(r, q); print("tcp", rob.tcp_now(r).round(3), "q", np.round(r.arm_q(),2), flush=True)
r.snap("birdview", "bv5.png")
print("DONE", flush=True)
EOF
python3 -u s4_look.py 2>&1 | tail -3
python3 -c "
import numpy as np
W=np.load('bv5_xyz.npy'); X,Y,Z=W[:,:,0],W[:,:,1],W[:,:,2]
b=(Z>0.905)&(Z<1.0)&(X>-0.45)&(X<-0.13)&(Y>0.0)&(Y<0.35)
print('bottle n',b.sum(),'x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]'%(X[b].min(),X[b].max(),Y[b].min(),Y[b].max(),Z[b].min(),Z[b].max()))
for y0 in np.arange(0.05,0.3,0.01):
    s=b&(Y>=y0)&(Y<y0+0.01)
    if s.sum(): print(' y%.2f n%3d x[%.3f,%.3f] xc %.3f zmax %.3f'%(y0,s.sum(),X[s].min(),X[s].max(),(X[s].min()+X[s].max())/2,Z[s].max()))
"

# openrua op 75
python3 -c "
import numpy as np
W=np.load('bv5_xyz.npy'); X,Y,Z=W[:,:,0],W[:,:,1],W[:,:,2]
b=(Z>0.905)&(Z<1.0)&(X>-0.45)&(X<-0.13)&(Y>0.0)&(Y<0.35)
P=np.stack([X[b],Y[b]],1); c=P.mean(0); U,S,Vt=np.linalg.svd(P-c); a=Vt[0]
if a[1]<0: a=-a
t=(P-c)@a; print('axis a',a.round(3),'angle deg',np.degrees(np.arctan2(a[1],a[0])).round(1),'t range',t.min().round(3),t.max().round(3),'center',c.round(3))
# neck points: thin, z<0.935
n=b&(Z<0.935)&(Y>0.14)
Pn=np.stack([X[n],Y[n]],1); tn=(Pn-c)@a
print('neck t range',tn.min().round(3),tn.max().round(3),'n',n.sum())
# perpendicular offsets of neck points
perp=np.array([-a[1],a[0]]); print('neck perp mean',((Pn-c)@perp).mean().round(4),'body perp mean',((P[t<0.02]-c)@perp).mean().round(4))
# neck axis point at t=0.065 from center
for tt in [0.05,0.06,0.07]:
    print('t',tt,'pt',(c+tt*a).round(4))
print('cap end approx',(c+t.max()*a).round(3),'base end',(c+t.min()*a).round(3))
"

# openrua op 76
cat > /workspace/s5_grasp.py <<'EOF'
import numpy as np, rob, sys
r = rob.Robot()
a = np.array([-0.796, 0.606, 0.0]); a /= np.linalg.norm(a)
f = np.array([a[1], -a[0], 0.0])           # perpendicular; choose sign so x_h = a
Qg = rob.frame_quat([0, 0, -1], f)
R = rob.quat_R(*Qg); print("x_h", R[:,0].round(3), "y_h", R[:,1].round(3), flush=True)
if R[:,0].dot(a) < 0.9:
    f = -f; Qg = rob.frame_quat([0, 0, -1], f); R = rob.quat_R(*Qg); print("flipped: x_h", R[:,0].round(3), flush=True)
pt = np.array([-0.240, 0.168])
print("== open", flush=True); r.gripper(0.04)
print("== above neck", flush=True)
q = r.move_pose(rob.hand_from_tcp([pt[0], pt[1], 1.05], Qg), Qg, seconds=8.0, max_jump=2.6)
if q is None: sys.exit("fail")
rob.settle(r, q); print("  tcp", rob.tcp_now(r).round(4), "q", np.round(r.arm_q(),2), flush=True)
print("== descend", flush=True)
if rob.go_tcp(r, [pt[0], pt[1], 0.913], Qg, steps=4, seconds=4.0, max_jump=0.8) is None: sys.exit("fail")
r.snap("robot0_eye_in_hand", "eih_grasp.png")
print("wrench before", None if r.wrench() is None else r.wrench().round(2), flush=True)
print("== close", flush=True)
fg = r.gripper(0.0)
print("wrench after", None if r.wrench() is None else r.wrench().round(2), flush=True)
print("== lift 3cm", flush=True)
if rob.go_tcp(r, [pt[0], pt[1], 0.945], Qg, steps=2, seconds=3.0, max_jump=0.8) is None: sys.exit("fail")
print("fingers", r.fingers(), "wrench", None if r.wrench() is None else r.wrench().round(2), flush=True)
r.snap("birdview", "bv6.png")
print("DONE", flush=True)
EOF
nohup python3 -u s5_grasp.py > s5.log 2>&1 &
sleep 60; cat s5.log

# openrua op 77
sleep 30; cat s5.log | tail -12

# openrua op 78
python3 -c "
import numpy as np
W=np.load('bv6_xyz.npy'); X,Y,Z=W[:,:,0],W[:,:,1],W[:,:,2]
b=(Z>0.905)&(Z<1.02)&(X>-0.45)&(X<-0.1)&(Y>0.0)&(Y<0.35)
print('bottle n',b.sum(),'x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]'%(X[b].min(),X[b].max(),Y[b].min(),Y[b].max(),Z[b].min(),Z[b].max()))
for y0 in np.arange(0.05,0.3,0.01):
    s=b&(Y>=y0)&(Y<y0+0.01)
    if s.sum(): print(' y%.2f n%3d x[%.3f,%.3f] xc %.3f z[%.3f,%.3f]'%(y0,s.sum(),X[s].min(),X[s].max(),(X[s].min()+X[s].max())/2,Z[s].min(),Z[s].max()))
"

# openrua op 79
python3 -c "
import cv2
im=cv2.imread('bv6.png'); c=im[200:330,240:420]; c=cv2.resize(c,None,fx=3,fy=3,interpolation=cv2.INTER_NEAREST); cv2.imwrite('bv6_crop.png',c)
"

# openrua op 80
python3 -c "
import numpy as np
W=np.load('bv6_xyz.npy'); X,Y,Z=W[:,:,0],W[:,:,1],W[:,:,2]
b=(Z>0.905)&(Z<1.05)&(X>-0.19)&(X<-0.10)&(Y>0.05)&(Y<0.13)
print('base region n',b.sum())
for z0 in np.arange(0.90,1.05,0.01):
    s=b&(Z>=z0)&(Z<z0+0.01)
    if s.sum(): print(' z%.2f n%3d x[%.3f,%.3f] y[%.3f,%.3f]'%(z0,s.sum(),X[s].min(),X[s].max(),Y[s].min(),Y[s].max()))
"

# openrua op 81
cat > /workspace/s6_place.py <<'EOF'
import numpy as np, rob, sys
r = rob.Robot()
p, q0 = r.fk(); print("start tcp", rob.tcp_now(r).round(4), "fingers", r.fingers(), flush=True)
Qg = q0
Qf = rob.frame_quat([0, 0, -1], [0, -1, 0])   # hand down, x_h=+x, fingers along y
print("== lift to 1.10", flush=True)
if rob.go_tcp(r, [-0.24, 0.168, 1.10], Qg, steps=3, seconds=4.0, max_jump=0.8) is None: sys.exit("fail")
print("fingers", r.fingers(), flush=True)
r.snap("agentview", "av7.png")
print("== yaw to x", flush=True)
if rob.go_tcp(r, [-0.24, 0.168, 1.10], Qf, steps=8, seconds=14.0, max_jump=0.9) is None: sys.exit("fail")
print("fingers", r.fingers(), "q", np.round(r.arm_q(),2), flush=True)
r.snap("agentview", "av8.png"); r.snap("birdview", "bv8.png")
print("== translate over drawer", flush=True)
if rob.go_tcp(r, [0.065, 0.105, 1.10], Qf, steps=6, seconds=8.0, max_jump=0.8) is None: sys.exit("fail")
print("fingers", r.fingers(), "q", np.round(r.arm_q(),2), flush=True)
r.snap("agentview", "av9.png"); r.snap("frontview", "fv9.png")
print("DONE", flush=True)
EOF
nohup python3 -u s6_place.py > s6.log 2>&1 &
sleep 50; cat s6.log

# openrua op 82
sleep 45; cat s6.log | tail -8

# openrua op 83
python3 -c "
import rob,numpy as np
r=rob.Robot()
img,W=r.snap('robot0_eye_in_hand','eih9.png')
X,Y,Z=W[:,:,0],W[:,:,1],W[:,:,2]
m=(Z>1.06)&(Z<1.14)&(X>-0.15)&(X<0.15)&(Y>0.0)&(Y<0.22)
print('n',m.sum(),'x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]'%(X[m].min(),X[m].max(),Y[m].min(),Y[m].max(),Z[m].min(),Z[m].max()))
for x0 in np.arange(-0.12,0.12,0.02):
    s=m&(X>=x0)&(X<x0+0.02)
    if s.sum(): print(' x%.2f n%3d y[%.3f,%.3f] yc %.3f z[%.3f,%.3f]'%(x0,s.sum(),Y[s].min(),Y[s].max(),(Y[s].min()+Y[s].max())/2,Z[s].min(),Z[s].max()))
r.snap('sideview','sv9.png')
"

# openrua op 84
python3 -c "
import numpy as np, rob
W=np.load('eih9_xyz.npy'); X,Y,Z=W[:,:,0],W[:,:,1],W[:,:,2]
print('center px world',W[240,320].round(3),'top-center',W[10,320].round(3),'bottom-center',W[470,320].round(3),'left',W[240,10].round(3),'right',W[240,630].round(3))
# bottle: dark green points below hand: z 0.95..1.13 and y in [0.05,0.16], x in [-0.12,0.12]
m=(Z>0.99)&(Z<1.135)&(X>-0.12)&(X<0.12)&(Y>0.06)&(Y<0.16)
print('n',m.sum())
for x0 in np.arange(-0.12,0.12,0.02):
    s=m&(X>=x0)&(X<x0+0.02)
    if s.sum(): print(' x%.2f n%3d y[%.3f,%.3f] z[%.3f,%.3f]'%(x0,s.sum(),Y[s].min(),Y[s].max(),Z[s].min(),Z[s].max()))
"

# openrua op 85
python3 -c "
import rob,numpy as np
r=rob.Robot()
img,W=r.snap('birdview','bv9.png')
X,Y,Z=W[:,:,0],W[:,:,1],W[:,:,2]
m=(Z>1.0)&(Z<1.16)&(X>-0.15)&(X<0.03)&(Y>0.0)&(Y<0.22)
print('n',m.sum())
for x0 in np.arange(-0.14,0.04,0.01):
    s=m&(X>=x0)&(X<x0+0.01)
    if s.sum(): print(' x%.2f n%3d y[%.3f,%.3f] yc %.3f z[%.3f,%.3f]'%(x0,s.sum(),Y[s].min(),Y[s].max(),(Y[s].min()+Y[s].max())/2,Z[s].min(),Z[s].max()))
"

# openrua op 86
python3 -c "
import cv2
im=cv2.imread('bv9.png'); c=im[230:340,300:440]; c=cv2.resize(c,None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST); cv2.imwrite('bv9_crop.png',c)
import numpy as np
W=np.load('bv9_xyz.npy'); X,Y,Z=W[:,:,0],W[:,:,1],W[:,:,2]
m=(Z>0.99)&(X>-0.15)&(X<0.03)&(Y>0.04)&(Y<0.2)
print('n',m.sum())
for x0 in np.arange(-0.14,0.04,0.01):
    s=m&(X>=x0)&(X<x0+0.01)
    if s.sum(): print(' x%.2f n%3d y[%.3f,%.3f] z[%.3f,%.3f]'%(x0,s.sum(),Y[s].min(),Y[s].max(),Z[s].min(),Z[s].max()))
"

# openrua op 87
cat > /workspace/s7_lower.py <<'EOF'
import numpy as np, rob, sys
r = rob.Robot()
p, Qf = r.fk(); print("start tcp", rob.tcp_now(r).round(4), "fingers", r.fingers(), flush=True)
print("== descend to 0.99", flush=True)
if rob.go_tcp(r, [0.065, 0.104, 0.99], Qf, steps=4, seconds=5.0, max_jump=0.8) is None: sys.exit("fail")
print("wrench", r.wrench().round(2), flush=True)
print("== descend to 0.955", flush=True)
if rob.go_tcp(r, [0.065, 0.104, 0.955], Qf, steps=3, seconds=4.0, max_jump=0.8) is None: sys.exit("fail")
print("wrench", r.wrench().round(2), "fingers", r.fingers(), flush=True)
r.snap("agentview", "av10.png")
print("== release", flush=True)
r.gripper(0.04)
print("== retreat up", flush=True)
if rob.go_tcp(r, [0.065, 0.104, 1.15], Qf, steps=3, seconds=4.0, max_jump=0.8) is None: sys.exit("fail")
print("== move away", flush=True)
q = r.move_pose(rob.hand_from_tcp([-0.25, -0.05, 1.20], Qf), Qf, seconds=6.0, max_jump=1.5)
rob.settle(r, q); print("tcp", rob.tcp_now(r).round(3), flush=True)
r.snap("birdview", "bv10.png"); r.snap("agentview", "av11.png")
print("DONE", flush=True)
EOF
nohup python3 -u s7_lower.py > s7.log 2>&1 &
sleep 60; cat s7.log

# openrua op 88
sleep 45; cat s7.log | tail -12

# openrua op 89
python3 -c "
import cv2
im=cv2.imread('bv10.png'); c=im[240:340,300:440]; c=cv2.resize(c,None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST); cv2.imwrite('bv10_crop.png',c)
import numpy as np
W=np.load('bv10_xyz.npy'); X,Y,Z=W[:,:,0],W[:,:,1],W[:,:,2]
m=(Z>0.93)&(Z<0.99)&(X>-0.12)&(X<0.12)&(Y>0.06)&(Y<0.215)
print('in-drawer elevated n',m.sum())
for x0 in np.arange(-0.12,0.12,0.01):
    s=m&(X>=x0)&(X<x0+0.01)
    if s.sum(): print(' x%.2f n%3d y[%.3f,%.3f] z[%.3f,%.3f]'%(x0,s.sum(),Y[s].min(),Y[s].max(),Z[s].min(),Z[s].max()))
"

# openrua op 90
python3 -c "
import numpy as np
W=np.load('bv10_xyz.npy'); X,Y,Z=W[:,:,0],W[:,:,1],W[:,:,2]
for xr in [(-0.06,-0.02),(0.02,0.06),(0.07,0.10)]:
  m=(X>xr[0])&(X<xr[1])&(Y>0.04)&(Y<0.23)
  print('x range',xr)
  for y0 in np.arange(0.04,0.23,0.005):
    s=m&(Y>=y0)&(Y<y0+0.005)
    if s.sum(): print('  y%.3f n%3d z[%.3f,%.3f]'%(y0,s.sum(),Z[s].min(),Z[s].max()))
" | head -80

# openrua op 91
python3 -c "
import rob,numpy as np
r=rob.Robot()
q=r.arm_q(); print('q',np.round(q,2))
Qv=rob.frame_quat([0,0,-1],[0,-1,0])
for tcp in ([-0.09,0.045,0.95],[-0.09,0.10,0.95]):
    s=rob.ik_near(r,rob.hand_from_tcp(tcp,Qv),Qv,seed=q,max_jump=1.5); print('phase1',tcp,None if s is None else np.round(s,2))
for a in (50,60):
    al=np.radians(a)
    for yh in ([1,0,0],[-1,0,0]):
        Q=rob.frame_quat([0,np.sin(al),-np.cos(al)],yh)
        print('alpha',a,'yh',yh,'x_h',rob.quat_R(*Q)[:,0].round(2))
        seed=q
        for tcp in ([-0.08,0.075,0.935],[-0.08,0.205,0.935]):
            s=rob.ik_near(r,rob.hand_from_tcp(tcp,Q),Q,seed=seed,max_jump=2.5)
            print('   ',tcp,None if s is None else np.round(s,2))
            if s is not None: seed=s
"

# openrua op 92
python3 -c "
import rob,numpy as np
r=rob.Robot()
for tag,q in [('p1a',[0.81,0.75,-0.7,-2.06,0.82,2.5,0.32]),('p1b',[0.87,0.75,-0.68,-2.04,0.79,2.5,0.44]),('p2a',[0.44,0.96,-0.66,-1.83,1.59,1.58,1.26]),('p2b',[0.61,0.92,-0.57,-1.85,1.72,1.74,1.34])]:
    print(tag, {l.replace('panda_',''):r.fk(q,l)[0].round(2).tolist() for l in ['panda_link4','panda_link5','panda_link6','panda_link7','panda_hand']})
    p,qq=r.fk(q); print('   tcp',(p+0.1034*rob.quat_R(*qq)[:,2]).round(3))
"

# openrua op 93
cat > /workspace/s8_close.py <<'EOF'
import numpy as np, rob, sys, time

def panel_y(r, tag):
    img, W = r.snap("birdview", f"bv_{tag}.png")
    X, Y, Z = W[:,:,0], W[:,:,1], W[:,:,2]
    m = (Z > 0.975) & (Z < 0.99) & (X > 0.0) & (X < 0.08) & (Y > 0.0) & (Y < 0.25)
    if m.sum():
        print(f"  [{tag}] panel top y[{Y[m].min():.3f},{Y[m].max():.3f}] n{m.sum()}", flush=True)
    else:
        print(f"  [{tag}] no panel-top points", flush=True)

def push(r, Q, x, y0, y1, zp, zsafe=1.05, tag=""):
    print(f"== push {tag}: y {y0}->{y1} at z {zp}", flush=True)
    q = r.move_pose(rob.hand_from_tcp([x, y0, zsafe], Q), Q, seconds=6.0, max_jump=2.0)
    if q is None: sys.exit("ik fail")
    rob.settle(r, q)
    if rob.go_tcp(r, [x, y0, zp], Q, steps=3, seconds=4.0, max_jump=0.8) is None: sys.exit("fail")
    w0 = r.wrench()
    if rob.go_tcp(r, [x, y1, zp], Q, steps=4, seconds=6.0, max_jump=0.8) is None: sys.exit("fail")
    print("  wrench before", w0.round(2), "after", r.wrench().round(2), flush=True)
    if rob.go_tcp(r, [x, y1, zsafe], Q, steps=2, seconds=3.0, max_jump=0.8) is None: sys.exit("fail")
    panel_y(r, tag)

r = rob.Robot()
r.gripper(0.0)
panel_y(r, "start")
Qv = rob.frame_quat([0, 0, -1], [0, -1, 0])
push(r, Qv, -0.09, 0.04, 0.10, 0.95, tag="p1")
Qx = rob.frame_quat([0, 0, -1], [-1, 0, 0])
push(r, Qx, -0.08, 0.09, 0.17, 0.95, tag="p2")
al = np.radians(45)
Q45 = rob.frame_quat([0, np.sin(al), -np.cos(al)], [-1, 0, 0])
push(r, Q45, -0.08, 0.16, 0.205, 0.935, zsafe=1.0, tag="p3")
print("DONE", flush=True)
EOF
nohup python3 -u s8_close.py > s8.log 2>&1 &
sleep 60; cat s8.log

# openrua op 94
sleep 60; cat s8.log | tail -12

# openrua op 95
sleep 60; cat s8.log | tail -12

# openrua op 96
pkill -f s8_close.py; sleep 2; python3 -c "
import rob,numpy as np
r=rob.Robot()
q=r.arm_q(); print('q',np.round(q,3)); p,qq=r.fk(); print('hand',p.round(3)); print('R',rob.quat_R(*qq).round(2)); print('tcp',rob.tcp_now(r).round(3))
Qx=rob.frame_quat([0,0,-1],[-1,0,0])
s=rob.ik_near(r,rob.hand_from_tcp([-0.08,0.09,1.05],Qx),Qx,seed=q,max_jump=3); print('ik target',None if s is None else np.round(s,2))
"

# openrua op 97
python3 -c "
import rob,numpy as np
r=rob.Robot()
q=r.arm_q(); print('q',np.round(q,3)); p,qq=r.fk(); print('hand',p.round(3)); print('R',rob.quat_R(*qq).round(2)); print('tcp',rob.tcp_now(r).round(3))
Qx=rob.frame_quat([0,0,-1],[-1,0,0])
s=rob.ik_near(r,rob.hand_from_tcp([-0.08,0.09,1.05],Qx),Qx,seed=q,max_jump=3); print('ik target',None if s is None else np.round(s,2))
"; tail -3 s8.log

# openrua op 98
python3 -c "
import rob,numpy as np
r=rob.Robot()
p,qq=r.fk()
print('wrench',r.wrench().round(2))
rob.go_tcp(r,[-0.078,0.085,1.10],qq,steps=3,seconds=4.0,max_jump=0.8)
print('q',np.round(r.arm_q(),3),'wrench',r.wrench().round(2))
r.snap('birdview','bv_chk.png'); r.snap('agentview','av_chk.png')
" 2>&1 | grep -v Warn

# openrua op 99
python3 -c "
import numpy as np
W=np.load('bv_chk_xyz.npy'); X,Y,Z=W[:,:,0],W[:,:,1],W[:,:,2]
m=(Z>0.975)&(Z<0.99)&(X>0.0)&(X<0.08)&(Y>0.0)&(Y<0.2)
print('panel top y[%.3f,%.3f]'%(Y[m].min(),Y[m].max()))
h=(Z>0.93)&(Z<0.97)&(X>-0.05)&(X<0.04)&(Y>0.0)&(Y<0.12)
print('handle y[%.3f,%.3f] z[%.3f,%.3f]'%(Y[h].min(),Y[h].max(),Z[h].min(),Z[h].max()))
b=(Z>0.94)&(Z<0.975)&(X>-0.09)&(X<0.1)&(Y>0.1)&(Y<0.21)
print('bottle x[%.3f,%.3f] y[%.3f,%.3f] ztop %.3f'%(X[b].min(),X[b].max(),Y[b].min(),Y[b].max(),Z[b].max()))
"

# openrua op 100
cat > /workspace/s9_close2.py <<'EOF'
import numpy as np, rob, sys
r = rob.Robot()
print("tcp", rob.tcp_now(r).round(3), "q", np.round(r.arm_q(),2), flush=True)
Qx = rob.frame_quat([0, 0, -1], [-1, 0, 0])
print("== rotate in place", flush=True)
if rob.go_tcp(r, [-0.08, 0.07, 1.10], Qx, steps=6, seconds=8.0, max_jump=0.9) is None: sys.exit("fail")
print("q", np.round(r.arm_q(),2), "R", rob.quat_R(*r.fk()[1]).round(2).tolist(), flush=True)
print("== lower", flush=True)
if rob.go_tcp(r, [-0.08, 0.07, 0.95], Qx, steps=3, seconds=4.0, max_jump=0.8) is None: sys.exit("fail")
print("== push", flush=True)
w0 = r.wrench()
if rob.go_tcp(r, [-0.08, 0.175, 0.95], Qx, steps=4, seconds=6.0, max_jump=0.8) is None: sys.exit("fail")
print("wrench before", w0.round(2), "after", r.wrench().round(2), flush=True)
print("== lift", flush=True)
p = rob.tcp_now(r)
if rob.go_tcp(r, [p[0], p[1], 1.10], Qx, steps=2, seconds=3.0, max_jump=0.8) is None: sys.exit("fail")
img, W = r.snap("birdview", "bv_p2.png")
X, Y, Z = W[:,:,0], W[:,:,1], W[:,:,2]
m = (Z > 0.975) & (Z < 0.99) & (X > 0.0) & (X < 0.08) & (Y > 0.0) & (Y < 0.2)
print("panel top y[%.3f,%.3f]" % (Y[m].min(), Y[m].max()), flush=True)
print("DONE", flush=True)
EOF
nohup python3 -u s9_close2.py > s9.log 2>&1 &
sleep 75; cat s9.log

# openrua op 101
sleep 45; cat s9.log | tail -10

# openrua op 102
sleep 40; tail -8 /workspace/s9.log; pgrep -f s9_close2.py || echo "no proc"

# openrua op 103
cd /workspace; pgrep -af s9_close2 | grep -v pgrep || echo "no proc"; python3 - <<'EOF'
import numpy as np
W=np.load('bv_p2_xyz.npy'); X,Y,Z=W[...,0],W[...,1],W[...,2]
# handle: points in front of panel face, x in [-0.06,0.05], z>0.92, y in [0.10,0.17]
m=(X>-0.06)&(X<0.05)&(Y>0.10)&(Y<0.166)&(Z>0.915)
print("handle pts",m.sum())
if m.sum(): print("x",X[m].min(),X[m].max(),"y",Y[m].min(),Y[m].max(),"z",Z[m].min(),Z[m].max())
# panel top edge
m2=(X>-0.09)&(X<0.09)&(Z>0.975)&(Z<0.99)&(Y>0.10)&(Y<0.21)
print("panel top y",Y[m2].min(),Y[m2].max(), "x",X[m2].min(),X[m2].max())
# anything in y 0.178..0.21 below cabinet top (drawer opening / floor visible?)
m3=(X>-0.09)&(X<0.09)&(Y>0.18)&(Y<0.21)&(Z<0.98)
print("interior visible pts",m3.sum(), Z[m3].min() if m3.sum() else None)
EOF

# openrua op 104
cd /workspace; python3 - <<'EOF'
import numpy as np
W=np.load('bv_p2_xyz.npy'); X,Y,Z=W[...,0],W[...,1],W[...,2]
m=(X>-0.06)&(X<0.05)&(Y>0.10)&(Y<0.166)&(Z>0.915)&(Z<1.0)
print("handle pts",m.sum(),"x",X[m].min(),X[m].max(),"y",Y[m].min(),Y[m].max(),"z",Z[m].min(),Z[m].max())
for zlo in [0.94,0.95,0.96,0.97,0.98]:
    mm=m&(Z>=zlo)&(Z<zlo+0.01); 
    if mm.sum(): print(zlo, mm.sum(), "y",Y[mm].min().round(3),Y[mm].max().round(3),"x",X[mm].min().round(3),X[mm].max().round(3))
EOF
python3 -c "
import rob; r=rob.Robot(); print('q',r.arm_q.round(3)); print('tcp',rob.tcp_now(r).round(4)); print('w',r.wrench.round(2))"

# openrua op 105
cd /workspace; grep -n "def \|^class\|^[A-Z_]* =" rob.py | head -60

# openrua op 106
mkdir -p "$(dirname /workspace/s10_close3.py)"
cat > /workspace/s10_close3.py <<'OPENRUA_EOF'
import numpy as np, rob, sys
r = rob.Robot("s10")
al = np.deg2rad(45)
Q45 = rob.frame_quat([0, np.sin(al), -np.cos(al)], [-1, 0, 0])
tcp0 = rob.tcp_now(r)
print("start q", r.arm_q().round(3), "tcp", tcp0.round(4), flush=True)

# dry-run IK for the key poses
seed = r.arm_q()
poses = [([-0.08, 0.155, 1.10], "rot"), ([-0.08, 0.145, 0.935], "low"), ([-0.08, 0.207, 0.935], "end")]
for p, nm in poses:
    sol = rob.ik_near(r, rob.hand_from_tcp(p, Q45), Q45, seed=seed, max_jump=1.5)
    print(nm, None if sol is None else np.round(sol, 3), flush=True)
    if sol is None:
        sys.exit("IK dry run failed")
    seed = sol
if "--dry" in sys.argv:
    sys.exit(0)

print("== rotate in place", flush=True)
if rob.go_tcp(r, [-0.08, 0.155, 1.10], Q45, steps=6, seconds=8.0, max_jump=0.9) is None: sys.exit("rot fail")
print("== lower", flush=True)
if rob.go_tcp(r, [-0.08, 0.145, 0.935], Q45, steps=4, seconds=5.0, max_jump=0.8) is None: sys.exit("low fail")
print("wrench", r.wrench().round(2), flush=True)
for y in (0.185, 0.207):
    print(f"== push to {y}", flush=True)
    rob.go_tcp(r, [-0.08, y, 0.935], Q45, steps=3, seconds=4.0, max_jump=0.8)
    print("wrench", r.wrench().round(2), flush=True)
print("== retreat/lift", flush=True)
rob.go_tcp(r, [-0.08, 0.17, 0.96], Q45, steps=2, seconds=3.0, max_jump=0.8)
rob.go_tcp(r, [-0.08, 0.15, 1.10], Q45, steps=3, seconds=5.0, max_jump=0.8)
print("wrench", r.wrench().round(2), flush=True)
for _ in range(3):
    try:
        r.snap("birdview", "bv_p3.png"); break
    except Exception as e:
        print("snap retry", e, flush=True)
W = np.load("bv_p3_xyz.npy"); X, Y, Z = W[..., 0], W[..., 1], W[..., 2]
m = (X > -0.09) & (X < 0.09) & (Z > 0.975) & (Z < 0.99) & (Y > 0.10) & (Y < 0.22)
print("panel top y", Y[m].min().round(3) if m.sum() else None, Y[m].max().round(3) if m.sum() else None, flush=True)
print("DONE", flush=True)
OPENRUA_EOF

# openrua op 107
cd /workspace; timeout 120 python3 -u s10_close3.py --dry

# openrua op 108
cd /workspace; sed -i 's/r.arm_q().round(3)/np.round(r.arm_q(),3)/; s/r.wrench().round(2)/np.round(r.wrench(),2)/g' s10_close3.py; timeout 120 python3 -u s10_close3.py --dry

# openrua op 109
cd /workspace; nohup python3 -u s10_close3.py > s10.log 2>&1 & sleep 50; cat /workspace/s10.log

# openrua op 110
sleep 45; cat /workspace/s10.log | tail -12

# openrua op 111
sleep 30; tail -5 /workspace/s10.log; pgrep -af s10_close3 | grep -v pgrep || echo "no proc"

# openrua op 112
cd /workspace; cat > s11_away.py <<'EOF'
import numpy as np, rob, sys
r = rob.Robot("s11")
al = np.deg2rad(45)
Q45 = rob.frame_quat([0, np.sin(al), -np.cos(al)], [-1, 0, 0])
Qd = rob.frame_quat([0, 0, -1], [0, 1, 0])
print("== up/away", flush=True)
rob.go_tcp(r, [-0.10, 0.08, 1.22], Q45, steps=3, seconds=5.0, max_jump=0.9)
rob.go_tcp(r, [-0.28, -0.06, 1.25], Q45, steps=4, seconds=6.0, max_jump=0.9)
print("== hand down", flush=True)
rob.go_tcp(r, [-0.28, -0.06, 1.25], Qd, steps=6, seconds=8.0, max_jump=0.9)
print("q", np.round(r.arm_q(), 3), "wrench", np.round(r.wrench(), 2), flush=True)
for cam, out in [("birdview", "bv_final.png"), ("agentview", "av_final.png"), ("frontview", "fv_final.png")]:
    for _ in range(3):
        try:
            r.snap(cam, out); print("saved", out, flush=True); break
        except Exception as e:
            print("snap retry", cam, e, flush=True)
print("DONE", flush=True)
EOF
ros2 topic list | grep color/image_raw; nohup python3 -u s11_away.py > s11.log 2>&1 & sleep 60; cat /workspace/s11.log

# openrua op 113
sleep 40; tail -6 /workspace/s11.log; pgrep -af s11_away | grep -v pgrep || echo "no proc"

# openrua op 114
cd /workspace; python3 - <<'EOF'
import numpy as np
W=np.load('bv_final_xyz.npy'); X,Y,Z=W[...,0],W[...,1],W[...,2]
# bottom-drawer front panel top edge across x
for xlo in np.arange(-0.10,0.10,0.04):
    m=(X>xlo)&(X<xlo+0.04)&(Z>0.975)&(Z<0.99)&(Y>0.10)&(Y<0.22)
    print(f"x[{xlo:.2f},{xlo+0.04:.2f}] panel-top y {Y[m].min():.3f}..{Y[m].max():.3f} n={m.sum()}" if m.sum() else f"x[{xlo:.2f}] none")
# any drawer-interior/floor points (z<0.98) between y 0.05 and 0.214 in front of cabinet?
m=(X>-0.13)&(X<0.13)&(Y>0.0)&(Y<0.212)&(Z>0.905)&(Z<0.985)
print("points above table in front of cabinet (z 0.905-0.985):",m.sum())
if m.sum(): print(" x",X[m].min().round(3),X[m].max().round(3)," y",Y[m].min().round(3),Y[m].max().round(3)," z",Z[m].min().round(3),Z[m].max().round(3))
# bottle-like objects anywhere on table outside cabinet (z 0.91-0.98, exclude cabinet footprint & bowl & robot)
m=(Z>0.91)&(Z<0.99)&~((X>-0.14)&(X<0.14)&(Y>0.19)&(Y<0.42))&(X>-0.45)&(X<0.45)&(Y>-0.7)&(Y<0.5)
print("other table objects pts:",m.sum())
if m.sum():
    print(" x",X[m].min().round(3),X[m].max().round(3)," y",Y[m].min().round(3),Y[m].max().round(3))
EOF

# openrua op 115
cd /workspace; python3 - <<'EOF'
import numpy as np, cv2
W=np.load('bv_final_xyz.npy'); X,Y,Z=W[...,0],W[...,1],W[...,2]
m=(X>-0.13)&(X<0.13)&(Y>0.0)&(Y<0.212)&(Z>0.905)&(Z<0.985)
vs,us=np.nonzero(m); print("px rows",vs.min(),vs.max(),"cols",us.min(),us.max())
for zlo in [0.92,0.93,0.94,0.95,0.96]:
    mm=m&(Z>=zlo)&(Z<zlo+0.01)
    if mm.sum(): print(zlo,mm.sum(),"x",X[mm].min().round(3),X[mm].max().round(3),"y",Y[mm].min().round(3),Y[mm].max().round(3))
img=cv2.imread('bv_final.png'); 
# crop around cabinet region: find pixel of world (0,0.2)
d=np.hypot(X-0.0,Y-0.2); v,u=np.unravel_index(np.nanargmin(d),d.shape); print("center px",u,v)
crop=img[v-90:v+90,u-110:u+110]; crop=cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_NEAREST); cv2.imwrite('bv_final_crop.png',crop)
EOF

# openrua op 116
pgrep -af "python3 -u s" | grep -v pgrep || echo "no background scripts"
