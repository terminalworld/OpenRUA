#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
cat /workspace/tools/action/fjt_send.py; echo ----; cat /workspace/tools/action/gripper_cmd.py; echo ----; cat /workspace/tools/action/ik_move.py

# openrua op 3
timeout 30 ros2 topic list; echo ---; timeout 20 ros2 action list; echo ---; timeout 20 ros2 service list | grep -v -E "parameter|describe|get_type" ; echo ---; timeout 20 ros2 node list

# openrua op 4
timeout 30 ros2 topic echo /joint_states --once; for c in agentview frontview sideview birdview robot0_eye_in_hand robot0_robotview; do timeout 60 python3 tools/perception/cam_snap.py $c; done

# openrua op 5
timeout 20 ros2 topic echo /tf_static --once 2>&1 | head -80; echo ---; timeout 10 ros2 topic echo /tf --once 2>&1 | grep -E "frame_id" | sort | uniq -c

# openrua op 6
timeout 10 ros2 topic echo /tf --once 2>&1 | grep -A 12 -E "child_frame_id: (panda_link0|agentview_optical_frame|birdview_optical_frame)"

# openrua op 7
mkdir -p "$(dirname /workspace/grab.py)"
cat > /workspace/grab.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Grab color, depth, intrinsics and world<-optical TF for cameras; save npz.
Usage: python3 grab.py cam1 [cam2 ...]
"""
import sys
import numpy as np
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
import cv2


def grab(node, topic, mtype, timeout=30.0):
    got = {}
    sub = node.create_subscription(mtype, topic, lambda m: got.setdefault("m", m), 1)
    import time
    end = time.time() + timeout
    while "m" not in got and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    if "m" not in got:
        raise SystemExit(f"no msg on {topic}")
    return got["m"]


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main():
    rclpy.init()
    node = rclpy.create_node("grab")
    tfbuf = Buffer()
    TransformListener(tfbuf, node)
    br = CvBridge()
    import time
    for cam in sys.argv[1:]:
        color = br.imgmsg_to_cv2(grab(node, f"/{cam}/color/image_raw", Image), "bgr8")
        depth = br.imgmsg_to_cv2(grab(node, f"/{cam}/depth/image_raw", Image), "passthrough").astype(np.float32)
        info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
        frame = f"{cam}_optical_frame"
        end = time.time() + 15
        while time.time() < end and not tfbuf.can_transform("world", frame, rclpy.time.Time()):
            rclpy.spin_once(node, timeout_sec=0.2)
        t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
        q = t.transform.rotation
        T = np.eye(4)
        T[:3, :3] = quat_to_R(q.x, q.y, q.z, q.w)
        T[:3, 3] = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
        K = np.array(info.k).reshape(3, 3)
        np.savez(f"{cam}.npz", color=color, depth=depth, K=K, T=T)
        cv2.imwrite(f"{cam}.png", color)
        print(cam, color.shape, depth.shape, "K", K[0, 0], K[1, 1], K[0, 2], K[1, 2], "T", T[:3, 3])
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 8
timeout 120 python3 grab.py birdview agentview frontview sideview robot0_eye_in_hand

# openrua op 9
mkdir -p "$(dirname /workspace/cloud.py)"
cat > /workspace/cloud.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Build a world point cloud from a saved npz and print a height map summary."""
import sys
import numpy as np


def cloud(cam):
    d = np.load(f"{cam}.npz")
    depth, K, T, color = d["depth"], d["K"], d["T"], d["color"]
    h, w = depth.shape
    v, u = np.mgrid[0:h, 0:w]
    z = depth
    x = (u - K[0, 2]) * z / K[0, 0]
    y = (v - K[1, 2]) * z / K[1, 1]
    p = np.stack([x, y, z, np.ones_like(z)], -1).reshape(-1, 4)
    pw = (T @ p.T).T[:, :3].reshape(h, w, 3)
    return pw, color


if __name__ == "__main__":
    cam = sys.argv[1]
    pw, color = cloud(cam)
    Z = pw[..., 2]
    ok = np.isfinite(Z) & (pw[..., 0] > -0.7) & (pw[..., 0] < 1.0) & (np.abs(pw[..., 1]) < 1.0)
    print("z percentiles", np.percentile(Z[ok], [1, 5, 25, 50, 75, 95, 99]))
    # coarse height histogram
    hist, edges = np.histogram(Z[ok], bins=np.arange(0.5, 1.6, 0.02))
    for c, e in zip(hist, edges):
        if c > 50:
            print(f"{e:.2f} {c}")
OPENRUA_EOF

# openrua op 10
python3 cloud.py birdview

# openrua op 11
python3 -c "
import numpy as np
from cloud import cloud
pw, color = cloud('birdview')
Z = pw[...,2]
# bottle region near pixel (337,257)
for (u,v) in [(337,257),(310,300),(365,300),(415,300),(440,300),(350,270),(390,270)]:
    print((u,v), pw[v,u].round(3))
# find bottle top: highest points within 0.85<z<1.3 near bottle
m = (Z>0.95)&(Z<1.4)
ys,xs = np.where(m)
print('tall pts count', m.sum())
# cluster by pixel region
import collections
for name,(u0,u1,v0,v1) in {'bottle':(320,360,240,275),'cab':(340,460,255,350),'tray':(190,290,220,320)}.items():
    sub = m[v0:v1,u0:u1]
    P = pw[v0:v1,u0:u1][sub]
    if len(P):
        print(name, 'n',len(P),'x',P[:,0].min().round(3),P[:,0].max().round(3),'y',P[:,1].min().round(3),P[:,1].max().round(3),'z',P[:,2].min().round(3),P[:,2].max().round(3))
"


# openrua op 12
python3 -c "
import numpy as np
from cloud import cloud
pw, color = cloud('birdview')
Z = pw[...,2]
sub = Z[240:280,320:360]
i = np.unravel_index(np.argmax(sub), sub.shape)
print('max z', sub[i], 'at px', (320+i[1], 240+i[0]), pw[240+i[0],320+i[1]])
# print z grid around bottle
np.set_printoptions(linewidth=250, precision=2)
print(Z[245:272:2, 322:356:2])
"


# openrua op 13
python3 -c "
import numpy as np
from cloud import cloud
pw, color = cloud('birdview')
Z = pw[...,2]
win = (slice(252,270), slice(330,348))
m = (Z[win]>0.95)&(Z[win]<1.3)
P = pw[win][m]
print('bottle n', len(P))
print('x', P[:,0].min(), P[:,0].max(), 'y', P[:,1].min(), P[:,1].max(), 'zmax', P[:,2].max())
print('centroid', P.mean(0))
# body only (z<1.09)
B = P[P[:,2]<1.09]
print('body x', B[:,0].min(), B[:,0].max(), 'y', B[:,1].min(), B[:,1].max(), 'center', B[:,:2].mean(0))
# cabinet analysis: points z>0.95 in region
win2 = (slice(255,350), slice(345,460))
m2 = (Z[win2]>0.93)&(Z[win2]<1.2)
C = pw[win2][m2]
print('cab n', len(C))
for zl,zh in [(0.93,1.0),(1.0,1.1),(1.1,1.15)]:
    S = C[(C[:,2]>=zl)&(C[:,2]<zh)]
    if len(S): print(f'z[{zl},{zh}) n={len(S)} x {S[:,0].min():.3f}..{S[:,0].max():.3f} y {S[:,1].min():.3f}..{S[:,1].max():.3f}')
"


# openrua op 14
python3 -c "
import numpy as np
from cloud import cloud
pw, color = cloud('birdview')
Z = pw[...,2]
X = pw[...,0]; Y = pw[...,1]
ok = np.isfinite(Z)&(Z>0.5)&(Z<2.0)
xs = np.arange(-0.16,0.17,0.02); ys=np.arange(0.02,0.46,0.02)
np.set_printoptions(linewidth=250, precision=2)
print('rows: x from %.2f, cols: y from %.2f step .02 -> max z'%(xs[0],ys[0]))
print('      '+' '.join(f'{y:5.2f}' for y in ys))
for x in xs:
    row=[]
    for y in ys:
        m = ok&(np.abs(X-x)<0.01)&(np.abs(Y-y)<0.01)
        row.append(Z[m].max() if m.any() else np.nan)
    print(f'{x:5.2f} '+' '.join(f'{v:5.2f}' for v in row))
"


# openrua op 15
timeout 60 ros2 service call /get_urdf std_srvs/srv/Trigger 2>/dev/null | head -c 300; echo; timeout 30 ros2 service type /get_urdf

# openrua op 16
timeout 60 python3 -c "
import rclpy
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
rclpy.init(); n=rclpy.create_node('rd')
got=[]
qos=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
n.create_subscription(String,'/robot_description',lambda m: got.append(m.data),qos)
import time; t=time.time()
while not got and time.time()-t<20: rclpy.spin_once(n,timeout_sec=0.2)
open('robot.urdf','w').write(got[0]); print(len(got[0]))
"; grep -n -E "<link name=|<collision|<box|<cylinder|<sphere|<mesh|<origin" robot.urdf | grep -A8 -E "link name=\"panda_(hand|leftfinger|rightfinger|link7|link8)\"" | head -80

# openrua op 17
python3 -c "
import numpy as np, struct
def stl_bounds(fn):
    d=open(fn,'rb').read()
    if d[:5]==b'solid':
        import re
        v=np.array([[float(x) for x in l.split()[1:]] for l in d.decode().splitlines() if l.strip().startswith('vertex')])
    else:
        n=struct.unpack('<I',d[80:84])[0]
        arr=np.frombuffer(d[84:84+n*50],dtype=np.dtype([('n','<3f4'),('v','<9f4'),('a','<u2')]))
        v=arr['v'].reshape(-1,3)
    return v.min(0), v.max(0)
for f in ['hand','finger','link7']:
    print(f, stl_bounds(f'/opt/ros/jazzy/share/moveit_resources_panda_description/meshes/collision/{f}.stl'))
"

# openrua op 18
find / -iname "hand*.stl" -o -iname "finger*.stl" 2>/dev/null | head; find / -path /proc -prune -o -iname "*.xml" -print 2>/dev/null | grep -i -E "cabinet|bottle|libero|robosuite" | head

# openrua op 19
python3 -c "
import numpy as np
from cloud import cloud
for cam in ['frontview','agentview']:
    pw, color = cloud(cam)
    X,Y,Z = pw[...,0],pw[...,1],pw[...,2]
    m = np.isfinite(Z)&(np.abs(X+0.13)<0.05)&(np.abs(Y-0.06)<0.05)&(Z>0.905)&(Z<1.3)
    P = pw[m]
    print(cam, 'bottle pts', len(P), 'z', P[:,2].min().round(3), P[:,2].max().round(3))
    for zl in np.arange(0.90,1.14,0.02):
        S = P[(P[:,2]>=zl)&(P[:,2]<zl+0.02)]
        if len(S): print(f'  z {zl:.2f}: n={len(S)} x {S[:,0].min():.3f}..{S[:,0].max():.3f} y {S[:,1].min():.3f}..{S[:,1].max():.3f}')
"


# openrua op 20
python3 -c "
import numpy as np
from cloud import cloud
pw, color = cloud('birdview')
Z = pw[...,2]; X = pw[...,0]; Y = pw[...,1]
ok = np.isfinite(Z)&(Z>0.5)&(Z<1.3)
xs = np.arange(-0.12,0.14,0.01); ys=np.arange(0.00,0.13,0.01)
print('      '+' '.join(f'{y:5.2f}' for y in ys))
for x in xs:
    row=[]
    for y in ys:
        m = ok&(np.abs(X-x)<0.005)&(np.abs(Y-y)<0.005)
        row.append(Z[m].max() if m.any() else np.nan)
    print(f'{x:5.2f} '+' '.join(f'{v:5.2f}' for v in row))
"


# openrua op 21
timeout 20 ros2 interface show moveit_msgs/srv/GetPositionFK | head -30; timeout 20 ros2 interface show moveit_msgs/srv/GetCartesianPath | grep -v "^ *#" | head -40

# openrua op 22
mkdir -p "$(dirname /workspace/panda.py)"
cat > /workspace/panda.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Robot helper library: FK/IK (numpy), trajectory, gripper, sensors.

World frame = panda_link0 + BASE_OFF.  Poses handled as 4x4 matrices.
TCP = panda_hand origin + 0.1034 along hand z (fingertip pad centre).
"""
import math
import sys
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import WrenchStamped
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

BASE_OFF = np.array([-0.66, 0.0, 0.912])
JOINTS = [f"panda_joint{i}" for i in range(1, 8)]
LIM = np.array([[-2.9, 2.9], [-1.76, 1.76], [-2.9, 2.9], [-3.07, -0.07],
                [-2.9, 2.9], [-0.02, 3.75], [-2.9, 2.9]])
TCP_OFF = 0.1034
# modified DH: (a, d, alpha)
DH = [(0, 0.333, 0), (0, 0, -math.pi / 2), (0, 0.316, math.pi / 2),
      (0.0825, 0, math.pi / 2), (-0.0825, 0.384, -math.pi / 2),
      (0, 0, math.pi / 2), (0.088, 0, math.pi / 2)]


def _tf(a, d, alpha, theta):
    ca, sa = math.cos(alpha), math.sin(alpha)
    ct, st = math.cos(theta), math.sin(theta)
    return np.array([[ct, -st, 0, a],
                     [st * ca, ct * ca, -sa, -d * sa],
                     [st * sa, ct * sa, ca, d * ca],
                     [0, 0, 0, 1]])


def rotz(t):
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])


def fk_hand_base(q):
    """panda_hand pose in panda_link0 frame."""
    T = np.eye(4)
    for (a, d, al), th in zip(DH, q):
        T = T @ _tf(a, d, al, th)
    T = T @ _tf(0, 0.107, 0, 0) @ rotz(-math.pi / 4)
    return T


def fk_tcp(q):
    """TCP pose in WORLD frame."""
    T = fk_hand_base(q)
    T = T @ np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, TCP_OFF], [0, 0, 0, 1]])
    T[:3, 3] += BASE_OFF
    return T


def pose_err(T, Tt):
    """6-vector: position error and rotation error (axis-angle) of T vs target."""
    dp = Tt[:3, 3] - T[:3, 3]
    Rd = Tt[:3, :3] @ T[:3, :3].T
    ang = math.acos(max(-1.0, min(1.0, (np.trace(Rd) - 1) / 2)))
    if ang < 1e-9:
        w = np.zeros(3)
    else:
        w = ang / (2 * math.sin(ang)) * np.array([Rd[2, 1] - Rd[1, 2], Rd[0, 2] - Rd[2, 0], Rd[1, 0] - Rd[0, 1]])
    return np.concatenate([dp, w])


def ik(Tt, q0, iters=200, tol=1e-4, pos_only=False, w_rot=1.0):
    """Damped least squares IK for TCP target Tt (world). Returns q or None."""
    q = np.array(q0, float).copy()
    for it in range(iters):
        T = fk_tcp(q)
        e = pose_err(T, Tt)
        if pos_only:
            e[3:] = 0
        e[3:] *= w_rot
        if np.linalg.norm(e[:3]) < tol and np.linalg.norm(e[3:]) < tol * 10:
            return q
        # numeric jacobian
        J = np.zeros((6, 7))
        h = 1e-6
        for i in range(7):
            dq = np.zeros(7); dq[i] = h
            Ti = fk_tcp(q + dq)
            J[:, i] = -pose_err(Ti, Tt) + e  # (e(q) - e(q+dq))/h * ... sign handled below
        J /= h
        # J now approximates d(-e)/dq = d(pose)/dq ; solve J dq = e
        lam = 1e-3
        dq = J.T @ np.linalg.solve(J @ J.T + lam * np.eye(6), e)
        # limit step
        n = np.linalg.norm(dq)
        if n > 0.3:
            dq *= 0.3 / n
        q = q + dq
        q = np.clip(q, LIM[:, 0] + 0.01, LIM[:, 1] - 0.01)
    e = pose_err(fk_tcp(q), Tt)
    if np.linalg.norm(e[:3]) < 1e-3 and (pos_only or np.linalg.norm(e[3:]) < 1e-2):
        return q
    return None


def make_T(p, R):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    return T


def R_from_axes(z, x=None, y=None):
    """Rotation with given hand z (approach) and hand x or y direction."""
    z = np.array(z, float); z /= np.linalg.norm(z)
    if x is not None:
        x = np.array(x, float); x -= x.dot(z) * z; x /= np.linalg.norm(x)
        y = np.cross(z, x)
    else:
        y = np.array(y, float); y -= y.dot(z) * z; y /= np.linalg.norm(y)
        x = np.cross(y, z)
    return np.stack([x, y, z], 1)


def topdown(yaw=0.0):
    """Hand z down; hand y (finger axis) at `yaw` from world y... precisely:
    hand x = (cos yaw, sin yaw, 0), z = -Z."""
    x = np.array([math.cos(yaw), math.sin(yaw), 0])
    return R_from_axes([0, 0, -1], x=x)


class Robot:
    def __init__(self, name="panda_lib"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = None
        self._wr = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.node.create_subscription(WrenchStamped, "/franka_robot_state_broadcaster/external_wrench", self._on_wr, 10)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, "/panda_arm_controller/follow_joint_trajectory")
        self.grip_cli = ActionClient(self.node, GripperCommand, "/franka_gripper/gripper_action")
        self.fjt.wait_for_server(timeout_sec=20)
        self.grip_cli.wait_for_server(timeout_sec=20)

    def _on_js(self, m):
        self._js = m

    def _on_wr(self, m):
        self._wr = m

    def spin(self, t=0.05):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self, fresh=True):
        if fresh:
            self._js = None
            t0 = time.time()
            while self._js is None and time.time() - t0 < 20:
                self.spin(0.1)
        d = dict(zip(self._js.name, self._js.position))
        return d

    def q(self):
        d = self.joints()
        return np.array([d[j] for j in JOINTS])

    def fingers(self):
        d = self.joints()
        return d.get("panda_finger_joint1"), d.get("panda_finger_joint2")

    def wrench(self):
        self._wr = None
        t0 = time.time()
        while self._wr is None and time.time() - t0 < 10:
            self.spin(0.1)
        if self._wr is None:
            return None
        f, t = self._wr.wrench.force, self._wr.wrench.torque
        return np.array([f.x, f.y, f.z, t.x, t.y, t.z])

    def tcp(self):
        return fk_tcp(self.q())

    def move_traj(self, qs, times, timeout=600):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        for qv, t in zip(qs, times):
            pt = JointTrajectoryPoint(positions=[float(v) for v in qv])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        fut = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        gh = fut.result()
        if gh is None or not gh.accepted:
            print("goal rejected", flush=True)
            return None
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=timeout)
        if rf.result() is None:
            print("traj result timeout", flush=True)
            return None
        code = rf.result().result.error_code
        return code

    def move_q(self, qt, T=3.0):
        code = self.move_traj([qt], [T])
        qa = self.q()
        err = np.abs(qa - np.array(qt)).max()
        print(f"move_q code={code} max_joint_err={err:.4f}", flush=True)
        return code, err

    def gripper(self, width):
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = 30.0
        fut = self.grip_cli.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result if rf.result() else None
        f = self.fingers()
        print(f"gripper({width}) reached={getattr(r,'reached_goal',None)} stalled={getattr(r,'stalled',None)} fingers={f}", flush=True)
        return f

    def move_tcp(self, Tt, T=3.0, q0=None, n_wp=1):
        """IK then trajectory to target TCP pose (world). n_wp>1 -> straight-line waypoints."""
        q0 = self.q() if q0 is None else np.array(q0)
        if n_wp <= 1:
            qt = ik(Tt, q0)
            if qt is None:
                print("IK failed", flush=True)
                return None
            return self.move_q(qt, T)
        T0 = fk_tcp(q0)
        qs, ts = [], []
        qprev = q0
        for i in range(1, n_wp + 1):
            s = i / n_wp
            Ti = interp_T(T0, Tt, s)
            qi = ik(Ti, qprev)
            if qi is None:
                print(f"IK failed at waypoint {i}", flush=True)
                return None
            qs.append(qi); ts.append(T * s); qprev = qi
        code = self.move_traj(qs, ts)
        qa = self.q()
        err = np.abs(qa - qs[-1]).max()
        print(f"move_tcp code={code} max_joint_err={err:.4f}", flush=True)
        return code, err


def slerp_R(R0, R1, s):
    Rd = R1 @ R0.T
    ang = math.acos(max(-1.0, min(1.0, (np.trace(Rd) - 1) / 2)))
    if ang < 1e-9:
        return R0
    w = np.array([Rd[2, 1] - Rd[1, 2], Rd[0, 2] - Rd[2, 0], Rd[1, 0] - Rd[0, 1]]) / (2 * math.sin(ang))
    K = np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])
    a = ang * s
    Rs = np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * K @ K
    return Rs @ R0


def interp_T(T0, T1, s):
    T = np.eye(4)
    T[:3, :3] = slerp_R(T0[:3, :3], T1[:3, :3], s)
    T[:3, 3] = T0[:3, 3] * (1 - s) + T1[:3, 3] * s
    return T


if __name__ == "__main__":
    r = Robot()
    q = r.q()
    print("q", q.round(4))
    T = fk_tcp(q)
    np.set_printoptions(precision=4, suppress=True)
    print("TCP world:\n", T)
    print("fingers", r.fingers())
    print("wrench", r.wrench())
OPENRUA_EOF

# openrua op 23
timeout 120 python3 panda.py 2>&1 | tail -20; timeout 60 python3 -c "
import rclpy, numpy as np
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState
from panda import JOINTS, fk_hand_base
rclpy.init(); n=rclpy.create_node('fkchk')
cli=n.create_client(GetPositionFK,'/compute_fk'); cli.wait_for_service(10)
req=GetPositionFK.Request(); req.fk_link_names=['panda_hand']
q=[0.0,-0.161037389,0.0,-2.44459747,0.0,2.2267522,0.7853981633974483]
req.robot_state.joint_state=JointState(name=JOINTS, position=q)
f=cli.call_async(req); rclpy.spin_until_future_complete(n,f,timeout_sec=30)
r=f.result(); p=r.pose_stamped[0]; print('moveit fk frame', p.header.frame_id, p.pose.position, p.pose.orientation, 'code', r.error_code.val)
np.set_printoptions(precision=4, suppress=True); print('mine hand in base\n', fk_hand_base(q))
"

# openrua op 24
python3 -c "
import numpy as np
from cloud import cloud
pw, color = cloud('birdview')
X,Y,Z = pw[...,0],pw[...,1],pw[...,2]
m = np.isfinite(Z)&(Z>0.93)&(Z<1.05)&(X>-0.12)&(X<0.12)&(Y>-0.2)&(Y<0.03)
P = pw[m]
print('bowl pts', len(P), 'x', P[:,0].min().round(3), P[:,0].max().round(3), 'y', P[:,1].min().round(3), P[:,1].max().round(3), 'zmax', P[:,2].max().round(3))
rim = P[P[:,2]>1.0]
print('rim center', rim[:,:2].mean(0).round(3), 'rim x', rim[:,0].min().round(3), rim[:,0].max().round(3), 'y', rim[:,1].min().round(3), rim[:,1].max().round(3))
# shelf extents
m2 = np.isfinite(Z)&(Z>0.95)&(Z<1.4)&(X>-0.4)&(X<0.1)&(Y<-0.08)
S=pw[m2]; print('shelf x', S[:,0].min().round(3), S[:,0].max().round(3), 'y', S[:,1].min().round(3), S[:,1].max().round(3), 'z', S[:,2].max().round(3))
# table extent
mt = np.isfinite(Z)&(np.abs(Z-0.90)<0.01)
Tt=pw[mt]; print('table x', Tt[:,0].min().round(3), Tt[:,0].max().round(3), 'y', Tt[:,1].min().round(3), Tt[:,1].max().round(3))
"


# openrua op 25
timeout 60 python3 -c "
import numpy as np, math
from panda import *
np.set_printoptions(precision=4, suppress=True)
q0=np.array([0.0,-0.161037389,0.0,-2.44459747,0.0,2.2267522,0.7853981633974483])
import time
for name,(p,yaw) in {'above_bowl':([0.075,-0.045,1.15],math.pi/2),'bowl_grasp':([0.075,-0.045,0.99],math.pi/2),'bowl_place':([-0.35,0.30,1.0],math.pi/2), 'high':([-0.35,0.30,1.30],math.pi/2)}.items():
    t=time.time(); q=ik(make_T(p, topdown(yaw)), q0)
    print(name, 'q', None if q is None else q.round(3), f'{time.time()-t:.2f}s')
    if q is not None: print('   fk', fk_tcp(q)[:3,3].round(4), 'hand x', fk_tcp(q)[:3,0].round(2), 'hand y', fk_tcp(q)[:3,1].round(2))
"

# openrua op 26
mkdir -p "$(dirname /workspace/collide.py)"
cat > /workspace/collide.py <<'OPENRUA_EOF'
"""Cheap collision check: arm capsules + hand box vs world AABBs."""
import math
import numpy as np
from panda import DH, _tf, rotz, BASE_OFF, fk_tcp

TABLE_Z = 0.90
# name: (min xyz, max xyz) in world
OBST = {
    "shelf": ([-0.245, -0.38, 0.90], [0.04, -0.075, 1.31]),
    "cabinet": ([-0.14, 0.20, 0.90], [0.145, 0.44, 1.14]),
    "drawer": ([-0.115, 0.06, 0.90], [0.135, 0.21, 0.99]),
    "bottle": ([-0.165, 0.025, 0.90], [-0.10, 0.09, 1.135]),
    "bowl": ([-0.085, -0.13, 0.90], [0.085, 0.04, 1.03]),
}
LINK_R = 0.055


def frames(q):
    """World poses of link frames 1..7, flange, hand."""
    T = np.eye(4)
    T[:3, 3] = BASE_OFF
    out = [T.copy()]
    for (a, d, al), th in zip(DH, q):
        T = T @ _tf(a, d, al, th)
        out.append(T.copy())
    T = T @ _tf(0, 0.107, 0, 0)
    out.append(T.copy())  # flange
    T = T @ rotz(-math.pi / 4)
    out.append(T.copy())  # hand
    return out


def pt_box_dist(p, lo, hi):
    d = np.maximum(np.maximum(lo - p, 0), p - hi)
    return np.linalg.norm(d, axis=-1)


def hand_points(Th):
    """Sample points of the hand box + fingers in world."""
    pts = []
    for x in (-0.03, 0.03):
        for y in np.linspace(-0.10, 0.10, 9):
            for z in (-0.04, 0.0, 0.035, 0.07):
                pts.append([x, y, z, 1])
    for x in (-0.012, 0.012):
        for y in (-0.05, -0.025, 0, 0.025, 0.05):
            for z in (0.09, 0.115):
                pts.append([x, y, z, 1])
    P = (Th @ np.array(pts).T).T[:, :3]
    return P


def arm_points(q, n=6):
    fr = frames(q)
    pts = []
    # capsule segments between consecutive frame origins (skip base->1 which is the pedestal column)
    origins = [f[:3, 3] for f in fr]
    for i in range(1, len(origins) - 2):  # up to flange
        a, b = origins[i], origins[i + 1]
        for s in np.linspace(0, 1, n):
            pts.append(a * (1 - s) + b * s)
    return np.array(pts), fr[-1]


def check(q, ignore=(), verbose=False, extra=None):
    """Return (min clearance, worst obstacle name). Negative = collision."""
    obst = dict(OBST)
    if extra:
        obst.update(extra)
    P, Th = arm_points(q)
    H = hand_points(Th)
    worst = (1e9, None)
    for name, (lo, hi) in obst.items():
        if name in ignore:
            continue
        lo, hi = np.array(lo), np.array(hi)
        d1 = pt_box_dist(P, lo, hi).min() - LINK_R
        d2 = pt_box_dist(H, lo, hi).min()
        d = min(d1, d2)
        if verbose:
            print(f"  {name}: arm {d1:+.3f} hand {d2:+.3f}")
        if d < worst[0]:
            worst = (d, name)
    # table
    dt = min(P[:, 2].min() - LINK_R, H[:, 2].min()) - TABLE_Z
    if verbose:
        print(f"  table: {dt:+.3f}")
    if dt < worst[0]:
        worst = (dt, "table")
    return worst


def check_path(qs, ignore=(), steps=10, extra=None):
    worst = (1e9, None, -1)
    for i in range(len(qs) - 1):
        for s in np.linspace(0, 1, steps):
            q = np.array(qs[i]) * (1 - s) + np.array(qs[i + 1]) * s
            d, n = check(q, ignore, extra=extra)
            if d < worst[0]:
                worst = (d, n, i + s)
    return worst
OPENRUA_EOF

# openrua op 27
timeout 60 python3 -c "
import numpy as np, math
from panda import *
from collide import *
np.set_printoptions(precision=4, suppress=True)
q0=np.array([0.0,-0.161037389,0.0,-2.44459747,0.0,2.2267522,0.7853981633974483])
print('current cfg clearance:'); print(check(q0, verbose=True))
fr = frames(q0)
for i,f in enumerate(fr): print(i, f[:3,3].round(3))
print('tcp', fk_tcp(q0)[:3,3].round(3))
for name,(p,yaw) in {'above_bowl':([0.075,-0.045,1.15],math.pi/2),'bowl_grasp':([0.075,-0.045,0.99],math.pi/2)}.items():
    q=ik(make_T(p, topdown(yaw)), q0); print(name, q.round(3)); print(check(q, ignore=('bowl',), verbose=True))
"

# openrua op 28
python3 -c "
import numpy as np
from cloud import cloud
pw, color = cloud('birdview')
Z = pw[...,2]; X = pw[...,0]; Y = pw[...,1]
ok = np.isfinite(Z)&(Z>0.5)&(Z<1.6)
xs = np.arange(-0.30,0.08,0.02); ys=np.arange(-0.40,-0.02,0.02)
print('      '+' '.join(f'{y:5.2f}' for y in ys))
for x in xs:
    row=[]
    for y in ys:
        m = ok&(np.abs(X-x)<0.01)&(np.abs(Y-y)<0.01)
        row.append(Z[m].max() if m.any() else np.nan)
    print(f'{x:5.2f} '+' '.join(f'{v:5.2f}' for v in row))
"


# openrua op 29
python3 -c "
import numpy as np
from cloud import cloud
pw, color = cloud('birdview')
X,Y,Z = pw[...,0],pw[...,1],pw[...,2]
m = np.isfinite(Z)&(Z>0.95)&(Z<1.035)&(X>-0.1)&(X<0.1)&(Y>-0.16)&(Y<0.03)
P = pw[m]
print('bowl pts', len(P), 'x', P[:,0].min().round(3), P[:,0].max().round(3), 'y', P[:,1].min().round(3), P[:,1].max().round(3), 'zmax', P[:,2].max().round(3))
rim = P[P[:,2]>1.005]
# fit circle to rim xy
A = np.c_[2*rim[:,0], 2*rim[:,1], np.ones(len(rim))]; b = (rim[:,0]**2+rim[:,1]**2)
c = np.linalg.lstsq(A,b,rcond=None)[0]; r = np.sqrt(c[2]+c[0]**2+c[1]**2)
print('rim circle center', c[:2].round(3), 'radius', r.round(3), 'n', len(rim))
# shelf box
m2 = np.isfinite(Z)&(Z>0.95)&(Z<1.26)&(X>-0.3)&(X<0.1)&(Y<-0.15)
S=pw[m2]; print('shelf x', S[:,0].min().round(3), S[:,0].max().round(3), 'y', S[:,1].min().round(3), S[:,1].max().round(3), 'z', S[:,2].max().round(3))
"


# openrua op 30
python3 - <<'EOF'
import re
s=open('collide.py').read()
s=s.replace('"shelf": ([-0.245, -0.38, 0.90], [0.04, -0.075, 1.31]),','"shelf": ([-0.245, -0.38, 0.90], [0.04, -0.17, 1.25]),')
s=s.replace('"bowl": ([-0.085, -0.13, 0.90], [0.085, 0.04, 1.03]),','"bowl": ([-0.06, -0.09, 0.90], [0.065, 0.03, 1.03]),')
open('collide.py','w').write(s)
EOF
cat > step1.py <<'EOF'
import numpy as np, math, sys
from panda import *
from collide import check, check_path
np.set_printoptions(precision=4, suppress=True)
r = Robot()
q0 = r.q(); print('q0', q0.round(3), 'tcp', fk_tcp(q0)[:3,3].round(3))
r.gripper(0.04)
Tt = make_T([0.055, -0.029, 1.15], topdown(math.pi/2))
qt = ik(Tt, q0); print('target q', qt.round(3), 'clear', check(qt, ignore=('bowl',)))
print('path clear', check_path([q0, qt], ignore=('bowl',)))
if '--go' in sys.argv:
    print(r.move_q(qt, 3.0))
    print('tcp now', r.tcp()[:3,3].round(4))
EOF
timeout 300 python3 step1.py

# openrua op 31
timeout 600 python3 step1.py --go && timeout 60 python3 grab.py robot0_eye_in_hand

# openrua op 32
timeout 60 python3 -c "
from panda import *
import numpy as np; np.set_printoptions(precision=4, suppress=True)
r=Robot(); q=r.q(); print(q.round(3)); print(fk_tcp(q)[:3,3].round(4)); print(r.wrench())
qt=np.array([-0.095,0.604,0.071,-1.431,-0.045,2.034,-0.802]); print('err', (q-qt).round(3))
"

# openrua op 33
cat > goq.py <<'EOF'
import sys, numpy as np
from panda import *
np.set_printoptions(precision=4, suppress=True)
r = Robot()
qt = np.array([float(x) for x in sys.argv[1].split(',')])
T = float(sys.argv[2]) if len(sys.argv) > 2 else 4.0
print(r.move_q(qt, T))
q = r.q(); print('q', q.round(3), 'err', (q-qt).round(3)); print('tcp', fk_tcp(q)[:3,3].round(4))
EOF
timeout 600 python3 goq.py -0.095,0.604,0.071,-1.431,-0.045,2.034,-0.802 4

# openrua op 34
python3 - <<'EOF'
s=open('panda.py').read()
s=s.replace('''    def move_q(self, qt, T=3.0):
        code = self.move_traj([qt], [T])''','''    def move_q(self, qt, T=None, vmax=0.15):
        qc = self.q()
        Tmin = float(np.abs(np.array(qt) - qc).max()) / vmax + 0.5
        T = Tmin if T is None else max(T, Tmin)
        code = self.move_traj([qt], [T])''')
s=s.replace('''    def move_tcp(self, Tt, T=3.0, q0=None, n_wp=1):''','''    def move_tcp(self, Tt, T=None, q0=None, n_wp=1, vmax=0.15):''')
s=s.replace('''            qs.append(qi); ts.append(T * s); qprev = qi
        code = self.move_traj(qs, ts)''','''            qs.append(qi); qprev = qi
        # time parametrise by joint velocity cap
        ts, t = [], 0.0
        qprev = q0
        for qi in qs:
            t += max(float(np.abs(qi - qprev).max()) / vmax, 0.05)
            ts.append(t); qprev = qi
        if T is not None and T > ts[-1]:
            ts = [x * T / ts[-1] for x in ts]
        ts = [x + 0.3 for x in ts]
        code = self.move_traj(qs, ts)''')
open('panda.py','w').write(s)
EOF
timeout 600 python3 goq.py -0.095,0.604,0.071,-1.431,-0.045,2.034,-0.802

# openrua op 35
timeout 60 python3 grab.py robot0_eye_in_hand >/dev/null && python3 -c "
import numpy as np
from cloud import cloud
pw, color = cloud('robot0_eye_in_hand')
X,Y,Z = pw[...,0],pw[...,1],pw[...,2]
m = np.isfinite(Z)&(Z>0.95)&(Z<1.05)&(X>-0.15)&(X<0.2)&(Y>-0.2)&(Y<0.05)
P = pw[m]
print('pts', len(P), 'x', P[:,0].min().round(3), P[:,0].max().round(3), 'y', P[:,1].min().round(3), P[:,1].max().round(3), 'zmax', P[:,2].max().round(3))
rim = P[P[:,2]>1.005]
A = np.c_[2*rim[:,0], 2*rim[:,1], np.ones(len(rim))]; b = (rim[:,0]**2+rim[:,1]**2)
c = np.linalg.lstsq(A,b,rcond=None)[0]; r = np.sqrt(c[2]+c[0]**2+c[1]**2)
print('rim circle center', c[:2].round(3), 'radius', r.round(3), 'n', len(rim))
"

# openrua op 36
cat > step2.py <<'EOF'
import numpy as np, math, sys
from panda import *
from collide import check, check_path
np.set_printoptions(precision=4, suppress=True)
r = Robot()
q0 = r.q(); print('q0', q0.round(3), 'tcp', fk_tcp(q0)[:3,3].round(3))
Tt = make_T([0.050, -0.029, 0.99], topdown(math.pi/2))
qt = ik(Tt, q0); print('target q', qt.round(3), 'clear', check(qt, ignore=('bowl',), verbose=True))
print('path clear', check_path([q0, qt], ignore=('bowl',)))
if '--go' in sys.argv:
    print(r.move_tcp(Tt, n_wp=4))
    print('tcp now', r.tcp()[:3,3].round(4), 'wrench', r.wrench().round(2))
    r.gripper(0.0)
    print('fingers', r.fingers())
EOF
timeout 600 python3 step2.py --go

# openrua op 37
cat > step3.py <<'EOF'
import numpy as np, math, sys
from panda import *
from collide import check, check_path
np.set_printoptions(precision=4, suppress=True)
r = Robot()
q0 = r.q(); print('q0', q0.round(3), 'tcp', fk_tcp(q0)[:3,3].round(3))
# lift straight up
T1 = make_T([0.050, -0.029, 1.30], topdown(math.pi/2))
T2 = make_T([-0.35, 0.30, 1.30], topdown(math.pi/2))
T3 = make_T([-0.35, 0.30, 1.00], topdown(math.pi/2))
q1 = ik(T1, q0); q2 = ik(T2, q1); q3 = ik(T3, q2)
for n,q in (('q1',q1),('q2',q2),('q3',q3)):
    print(n, q.round(3), check(q, ignore=('bowl',)))
print('path', check_path([q0,q1,q2,q3], ignore=('bowl',)))
if '--go' in sys.argv:
    print(r.move_tcp(T1, n_wp=3)); print('fingers', r.fingers())
    print(r.move_q(q2)); print('tcp', r.tcp()[:3,3].round(3), 'fingers', r.fingers())
    print(r.move_tcp(T3, n_wp=3)); print('tcp', r.tcp()[:3,3].round(3), 'wrench', r.wrench().round(2))
    r.gripper(0.04)
    T4 = make_T([-0.35, 0.30, 1.20], topdown(math.pi/2))
    print(r.move_tcp(T4, n_wp=2)); print('tcp', r.tcp()[:3,3].round(3))
EOF
timeout 900 python3 step3.py --go

# openrua op 38
timeout 120 python3 grab.py birdview agentview >/dev/null; python3 -c "
import numpy as np
from cloud import cloud
pw, color = cloud('birdview')
X,Y,Z = pw[...,0],pw[...,1],pw[...,2]
m = np.isfinite(Z)&(Z>0.95)&(Z<1.035)&(X>-0.5)&(X<0.3)&(Y>-0.16)&(Y<0.5)
P = pw[m]
# cluster roughly: print bounding boxes of points in coarse grid cells
from collections import defaultdict
cells=defaultdict(list)
for p in P: cells[(round(p[0]/0.1),round(p[1]/0.1))].append(p)
for k,v in sorted(cells.items()):
    v=np.array(v); print(k, len(v), 'x',v[:,0].min().round(3),v[:,0].max().round(3),'y',v[:,1].min().round(3),v[:,1].max().round(3),'z',v[:,2].max().round(3))
"

# openrua op 39
python3 -c "
import numpy as np
from cloud import cloud
pw, color = cloud('birdview')
Z = pw[...,2]; X = pw[...,0]; Y = pw[...,1]
ok = np.isfinite(Z)&(Z>0.5)&(Z<1.6)
xs = np.arange(-0.16,0.14,0.02); ys=np.arange(-0.16,0.10,0.02)
print('      '+' '.join(f'{y:5.2f}' for y in ys))
for x in xs:
    row=[]
    for y in ys:
        m = ok&(np.abs(X-x)<0.01)&(np.abs(Y-y)<0.01)
        row.append(Z[m].max() if m.any() else np.nan)
    print(f'{x:5.2f} '+' '.join(f'{v:5.2f}' for v in row))
"


# openrua op 40
python3 -c "
import numpy as np
from cloud import cloud
pw, color = cloud('birdview')
X,Y,Z = pw[...,0],pw[...,1],pw[...,2]
ok=np.isfinite(Z)
def box(name, m):
    P=pw[m]; print(f'{name}: n={len(P)} x {P[:,0].min():.3f}..{P[:,0].max():.3f} y {P[:,1].min():.3f}..{P[:,1].max():.3f} z {P[:,2].min():.3f}..{P[:,2].max():.3f}')
    return P
B=box('bottle', ok&(Z>0.95)&(Z<1.3)&(X>-0.2)&(X<-0.09)&(Y>0.0)&(Y<0.12))
for zl in np.arange(0.95,1.16,0.02):
    S=B[(B[:,2]>=zl)&(B[:,2]<zl+0.02)]
    if len(S): print(f'  z {zl:.2f}: n={len(S)} x {S[:,0].min():.3f}..{S[:,0].max():.3f} y {S[:,1].min():.3f}..{S[:,1].max():.3f}')
box('bowl', ok&(Z>0.92)&(Z<1.0)&(X>-0.08)&(X<0.09)&(Y>-0.12)&(Y<0.05))
box('cabinet top', ok&(Z>1.05)&(X>-0.2)&(X<0.2)&(Y>0.15)&(Y<0.5))
box('drawer walls', ok&(Z>0.96)&(Z<1.0)&(X>-0.15)&(X<0.15)&(Y>0.05)&(Y<0.22))
box('drawer floor', ok&(Z>0.91)&(Z<0.93)&(X>-0.15)&(X<0.15)&(Y>0.05)&(Y<0.25))
box('shelf', ok&(Z>0.95)&(Z<1.3)&(X>-0.3)&(X<0.1)&(Y<-0.15))
print('table z', np.median(Z[ok&(np.abs(X+0.4)<0.05)&(np.abs(Y)<0.05)]))
"

# openrua op 41
python3 - <<'EOF'
s=open('collide.py').read()
s=s.replace('"drawer": ([-0.115, 0.06, 0.90], [0.135, 0.21, 0.99]),','"drawer": ([-0.115, 0.06, 0.90], [0.135, 0.21, 1.0]),')
s=s.replace('"bottle": ([-0.165, 0.025, 0.90], [-0.10, 0.09, 1.135]),','"bottle": ([-0.165, 0.025, 0.90], [-0.10, 0.09, 1.06]),')
s=s.replace('"bowl": ([-0.06, -0.09, 0.90], [0.065, 0.03, 1.03]),','"bowl": ([-0.06, -0.09, 0.90], [0.065, 0.03, 0.955]),')
open('collide.py','w').write(s)
EOF
cat > step4.py <<'EOF'
import numpy as np, math, sys
from panda import *
from collide import check, check_path
np.set_printoptions(precision=4, suppress=True)
r = Robot()
q0 = r.q(); print('q0', q0.round(3), 'tcp', fk_tcp(q0)[:3,3].round(3))
R = topdown(math.pi/2)   # finger axis along world x
Ta = make_T([-0.132, 0.058, 1.20], R)
Tg = make_T([-0.132, 0.058, 1.025], R)
qa = ik(Ta, q0); qg = ik(Tg, qa)
print('qa', qa.round(3), check(qa, verbose=False))
print('qg', qg.round(3), check(qg, ignore=('bottle',), verbose=True))
print('path0', check_path([q0, qa]), 'path1', check_path([qa, qg], ignore=('bottle',)))
if '--go' in sys.argv:
    print(r.move_q(qa)); print('tcp', r.tcp()[:3,3].round(4))
    print(r.move_tcp(Tg, n_wp=3)); print('tcp', r.tcp()[:3,3].round(4))
    r.gripper(0.0)
    print('fingers', r.fingers(), 'wrench', r.wrench().round(2))
EOF
timeout 900 python3 step4.py --go

# openrua op 42
cat > step5.py <<'EOF'
import numpy as np, math, sys
from panda import *
from collide import check, check_path
np.set_printoptions(precision=4, suppress=True)
r = Robot()
q0 = r.q(); print('q0', q0.round(3), 'tcp', fk_tcp(q0)[:3,3].round(3), 'fingers', r.fingers())
R = topdown(math.pi/2)
T1 = make_T([-0.132, 0.058, 1.20], R)     # lift
T2 = make_T([-0.04, 0.14, 1.20], R)       # over drawer open area
T3 = make_T([-0.04, 0.14, 1.052], R)      # bottle base ~4 mm above drawer floor (0.923)
q1 = ik(T1, q0); q2 = ik(T2, q1); q3 = ik(T3, q2)
for n,q in (('q1',q1),('q2',q2),('q3',q3)):
    print(n, q.round(3), check(q, ignore=('bottle','drawer'), verbose=(n=='q3')))
print('path', check_path([q0,q1,q2], ignore=('bottle',)), check_path([q2,q3], ignore=('bottle','drawer')))
if '--go' in sys.argv:
    print(r.move_tcp(T1, n_wp=2)); print('tcp', r.tcp()[:3,3].round(4), 'fingers', r.fingers())
    print(r.move_q(q2)); print('tcp', r.tcp()[:3,3].round(4), 'fingers', r.fingers())
    print(r.move_tcp(T3, n_wp=3)); print('tcp', r.tcp()[:3,3].round(4), 'fingers', r.fingers(), 'wrench', r.wrench().round(2))
    r.gripper(0.04)
    print(r.move_tcp(T2, n_wp=2)); print('tcp', r.tcp()[:3,3].round(4))
EOF
timeout 900 python3 step5.py --go && python3 grab.py birdview >/dev/null && python3 - <<'EOF'
import numpy as np
from cloud import cloud
pw,_ = cloud('birdview')
X,Y,Z = pw[...,0],pw[...,1],pw[...,2]
# region of drawer + surroundings
m = np.isfinite(Z)&(X>-0.2)&(X<0.2)&(Y>0.0)&(Y<0.45)
for zlo,zhi in [(0.93,0.96),(0.96,1.0),(1.0,1.05),(1.05,1.09),(1.09,1.2)]:
    s = m&(Z>=zlo)&(Z<zhi)
    if s.sum()>5: print(f"z {zlo}-{zhi}: n={s.sum()} x {X[s].min():.3f}..{X[s].max():.3f} y {Y[s].min():.3f}..{Y[s].max():.3f}")
# anything tall near where bottle should be
s = m&(Z>1.03)&(Y<0.19)
print('bottle top region: n',s.sum(), 'x',X[s].min() if s.sum() else None, X[s].max() if s.sum() else None,'y',Y[s].min() if s.sum() else None,Y[s].max() if s.sum() else None,'zmax',Z[s].max() if s.sum() else None)
EOF

# openrua op 43
cat > park.py <<'EOF'
import numpy as np, math, sys
from panda import *
from collide import check, check_path
r = Robot()
q0 = r.q()
p = [float(v) for v in sys.argv[1].split(',')]
T = make_T(p, topdown(math.pi/2))
qt = ik(T, q0); print('qt', qt.round(3), check(qt), check_path([q0,qt]))
print(r.move_q(qt)); print('tcp', r.tcp()[:3,3].round(4))
EOF
timeout 600 python3 park.py -0.30,-0.05,1.35 && python3 grab.py birdview >/dev/null && python3 - <<'EOF'
import numpy as np
from cloud import cloud
pw,_ = cloud('birdview')
X,Y,Z = pw[...,0],pw[...,1],pw[...,2]
m = np.isfinite(Z)&(X>-0.2)&(X<0.2)&(Y>0.0)&(Y<0.45)
for zlo,zhi in [(0.93,0.96),(0.96,1.0),(1.0,1.05),(1.05,1.09),(1.09,1.2)]:
    s = m&(Z>=zlo)&(Z<zhi)&(Y<0.195)
    if s.sum()>3: print(f"z {zlo}-{zhi} (y<0.195): n={s.sum()} x {X[s].min():.3f}..{X[s].max():.3f} y {Y[s].min():.3f}..{Y[s].max():.3f}")
s = m&(Z>1.03)&(Y<0.195)
print('bottle top: n',s.sum(), 'x %.3f..%.3f y %.3f..%.3f zmax %.3f'%(X[s].min(),X[s].max(),Y[s].min(),Y[s].max(),Z[s].max()) if s.sum() else '')
# old bottle spot
s = m&(X<-0.10)&(Y<0.12)&(Z>0.92)
print('old bottle spot pts above table:', s.sum())
EOF

# openrua op 44
python3 - <<'EOF'
import numpy as np
from cloud import cloud
pw,_ = cloud('birdview')
X,Y,Z = pw[...,0],pw[...,1],pw[...,2]
t = np.isfinite(Z)&(np.abs(Z-0.90)<0.01)
print('table extent x %.2f..%.2f y %.2f..%.2f'%(X[t].min(),X[t].max(),Y[t].min(),Y[t].max()))
s = np.isfinite(Z)&(X>0.06)&(X<0.4)&(Y>-0.15)&(Y<0.1)&(Z>0.91)
print('objects in bowl-destination zone:', s.sum())
b = np.isfinite(Z)&(Z>0.92)&(Z<0.96)&(X>-0.08)&(X<0.08)&(Y>-0.12)&(Y<0.04)
print('bowl now x %.3f..%.3f y %.3f..%.3f zmax %.3f'%(X[b].min(),X[b].max(),Y[b].min(),Y[b].max(),Z[b].max()))
EOF

# openrua op 45
python3 - <<'EOF'
import numpy as np
from cloud import cloud
pw,_ = cloud('birdview')
X,Y,Z = pw[...,0],pw[...,1],pw[...,2]
s = np.isfinite(Z)&(X>0.06)&(X<0.4)&(Y>-0.15)&(Y<0.1)&(Z>0.91)
print('x %.3f..%.3f y %.3f..%.3f z %.3f..%.3f'%(X[s].min(),X[s].max(),Y[s].min(),Y[s].max(),Z[s].min(),Z[s].max()))
EOF

# openrua op 46
cat > step6.py <<'EOF'
import numpy as np, math, sys
from panda import *
from collide import check, check_path
np.set_printoptions(precision=4, suppress=True)
r = Robot()
q0 = r.q(); print('q0', q0.round(3), 'tcp', fk_tcp(q0)[:3,3].round(3))
R = topdown(math.pi/2)
Ta = make_T([-0.085, -0.03, 1.10], R)   # above start
Tb = make_T([-0.085, -0.03, 0.93], R)   # pusher down behind bowl (-x side)
Tc = make_T([ 0.13, -0.03, 0.93], R)    # push +x
Td = make_T([ 0.13, -0.03, 1.15], R)    # lift
qa = ik(Ta, q0); qb = ik(Tb, qa); qc = ik(Tc, qb); qd = ik(Td, qc)
for n,q in (('qa',qa),('qb',qb),('qc',qc),('qd',qd)):
    print(n, q.round(3), check(q, ignore=('bowl',), verbose=(n in('qb','qc'))))
print('paths', check_path([q0,qa]), check_path([qa,qb], ignore=('bowl',)), check_path([qb,qc], ignore=('bowl',)), check_path([qc,qd], ignore=('bowl',)))
if '--go' in sys.argv:
    r.gripper(0.0)
    print(r.move_q(qa)); print('tcp', r.tcp()[:3,3].round(4))
    print(r.move_tcp(Tb, n_wp=2)); print('tcp', r.tcp()[:3,3].round(4))
    print(r.move_tcp(Tc, n_wp=5)); print('tcp', r.tcp()[:3,3].round(4), 'wrench', r.wrench().round(2))
    print(r.move_tcp(Td, n_wp=2)); print('tcp', r.tcp()[:3,3].round(4))
EOF
timeout 900 python3 step6.py --go && timeout 600 python3 park.py -0.30,-0.05,1.35 && python3 grab.py birdview >/dev/null && python3 - <<'EOF'
import numpy as np
from cloud import cloud
pw,_ = cloud('birdview')
X,Y,Z = pw[...,0],pw[...,1],pw[...,2]
b = np.isfinite(Z)&(Z>0.92)&(Z<0.97)&(X>-0.1)&(X<0.45)&(Y>-0.2)&(Y<0.05)
print('bowl now x %.3f..%.3f y %.3f..%.3f zmax %.3f n %d'%(X[b].min(),X[b].max(),Y[b].min(),Y[b].max(),Z[b].max(),b.sum()))
s = np.isfinite(Z)&(Z>1.03)&(Y<0.195)&(Y>0)&(X>-0.2)&(X<0.2)
print('bottle top: x %.3f..%.3f y %.3f..%.3f zmax %.3f'%(X[s].min(),X[s].max(),Y[s].min(),Y[s].max(),Z[s].max()))
EOF

# openrua op 47
python3 - <<'EOF'
import numpy as np
from cloud import cloud
pw,_ = cloud('birdview')
X,Y,Z = pw[...,0],pw[...,1],pw[...,2]
b = np.isfinite(Z)&(Z>0.915)&(Z<0.97)&(X>-0.1)&(X<0.45)&(Y>-0.2)&(Y<0.05)
for xlo in np.arange(-0.1,0.45,0.05):
    s=b&(X>=xlo)&(X<xlo+0.05)
    if s.sum(): print(f"x {xlo:.2f}: n={s.sum()} y {Y[s].min():.3f}..{Y[s].max():.3f} z {Z[s].min():.3f}..{Z[s].max():.3f}")
EOF

# openrua op 48
cat > step7.py <<'EOF'
import numpy as np, math, sys
from panda import *
from collide import check, check_path
np.set_printoptions(precision=4, suppress=True)
r = Robot()
q0 = r.q(); print('q0', q0.round(3), 'tcp', fk_tcp(q0)[:3,3].round(3), 'fingers', r.fingers())
R = topdown(math.pi/2)
Ta = make_T([0.08, 0.12, 1.20], R)
Tb = make_T([0.08, 0.12, 0.975], R)   # hook inside, behind front panel
Tc = make_T([0.08, 0.00, 0.975], R)   # pull -y 12 cm
Td = make_T([0.08, 0.00, 1.20], R)
qa = ik(Ta, q0); qb = ik(Tb, qa); qc = ik(Tc, qb); qd = ik(Td, qc)
for n,q in (('qa',qa),('qb',qb),('qc',qc),('qd',qd)):
    print(n, q.round(3), check(q, ignore=('drawer','bottle','bowl'), verbose=(n in('qb','qc'))))
print('paths', check_path([q0,qa], ignore=('bowl',)), check_path([qa,qb], ignore=('drawer','bottle','bowl')), check_path([qb,qc], ignore=('drawer','bottle','bowl')))
if '--go' in sys.argv:
    print(r.move_q(qa)); print('tcp', r.tcp()[:3,3].round(4))
    print(r.move_tcp(Tb, n_wp=3)); print('tcp', r.tcp()[:3,3].round(4))
    print(r.move_tcp(Tc, n_wp=6)); print('tcp', r.tcp()[:3,3].round(4), 'wrench', r.wrench().round(2))
    print(r.move_tcp(Td, n_wp=2)); print('tcp', r.tcp()[:3,3].round(4))
EOF
timeout 900 python3 step7.py --go && timeout 600 python3 park.py -0.30,-0.05,1.35 && python3 grab.py birdview >/dev/null && python3 - <<'EOF'
import numpy as np
from cloud import cloud
pw,_ = cloud('birdview')
X,Y,Z = pw[...,0],pw[...,1],pw[...,2]
m = np.isfinite(Z)&(X>-0.2)&(X<0.2)&(Y>-0.15)&(Y<0.45)
for zlo,zhi in [(0.915,0.93),(0.93,0.96),(0.96,1.0),(1.0,1.05),(1.05,1.09)]:
    s = m&(Z>=zlo)&(Z<zhi)&(Y<0.195)
    if s.sum()>3: print(f"z {zlo}-{zhi} (y<0.195): n={s.sum()} x {X[s].min():.3f}..{X[s].max():.3f} y {Y[s].min():.3f}..{Y[s].max():.3f}")
# front panel top edge (z 0.96-1.0): y-profile
s = m&(Z>0.96)&(Z<1.0)&(X>-0.02)&(X<0.02)
print('panel/handle y-values at x~0:', np.unique(np.round(Y[s],3))[:20])
s = m&(Z>1.03)&(Y<0.195)
print('bottle top: x %.3f..%.3f y %.3f..%.3f zmax %.3f'%(X[s].min(),X[s].max(),Y[s].min(),Y[s].max(),Z[s].max()))
EOF

# openrua op 49
cat > step7b.py <<'EOF'
import numpy as np, math, sys
from panda import *
from collide import check, check_path
np.set_printoptions(precision=4, suppress=True)
r = Robot()
q0 = r.q(); print('q0', q0.round(3), 'tcp', fk_tcp(q0)[:3,3].round(3), 'fingers', r.fingers())
R = topdown(math.pi/2)
Ta = make_T([0.08, 0.10, 1.20], R)
Tb = make_T([0.08, 0.10, 0.975], R)
Tc = make_T([0.08, 0.07, 0.975], R)   # gentle 3 cm pull
Td = make_T([0.08, 0.07, 1.20], R)
qa = ik(Ta, q0); qb = ik(Tb, qa); qc = ik(Tc, qb); qd = ik(Td, qc)
print('paths', check_path([q0,qa], ignore=('bowl',)), check_path([qa,qb], ignore=('drawer','bottle','bowl')))
print(r.move_q(qa)); print(r.move_tcp(Tb, n_wp=3)); print('tcp', r.tcp()[:3,3].round(4), 'wrench', r.wrench().round(2))
print(r.move_tcp(Tc, n_wp=3, vmax=0.05)); print('tcp', r.tcp()[:3,3].round(4), 'wrench', r.wrench().round(2))
print(r.move_tcp(Td, n_wp=2)); print('tcp', r.tcp()[:3,3].round(4))
EOF
timeout 900 python3 step7b.py && timeout 600 python3 park.py -0.30,-0.05,1.35 && python3 grab.py birdview >/dev/null && python3 - <<'EOF'
import numpy as np
from cloud import cloud
pw,_ = cloud('birdview')
X,Y,Z = pw[...,0],pw[...,1],pw[...,2]
m = np.isfinite(Z)&(X>-0.2)&(X<0.2)
s = m&(Z>0.96)&(Z<1.0)&(X>-0.09)&(X<-0.06)
print('panel top y at x~-0.075:', np.unique(np.round(Y[s],3)))
s = m&(Z>0.94)&(Z<0.96)&(X>-0.02)&(X<0.02)&(Y>-0.1)&(Y<0.1)
print('handle bar y at x~0:', np.unique(np.round(Y[s],3)))
s = m&(Z>1.03)&(Y<0.195)&(Y>0)
print('bottle top: x %.3f..%.3f y %.3f..%.3f zmax %.3f'%(X[s].min(),X[s].max(),Y[s].min(),Y[s].max(),Z[s].max()))
EOF

# openrua op 50
cat > geom.py <<'EOF'
"""Key-point clearance check for hand (+held bottle) against scene boxes."""
import numpy as np, math
from panda import R_from_axes, make_T
BOXES = {
 'panel':  ([-0.11, 0.070, 0.90], [0.12, 0.085, 1.00]),
 'handle': ([-0.04, 0.039, 0.94], [0.05, 0.053, 0.96]),
 'wall-x': ([-0.11, 0.085, 0.90], [-0.09, 0.30, 1.00]),
 'wall+x': ([0.11, 0.085, 0.90], [0.12, 0.30, 1.00]),
 'cabinet':([-0.116, 0.198, 0.90], [0.136, 0.417, 1.127]),
 'tophandle':([-0.045, 0.165, 1.04], [0.055, 0.20, 1.115]),
 'floor':  ([-0.09, 0.085, 0.80], [0.11, 0.30, 0.923]),
 'table':  ([-1, -1, 0.0], [1, 1, 0.90]),
}
def hand_pts(T, open_=True):
    """points in hand frame (TCP origin, z=approach, y=finger axis)."""
    pts = []
    for x in (-0.03, 0.03):
        for y in np.linspace(-0.10, 0.10, 11):
            for z in (-0.1034, -0.07, -0.0374):   # palm from flange side to bottom face
                pts.append([x, y, z])
    fy = 0.05 if open_ else 0.017
    for x in (-0.01, 0.01):
        for y in (-fy, fy):
            for z in np.linspace(-0.0374, 0.0, 5):  # fingers from palm to pad center (pad center = TCP)
                pts.append([x, y, z])
                pts.append([x, y, z + 0.0115])      # pad tips a bit beyond TCP
    P = np.array(pts)
    return (T[:3, :3] @ P.T).T + T[:3, 3]
def bottle_pts(T, axis_dir_hand):
    """bottle held at neck (TCP), 2 cm from tip; body toward axis_dir_hand (unit, hand frame)."""
    d = np.array(axis_dir_hand, float)
    pts = []
    # neck: from -0.02 to +0.04 along d, r=0.007; body from 0.04 to 0.14, r=0.0275
    for s, r in [(-0.02, 0.007), (0.0, 0.007), (0.04, 0.007), (0.045, 0.0275), (0.09, 0.0275), (0.14, 0.0275)]:
        for ang in np.linspace(0, 2*math.pi, 12, endpoint=False):
            # perpendicular basis in hand frame
            u = np.cross(d, [0, 0, 1.0]);
            if np.linalg.norm(u) < 1e-6: u = np.cross(d, [0, 1.0, 0])
            u /= np.linalg.norm(u); v = np.cross(d, u)
            pts.append(s * d + r * (math.cos(ang) * u + math.sin(ang) * v))
    P = np.array(pts)
    return (T[:3, :3] @ P.T).T + T[:3, 3]
def dist(P, lo, hi):
    lo, hi = np.array(lo), np.array(hi)
    d = np.maximum(np.maximum(lo - P, 0), P - hi)
    return np.linalg.norm(d, axis=-1).min()
def report(T, open_=True, bottle_dir=None, skip=()):
    H = hand_pts(T, open_)
    out = {}
    for n, (lo, hi) in BOXES.items():
        if n in skip: continue
        out[n] = dist(H, lo, hi)
        if bottle_dir is not None:
            out[n + '/bottle'] = dist(bottle_pts(T, bottle_dir), lo, hi)
    return out
def place_pose(beta, gamma, p):
    """finger axis f rolled by beta about x (+y finger up), approach down/+y; then pitch gamma about world y."""
    f = np.array([0, math.cos(beta), math.sin(beta)])
    a = np.array([0, math.sin(beta), -math.cos(beta)])
    R0 = R_from_axes(a, y=f)
    c, s = math.cos(gamma), math.sin(gamma)
    Ry = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    return make_T(p, Ry @ R0)
if __name__ == '__main__':
    import sys
    beta, gamma, z = [float(v) for v in sys.argv[1:4]]
    T = place_pose(math.radians(beta), math.radians(gamma), [-0.05, 0.135, z])
    print('hand x axis (bottle axis = -x):', T[:3, 0].round(3), 'z axis', T[:3, 2].round(3))
    bd = [-1, 0, 0]  # body toward hand -x  (world +x)
    B = bottle_pts(T, bd)
    print('bottle world x %.3f..%.3f y %.3f..%.3f z %.3f..%.3f' % (B[:,0].min(), B[:,0].max(), B[:,1].min(), B[:,1].max(), B[:,2].min(), B[:,2].max()))
    for k, v in report(T, True, bd).items():
        print(f'  {k:18s} {v:+.3f}')
EOF
python3 geom.py 45 15 1.04

# openrua op 51
python3 - <<'EOF'
import math, numpy as np
from geom import *
best=[]
for beta in (40,45,50,55):
  for gamma in (0,10,15,20):
    for z in (1.02,1.03,1.04,1.05,1.06):
      for yc in (0.125,0.13,0.135,0.14):
        T = place_pose(math.radians(beta), math.radians(gamma), [-0.05, yc, z])
        rep = report(T, True, [-1,0,0])
        m = min(rep.values())
        B = bottle_pts(T,[-1,0,0])
        best.append((m, beta,gamma,z,yc, B[:,2].min(), min(rep,key=rep.get)))
best.sort(reverse=True)
for b in best[:12]: print('clr %.3f beta %d gamma %d z %.2f yc %.3f  bottle zmin %.3f  worst %s'%b)
EOF

# openrua op 52
python3 - <<'EOF'
import numpy as np
from cloud import cloud
pw,_ = cloud('birdview')
X,Y,Z = pw[...,0],pw[...,1],pw[...,2]
m = np.isfinite(Z)&(X>-0.2)&(X<0.2)&(Y>0.14)&(Y<0.21)&(Z>1.0)
for ylo in np.arange(0.14,0.21,0.005):
    s=m&(Y>=ylo)&(Y<ylo+0.005)
    if s.sum(): print(f"y {ylo:.3f}: n={s.sum()} x {X[s].min():.3f}..{X[s].max():.3f} z {Z[s].min():.3f}..{Z[s].max():.3f}")
EOF

# openrua op 53
ros2 topic list | grep image_raw; python3 grab.py agentview 2>/dev/null | tail -1; python3 - <<'EOF'
import cv2, numpy as np
im = cv2.imread('birdview.png'); h,w = im.shape[:2]
d = np.load('birdview.npz'); K=d['K']; T=d['T']
# project world points to pixels for crop: cabinet region
def px(p):
    pc = np.linalg.inv(T) @ np.array([*p,1.0]); u = K[0,0]*pc[0]/pc[2]+K[0,2]; v = K[1,1]*pc[1]/pc[2]+K[1,2]; return int(u),int(v)
u0,v0 = px([-0.2,-0.1,0.95]); u1,v1 = px([0.2,0.45,0.95])
crop = im[min(v0,v1):max(v0,v1), min(u0,u1):max(u0,u1)]
cv2.imwrite('crop_bird.png', cv2.resize(crop, None, fx=3, fy=3, interpolation=cv2.INTER_NEAREST))
print(crop.shape)
EOF

# openrua op 54
python3 grab.py frontview sideview 2>/dev/null | tail -2; python3 -c "
import cv2; im=cv2.imread('agentview.png'); cv2.imwrite('agent_big.png', im)"

# openrua op 55
python3 - <<'EOF'
import numpy as np
from cloud import cloud
pw,_ = cloud('birdview')
X,Y,Z = pw[...,0],pw[...,1],pw[...,2]
m = np.isfinite(Z)&(X>-0.2)&(X<0.2)&(Z>1.11)
print('cabinet top (z>1.11): y %.3f..%.3f x %.3f..%.3f'%(Y[m].min(),Y[m].max(),X[m].min(),X[m].max()))
m2 = np.isfinite(Z)&(X>0.06)&(X<0.2)&(Z>1.0)&(Y<0.25)
print('cabinet front region x>0.06 (no handle): ymin %.3f'%Y[m2].min())
# sideview cloud: look at the cabinet front face profile (y vs z) near x~0.08
pw2,_ = cloud('sideview'); X2,Y2,Z2 = pw2[...,0],pw2[...,1],pw2[...,2]
s = np.isfinite(Z2)&(X2>0.06)&(X2<0.10)&(Y2>0.05)&(Y2<0.3)&(Z2>0.9)&(Z2<1.15)
for zlo in np.arange(0.90,1.15,0.02):
    t=s&(Z2>=zlo)&(Z2<zlo+0.02)
    if t.sum(): print(f"side z {zlo:.2f}: ymin {Y2[t].min():.3f} ymax {Y2[t].max():.3f} n {t.sum()}")
EOF

# openrua op 56
python3 - <<'EOF'
import re
s=open('geom.py').read()
s=s.replace("'cabinet':([-0.116, 0.198, 0.90], [0.136, 0.417, 1.127]),","'cabinet':([-0.116, 0.228, 0.90], [0.136, 0.417, 1.127]),")
s=s.replace("'tophandle':([-0.045, 0.165, 1.04], [0.055, 0.20, 1.115]),","'tophandle':([-0.045, 0.188, 1.015], [0.055, 0.23, 1.105]),")
open('geom.py','w').write(s)
s=open('collide.py').read()
s=s.replace('"cabinet": ([-0.14, 0.20, 0.90], [0.145, 0.44, 1.14]),','"cabinet": ([-0.14, 0.228, 0.90], [0.145, 0.44, 1.14]),\n    "tophandle": ([-0.05, 0.185, 1.01], [0.06, 0.23, 1.11]),')
s=s.replace('"drawer": ([-0.115, 0.06, 0.90], [0.135, 0.21, 1.0]),','"drawer": ([-0.115, 0.035, 0.90], [0.135, 0.23, 1.0]),')
s=s.replace('"bottle": ([-0.165, 0.025, 0.90], [-0.10, 0.09, 1.06]),','"bottle": ([-0.08, 0.09, 0.90], [-0.01, 0.16, 1.09]),')
s=s.replace('"bowl": ([-0.06, -0.09, 0.90], [0.065, 0.03, 0.955]),','"bowl": ([0.13, -0.09, 0.90], [0.26, 0.03, 0.955]),')
open('collide.py','w').write(s)
EOF
python3 - <<'EOF'
import math, numpy as np
from geom import *
best=[]
for beta in (35,40,45,50,55):
  for gamma in (0,10,15,20):
    for z in (0.99,1.0,1.01,1.02,1.03,1.04):
      for yc in (0.13,0.14,0.15,0.16):
        T = place_pose(math.radians(beta), math.radians(gamma), [-0.05, yc, z])
        rep = report(T, True, [-1,0,0])
        m = min(rep.values())
        B = bottle_pts(T,[-1,0,0])
        best.append((m, beta,gamma,z,yc, B[:,2].min(), min(rep,key=rep.get)))
best.sort(reverse=True)
for b in best[:15]: print('clr %.3f beta %d gamma %d z %.2f yc %.3f  bottle zmin %.3f  worst %s'%b)
EOF

# openrua op 57
python3 - <<'EOF'
import math
from geom import *
for beta,gamma,z,yc in [(45,15,1.03,0.15),(45,15,1.03,0.14),(55,15,1.02,0.15),(35,15,1.03,0.15)]:
    T = place_pose(math.radians(beta), math.radians(gamma), [-0.05, yc, z])
    rep = report(T, True, [-1,0,0]); B=bottle_pts(T,[-1,0,0])
    print(beta,gamma,z,yc, 'bottle z %.3f..%.3f y %.3f..%.3f'%(B[:,2].min(),B[:,2].max(),B[:,1].min(),B[:,1].max()))
    print('   ', {k: round(v,3) for k,v in rep.items() if v<0.03})
EOF

# openrua op 58
python3 - <<'EOF'
import math, numpy as np
from geom import *
T = place_pose(math.radians(45), math.radians(15), [-0.05, 0.15, 1.03])
H = hand_pts(T, True)
lo,hi = np.array(BOXES['panel'][0]), np.array(BOXES['panel'][1])
d = np.linalg.norm(np.maximum(np.maximum(lo-H,0),H-hi),axis=-1)
i = d.argsort()[:5]
print(H[i].round(3), d[i].round(3))
print('hand axes x,y,z:\n', T[:3,:3].round(3))
EOF

# openrua op 59
python3 - <<'EOF'
import math
from geom import *
for beta,gamma,z,yc in [(45,15,1.035,0.155),(45,20,1.035,0.155),(40,20,1.035,0.16),(45,20,1.04,0.16),(42,18,1.035,0.158)]:
    T = place_pose(math.radians(beta), math.radians(gamma), [-0.05, yc, z])
    rep = report(T, True, [-1,0,0]); B=bottle_pts(T,[-1,0,0])
    print(beta,gamma,z,yc, 'bottle z %.3f..%.3f y %.3f..%.3f x %.3f..%.3f'%(B[:,2].min(),B[:,2].max(),B[:,1].min(),B[:,1].max(),B[:,0].min(),B[:,0].max()))
    print('   ', {k: round(v,3) for k,v in rep.items() if v<0.03})
EOF

# openrua op 60
python3 - <<'EOF'
import math
from geom import *
best=[]
for beta in (46,48,50,52,54):
  for gamma in (15,20,25):
    for z in (1.02,1.025,1.03,1.035):
      for yc in (0.135,0.14,0.145,0.15):
        T = place_pose(math.radians(beta), math.radians(gamma), [-0.05, yc, z])
        rep = report(T, True, [-1,0,0]); m=min(rep.values()); B=bottle_pts(T,[-1,0,0])
        best.append((m,beta,gamma,z,yc,B[:,2].min(),min(rep,key=rep.get)))
best.sort(reverse=True)
for b in best[:10]: print('clr %.3f beta %d gamma %d z %.3f yc %.3f bottle zmin %.3f worst %s'%b)
T = place_pose(math.radians(best[0][1]), math.radians(best[0][2]), [-0.05, best[0][4], best[0][3]])
print({k: round(v,3) for k,v in report(T,True,[-1,0,0]).items() if v<0.04})
EOF

# openrua op 61
python3 - <<'EOF'
import math
from geom import *
best=[]
for beta in (48,50,52,54,56,58):
  for gamma in (15,20,25):
    for z in (1.025,1.03,1.035,1.04):
      for yc in (0.115,0.12,0.125,0.13,0.135):
        T = place_pose(math.radians(beta), math.radians(gamma), [-0.05, yc, z])
        rep = report(T, True, [-1,0,0]); m=min(rep.values()); B=bottle_pts(T,[-1,0,0])
        best.append((m,beta,gamma,z,yc,B[:,2].min(),min(rep,key=rep.get)))
best.sort(reverse=True)
for b in best[:10]: print('clr %.3f beta %d gamma %d z %.3f yc %.3f bottle zmin %.3f worst %s'%b)
b=best[0]
T = place_pose(math.radians(b[1]), math.radians(b[2]), [-0.05, b[4], b[3]])
print({k: round(v,3) for k,v in report(T,True,[-1,0,0]).items() if v<0.04})
EOF

# openrua op 62
python3 - <<'EOF'
import math, numpy as np
from geom import *
T = place_pose(math.radians(50), math.radians(20), [-0.05, 0.125, 1.03])
H = hand_pts(T, True)
for name in ('panel','handle','wall-x'):
    lo,hi = np.array(BOXES[name][0]), np.array(BOXES[name][1])
    d = np.linalg.norm(np.maximum(np.maximum(lo-H,0),H-hi),axis=-1)
    i = d.argsort()[:3]
    print(name, H[i].round(3).tolist(), d[i].round(3))
EOF

# openrua op 63
python3 - <<'EOF'
import math, numpy as np
from geom import *
T = place_pose(math.radians(50), math.radians(20), [-0.05, 0.125, 1.03])
# rebuild hand-frame points list identically
pts=[]
for x in (-0.03,0.03):
    for y in np.linspace(-0.10,0.10,11):
        for z in (-0.1034,-0.07,-0.0374): pts.append([x,y,z])
fy=0.05
for x in (-0.01,0.01):
    for y in (-fy,fy):
        for z in np.linspace(-0.0374,0.0,5): pts.append([x,y,z]); pts.append([x,y,z+0.0115])
P=np.array(pts); W=(T[:3,:3]@P.T).T+T[:3,3]
sel=(W[:,1]>0.06)&(W[:,1]<0.095)&(W[:,2]<1.01)
for p,w in zip(P[sel],W[sel]): print('hand',p.round(3),'world',w.round(3))
print('R=\n',T[:3,:3].round(3))
EOF

# openrua op 64
python3 - <<'EOF'
import math
from geom import *
for beta,gamma,z,yc in [(50,25,1.045,0.14),(50,25,1.05,0.14),(50,20,1.045,0.14),(52,25,1.045,0.14),(50,25,1.045,0.145)]:
    T = place_pose(math.radians(beta), math.radians(gamma), [-0.05, yc, z])
    rep = report(T, True, [-1,0,0]); B=bottle_pts(T,[-1,0,0])
    print(beta,gamma,z,yc, 'bottle z %.3f..%.3f y %.3f..%.3f x %.3f..%.3f'%(B[:,2].min(),B[:,2].max(),B[:,1].min(),B[:,1].max(),B[:,0].min(),B[:,0].max()))
    print('   ', {k: round(v,3) for k,v in rep.items() if v<0.04})
EOF

# openrua op 65
cat > step8.py <<'EOF'
import numpy as np, math, sys
from panda import *
from collide import check, check_path
from geom import place_pose, report, bottle_pts
np.set_printoptions(precision=4, suppress=True)
r = Robot()
q0 = r.q(); print('q0', q0.round(3), 'tcp', fk_tcp(q0)[:3,3].round(3), 'fingers', r.fingers())
Rt = topdown(math.pi/2)
BX, BY = -0.044, 0.122
T1 = make_T([BX, BY, 1.20], Rt)
T2 = make_T([BX, BY, 1.048], Rt)      # neck grasp
T3 = make_T([BX, BY, 1.22], Rt)       # lift
T4 = make_T([-0.12, 0.0, 1.25], Rt)   # free space
Rp = place_pose(math.radians(50), math.radians(22), [0,0,0])[:3,:3]
T5 = make_T([-0.12, 0.0, 1.25], Rp)   # reoriented
T6 = make_T([-0.05, 0.14, 1.25], Rp)  # above drawer
T7 = make_T([-0.05, 0.14, 1.045], Rp) # release pose
ign = ('bottle','drawer','tophandle')
q1 = ik(T1, q0); q2 = ik(T2, q1); q3 = ik(T3, q2); q4 = ik(T4, q3)
# reorientation via slerp waypoints
qs5 = []; qp = q4
for s in np.linspace(0, 1, 8)[1:]:
    Ti = interp_T(T4, T5, s); qi = ik(Ti, qp); assert qi is not None, ('ik fail reorient', s); qs5.append(qi); qp = qi
q5 = qs5[-1]
q6 = ik(T6, q5); q7 = ik(T7, q6)
for n,q in (('q1',q1),('q2',q2),('q3',q3),('q4',q4),('q5',q5),('q6',q6),('q7',q7)):
    print(n, q.round(3), check(q, ignore=ign, verbose=(n in ('q5','q7'))))
print('paths', check_path([q0,q1], ignore=('bottle',)), check_path([q1,q2,q3], ignore=ign), check_path([q3,q4], ignore=ign),
      check_path([q4]+qs5, ignore=ign), check_path([q5,q6], ignore=ign), check_path([q6,q7], ignore=ign))
# geometric check of descent
for s in np.linspace(0,1,6):
    Ti = interp_T(T6, T7, s); rep = report(Ti, False, [-1,0,0]); print('descent s=%.1f min clr %.3f (%s)'%(s, min(rep.values()), min(rep,key=rep.get)))
rep = report(T7, True, [-1,0,0]); print('release (open) min clr %.3f (%s)'%(min(rep.values()), min(rep,key=rep.get)))
B = bottle_pts(T7,[-1,0,0]); print('bottle at release x %.3f..%.3f y %.3f..%.3f z %.3f..%.3f'%(B[:,0].min(),B[:,0].max(),B[:,1].min(),B[:,1].max(),B[:,2].min(),B[:,2].max()))
if '--go' in sys.argv:
    r.gripper(0.04)
    print(r.move_q(q1)); print(r.move_tcp(T2, n_wp=3)); print('tcp', r.tcp()[:3,3].round(4))
    f = r.gripper(0.0)
    if not (0.004 < f[0] < 0.012): print('GRASP LOOKS WRONG, stopping'); sys.exit(1)
    print(r.move_tcp(T3, n_wp=2)); print('fingers', r.fingers())
    print(r.move_q(q4)); print('fingers', r.fingers())
    print(r.move_traj_checked if False else '')
    # reorientation
    ts=[]; t=0; qp=q4
    for qi in qs5: t += max(np.abs(qi-qp).max()/0.15, 0.05)+0.2; ts.append(t); qp=qi
    print('reorient code', r.move_traj(qs5, ts)); print('q err', np.abs(r.q()-q5).max().round(4), 'fingers', r.fingers(), 'tcp', r.tcp()[:3,3].round(4))
    print(r.move_q(q6)); print('fingers', r.fingers())
    print(r.move_tcp(T7, n_wp=4)); print('tcp', r.tcp()[:3,3].round(4), 'fingers', r.fingers(), 'wrench', r.wrench().round(2))
    r.gripper(0.04)
    print(r.move_tcp(T6, n_wp=3)); print('tcp', r.tcp()[:3,3].round(4))
EOF
timeout 300 python3 step8.py

# openrua op 66
sed -i 's/T4 = make_T(\[-0.12, 0.0, 1.25\], Rt)/T4 = make_T([-0.10, 0.06, 1.30], Rt)/; s/T5 = make_T(\[-0.12, 0.0, 1.25\], Rp)/T5 = make_T([-0.10, 0.06, 1.30], Rp)/' step8.py && timeout 300 python3 step8.py 2>&1 | grep -E "^q[4-7]|paths|shelf|cabinet|table"

# openrua op 67
timeout 1200 python3 step8.py --go 2>&1 | grep -vE "^  (shelf|cabinet|bowl|table)|^descent|^q[1-7] "

# openrua op 68
timeout 600 python3 park.py -0.30,-0.05,1.35 2>&1 | tail -1 && python3 grab.py birdview agentview >/dev/null && python3 - <<'EOF'
import numpy as np
from cloud import cloud
pw,_ = cloud('birdview')
X,Y,Z = pw[...,0],pw[...,1],pw[...,2]
m = np.isfinite(Z)&(X>-0.25)&(X<0.25)&(Y>-0.1)&(Y<0.45)
for zlo,zhi in [(0.915,0.93),(0.93,0.96),(0.96,1.0),(1.0,1.02),(1.02,1.05),(1.05,1.09),(1.09,1.2)]:
    s = m&(Z>=zlo)&(Z<zhi)&(Y<0.228)
    if s.sum()>3: print(f"z {zlo}-{zhi} (y<0.228): n={s.sum()} x {X[s].min():.3f}..{X[s].max():.3f} y {Y[s].min():.3f}..{Y[s].max():.3f}")
EOF

# openrua op 69
python3 - <<'EOF'
import numpy as np
from cloud import cloud
pw,_ = cloud('birdview')
X,Y,Z = pw[...,0],pw[...,1],pw[...,2]
m = np.isfinite(Z)&(X>-0.089)&(X<0.109)&(Y>0.086)&(Y<0.228)&(Z>0.93)
print('interior pts above floor+7mm: n', m.sum(), 'x %.3f..%.3f y %.3f..%.3f z %.3f..%.3f'%(X[m].min(),X[m].max(),Y[m].min(),Y[m].max(),Z[m].min(),Z[m].max()))
# grid summary
for ylo in np.arange(0.086,0.23,0.02):
    s=m&(Y>=ylo)&(Y<ylo+0.02)
    if s.sum(): print(f" y {ylo:.3f}: x {X[s].min():.3f}..{X[s].max():.3f} zmax {Z[s].max():.3f} n {s.sum()}")
# anything above 1.0 near drawer/handle front (bottle sticking out?)
s = np.isfinite(Z)&(X>-0.12)&(X<0.13)&(Y>0.03)&(Y<0.19)&(Z>1.0)
print('pts above 1.0 over drawer front region:', s.sum(), (X[s].min(),X[s].max(),Y[s].min(),Y[s].max(),Z[s].max()) if s.sum() else '')
# panel-top / handle region: anything unusual (bottle neck resting on panel?)
s = np.isfinite(Z)&(X>-0.12)&(X<0.13)&(Y>0.03)&(Y<0.086)&(Z>0.96)
print('pts above 0.96 in front/panel zone y<0.086:', s.sum(), 'x %.3f..%.3f y %.3f..%.3f zmax %.3f'%(X[s].min(),X[s].max(),Y[s].min(),Y[s].max(),Z[s].max()))
EOF

# openrua op 70
cat > step9.py <<'EOF'
import numpy as np, math, sys
from panda import *
from collide import check, check_path
np.set_printoptions(precision=4, suppress=True)
r = Robot()
q0 = r.q(); print('q0', q0.round(3), 'fingers', r.fingers())
R = topdown(math.pi/2)
P = [([-0.04, 0.05, 1.15], 1), ([-0.04, 0.05, 1.005], 3), ([0.03, 0.05, 1.005], 4), ([0.03, 0.05, 1.017], 1),
     ([0.06, 0.08, 1.017], 3), ([0.085, 0.105, 1.017], 3), ([0.085, 0.105, 1.20], 2)]
ign = ('bottle','drawer','tophandle')
qs = []; qp = q0
for p, n in P:
    q = ik(make_T(p, R), qp); assert q is not None, p
    print(p, q.round(3), check(q, ignore=ign)); qs.append(q); qp = q
print('path', check_path([q0]+qs, ignore=ign))
if '--go' in sys.argv:
    r.gripper(0.0)
    for (p, n), q in zip(P, qs):
        print(p, r.move_tcp(make_T(p, R), n_wp=n), 'tcp', r.tcp()[:3,3].round(3), 'wrench', r.wrench().round(1))
EOF
timeout 900 python3 step9.py --go && timeout 600 python3 park.py -0.30,-0.05,1.35 | tail -1 && python3 grab.py birdview agentview >/dev/null && python3 - <<'EOF'
import numpy as np
from cloud import cloud
pw,_ = cloud('birdview')
X,Y,Z = pw[...,0],pw[...,1],pw[...,2]
m = np.isfinite(Z)&(X>-0.089)&(X<0.109)&(Y>0.086)&(Y<0.228)&(Z>0.93)
print('interior pts: n', m.sum(), 'x %.3f..%.3f y %.3f..%.3f zmax %.3f'%(X[m].min(),X[m].max(),Y[m].min(),Y[m].max(),Z[m].max()))
for ylo in np.arange(0.086,0.23,0.02):
    s=m&(Y>=ylo)&(Y<ylo+0.02)&(Z<1.0)
    if s.sum(): print(f" y {ylo:.3f}: x {X[s].min():.3f}..{X[s].max():.3f} zmax {Z[s].max():.3f} n {s.sum()}")
s = np.isfinite(Z)&(X>-0.12)&(X<0.13)&(Y>0.0)&(Y<0.086)&(Z>0.965)
print('pts above 0.965 in front zone y<0.086:', s.sum(), 'x %.3f..%.3f y %.3f..%.3f zmax %.3f'%(X[s].min(),X[s].max(),Y[s].min(),Y[s].max(),Z[s].max()) if s.sum() else '')
s = np.isfinite(Z)&(X>-0.12)&(X<0.13)&(Y>0.0)&(Y<0.186)&(Z>1.0)
print('pts above 1.0 over drawer (y<0.186):', s.sum())
EOF

# openrua op 71
python3 - <<'EOF'
import numpy as np, cv2
from cloud import cloud
pw,col = cloud('birdview')
X,Y,Z = pw[...,0],pw[...,1],pw[...,2]
s = np.isfinite(Z)&(X>-0.12)&(X<0.13)&(Y>0.0)&(Y<0.186)&(Z>1.0)
print('above 1.0:', 'x %.3f..%.3f y %.3f..%.3f z %.3f..%.3f'%(X[s].min(),X[s].max(),Y[s].min(),Y[s].max(),Z[s].min(),Z[s].max()))
# front zone detail by x
s = np.isfinite(Z)&(X>-0.12)&(X<0.13)&(Y>0.03)&(Y<0.086)&(Z>0.965)
for xlo in np.arange(-0.10,0.12,0.02):
    t=s&(X>=xlo)&(X<xlo+0.02)
    if t.sum(): print(f" x {xlo:.2f}: y {Y[t].min():.3f}..{Y[t].max():.3f} zmax {Z[t].max():.3f} n {t.sum()}")
# pixel crop of drawer region
vs,us = np.where(np.isfinite(Z)&(X>-0.15)&(X<0.15)&(Y>-0.02)&(Y<0.25))
crop = col[vs.min():vs.max(), us.min():us.max()]
cv2.imwrite('bird_crop.png', cv2.resize(crop, None, fx=4, fy=4, interpolation=cv2.INTER_NEAREST))
a = cv2.imread('agentview.png'); h,w = a.shape[:2]
cv2.imwrite('agent_crop.png', cv2.resize(a[h//4:3*h//4, w//4:3*w//4], None, fx=2, fy=2))
EOF

# openrua op 72
python3 - <<'EOF'
import numpy as np
from cloud import cloud
pw,col = cloud('birdview')
X,Y,Z = pw[...,0],pw[...,1],pw[...,2]
for name,m in [('wall-x',(X>-0.115)&(X<-0.088)&(Y>0.09)&(Y<0.2)),('wall+x',(X>0.108)&(X<0.14)&(Y>0.09)&(Y<0.2)),('panel',(X>0.05)&(X<0.1)&(Y>0.065)&(Y<0.09)),('handlebar',(X>-0.02)&(X<0.02)&(Y>0.035)&(Y<0.058)),('table',(X>-0.3)&(X<-0.2)&(Y>0.0)&(Y<0.1))]:
    s=np.isfinite(Z)&m&(Z>0.9)
    print(name, 'z %.3f..%.3f  n %d'%(Z[s].min(),Z[s].max(),s.sum()), 'pct', np.percentile(Z[s],[50,90,99]).round(3))
EOF

# openrua op 73
cat > step10.py <<'EOF'
import numpy as np, math, sys
from panda import *
from collide import check, check_path
np.set_printoptions(precision=4, suppress=True)
r = Robot()
q0 = r.q(); print('q0', q0.round(3), 'fingers', r.fingers())
R = topdown(0.0)   # finger axis along world y
zb = 1.0015
P = [([-0.01, 0.055, 1.15], 1), ([-0.01, 0.055, zb], 3), ([0.05, 0.07, zb], 4), ([0.08, 0.08, zb], 3), ([0.08, 0.08, 1.20], 2)]
ign = ('bottle','drawer')
qs = []; qp = q0
for p, n in P:
    q = ik(make_T(p, R), qp); assert q is not None, p
    print(p, q.round(3), check(q, ignore=ign)); qs.append(q); qp = q
print('path', check_path([q0]+qs, ignore=ign))
if '--go' in sys.argv:
    r.gripper(0.0)
    for (p, n), q in zip(P, qs):
        print(p, r.move_tcp(make_T(p, R), n_wp=n), 'tcp', r.tcp()[:3,3].round(3), 'wrench', r.wrench().round(1))
EOF
timeout 900 python3 step10.py --go && timeout 600 python3 park.py -0.30,-0.05,1.35 | tail -1 && python3 grab.py birdview agentview >/dev/null && python3 - <<'EOF'
import numpy as np, cv2
from cloud import cloud
pw,col = cloud('birdview')
X,Y,Z = pw[...,0],pw[...,1],pw[...,2]
m = np.isfinite(Z)&(X>-0.089)&(X<0.109)&(Y>0.086)&(Y<0.228)&(Z>0.93)&(Z<0.999)
print('interior pts: n', m.sum(), 'x %.3f..%.3f y %.3f..%.3f zmax %.3f'%(X[m].min(),X[m].max(),Y[m].min(),Y[m].max(),Z[m].max()))
s = np.isfinite(Z)&(X>-0.12)&(X<0.13)&(Y>0.03)&(Y<0.086)&(Z>0.965)
print('front zone y<0.086 z>0.965: n', s.sum(), 'zmax %.3f'%Z[s].max(), 'x of pts>0.99:', X[s&(Z>0.99)].round(3) if (s&(Z>0.99)).sum() else 'none')
vs,us = np.where(np.isfinite(Z)&(X>-0.15)&(X<0.15)&(Y>-0.02)&(Y<0.25))
cv2.imwrite('bird_crop.png', cv2.resize(col[vs.min():vs.max(), us.min():us.max()], None, fx=4, fy=4, interpolation=cv2.INTER_NEAREST))
EOF

# openrua op 74
cat > step11.py <<'EOF'
import numpy as np, math, sys
from panda import *
from collide import check, check_path
np.set_printoptions(precision=4, suppress=True)
r = Robot()
q0 = r.q(); print('q0', q0.round(3), 'fingers', r.fingers())
R = topdown(math.pi/2)   # finger axis along world x -> wide face pushes +y
P = [([0.04, 0.045, 1.15], 1), ([0.04, 0.045, 0.9766], 3), ([0.04, 0.06, 0.9766], 2), ([0.04, 0.06, 1.0015], 1),
     ([0.04, 0.10, 1.0015], 3), ([0.04, 0.10, 1.20], 2)]
ign = ('bottle','drawer')
qs = []; qp = q0
for p, n in P:
    q = ik(make_T(p, R), qp); assert q is not None, p
    print(p, q.round(3), check(q, ignore=ign)); qs.append(q); qp = q
print('path', check_path([q0]+qs, ignore=ign))
if '--go' in sys.argv:
    r.gripper(0.0)
    for (p, n), q in zip(P, qs):
        print(p, r.move_tcp(make_T(p, R), n_wp=n), 'tcp', r.tcp()[:3,3].round(3), 'wrench', r.wrench().round(1))
EOF
timeout 900 python3 step11.py --go && timeout 600 python3 park.py -0.30,-0.05,1.35 | tail -1 && python3 grab.py birdview agentview >/dev/null && python3 - <<'EOF'
import numpy as np, cv2
from cloud import cloud
pw,col = cloud('birdview')
X,Y,Z = pw[...,0],pw[...,1],pw[...,2]
m = np.isfinite(Z)&(X>-0.089)&(X<0.109)&(Y>0.086)&(Y<0.228)&(Z>0.93)&(Z<0.999)
print('interior pts: n', m.sum(), 'x %.3f..%.3f y %.3f..%.3f zmax %.3f'%(X[m].min(),X[m].max(),Y[m].min(),Y[m].max(),Z[m].max()))
s = np.isfinite(Z)&(X>-0.12)&(X<0.13)&(Y>0.03)&(Y<0.086)&(Z>0.965)
print('front zone y<0.086 z>0.965: n', s.sum(), 'zmax %.3f'%Z[s].max(), 'x of pts>0.99:', X[s&(Z>0.99)].round(3) if (s&(Z>0.99)).sum() else 'none')
vs,us = np.where(np.isfinite(Z)&(X>-0.15)&(X<0.15)&(Y>-0.02)&(Y<0.25))
cv2.imwrite('bird_crop.png', cv2.resize(col[vs.min():vs.max(), us.min():us.max()], None, fx=4, fy=4, interpolation=cv2.INTER_NEAREST))
EOF

# openrua op 75
python3 - <<'EOF'
import numpy as np, cv2
from cloud import cloud
pw,col = cloud('birdview')
X,Y,Z = pw[...,0],pw[...,1],pw[...,2]
# dark pixels (bottle is dark green) within the drawer vicinity
b,g,r = col[...,0].astype(int),col[...,1].astype(int),col[...,2].astype(int)
dark = (r<70)&(g<90)&(b<70)&np.isfinite(Z)&(X>-0.15)&(X<0.15)&(Y>0.0)&(Y<0.25)&(Z>0.93)
print('dark pts n', dark.sum(), 'x %.3f..%.3f y %.3f..%.3f z %.3f..%.3f'%(X[dark].min(),X[dark].max(),Y[dark].min(),Y[dark].max(),Z[dark].min(),Z[dark].max()))
for ylo in np.arange(0.04,0.20,0.01):
    s=dark&(Y>=ylo)&(Y<ylo+0.01)
    if s.sum(): print(f" y {ylo:.2f}: x {X[s].min():.3f}..{X[s].max():.3f} z {Z[s].min():.3f}..{Z[s].max():.3f} n {s.sum()}")
# orange cap
cap = (r>150)&(g>80)&(g<180)&(b<100)&np.isfinite(Z)&(X>-0.15)&(X<0.15)&(Y>0.0)&(Y<0.25)&(Z>0.93)
print('cap pts n', cap.sum(), 'x %.3f..%.3f y %.3f..%.3f z %.3f..%.3f'%(X[cap].min(),X[cap].max(),Y[cap].min(),Y[cap].max(),Z[cap].min(),Z[cap].max()) if cap.sum() else '')
a = cv2.imread('agentview.png'); h,w = a.shape[:2]
cv2.imwrite('agent_crop.png', cv2.resize(a[h//3:5*h//6, w//4:3*w//4], None, fx=2, fy=2))
EOF

# openrua op 76
cat > step12.py <<'EOF'
import numpy as np, math, sys
from panda import *
from collide import check, check_path
np.set_printoptions(precision=4, suppress=True)
r = Robot()
q0 = r.q(); print('q0', q0.round(3), 'fingers', r.fingers())
R = topdown(math.pi/2)   # finger axis along world x
P = [([0.04, 0.058, 1.15], 1, None), ([0.04, 0.058, 0.992], 3, 'close'), ([0.04, 0.058, 1.02], 1, None),
     ([0.04, 0.10, 1.02], 2, None), ([0.04, 0.10, 0.995], 1, 'open'), ([0.04, 0.10, 1.20], 2, None)]
ign = ('bottle','drawer')
qs = []; qp = q0
for p, n, a in P:
    q = ik(make_T(p, R), qp); assert q is not None, p
    print(p, q.round(3), check(q, ignore=ign)); qs.append(q); qp = q
print('path', check_path([q0]+qs, ignore=ign))
if '--go' in sys.argv:
    r.gripper(0.08)
    for (p, n, a), q in zip(P, qs):
        print(p, r.move_tcp(make_T(p, R), n_wp=n), 'tcp', r.tcp()[:3,3].round(3), 'wrench', r.wrench().round(1))
        if a == 'close': r.gripper(0.0)
        if a == 'open': r.gripper(0.08)
EOF
timeout 900 python3 step12.py --go && timeout 600 python3 park.py -0.30,-0.05,1.35 | tail -1 && python3 grab.py birdview agentview >/dev/null && python3 - <<'EOF'
import numpy as np, cv2
from cloud import cloud
pw,col = cloud('birdview')
X,Y,Z = pw[...,0],pw[...,1],pw[...,2]
m = np.isfinite(Z)&(X>-0.089)&(X<0.109)&(Y>0.086)&(Y<0.228)&(Z>0.93)&(Z<0.999)
print('interior pts: n', m.sum(), 'x %.3f..%.3f y %.3f..%.3f zmax %.3f'%(X[m].min(),X[m].max(),Y[m].min(),Y[m].max(),Z[m].max()))
s = np.isfinite(Z)&(X>-0.12)&(X<0.13)&(Y>0.03)&(Y<0.086)&(Z>0.965)
print('front zone y<0.086 z>0.965: n', s.sum(), 'zmax %.3f'%Z[s].max(), 'pts>0.99:', (s&(Z>0.99)).sum())
b,g,r = col[...,0].astype(int),col[...,1].astype(int),col[...,2].astype(int)
cap = (r>150)&(g>80)&(g<180)&(b<100)&np.isfinite(Z)&(X>-0.15)&(X<0.15)&(Y>0.0)&(Y<0.25)&(Z>0.93)
print('cap pts n', cap.sum(), 'x %.3f..%.3f y %.3f..%.3f z %.3f..%.3f'%(X[cap].min(),X[cap].max(),Y[cap].min(),Y[cap].max(),Z[cap].min(),Z[cap].max()) if cap.sum() else '')
vs,us = np.where(np.isfinite(Z)&(X>-0.15)&(X<0.15)&(Y>-0.02)&(Y<0.25))
cv2.imwrite('bird_crop.png', cv2.resize(col[vs.min():vs.max(), us.min():us.max()], None, fx=4, fy=4, interpolation=cv2.INTER_NEAREST))
EOF

# openrua op 77
python3 - <<'EOF'
import numpy as np
from cloud import cloud
pw,col = cloud('birdview')
X,Y,Z = pw[...,0],pw[...,1],pw[...,2]
s = np.isfinite(Z)&(X>0.0)&(X<0.08)&(Y>0.03)&(Y<0.13)&(Z>0.985)
idx = np.argsort(Y[s])
for x,y,z,c in zip(X[s][idx],Y[s][idx],Z[s][idx],col[s][idx]): print(f"{x:.3f} {y:.3f} {z:.3f} {c}")
d = np.load('birdview.npz'); print('K', d['K'][0,0], 'depth px size at 2.05m ~', 2.05/d['K'][0,0])
EOF

# openrua op 78
cat > step13.py <<'EOF'
import numpy as np, math, sys
from panda import *
from collide import check, check_path
np.set_printoptions(precision=4, suppress=True)
r = Robot()
q0 = r.q(); print('q0', q0.round(3), 'fingers', r.fingers())
R = topdown(math.pi/2)   # finger axis along world x
P = [([0.042, 0.084, 1.15], 1, None), ([0.042, 0.084, 0.996], 3, 'close'), ([0.042, 0.084, 1.03], 1, None),
     ([0.03, 0.12, 1.03], 2, None), ([0.03, 0.12, 0.996], 1, 'open'), ([0.03, 0.12, 1.20], 2, None)]
ign = ('bottle','drawer')
qs = []; qp = q0
for p, n, a in P:
    q = ik(make_T(p, R), qp); assert q is not None, p
    print(p, q.round(3), check(q, ignore=ign)); qs.append(q); qp = q
print('path', check_path([q0]+qs, ignore=ign))
if '--go' in sys.argv:
    r.gripper(0.08)
    for (p, n, a), q in zip(P, qs):
        print(p, r.move_tcp(make_T(p, R), n_wp=n), 'tcp', r.tcp()[:3,3].round(3), 'wrench', r.wrench().round(1))
        if a == 'close':
            f = r.gripper(0.0)
            if f[0] < 0.003:
                print('GRASP MISSED - aborting sequence'); r.gripper(0.08); break
        if a == 'open': r.gripper(0.08)
EOF
timeout 900 python3 step13.py --go && timeout 600 python3 park.py -0.30,-0.05,1.35 | tail -1 && python3 grab.py birdview agentview >/dev/null && python3 - <<'EOF'
import numpy as np, cv2
from cloud import cloud
pw,col = cloud('birdview')
X,Y,Z = pw[...,0],pw[...,1],pw[...,2]
m = np.isfinite(Z)&(X>-0.089)&(X<0.109)&(Y>0.086)&(Y<0.228)&(Z>0.93)&(Z<0.999)
print('interior pts: n', m.sum(), 'x %.3f..%.3f y %.3f..%.3f zmax %.3f'%(X[m].min(),X[m].max(),Y[m].min(),Y[m].max(),Z[m].max()))
s = np.isfinite(Z)&(X>-0.12)&(X<0.13)&(Y>0.03)&(Y<0.086)&(Z>0.965)
print('front zone y<0.086 z>0.965: n', s.sum(), 'zmax %.3f'%Z[s].max(), 'pts>0.99:', (s&(Z>0.99)).sum())
b,g,r = col[...,0].astype(int),col[...,1].astype(int),col[...,2].astype(int)
cap = (r>150)&(g>80)&(g<180)&(b<100)&np.isfinite(Z)&(X>-0.15)&(X<0.15)&(Y>0.0)&(Y<0.25)&(Z>0.93)
print('cap pts n', cap.sum(), 'x %.3f..%.3f y %.3f..%.3f z %.3f..%.3f'%(X[cap].min(),X[cap].max(),Y[cap].min(),Y[cap].max(),Z[cap].min(),Z[cap].max()) if cap.sum() else '')
vs,us = np.where(np.isfinite(Z)&(X>-0.15)&(X<0.15)&(Y>-0.02)&(Y<0.25))
cv2.imwrite('bird_crop.png', cv2.resize(col[vs.min():vs.max(), us.min():us.max()], None, fx=4, fy=4, interpolation=cv2.INTER_NEAREST))
EOF

# openrua op 79
sed -e 's/\[0.042, 0.084, 1.15\]/[0.056, 0.086, 1.15]/; s/\[0.042, 0.084, 0.996\]/[0.056, 0.086, 1.0]/; s/\[0.042, 0.084, 1.03\]/[0.056, 0.086, 1.03]/; s/\[0.03, 0.12, 1.03\]/[0.08, 0.15, 1.03]/; s/\[0.03, 0.12, 0.996\]/[0.08, 0.15, 1.0]/; s/\[0.03, 0.12, 1.20\]/[0.08, 0.15, 1.20]/' step13.py > step14.py
timeout 900 python3 step14.py --go && timeout 600 python3 park.py -0.30,-0.05,1.35 | tail -1 && python3 grab.py birdview agentview >/dev/null && python3 - <<'EOF'
import numpy as np, cv2
from cloud import cloud
pw,col = cloud('birdview')
X,Y,Z = pw[...,0],pw[...,1],pw[...,2]
m = np.isfinite(Z)&(X>-0.089)&(X<0.109)&(Y>0.086)&(Y<0.228)&(Z>0.93)&(Z<0.999)
print('interior pts: n', m.sum(), 'x %.3f..%.3f y %.3f..%.3f zmax %.3f'%(X[m].min(),X[m].max(),Y[m].min(),Y[m].max(),Z[m].max()))
s = np.isfinite(Z)&(X>-0.12)&(X<0.13)&(Y>0.03)&(Y<0.086)&(Z>0.965)
print('front zone y<0.086 z>0.965: n', s.sum(), 'zmax %.3f'%Z[s].max(), 'pts>0.99:', (s&(Z>0.99)).sum())
b,g,r = col[...,0].astype(int),col[...,1].astype(int),col[...,2].astype(int)
cap = (r>150)&(g>80)&(g<180)&(b<100)&np.isfinite(Z)&(X>-0.15)&(X<0.15)&(Y>0.0)&(Y<0.25)&(Z>0.93)
print('cap pts n', cap.sum(), 'x %.3f..%.3f y %.3f..%.3f z %.3f..%.3f'%(X[cap].min(),X[cap].max(),Y[cap].min(),Y[cap].max(),Z[cap].min(),Z[cap].max()) if cap.sum() else '')
vs,us = np.where(np.isfinite(Z)&(X>-0.15)&(X<0.15)&(Y>-0.02)&(Y<0.25))
cv2.imwrite('bird_crop.png', cv2.resize(col[vs.min():vs.max(), us.min():us.max()], None, fx=4, fy=4, interpolation=cv2.INTER_NEAREST))
EOF

# openrua op 80
cat > step15.py <<'EOF'
import numpy as np, math, sys, subprocess
from panda import *
from collide import check, check_path
np.set_printoptions(precision=4, suppress=True)
r = Robot()
R = topdown(math.pi/2)
def go(p, n=1):
    q = ik(make_T(p, R), r.q()); c = check(q, ignore=('bottle','drawer'))
    print('->', p, 'clear', c); assert c[0] > -0.005
    print(r.move_tcp(make_T(p, R), n_wp=n), 'tcp', r.tcp()[:3,3].round(3), 'fingers', np.round(r.fingers(),4), 'wrench', r.wrench().round(1), flush=True)
def snap(tag):
    subprocess.run(['python3','grab.py','birdview','agentview','sideview','frontview'], check=True, stdout=subprocess.DEVNULL)
    import shutil
    for c in ['birdview','agentview','sideview','frontview']:
        shutil.copy(f'{c}.png', f'{tag}_{c}.png'); shutil.copy(f'{c}.npz', f'{tag}_{c}.npz')
r.gripper(0.08)
go([0.056, 0.086, 1.15]); go([0.056, 0.086, 1.0], 3)
r.gripper(0.0)
go([0.056, 0.086, 1.05]); snap('hold1')
go([0.08, 0.14, 1.05], 3); snap('hold2')
print('fingers after move', np.round(r.fingers(),4))
EOF
timeout 900 python3 step15.py && python3 - <<'EOF'
import numpy as np, cv2
from cloud import cloud
for tag in ['hold1','hold2']:
    pw,col = cloud(f'{tag}_birdview')
    X,Y,Z = pw[...,0],pw[...,1],pw[...,2]
    b,g,r = col[...,0].astype(int),col[...,1].astype(int),col[...,2].astype(int)
    cap = (r>150)&(g>80)&(g<180)&(b<100)&np.isfinite(Z)&(X>-0.15)&(X<0.15)&(Y>0.0)&(Y<0.25)&(Z>0.93)
    print(tag, 'cap pts n', cap.sum(), 'x %.3f..%.3f y %.3f..%.3f z %.3f..%.3f'%(X[cap].min(),X[cap].max(),Y[cap].min(),Y[cap].max(),Z[cap].min(),Z[cap].max()) if cap.sum() else '')
    vs,us = np.where(np.isfinite(Z)&(X>-0.15)&(X<0.15)&(Y>-0.02)&(Y<0.25))
    cv2.imwrite(f'{tag}_crop.png', cv2.resize(col[vs.min():vs.max(), us.min():us.max()], None, fx=4, fy=4, interpolation=cv2.INTER_NEAREST))
EOF

# openrua op 81
cat > step16.py <<'EOF'
import numpy as np, math, sys, subprocess, shutil
from panda import *
from collide import check
np.set_printoptions(precision=4, suppress=True)
r = Robot()
R = topdown(math.pi/2)
def go(p, n=1):
    q = ik(make_T(p, R), r.q()); c = check(q, ignore=('bottle','drawer'))
    print('->', p, 'clear', c); assert c[0] > -0.005
    print(r.move_tcp(make_T(p, R), n_wp=n), 'tcp', r.tcp()[:3,3].round(3), 'fingers', np.round(r.fingers(),4), 'wrench', r.wrench().round(1), flush=True)
def snap(tag):
    subprocess.run(['python3','grab.py','birdview','agentview'], check=True, stdout=subprocess.DEVNULL)
    for c in ['birdview','agentview']:
        shutil.copy(f'{c}.png', f'{tag}_{c}.png'); shutil.copy(f'{c}.npz', f'{tag}_{c}.npz')
print('start tcp', r.tcp()[:3,3].round(3), 'fingers', r.fingers())
r.gripper(0.08)
go([0.056, 0.086, 1.15]); go([0.056, 0.086, 1.0], 3)
r.gripper(0.0)
go([0.065, 0.10, 1.0], 2); go([0.075, 0.125, 1.0], 2); go([0.085, 0.145, 1.0], 2)
snap('drag')
r.gripper(0.08)
go([0.085, 0.145, 1.2], 2)
EOF
timeout 900 python3 step16.py && timeout 600 python3 park.py -0.30,-0.05,1.35 | tail -1 && python3 grab.py birdview agentview >/dev/null && python3 - <<'EOF'
import numpy as np, cv2
from cloud import cloud
for tag in ['drag_birdview','birdview']:
    pw,col = cloud(tag)
    X,Y,Z = pw[...,0],pw[...,1],pw[...,2]
    m = np.isfinite(Z)&(X>-0.089)&(X<0.109)&(Y>0.086)&(Y<0.228)&(Z>0.93)&(Z<0.999)
    print(tag, 'interior pts: n', m.sum(), 'x %.3f..%.3f y %.3f..%.3f zmax %.3f'%(X[m].min(),X[m].max(),Y[m].min(),Y[m].max(),Z[m].max()))
    s = np.isfinite(Z)&(X>-0.12)&(X<0.13)&(Y>0.03)&(Y<0.086)&(Z>0.965)
    print('  front zone y<0.086 z>0.965: n', s.sum(), 'zmax %.3f'%Z[s].max(), 'pts>0.99:', (s&(Z>0.99)).sum())
    b,g,r = col[...,0].astype(int),col[...,1].astype(int),col[...,2].astype(int)
    cap = (r>150)&(g>80)&(g<180)&(b<100)&np.isfinite(Z)&(X>-0.15)&(X<0.15)&(Y>0.0)&(Y<0.25)&(Z>0.93)&(Z<1.1)
    print('  cap pts n', cap.sum(), 'x %.3f..%.3f y %.3f..%.3f z %.3f..%.3f'%(X[cap].min(),X[cap].max(),Y[cap].min(),Y[cap].max(),Z[cap].min(),Z[cap].max()) if cap.sum() else '')
    vs,us = np.where(np.isfinite(Z)&(X>-0.15)&(X<0.15)&(Y>-0.02)&(Y<0.25))
    cv2.imwrite(f'{tag}_crop.png', cv2.resize(col[vs.min():vs.max(), us.min():us.max()], None, fx=4, fy=4, interpolation=cv2.INTER_NEAREST))
EOF

# openrua op 82
cat > step17.py <<'EOF'
import numpy as np, math, sys, subprocess, shutil
from panda import *
from collide import check
np.set_printoptions(precision=4, suppress=True)
r = Robot()
R = topdown(math.pi/2)
def go(p, n=1):
    q = ik(make_T(p, R), r.q()); c = check(q, ignore=('bottle','drawer'))
    print('->', p, 'clear', c); assert c[0] > -0.005
    print(r.move_tcp(make_T(p, R), n_wp=n), 'tcp', r.tcp()[:3,3].round(3), 'fingers', np.round(r.fingers(),4), 'wrench', r.wrench().round(1), flush=True)
print('start tcp', r.tcp()[:3,3].round(3), 'fingers', r.fingers())
r.gripper(0.08)
go([0.028, 0.098, 1.15]); go([0.028, 0.098, 0.985], 3)
r.gripper(0.0)
go([0.045, 0.11, 0.985], 2); go([0.06, 0.125, 0.985], 2); go([0.075, 0.14, 0.985], 2)
r.gripper(0.08)
go([0.075, 0.14, 1.2], 2)
EOF
timeout 900 python3 step17.py; timeout 600 python3 park.py -0.30,-0.05,1.35 | tail -1 && python3 grab.py birdview agentview >/dev/null && python3 - <<'EOF'
import numpy as np, cv2
from cloud import cloud
pw,col = cloud('birdview')
X,Y,Z = pw[...,0],pw[...,1],pw[...,2]
m = np.isfinite(Z)&(X>-0.089)&(X<0.109)&(Y>0.086)&(Y<0.228)&(Z>0.93)&(Z<0.999)
print('interior pts: n', m.sum(), 'x %.3f..%.3f y %.3f..%.3f zmax %.3f'%(X[m].min(),X[m].max(),Y[m].min(),Y[m].max(),Z[m].max()))
s = np.isfinite(Z)&(X>-0.12)&(X<0.13)&(Y>0.03)&(Y<0.086)&(Z>0.965)
print('front zone y<0.086 z>0.965: n', s.sum(), 'zmax %.3f'%Z[s].max(), 'pts>0.99:', (s&(Z>0.99)).sum())
b,g,r = col[...,0].astype(int),col[...,1].astype(int),col[...,2].astype(int)
cap = (r>150)&(g>80)&(g<180)&(b<100)&np.isfinite(Z)&(X>-0.15)&(X<0.15)&(Y>0.0)&(Y<0.25)&(Z>0.93)&(Z<1.1)
print('cap pts n', cap.sum(), 'x %.3f..%.3f y %.3f..%.3f z %.3f..%.3f'%(X[cap].min(),X[cap].max(),Y[cap].min(),Y[cap].max(),Z[cap].min(),Z[cap].max()) if cap.sum() else '')
vs,us = np.where(np.isfinite(Z)&(X>-0.15)&(X<0.15)&(Y>-0.02)&(Y<0.25))
cv2.imwrite('bird_crop.png', cv2.resize(col[vs.min():vs.max(), us.min():us.max()], None, fx=4, fy=4, interpolation=cv2.INTER_NEAREST))
EOF

# openrua op 83
cat > step18.py <<'EOF'
import numpy as np, math, sys
from panda import *
from collide import check, check_path
np.set_printoptions(precision=4, suppress=True)
r = Robot()
th = math.radians(40)
a = [0, math.cos(th), -math.sin(th)]       # approach: forward (+y) and down
R = R_from_axes(a, y=[1, 0, 0])            # finger axis along world x
z = 0.95
ign = ('drawer',)
def go(p, n=1):
    q = ik(make_T(p, R), r.q()); c = check(q, ignore=ign)
    print('->', p, 'clear', c, flush=True); assert c[0] > -0.003
    res = r.move_tcp(make_T(p, R), n_wp=n)
    print('  ', res, 'tcp', r.tcp()[:3,3].round(3), 'wrench', r.wrench().round(1), flush=True)
    return res
if '--go' not in sys.argv:
    qp = r.q()
    for p in ([0.0, 0.02, 1.15], [0.0, 0.02, z], [0.0, 0.10, z], [0.0, 0.20, z]):
        q = ik(make_T(p, R), qp); assert q is not None, p
        print(p, q.round(3), check(q, ignore=ign, verbose=True)); qp = q
    sys.exit()
r.gripper(0.0)
go([0.0, 0.02, 1.15]); go([0.0, 0.02, z], 3)
y = 0.02
while y < 0.20:
    y = min(y + 0.03, 0.20)
    res = go([0.0, y, z], 2)
    w = r.wrench()
    if res is None or res[0] != 0 or abs(w[1]) > 15:
        print('stopped at y', y, 'code', res, 'wrench', w.round(1)); break
print('final tcp', r.tcp()[:3,3].round(3))
# retreat
go([0.0, r.tcp()[1,3] - 0.04, z], 2); go([0.0, r.tcp()[1,3], 1.2], 2)
EOF
timeout 300 python3 step18.py

# openrua op 84
cat > seed18.py <<'EOF'
import numpy as np, math
from panda import *
from collide import check, check_path
np.set_printoptions(precision=3, suppress=True)
th = math.radians(40)
R = R_from_axes([0, math.cos(th), -math.sin(th)], y=[1, 0, 0])
z = 0.95; ign = ('drawer','bottle')
P = [[0.0, 0.02, 1.15], [0.0, 0.02, z], [0.0, 0.10, z], [0.0, 0.20, z]]
rng = np.random.default_rng(0)
best = None
for t in range(300):
    q0 = rng.uniform(LIM[:,0]+0.1, LIM[:,1]-0.1)
    qs = []; qp = q0; ok = True
    for p in P:
        q = ik(make_T(p, R), qp)
        if q is None: ok = False; break
        qs.append(q); qp = q
    if not ok: continue
    c = check_path(qs, ignore=ign)
    if best is None or c[0] > best[0][0]:
        best = (c, qs); print(t, c, qs[0].round(2))
print('BEST', best[0]); 
for q in best[1]: print(q.round(3), check(q, ignore=ign))
np.save('seed18.npy', np.array(best[1]))
EOF
timeout 600 python3 seed18.py 2>&1 | tail -8

# openrua op 85
cat > seed18.py <<'EOF'
import numpy as np, math
from panda import *
from collide import check, check_path
np.set_printoptions(precision=3, suppress=True)
ign = ('drawer','bottle')
for deg in (50, 55, 60, 65):
  for x, z in ((0.0, 0.95), (0.09, 0.965)):
    th = math.radians(deg)
    R = R_from_axes([0, math.cos(th), -math.sin(th)], y=[1, 0, 0])
    P = [[x, 0.02, 1.15], [x, 0.02, z], [x, 0.10, z], [x, 0.20, z]]
    rng = np.random.default_rng(0); best = None
    for t in range(60):
        q0 = rng.uniform(LIM[:,0]+0.1, LIM[:,1]-0.1)
        qs = []; qp = q0; ok = True
        for p in P:
            q = ik(make_T(p, R), qp)
            if q is None: ok = False; break
            qs.append(q); qp = q
        if not ok: continue
        c = check_path(qs, ignore=ign)
        if best is None or c[0] > best[0][0]: best = (c, qs)
    print(deg, x, z, 'best', best[0] if best else None, [check(q, ignore=ign) for q in best[1]] if best else '')
    if best: np.save(f'seed18_{deg}_{x}.npy', np.array(best[1]))
EOF
timeout 900 python3 seed18.py 2>&1 | tail -8

# openrua op 86
python3 - <<'EOF'
import numpy as np, math
from panda import *
from collide import check, hand_points, frames
np.set_printoptions(precision=3, suppress=True)
qs = np.load('seed18_60_0.0.npy')
q = qs[3]
print(fk_tcp(q).round(3))
check(q, ignore=('drawer','bottle'), verbose=True)
H = hand_points(frames(q)[-1])
lo, hi = np.array([-0.05,0.185,1.01]), np.array([0.06,0.23,1.11])
d = np.linalg.norm(np.maximum(np.maximum(lo-H,0),H-hi),axis=1)
print('closest hand pts', H[np.argsort(d)[:5]].round(3))
EOF

# openrua op 87
python3 - <<'EOF'
import numpy as np, math
from panda import *
from collide import check, hand_points, frames
np.set_printoptions(precision=3, suppress=True)
qs = np.load('seed18_60_0.0.npy'); q = qs[3]
Th = frames(q)[-1]; print('hand frame\n', Th.round(3))
print('tcp from hand', (Th @ np.array([0,0,0.1034,1]))[:3].round(3))
H = hand_points(Th)
lo, hi = np.array([-0.05,0.185,1.01]), np.array([0.06,0.23,1.11])
d = np.linalg.norm(np.maximum(np.maximum(lo-H,0),H-hi),axis=1)
i = np.argsort(d)[:3]
pts=[]
for x in (-0.03, 0.03):
    for y in np.linspace(-0.10, 0.10, 9):
        for z in (-0.04, 0.0, 0.035, 0.07):
            pts.append([x, y, z])
for x in (-0.012, 0.012):
    for y in (-0.05, -0.025, 0, 0.025, 0.05):
        for z in (0.09, 0.115):
            pts.append([x, y, z])
pts=np.array(pts); print('local pts of closest', pts[i], 'world', H[i].round(3))
EOF

# openrua op 88
cat > seed18.py <<'EOF'
import numpy as np, math
from panda import *
from collide import check, check_path
np.set_printoptions(precision=3, suppress=True)
ign = ('drawer','bottle')
for deg in (42, 45, 48, 50, 52):
    x, z = 0.0, 0.95
    th = math.radians(deg)
    R = R_from_axes([0, math.cos(th), -math.sin(th)], y=[1, 0, 0])
    P = [[x, 0.025, 1.15], [x, 0.025, z], [x, 0.10, z], [x, 0.185, z]]
    rng = np.random.default_rng(0); best = None
    for t in range(60):
        q0 = rng.uniform(LIM[:,0]+0.1, LIM[:,1]-0.1)
        qs = []; qp = q0; ok = True
        for p in P:
            q = ik(make_T(p, R), qp)
            if q is None: ok = False; break
            qs.append(q); qp = q
        if not ok: continue
        c = check_path(qs, ignore=ign)
        if best is None or c[0] > best[0][0]: best = (c, qs)
    print(deg, 'best', best[0] if best else None, [tuple(np.round(check(q, ignore=ign)[0],3)) for q in best[1]] if best else '')
    if best: np.save(f'seed18_{deg}.npy', np.array(best[1]))
EOF
timeout 900 python3 seed18.py 2>&1 | tail -6

# openrua op 89
sed -i 's/\[tuple(np.round(check(q, ignore=ign)\[0\],3)) for q in best\[1\]\]/[(round(float(check(q, ignore=ign)[0]),3), check(q, ignore=ign)[1]) for q in best[1]]/' seed18.py && timeout 900 python3 seed18.py 2>&1 | tail -6

# openrua op 90
sed -i 's/for deg in (42, 45, 48, 50, 52):/for deg in (52, 55):/; s/x, z = 0.0, 0.95/x, z = 0.0, 0.959/' seed18.py && timeout 900 python3 seed18.py 2>&1 | tail -3

# openrua op 91
cat > step18.py <<'EOF'
import numpy as np, math, sys
from panda import *
from collide import check, check_path
np.set_printoptions(precision=4, suppress=True)
r = Robot()
th = math.radians(52)
R = R_from_axes([0, math.cos(th), -math.sin(th)], y=[1, 0, 0])
z = 0.959; ign = ('drawer','bottle')
seeds = np.load('seed18_52.npy')
def go(p, n=1, seed=None):
    q = ik(make_T(p, R), r.q() if seed is None else seed); c = check(q, ignore=ign)
    print('->', p, 'clear', c, flush=True); assert c[0] > -0.003
    res = r.move_q(q) if n <= 1 else r.move_tcp(make_T(p, R), n_wp=n, q0=r.q())
    print('  ', res, 'tcp', r.tcp()[:3,3].round(3), 'wrench', r.wrench().round(1), flush=True)
    return res
print('start', r.tcp()[:3,3].round(3), r.fingers())
r.gripper(0.0)
# go to first seed configuration via joint move (check path from current)
print('path to seed0', check_path([r.q(), seeds[0]], ignore=ign))
go([0.0, 0.025, 1.15], seed=seeds[0]); go([0.0, 0.025, z], 3)
y = 0.025
while y < 0.185:
    y = min(y + 0.03, 0.185)
    res = go([0.0, y, z], 2)
    w = r.wrench()
    if res is None or res[0] != 0 or abs(w[1]) > 15:
        print('stopped at y', y, 'code', res, 'wrench', w.round(1)); break
print('final tcp', r.tcp()[:3,3].round(3))
yc = r.tcp()[1,3]
go([0.0, yc - 0.04, z], 2); go([0.0, yc - 0.04, 1.2], 2)
EOF
timeout 900 python3 step18.py

# openrua op 92
timeout 600 python3 park.py -0.30,-0.05,1.35 | tail -1 && python3 grab.py birdview agentview frontview >/dev/null && python3 - <<'EOF'
import numpy as np, cv2
from cloud import cloud
pw,col = cloud('birdview')
X,Y,Z = pw[...,0],pw[...,1],pw[...,2]
# find drawer front panel: points z 0.97-0.99 in x -0.11..0.12, y 0..0.3
s = np.isfinite(Z)&(X>-0.11)&(X<0.12)&(Y>-0.05)&(Y<0.3)&(Z>0.965)&(Z<0.99)
print('panel-height pts: n', s.sum(), 'y %.3f..%.3f'%(Y[s].min(),Y[s].max()) if s.sum() else '')
for ylo in np.arange(0.0,0.3,0.02):
    t=np.isfinite(Z)&(X>-0.11)&(X<0.12)&(Y>=ylo)&(Y<ylo+0.02)&(Z>0.92)
    if t.sum(): print(f" y {ylo:.2f}: zmax {Z[t].max():.3f} z50 {np.median(Z[t]):.3f} n {t.sum()}")
# handle bar: pts z 0.94-0.965 in y<0.25
s = np.isfinite(Z)&(X>-0.05)&(X<0.06)&(Y>-0.05)&(Y<0.25)&(Z>0.935)&(Z<0.965)
print('bar-height pts: n', s.sum(), 'y %.3f..%.3f'%(Y[s].min(),Y[s].max()) if s.sum() else '')
# shelf check: anything on table near shelf?
s = np.isfinite(Z)&(X>-0.3)&(X<0.1)&(Y>-0.45)&(Y<-0.15)&(Z>0.9)
print('shelf region pts z range %.3f..%.3f x %.3f..%.3f y %.3f..%.3f'%(Z[s].min(),Z[s].max(),X[s].min(),X[s].max(),Y[s].min(),Y[s].max()))
vs,us = np.where(np.isfinite(Z)&(X>-0.2)&(X<0.2)&(Y>-0.1)&(Y<0.45))
cv2.imwrite('bird_crop.png', cv2.resize(col[vs.min():vs.max(), us.min():us.max()], None, fx=3, fy=3, interpolation=cv2.INTER_NEAREST))
EOF

# openrua op 93
python3 - <<'EOF'
import numpy as np
from cloud import cloud
pw,col = cloud('birdview')
X,Y,Z = pw[...,0],pw[...,1],pw[...,2]
for name,(xlo,xhi) in [('panel-only x-0.08..-0.06',(-0.08,-0.06)),('through handle x-0.01..0.01',(-0.01,0.01)),('x 0.07..0.09',(0.07,0.09))]:
    print(name)
    for ylo in np.arange(0.12,0.25,0.005):
        t=np.isfinite(Z)&(X>xlo)&(X<xhi)&(Y>=ylo)&(Y<ylo+0.005)&(Z>0.92)
        if t.sum(): print(f"  y {ylo:.3f}: z {Z[t].min():.3f}..{Z[t].max():.3f} n {t.sum()}")
EOF

# openrua op 94
python3 grab.py sideview frontview >/dev/null; python3 - <<'EOF'
import numpy as np, cv2
from cloud import cloud
for cam in ['sideview','frontview','agentview']:
    pw,col = cloud(cam)
    X,Y,Z = pw[...,0],pw[...,1],pw[...,2]
    # top handle region: x -0.05..0.06, y 0.15..0.225, z 0.99..1.12
    s = np.isfinite(Z)&(X>-0.06)&(X<0.07)&(Y>0.14)&(Y<0.226)&(Z>0.99)&(Z<1.12)
    print(cam, 'tophandle pts n', s.sum())
    if s.sum():
        for zlo in np.arange(0.99,1.12,0.01):
            t=s&(Z>=zlo)&(Z<zlo+0.01)
            if t.sum(): print(f"  z {zlo:.2f}: y {Y[t].min():.3f}..{Y[t].max():.3f} x {X[t].min():.3f}..{X[t].max():.3f} n {t.sum()}")
    d = np.load(f'{cam}.npz'); print('  cam T', d['T'][:3,3].round(3))
EOF

# openrua op 95
cat > plan19.py <<'EOF'
import numpy as np, math
from panda import *
from collide import check, check_path, OBST
np.set_printoptions(precision=3, suppress=True)
OBST['tophandle'] = ([-0.05, 0.19, 1.005], [0.06, 0.23, 1.11])
ign = ('drawer','bottle')
def extras(yp):  # bottom handle bar + posts for panel front at yp
    return {'bar': ([-0.045, yp-0.033, 0.938], [0.055, yp-0.009, 0.962]),
            'postL': ([-0.037, yp-0.02, 0.938], [-0.023, yp, 0.962]),
            'postR': ([0.028, yp-0.02, 0.938], [0.043, yp, 0.962]),
            'paneltop': ([-0.11, yp, 0.983], [0.12, yp+0.02, 0.99])}
x = -0.08
for deg in (70, 75, 80, 85):
  for zc in (0.945, 0.955, 0.965):
    th = math.radians(deg)
    a = np.array([0, math.sin(th), -math.cos(th)]); xh = np.array([0, math.cos(th), math.sin(th)])
    R = R_from_axes(a, y=[1, 0, 0])
    # contact bottom edge = TCP + 0.0116 a - 0.012 xh ; want edge at (x, yc, zc)
    off = 0.0116*a - 0.012*xh
    res = []
    rng = np.random.default_rng(1); best=None
    for t in range(40):
        q0 = rng.uniform(LIM[:,0]+0.1, LIM[:,1]-0.1); qs=[]; qp=q0; ok=True; cs=[]
        for yp in (0.165, 0.20, 0.228):
            p = np.array([x, yp, zc]) - off
            q = ik(make_T(p, R), qp)
            if q is None: ok=False; break
            qs.append(q); qp=q; cs.append(check(q, ignore=ign, extra=extras(yp)))
        if not ok: continue
        worst = min(c[0] for c in cs)
        if best is None or worst > best[0]: best=(worst, cs, qs)
    print(deg, zc, 'best worst %.3f'%best[0] if best else None, [(round(float(c[0]),3), c[1]) for c in best[1]] if best else '')
    if best and best[0] > 0.005: np.save(f'plan19_{deg}_{zc}.npy', np.array(best[2]))
EOF
timeout 1200 python3 plan19.py 2>&1 | tail -12

# openrua op 96
cat > plan19.py <<'EOF'
import numpy as np, math
import collide
from panda import *
from collide import check, OBST
np.set_printoptions(precision=3, suppress=True)
def hand_points_closed(Th):
    pts = []
    for x in (-0.03, 0.03):
        for y in np.linspace(-0.10, 0.10, 9):
            for z in (-0.04, 0.0, 0.035, 0.07):
                pts.append([x, y, z, 1])
    for x in (-0.012, 0.012):
        for y in (-0.01, 0.0, 0.01):
            for z in (0.075, 0.09, 0.105):   # finger body; tips (0.115) excluded -> they touch the panel
                pts.append([x, y, z, 1])
    return (Th @ np.array(pts).T).T[:, :3]
collide.hand_points = hand_points_closed
OBST['tophandle'] = ([-0.05, 0.19, 1.005], [0.06, 0.23, 1.11])
OBST['cabinet'] = ([-0.14, 0.24, 0.90], [0.145, 0.44, 1.14])       # body behind panel plane
OBST['lintel'] = ([-0.14, 0.228, 0.99], [0.145, 0.44, 1.14])       # frame above bottom drawer opening
ign = ('drawer','bottle')
def extras(yp):
    return {'bar': ([-0.045, yp-0.033, 0.938], [0.055, yp-0.009, 0.962]),
            'postL': ([-0.037, yp-0.02, 0.938], [-0.023, yp, 0.962]),
            'postR': ([0.028, yp-0.02, 0.938], [0.043, yp, 0.962]),
            'paneltop': ([-0.11, yp, 0.983], [0.12, yp+0.02, 0.99])}
x = -0.08
for deg in (70, 75, 80, 85):
  for zc in (0.945, 0.955):
    th = math.radians(deg)
    a = np.array([0, math.sin(th), -math.cos(th)]); xh = np.array([0, math.cos(th), math.sin(th)])
    R = R_from_axes(a, y=[1, 0, 0])
    off = 0.0116*a - 0.012*xh
    rng = np.random.default_rng(1); best=None
    for t in range(40):
        q0 = rng.uniform(LIM[:,0]+0.1, LIM[:,1]-0.1); qs=[]; qp=q0; ok=True; cs=[]
        for yp in (0.165, 0.20, 0.228):
            p = np.array([x, yp, zc]) - off
            q = ik(make_T(p, R), qp)
            if q is None: ok=False; break
            qs.append(q); qp=q; cs.append(check(q, ignore=ign, extra=extras(yp)))
        if not ok: continue
        worst = min(c[0] for c in cs)
        if best is None or worst > best[0]: best=(worst, cs, qs)
    print(deg, zc, 'best worst %.3f'%best[0] if best else None, [(round(float(c[0]),3), c[1]) for c in best[1]] if best else '')
    if best and best[0] > 0.003: np.save(f'plan19_{deg}_{zc}.npy', np.array(best[2]))
EOF
timeout 1200 python3 plan19.py 2>&1 | tail -8

# openrua op 97
cat > step19.py <<'EOF'
import numpy as np, math, sys
import collide
from panda import *
from collide import check, check_path, OBST
exec(open('plan19.py').read().split("x = -0.08")[0].split("np.set_printoptions")[1].split("\n",1)[1])  # reuse hand model + boxes + extras
np.set_printoptions(precision=4, suppress=True)
r = Robot()
deg, zc, x = 80, 0.945, -0.08
th = math.radians(deg)
a = np.array([0, math.sin(th), -math.cos(th)]); xh = np.array([0, math.cos(th), math.sin(th)])
R = R_from_axes(a, y=[1, 0, 0]); off = 0.0116*a - 0.012*xh
seeds = np.load(f'plan19_{deg}_{zc}.npy')
def tcp_for(yc, z): return np.array([x, yc, z]) - off
def go(p, n=1, yp=0.175, seed=None):
    q = ik(make_T(p, R), r.q() if seed is None else seed); assert q is not None
    c = check(q, ignore=ign, extra=extras(yp)); print('->', p.round(3), 'clear', c, flush=True); assert c[0] > 0.002
    if seed is not None:
        pc = check_path([r.q(), q], ignore=ign, extra=extras(yp)); print('   path', pc); assert pc[0] > -0.01
        res = r.move_q(q)
    else:
        res = r.move_tcp(make_T(p, R), n_wp=n)
    print('  ', res, 'tcp', r.tcp()[:3,3].round(3), 'wrench', r.wrench().round(1), flush=True)
    return res
print('start', r.tcp()[:3,3].round(3), r.fingers())
r.gripper(0.0)
go(tcp_for(0.15, 1.10), yp=0.175, seed=seeds[0])
go(tcp_for(0.15, zc), n=3, yp=0.175)
yc = 0.15; w0 = r.wrench()
while yc < 0.236:
    yc = min(yc + 0.015, 0.236)
    res = go(tcp_for(yc, zc), n=2, yp=max(0.175, yc))
    w = r.wrench()
    if res is None or res[0] != 0 or abs(w[1]-w0[1]) > 8:
        print('STOP at contact-edge y', yc, 'code', res, 'wrench', w.round(1)); break
print('final tcp', r.tcp()[:3,3].round(3))
go(tcp_for(yc - 0.03, zc), n=2, yp=0.228); go(tcp_for(yc - 0.03, 1.10), n=2, yp=0.228)
EOF
timeout 900 python3 step19.py

# openrua op 98
cat > via19.py <<'EOF'
import numpy as np, math
exec(open('plan19.py').read().split("x = -0.08")[0].split("np.set_printoptions")[1].split("\n",1)[1])
from panda import *
from collide import check, check_path
import panda
r = Robot(); q0 = r.q()
deg, zc, x = 80, 0.945, -0.08
th = math.radians(deg)
a = np.array([0, math.sin(th), -math.cos(th)]); xh = np.array([0, math.cos(th), math.sin(th)])
R = R_from_axes(a, y=[1, 0, 0]); off = 0.0116*a - 0.012*xh
qt = np.load(f'plan19_{deg}_{zc}.npy')[0]
Tpre = make_T(np.array([x, 0.15, 1.10]) - off, R)
qpre = ik(Tpre, qt); print('qpre', qpre.round(3), check(qpre, ignore=ign, extra=extras(0.175)))
print('direct', check_path([q0, qpre], ignore=ign, extra=extras(0.175)))
rng = np.random.default_rng(3); best=None
for t in range(400):
    qm = rng.uniform(LIM[:,0]+0.1, LIM[:,1]-0.1)
    # bias: interpolate then perturb
    s = rng.uniform(0.2,0.8); qm = q0*(1-s)+qpre*s + rng.normal(0,0.5,7); qm = np.clip(qm, LIM[:,0]+0.05, LIM[:,1]-0.05)
    c1 = check_path([q0, qm], ignore=ign, extra=extras(0.175), steps=8); c2 = check_path([qm, qpre], ignore=ign, extra=extras(0.175), steps=8)
    w = min(c1[0], c2[0])
    if best is None or w > best[0]: best=(w, qm, c1, c2)
print('best via', best[0], best[1].round(3), best[2], best[3])
np.save('via19.npy', np.array([best[1], qpre]))
EOF
timeout 900 python3 via19.py

# openrua op 99
cat > pushmodel.py <<'EOF'
import numpy as np, math
import collide
from collide import OBST
def hand_points_closed(Th):
    pts = []
    for x in (-0.03, 0.03):
        for y in np.linspace(-0.10, 0.10, 9):
            for z in (-0.04, 0.0, 0.035, 0.07):
                pts.append([x, y, z, 1])
    for x in (-0.012, 0.012):
        for y in (-0.01, 0.0, 0.01):
            for z in (0.075, 0.09, 0.105):
                pts.append([x, y, z, 1])
    return (Th @ np.array(pts).T).T[:, :3]
collide.hand_points = hand_points_closed
OBST['tophandle'] = ([-0.05, 0.19, 1.005], [0.06, 0.23, 1.11])
OBST['cabinet'] = ([-0.14, 0.24, 0.90], [0.145, 0.44, 1.14])
OBST['lintel'] = ([-0.14, 0.228, 0.99], [0.145, 0.44, 1.14])
ign = ('drawer','bottle')
def extras(yp):
    return {'bar': ([-0.045, yp-0.033, 0.938], [0.055, yp-0.009, 0.962]),
            'postL': ([-0.037, yp-0.02, 0.938], [-0.023, yp, 0.962]),
            'postR': ([0.028, yp-0.02, 0.938], [0.043, yp, 0.962]),
            'paneltop': ([-0.11, yp, 0.983], [0.12, yp+0.02, 0.99])}
EOF
sed -i '2s/.*/from pushmodel import */' via19.py && sed -i 's/^exec(open.*$/from pushmodel import */' step19.py && timeout 900 python3 via19.py

# openrua op 100
python3 - <<'EOF'
import re
s = open('step19.py').read()
s = s.replace("go(tcp_for(0.15, 1.10), yp=0.175, seed=seeds[0])",
"""via = np.load('via19.npy')
print('via path', check_path([r.q(), via[0]], ignore=ign, extra=extras(0.175)), check_path([via[0], via[1]], ignore=ign, extra=extras(0.175)))
print(r.move_q(via[0]), 'tcp', r.tcp()[:3,3].round(3))
print(r.move_q(via[1]), 'tcp', r.tcp()[:3,3].round(3), 'wrench', r.wrench().round(1))""")
open('step19.py','w').write(s)
EOF
timeout 900 python3 step19.py

# openrua op 101
timeout 600 python3 park.py -0.30,-0.05,1.35 | tail -1 && python3 grab.py birdview agentview >/dev/null && python3 - <<'EOF'
import numpy as np, cv2
from cloud import cloud
pw,col = cloud('birdview')
X,Y,Z = pw[...,0],pw[...,1],pw[...,2]
for name,(xlo,xhi) in [('panel-only x-0.08..-0.06',(-0.08,-0.06)),('through handle x-0.01..0.01',(-0.01,0.01))]:
    print(name)
    for ylo in np.arange(0.14,0.25,0.005):
        t=np.isfinite(Z)&(X>xlo)&(X<xhi)&(Y>=ylo)&(Y<ylo+0.005)&(Z>0.92)
        if t.sum(): print(f"  y {ylo:.3f}: z {Z[t].min():.3f}..{Z[t].max():.3f} n {t.sum()}")
vs,us = np.where(np.isfinite(Z)&(X>-0.2)&(X<0.2)&(Y>-0.1)&(Y<0.45))
cv2.imwrite('bird_crop.png', cv2.resize(col[vs.min():vs.max(), us.min():us.max()], None, fx=3, fy=3, interpolation=cv2.INTER_NEAREST))
a = cv2.imread('agentview.png'); h,w = a.shape[:2]
cv2.imwrite('agent_crop.png', cv2.resize(a[h//3:5*h//6, w//3:5*w//6], None, fx=2, fy=2))
EOF

# openrua op 102
mkdir -p "$(dirname /claude-config/projects/-workspace/memory/panda-sim-quirks.md)"
cat > /claude-config/projects/-workspace/memory/panda-sim-quirks.md <<'OPENRUA_EOF'
---
name: panda-sim-quirks
description: Non-obvious behaviours of the /workspace Panda ROS 2 sim (paused clock, wrench insensitivity, gripper slip on neck lifts, drawer handle geometry) learned during the bottle-in-drawer task (Sept 2026)
metadata:
  type: project
---

Learned while doing "put the bottle in the bottom drawer and close it" (completed 2026-09-12):

- Sim clock only advances while a command executes; re-read sensors after every move. Birdview is the reliable camera but the arm occludes it — park TCP at (-0.30,-0.05,1.35) before snapshots.
- `/franka_robot_state_broadcaster/external_wrench` barely reacts to light contacts (drawer sliding, pushing a bottle); only hard blocks (~20-30 N, error_code -5) show. Don't rely on it to detect first contact — verify by camera instead.
- Gripper fingers slip off the bottle neck when lifting/dragging near the cap; grasp lower on the neck (~0.125 above base) or accept dragging.
- `collide.pt_box_dist` returns 0 (not negative) for points inside a box; `collide.hand_points` models OPEN fingers — swap in `pushmodel.hand_points_closed` when fingers are closed.
- Drawer wall/panel tops are z≈0.983 (birdview), bottom handle bar 3 cm in front of panel (z 0.94-0.96), top handle occupies y≥0.197, z≥1.005. Pushing a round bar with a tilted fingertip face makes it slide off; push the flat panel face at x≈-0.08 with the hand ~10° below horizontal (see step19.py) instead.

**Why:** these cost many iterations to discover and are not written in machine.yaml/docs.
**How to apply:** if working in this workspace again, reuse panda.py/collide.py/pushmodel.py and the step*.py dry-run→`--go` pattern.
OPENRUA_EOF

# openrua op 103
cd /claude-config/projects/-workspace/memory && (test -f MEMORY.md && cat MEMORY.md || true) && echo "- [Panda sim quirks](panda-sim-quirks.md) — paused clock, wrench insensitivity, gripper slip, drawer/handle geometry, collide.py caveats" >> MEMORY.md && python3 /workspace/panda.py 2>&1 | tail -4
