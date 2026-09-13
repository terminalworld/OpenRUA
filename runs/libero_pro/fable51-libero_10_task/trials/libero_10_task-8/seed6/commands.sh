#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
cat /workspace/tools/action/fjt_send.py; echo ----; cat /workspace/tools/action/gripper_cmd.py; echo ----; cat /workspace/tools/action/ik_move.py

# openrua op 3
timeout 30 ros2 topic list; echo ----; timeout 20 ros2 action list; echo ----; timeout 20 ros2 node list

# openrua op 4
for c in agentview frontview birdview sideview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c; done; timeout 20 ros2 topic echo /joint_states --once

# openrua op 5
timeout 30 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -12; echo ----; timeout 20 ros2 topic echo /agentview/color/camera_info --once | head -30

# openrua op 6
mkdir -p "$(dirname /workspace/scene.py)"
cat > /workspace/scene.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Grab depth+info+TF for a camera once, convert many pixels to world.
Usage: python3 scene.py <camera> u,v [u,v ...]
Also saves <camera>_depth.npy and prints the camera pose in world.
"""
import struct
import sys

import numpy as np
import rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge


def grab(node, topic, msg_type, timeout=20.0):
    got = {}
    sub = node.create_subscription(msg_type, topic, lambda m: got.setdefault("m", m), 1)
    import time
    end = time.time() + timeout
    while "m" not in got and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    if "m" not in got:
        raise SystemExit(f"no message on {topic}")
    return got["m"]


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main():
    cam = sys.argv[1]
    pts = [tuple(int(a) for a in p.split(",")) for p in sys.argv[2:]]
    rclpy.init()
    node = rclpy.create_node("scene")
    tfbuf = Buffer()
    TransformListener(tfbuf, node)
    depth_msg = grab(node, f"/{cam}/depth/image_raw", Image)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    depth = CvBridge().imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
    np.save(f"{cam}_depth.npy", depth)
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    frame = f"{cam}_optical_frame"
    import time
    end = time.time() + 10
    while time.time() < end and not tfbuf.can_transform("world", frame, rclpy.time.Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    T = np.eye(4)
    T[:3, :3] = quat_R(q.x, q.y, q.z, q.w)
    T[:3, 3] = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
    np.save(f"{cam}_T.npy", T)
    np.save(f"{cam}_K.npy", np.array(info.k).reshape(3, 3))
    print("cam pose world:", T[:3, 3].round(3), "\nR=\n", T[:3, :3].round(3))
    for (u, v) in pts:
        z = float(depth[v, u])
        p = T @ np.array([(u - cx) * z / fx, (v - cy) * z / fy, z, 1.0])
        print(f"px({u},{v}) depth={z:.3f} -> world {p[0]:.4f} {p[1]:.4f} {p[2]:.4f}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 7
timeout 90 python3 scene.py birdview 262,282 330,313 387,281 262,235 300,400 200,200

# openrua op 8
python3 -c "
import numpy as np
d=np.load('birdview_depth.npy'); T=np.load('birdview_T.npy'); K=np.load('birdview_K.npy')
fx,fy,cx,cy=K[0,0],K[1,1],K[0,2],K[1,2]
H,W=d.shape
vs,us=np.mgrid[0:H,0:W]
z=d
X=(us-cx)*z/fx; Y=(vs-cy)*z/fy
P=np.stack([X,Y,z,np.ones_like(z)],-1)@T.T
wx,wy,wz=P[...,0],P[...,1],P[...,2]
# pot A region: world x in [-0.05,0.15], y in [-0.06,0.13]
for name,(x0,x1,y0,y1) in {'potA':(-0.05,0.15,-0.07,0.13),'potB':(-0.16,0.04,0.13,0.33),'stove':(-0.15,0.05,-0.31,-0.10)}.items():
    m=(wx>x0)&(wx<x1)&(wy>y0)&(wy<y1)&(wz>0.905)
    print(name,'n',m.sum(),'zmax',wz[m].max().round(3))
    for zl in [0.92,0.95,0.98,1.0,1.02,1.03]:
        mm=m&(wz>zl)
        if mm.sum()==0: continue
        print(f'  z>{zl}: n={mm.sum()} x[{wx[mm].min():.3f},{wx[mm].max():.3f}] y[{wy[mm].min():.3f},{wy[mm].max():.3f}] cx={wx[mm].mean():.3f} cy={wy[mm].mean():.3f}')
"


# openrua op 9
timeout 60 python3 scene.py sideview 310,330 260,290 >/dev/null; python3 -c "
import numpy as np
d=np.load('sideview_depth.npy'); T=np.load('sideview_T.npy'); K=np.load('sideview_K.npy')
print('cam',T[:3,3].round(3))
fx,fy,cx,cy=K[0,0],K[1,1],K[0,2],K[1,2]
H,W=d.shape
vs,us=np.mgrid[0:H,0:W]
z=d
P=np.stack([(us-cx)*z/fx,(vs-cy)*z/fy,z,np.ones_like(z)],-1)@T.T
wx,wy,wz=P[...,0],P[...,1],P[...,2]
m=(wx>-0.02)&(wx<0.12)&(wy>-0.10)&(wy<0.12)&(wz>0.9)&(wz<1.1)
print('potA pts',m.sum())
for zl in np.arange(0.90,1.07,0.01):
    mm=m&(wz>=zl)&(wz<zl+0.01)
    if mm.sum()==0: continue
    print(f'z {zl:.2f}: n={mm.sum():4d} x[{wx[mm].min():.3f},{wx[mm].max():.3f}] w={wx[mm].max()-wx[mm].min():.3f} ymin={wy[mm].min():.3f}')
"


# openrua op 10
timeout 60 python3 scene.py frontview 340,380 >/dev/null; python3 -c "
import numpy as np
d=np.load('frontview_depth.npy'); T=np.load('frontview_T.npy'); K=np.load('frontview_K.npy')
print('cam',T[:3,3].round(3))
fx,fy,cx,cy=K[0,0],K[1,1],K[0,2],K[1,2]
H,W=d.shape
vs,us=np.mgrid[0:H,0:W]
z=d
P=np.stack([(us-cx)*z/fx,(vs-cy)*z/fy,z,np.ones_like(z)],-1)@T.T
wx,wy,wz=P[...,0],P[...,1],P[...,2]
m=(wx>-0.05)&(wx<0.15)&(wy>-0.10)&(wy<0.14)&(wz>0.905)&(wz<1.1)
print('potA pts',m.sum())
for zl in np.arange(0.90,1.07,0.01):
    mm=m&(wz>=zl)&(wz<zl+0.01)
    if mm.sum()==0: continue
    print(f'z {zl:.2f}: n={mm.sum():4d} y[{wy[mm].min():.3f},{wy[mm].max():.3f}] w={wy[mm].max()-wy[mm].min():.3f} x[{wx[mm].min():.3f},{wx[mm].max():.3f}]')
# handle: points with y < 0.0 (body edge ~0.0)
print('handle region (y<0.005):')
h=m&(wy<0.005)
for zl in np.arange(0.90,1.07,0.01):
    mm=h&(wz>=zl)&(wz<zl+0.01)
    if mm.sum()==0: continue
    print(f'z {zl:.2f}: n={mm.sum():4d} y[{wy[mm].min():.3f},{wy[mm].max():.3f}] x[{wx[mm].min():.3f},{wx[mm].max():.3f}] xmean={wx[mm].mean():.3f}')
"


# openrua op 11
python3 -c "
import numpy as np
d=np.load('birdview_depth.npy'); T=np.load('birdview_T.npy'); K=np.load('birdview_K.npy')
fx,fy,cx,cy=K[0,0],K[1,1],K[0,2],K[1,2]
H,W=d.shape
vs,us=np.mgrid[0:H,0:W]
z=d
P=np.stack([(us-cx)*z/fx,(vs-cy)*z/fy,z,np.ones_like(z)],-1)@T.T
wx,wy,wz=P[...,0],P[...,1],P[...,2]
for name,(x0,x1,y0,y1) in {'potA':(-0.05,0.15,-0.07,0.13),'potB':(-0.16,0.04,0.13,0.33)}.items():
  m=(wx>x0)&(wx<x1)&(wy>y0)&(wy<y1)&(wz>0.905)
  print(name)
  for zl,zh in [(1.025,1.038),(1.038,1.06),(0.905,1.025)]:
    mm=m&(wz>=zl)&(wz<zh)
    print(f' z[{zl},{zh}] n={mm.sum()} x[{wx[mm].min():.3f},{wx[mm].max():.3f}] y[{wy[mm].min():.3f},{wy[mm].max():.3f}] med=({np.median(wx[mm]):.3f},{np.median(wy[mm]):.3f})')
  # y histogram of lid slice
  mm=m&(wz>=1.025)&(wz<1.038)
  hy,ey=np.histogram(wy[mm],bins=np.arange(y0,y1,0.01))
  print(' y hist', list(zip(ey[:-1].round(2),hy)))
  hx,ex=np.histogram(wx[mm],bins=np.arange(x0,x1,0.01))
  print(' x hist', list(zip(ex[:-1].round(2),hx)))
"


# openrua op 12
timeout 60 python3 -c "
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
rclpy.init(); n=rclpy.create_node('rd')
got=[]
qos=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
n.create_subscription(String,'/robot_description',got.append,qos)
import time; t=time.time()
while not got and time.time()-t<20: rclpy.spin_once(n,timeout_sec=0.5)
open('robot.urdf','w').write(got[0].data); print(len(got[0].data))
"; grep -n -A12 'link name="panda_hand"' robot.urdf | head -60; grep -n -B2 -A12 'link name="panda_leftfinger"' robot.urdf | head -50; grep -n -A6 'joint name="panda_hand_joint"' robot.urdf; grep -n -A6 'joint name="panda_finger_joint1"' robot.urdf; grep -n -B1 -A8 'panda_hand_tcp' robot.urdf | head -30

# openrua op 13
python3 - <<'EOF'
import re
s=open('robot.urdf').read()
for m in re.finditer(r'<joint name="(panda_hand_joint|panda_finger_joint1|panda_finger_joint2|panda_joint8|panda_hand_tcp_joint)"[^>]*>(.*?)</joint>', s, re.S):
    print(m.group(1), re.findall(r'<origin[^>]*>', m.group(2)), re.findall(r'<parent[^>]*>|<child[^>]*>', m.group(2)))
for l in ['panda_hand','panda_leftfinger','panda_link8','panda_hand_tcp']:
    m=re.search(rf'<link name="{l}">(.*?)</link>', s, re.S)
    print(l, m.group(1)[:400] if m else None)
EOF
ls /opt/ros/jazzy/share/moveit_resources_panda_description/meshes/collision/

# openrua op 14
python3 -c "
import cv2
im=cv2.imread('agentview.png'); crop=im[240:380,270:420]; cv2.imwrite('potA_zoom.png', cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('sideview.png'); crop=im[280:380,260:360]; cv2.imwrite('potA_side_zoom.png', cv2.resize(crop,None,fx=5,fy=5,interpolation=cv2.INTER_CUBIC))
"

# openrua op 15
timeout 120 python3 tools/action/gripper_cmd.py 0.04; timeout 20 ros2 topic echo /joint_states --once | grep -A9 position | tail -3

# openrua op 16
mkdir -p "$(dirname /workspace/rob.py)"
cat > /workspace/rob.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Small helper layer over the machine's ROS ports (built from machine.yaml).

Frames: world -> panda_link0 is a pure translation BASE_T (read from TF once);
poses given here are in WORLD and converted to the base frame for MoveIt.
"""
import sys
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import WrenchStamped
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState, Image, CameraInfo
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]
TCP = float(M["hand"]["tcp_offset_m"])
BASE_T = np.array([-0.660, 0.0, 0.912])  # world -> panda_link0 (tf2_echo)


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def R_quat(R):
    """rotation matrix -> (x,y,z,w)"""
    from scipy.spatial.transform import Rotation
    return Rotation.from_matrix(R).as_quat()


# hand orientations (world == base orientation). Hand z = approach axis,
# hand y = finger closing axis.
def R_topdown(close_axis="y"):
    """hand pointing straight down; fingers close along world x or y."""
    if close_axis == "y":
        return np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]], float)
    return np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]], float)


def R_from_axes(z_axis, y_axis):
    z = np.asarray(z_axis, float); z /= np.linalg.norm(z)
    y = np.asarray(y_axis, float); y -= y.dot(z) * z; y /= np.linalg.norm(y)
    x = np.cross(y, z)
    return np.stack([x, y, z], axis=1)


class Robot:
    def __init__(self, name="rob"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 1)
        self._wr = None
        self.node.create_subscription(WrenchStamped, "/franka_robot_state_broadcaster/external_wrench", self._on_wr, 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")

    def _on_js(self, m): self._wr_dummy = None; self._js = m
    def _on_wr(self, m): self._wr = m

    def spin(self, t=0.1):
        end = time.time() + t
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    # ---------------- sensing
    def joints(self, fresh=True):
        if fresh:
            self._js = None
            while self._js is None:
                rclpy.spin_once(self.node, timeout_sec=0.2)
        d = dict(zip(self._js.name, self._js.position))
        return d

    def arm_q(self):
        d = self.joints()
        return np.array([d[j] for j in ARM])

    def fingers(self):
        d = self.joints()
        return d["panda_finger_joint1"], d["panda_finger_joint2"]

    def wrench(self):
        self._wr = None
        end = time.time() + 5
        while self._wr is None and time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        if self._wr is None:
            return None
        f, t = self._wr.wrench.force, self._wr.wrench.torque
        return np.array([f.x, f.y, f.z, t.x, t.y, t.z])

    def fk_world(self, q=None, link="panda_hand"):
        """hand pose in WORLD: (pos, R)."""
        if q is None:
            q = self.arm_q()
        if not self.fk.wait_for_service(timeout_sec=10):
            raise RuntimeError("no FK service")
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = [float(x) for x in q]
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed {None if res is None else res.error_code.val}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE_T
        R = quat_R(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        return pos, R

    def tcp_world(self, q=None):
        pos, R = self.fk_world(q)
        return pos + TCP * R[:, 2], R

    # ---------------- IK
    def ik_world(self, pos, R, seed=None, tcp=True, attempts=1, timeout=1.0):
        """joint solution for hand (or tcp) pose in WORLD, or None."""
        pos = np.asarray(pos, float)
        if tcp:
            pos = pos - TCP * R[:, 2]
        pb = pos - BASE_T
        q = R_quat(R)
        if seed is None:
            seed = self.arm_q()
        if not self.ik.wait_for_service(timeout_sec=10):
            raise RuntimeError("no IK service")
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pb)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q)
        req.ik_request.robot_state.joint_state.name = list(ARM)
        req.ik_request.robot_state.joint_state.position = [float(x) for x in seed]
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=int(timeout), nanosec=int((timeout % 1) * 1e9))
        best = None
        for _ in range(attempts):
            fut = self.ik.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            res = fut.result()
            if res is not None and res.error_code.val == 1:
                sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
                cand = np.array([sol[j] for j in ARM])
                if best is None or np.abs(cand - seed).sum() < np.abs(best - seed).sum():
                    best = cand
        return best

    # ---------------- acting
    def move_q(self, q, seconds=3.0, via=None):
        """one trajectory; via = list of (q, t) intermediate points."""
        if not self.fjt.wait_for_server(timeout_sec=10):
            raise RuntimeError("no FJT server")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pts = []
        for qq, tt in (via or []) + [(q, seconds)]:
            pt = JointTrajectoryPoint(positions=[float(x) for x in qq])
            pt.time_from_start = Duration(sec=int(tt), nanosec=int((tt % 1) * 1e9))
            pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        qa = self.arm_q()
        err = np.abs(qa - np.asarray(q)).max()
        print(f"  move done code={code} max_joint_err={err:.4f}", flush=True)
        return code, err

    def move_tcp(self, pos, R, seconds=3.0, seed=None, attempts=3):
        q = self.ik_world(pos, R, seed=seed, attempts=attempts)
        if q is None:
            print(f"  IK FAILED for tcp {np.round(pos,3)}", flush=True)
            return None
        self.move_q(q, seconds)
        p, _ = self.tcp_world()
        print(f"  tcp now {np.round(p,4)} (target {np.round(pos,4)})", flush=True)
        return q

    def gripper(self, width):
        if not self.grip.wait_for_server(timeout_sec=10):
            raise RuntimeError("no gripper server")
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result
        f = self.fingers()
        print(f"  gripper -> reached={r.reached_goal} stalled={r.stalled} fingers={f[0]:.4f},{f[1]:.4f}", flush=True)
        return f

    def snap(self, cam, out=None):
        from cv_bridge import CvBridge
        import cv2
        got = {}
        sub = self.node.create_subscription(Image, f"/{cam}/color/image_raw", lambda m: got.setdefault("m", m), 1)
        end = time.time() + 30
        while "m" not in got and time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        self.node.destroy_subscription(sub)
        out = out or f"{cam}.png"
        cv2.imwrite(out, CvBridge().imgmsg_to_cv2(got["m"], desired_encoding="bgr8"))
        return out

    def depth_cloud(self, cam):
        """world-frame point cloud (H,W,3) from a camera's current depth."""
        from cv_bridge import CvBridge
        got = {}
        sub = self.node.create_subscription(Image, f"/{cam}/depth/image_raw", lambda m: got.setdefault("m", m), 1)
        end = time.time() + 30
        while "m" not in got and time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        self.node.destroy_subscription(sub)
        d = CvBridge().imgmsg_to_cv2(got["m"], desired_encoding="passthrough").astype(float)
        T = np.load(f"/workspace/{cam}_T.npy"); K = np.load(f"/workspace/{cam}_K.npy")
        fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
        H, W = d.shape
        vs, us = np.mgrid[0:H, 0:W]
        P = np.stack([(us - cx) * d / fx, (vs - cy) * d / fy, d, np.ones_like(d)], -1) @ T.T
        return P[..., :3]
OPENRUA_EOF

# openrua op 17
timeout 120 python3 -c "
from rob import *
r=Robot()
q=r.arm_q(); print('q',q.round(3))
p,R=r.fk_world(q); print('hand world',p.round(4)); print(R.round(3))
t,_=r.tcp_world(q); print('tcp world',t.round(4))
print('fingers',r.fingers()); print('wrench',r.wrench())
"

# openrua op 18
timeout 120 python3 -c "
from rob import *
import rob
r=Robot()
q=r.arm_q()
# raw FK (no BASE_T added)
p,R=r.fk_world(q); raw=p-BASE_T
print('raw fk pos',raw.round(4))
# IK with raw pose (tcp=False), no BASE_T shift -> set BASE_T to zero temporarily
rob.BASE_T=np.zeros(3)
s=r.ik_world(raw,R,seed=q,tcp=False); print('IK@raw  :',None if s is None else (s.round(3), np.abs(s-q).max().round(4)))
rob.BASE_T=np.array([-0.66,0,0.912])
s=r.ik_world(raw,R,seed=q,tcp=False); print('IK@raw-base:',None if s is None else (s.round(3), np.abs(s-q).max().round(4)))
"

# openrua op 19
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rob.py", "old_string": "BASE_T = np.array([-0.660, 0.0, 0.912])  # world -> panda_link0 (tf2_echo)", "new_string": "# NOTE: verified empirically: /compute_fk and /compute_ik with empty frame_id\\n# work in the WORLD frame (FK of the start pose gives z=1.27, matching the\\n# birdview), so no base shift is applied.\\nBASE_T = np.zeros(3)", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 20
timeout 300 python3 -u -c "
from rob import *
r=Robot()
R=R_topdown('x')
q=r.move_tcp([0.049,0.040,1.20],R,seconds=4.0)
print('q',None if q is None else q.round(3))
p,Rh=r.tcp_world(); print('R hand\n',Rh.round(3))
" 2>&1 | tail -8

# openrua op 21
timeout 60 python3 tools/perception/cam_snap.py sideview sv1.png >/dev/null; timeout 60 python3 tools/perception/cam_snap.py agentview av1.png >/dev/null; echo ok

# openrua op 22
timeout 400 python3 -u -c "
from rob import *
r=Robot()
R=R_topdown('x')
q=r.move_tcp([0.049,0.040,1.20],R,seconds=5.0)
print('q',None if q is None else q.round(3))
p,Rh=r.tcp_world(); print('R hand\n',Rh.round(3))
" 2>&1 | tail -8

# openrua op 23
timeout 100 python3 -u -c "
from rob import *
r=Robot()
q=r.arm_q(); print('actual q',q.round(3))
target=np.array([-0.088,0.552,0.191,-1.396,-0.107,1.938,-1.457])
print('diff',(q-target).round(3))
" 2>&1 | tail -3

# openrua op 24
timeout 200 python3 -u -c "
from rob import *
r=Robot()
q=r.arm_q(); q2=q.copy(); q2[6]=-1.2
r.move_q(q2,3.0)
print('actual q',r.arm_q().round(3))
" 2>&1 | tail -3

# openrua op 25
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rob.py", "old_string": "        code = res.result().result.error_code\\n        qa = self.arm_q()\\n        err = np.abs(qa - np.asarray(q)).max()\\n        print(f\\"  move done code={code} max_joint_err={err:.4f}\\", flush=True)\\n        return code, err", "new_string": "        code = res.result().result.error_code\\n        qa = self.arm_q()\\n        err = np.abs(qa - np.asarray(q)).max()\\n        print(f\\"  move done code={code} max_joint_err={err:.4f}\\", flush=True)\\n        # controller lag on big moves: resend the remainder (see docs)\\n        if err > tol and retries > 0:\\n            return self.move_q(q, max(1.5, seconds * min(1.0, err / 0.5)), retries=retries - 1, tol=tol)\\n        return code, err", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 26
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rob.py", "old_string": "    def move_q(self, q, seconds=3.0, via=None):", "new_string": "    def move_q(self, q, seconds=3.0, via=None, retries=2, tol=0.01):", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 27
timeout 400 python3 -u -c "
from rob import *
r=Robot()
R=R_topdown('x')
q=r.move_tcp([0.049,0.040,1.20],R,seconds=3.0)
p,Rh=r.tcp_world(); print('R hand\n',Rh.round(3))
print('fingers',r.fingers())
r.snap('sideview','sv2.png'); r.snap('robot0_eye_in_hand','eih2.png')
" 2>&1 | tail -8

# openrua op 28
python3 - <<'EOF'
s=open('rob.py').read()
s=s.replace('''        pb = pos - BASE_T
        q = R_quat(R)''','''        pb = pos - BASE_T
        # IK tip link is panda_link8; panda_hand = link8 * Rz(-pi/4)
        c, s_ = np.cos(np.pi / 4), np.sin(np.pi / 4)
        R8 = R @ np.array([[c, -s_, 0], [s_, c, 0], [0, 0, 1]])
        q = R_quat(R8)''')
open('rob.py','w').write(s)
EOF
timeout 400 python3 -u -c "
from rob import *
r=Robot()
R=R_topdown('x')
q=r.move_tcp([0.049,0.040,1.20],R,seconds=3.0)
p,Rh=r.tcp_world(); print('R hand\n',Rh.round(3))
r.snap('sideview','sv2.png'); r.snap('robot0_eye_in_hand','eih2.png')
" 2>&1 | tail -8

# openrua op 29
timeout 100 python3 -u -c "
from rob import *
r=Robot()
for cam in ['sideview','frontview']:
    P=r.depth_cloud(cam)
    x,y,z=P[...,0],P[...,1],P[...,2]
    m=(np.abs(x-0.049)<0.08)&(np.abs(y-0.04)<0.12)&(z>1.10)&(z<1.32)
    print(cam,'gripper pts',m.sum(),'zmin',z[m].min().round(4))
    for zl in np.arange(1.15,1.32,0.01):
        mm=m&(z>=zl)&(z<zl+0.01)
        if mm.sum(): print(f'  z{zl:.2f} n={mm.sum()} x[{x[mm].min():.3f},{x[mm].max():.3f}] y[{y[mm].min():.3f},{y[mm].max():.3f}]')
t,_=r.tcp_world(); print('tcp',t.round(4))
" 2>&1 | tail -40

# openrua op 30
timeout 600 python3 -u -c "
from rob import *
r=Robot()
R=R_topdown('x')
r.move_tcp([0.048,0.041,1.10],R,seconds=2.5)
r.move_tcp([0.048,0.041,1.043],R,seconds=2.0)
print('wrench before',r.wrench().round(2))
r.snap('robot0_eye_in_hand','eih3.png'); r.snap('sideview','sv3.png')
" 2>&1 | grep -v '^$' | tail -8

# openrua op 31
timeout 300 python3 -u -c "
from rob import *
r=Robot()
f=r.gripper(0.0)
print('wrench after',r.wrench().round(2))
r.snap('sideview','sv4.png')
" 2>&1 | grep -v '^$' | tail -4

# openrua op 32
timeout 400 python3 -u -c "
from rob import *
r=Robot()
R=R_topdown('x')
r.move_tcp([0.048,0.041,1.10],R,seconds=2.0)
print('fingers',np.round(r.fingers(),4),'wrench',r.wrench().round(2))
P=r.depth_cloud('birdview'); x,y,z=P[...,0],P[...,1],P[...,2]
m=(np.abs(x-0.048)<0.07)&(np.abs(y-0.041)<0.09)&(z>0.905)&(z<1.09)
print('potA-region pts',m.sum(),'zmin',z[m].min().round(3),'zmax',z[m].max().round(3))
for zl in np.arange(0.90,1.10,0.02):
    mm=m&(z>=zl)&(z<zl+0.02)
    if mm.sum(): print(f'  z{zl:.2f} n={mm.sum()} x[{x[mm].min():.3f},{x[mm].max():.3f}] y[{y[mm].min():.3f},{y[mm].max():.3f}]')
r.snap('sideview','sv5.png')
" 2>&1 | grep -v '^$' | tail -14

# openrua op 33
timeout 200 python3 -u -c "
from rob import *
r=Robot()
t,_=r.tcp_world(); print('tcp',t.round(4))
P=r.depth_cloud('sideview'); x,y,z=P[...,0],P[...,1],P[...,2]
m=(np.abs(x-0.048)<0.06)&(y>-0.06)&(y<0.12)&(z>0.905)&(z<1.05)
print('pot pts',m.sum(),'zmin',z[m].min().round(4))
for zl in np.arange(0.94,1.05,0.01):
    mm=m&(z>=zl)&(z<zl+0.01)
    if mm.sum(): print(f'  z{zl:.2f} n={mm.sum()} x[{x[mm].min():.3f},{x[mm].max():.3f}]')
" 2>&1 | tail -14

# openrua op 34
timeout 900 python3 -u -c "
from rob import *
r=Robot()
R=R_topdown('x')
r.move_tcp([0.048,0.041,1.20],R,seconds=2.5)
print('fingers',np.round(r.fingers(),4))
r.move_tcp([-0.058,-0.208,1.20],R,seconds=4.0)
print('fingers',np.round(r.fingers(),4))
r.snap('sideview','sv6.png'); r.snap('robot0_eye_in_hand','eih6.png')
" 2>&1 | grep -v '^$' | tail -8

# openrua op 35
timeout 900 python3 -u -c "
from rob import *
r=Robot()
R=R_topdown('x')
r.move_tcp([-0.058,-0.208,1.12],R,seconds=2.5)
print('wrench',r.wrench().round(2))
r.move_tcp([-0.058,-0.208,1.074],R,seconds=2.0)
print('wrench',r.wrench().round(2),'fingers',np.round(r.fingers(),4))
r.gripper(0.04)
r.move_tcp([-0.058,-0.208,1.20],R,seconds=2.5)
r.snap('sideview','sv7.png'); r.snap('agentview','av7.png'); r.snap('frontview','fv7.png')
" 2>&1 | grep -v '^$' | tail -12

# openrua op 36
timeout 200 python3 -u -c "
from rob import *
r=Robot()
P=r.depth_cloud('birdview'); x,y,z=P[...,0],P[...,1],P[...,2]
# stove region: anything above the stove plate (0.93) near stove center
m=(np.abs(x+0.058)<0.10)&(np.abs(y+0.208)<0.10)&(z>0.94)&(z<1.10)
print('pot-on-stove pts',m.sum(),'zmax',z[m].max().round(3))
lid=m&(z>1.025)&(z<1.038); knob=m&(z>1.038)
print('lid center',np.median(x[lid]).round(3),np.median(y[lid]).round(3),' knob center',np.median(x[knob]).round(3),np.median(y[knob]).round(3),'knob top',z[knob].max().round(3))
# old pot A location should now be empty table
m2=(np.abs(x-0.049)<0.06)&(np.abs(y-0.04)<0.06)&(z>0.905)
print('points left at old pot A spot:',m2.sum())
# pot B untouched?
m3=(np.abs(x+0.067)<0.06)&(np.abs(y-0.237)<0.08)&(z>1.038)
print('pot B knob',np.median(x[m3]).round(3),np.median(y[m3]).round(3),z[m3].max().round(3))
" 2>&1 | tail -5

# openrua op 37
timeout 600 python3 -u -c "
from rob import *
r=Robot()
R=R_topdown('x')
r.move_tcp([-0.25,0.0,1.30],R,seconds=3.0)
P=r.depth_cloud('birdview'); x,y,z=P[...,0],P[...,1],P[...,2]
m=(np.abs(x+0.058)<0.10)&(np.abs(y+0.208)<0.10)&(z>0.94)&(z<1.12)
print('pot-on-stove pts',m.sum(),'zmax',z[m].max().round(3))
for zl,zh in [(1.055,1.068),(1.068,1.10),(0.94,1.055)]:
    mm=m&(z>=zl)&(z<zh)
    if mm.sum(): print(f'  z[{zl},{zh}] n={mm.sum()} x[{x[mm].min():.3f},{x[mm].max():.3f}] y[{y[mm].min():.3f},{y[mm].max():.3f}] med=({np.median(x[mm]):.3f},{np.median(y[mm]):.3f})')
st=(np.abs(x+0.058)<0.12)&(np.abs(y+0.208)<0.12)&(z>0.92)&(z<0.94)
print('stove plate visible pts',st.sum(),'x',x[st].min().round(3),x[st].max().round(3),'y',y[st].min().round(3),y[st].max().round(3))
r.snap('sideview','sv8.png'); r.snap('agentview','av8.png'); r.snap('birdview','bv8.png')
" 2>&1 | tail -6
