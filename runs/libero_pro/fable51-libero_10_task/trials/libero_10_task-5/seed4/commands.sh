#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la && find . -maxdepth 3 -type f | head -100

# openrua op 1
cat tools/perception/cam_snap.py tools/perception/px2world.py tools/action/fjt_send.py tools/action/gripper_cmd.py tools/action/ik_move.py

# openrua op 2
timeout 30 ros2 topic list; echo ---; timeout 20 ros2 action list; echo ---; timeout 20 ros2 service list | grep -iE "ik|fk|plan|scene" ; echo ---; timeout 20 ros2 node list

# openrua op 3
timeout 30 ros2 topic echo /joint_states --once; for c in agentview frontview sideview birdview robot0_eye_in_hand robot0_robotview; do timeout 60 python3 tools/perception/cam_snap.py $c; done

# openrua op 4
python3 -c "
import cv2
im=cv2.imread('agentview.png'); c=im[140:290,110:380]; cv2.imwrite('crop_caddy_agent.png', cv2.resize(c,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('birdview.png'); c=im[140:300,200:400]; cv2.imwrite('crop_caddy_bird.png', cv2.resize(c,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
"

# openrua op 5
timeout 30 ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id|translation|x:|y:|z:|w:" | head -80; echo ---; timeout 20 ros2 topic echo /birdview/color/camera_info --once

# openrua op 6
mkdir -p "$(dirname /workspace/scene3d.py)"
cat > /workspace/scene3d.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Dump a camera's depth + TF as a world-frame point cloud helper.

Usage: python3 scene3d.py <camera>
Saves <camera>_xyz.npy (H x W x 3 world coords, NaN where invalid) and
prints the camera pose in world.
"""
import sys
import numpy as np
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage
from cv_bridge import CvBridge


def grab(node, topic, typ, qos=1):
    got = {}
    sub = node.create_subscription(typ, topic, lambda m: got.setdefault("m", m), qos)
    for _ in range(300):
        if "m" in got:
            break
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    if "m" not in got:
        raise SystemExit(f"no msg on {topic}")
    return got["m"]


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main():
    cam = sys.argv[1]
    rclpy.init()
    node = rclpy.create_node("scene3d")
    depth_msg = grab(node, f"/{cam}/depth/image_raw", Image)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    # collect TF (static + dynamic) for a moment
    tfs = {}
    def on_tf(m):
        for t in m.transforms:
            tfs[t.child_frame_id] = (t.header.frame_id, t.transform)
    qos_tl = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(TFMessage, "/tf_static", on_tf, qos_tl)
    node.create_subscription(TFMessage, "/tf", on_tf, 100)
    frame = f"{cam}_optical_frame"
    for _ in range(50):
        rclpy.spin_once(node, timeout_sec=0.2)
        if frame in tfs:
            break
    if frame not in tfs:
        print("known frames:", sorted(tfs))
        raise SystemExit(f"no TF for {frame}")
    # chain up to world
    T = np.eye(4)
    f = frame
    while f != "world":
        parent, tr = tfs[f]
        Ti = np.eye(4)
        q = tr.rotation
        Ti[:3, :3] = quat_R(q.x, q.y, q.z, q.w)
        Ti[:3, 3] = [tr.translation.x, tr.translation.y, tr.translation.z]
        T = Ti @ T
        f = parent
    print("cam->world T:\n", np.round(T, 4))
    depth = CvBridge().imgmsg_to_cv2(depth_msg, "passthrough").astype(np.float64)
    H, W = depth.shape
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    X = (u - cx) * depth / fx
    Y = (v - cy) * depth / fy
    P = np.stack([X, Y, depth, np.ones_like(depth)], -1) @ T.T
    xyz = P[..., :3]
    xyz[~np.isfinite(depth) | (depth <= 0)] = np.nan
    np.save(f"{cam}_xyz.npy", xyz)
    np.save(f"{cam}_depth.npy", depth)
    print("saved", f"{cam}_xyz.npy", "depth range", np.nanmin(depth), np.nanmax(depth))
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 7
timeout 120 python3 scene3d.py birdview && timeout 120 python3 scene3d.py agentview

# openrua op 8
python3 -c "
import numpy as np
xyz=np.load('birdview_xyz.npy')
z=xyz[...,2]
print('table z (median of large region):', np.nanmedian(z[300:450,300:600]))
# cup region in birdview approx (u 300-340, v 255-295)
for name,(v0,v1,u0,u1) in {'cup':(255,295,300,340),'caddy':(150,205,210,335),'book':(225,265,345,380)}.items():
    sub=xyz[v0:v1,u0:u1]
    zz=sub[...,2]
    print(name,'zmax',np.nanmax(zz),'z pct90',np.nanpercentile(zz,90))
    m=zz>np.nanmedian(z[300:450,300:600])+0.01
    pts=sub[m]
    print('  x range',np.nanmin(pts[:,0]),np.nanmax(pts[:,0]),' y range',np.nanmin(pts[:,1]),np.nanmax(pts[:,1]))
"


# openrua op 9
python3 -c "
import numpy as np
np.set_printoptions(linewidth=250, precision=2, suppress=True)
xyz=np.load('birdview_xyz.npy')
z=xyz[...,2]
# height map over caddy region every 4 px
sub=z[140:215:4,200:345:4]
print((sub-0.88).round(2))
print('cols u:',list(range(200,345,4)))
print('rows v:',list(range(140,215,4)))
"


# openrua op 10
python3 -c "
import numpy as np
np.set_printoptions(linewidth=250, precision=3, suppress=True)
xyz=np.load('birdview_xyz.npy')
z=xyz[...,2]
print('table near cup:', np.nanmedian(z[300:330,250:290]), np.nanmedian(z[250:300,200:240]))
sub=z[250:300:2,295:345:2]
print((sub-0.88).round(2))
print('cols',list(range(295,345,2)))
print('rows',list(range(250,300,2)))
# world coords of key pixels
for (u,v) in [(280,170),(280,190),(240,180),(320,270),(264,170),(296,170),(280,162),(280,178)]:
    print((u,v), xyz[v,u].round(3))
"


# openrua op 11
python3 -c "
import numpy as np, cv2
np.set_printoptions(linewidth=250, precision=3, suppress=True)
xyz=np.load('agentview_xyz.npy')
# cup in agentview approx u 260-360, v 290-390
sub=xyz[280:400,250:370]
z=sub[...,2]; x=sub[...,0]; y=sub[...,1]
m=(z>0.885)&(x>-0.2)&(x<0.05)&(np.abs(y)<0.12)
pts=sub[m]
print('n',len(pts))
for zl in np.arange(0.88,1.06,0.01):
    s=pts[(pts[:,2]>=zl)&(pts[:,2]<zl+0.01)]
    if len(s): print(f'z {zl:.2f}: n={len(s):4d} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]')
im=cv2.imread('agentview.png'); c=im[270:400,240:380]; cv2.imwrite('crop_cup_agent.png', cv2.resize(c,None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))
"


# openrua op 12
python3 -c "
import numpy as np
np.set_printoptions(linewidth=250, precision=2, suppress=True)
xyz=np.load('birdview_xyz.npy')
z=xyz[...,2]
# fine height profile along v at u=280 (middle column) and u=240 (left comp), and along u at v=170
print('v profile u=280:'); 
for v in range(155,205): print(v, round(xyz[v,280,0],3), round(z[v,280]-0.88,3))
print('u profile v=170:')
for u in range(212,304,2): print(u, round(xyz[170,u,1],3), round(z[170,u]-0.88,3))
"


# openrua op 13
timeout 30 python3 -c "
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf2_msgs.msg import TFMessage
rclpy.init(); n=rclpy.create_node('tfdump')
tfs={}
def cb(m):
    for t in m.transforms: tfs[t.child_frame_id]=(t.header.frame_id, t.transform.translation, t.transform.rotation)
n.create_subscription(TFMessage,'/tf_static',cb,QoSProfile(depth=100,durability=DurabilityPolicy.TRANSIENT_LOCAL))
n.create_subscription(TFMessage,'/tf',cb,100)
for _ in range(30): rclpy.spin_once(n,timeout_sec=0.2)
for c,(p,t,r) in sorted(tfs.items()):
    print(f'{p:>28} -> {c:<28} t=({t.x:.4f},{t.y:.4f},{t.z:.4f}) q=({r.x:.4f},{r.y:.4f},{r.z:.4f},{r.w:.4f})')
"

# openrua op 14
timeout 300 python3 tools/action/gripper_cmd.py 0.04; timeout 30 ros2 topic echo /joint_states --once | grep -A9 position

# openrua op 15
timeout 120 python3 scene3d.py birdview >/dev/null && python3 -c "
import numpy as np
xyz=np.load('birdview_xyz.npy'); z=xyz[...,2]
sub=z[255:300,300:340]; print('cup zmax', np.nanmax(sub), 'inner floor', np.nanmin(sub[15:30,12:28]))
# book
print('book zmax', np.nanmax(z[225:265,345:380]))
"; timeout 120 python3 scene3d.py agentview >/dev/null && python3 -c "
import numpy as np
xyz=np.load('agentview_xyz.npy')
sub=xyz[280:400,250:370]; z=sub[...,2]; x=sub[...,0]; y=sub[...,1]
m=(z>0.885)&(x>-0.2)&(x<0.05)&(np.abs(y)<0.12); pts=sub[m]
for zl in np.arange(0.88,1.06,0.01):
    s=pts[(pts[:,2]>=zl)&(pts[:,2]<zl+0.01)]
    if len(s): print(f'z {zl:.2f}: n={len(s):4d} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]')
"

# openrua op 16
python3 -c "
import numpy as np
xyz=np.load('birdview_xyz.npy'); z=xyz[...,2]
sub=xyz[250:300,295:345]; zz=sub[...,2]
m=zz>0.975  # rim points
pts=sub[m]
print('rim n',len(pts),'x',pts[:,0].min(),pts[:,0].max(),'y',pts[:,1].min(),pts[:,1].max())
# circle fit least squares
x,y=pts[:,0],pts[:,1]
A=np.c_[2*x,2*y,np.ones_like(x)]; b=x**2+y**2
c=np.linalg.lstsq(A,b,rcond=None)[0]; r=np.sqrt(c[2]+c[0]**2+c[1]**2)
print('rim circle center',c[0],c[1],'radius',r)
# handle pts (z between 0.93 and 0.97 in birdview, outside circle)
m2=(zz>0.90)&(zz<0.975); p2=sub[m2]
d=np.hypot(p2[:,0]-c[0],p2[:,1]-c[1]); h=p2[d>r]
print('handle n',len(h),'x',h[:,0].min(),h[:,0].max(),'y',h[:,1].min(),h[:,1].max(),'z',h[:,2].min(),h[:,2].max())
"

# openrua op 17
python3 -c "
import cv2
im=cv2.imread('frontview.png'); c=im[240:340,160:420]; cv2.imwrite('crop_caddy_front.png', cv2.resize(c,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('sideview.png'); c=im[200:340,380:520]; cv2.imwrite('crop_caddy_side.png', cv2.resize(c,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
"

# openrua op 18
timeout 30 python3 -c "
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
rclpy.init(); n=rclpy.create_node('rd')
got=[]
n.create_subscription(String,'/robot_description',lambda m: got.append(m.data),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
for _ in range(50):
    rclpy.spin_once(n,timeout_sec=0.2)
    if got: break
open('robot.urdf','w').write(got[0]); print(len(got[0]))
"; grep -n -A12 'link name="panda_hand"' robot.urdf | head -60; grep -n -B2 -A8 'link name="panda_leftfinger"' robot.urdf | head -40

# openrua op 19
python3 -c "
import numpy as np, struct
for name in ['hand','finger']:
    f=f'/opt/ros/jazzy/share/moveit_resources_panda_description/meshes/collision/{name}.stl'
    d=open(f,'rb').read()
    if d[:5]==b'solid':
        import re
        v=np.array([list(map(float,m)) for m in re.findall(r'vertex\s+(\S+)\s+(\S+)\s+(\S+)',d.decode())])
    else:
        n=struct.unpack('<I',d[80:84])[0]
        arr=np.frombuffer(d[84:84+n*50],dtype=np.dtype([('n','<f4',3),('v','<f4',(3,3)),('a','<u2')]))
        v=arr['v'].reshape(-1,3)
    print(name,'min',v.min(0).round(4),'max',v.max(0).round(4))
"

# openrua op 20
find / -iname "hand*.stl" -o -iname "hand*.obj" -o -iname "finger*.stl" 2>/dev/null | head; find / -path /proc -prune -o -iname "*caddy*" -print 2>/dev/null | head; find / -path /proc -prune -o -iname "*mug*" -print 2>/dev/null | head

# openrua op 21
mkdir -p "$(dirname /workspace/arm.py)"
cat > /workspace/arm.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Small helper library: joint state, FK, IK, trajectory, gripper.

World <-> base: panda_link0 sits at WORLD_BASE in world (from TF).
All pose helpers take WORLD-frame TCP poses (fingertip point) and
convert to the planner's base frame internally.
"""
import sys
import time
import numpy as np
import rclpy
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from geometry_msgs.msg import WrenchStamped

WORLD_BASE = np.array([-0.75, 0.0, 0.912])
TCP_OFF = 0.1034
JOINTS = [f"panda_joint{i}" for i in range(1, 8)]
FJT = "/panda_arm_controller/follow_joint_trajectory"
GRIP = "/franka_gripper/gripper_action"


def quat_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def R_quat(R):
    """rotation matrix -> quaternion (x,y,z,w)"""
    m = R
    t = np.trace(m)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)


def Rx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def Ry(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def Rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


# hand pointing down (hand z = -Z), fingers along world X (hand y = +X)
R_DOWN_FX = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]], dtype=float)
# hand pointing down, fingers along world Y (hand y = -Y, hand x = +X)
R_DOWN_FY = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]], dtype=float)


class Arm:
    def __init__(self, name="arm_helper"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = None
        self._wr = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.node.create_subscription(WrenchStamped,
                                      "/franka_robot_state_broadcaster/external_wrench",
                                      self._on_wr, 10)
        self.ik_cli = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT)
        self.grip = ActionClient(self.node, GripperCommand, GRIP)

    def _on_js(self, m):
        self._js = m

    def _on_wr(self, m):
        self._wr = m

    def spin(self, t=0.1):
        rclpy.spin_once(self.node, timeout_sec=t)

    # ---------------- sensing ----------------
    def joints(self, fresh=True):
        if fresh:
            self._js = None
        for _ in range(100):
            if self._js is not None:
                break
            self.spin(0.2)
        d = dict(zip(self._js.name, self._js.position))
        return d

    def arm_q(self):
        d = self.joints()
        return np.array([d[j] for j in JOINTS])

    def finger_gap(self):
        d = self.joints()
        return d["panda_finger_joint1"], d["panda_finger_joint2"]

    def wrench(self):
        self._wr = None
        for _ in range(50):
            if self._wr is not None:
                break
            self.spin(0.2)
        if self._wr is None:
            return None
        f, t = self._wr.wrench.force, self._wr.wrench.torque
        return np.array([f.x, f.y, f.z, t.x, t.y, t.z])

    # ---------------- kinematics ----------------
    def fk(self, q=None, link="panda_hand"):
        """returns (pos_world, R) of link for arm config q (default: current)"""
        if q is None:
            q = self.arm_q()
        if not self.fk_cli.wait_for_service(timeout_sec=10):
            raise RuntimeError("no FK service")
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = JOINTS
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {None if res is None else res.error_code.val}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + WORLD_BASE
        R = quat_R([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        return pos, R

    def tcp(self, q=None):
        pos, R = self.fk(q)
        return pos + TCP_OFF * R[:, 2], R

    def ik(self, tcp_world, R, seed=None, attempts=3):
        """IK for a TCP pose in world. Returns joint array or None."""
        hand = np.asarray(tcp_world, float) - TCP_OFF * R[:, 2]
        base = hand - WORLD_BASE
        q = R_quat(R)
        if seed is None:
            seed = self.arm_q()
        if not self.ik_cli.wait_for_service(timeout_sec=10):
            raise RuntimeError("no IK service")
        for _ in range(attempts):
            req = GetPositionIK.Request()
            req.ik_request.group_name = "panda_arm"
            req.ik_request.pose_stamped.header.frame_id = ""
            p = req.ik_request.pose_stamped.pose
            p.position.x, p.position.y, p.position.z = map(float, base)
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q)
            req.ik_request.robot_state.joint_state.name = JOINTS
            req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
            req.ik_request.avoid_collisions = False
            req.ik_request.timeout = Duration(sec=1)
            fut = self.ik_cli.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            res = fut.result()
            if res is not None and res.error_code.val == 1:
                sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
                return np.array([sol[j] for j in JOINTS])
        return None

    # ---------------- acting ----------------
    def move_q(self, q, seconds=3.0, via=None):
        """one trajectory goal through optional via points to q"""
        if not self.fjt.wait_for_server(timeout_sec=10):
            raise RuntimeError("no FJT server")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        pts = []
        wps = (via or []) + [q]
        for i, wp in enumerate(wps):
            t = seconds * (i + 1) / len(wps)
            pt = JointTrajectoryPoint(positions=[float(v) for v in wp])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        qn = self.arm_q()
        err = np.abs(qn - np.asarray(q)).max()
        print(f"move_q: error_code={code} max_joint_err={err:.4f}")
        return code, err

    def move_tcp(self, tcp_world, R, seconds=3.0, seed=None, max_jump=None):
        q = self.ik(tcp_world, R, seed=seed)
        if q is None:
            print("IK FAILED for", np.round(tcp_world, 3))
            return None
        q0 = self.arm_q()
        jump = np.abs(q - q0).max()
        print(f"IK ok, max joint jump {jump:.3f} rad")
        if max_jump is not None and jump > max_jump:
            print("jump too large; not moving")
            return None
        self.move_q(q, seconds)
        p, Rn = self.tcp()
        print("tcp now", np.round(p, 4), "target", np.round(tcp_world, 4),
              "err", np.round(np.linalg.norm(p - tcp_world), 4))
        return q

    def gripper(self, width, timeout=300):
        if not self.grip.wait_for_server(timeout_sec=10):
            raise RuntimeError("no gripper server")
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = 30.0
        fut = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=timeout)
        r = rf.result().result
        gap = self.finger_gap()
        print(f"gripper: reached={r.reached_goal} stalled={r.stalled} fingers={gap}")
        return gap
OPENRUA_EOF

# openrua op 22
timeout 120 python3 -c "
from arm import *
a=Arm()
q=a.arm_q(); print('q',q.round(3))
p,R=a.fk(q); print('hand world',p.round(4)); print(R.round(3))
t,_=a.tcp(q); print('tcp world',t.round(4))
print('quat',R_quat(R).round(4))
"

# openrua op 23
timeout 120 python3 -c "
from arm import *
import arm
a=Arm()
q=a.arm_q()
p,R=a.fk(q)
# arm.fk added WORLD_BASE; raw service value:
raw=p-WORLD_BASE
print('raw fk',raw.round(4))
# IK test: treat raw as base-frame (as arm.ik does when given world = raw+WORLD_BASE)
sol=a.ik(raw+WORLD_BASE+TCP_OFF*R[:,2], R, seed=q)
print('ik(raw as base):', None if sol is None else sol.round(3))
sol2=a.ik(raw+TCP_OFF*R[:,2]+WORLD_BASE-WORLD_BASE+np.array([0,0,0]), R, seed=q) if False else None
# IK treating raw as world: pass tcp_world=raw+TCP*Rz, but arm.ik subtracts WORLD_BASE -> so add it back
arm.WORLD_BASE=np.zeros(3)
sol2=a.ik(raw+TCP_OFF*R[:,2], R, seed=q)
print('ik(raw as world):', None if sol2 is None else sol2.round(3))
print('current q', q.round(3))
"

# openrua op 24
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/arm.py", "old_string": "WORLD_BASE = np.array([-0.75, 0.0, 0.912])", "new_string": "# Verified: /compute_fk for the current config returns x=-0.293 which matches\\n# the hand seen in birdview at world x=-0.294, so the planner\'s model frame\\n# IS world on this machine (panda_link0 is at (-0.75,0,0.912) in it).\\nWORLD_BASE = np.array([0.0, 0.0, 0.0])", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 25
timeout 300 python3 -c "
from arm import *
a=Arm()
q0=a.arm_q()
R1=R_DOWN_FX
R2=R_DOWN_FX@Rz(np.pi)  # rotate about hand z by 180
poses={'pregrasp':([-0.032,-0.003,1.10]),'grasp':([-0.032,-0.003,0.965]),'lift':([-0.032,-0.003,1.20])}
for name,p in poses.items():
    for rn,R in [('FX',R1),('FX180',R2)]:
        s=a.ik(p,R,seed=q0)
        print(name,rn, None if s is None else s.round(3))
# release pose: R = Rx(90) Rz(90) R_DOWN_FX
Rrel=Rx(np.pi/2)@Rz(np.pi/2)@R1
print('Rrel',Rrel.round(2))
for z in [1.14,1.20]:
    s=a.ik([-0.42,-0.195,z],Rrel,seed=q0); print('release',z,None if s is None else s.round(3))
"

# openrua op 26
timeout 600 python3 -c "
from arm import *
a=Arm()
R2=R_DOWN_FX@Rz(np.pi)
np.save('R_grasp.npy',R2)
q=a.move_tcp([-0.032,-0.003,1.10],R2,seconds=4.0)
print(a.arm_q().round(3))
" && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 27
timeout 600 python3 -c "
from arm import *
a=Arm()
R2=np.load('R_grasp.npy')
q=a.move_tcp([-0.032,-0.003,1.10],R2,seconds=4.0)
print(a.arm_q().round(3))
"

# openrua op 28
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand && timeout 120 python3 scene3d.py birdview > /dev/null && python3 -c "
import numpy as np
xyz=np.load('birdview_xyz.npy'); sub=xyz[250:300,295:345]; zz=sub[...,2]
m=(zz>0.975)&(zz<1.0); pts=sub[m]
x,y=pts[:,0],pts[:,1]
A=np.c_[2*x,2*y,np.ones_like(x)]; b=x**2+y**2
c=np.linalg.lstsq(A,b,rcond=None)[0]; print('rim center',c[:2].round(4),'r',np.sqrt(c[2]+c[0]**2+c[1]**2).round(4),'n',len(pts))
"

# openrua op 29
timeout 600 python3 -c "
from arm import *
a=Arm()
R2=np.load('R_grasp.npy')
q0=a.arm_q()
qv=a.ik([-0.032,-0.003,1.03],R2,seed=q0)
qg=a.ik([-0.032,-0.003,0.965],R2,seed=qv)
print('via',qv.round(3)); print('grasp',qg.round(3))
w0=a.wrench(); print('wrench before',None if w0 is None else w0.round(2))
a.move_q(qg,seconds=4.0,via=[qv])
p,R=a.tcp(); print('tcp',p.round(4))
w1=a.wrench(); print('wrench after',None if w1 is None else w1.round(2))
" && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 30
timeout 600 python3 -c "
from arm import *
a=Arm()
gap=a.gripper(0.0)
w=a.wrench(); print('wrench',w.round(2))
" && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 31
timeout 60 python3 tools/perception/cam_snap.py frontview && timeout 60 python3 tools/perception/cam_snap.py sideview && python3 -c "
import cv2
im=cv2.imread('frontview.png'); c=im[200:400,200:440]; cv2.imwrite('crop_front.png', cv2.resize(c,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('sideview.png'); c=im[150:350,200:450]; cv2.imwrite('crop_side.png', cv2.resize(c,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
"

# openrua op 32
timeout 600 python3 -c "
from arm import *
a=Arm()
R2=np.load('R_grasp.npy')
q0=a.arm_q()
q=a.move_tcp([-0.032,-0.003,1.12],R2,seconds=3.0,seed=q0)
print('fingers',a.finger_gap())
" && timeout 60 python3 tools/perception/cam_snap.py frontview && timeout 60 python3 tools/perception/cam_snap.py sideview && python3 -c "
import cv2
im=cv2.imread('frontview.png'); c=im[150:400,200:440]; cv2.imwrite('crop_front.png', cv2.resize(c,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('sideview.png'); c=im[100:350,200:450]; cv2.imwrite('crop_side.png', cv2.resize(c,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
"

# openrua op 33
timeout 600 python3 -c "
from arm import *
a=Arm()
R2=np.load('R_grasp.npy')
q0=a.arm_q()
RA=Rz(np.pi/2)@R2
RB=Rx(np.pi/2)@RA
print('RA',RA.round(2)); print('RB',RB.round(2))
qa=a.ik([-0.032,-0.003,1.20],RA,seed=q0); print('A',None if qa is None else qa.round(3), 'jump', None if qa is None else np.abs(qa-q0).max().round(3))
# B at various positions
for p in [[-0.032,-0.003,1.25],[-0.15,-0.05,1.25],[-0.25,-0.10,1.25],[-0.42,-0.195,1.20],[-0.42,-0.195,1.14]]:
    qb=a.ik(p,RB,seed=qa if qa is not None else q0)
    print('B',p,None if qb is None else qb.round(3),'jump',None if qb is None else np.abs(qb-(qa if qa is not None else q0)).max().round(3))
np.save('RA.npy',RA); np.save('RB.npy',RB)
"

# openrua op 34
timeout 900 python3 -c "
from arm import *
a=Arm()
RA=np.load('RA.npy')
q0=a.arm_q()
qa=a.ik([-0.032,-0.003,1.20],RA,seed=q0)
a.move_q(qa,seconds=4.0)
p,R=a.tcp(); print('tcp',p.round(4)); print(R.round(2)); print('fingers',a.finger_gap())
" && timeout 60 python3 tools/perception/cam_snap.py frontview && python3 -c "
import cv2
im=cv2.imread('frontview.png'); c=im[100:400,200:440]; cv2.imwrite('crop_front.png', cv2.resize(c,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
"

# openrua op 35
timeout 60 python3 tools/perception/cam_snap.py sideview && timeout 60 python3 tools/perception/cam_snap.py agentview && python3 -c "
import cv2
im=cv2.imread('sideview.png'); c=im[80:330,150:450]; cv2.imwrite('crop_side.png', cv2.resize(c,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('agentview.png'); c=im[100:350,180:450]; cv2.imwrite('crop_agent.png', cv2.resize(c,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
"

# openrua op 36
timeout 900 python3 -c "
from arm import *
a=Arm()
RA=np.load('RA.npy'); RB=np.load('RB.npy')
q0=a.arm_q(); print('q0',q0.round(3))
Rh=Rx(np.pi/4)@RA
qh=a.ik([-0.10,-0.03,1.25],Rh,seed=q0); print('half',qh.round(3),'jump',np.abs(qh-q0).max().round(3))
a.move_q(qh,seconds=4.0)
p,R=a.tcp(); print('tcp',p.round(4)); print(R.round(2))
q1=a.arm_q()
qb=a.ik([-0.15,-0.05,1.25],RB,seed=q1); print('B',qb.round(3),'jump',np.abs(qb-q1).max().round(3))
a.move_q(qb,seconds=4.0)
p,R=a.tcp(); print('tcp',p.round(4)); print(R.round(2)); print('fingers',a.finger_gap())
" && timeout 60 python3 tools/perception/cam_snap.py frontview && timeout 60 python3 tools/perception/cam_snap.py sideview && python3 -c "
import cv2
im=cv2.imread('sideview.png'); c=im[50:330,150:500]; cv2.imwrite('crop_side.png', cv2.resize(c,None,fx=2,fy=2,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('frontview.png'); c=im[50:350,150:450]; cv2.imwrite('crop_front.png', cv2.resize(c,None,fx=2,fy=2,interpolation=cv2.INTER_CUBIC))
"

# openrua op 37
timeout 300 python3 -c "
from arm import *
a=Arm()
home=np.array([0,-0.161,0,-2.445,0,2.227,0.785])
for dq7 in [0,np.pi/4,np.pi/2]:
    q=home.copy(); q[6]+=dq7
    p,R=a.fk(q); print('dq7',round(dq7,3),'hand y axis',R[:,1].round(3),'hand x',R[:,0].round(3))
qg=np.array([-0.006,0.881,0.002,-1.457,-0.003,2.338,1.568])
p,R=a.fk(qg); print('grasp cfg hand y',R[:,1].round(3),'x',R[:,0].round(3),'z',R[:,2].round(3))
q=a.arm_q(); p,R=a.fk(q); print('now q',q.round(3)); print(R.round(3))
"

# openrua op 38
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/arm.py", "old_string": "        hand = np.asarray(tcp_world, float) - TCP_OFF * R[:, 2]\\n        base = hand - WORLD_BASE\\n        q = R_quat(R)", "new_string": "        hand = np.asarray(tcp_world, float) - TCP_OFF * R[:, 2]\\n        base = hand - WORLD_BASE\\n        # the IK tip link is panda_link8, which is panda_hand rotated by\\n        # +45deg about z (panda_hand_joint rpy z=-0.785). Verified: asking\\n        # for hand y=-X returned a config whose FK hand y was 45deg off.\\n        q = R_quat(R @ Rz(np.pi / 4))", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 39
timeout 300 python3 -c "
from arm import *
a=Arm()
q=a.arm_q(); p,R=a.tcp(q); print('tcp',p.round(4)); print('R actual'); print(R.round(3))
# IK roundtrip check: ask for the current pose, see if solution FK matches
s=a.ik(p,R,seed=q); p2,R2=a.tcp(s); print('roundtrip pos err',np.linalg.norm(p2-p).round(4),'R err',np.abs(R2-R).max().round(4))
# Check IK for the target lying-orientation with fingers vertical
RB=np.load('RB.npy'); s=a.ik([-0.42,-0.195,1.14],RB,seed=q); print('RB sol',None if s is None else s.round(3))
if s is not None:
    p3,R3=a.tcp(s); print(R3.round(3))
"

# openrua op 40
timeout 200 python3 scene3d.py frontview >/dev/null && timeout 200 python3 scene3d.py sideview >/dev/null && python3 -c "
import numpy as np, cv2
for cam in ['frontview','sideview']:
    xyz=np.load(f'{cam}_xyz.npy'); im=cv2.imread(f'{cam}.png')
    b,g,r=[im[...,i].astype(int) for i in range(3)]
    yellow=(r>120)&(g>100)&(b<90)&(r-b>60)
    pts=xyz[yellow]; pts=pts[np.isfinite(pts).all(1)]
    pts=pts[pts[:,2]>1.0]
    print(cam,'yellow n',len(pts))
    if len(pts):
        print('  x',pts[:,0].min().round(3),pts[:,0].max().round(3),' y',pts[:,1].min().round(3),pts[:,1].max().round(3),' z',pts[:,2].min().round(3),pts[:,2].max().round(3))
        print('  mean',pts.mean(0).round(3))
    # all points in a box around the cup
    box=(xyz[...,0]>-0.25)&(xyz[...,0]<-0.02)&(xyz[...,1]>-0.15)&(xyz[...,1]<0.12)&(xyz[...,2]>1.0)&(xyz[...,2]<1.3)
    P=xyz[box]; print('  box n',len(P))
    np.save(f'{cam}_cupbox.npy',P)
"

# openrua op 41
python3 -c "
import numpy as np, cv2
np.set_printoptions(linewidth=250)
for cam in ['frontview','sideview']:
    xyz=np.load(f'{cam}_xyz.npy'); im=cv2.imread(f'{cam}.png')
    b,g,r=[im[...,i].astype(int) for i in range(3)]
    yellow=(r>120)&(g>100)&(b<90)&(r-b>60)
    box=(xyz[...,0]>-0.30)&(xyz[...,0]<-0.02)&(xyz[...,1]>-0.20)&(xyz[...,1]<0.12)&(xyz[...,2]>1.05)&(xyz[...,2]<1.35)
    P=xyz[box&yellow]; print(cam,'yellow in box n',len(P))
    if len(P): print('  x',P[:,0].min().round(3),P[:,0].max().round(3),' y',P[:,1].min().round(3),P[:,1].max().round(3),' z',P[:,2].min().round(3),P[:,2].max().round(3),' mean',P.mean(0).round(3))
    # gray-ish (non-yellow) points in box: hist in y
    Q=xyz[box&~yellow]
    h,e=np.histogram(Q[:,1],bins=np.arange(-0.20,0.13,0.01)); print('  nonyellow y hist',dict(zip(e[:-1].round(2),h)))
    h,e=np.histogram(P[:,1],bins=np.arange(-0.20,0.13,0.01)); print('  yellow y hist',dict(zip(e[:-1].round(2),h)))
"

# openrua op 42
python3 -c "
import numpy as np, cv2
allp=[]
for cam in ['frontview','sideview']:
    xyz=np.load(f'{cam}_xyz.npy'); im=cv2.imread(f'{cam}.png')
    b,g,r=[im[...,i].astype(int) for i in range(3)]
    yellow=(r>120)&(g>100)&(b<90)&(r-b>60)
    box=(xyz[...,0]>-0.30)&(xyz[...,0]<-0.02)&(xyz[...,1]>-0.20)&(xyz[...,1]<0.12)&(xyz[...,2]>1.05)&(xyz[...,2]<1.35)
    allp.append(xyz[box&yellow])
P=np.vstack(allp)
for y0,y1 in [(-0.07,-0.04),(-0.04,-0.01),(-0.01,0.02),(0.02,0.05)]:
    s=P[(P[:,1]>=y0)&(P[:,1]<y1)]
    if len(s)<10: continue
    x,z=s[:,0],s[:,2]
    A=np.c_[2*x,2*z,np.ones_like(x)]; bb=x**2+z**2
    c=np.linalg.lstsq(A,bb,rcond=None)[0]; r=np.sqrt(c[2]+c[0]**2+c[1]**2)
    print(f'y[{y0},{y1}) n={len(s)} circle center x={c[0]:.3f} z={c[1]:.3f} r={r:.3f}  x-range {x.min():.3f}..{x.max():.3f} z-range {z.min():.3f}..{z.max():.3f}')
"

# openrua op 43
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand && timeout 60 python3 tools/perception/cam_snap.py robot0_robotview && timeout 60 python3 tools/perception/cam_snap.py birdview

# openrua op 44
timeout 120 python3 scene3d.py birdview >/dev/null && python3 -c "
import numpy as np, cv2
xyz=np.load('birdview_xyz.npy'); im=cv2.imread('birdview.png')
b,g,r=[im[...,i].astype(int) for i in range(3)]
yellow=(r>120)&(g>100)&(b<90)&(r-b>60)
box=(xyz[...,0]>-0.30)&(xyz[...,0]<-0.02)&(xyz[...,1]>-0.20)&(xyz[...,1]<0.12)&(xyz[...,2]>1.05)&(xyz[...,2]<1.35)
P=xyz[box&yellow]; print('yellow n',len(P))
print(' x',P[:,0].min().round(3),P[:,0].max().round(3),' y',P[:,1].min().round(3),P[:,1].max().round(3),' z',P[:,2].min().round(3),P[:,2].max().round(3))
# per x-column max z (profile of top surface)
for x0 in np.arange(-0.16,-0.04,0.01):
    s=P[(P[:,0]>=x0)&(P[:,0]<x0+0.01)]
    if len(s): print(f'x {x0:.2f}: n={len(s)} zmax={s[:,2].max():.3f} zmed={np.median(s[:,2]):.3f} y {s[:,1].min():.3f}..{s[:,1].max():.3f}')
# also non-yellow points in the box that are not robot: hard. print all box points x-range with z<1.2
Q=xyz[box]; Q=Q[Q[:,2]<1.20]; print('box pts below 1.20: n',len(Q),' x',Q[:,0].min().round(3),Q[:,0].max().round(3),' y',Q[:,1].min().round(3),Q[:,1].max().round(3))
"

# openrua op 45
cat > place1.py <<'EOF'
import numpy as np, sys
from arm import *
a = Arm("place1")
q0 = a.arm_q(); p0, R0 = a.tcp(q0)
print("now tcp", np.round(p0,4)); print("R0\n", np.round(R0,3))
# cup axis offset from TCP (measured): (+0.054, 0, -0.031)
OFF = np.array([0.054, 0.0, -0.031])
def tcp_for_axis(ax): return np.asarray(ax) - OFF
targets = {
 "via":  tcp_for_axis([-0.29, -0.11, 1.22]),
 "high": tcp_for_axis([-0.422, -0.176, 1.20]),
 "low":  tcp_for_axis([-0.422, -0.176, 1.105]),
}
qs = {}
seed = q0
for k, t in targets.items():
    q = a.ik(t, R0, seed=seed)
    if q is None: print(k, "IK FAIL"); sys.exit(1)
    p, R = a.tcp(q)
    print(k, "tcp", np.round(t,3), "fk", np.round(p,3), "Rerr", np.abs(R-R0).max().round(3), "jump", np.round(np.abs(q-seed).max(),3), "q", np.round(q,3))
    qs[k] = q; seed = q
np.save("place_qs.npy", np.array([qs["via"], qs["high"], qs["low"]]))
EOF
timeout 300 python3 place1.py

# openrua op 46
cat > place2.py <<'EOF'
import numpy as np
from arm import *
a = Arm("place2")
qs = np.load("place_qs.npy")
code, err = a.move_q(qs[1], seconds=6.0, via=[qs[0]])
if code != 0 or err > 0.02:
    code, err = a.move_q(qs[1], seconds=3.0)
p, R = a.tcp(); print("tcp", np.round(p,4)); print("fingers", a.finger_gap())
EOF
timeout 300 python3 place2.py && timeout 60 ./tools/perception/cam_snap.py sideview sideview.png >/dev/null && timeout 60 ./tools/perception/cam_snap.py frontview frontview.png >/dev/null && echo ok

# openrua op 47
python3 -c "
import cv2
im=cv2.imread('sideview.png'); cv2.imwrite('crop_side.png', cv2.resize(im[100:400, 150:500], None, fx=2, fy=2))
im=cv2.imread('frontview.png'); cv2.imwrite('crop_front.png', cv2.resize(im[80:400, 200:560], None, fx=2, fy=2))
"

# openrua op 48
timeout 120 python3 scene3d.py sideview >/dev/null && timeout 120 python3 scene3d.py frontview >/dev/null && python3 -c "
import numpy as np, cv2
for cam in ['sideview','frontview']:
    xyz=np.load(cam+'_xyz.npy'); im=cv2.imread(cam+'.png')
    b,g,r=[im[...,i].astype(int) for i in range(3)]
    yellow=(r>120)&(g>100)&(b<90)&(r-b>60)
    box=(xyz[...,0]>-0.60)&(xyz[...,0]<-0.30)&(xyz[...,1]>-0.30)&(xyz[...,1]<-0.02)&(xyz[...,2]>1.06)&(xyz[...,2]<1.35)
    P=xyz[box&yellow]; print(cam,'yellow n',len(P))
    if len(P): print(' x',P[:,0].min().round(3),P[:,0].max().round(3),' y',P[:,1].min().round(3),P[:,1].max().round(3),' z',P[:,2].min().round(3),P[:,2].max().round(3))
    Q=xyz[box]; print(' all box pts n',len(Q),' x',Q[:,0].min().round(3),Q[:,0].max().round(3),' y',Q[:,1].min().round(3),Q[:,1].max().round(3),' z',Q[:,2].min().round(3),Q[:,2].max().round(3))
    # lowest points of anything in box (cup underside / fingers)
    lo=Q[Q[:,2]<Q[:,2].min()+0.01]; print(' lowest pts mean',np.round(lo.mean(0),3), 'n',len(lo))
"

# openrua op 49
cat > place3.py <<'EOF'
import numpy as np
from arm import *
a = Arm("place3")
qs = np.load("place_qs.npy")
code, err = a.move_q(qs[2], seconds=5.0)
if code != 0 or err > 0.02:
    code, err = a.move_q(qs[2], seconds=3.0)
p, R = a.tcp(); print("tcp", np.round(p,4)); print("fingers", a.finger_gap()); print("wrench", np.round(a.wrench(),2))
EOF
timeout 300 python3 place3.py && timeout 120 python3 scene3d.py sideview >/dev/null && timeout 60 ./tools/perception/cam_snap.py sideview sideview.png >/dev/null && python3 -c "
import numpy as np, cv2
xyz=np.load('sideview_xyz.npy'); im=cv2.imread('sideview.png')
b,g,r=[im[...,i].astype(int) for i in range(3)]
yellow=(r>120)&(g>100)&(b<90)&(r-b>60)
box=(xyz[...,0]>-0.60)&(xyz[...,0]<-0.30)&(xyz[...,1]>-0.30)&(xyz[...,1]<-0.02)&(xyz[...,2]>1.0)&(xyz[...,2]<1.35)
P=xyz[box&yellow]; print('yellow n',len(P),' x',P[:,0].min().round(3),P[:,0].max().round(3),' y',P[:,1].min().round(3),P[:,1].max().round(3),' z',P[:,2].min().round(3),P[:,2].max().round(3))
Q=xyz[box]; lo=Q[Q[:,2]<Q[:,2].min()+0.01]; print('lowest pts mean',np.round(lo.mean(0),3),'zmin',Q[:,2].min().round(3))
"

# openrua op 50
cat > place4.py <<'EOF'
import numpy as np
from arm import *
a = Arm("place4")
print("open ->", a.gripper(0.04))
print("wrench", np.round(a.wrench(),2))
EOF
timeout 300 python3 place4.py && timeout 60 ./tools/perception/cam_snap.py sideview sideview.png >/dev/null && timeout 60 ./tools/perception/cam_snap.py frontview frontview.png >/dev/null && python3 -c "
import cv2
im=cv2.imread('sideview.png'); cv2.imwrite('crop_side.png', cv2.resize(im[100:400, 300:640], None, fx=2, fy=2))
im=cv2.imread('frontview.png'); cv2.imwrite('crop_front.png', cv2.resize(im[80:400, 150:500], None, fx=2, fy=2))
"

# openrua op 51
timeout 120 python3 scene3d.py sideview >/dev/null && timeout 120 python3 scene3d.py frontview >/dev/null && python3 -c "
import numpy as np, cv2
for cam in ['sideview','frontview']:
    xyz=np.load(cam+'_xyz.npy'); im=cv2.imread(cam+'.png')
    b,g,r=[im[...,i].astype(int) for i in range(3)]
    yellow=(r>120)&(g>100)&(b<90)&(r-b>60)
    box=(xyz[...,0]>-0.60)&(xyz[...,0]<-0.30)&(xyz[...,1]>-0.30)&(xyz[...,1]<-0.02)&(xyz[...,2]>0.9)&(xyz[...,2]<1.35)
    P=xyz[box&yellow]; print(cam,'yellow n',len(P),' x',P[:,0].min().round(3),P[:,0].max().round(3),' y',P[:,1].min().round(3),P[:,1].max().round(3),' z',P[:,2].min().round(3),P[:,2].max().round(3))
    for z0 in np.arange(1.0,1.16,0.02):
        s=P[(P[:,2]>=z0)&(P[:,2]<z0+0.02)]
        if len(s): print(f'  z {z0:.2f}: n={len(s)} x {s[:,0].min():.3f}..{s[:,0].max():.3f} y {s[:,1].min():.3f}..{s[:,1].max():.3f}')
"

# openrua op 52
cat > place5.py <<'EOF'
import numpy as np
from arm import *
a = Arm("place5")
q0 = a.arm_q(); p0, R0 = a.tcp(q0)
t = p0 + np.array([0, -0.10, 0.0])
q = a.ik(t, R0, seed=q0); print("jump", np.abs(q-q0).max().round(3))
code, err = a.move_q(q, seconds=4.0)
if code != 0 or err > 0.02: a.move_q(q, seconds=3.0)
p, R = a.tcp(); print("tcp", np.round(p,4))
EOF
timeout 300 python3 place5.py && timeout 120 python3 scene3d.py sideview >/dev/null && timeout 120 python3 scene3d.py frontview >/dev/null && timeout 60 ./tools/perception/cam_snap.py agentview agentview.png >/dev/null && python3 -c "
import numpy as np, cv2
for cam in ['sideview','frontview']:
    xyz=np.load(cam+'_xyz.npy'); im=cv2.imread(cam+'.png')
    b,g,r=[im[...,i].astype(int) for i in range(3)]
    yellow=(r>120)&(g>100)&(b<90)&(r-b>60)
    box=(xyz[...,0]>-0.60)&(xyz[...,0]<-0.30)&(xyz[...,1]>-0.30)&(xyz[...,1]<-0.02)&(xyz[...,2]>0.9)&(xyz[...,2]<1.35)
    P=xyz[box&yellow]; print(cam,'yellow n',len(P),' x',P[:,0].min().round(3),P[:,0].max().round(3),' y',P[:,1].min().round(3),P[:,1].max().round(3),' z',P[:,2].min().round(3),P[:,2].max().round(3))
    for z0 in np.arange(1.0,1.16,0.02):
        s=P[(P[:,2]>=z0)&(P[:,2]<z0+0.02)]
        if len(s): print(f'  z {z0:.2f}: n={len(s)} x {s[:,0].min():.3f}..{s[:,0].max():.3f} y {s[:,1].min():.3f}..{s[:,1].max():.3f}')
im=cv2.imread('sideview.png'); cv2.imwrite('crop_side.png', cv2.resize(im[150:380, 330:600], None, fx=3, fy=3))
im=cv2.imread('agentview.png'); cv2.imwrite('crop_agent.png', cv2.resize(im[100:400, 100:450], None, fx=2, fy=2))
"

# openrua op 53
python3 -c "
import numpy as np, cv2
xyz=np.load('frontview_xyz.npy'); im=cv2.imread('frontview.png')
b,g,r=[im[...,i].astype(int) for i in range(3)]
yellow=(r>120)&(g>100)&(b<90)&(r-b>60)
slab=(xyz[...,1]>-0.17)&(xyz[...,1]<-0.12)&(xyz[...,0]>-0.60)&(xyz[...,0]<-0.30)&(xyz[...,2]>0.85)&(xyz[...,2]<1.2)
P=xyz[slab]; Y=yellow[slab]
print('slab pts',len(P),'yellow',Y.sum())
# profile: for each x bin, max z of yellow and of nonyellow
for x0 in np.arange(-0.50,-0.34,0.01):
    m=(P[:,0]>=x0)&(P[:,0]<x0+0.01)
    sy=P[m&Y]; sn=P[m&~Y]
    print(f'x {x0:.2f}: yellow n={len(sy):3d} z {sy[:,2].min() if len(sy) else 0:.3f}..{sy[:,2].max() if len(sy) else 0:.3f} | other n={len(sn):3d} z {sn[:,2].min() if len(sn) else 0:.3f}..{sn[:,2].max() if len(sn) else 0:.3f}')
"; 

# openrua op 54
cat > place6.py <<'EOF'
import numpy as np
from arm import *
a = Arm("place6")
q0 = a.arm_q(); p0, R0 = a.tcp(q0)
t = p0 + np.array([0.05, -0.05, 0.15])
q = a.ik(t, R0, seed=q0); print("jump", np.abs(q-q0).max().round(3))
code, err = a.move_q(q, seconds=5.0)
if code != 0 or err > 0.02: a.move_q(q, seconds=3.0)
p, R = a.tcp(); print("tcp", np.round(p,4)); print("fingers", a.finger_gap())
EOF
timeout 300 python3 place6.py && timeout 120 python3 scene3d.py birdview >/dev/null && timeout 60 ./tools/perception/cam_snap.py birdview birdview.png >/dev/null && timeout 60 ./tools/perception/cam_snap.py agentview agentview.png >/dev/null && timeout 60 ./tools/perception/cam_snap.py frontview frontview.png >/dev/null && python3 -c "
import numpy as np, cv2
xyz=np.load('birdview_xyz.npy'); im=cv2.imread('birdview.png')
b,g,r=[im[...,i].astype(int) for i in range(3)]
yellow=(r>120)&(g>100)&(b<90)&(r-b>60)
box=(xyz[...,0]>-0.60)&(xyz[...,0]<-0.30)&(xyz[...,1]>-0.30)&(xyz[...,1]<-0.02)&(xyz[...,2]>0.9)&(xyz[...,2]<1.35)
P=xyz[box&yellow]; print('bird yellow n',len(P),' x',P[:,0].min().round(3),P[:,0].max().round(3),' y',P[:,1].min().round(3),P[:,1].max().round(3),' z',P[:,2].min().round(3),P[:,2].max().round(3))
# top ridge: per x bin max z
for x0 in np.arange(-0.49,-0.36,0.01):
    s=P[(P[:,0]>=x0)&(P[:,0]<x0+0.01)]
    if len(s): print(f'x {x0:.2f}: n={len(s)} zmax={s[:,2].max():.3f} y {s[:,1].min():.3f}..{s[:,1].max():.3f}')
# everything in the slot region above floor (x -0.47..-0.41, y -0.205..-0.074)
S=xyz[(xyz[...,0]>-0.471)&(xyz[...,0]<-0.412)&(xyz[...,1]>-0.20)&(xyz[...,1]<-0.08)]
print('slot region visible z range', np.nanmin(S[:,2]).round(3), np.nanmax(S[:,2]).round(3), 'n', len(S))
im=cv2.imread('birdview.png'); cv2.imwrite('crop_bird.png', cv2.resize(im[180:330, 130:330], None, fx=3, fy=3))
im=cv2.imread('agentview.png'); cv2.imwrite('crop_agent.png', cv2.resize(im[100:400, 80:420], None, fx=2, fy=2))
"

# openrua op 55
python3 -c "
import cv2
im=cv2.imread('birdview.png'); cv2.imwrite('crop_bird.png', cv2.resize(im[120:260, 150:340], None, fx=3, fy=3))
"

# openrua op 56
timeout 120 python3 scene3d.py frontview >/dev/null && python3 -c "
import numpy as np, cv2
xyz=np.load('frontview_xyz.npy'); im=cv2.imread('frontview.png')
b,g,r=[im[...,i].astype(int) for i in range(3)]
yellow=(r>120)&(g>100)&(b<90)&(r-b>60)
box=(xyz[...,0]>-0.60)&(xyz[...,0]<-0.30)&(xyz[...,1]>-0.30)&(xyz[...,1]<-0.02)&(xyz[...,2]>0.9)&(xyz[...,2]<1.35)
P=xyz[box&yellow]; print('front yellow n',len(P),' x',P[:,0].min().round(3),P[:,0].max().round(3),' y',P[:,1].min().round(3),P[:,1].max().round(3),' z',P[:,2].min().round(3),P[:,2].max().round(3))
"

# openrua op 57
python3 -c "
import cv2
im=cv2.imread('frontview.png'); cv2.imwrite('crop_front.png', cv2.resize(im[200:400, 150:450], None, fx=3, fy=3))
"

# openrua op 58
cat > away.py <<'EOF'
import numpy as np
from arm import *
a = Arm("away")
q0 = a.arm_q(); p0, R0 = a.tcp(q0)
t = np.array([-0.30, -0.40, 1.40])
q = a.ik(t, R0, seed=q0); print("jump", np.abs(q-q0).max().round(3))
code, err = a.move_q(q, seconds=5.0)
if code != 0 or err > 0.02: a.move_q(q, seconds=3.0)
p, R = a.tcp(); print("tcp", np.round(p,4))
EOF
timeout 300 python3 away.py && for c in birdview frontview agentview; do timeout 120 python3 scene3d.py $c >/dev/null; timeout 60 ./tools/perception/cam_snap.py $c $c.png >/dev/null; done; python3 -c "
import numpy as np, cv2
for cam in ['birdview','frontview','agentview']:
    xyz=np.load(cam+'_xyz.npy'); im=cv2.imread(cam+'.png')
    b,g,r=[im[...,i].astype(int) for i in range(3)]
    yellow=(r>120)&(g>100)&(b<90)&(r-b>60)
    box=(xyz[...,0]>-0.60)&(xyz[...,0]<-0.30)&(xyz[...,1]>-0.30)&(xyz[...,1]<-0.02)&(xyz[...,2]>0.9)&(xyz[...,2]<1.35)
    P=xyz[box&yellow]
    print(cam,'yellow n',len(P),' x',P[:,0].min().round(3),P[:,0].max().round(3),' y',P[:,1].min().round(3),P[:,1].max().round(3),' z',P[:,2].min().round(3),P[:,2].max().round(3))
    for x0 in np.arange(-0.47,-0.33,0.01):
        s=P[(P[:,0]>=x0)&(P[:,0]<x0+0.01)]
        if len(s): print(f'  x {x0:.2f}: n={len(s):3d} z {s[:,2].min():.3f}..{s[:,2].max():.3f} y {s[:,1].min():.3f}..{s[:,1].max():.3f}')
"

# openrua op 59
python3 -c "
import numpy as np, cv2
xyz=np.load('agentview_xyz.npy')
# find pixel region of cup: yellow pixels
im=cv2.imread('agentview.png'); b,g,r=[im[...,i].astype(int) for i in range(3)]
yellow=(r>120)&(g>100)&(b<90)&(r-b>60)
ys,xs=np.where(yellow); print('yellow px rows',ys.min(),ys.max(),'cols',xs.min(),xs.max())
r0,r1,c0,c1=max(ys.min()-80,0),min(ys.max()+80,480),max(xs.min()-120,0),min(xs.max()+120,640)
cv2.imwrite('crop_agent.png', cv2.resize(im[r0:r1,c0:c1], None, fx=3, fy=3))
print(r0,r1,c0,c1)
"

# openrua op 60
timeout 120 python3 scene3d.py birdview >/dev/null; python3 -c "
import numpy as np
from arm import *
a=Arm('m'); q=a.arm_q(); p,R=a.fk(q); t,_=a.tcp(q)
print('hand origin',np.round(p,3),'tcp',np.round(t,3)); print('R\n',np.round(R,3))
xyz=np.load('birdview_xyz.npy')
# points near the hand: within 0.15 of the hand origin
P=xyz.reshape(-1,3); P=P[np.isfinite(P[:,0])]
d=P-p; m=np.linalg.norm(d,axis=1)<0.16
Q=d[m]
# express in hand frame
L=Q@R
print('n',len(Q))
for k,name in enumerate('xyz'):
    print(f'hand {name}: {L[:,k].min():.3f} .. {L[:,k].max():.3f}')
# points along hand z between 0 and 0.066 (body): y extent
body=L[(L[:,2]>0.0)&(L[:,2]<0.07)]; print('body y-extent',body[:,1].min().round(3),body[:,1].max().round(3),'x-extent',body[:,0].min().round(3),body[:,0].max().round(3))
fing=L[(L[:,2]>0.07)&(L[:,2]<0.13)]; print('finger zone y-extent',fing[:,1].min().round(3),fing[:,1].max().round(3),'x',fing[:,0].min().round(3),fing[:,0].max().round(3), 'zmax', fing[:,2].max().round(3))
"

# openrua op 61
python3 -c "
import numpy as np
xyz=np.load('birdview_xyz.npy'); P=xyz.reshape(-1,3); P=P[np.isfinite(P[:,0])]
def sl(xr,yr,zmin=0.9):
    m=(P[:,0]>xr[0])&(P[:,0]<xr[1])&(P[:,1]>yr[0])&(P[:,1]<yr[1])&(P[:,2]>zmin); return P[m]
# left/middle divider: scan y bins around -0.21
for y0 in np.arange(-0.23,-0.19,0.005):
    s=sl((-0.47,-0.36),(y0,y0+0.005))
    if len(s): print(f'y {y0:.3f}: n={len(s)} zmax={s[:,2].max():.3f} z90={np.percentile(s[:,2],90):.3f}')
print('--- cup top ridge (x bins), y in [-0.19,-0.10]')
for x0 in np.arange(-0.47,-0.33,0.01):
    s=sl((x0,x0+0.01),(-0.19,-0.10))
    if len(s): print(f'x {x0:.2f}: n={len(s)} zmax={s[:,2].max():.3f}')
print('--- back wall top (y -0.19..-0.10) x bins fine')
for x0 in np.arange(-0.49,-0.46,0.005):
    s=sl((x0,x0+0.005),(-0.19,-0.10))
    if len(s): print(f'x {x0:.3f}: n={len(s)} zmax={s[:,2].max():.3f}')
"

# openrua op 62
cat > away2.py <<'EOF'
import numpy as np
from arm import *
a = Arm("away2")
q0 = a.arm_q()
for t in ([-0.25, -0.30, 1.30], [-0.30, -0.25, 1.30], [-0.20, -0.20, 1.25]):
    q = a.ik(t, R_DOWN_FX, seed=q0)
    if q is not None:
        print("target", t, "jump", np.abs(q-q0).max().round(3)); break
code, err = a.move_q(q, seconds=6.0)
if code != 0 or err > 0.02: a.move_q(q, seconds=3.0)
p, R = a.tcp(); print("tcp", np.round(p,4), "fingers", a.finger_gap())
EOF
timeout 300 python3 away2.py && timeout 120 python3 scene3d.py birdview >/dev/null && python3 -c "
import numpy as np
xyz=np.load('birdview_xyz.npy'); P=xyz.reshape(-1,3); P=P[np.isfinite(P[:,0])]
def sl(xr,yr,zmin=0.9):
    m=(P[:,0]>xr[0])&(P[:,0]<xr[1])&(P[:,1]>yr[0])&(P[:,1]<yr[1])&(P[:,2]>zmin); return P[m]
print('cup top ridge (x bins), y in [-0.19,-0.10]')
for x0 in np.arange(-0.47,-0.33,0.01):
    s=sl((x0,x0+0.01),(-0.19,-0.10))
    if len(s): print(f'x {x0:.2f}: n={len(s)} zmax={s[:,2].max():.3f}')
print('cup y extent at z>1.06:'); s=sl((-0.46,-0.39),(-0.25,-0.05),1.06); print(s[:,1].min().round(3), s[:,1].max().round(3))
"

# openrua op 63
python3 -c "
import numpy as np
xyz=np.load('birdview_xyz.npy'); P=xyz.reshape(-1,3); P=P[np.isfinite(P[:,0])]
def sl(xr,yr,zmin=0.9,zmax=1.2):
    m=(P[:,0]>xr[0])&(P[:,0]<xr[1])&(P[:,1]>yr[0])&(P[:,1]<yr[1])&(P[:,2]>zmin)&(P[:,2]<zmax); return P[m]
print('cup top ridge (x bins), y in [-0.19,-0.10], z<1.2')
for x0 in np.arange(-0.47,-0.33,0.01):
    s=sl((x0,x0+0.01),(-0.19,-0.10))
    if len(s): print(f'x {x0:.2f}: n={len(s)} zmax={s[:,2].max():.3f}')
s=sl((-0.46,-0.39),(-0.25,-0.05),1.06); print('cup y extent at z>1.06:', s[:,1].min().round(3), s[:,1].max().round(3), 'n', len(s))
"; timeout 60 ./tools/perception/cam_snap.py birdview birdview.png >/dev/null; timeout 60 ./tools/perception/cam_snap.py agentview agentview.png >/dev/null; python3 -c "
import cv2
im=cv2.imread('birdview.png'); cv2.imwrite('crop_bird.png', cv2.resize(im[120:260, 150:340], None, fx=3, fy=3))
im=cv2.imread('agentview.png'); cv2.imwrite('crop_agent.png', cv2.resize(im[120:300, 80:400], None, fx=2, fy=2))
"

# openrua op 64
cat > away3.py <<'EOF'
import numpy as np
from arm import *
a = Arm("away3")
q0 = a.arm_q()
for t in ([-0.15, 0.25, 1.30], [-0.10, 0.20, 1.30], [-0.20, 0.15, 1.30]):
    q = a.ik(t, R_DOWN_FX, seed=q0)
    if q is not None:
        print("target", t, "jump", np.abs(q-q0).max().round(3)); break
code, err = a.move_q(q, seconds=6.0)
if code != 0 or err > 0.02: a.move_q(q, seconds=3.0)
p, R = a.tcp(); print("tcp", np.round(p,4), "fingers", a.finger_gap())
EOF
timeout 300 python3 away3.py && for c in birdview agentview frontview; do timeout 60 ./tools/perception/cam_snap.py $c $c.png >/dev/null; done; timeout 120 python3 scene3d.py birdview >/dev/null; python3 -c "
import numpy as np, cv2
xyz=np.load('birdview_xyz.npy'); P=xyz.reshape(-1,3); P=P[np.isfinite(P[:,0])]
def sl(xr,yr,zmin=0.9,zmax=1.2):
    m=(P[:,0]>xr[0])&(P[:,0]<xr[1])&(P[:,1]>yr[0])&(P[:,1]<yr[1])&(P[:,2]>zmin)&(P[:,2]<zmax); return P[m]
for x0 in np.arange(-0.47,-0.33,0.01):
    s=sl((x0,x0+0.01),(-0.19,-0.10))
    if len(s): print(f'x {x0:.2f}: n={len(s)} zmax={s[:,2].max():.3f}')
s=sl((-0.46,-0.39),(-0.25,-0.05),1.06); print('cup y extent at z>1.06:', s[:,1].min().round(3), s[:,1].max().round(3))
im=cv2.imread('birdview.png'); cv2.imwrite('crop_bird.png', cv2.resize(im[120:260, 150:340], None, fx=3, fy=3))
im=cv2.imread('agentview.png'); cv2.imwrite('crop_agent.png', cv2.resize(im[120:300, 80:400], None, fx=2, fy=2))
"
