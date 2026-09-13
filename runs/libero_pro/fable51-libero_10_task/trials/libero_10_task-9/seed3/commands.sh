#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
cat tools/action/fjt_send.py; echo ----; cat tools/action/gripper_cmd.py; echo ----; cat tools/action/ik_move.py

# openrua op 3
ros2 topic list; echo ----; ros2 action list; echo ----; ros2 service list | grep -v -E "parameter|describe|get_type" ; echo ----; ros2 node list

# openrua op 4
for c in agentview frontview sideview birdview robot0_eye_in_hand robot0_robotview; do timeout 60 python3 tools/perception/cam_snap.py $c; done; ros2 topic echo /joint_states --once

# openrua op 5
python3 -c "
import cv2
im=cv2.imread('agentview.png'); cv2.imwrite('crop_mugs.png', cv2.resize(im[170:320, 80:380], None, fx=3, fy=3, interpolation=cv2.INTER_CUBIC))
"; ros2 node info /robot_bridge 2>&1 | head -60

# openrua op 6
python3 -c "
import cv2, numpy as np
im=cv2.imread('agentview.png')
# left mug body region approx (x 110-175, y 200-265); right mug body (x 300-360, y 235-300)
for name,(x0,x1,y0,y1) in {'left':(115,170,205,260),'right':(305,355,240,295)}.items():
    r=im[y0:y1,x0:x1].reshape(-1,3).astype(float)
    print(name,'mean BGR',r.mean(0).round(), 'max',r.max(0), 'p90', np.percentile(r,90,axis=0).round())
"


# openrua op 7
mkdir -p "$(dirname /workspace/cloud.py)"
cat > /workspace/cloud.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Dump a camera's full depth frame as a world-frame point cloud.

Usage: python3 cloud.py <camera>   -> <camera>_xyz.npy (H,W,3 world coords)
"""
import sys
import numpy as np
import rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge


def grab(node, topic, msg_type, timeout=30.0):
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
    rclpy.init()
    node = rclpy.create_node("cloud")
    tfbuf = Buffer()
    TransformListener(tfbuf, node)
    depth_msg = grab(node, f"/{cam}/depth/image_raw", Image)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    depth = CvBridge().imgmsg_to_cv2(depth_msg, desired_encoding="passthrough").astype(np.float64)
    H, W = depth.shape
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    frame = f"{cam}_optical_frame"
    import time
    end = time.time() + 15
    while time.time() < end and not tfbuf.can_transform("world", frame, rclpy.time.Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    R = quat_R(q.x, q.y, q.z, q.w)
    T = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    X = (u - cx) * depth / fx
    Y = (v - cy) * depth / fy
    P = np.stack([X, Y, depth], -1)
    Pw = P @ R.T + T
    Pw[~np.isfinite(depth) | (depth <= 0)] = np.nan
    np.save(f"{cam}_xyz.npy", Pw)
    print(f"{cam}_xyz.npy", Pw.shape, "cam pos", T.round(3))
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 8
for c in birdview agentview frontview sideview; do timeout 120 python3 cloud.py $c; done; ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -8

# openrua op 9
python3 -c "
import numpy as np
P=np.load('birdview_xyz.npy')
def q(u,v): print((u,v), np.round(P[v,u],3))
# left mug center, right mug center, microwave body corners, door, table
for uv in [(233,275),(322,298),(233,240),(150,300),(430,300),(400,300),(460,300),(430,250),(430,345),(360,243),(340,243),(392,330),(300,400)]:
    q(*uv)
"


# openrua op 10
python3 -c "
import cv2
im=cv2.imread('frontview.png'); cv2.imwrite('crop_mw.png', cv2.resize(im[240:450, 300:640], None, fx=2.5, fy=2.5, interpolation=cv2.INTER_CUBIC))
im=cv2.imread('agentview.png'); cv2.imwrite('crop_mw2.png', cv2.resize(im[140:400, 300:640], None, fx=2.5, fy=2.5, interpolation=cv2.INTER_CUBIC))
"

# openrua op 11
python3 -c "
import numpy as np
P=np.load('frontview_xyz.npy')
def q(u,v): print((u,v), np.round(P[v,u],3))
for uv in [(340,270),(340,330),(440,270),(440,330),(380,300),(400,300),(460,290),(470,350),(500,350),(560,300),(460,420)]:
    q(*uv)
print('---sideview')
P=np.load('sideview_xyz.npy')
for uv in [(390,240),(390,290),(200,300),(350,350),(280,330),(312,230),(280,260),(300,200)]:
    q(*uv)
"


# openrua op 12
python3 -c "
import numpy as np
P=np.load('agentview_xyz.npy')
np.set_printoptions(suppress=True, linewidth=200)
for v in range(160,310,10):
    row=[]
    for u in range(460,520,8):
        p=P[v,u]; row.append(f'{u}:({p[0]:.2f},{p[1]:.2f},{p[2]:.2f})')
    print(v,' '.join(row))
"


# openrua op 13
python3 -c "
import numpy as np
P=np.load('birdview_xyz.npy')
reg=P[240:320,190:280]
m=np.isfinite(reg[...,2])&(reg[...,2]>0.92)
pts=reg[m]
print('left mug pts',len(pts),'x',pts[:,0].min().round(3),pts[:,0].max().round(3),'y',pts[:,1].min().round(3),pts[:,1].max().round(3),'z',pts[:,2].min().round(3),pts[:,2].max().round(3))
# rim points only (z>0.97)
r=pts[pts[:,2]>0.97]
print('rim: x',r[:,0].min().round(3),r[:,0].max().round(3),'y',r[:,1].min().round(3),r[:,1].max().round(3),'center',r[:,:2].mean(0).round(3))
reg=P[270:330,290:360]
m=np.isfinite(reg[...,2])&(reg[...,2]>0.92)
pts=reg[m]
print('right mug pts',len(pts),'x',pts[:,0].min().round(3),pts[:,0].max().round(3),'y',pts[:,1].min().round(3),pts[:,1].max().round(3),'z',pts[:,2].min().round(3),pts[:,2].max().round(3))
r=pts[pts[:,2]>0.97]
print('rim: x',r[:,0].min().round(3),r[:,0].max().round(3),'y',r[:,1].min().round(3),r[:,1].max().round(3),'center',r[:,:2].mean(0).round(3))
"


# openrua op 14
python3 -c "
import numpy as np
P=np.load('birdview_xyz.npy')
reg=P[240:320,190:280]
z=reg[...,2]
print(np.histogram(z[np.isfinite(z)], bins=[0.89,0.91,0.93,0.95,0.97,0.975,0.98,0.985,0.99,1.0,1.05,1.1])) 
m=np.isfinite(z)&(z>0.975)&(z<0.99)
pts=reg[m]
print('rim band pts',len(pts),'x',pts[:,0].min().round(3),pts[:,0].max().round(3),'y',pts[:,1].min().round(3),pts[:,1].max().round(3))
m=np.isfinite(z)&(z>1.0)
pts=reg[m]
print('high pts',len(pts),'x',pts[:,0].min().round(3),pts[:,0].max().round(3),'y',pts[:,1].min().round(3),pts[:,1].max().round(3))
# print a coarse map of z
import sys
for v in range(245,315,3):
    print(''.join('#' if (np.isfinite(z[v-240,u-190]) and z[v-240,u-190]>1.0) else ('o' if (np.isfinite(z[v-240,u-190]) and z[v-240,u-190]>0.95) else '.') for u in range(190,280)))
"


# openrua op 15
python3 -c "
import numpy as np
P=np.load('frontview_xyz.npy')
np.set_printoptions(suppress=True, linewidth=200)
for v in range(280,350,5):
    row=[]
    for u in range(150,225,8):
        p=P[v,u]; row.append(f'{u}:({p[0]:.2f},{p[1]:.2f},{p[2]:.3f})')
    print(v,' '.join(row))
"


# openrua op 16
timeout 60 python3 -c "
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
rclpy.init(); n=rclpy.create_node('rd')
got=[]
qos=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
n.create_subscription(String,'/robot_description',got.append,qos)
while not got: rclpy.spin_once(n,timeout_sec=0.5)
open('robot.urdf','w').write(got[0].data); print(len(got[0].data))
"; grep -n -A12 '<link name="panda_hand"' robot.urdf | head -60; grep -n 'finger' robot.urdf | head; grep -n -B2 -A10 'name="panda_leftfinger"' robot.urdf | head -60

# openrua op 17
ls /opt/ros/jazzy/share/moveit_resources_panda_description/meshes/collision/ && python3 -c "
import numpy as np, struct
def stl_bounds(path):
    d=open(path,'rb').read()
    if d[:5]==b'solid':
        import re
        v=np.array([[float(x) for x in m.groups()] for m in re.finditer(r'vertex\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)',d.decode())])
    else:
        n=struct.unpack_from('<I',d,80)[0]
        arr=np.frombuffer(d[84:84+n*50],dtype=np.dtype([('n','<3f4'),('v','<9f4'),('a','<u2')]))
        v=arr['v'].reshape(-1,3)
    return v.min(0), v.max(0)
for f in ['hand.stl','finger.stl','link7.stl','link6.stl']:
    lo,hi=stl_bounds('/opt/ros/jazzy/share/moveit_resources_panda_description/meshes/collision/'+f)
    print(f,'min',lo.round(4),'max',hi.round(4),'size',(hi-lo).round(4))
"; grep -o '<joint name="panda_hand_joint"[^>]*>.\{0,300\}' robot.urdf; grep -o '<joint name="panda_finger_joint1"[^>]*>.\{0,400\}' robot.urdf

# openrua op 18
find / -name "hand*.stl" -o -name "hand*.dae" -o -name "finger*.stl" 2>/dev/null | head; 

# openrua op 19
timeout 300 python3 tools/action/fjt_send.py 0.0,-0.161037389,0.0,-2.44459747,0.0,2.2267522,0.7853981633974483 2.0; ros2 topic echo /clock --once | head -4

# openrua op 20
timeout 120 python3 cloud.py birdview; timeout 120 python3 cloud.py frontview; timeout 60 python3 tools/perception/cam_snap.py agentview; python3 -c "
import numpy as np
P=np.load('frontview_xyz.npy')
np.set_printoptions(suppress=True, linewidth=200)
reg=P[250:400,140:240]; z=reg[...,2]
m=np.isfinite(z)&(z>0.905)&(reg[...,0]>-0.3)&(reg[...,0]<0.1)
pts=reg[m]
print('left mug frontview: x',pts[:,0].min().round(3),pts[:,0].max().round(3),'y',pts[:,1].min().round(3),pts[:,1].max().round(3),'z',pts[:,2].min().round(3),pts[:,2].max().round(3))
B=np.load('birdview_xyz.npy')
reg=B[240:320,190:280]; z=reg[...,2]
m=np.isfinite(z)&(z>0.92); pts=reg[m]
print('left mug bird: x',pts[:,0].min().round(3),pts[:,0].max().round(3),'y',pts[:,1].min().round(3),pts[:,1].max().round(3),'z',pts[:,2].min().round(3),pts[:,2].max().round(3))
reg=B[270:330,290:360]; z=reg[...,2]
m=np.isfinite(z)&(z>0.92); pts=reg[m]
print('right mug bird: x',pts[:,0].min().round(3),pts[:,0].max().round(3),'y',pts[:,1].min().round(3),pts[:,1].max().round(3),'z',pts[:,2].min().round(3),pts[:,2].max().round(3))
"


# openrua op 21
timeout 120 python3 cloud.py agentview; python3 -c "
import numpy as np
P=np.load('agentview_xyz.npy')
np.set_printoptions(suppress=True, linewidth=250)
for u in [470,478,486,494]:
    print('col',u)
    print(' '.join(f'{v}:({P[v,u][0]:.2f},{P[v,u][1]:.2f},{P[v,u][2]:.3f})' for v in range(160,300,4)))
"


# openrua op 22
python3 -c "
import numpy as np
P=np.load('frontview_xyz.npy')
np.set_printoptions(suppress=True, linewidth=250)
for v in [270,285,300,315,330,345]:
    print('row',v,' '.join(f'{u}:({P[v,u][0]:.3f},{P[v,u][1]:.2f},{P[v,u][2]:.3f})' for u in range(440,486,3)))
"


# openrua op 23
python3 -c "
import numpy as np
P=np.load('frontview_xyz.npy')
reg=P[250:400,140:240].reshape(-1,3)
m=np.isfinite(reg[:,2])&(reg[:,2]>0.903)&(reg[:,0]>-0.3)&(reg[:,0]<0.1)&(reg[:,1]<-0.15)
pts=reg[m]
for z0 in np.arange(0.90,1.02,0.01):
    s=pts[(pts[:,2]>=z0)&(pts[:,2]<z0+0.01)]
    if len(s): print(f'z {z0:.2f}-{z0+0.01:.2f}: n={len(s)} x {s[:,0].min():.3f}..{s[:,0].max():.3f} width {s[:,0].max()-s[:,0].min():.3f}  y {s[:,1].min():.3f}..{s[:,1].max():.3f}')
"


# openrua op 24
find / \( -name "*.stl" -o -name "*.dae" -o -name "*.obj" -o -name "*.xml" \) 2>/dev/null | grep -i -E "panda|franka|hand|gripper|microwave|mug" | head -20; pip list 2>/dev/null | grep -i -E "mujoco|robosuite|robocasa|pybullet|isaac"

# openrua op 25
python3 -c "
import numpy as np
B=np.load('birdview_xyz.npy')
reg=B[200:270,270:370]; z=reg[...,2]
m=np.isfinite(z)&(z>1.0)&(z<1.4); pts=reg[m]
print('n',len(pts),'x',pts[:,0].min().round(3),pts[:,0].max().round(3),'y',pts[:,1].min().round(3),pts[:,1].max().round(3),'z',pts[:,2].min().round(3),pts[:,2].max().round(3))
for z0 in np.arange(1.0,1.4,0.02):
    s=pts[(pts[:,2]>=z0)&(pts[:,2]<z0+0.02)]
    if len(s): print(f'z {z0:.2f}: n={len(s)} x {s[:,0].min():.3f}..{s[:,0].max():.3f} y {s[:,1].min():.3f}..{s[:,1].max():.3f}')
"


# openrua op 26
timeout 60 python3 tools/perception/cam_snap.py galleryview; timeout 60 python3 tools/perception/cam_snap.py paperview; timeout 60 python3 tools/perception/cam_snap.py robot0_robotview

# openrua op 27
mkdir -p "$(dirname /workspace/rb.py)"
cat > /workspace/rb.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Robot helper: FK/IK (world frame), trajectory, gripper, wrench, snapshots.

Import and use inside a script (rclpy node built once per process).
World frame = panda_link0 + BASE offset.
"""
import time
import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import WrenchStamped, TwistStamped
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState, Image
from trajectory_msgs.msg import JointTrajectoryPoint
from scipy.spatial.transform import Rotation as Rot

BASE = np.array([-0.66, 0.0, 0.912])  # world -> panda_link0
M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
ARM = FJT["joints"]
LIMITS = FJT["limits_rad"]
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")


class Robot:
    def __init__(self, name="rb"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self._wr = None
        self.node.create_subscription(WrenchStamped, "/franka_robot_state_broadcaster/external_wrench", self._on_wr, 10)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.gr = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik_cli = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.twist_pub = self.node.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)

    def _on_js(self, m):
        self._js = m

    def _on_wr(self, m):
        self._wr = m

    def spin(self, t=0.1):
        end = time.time() + t
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    # ---------- sensing ----------
    def js(self, fresh=True):
        if fresh:
            self._js = None
        while self._js is None:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        return dict(zip(self._js.name, self._js.position))

    def arm_q(self):
        j = self.js()
        return np.array([j[n] for n in ARM])

    def fingers(self):
        j = self.js()
        return j["panda_finger_joint1"], j["panda_finger_joint2"]

    def wrench(self):
        self._wr = None
        end = time.time() + 5
        while self._wr is None and time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        if self._wr is None:
            return None
        f, t = self._wr.wrench.force, self._wr.wrench.torque
        return np.array([f.x, f.y, f.z, t.x, t.y, t.z])

    def snap(self, cam, out=None):
        from cv_bridge import CvBridge
        import cv2
        topic = f"/{cam}/color/image_raw"
        got = []
        sub = self.node.create_subscription(Image, topic, lambda m: got.append(m), 1)
        while not got:
            rclpy.spin_once(self.node, timeout_sec=0.5)
        self.node.destroy_subscription(sub)
        out = out or f"{cam}.png"
        cv2.imwrite(out, CvBridge().imgmsg_to_cv2(got[0], desired_encoding="bgr8"))
        return out

    # ---------- kinematics ----------
    def fk(self, q=None, link="panda_hand"):
        """Return (pos_world, R_world) of link for arm config q (default current)."""
        if q is None:
            q = self.arm_q()
        req = GetPositionFK.Request()
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = [float(v) for v in q]
        req.header.frame_id = ""
        self.fk_cli.wait_for_service(timeout_sec=10)
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {None if res is None else res.error_code.val}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE
        R = Rot.from_quat([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]).as_matrix()
        return pos, R

    def ik(self, pos_world, R, seed=None, attempts=3):
        """IK for panda_hand at world pos with rotation matrix R. Returns q or None."""
        if seed is None:
            seed = self.arm_q()
        pb = np.asarray(pos_world) - BASE
        qt = Rot.from_matrix(R).as_quat()
        for _ in range(attempts):
            req = GetPositionIK.Request()
            req.ik_request.group_name = "panda_arm"
            req.ik_request.ik_link_name = "panda_hand"
            req.ik_request.pose_stamped.header.frame_id = ""
            p = req.ik_request.pose_stamped.pose
            p.position.x, p.position.y, p.position.z = map(float, pb)
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, qt)
            req.ik_request.robot_state.joint_state.name = list(ARM)
            req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
            req.ik_request.avoid_collisions = False
            req.ik_request.timeout = Duration(sec=0, nanosec=200_000_000)
            self.ik_cli.wait_for_service(timeout_sec=10)
            fut = self.ik_cli.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            res = fut.result()
            if res is not None and res.error_code.val == 1:
                sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
                q = np.array([sol[n] for n in ARM])
                # verify
                fp, fR = self.fk(q)
                err = np.linalg.norm(fp - pos_world)
                if err < 0.005:
                    return q
            seed = np.array(seed) + np.random.uniform(-0.2, 0.2, len(seed))
        return None

    # ---------- action ----------
    def move(self, q, sec=3.0, extra_points=None):
        """Send trajectory to q (list of arm positions). extra_points: list of (q, t) before final."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pts = []
        if extra_points:
            for qq, t in extra_points:
                pt = JointTrajectoryPoint(positions=[float(v) for v in qq])
                pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
                pts.append(pt)
        pt = JointTrajectoryPoint(positions=[float(v) for v in q])
        pt.time_from_start = Duration(sec=int(sec), nanosec=int((sec % 1) * 1e9))
        pts.append(pt)
        goal.trajectory.points = pts
        self.fjt.wait_for_server(timeout_sec=10)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res)
        code = res.result().result.error_code
        qa = self.arm_q()
        err = np.abs(qa - np.array(q)).max()
        return code, err

    def move_pose(self, pos_world, R, sec=3.0, seed=None):
        q = self.ik(pos_world, R, seed=seed)
        if q is None:
            raise RuntimeError(f"IK failed for {np.round(pos_world,3)}")
        code, err = self.move(q, sec)
        p, _ = self.fk()
        return code, err, p

    def grip(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP.get("max_effort", 30.0))
        self.gr.wait_for_server(timeout_sec=10)
        fut = self.gr.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=120)
        r = rf.result().result
        return r.reached_goal, r.stalled, self.fingers()

    def servo(self, lin, ang=(0, 0, 0), n=20, frame="panda_link0"):
        msg = TwistStamped()
        msg.header.frame_id = frame
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, lin)
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = map(float, ang)
        for _ in range(n):
            self.twist_pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.05)


def R_from(approach, closing):
    """Rotation matrix for panda_hand: z=approach, y=closing (both world vectors)."""
    z = np.asarray(approach, float); z /= np.linalg.norm(z)
    y = np.asarray(closing, float); y -= y.dot(z) * z; y /= np.linalg.norm(y)
    x = np.cross(y, z)
    return np.column_stack([x, y, z])
OPENRUA_EOF

# openrua op 28
timeout 120 python3 -c "
from rb import *
r=Robot()
print('q',r.arm_q().round(3))
p,R=r.fk(); print('hand world',p.round(3)); print(R.round(3))
p8,_=r.fk(link='panda_link8'); print('link8',p8.round(3))
print('fingers',r.fingers()); print('wrench',r.wrench())
"

# openrua op 29
python3 -c "
import numpy as np
def T(a,d,al,th):
    ca,sa,ct,st=np.cos(al),np.sin(al),np.cos(th),np.sin(th)
    return np.array([[ct,-st,0,a],[st*ca,ct*ca,-sa,-d*sa],[st*sa,ct*sa,ca,d*ca],[0,0,0,1]])
q=[0,-0.161,0,-2.445,0,2.227,0.785]
a=[0,0,0,0.0825,-0.0825,0,0.088,0]; d=[0.333,0,0.316,0,0.384,0,0,0.107]; al=[0,-np.pi/2,np.pi/2,np.pi/2,-np.pi/2,np.pi/2,np.pi/2,0]
M=np.eye(4)
for i in range(8):
    M=M@T(a[i],d[i],al[i],q[i] if i<7 else 0)
print('link8 in link0:',M[:3,3].round(3)); print(M[:3,:3].round(3))
"


# openrua op 30
sed -i 's/^BASE = np.array(\[-0.66, 0.0, 0.912\]).*/BASE = np.array([0.0, 0.0, 0.0])  # MoveIt model frame == world (verified by FK)/' rb.py && grep -n "^BASE" rb.py && timeout 120 python3 -c "
from rb import *
r=Robot()
p,R=r.fk(); print('hand world',p.round(3))
q=r.ik(p+np.array([0,0,-0.05]),R); print('ik',None if q is None else q.round(3))
if q is not None: print(r.fk(q)[0].round(3))
"

# openrua op 31
timeout 30 ros2 run tf2_ros tf2_echo panda_hand robot0_eye_in_hand_optical_frame 2>&1 | grep -A3 "At time" | head -8; ros2 topic echo /tf_static --once 2>/dev/null | grep -E "frame_id|child" | head -40

# openrua op 32
timeout 300 python3 -c "
from rb import *
r=Robot()
R=R_from([0,np.cos(np.radians(15)),-np.sin(np.radians(15))],[1,0,0])
print(R.round(3))
code,err,p=r.move_pose(np.array([-0.05,-0.163,1.052]),R,sec=4)
print('code',code,'err',err.round(4),'hand at',p.round(3))
print(r.snap('robot0_eye_in_hand','eih1.png'))
"

# openrua op 33
timeout 600 python3 -c "
from rb import *
r=Robot()
R=R_from([0,np.cos(np.radians(15)),-np.sin(np.radians(15))],[1,0,0])
q=r.ik(np.array([-0.05,-0.163,1.052]),R)
print('target q',q.round(3)); print('current',r.arm_q().round(3))
code,err=r.move(q,sec=6)
print('code',code,'err',err.round(4),'hand at',r.fk()[0].round(3))
if err>0.05:
    code,err=r.move(q,sec=6); print('retry code',code,'err',err.round(4),'hand at',r.fk()[0].round(3))
print(r.snap('robot0_eye_in_hand','eih1.png'))
"

# openrua op 34
mkdir -p "$(dirname /workspace/eih_cloud.py)"
cat > /workspace/eih_cloud.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Eye-in-hand depth -> world point cloud using FK for the camera pose.
Usage: python3 eih_cloud.py [out_prefix]
"""
import sys
import numpy as np
import rclpy
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation as Rot
from rb import Robot

CAM = "robot0_eye_in_hand"
# camera optical frame relative to panda_hand (from tf_static)
T_HC = np.eye(4)
T_HC[:3, 3] = [0.050, 0.0, -0.001]
T_HC[:3, :3] = Rot.from_quat([0, 0, 0.707, 0.707]).as_matrix()


def grab(r, topic, msg_type):
    got = []
    sub = r.node.create_subscription(msg_type, topic, lambda m: got.append(m), 1)
    while not got:
        rclpy.spin_once(r.node, timeout_sec=0.3)
    r.node.destroy_subscription(sub)
    return got[0]


def cloud(r, prefix="eih"):
    import cv2
    p, R = r.fk()
    T_WH = np.eye(4); T_WH[:3, :3] = R; T_WH[:3, 3] = p
    T_WC = T_WH @ T_HC
    depth_msg = grab(r, f"/{CAM}/depth/image_raw", Image)
    info = grab(r, f"/{CAM}/color/camera_info", CameraInfo)
    color_msg = grab(r, f"/{CAM}/color/image_raw", Image)
    cv2.imwrite(f"{prefix}.png", CvBridge().imgmsg_to_cv2(color_msg, desired_encoding="bgr8"))
    depth = CvBridge().imgmsg_to_cv2(depth_msg, desired_encoding="passthrough").astype(np.float64)
    H, W = depth.shape
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    P = np.stack([(u - cx) * depth / fx, (v - cy) * depth / fy, depth], -1)
    Pw = P @ T_WC[:3, :3].T + T_WC[:3, 3]
    Pw[~np.isfinite(depth) | (depth <= 0)] = np.nan
    np.save(f"{prefix}_xyz.npy", Pw)
    print(f"{prefix}_xyz.npy cam at", T_WC[:3, 3].round(3))
    return Pw


if __name__ == "__main__":
    r = Robot("eihcloud")
    cloud(r, sys.argv[1] if len(sys.argv) > 1 else "eih")
OPENRUA_EOF

# openrua op 35
timeout 200 python3 eih_cloud.py eih1 && python3 -c "
import numpy as np
P=np.load('eih1_xyz.npy')
np.set_printoptions(suppress=True, linewidth=250)
# sanity: table plane points, microwave top
print('table sample', P[300,300].round(3), 'door', P[200,60].round(3))
# cavity region approx pixels x 235-410, y 150-280
reg=P[140:290,225:420].reshape(-1,3); reg=reg[np.isfinite(reg[:,0])]
inside=reg[(reg[:,1]>0.26)&(reg[:,1]<0.6)]
print('cavity pts',len(inside))
print('x range',np.percentile(inside[:,0],[0.5,99.5]).round(3),'y range',np.percentile(inside[:,1],[0.5,99.5]).round(3),'z range',np.percentile(inside[:,2],[0.5,99.5]).round(3))
# walls: points with |x - xmin| small etc. Histogram of x
h,e=np.histogram(inside[:,0],bins=np.arange(-0.25,0.15,0.01)); print('x hist',list(zip(e[:-1].round(2),h)))
h,e=np.histogram(inside[:,2],bins=np.arange(0.9,1.15,0.01)); print('z hist',list(zip(e[:-1].round(2),h)))
h,e=np.histogram(inside[:,1],bins=np.arange(0.25,0.6,0.02)); print('y hist',list(zip(e[:-1].round(2),h)))
"

# openrua op 36
python3 -c "
import numpy as np
P=np.load('eih1_xyz.npy')
reg=P.reshape(-1,3); reg=reg[np.isfinite(reg[:,0])]
inn=reg[(reg[:,1]>0.30)&(reg[:,1]<0.42)&(reg[:,0]>-0.2)&(reg[:,0]<0.1)]
print('interior z range',inn[:,2].min().round(3),inn[:,2].max().round(3),' x range',inn[:,0].min().round(3),inn[:,0].max().round(3))
h,e=np.histogram(inn[:,2],bins=np.arange(0.9,1.15,0.005)); print([(round(a,3),b) for a,b in zip(e[:-1],h) if b>0])
# front face: points with y in 0.24..0.29, z>1.0 : where is the front plane, and the opening's top edge
fr=reg[(reg[:,1]>0.22)&(reg[:,1]<0.30)&(reg[:,0]>-0.15)&(reg[:,0]<0.04)&(reg[:,2]>0.95)]
h,e=np.histogram(fr[:,1],bins=np.arange(0.22,0.30,0.005)); print('front y hist',[(round(a,3),b) for a,b in zip(e[:-1],h) if b>0])
top=fr[fr[:,2]>1.06]
print('front top pts y', np.percentile(top[:,1],[5,50,95]).round(3), 'z', np.percentile(top[:,2],[5,50,95]).round(3))
# floor front edge: points z in 0.93-0.95 with smallest y
fl=reg[(reg[:,2]>0.935)&(reg[:,2]<0.95)&(reg[:,0]>-0.15)&(reg[:,0]<0.04)]
print('floor y range',np.percentile(fl[:,1],[1,50,99]).round(3))
"

# openrua op 37
python3 -c "
import numpy as np
P=np.load('eih1_xyz.npy')
reg=P.reshape(-1,3); reg=reg[np.isfinite(reg[:,0])]
d=reg[(reg[:,0]<-0.17)&(reg[:,0]>-0.26)&(reg[:,1]>0.0)&(reg[:,1]<0.27)&(reg[:,2]>0.92)]
print('door pts',len(d),'x',np.percentile(d[:,0],[1,50,99]).round(3),'y',np.percentile(d[:,1],[1,50,99]).round(3),'z',np.percentile(d[:,2],[1,50,99]).round(3))
h,e=np.histogram(d[:,0],bins=np.arange(-0.26,-0.16,0.005)); print('door x hist',[(round(a,3),b) for a,b in zip(e[:-1],h) if b>0])
h,e=np.histogram(d[:,2],bins=np.arange(0.92,1.12,0.01)); print('door z hist',[(round(a,3),b) for a,b in zip(e[:-1],h) if b>0])
h,e=np.histogram(d[:,1],bins=np.arange(0.0,0.28,0.01)); print('door y hist',[(round(a,3),b) for a,b in zip(e[:-1],h) if b>0])
# door handle? points x < -0.215
hd=d[d[:,0]<-0.215]; print('handle-ish pts',len(hd), 'y',np.percentile(hd[:,1],[5,50,95]).round(3) if len(hd) else '', 'z',np.percentile(hd[:,2],[5,50,95]).round(3) if len(hd) else '')
"

# openrua op 38
cat > grasp.py <<'EOF'
from rb import *
import sys
r=Robot('grasp')
beta=np.radians(20)
zh=np.array([0,np.sin(beta),-np.cos(beta)]); yh=np.array([0,np.cos(beta),np.sin(beta)])
R=R_from(zh,yh)
x_mug,y_mug,rim=-0.08,-0.287,1.013
tcp=np.array([x_mug, y_mug-0.0435, rim-0.012])
def flange(t): return t-0.1034*zh
print('fingers',r.fingers())
# pre-grasp
p1=flange(tcp+np.array([0,0,0.12]))
q1=r.ik(p1,R); print('q1',None if q1 is None else q1.round(3))
code,err=r.move(q1,5); print('pre code',code,err.round(4), r.fk()[0].round(3), 'target',p1.round(3))
if err>0.02: code,err=r.move(q1,5); print('pre retry',code,err.round(4))
# descend in two steps
p2=flange(tcp+np.array([0,0,0.05])); q2=r.ik(p2,R,seed=q1)
p3=flange(tcp); q3=r.ik(p3,R,seed=q2)
print('q3',q3.round(3))
code,err=r.move(q3,4,extra_points=[(q2,2.0)]); print('descend code',code,err.round(4),r.fk()[0].round(3),'target',p3.round(3))
if err>0.01: code,err=r.move(q3,3); print('descend retry',code,err.round(4),r.fk()[0].round(3))
print('wrench before',r.wrench().round(2))
r.snap('robot0_eye_in_hand','eih_pregrasp.png'); r.snap('agentview','agent_pregrasp.png')
EOF
timeout 900 python3 grasp.py

# openrua op 39
cat > close.py <<'EOF'
from rb import *
r=Robot('close')
print('grip',r.grip(0.0))
r.spin(0.5)
print('fingers',r.fingers(),'wrench',r.wrench().round(2))
# lift 4 cm straight up keeping orientation
p,R=r.fk()
q=r.ik(p+np.array([0,0,0.04]),R)
print('lift',r.move(q,2.5), r.fk()[0].round(3))
print('fingers after lift',r.fingers(),'wrench',r.wrench().round(2))
r.snap('agentview','agent_lift.png'); r.snap('frontview','front_lift.png')
EOF
timeout 600 python3 close.py

# openrua op 40
cat > carry.py <<'EOF'
from rb import *
r=Robot('carry')
beta=np.radians(20)
zh=np.array([0,np.sin(beta),-np.cos(beta)]); yh=np.array([0,np.cos(beta),np.sin(beta)])
R=R_from(zh,yh)
def flange(t): return t-0.1034*zh
print('fingers',r.fingers())
# current TCP
p,_=r.fk(); tcp=p+0.1034*zh; print('tcp now',tcp.round(3))
rim=tcp[2]+0.012; print('mug rim',rim.round(3),'bottom',(rim-0.11).round(3))
# lift so mug bottom >= 1.13: rim >= 1.24 -> tcp z = 1.228
q0=r.arm_q()
t1=np.array([tcp[0],tcp[1],1.228]); q1=r.ik(flange(t1),R,seed=q0)
# move over to cavity at same height: mug center target (-0.055,0.30) -> tcp y = 0.30-0.0435
t2=np.array([-0.055,0.30-0.0435,1.228]); q2=r.ik(flange(t2),R,seed=q1)
print('q1',q1.round(3)); print('q2',q2.round(3))
code,err=r.move(q2,8,extra_points=[(q1,3.0)]); print('carry',code,err.round(4),r.fk()[0].round(3))
if err>0.01: code,err=r.move(q2,4); print('carry retry',code,err.round(4),r.fk()[0].round(3))
print('fingers',r.fingers(),'wrench',r.wrench().round(2))
r.snap('agentview','agent_carry.png'); r.snap('frontview','front_carry.png')
EOF
timeout 600 python3 carry.py

# openrua op 41
cat > place.py <<'EOF'
from rb import *
r=Robot('place')
beta=np.radians(20)
zh=np.array([0,np.sin(beta),-np.cos(beta)]); yh=np.array([0,np.cos(beta),np.sin(beta)])
R=R_from(zh,yh)
def flange(t): return t-0.1034*zh
p,_=r.fk(); tcp=p+0.1034*zh; print('tcp now',tcp.round(3))
w0=r.wrench()
q=r.arm_q()
for z in [1.15,1.10,1.075,1.06]:
    t=np.array([-0.055,0.30-0.0435,z]); q=r.ik(flange(t),R,seed=q)
    code,err=r.move(q,3); 
    if err>0.01: code,err=r.move(q,2)
    w=r.wrench(); print('z',z,code,err.round(4),r.fk()[0].round(3),'dW',(w-w0).round(2),'fingers',np.round(r.fingers(),4))
r.snap('agentview','agent_place.png'); r.snap('frontview','front_place.png')
EOF
timeout 600 python3 place.py

# openrua op 42
python3 cloud.py agentview >/dev/null && python3 cloud.py sideview >/dev/null && python3 - <<'EOF'
import numpy as np
for cam in ['agentview','sideview']:
    P=np.load(f'{cam}_xyz.npy').reshape(-1,3); P=P[np.isfinite(P[:,0])]
    m=(P[:,0]>-0.2)&(P[:,0]<0.1)&(P[:,1]>0.1)&(P[:,1]<0.5)&(P[:,2]>1.0)&(P[:,2]<1.35)
    Q=P[m]
    print(cam, len(Q))
    # histogram by z
    for z0 in np.arange(1.0,1.35,0.02):
        s=Q[(Q[:,2]>=z0)&(Q[:,2]<z0+0.02)]
        if len(s): print(f' z{z0:.2f} n={len(s):4d} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]')
EOF

# openrua op 43
python3 - <<'EOF'
import numpy as np
P=np.load('agentview_xyz.npy').reshape(-1,3); P=P[np.isfinite(P[:,0])]
m=(P[:,0]>-0.15)&(P[:,0]<0.04)&(P[:,1]>0.25)&(P[:,1]<0.33)&(P[:,2]>0.9)&(P[:,2]<1.13)
Q=P[m]
for z0 in np.arange(0.9,1.13,0.01):
    s=Q[(Q[:,2]>=z0)&(Q[:,2]<z0+0.01)]
    if len(s): print(f' z{z0:.2f} n={len(s):4d} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}] ymed {np.median(s[:,1]):.3f}')
EOF
python3 -c "
import cv2; im=cv2.imread('agentview.png'); cv2.imwrite('crop_place.png', cv2.resize(im[120:260,380:640],None,fx=3,fy=3,interpolation=cv2.INTER_NEAREST))"

# openrua op 44
cat > release.py <<'EOF'
from rb import *
r=Robot('rel')
print('open',r.grip(0.04))
p,R=r.fk(); print('hand',p.round(3))
zh=R[:,2]
# retreat along -zh (up/back) by 12 cm
q=r.ik(p-0.12*zh,R); code,err=r.move(q,3)
if err>0.01: code,err=r.move(q,2)
print('retreat',code,err.round(4),r.fk()[0].round(3),'wrench',r.wrench().round(2))
r.snap('agentview','agent_rel.png'); r.snap('frontview','front_rel.png')
EOF
timeout 600 python3 release.py && python3 cloud.py agentview >/dev/null && python3 - <<'EOF'
import numpy as np
P=np.load('agentview_xyz.npy').reshape(-1,3); P=P[np.isfinite(P[:,0])]
m=(P[:,0]>-0.2)&(P[:,0]<0.1)&(P[:,1]>0.2)&(P[:,1]<0.5)&(P[:,2]>1.108)&(P[:,2]<1.3)
Q=P[m]
for z0 in np.arange(1.10,1.3,0.02):
    s=Q[(Q[:,2]>=z0)&(Q[:,2]<z0+0.02)]
    if len(s): print(f' z{z0:.2f} n={len(s):4d} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]')
EOF

# openrua op 45
cat > regrasp.py <<'EOF'
from rb import *
r=Robot('regrasp')
beta=np.radians(20)
zh=np.array([0,np.sin(beta),-np.cos(beta)]); yh=np.array([0,np.cos(beta),np.sin(beta)])
R=R_from(zh,yh)
def flange(t): return t-0.1034*zh
x_mug,y_mug,rim=-0.011,0.323,1.22
tcp=np.array([x_mug,y_mug-0.0435,rim-0.012])
def go(t,sec,seed=None):
    q=r.ik(flange(t),R,seed=seed)
    if q is None: raise SystemExit('IK fail '+str(t))
    code,err=r.move(q,sec)
    if err>0.01: code,err=r.move(q,3)
    p=r.fk()[0]; print('at tcp',(p+0.1034*zh).round(3),'target',t.round(3),code,err.round(4))
    return q
q=go(tcp+np.array([0,0,0.10]),5)
q=go(tcp+np.array([0,0,0.04]),3,q)
q=go(tcp,3,q)
print('wrench',r.wrench().round(2))
print('grip',r.grip(0.0))
r.spin(0.3); print('fingers',np.round(r.fingers(),4))
q=go(tcp+np.array([0,0,0.035]),3,q)
print('fingers after lift',np.round(r.fingers(),4),'wrench',r.wrench().round(2))
r.snap('agentview','agent_regrasp.png')
EOF
timeout 600 python3 regrasp.py

# openrua op 46
cat > insert.py <<'EOF'
from rb import *
r=Robot('insert')
beta=np.radians(20)
zh=np.array([0,np.sin(beta),-np.cos(beta)]); yh=np.array([0,np.cos(beta),np.sin(beta)])
R=R_from(zh,yh)
def flange(t): return t-0.1034*zh
w0=r.wrench()
def go(t,sec,seed=None,tag=''):
    q=r.ik(flange(t),R,seed=seed)
    if q is None: raise SystemExit('IK fail '+str(t))
    code,err=r.move(q,sec)
    if err>0.01: code,err=r.move(q,3)
    p=r.fk()[0]; w=r.wrench()
    print(tag,'tcp',(p+0.1034*zh).round(3),'tgt',t.round(3),code,err.round(4),'dW',(w-w0).round(1),'fing',np.round(r.fingers(),4))
    return q, np.abs(w-w0)[:3].max()
p,_=r.fk(); tcp=p+0.1034*zh
q,_=go(np.array([-0.05,0.17-0.0435,tcp[2]]),6,tag='out')
for z in [1.15,1.10,1.058]:
    q,f=go(np.array([-0.05,0.17-0.0435,z]),3,q,tag='down')
    if f>8: raise SystemExit('force!')
for y in [0.22,0.26,0.30]:
    q,f=go(np.array([-0.05,y-0.0435,1.058]),3,q,tag='in')
    if f>8: raise SystemExit('force!')
r.snap('agentview','agent_in.png'); r.snap('frontview','front_in.png')
EOF
timeout 900 python3 insert.py

# openrua op 47
python3 -c "
from rb import *
r=Robot('s'); r.snap('agentview','agent_in.png'); r.snap('frontview','front_in.png'); print(r.fk()[0].round(3), r.fingers())"

# openrua op 48
python3 cloud.py agentview >/dev/null && python3 - <<'EOF'
import numpy as np
P=np.load('agentview_xyz.npy').reshape(-1,3); P=P[np.isfinite(P[:,0])]
m=(P[:,0]>-0.2)&(P[:,0]<0.1)&(P[:,1]>0.08)&(P[:,1]<0.27)&(P[:,2]>0.93)&(P[:,2]<1.03)
Q=P[m]
for z0 in np.arange(0.93,1.03,0.01):
    s=Q[(Q[:,2]>=z0)&(Q[:,2]<z0+0.01)]
    if len(s): print(f' z{z0:.2f} n={len(s):4d} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]')
# rim band: z 1.005-1.02
s=Q[(Q[:,2]>1.0)&(Q[:,2]<1.02)]
print('rim band center', s[:,0].mean().round(3), s[:,1].mean().round(3), 'x mid',( (s[:,0].min()+s[:,0].max())/2).round(3))
EOF

# openrua op 49
python3 - <<'EOF'
import numpy as np
P=np.load('agentview_xyz.npy').reshape(-1,3); P=P[np.isfinite(P[:,0])]
m=(P[:,0]>-0.15)&(P[:,0]<0.06)&(P[:,1]>0.08)&(P[:,1]<0.245)&(P[:,2]>0.93)&(P[:,2]<1.03)
Q=P[m]
for z0 in np.arange(0.93,1.03,0.01):
    s=Q[(Q[:,2]>=z0)&(Q[:,2]<z0+0.01)]
    if len(s): print(f' z{z0:.2f} n={len(s):4d} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]')
s=Q[(Q[:,2]>1.0)&(Q[:,2]<1.02)]
print('rim pts', len(s), 'x range',s[:,0].min().round(3),s[:,0].max().round(3),'y range',s[:,1].min().round(3),s[:,1].max().round(3))
EOF

# openrua op 50
cat > grasp2.py <<'EOF'
from rb import *
r=Robot('grasp2')
zh=np.array([0,0,-1.0]); yh=np.array([0,1.0,0])
R=R_from(zh,yh)
def flange(t): return t-0.1034*zh
x_mug,y_mug,rim=-0.0435,0.1425,1.013
tcp=np.array([x_mug,y_mug-0.0435,rim-0.012])
w0=r.wrench()
def go(t,sec,seed=None,tag='',fmax=8):
    q=r.ik(flange(t),R,seed=seed)
    if q is None: raise SystemExit('IK fail '+str(t))
    code,err=r.move(q,sec)
    if err>0.01: code,err=r.move(q,3)
    p=r.fk()[0]; w=r.wrench()
    print(tag,'tcp',(p+0.1034*zh).round(3),'tgt',t.round(3),code,err.round(4),'dW',(w-w0).round(1),'fing',np.round(r.fingers(),4))
    if np.abs(w-w0)[:3].max()>fmax: raise SystemExit('force!')
    return q
print('fingers',r.fingers())
if r.fingers()[0]<0.035: print('open',r.grip(0.04))
q=go(tcp+np.array([0,0,0.12]),6,tag='pre')
q=go(tcp+np.array([0,0,0.05]),3,q,tag='mid')
q=go(tcp,3,q,tag='grasp')
print('grip',r.grip(0.0)); r.spin(0.3); print('fingers',np.round(r.fingers(),4))
q=go(tcp+np.array([0,0,0.0675]),3,q,tag='lift')   # rim -> 1.0805
print('fingers',np.round(r.fingers(),4))
r.snap('agentview','agent_g2.png')
EOF
timeout 900 python3 grasp2.py

# openrua op 51
python3 -c "
from rb import *
r=Robot('s'); r.snap('agentview','agent_g2.png'); print(r.fk()[0].round(3), r.wrench().round(2))"

# openrua op 52
cat > grasp2b.py <<'EOF'
from rb import *
r=Robot('grasp2b')
zh=np.array([0,0,-1.0]); yh=np.array([0,1.0,0])
R=R_from(zh,yh)
def flange(t): return t-0.1034*zh
x_mug,y_mug,rim=-0.0435,0.1425,1.013
tcp=np.array([x_mug,y_mug-0.0435,rim-0.012])
w0=r.wrench()
def go(t,sec,seed=None,tag='',fmax=8):
    q=r.ik(flange(t),R,seed=seed)
    if q is None: raise SystemExit('IK fail '+str(t))
    code,err=r.move(q,sec)
    if err>0.01: code,err=r.move(q,3)
    p=r.fk()[0]; w=r.wrench()
    print(tag,'tcp',(p+0.1034*zh).round(3),'tgt',t.round(3),code,err.round(4),'dW',(w-w0).round(1),'fing',np.round(r.fingers(),4))
    if np.abs(w-w0)[:3].max()>fmax: raise SystemExit('force!')
    return q
q=r.arm_q()
q=go(tcp+np.array([0,0,0.05]),3,q,tag='mid')
q=go(tcp,3,q,tag='grasp')
print('grip',r.grip(0.0)); r.spin(0.3); print('fingers',np.round(r.fingers(),4))
q=go(tcp+np.array([0,0,0.0675]),3,q,tag='lift')   # rim -> 1.0805
print('fingers',np.round(r.fingers(),4))
r.snap('agentview','agent_g2.png')
EOF
timeout 900 python3 grasp2b.py

# openrua op 53
cat > insert2.py <<'EOF'
from rb import *
r=Robot('insert2')
zh=np.array([0,0,-1.0]); yh=np.array([0,1.0,0])
R=R_from(zh,yh)
def flange(t): return t-0.1034*zh
w0=r.wrench()
def go(t,sec,seed=None,tag='',fmax=6):
    q=r.ik(flange(t),R,seed=seed)
    if q is None: raise SystemExit('IK fail '+str(t))
    code,err=r.move(q,sec)
    if err>0.01: code,err=r.move(q,3)
    p=r.fk()[0]; w=r.wrench()
    print(tag,'tcp',(p+0.1034*zh).round(3),'tgt',t.round(3),code,err.round(4),'dW',(w-w0).round(1),'fing',np.round(r.fingers(),4))
    if np.abs(w-w0)[:3].max()>fmax: raise SystemExit('force!')
    return q
q=r.arm_q()
p,_=r.fk(); tcp=p+0.1034*zh
x=-0.05
for y in [0.13,0.17,0.20,0.22,0.235,0.2495]:
    q=go(np.array([x,y,1.0685]),3,q,tag=f'in y={y}')
    if r.fingers()[0]<0.001: raise SystemExit('slipped!')
r.snap('agentview','agent_in2.png'); r.snap('frontview','front_in2.png')
EOF
timeout 900 python3 insert2.py

# openrua op 54
python3 -c "
from rb import *
r=Robot('s'); r.snap('agentview','agent_now.png'); print(r.fk()[0].round(3), np.round(r.fingers(),4), r.wrench().round(2))"

# openrua op 55
cat > setdown.py <<'EOF'
from rb import *
r=Robot('setdown')
zh=np.array([0,0,-1.0]); yh=np.array([0,1.0,0])
R=R_from(zh,yh)
def flange(t): return t-0.1034*zh
w0=r.wrench(); q=r.arm_q()
for z in [1.04,1.02,1.008,1.0]:
    t=np.array([-0.05,0.17,z]); q=r.ik(flange(t),R,seed=q); code,err=r.move(q,2)
    w=r.wrench(); print('z',z,code,err.round(4),'dW',(w-w0).round(1),'fing',np.round(r.fingers(),4))
print('open',r.grip(0.04))
t=np.array([-0.05,0.17,1.12]); q=r.ik(flange(t),R,seed=q); print(r.move(q,3))
r.snap('agentview','agent_sd.png')
EOF
timeout 600 python3 setdown.py && python3 cloud.py agentview >/dev/null && python3 - <<'EOF'
import numpy as np
P=np.load('agentview_xyz.npy').reshape(-1,3); P=P[np.isfinite(P[:,0])]
m=(P[:,0]>-0.16)&(P[:,0]<0.06)&(P[:,1]>0.05)&(P[:,1]<0.26)&(P[:,2]>0.93)&(P[:,2]<1.06)
Q=P[m]
for z0 in np.arange(0.93,1.06,0.01):
    s=Q[(Q[:,2]>=z0)&(Q[:,2]<z0+0.01)]
    if len(s): print(f' z{z0:.2f} n={len(s):4d} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]')
EOF

# openrua op 56
cat > setdown2.py <<'EOF'
from rb import *
r=Robot('setdown2')
zh=np.array([0,0,-1.0]); yh=np.array([0,1.0,0])
R=R_from(zh,yh)
def flange(t): return t-0.1034*zh
w0=r.wrench(); q=r.arm_q()
for z in [1.08,1.05,1.03,1.01,0.995,0.98]:
    t=np.array([-0.05,0.17,z]); q=r.ik(flange(t),R,seed=q); code,err=r.move(q,2)
    w=r.wrench(); print('z',z,code,err.round(4),'dW',(w-w0).round(1),'fing',np.round(r.fingers(),4))
    if np.abs(w-w0)[:3].max()>4: break
print('open',r.grip(0.04)); r.spin(0.5); print('fingers',np.round(r.fingers(),4))
r.snap('agentview','agent_sd2.png')
EOF
timeout 600 python3 setdown2.py

# openrua op 57
python3 - <<'EOF'
from rb import *
r=Robot('up')
p,R=r.fk(); q=r.ik(p+np.array([0,0,0.15]),R); print(r.move(q,3), r.fk()[0].round(3))
r.snap('agentview','agent_up.png')
EOF
python3 cloud.py agentview >/dev/null && python3 - <<'EOF'
import numpy as np
P=np.load('agentview_xyz.npy').reshape(-1,3); P=P[np.isfinite(P[:,0])]
m=(P[:,0]>-0.16)&(P[:,0]<0.08)&(P[:,1]>0.06)&(P[:,1]<0.26)&(P[:,2]>0.93)&(P[:,2]<1.06)
Q=P[m]
for z0 in np.arange(0.93,1.06,0.01):
    s=Q[(Q[:,2]>=z0)&(Q[:,2]<z0+0.01)]
    if len(s): print(f' z{z0:.2f} n={len(s):4d} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]')
s=Q[(Q[:,2]>1.0)&(Q[:,2]<1.02)&(Q[:,1]>0.08)]
print('rim x',s[:,0].min().round(3),s[:,0].max().round(3),'y',s[:,1].min().round(3),s[:,1].max().round(3))
EOF

# openrua op 58
python3 - <<'EOF'
import numpy as np
P=np.load('agentview_xyz.npy').reshape(-1,3); P=P[np.isfinite(P[:,0])]
m=(P[:,0]>-0.16)&(P[:,0]<0.06)&(P[:,1]>0.06)&(P[:,1]<0.24)&(P[:,2]>1.0)&(P[:,2]<1.02)
s=P[m]
print('rim band x',s[:,0].min().round(3),s[:,0].max().round(3),'y',s[:,1].min().round(3),s[:,1].max().round(3), 'n',len(s))
# x histogram
h,e=np.histogram(s[:,0],bins=np.arange(-0.16,0.06,0.01)); print(list(zip(e[:-1].round(2),h)))
h,e=np.histogram(s[:,1],bins=np.arange(0.06,0.24,0.01)); print(list(zip(e[:-1].round(2),h)))
EOF

# openrua op 59
cat > quick.py <<'EOF'
from rb import *
import time
r=Robot('quick')
zh=np.array([0,0,-1.0]); yh=np.array([0,1.0,0])
R=R_from(zh,yh)
def flange(t): return t-0.1034*zh
x_mug,y_mug,rim=-0.052,0.122,1.013
tcp=np.array([x_mug,y_mug-0.0435,rim-0.014])
w0=r.wrench()
def go(t,sec,seed=None,tag='',fmax=6):
    q=r.ik(flange(t),R,seed=seed)
    if q is None: raise SystemExit('IK fail '+str(t))
    code,err=r.move(q,sec)
    if err>0.01: code,err=r.move(q,3)
    p=r.fk()[0]; w=r.wrench()
    print(tag,'tcp',(p+0.1034*zh).round(3),'tgt',t.round(3),code,err.round(4),'dW',(w-w0).round(1),'fing',np.round(r.fingers(),4))
    if np.abs(w-w0)[:3].max()>fmax: raise SystemExit('force!')
    return q
q=go(tcp+np.array([0,0,0.10]),5,tag='pre')
q=go(tcp+np.array([0,0,0.04]),2,q,tag='mid')
q=go(tcp,2,q,tag='grasp')
# precompute trajectory
qa=r.ik(flange(np.array([x_mug,y_mug-0.0435,1.067])),R,seed=q)
qb=r.ik(flange(np.array([-0.05,0.16,1.067])),R,seed=qa)
qc=r.ik(flange(np.array([-0.05,0.2495,1.067])),R,seed=qb)
print('traj ok', qa is not None, qb is not None, qc is not None)
t0=time.time()
print('grip',r.grip(0.0), time.time()-t0)
code,err=r.move(qc,4.5,extra_points=[(qa,1.5),(qb,3.0)])
print('insert',code,err.round(4),'t',round(time.time()-t0,1),'fing',np.round(r.fingers(),4),'tcp',(r.fk()[0]+0.1034*zh).round(3))
print('open',r.grip(0.04), round(time.time()-t0,1))
print('wrench',(r.wrench()-w0).round(1))
r.snap('agentview','agent_q.png'); r.snap('frontview','front_q.png')
EOF
timeout 900 python3 quick.py

# openrua op 60
python3 - <<'EOF'
import numpy as np
P=np.load('agentview_xyz.npy').reshape(-1,3); P=P[np.isfinite(P[:,0])]
m=(P[:,0]>-0.16)&(P[:,0]<0.03)&(P[:,1]>0.06)&(P[:,1]<0.26)&(P[:,2]>0.9)&(P[:,2]<1.03)
Q=P[m]
for z0 in np.arange(0.90,1.03,0.01):
    s=Q[(Q[:,2]>=z0)&(Q[:,2]<z0+0.01)]
    if len(s)>5:
        xs=np.sort(s[:,0]); lo,hi=xs[int(0.02*len(xs))],xs[int(0.98*len(xs))-1]
        print(f' z{z0:.2f} n={len(s):4d} x2%[{lo:.3f},{hi:.3f}] diam {hi-lo:.3f} y[{s[:,1].min():.3f},{s[:,1].max():.3f}]')
EOF

# openrua op 61
python3 -c "
import cv2; im=cv2.imread('agent_carry.png'); cv2.imwrite('crop_carry.png', cv2.resize(im[0:240,400:640],None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))"

# openrua op 62
timeout 40 python3 - <<'EOF'
import time, rclpy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
rclpy.init(); n=rclpy.create_node('clk')
got=[]
n.create_subscription(JointState,'/joint_states',lambda m:got.append((time.time(),m.header.stamp.sec+m.header.stamp.nanosec*1e-9)),10)
t0=time.time()
while time.time()-t0<8: rclpy.spin_once(n,timeout_sec=0.1)
print(len(got), 'msgs; wall dt',got[-1][0]-got[0][0],'sim dt',got[-1][1]-got[0][1])
EOF

# openrua op 63
timeout 120 python3 - <<'EOF'
import time, numpy as np, rclpy
from rb import *
from sensor_msgs.msg import JointState
r=Robot('rate')
got=[]
r.node.create_subscription(JointState,'/joint_states',lambda m:got.append((time.time(),m.header.stamp.sec+m.header.stamp.nanosec*1e-9)),10)
q=r.arm_q()
t0=time.time(); code,err=r.move(q,2.0); t1=time.time()
print('move code',code,'wall',round(t1-t0,2))
g=[x for x in got if t0<=x[0]<=t1]
print(len(g),'sim dt',g[-1][1]-g[0][1],'wall dt',g[-1][0]-g[0][0])
print('fk',r.fk()[0].round(3),'fingers',np.round(r.fingers(),4))
EOF

# openrua op 64
mkdir -p "$(dirname /workspace/clear.py)"
cat > /workspace/clear.py <<'OPENRUA_EOF'
"""Hand vs microwave-front clearance model (world frame).

Hand model (relative to TCP, hand axes x_h,y_h,z_h): two fingers (each 0.01 along y_h,
0.02 along x_h, 0.045 along -z_h from the tip), body box 0.20 (y_h) x 0.065 (x_h) x 0.058 (-z_h)
sitting on top of the fingers. Finger gap g (per finger position) is given.
Microwave: front face y=0.271. Top frame slab: y>=0.271, z in [1.088,1.107] and above (top surface 1.107,
anything at y>=0.271 must have z>=1.107 or be inside the cavity: x in [-0.16,0.05], z in [0.945,1.088]).
Returns min clearance (negative = penetration) over sampled points.
"""
import numpy as np

YF = 0.271
ZTOP = 1.107
ZCEIL = 1.088
ZFLOOR = 0.945
XL, XR = -0.16, 0.05


def hand_points(tcp, R, g=0.004, n=6):
    xh, yh, zh = R[:, 0], R[:, 1], R[:, 2]
    pts = []
    # fingers
    for s in (+1, -1):
        for a in np.linspace(-0.01, 0.01, 3):          # along x_h
            for b in np.linspace(g, g + 0.01, 3):      # along y_h (from gap face outward)
                for c in np.linspace(0, 0.045, n):     # along -z_h from tip
                    pts.append(tcp + a * xh + s * b * yh - c * zh)
    # body
    for a in np.linspace(-0.0325, 0.0325, 3):
        for b in np.linspace(-0.1, 0.1, 9):
            for c in np.linspace(0.045, 0.045 + 0.058, 4):
                pts.append(tcp + a * xh + b * yh - c * zh)
    return np.array(pts)


def clearance(pts):
    """Signed clearance of points from microwave solid (front region only)."""
    y, z, x = pts[:, 1], pts[:, 2], pts[:, 0]
    d = np.full(len(pts), 1.0)
    inside_y = y >= YF
    # above top: fine (clearance = z - ZTOP)
    above = z >= ZTOP
    d[inside_y & above] = np.minimum(d[inside_y & above], (z - ZTOP)[inside_y & above])
    # in cavity: clearance to ceiling/floor/side walls
    cav = inside_y & (z < ZCEIL) & (z > ZFLOOR) & (x > XL) & (x < XR)
    d[cav] = np.minimum.reduce([d[cav], (ZCEIL - z)[cav], (z - ZFLOOR)[cav], (x - XL)[cav], (XR - x)[cav]])
    # in frame slab or walls: penetration
    bad = inside_y & ~above & ~cav
    pen = np.minimum(np.abs(z - ZCEIL), np.abs(z - ZTOP))
    d[bad] = -pen[bad]
    # points in front of the face: clearance = YF - y (only matters if within frame z band)
    front = ~inside_y
    band = (z > ZCEIL) & (z < ZTOP)
    d[front & band] = np.minimum(d[front & band], (YF - y)[front & band])
    return d.min()


def hand_clearance(tcp, R, g=0.004):
    return clearance(hand_points(np.asarray(tcp, float), R, g))
OPENRUA_EOF

# openrua op 65
python3 - <<'EOF'
import numpy as np
from rb import R_from
from clear import hand_clearance
for beta in (0,20,30,45,60):
    b=np.radians(beta); zh=np.array([0,np.sin(b),-np.cos(b)]); yh=np.array([0,np.cos(b),np.sin(b)]); R=R_from(zh,yh)
    for ym in (0.28,0.29,0.30,0.31,0.32):
        ok=[z for z in np.arange(1.00,1.10,0.002) if hand_clearance([-0.055,ym-0.042,z],R,g=0.005)>=0.004]
        if ok: print(f'beta {beta} y_mug {ym}: TCP z {min(ok):.3f}..{max(ok):.3f}  -> mug bottom (d=.012) {min(ok)+0.012-0.108:.3f}..{max(ok)+0.012-0.108:.3f}')
        else: print(f'beta {beta} y_mug {ym}: none')
EOF

# openrua op 66
python3 - <<'EOF'
import numpy as np
from rb import R_from
from clear import hand_clearance
for beta in (35,40,45,50):
    b=np.radians(beta); zh=np.array([0,np.sin(b),-np.cos(b)]); yh=np.array([0,np.cos(b),np.sin(b)]); R=R_from(zh,yh)
    for ym in (0.29,0.30,0.31,0.32):
        ok=[z for z in np.arange(1.00,1.10,0.002) if hand_clearance([-0.07,ym-0.0367,z],R,g=0.013)>=0.004]
        if ok: print(f'beta {beta} y_mug {ym}: TCP z {min(ok):.3f}..{max(ok):.3f} -> mug bottom(D=.02) {min(ok)+0.02-0.108:.3f}..{max(ok)+0.02-0.108:.3f}')
        else: print(f'beta {beta} y_mug {ym}: none')
EOF

# openrua op 67
mkdir -p "$(dirname /workspace/push.py)"
cat > /workspace/push.py <<'OPENRUA_EOF'
from rb import *
r = Robot('push')
# vertical hand, closing along x -> body long axis along x, pads face +-x (pushing face)
zh = np.array([0, 0, -1.0]); yh = np.array([1.0, 0, 0]); R = R_from(zh, yh)
def flange(t): return t - 0.1034 * zh
xm, ym = -0.052, 0.122
print('closing fingers'); print(r.grip(0.0))
w0 = None
def go(t, sec, tag='', fmax=8):
    global w0
    q = r.ik(flange(t), R)
    if q is None: raise SystemExit('ik fail ' + tag)
    code, err = r.move(q, sec)
    if err > 0.01: code, err = r.move(q, 3)
    p = r.fk()[0]; w = r.wrench()
    if w0 is None: w0 = w
    print(tag, 'tcp', (p + 0.1034 * zh).round(3), 'code', code, 'err', round(err, 3), 'dW', (w - w0).round(1))
    if np.abs(w - w0)[:3].max() > fmax: raise SystemExit('force!')
x0 = xm + 0.046 + 0.012
go([x0, ym, 1.06], 5, 'above')
go([x0, ym, 0.95], 3, 'down')
go([x0 - 0.045, ym, 0.95], 4, 'push1')
go([-0.034, ym, 0.95], 4, 'push2')
go([-0.034, ym, 1.06], 3, 'up')
r.snap('agentview', 'agent_push.png')
OPENRUA_EOF

# openrua op 68
timeout 600 python3 push.py 2>&1 | tail -20

# openrua op 69
timeout 60 python3 -c "
from rb import *
r=Robot('chk'); print(r.fingers()); print(r.fk()[0].round(3)); r.snap('agentview','agent_now.png'); r.snap('robot0_eye_in_hand','eih_now.png')"

# openrua op 70
mkdir -p "$(dirname /workspace/push.py)"
cat > /workspace/push.py <<'OPENRUA_EOF'
from rb import *
r = Robot('push')
# fingers pitched 30 deg: pointing down and -x; body leans up/+x. closing axis in x-z plane
b = np.radians(30)
zh = np.array([-np.sin(b), 0, -np.cos(b)]); yh = np.array([np.cos(b), 0, -np.sin(b)]); R = R_from(zh, yh)
def flange(t): return t - 0.1034 * zh
xm, ym = -0.052, 0.122
print('fingers', np.round(r.fingers(), 4))
w0 = None
def go(t, sec, tag='', fmax=8):
    global w0
    q = r.ik(flange(t), R)
    if q is None: raise SystemExit('ik fail ' + tag)
    code, err = r.move(q, sec)
    if err > 0.01: code, err = r.move(q, 3)
    p = r.fk()[0]; w = r.wrench()
    if w0 is None: w0 = w
    print(tag, 'tcp', (p + 0.1034 * zh).round(3), 'code', code, 'err', round(err, 3), 'dW', (w - w0).round(1))
    if np.abs(w - w0)[:3].max() > fmax: raise SystemExit('force!')
x0 = xm + 0.046 + 0.02
go([x0, ym, 1.05], 5, 'above')
go([x0, ym, 0.93], 3, 'down')
go([x0 - 0.04, ym, 0.93], 4, 'push1')
go([-0.036, ym, 0.93], 4, 'push2')
go([-0.036, ym, 1.05], 3, 'up')
r.snap('agentview', 'agent_push.png')
OPENRUA_EOF

# openrua op 71
timeout 600 python3 push.py 2>&1 | tail -20

# openrua op 72
timeout 120 python3 cloud.py agentview && python3 - <<'EOF'
import numpy as np
P=np.load('agentview_xyz.npy').reshape(-1,3); P=P[np.isfinite(P[:,0])]
# white mug region
m=(P[:,0]>-0.16)&(P[:,0]<0.08)&(P[:,1]>0.06)&(P[:,1]<0.20)&(P[:,2]>0.91)&(P[:,2]<1.03)
Q=P[m]; print(len(Q))
for z0 in np.arange(0.91,1.03,0.01):
    s=Q[(Q[:,2]>=z0)&(Q[:,2]<z0+0.01)]
    if len(s): print(f'z{z0:.2f} n{len(s):4d} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]')
EOF

# openrua op 73
timeout 300 python3 - <<'EOF' 2>&1 | tail
from rb import *
r=Robot('away')
zh=np.array([0,0,-1.0]); yh=np.array([0,1.0,0]); R=R_from(zh,yh)
q=r.ik(np.array([0.05,0.30,1.25])-0.1034*zh,R)
print(r.move(q,5)); print(r.fk()[0].round(3))
EOF
timeout 120 python3 cloud.py agentview && timeout 120 python3 cloud.py frontview && python3 - <<'EOF'
import numpy as np
for cam in ('agentview','frontview'):
    P=np.load(f'{cam}_xyz.npy').reshape(-1,3); P=P[np.isfinite(P[:,0])]
    m=(P[:,0]>-0.20)&(P[:,0]<0.08)&(P[:,1]>0.04)&(P[:,1]<0.22)&(P[:,2]>0.905)&(P[:,2]<1.03)
    Q=P[m]; print(cam,len(Q))
    for z0 in np.arange(0.91,1.03,0.01):
        s=Q[(Q[:,2]>=z0)&(Q[:,2]<z0+0.01)]
        if len(s): print(f'z{z0:.2f} n{len(s):4d} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]')
EOF

# openrua op 74
python3 - <<'EOF'
import numpy as np
P=np.load('agentview_xyz.npy').reshape(-1,3); P=P[np.isfinite(P[:,0])]
for zr in ((0.93,0.95),(0.96,1.0),(1.0,1.03)):
    m=(P[:,0]>-0.19)&(P[:,0]<0.10)&(P[:,1]>-0.06)&(P[:,1]<0.24)&(P[:,2]>zr[0])&(P[:,2]<zr[1])
    Q=P[m]
    xs=np.arange(-0.19,0.10,0.01); ys=np.arange(-0.06,0.24,0.01)
    H,_,_=np.histogram2d(Q[:,0],Q[:,1],[xs,ys])
    print('z',zr,'  y from -0.06 to 0.24 (cols), x rows from -0.19')
    for i,x in enumerate(xs[:-1]):
        print(f'{x:+.2f} '+''.join('#' if H[i,j]>3 else ('.' if H[i,j]>0 else ' ') for j in range(len(ys)-1)))
EOF

# openrua op 75
timeout 60 python3 -c "
from rb import *
import cv2
r=Robot('s'); r.snap('agentview','agent_now.png'); r.snap('birdview','bird_now.png')
im=cv2.imread('agent_now.png'); cv2.imwrite('crop_now.png', cv2.resize(im[200:400,220:480],None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
"

# openrua op 76
python3 - <<'EOF'
import numpy as np
for cam in ('agentview','frontview'):
    P=np.load(f'{cam}_xyz.npy').reshape(-1,3); P=P[np.isfinite(P[:,0])]
    m=(P[:,0]>-0.13)&(P[:,0]<0.0)&(P[:,1]>0.05)&(P[:,1]<0.17)&(P[:,2]>1.0)&(P[:,2]<1.03)
    Q=P[m][:,:2]
    # algebraic circle fit
    A=np.c_[2*Q[:,0],2*Q[:,1],np.ones(len(Q))]; b=(Q**2).sum(1)
    c=np.linalg.lstsq(A,b,rcond=None)[0]; r=np.sqrt(c[2]+c[0]**2+c[1]**2)
    res=np.abs(np.hypot(Q[:,0]-c[0],Q[:,1]-c[1])-r)
    keep=res<0.008; Q2=Q[keep]
    A=np.c_[2*Q2[:,0],2*Q2[:,1],np.ones(len(Q2))]; b=(Q2**2).sum(1)
    c=np.linalg.lstsq(A,b,rcond=None)[0]; r=np.sqrt(c[2]+c[0]**2+c[1]**2)
    print(cam,'n',len(Q),len(Q2),'center',c[:2].round(4),'r',round(r,4))
    zr=P[m][:,2]; print(' rim z max',zr.max().round(3), 'pct95',np.percentile(zr,95).round(3))
EOF

# openrua op 77
timeout 120 python3 cloud.py birdview && python3 - <<'EOF'
import numpy as np
P=np.load('birdview_xyz.npy').reshape(-1,3); P=P[np.isfinite(P[:,0])]
m=(P[:,0]>-0.16)&(P[:,0]<0.10)&(P[:,1]>-0.08)&(P[:,1]<0.24)&(P[:,2]>0.985)&(P[:,2]<1.03)
Q=P[m]
xs=np.arange(-0.16,0.10,0.005); ys=np.arange(-0.08,0.24,0.005)
H,_,_=np.histogram2d(Q[:,0],Q[:,1],[xs,ys])
print('cols y from -0.08 step 0.005; marks every 0.05: ', ''.join('|' if abs((y*100)%5)<0.01 else ' ' for y in ys[:-1]))
for i,x in enumerate(xs[:-1]):
    print(f'{x:+.3f} '+''.join('#' if H[i,j]>0 else ' ' for j in range(len(ys)-1)))
EOF

# openrua op 78
python3 - <<'EOF'
import numpy as np
P=np.load('birdview_xyz.npy').reshape(-1,3); P=P[np.isfinite(P[:,0])]
def fit(Q):
    A=np.c_[2*Q[:,0],2*Q[:,1],np.ones(len(Q))]; b=(Q**2).sum(1)
    c=np.linalg.lstsq(A,b,rcond=None)[0]; return c[:2], np.sqrt(c[2]+c[0]**2+c[1]**2)
m=(P[:,0]>-0.125)&(P[:,0]<-0.015)&(P[:,1]>0.06)&(P[:,1]<0.165)&(P[:,2]>0.985)&(P[:,2]<1.03)
Q=P[m]; c,r=fit(Q[:,:2]); res=np.abs(np.hypot(Q[:,0]-c[0],Q[:,1]-c[1])-r); Q2=Q[res<0.006]; c,r=fit(Q2[:,:2])
print('white center',c.round(4),'r',round(r,4),'n',len(Q2),'rim z',np.percentile(Q2[:,2],[50,95]).round(3))
m=(P[:,0]>-0.06)&(P[:,0]<0.05)&(P[:,1]>-0.03)&(P[:,1]<0.09)&(P[:,2]>0.985)&(P[:,2]<1.03)
Q=P[m]; c,r=fit(Q[:,:2]); res=np.abs(np.hypot(Q[:,0]-c[0],Q[:,1]-c[1])-r); Q2=Q[res<0.006]; c,r=fit(Q2[:,:2])
print('yellow center',c.round(4),'r',round(r,4),'n',len(Q2))
# handle extent
h=(P[:,0]>-0.09)&(P[:,0]<-0.04)&(P[:,1]>0.165)&(P[:,1]<0.22)&(P[:,2]>0.92)&(P[:,2]<1.03)
print('handle y max',P[h][:,1].max().round(3),'z range',P[h][:,2].min().round(3),P[h][:,2].max().round(3),'x range',P[h][:,0].min().round(3),P[h][:,0].max().round(3))
EOF

# openrua op 79
python3 - <<'EOF'
import numpy as np
P=np.load('birdview_xyz.npy').reshape(-1,3); P=P[np.isfinite(P[:,0])]
m=(P[:,2]>0.985)&(P[:,2]<1.03)
Q=P[m]
w=Q[(Q[:,0]>-0.075)&(Q[:,0]<-0.055)&(Q[:,1]>0.06)&(Q[:,1]<0.20)]
print('white x-band y values: min',w[:,1].min().round(3),'max',w[:,1].max().round(3)); 
ys=np.sort(w[:,1]); print(np.round(ys[::max(1,len(ys)//25)],3))
v=Q[(Q[:,1]>0.11)&(Q[:,1]<0.135)&(Q[:,0]>-0.13)&(Q[:,0]<0.0)]
print('white y-band x values: min',v[:,0].min().round(3),'max',v[:,0].max().round(3))
y2=Q[(Q[:,0]>-0.015)&(Q[:,0]<0.005)&(Q[:,1]>-0.08)&(Q[:,1]<0.09)]
print('yellow x-band y values:',y2[:,1].min().round(3),y2[:,1].max().round(3))
x2=Q[(Q[:,1]>0.02)&(Q[:,1]<0.04)&(Q[:,0]>-0.07)&(Q[:,0]<0.07)]
print('yellow y-band x values:',x2[:,0].min().round(3),x2[:,0].max().round(3))
EOF

# openrua op 80
python3 - <<'EOF'
import numpy as np
P=np.load('birdview_xyz.npy').reshape(-1,3); P=P[np.isfinite(P[:,0])]
Q=P[(P[:,2]>0.985)&(P[:,2]<1.03)]
w=Q[(Q[:,0]>-0.075)&(Q[:,0]<-0.055)]
print(len(w)); print(np.round(w[np.argsort(w[:,1])][::max(1,len(w)//40)],3))
EOF

# openrua op 81
sed -i 's/^xm, ym = .*/xm, ym = -0.061, 0.088/; s/^go(\[-0.036, ym, 0.93\], 4, .push2.)/go([-0.040, ym, 0.93], 4, "push2")/; s/^go(\[-0.036, ym, 1.05\], 3, .up.)/go([-0.040, ym, 1.05], 3, "up")/' push.py && grep -n "xm, ym\|push2\|'up'\|\"up\"" push.py && timeout 600 python3 push.py 2>&1 | tail

# openrua op 82
timeout 60 python3 -c "
from rb import *
import cv2
r=Robot('s'); print(r.fk()[0].round(3)); r.snap('agentview','agent_now.png'); r.snap('frontview','front_now.png')
im=cv2.imread('agent_now.png'); cv2.imwrite('crop_now.png', cv2.resize(im[150:400,200:500],None,fx=2.5,fy=2.5,interpolation=cv2.INTER_CUBIC))
"

# openrua op 83
python3 - <<'EOF'
import numpy as np
for cam in ('birdview','frontview','agentview'):
    P=np.load(f'{cam}_xyz.npy').reshape(-1,3); P=P[np.isfinite(P[:,0])]
    for (x0,x1,y0,y1) in ((0.05,0.15,0.05,0.15),(-0.15,-0.05,0.18,0.24),(0.1,0.3,-0.2,-0.1),(-0.1,0.0,0.16,0.2)):
        s=P[(P[:,0]>x0)&(P[:,0]<x1)&(P[:,1]>y0)&(P[:,1]<y1)&(P[:,2]<1.0)]
        if len(s): print(cam,(x0,x1,y0,y1),'n',len(s),'z pct 5/50/95',np.percentile(s[:,2],[5,50,95]).round(3))
EOF

# openrua op 84
mkdir -p "$(dirname /workspace/relocate.py)"
cat > /workspace/relocate.py <<'OPENRUA_EOF'
from rb import *
r = Robot('reloc')
zh = np.array([0, 0, -1.0]); yh = np.array([1.0, 0, 0]); R = R_from(zh, yh)   # vertical, closing along x
def flange(t): return t - 0.1034 * zh
xm, ym, rim = -0.061, 0.088, 1.012
dst = np.array([-0.105, 0.150])
print('open', r.grip(0.08))
w0 = None
def go(t, sec, tag='', fmax=8):
    global w0
    q = r.ik(flange(t), R)
    if q is None: raise SystemExit('ik fail ' + tag)
    code, err = r.move(q, sec)
    if err > 0.01: code, err = r.move(q, 3)
    p = r.fk()[0]; w = r.wrench()
    if w0 is None: w0 = w
    print(tag, 'tcp', (p + 0.1034 * zh).round(3), 'code', code, 'err', round(err, 3), 'dW', (w - w0).round(1), 'fing', np.round(r.fingers(), 4))
    if np.abs(w - w0)[:3].max() > fmax: raise SystemExit('force!')
    return q
g = np.array([xm + 0.042, ym, rim - 0.014])
go([g[0], g[1], 1.09], 5, 'above')
go(g, 3, 'grasp pose')
print('close', r.grip(0.0))
t0 = time.time()
q1 = r.ik(flange([g[0], g[1], rim + 0.02]), R)
q2 = r.ik(flange([dst[0] + 0.042, dst[1], rim + 0.02]), R, seed=q1)
q3 = r.ik(flange([dst[0] + 0.042, dst[1], rim - 0.014]), R, seed=q2)
for a, b in ((r.arm_q(), q1), (q1, q2), (q2, q3)):
    print('jump', np.abs(np.array(a) - np.array(b)).max().round(3))
code, err = r.move(q3, 3.5, extra_points=[(q1, 1.0), (q2, 2.5)])
print('traj', code, round(err, 3), 'wall', round(time.time() - t0, 1), 'fing', np.round(r.fingers(), 4))
print('open', r.grip(0.08))
go([dst[0] + 0.042, dst[1], 1.09], 3, 'up')
r.snap('agentview', 'agent_reloc.png')
OPENRUA_EOF

# openrua op 85
timeout 900 python3 relocate.py 2>&1 | tail -20

# openrua op 86
timeout 120 python3 cloud.py birdview && python3 - <<'EOF'
import numpy as np
P=np.load('birdview_xyz.npy').reshape(-1,3); P=P[np.isfinite(P[:,0])]
Q=P[(P[:,2]>0.985)&(P[:,2]<1.03)]
# white mug: search ring around expected
w=Q[(Q[:,0]>-0.17)&(Q[:,0]<-0.04)&(Q[:,1]>0.08)&(Q[:,1]<0.25)]
print('white pts',len(w),'x',w[:,0].min().round(3),w[:,0].max().round(3),'y',w[:,1].min().round(3),w[:,1].max().round(3),'z max',w[:,2].max().round(3))
xb=w[(w[:,0]>w[:,0].mean()-0.01)&(w[:,0]<w[:,0].mean()+0.01)]; print(' y at center band',xb[:,1].min().round(3),xb[:,1].max().round(3))
yb=w[(w[:,1]>0.14)&(w[:,1]<0.16)]; print(' x at y band',yb[:,0].min().round(3),yb[:,0].max().round(3))
y2=Q[(Q[:,0]>-0.06)&(Q[:,0]<0.06)&(Q[:,1]>-0.08)&(Q[:,1]<0.08)]
print('yellow x',y2[:,0].min().round(3),y2[:,0].max().round(3),'y',y2[:,1].min().round(3),y2[:,1].max().round(3))
h=P[(P[:,0]>-0.15)&(P[:,0]<-0.05)&(P[:,1]>0.19)&(P[:,1]<0.26)&(P[:,2]>0.92)&(P[:,2]<1.0)]
print('handle pts',len(h), 'y max', h[:,1].max().round(3) if len(h) else None, 'x', (h[:,0].min().round(3),h[:,0].max().round(3)) if len(h) else None)
EOF

# openrua op 87
timeout 300 python3 - <<'EOF' 2>&1 | tail -3
from rb import *
r=Robot('away')
zh=np.array([0,0,-1.0]); yh=np.array([1.0,0,0]); R=R_from(zh,yh)
q=r.ik(np.array([0.10,0.32,1.30])-0.1034*zh,R)
print(r.move(q,5)); print(r.fk()[0].round(3))
EOF
timeout 120 python3 cloud.py birdview && python3 - <<'EOF'
import numpy as np
P=np.load('birdview_xyz.npy').reshape(-1,3); P=P[np.isfinite(P[:,0])]
Q=P[(P[:,2]>0.985)&(P[:,2]<1.03)]
w=Q[(Q[:,0]>-0.19)&(Q[:,0]<-0.02)&(Q[:,1]>0.08)&(Q[:,1]<0.26)]
print('white pts',len(w),'x',w[:,0].min().round(3),w[:,0].max().round(3),'y',w[:,1].min().round(3),w[:,1].max().round(3),'z max',w[:,2].max().round(3))
cx=(w[:,0].min()+w[:,0].max())/2
xb=w[(w[:,0]>cx-0.01)&(w[:,0]<cx+0.01)]; print(' y at center x band',xb[:,1].min().round(3),xb[:,1].max().round(3))
cy=(xb[:,1].min()+xb[:,1].max())/2
yb=w[(w[:,1]>cy-0.01)&(w[:,1]<cy+0.01)]; print(' x at center y band',yb[:,0].min().round(3),yb[:,0].max().round(3))
y2=Q[(Q[:,0]>-0.06)&(Q[:,0]<0.06)&(Q[:,1]>-0.08)&(Q[:,1]<0.08)]
print('yellow x',y2[:,0].min().round(3),y2[:,0].max().round(3),'y',y2[:,1].min().round(3),y2[:,1].max().round(3))
h=P[(P[:,0]>-0.19)&(P[:,0]<-0.02)&(P[:,1]>cy+0.05)&(P[:,1]<cy+0.12)&(P[:,2]>0.92)&(P[:,2]<1.0)]
print('handle pts',len(h), 'y max', h[:,1].max().round(3) if len(h) else None, 'x', (h[:,0].min().round(3),h[:,0].max().round(3)) if len(h) else None)
EOF

# openrua op 88
timeout 60 python3 -c "
from rb import *
import cv2
r=Robot('s'); r.snap('agentview','agent_now.png'); r.snap('birdview','bird_now.png')
im=cv2.imread('agent_now.png'); cv2.imwrite('crop_now.png', cv2.resize(im[120:400,150:520],None,fx=2,fy=2,interpolation=cv2.INTER_CUBIC))
"

# openrua op 89
timeout 120 python3 cloud.py agentview && timeout 120 python3 cloud.py frontview && python3 - <<'EOF'
import numpy as np
for cam in ('agentview','frontview'):
    P=np.load(f'{cam}_xyz.npy').reshape(-1,3); P=P[np.isfinite(P[:,0])]
    w=P[(P[:,0]>-0.19)&(P[:,0]<-0.02)&(P[:,1]>0.08)&(P[:,1]<0.26)&(P[:,2]>0.98)&(P[:,2]<1.03)]
    print(cam,'white rim pts',len(w),'x',w[:,0].min().round(3),w[:,0].max().round(3),'y',w[:,1].min().round(3),w[:,1].max().round(3),'z max',w[:,2].max().round(3))
    for z0 in (0.93,0.95,0.97):
        s=P[(P[:,0]>-0.19)&(P[:,0]<-0.02)&(P[:,1]>0.08)&(P[:,1]<0.26)&(P[:,2]>z0)&(P[:,2]<z0+0.02)]
        print(f'  z{z0} n{len(s)} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]')
EOF

# openrua op 90
python3 - <<'EOF'
import numpy as np
P=np.load('agentview_xyz.npy').reshape(-1,3); P=P[np.isfinite(P[:,0])]
m=(P[:,0]>-0.20)&(P[:,0]<0.0)&(P[:,1]>0.06)&(P[:,1]<0.28)&(P[:,2]>0.95)&(P[:,2]<1.03)
Q=P[m]
xs=np.arange(-0.20,0.0,0.005); ys=np.arange(0.06,0.28,0.005)
H,_,_=np.histogram2d(Q[:,0],Q[:,1],[xs,ys])
print('       y: '+''.join(str(int(round(y*100))%10) if abs(round(y*100)-y*100)<0.01 and int(round(y*100))%2==0 else ' ' for y in ys[:-1]))
for i,x in enumerate(xs[:-1]):
    print(f'{x:+.3f} '+''.join('#' if H[i,j]>2 else ('.' if H[i,j]>0 else ' ') for j in range(len(ys)-1)))
EOF

# openrua op 91
python3 - <<'EOF'
import numpy as np
for cam in ('agentview','frontview'):
    P=np.load(f'{cam}_xyz.npy').reshape(-1,3); P=P[np.isfinite(P[:,0])]
    Q=P[(P[:,0]>-0.18)&(P[:,0]<-0.03)&(P[:,1]>0.08)&(P[:,1]<0.25)&(P[:,2]>1.004)&(P[:,2]<1.03)]
    print(cam,'rim pts',len(Q),'x',Q[:,0].min().round(3),Q[:,0].max().round(3),'y',Q[:,1].min().round(3),Q[:,1].max().round(3))
    cx=(Q[:,0].min()+Q[:,0].max())/2
    b=Q[np.abs(Q[:,0]-cx)<0.01]; print('  center x',round(cx,3),' y at center band',b[:,1].min().round(3),b[:,1].max().round(3))
    cy=(b[:,1].min()+b[:,1].max())/2
    b2=Q[np.abs(Q[:,1]-cy)<0.01]; print('  center y',round(cy,3),' x at center band',b2[:,0].min().round(3),b2[:,0].max().round(3))
    # handle: points beyond +y rim, z<1.0
    h=P[(P[:,0]>cx-0.03)&(P[:,0]<cx+0.03)&(P[:,1]>cy+0.05)&(P[:,1]<cy+0.12)&(P[:,2]>0.92)&(P[:,2]<1.0)]
    print('  handle n',len(h),'y max',h[:,1].max().round(3) if len(h) else None,'z',(h[:,2].min().round(3),h[:,2].max().round(3)) if len(h) else None)
EOF

# openrua op 92
mkdir -p "$(dirname /workspace/insert3.py)"
cat > /workspace/insert3.py <<'OPENRUA_EOF'
from rb import *
from clear import hand_clearance
r = Robot('ins3')
b = np.radians(45)
zh = np.array([0, np.sin(b), -np.cos(b)]); yh = np.array([0, np.cos(b), np.sin(b)]); R = R_from(zh, yh)
def flange(t): return t - 0.1034 * zh
xm, ym, rim = -0.104, 0.150, 1.012
yo = ym - 0.046
g = np.array([xm, yo + 0.0093, rim - 0.02])
XT, YT = -0.07, 0.30                      # target mug centre in cavity
print('open', r.grip(0.08))
w0 = None
def go(t, sec, tag='', fmax=8, seed=None):
    global w0
    q = r.ik(flange(t), R, seed=seed)
    if q is None: raise SystemExit('ik fail ' + tag)
    jump = np.abs(q - r.arm_q()).max()
    code, err = r.move(q, sec)
    if err > 0.01: code, err = r.move(q, 2)
    p = r.fk()[0]; w = r.wrench()
    if w0 is None: w0 = w
    print(tag, 'tcp', (p + 0.1034 * zh).round(3), 'code', code, 'err', round(err, 3), 'jump', round(jump, 2), 'dW', (w - w0).round(1), 'fing', np.round(r.fingers(), 4), 'clr', round(hand_clearance(p + 0.1034 * zh, R, 0.013), 3))
    if np.abs(w - w0)[:3].max() > fmax: raise SystemExit('force!')
    return q
go([g[0], g[1], 1.07], 5, 'above')
go(g, 3, 'grasp pose')
print('close', r.grip(0.0)); r.spin(0.5); print('fingers', np.round(r.fingers(), 4))
wg = r.wrench(); print('dW after grasp', (wg - w0).round(1))
z1 = 1.055
go([g[0], g[1], z1], 1.2, 'lift', fmax=12)
go([XT, YT - 0.0367 - 0.08, z1], 1.5, 'approach', fmax=12)      # mug far edge just before ledge
go([XT, YT - 0.0367, z1], 1.2, 'in', fmax=12)
go([XT, YT - 0.0367, 1.034], 0.8, 'down', fmax=15)
print('open', r.grip(0.08))
go([XT, YT - 0.0367 - 0.06, 1.05], 2, 'back')
go([XT, 0.12, 1.15], 3, 'up')
r.snap('agentview', 'agent_ins3.png'); r.snap('frontview', 'front_ins3.png')
OPENRUA_EOF

# openrua op 93
python3 - <<'EOF'
import re
s=open('insert3.py').read()
s=s.replace("go([XT, YT - 0.0367, 1.034], 0.8, 'down', fmax=15)\n","go([XT, YT - 0.0367, 1.034], 0.8, 'down', fmax=15)\ngo([XT, YT - 0.0367 - 0.01, 1.034], 0.5, 'drag', fmax=15)\n")
open('insert3.py','w').write(s)
EOF
grep -n "drag" insert3.py && timeout 900 python3 insert3.py 2>&1 | tail -20

# openrua op 94
timeout 60 ros2 param get /move_group robot_description 2>/dev/null > urdf.txt; wc -c urdf.txt; grep -n -i -A6 "<link name=\"panda_hand\"\|<link name=\"panda_leftfinger\"\|<link name=\"panda_link7\"\|<link name=\"panda_link8\"" urdf.txt | grep -i "mesh\|box\|cylinder\|origin\|link name" | head -40

# openrua op 95
python3 - <<'EOF'
import numpy as np, struct
def load_stl(fn):
    d=open(fn,'rb').read()
    if d[:5]==b'solid' and b'facet' in d[:300]:
        import re
        v=np.array([list(map(float,m)) for m in re.findall(rb'vertex\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)',d)])
        return v
    n=struct.unpack('<I',d[80:84])[0]
    a=np.frombuffer(d[84:84+n*50],dtype=np.dtype([('n','<3f4'),('v','<9f4'),('a','<u2')]))
    return a['v'].reshape(-1,3).astype(float)
base='/opt/ros/jazzy/share/moveit_resources_panda_description/meshes/collision/'
for f in ('hand.stl','finger.stl','link7.stl'):
    v=load_stl(base+f); print(f, 'n',len(v),'min',v.min(0).round(4),'max',v.max(0).round(4))
h=load_stl(base+'hand.stl')
# extents of hand vs z (hand frame z = approach)
for z0 in np.arange(-0.01,0.07,0.01):
    s=h[(h[:,2]>=z0)&(h[:,2]<z0+0.01)]
    if len(s): print(f'hand z[{z0:.2f},{z0+0.01:.2f}) x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]')
fg=load_stl(base+'finger.stl')
for z0 in np.arange(0.0,0.06,0.01):
    s=fg[(fg[:,2]>=z0)&(fg[:,2]<z0+0.01)]
    if len(s): print(f'finger z[{z0:.2f},{z0+0.01:.2f}) x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]')
EOF

# openrua op 96
find / -name "hand.stl" -o -name "finger.stl" -o -name "hand.dae" -o -name "hand*.obj" 2>/dev/null | head; ls /opt/ros/jazzy/share 2>/dev/null | grep -i panda

# openrua op 97
python3 - <<'EOF'
import numpy as np
for cam in ('agentview','frontview'):
    P=np.load(f'{cam}_xyz.npy').reshape(-1,3); P=P[np.isfinite(P[:,0])]
    s=P[(P[:,0]>-0.12)&(P[:,0]<0.0)&(P[:,1]>0.22)&(P[:,1]<0.34)&(P[:,2]>1.06)]
    print(cam)
    for y0 in np.arange(0.22,0.34,0.01):
        t=s[(s[:,1]>=y0)&(s[:,1]<y0+0.01)]
        if len(t): print(f'  y[{y0:.2f}] n{len(t):4d} z[{t[:,2].min():.3f},{t[:,2].max():.3f}]')
EOF

# openrua op 98
sed -i 's/for c in np.linspace(0.045, 0.045 + 0.058, 4):/for c in np.linspace(0.033, 0.045 + 0.058, 5):/; s/for b in np.linspace(-0.1, 0.1, 9):/for b in np.linspace(-0.105, 0.105, 9):/' clear.py && python3 - <<'EOF'
import numpy as np
from rb import R_from
from clear import hand_clearance
for beta in (45,50,55,60):
    b=np.radians(beta); zh=np.array([0,np.sin(b),-np.cos(b)]); yh=np.array([0,np.cos(b),np.sin(b)]); R=R_from(zh,yh)
    for ym in (0.28,0.29,0.30,0.31):
        ok=[z for z in np.arange(1.00,1.10,0.002) if hand_clearance([-0.07,ym-0.0367,z],R,g=0.013)>=0.004]
        if ok: print(f'beta {beta} y_mug {ym}: TCP z {min(ok):.3f}..{max(ok):.3f} -> mug bottom(D=.02) {min(ok)+0.02-0.108:.3f}..{max(ok)+0.02-0.108:.3f}')
        else: print(f'beta {beta} y_mug {ym}: none')
EOF

# openrua op 99
python3 - <<'EOF'
import numpy as np
from rb import R_from
from clear import hand_clearance
for beta in (0,10,20,30,35,40):
    b=np.radians(beta); zh=np.array([0,np.sin(b),-np.cos(b)]); yh=np.array([0,np.cos(b),np.sin(b)]); R=R_from(zh,yh)
    for ym in (0.27,0.28,0.29,0.30):
        ok=[z for z in np.arange(1.00,1.10,0.002) if hand_clearance([-0.07,ym-0.040,z],R,g=0.006)>=0.004]
        if ok: print(f'beta {beta} y_mug {ym}: TCP z {min(ok):.3f}..{max(ok):.3f} -> mug bottom(D=.014) {min(ok)+0.014-0.108:.3f}')
        else: print(f'beta {beta} y_mug {ym}: none')
EOF

# openrua op 100
mkdir -p "$(dirname /workspace/insert4.py)"
cat > /workspace/insert4.py <<'OPENRUA_EOF'
from rb import *
from clear import hand_clearance
import sys
r = Robot('ins4')
BETA = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
b = np.radians(BETA)
zh = np.array([0, np.sin(b), -np.cos(b)]); yh = np.array([0, np.cos(b), np.sin(b)]); R = R_from(zh, yh)
def flange(t): return t - 0.1034 * zh
xm, ym, rim = -0.104, 0.150, 1.012
D = 0.016
g = np.array([xm, ym - 0.046 + 0.004, rim - D])
XT, YT = -0.07, 0.285
ZH = 1.056
print('open', r.grip(0.08))
w0 = None
def go(t, sec, tag='', fmax=8, seed=None):
    global w0
    q = r.ik(flange(t), R, seed=seed)
    if q is None: raise SystemExit('ik fail ' + tag)
    jump = np.abs(q - r.arm_q()).max()
    code, err = r.move(q, sec)
    if err > 0.01: code, err = r.move(q, 2)
    p = r.fk()[0]; w = r.wrench()
    if w0 is None: w0 = w
    f = r.fingers()
    print(tag, 'tcp', (p + 0.1034 * zh).round(3), 'code', code, 'err', round(err, 3), 'jump', round(jump, 2), 'dW', (w - w0).round(1), 'fing', np.round(f, 4), 'clr', round(hand_clearance(p + 0.1034 * zh, R, 0.006), 3))
    if np.abs(w - w0)[:3].max() > fmax: raise SystemExit('force!')
    return q, f
go([g[0], g[1], 1.09], 5, 'above')
go(g, 3, 'grasp pose')
print('close', r.grip(0.0)); r.spin(0.3); print('fingers', np.round(r.fingers(), 4))
_, f = go([g[0], g[1], ZH], 1.2, 'lift', fmax=12)
if f[0] < 0.0015:
    print('SLIPPED to lip -> set down'); go(g, 1.2, 'setdown'); r.grip(0.08); go([g[0], g[1], 1.09], 3, 'up'); raise SystemExit
r.snap('frontview', 'front_hang.png'); r.snap('agentview', 'agent_hang.png')
_, f = go([XT, YT - 0.042 - 0.07, ZH], 1.6, 'approach', fmax=12)
r.snap('frontview', 'front_appr.png')
_, f = go([XT, YT - 0.042, ZH], 1.0, 'in', fmax=12)
print('open', r.grip(0.08))
go([XT, YT - 0.042 - 0.06, ZH + 0.01], 2, 'back')
go([XT, 0.12, 1.15], 3, 'up')
r.snap('agentview', 'agent_ins4.png'); r.snap('frontview', 'front_ins4.png')
OPENRUA_EOF

# openrua op 101
timeout 900 python3 insert4.py 30 2>&1 | tail -20

# openrua op 102
sed -i 's/for c in np.linspace(0.033, 0.045 + 0.058, 5):/for c in np.linspace(0.045, 0.045 + 0.058, 4):/' clear.py && python3 - <<'EOF'
import numpy as np
from rb import R_from
from clear import hand_clearance
for beta in (0,10,20,30):
    b=np.radians(beta); zh=np.array([0,np.sin(b),-np.cos(b)]); yh=np.array([0,np.cos(b),np.sin(b)]); R=R_from(zh,yh)
    for ym in (0.28,0.29,0.30):
        ok=[z for z in np.arange(1.00,1.10,0.002) if hand_clearance([-0.07,ym-0.040,z],R,g=0.006)>=0.004]
        if ok: print(f'beta {beta} y_mug {ym}: TCP z {min(ok):.3f}..{max(ok):.3f} -> rim(D=.014) {min(ok)+0.014:.3f} bottom {min(ok)+0.014-0.108:.3f}')
        else: print(f'beta {beta} y_mug {ym}: none')
EOF

# openrua op 103
python3 cloud.py agentview >/dev/null 2>&1; python3 - <<'EOF'
import numpy as np
P=np.load('agentview_xyz.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,2]>1.004)&(P[:,2]<1.03)&(P[:,0]<-0.03)&(P[:,1]>0.05)&(P[:,1]<0.26)
Q=P[m]; print(len(Q))
if len(Q):
    print('x',Q[:,0].min().round(3),Q[:,0].max().round(3),'y',Q[:,1].min().round(3),Q[:,1].max().round(3),'zmax',Q[:,2].max().round(3))
    # center via extents at band
    cx=(Q[:,0].min()+Q[:,0].max())/2; cy=(Q[:,1].min()+Q[:,1].max())/2; print('center est',round(cx,3),round(cy,3))
EOF

# openrua op 104
python3 - <<'EOF'
import numpy as np
P=np.load('agentview_xyz.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,2]>1.004)&(P[:,2]<1.02)&(P[:,0]<-0.03)&(P[:,0]>-0.18)&(P[:,1]>0.05)&(P[:,1]<0.24)
Q=P[m]; print(len(Q))
print('x',Q[:,0].min().round(3),Q[:,0].max().round(3),'y',Q[:,1].min().round(3),Q[:,1].max().round(3))
# y extents in a central x band, x extents in a central y band
for cx in (-0.104,):
    b=Q[np.abs(Q[:,0]-cx)<0.01]; print('y range at x~cx',b[:,1].min().round(3),b[:,1].max().round(3))
for cy in (0.15,):
    b=Q[np.abs(Q[:,1]-cy)<0.01]; print('x range at y~cy',b[:,0].min().round(3),b[:,0].max().round(3))
# side wall points (z 0.95-1.0) to check
m2=(P[:,2]>0.95)&(P[:,2]<1.0)&(P[:,0]<-0.03)&(P[:,0]>-0.18)&(P[:,1]>0.05)&(P[:,1]<0.24)
Q2=P[m2]; print('wall x',Q2[:,0].min().round(3),Q2[:,0].max().round(3),'y',Q2[:,1].min().round(3),Q2[:,1].max().round(3))
EOF

# openrua op 105
python3 -c "
from rb import *; r=Robot('s'); r.snap('agentview','agent_now.png'); r.snap('birdview','bird_now.png')" && python3 -c "
from PIL import Image; im=Image.open('agent_now.png'); print(im.size); im.crop((0,0,im.size[0],im.size[1])).resize((im.size[0]*2,im.size[1]*2)).save('agent_now2.png')"

# openrua op 106
python3 cloud.py birdview >/dev/null 2>&1; python3 - <<'EOF'
import numpy as np
P=np.load('birdview_xyz.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,2]>1.0)&(P[:,2]<1.03)&(P[:,0]<-0.03)&(P[:,0]>-0.18)&(P[:,1]>0.05)&(P[:,1]<0.24)
Q=P[m]; print(len(Q),'x',Q[:,0].min().round(3),Q[:,0].max().round(3),'y',Q[:,1].min().round(3),Q[:,1].max().round(3))
# fit circle by least squares on rim points
x,y=Q[:,0],Q[:,1]; A=np.c_[2*x,2*y,np.ones_like(x)]; b=x**2+y**2
c=np.linalg.lstsq(A,b,rcond=None)[0]; cx,cy=c[0],c[1]; rr=np.sqrt(c[2]+cx**2+cy**2); print('circle',round(cx,3),round(cy,3),round(rr,3))
# extents at centre bands
bx=Q[np.abs(Q[:,1]-cy)<0.008]; by=Q[np.abs(Q[:,0]-cx)<0.008]
print('x ext',bx[:,0].min().round(3),bx[:,0].max().round(3),'y ext',by[:,1].min().round(3),by[:,1].max().round(3))
# hand occlusion? points above 1.05 in region
h=P[(P[:,2]>1.04)&(P[:,0]<-0.03)&(P[:,0]>-0.2)&(P[:,1]>0.0)&(P[:,1]<0.25)]; print('hand pts y range',h[:,1].min().round(3) if len(h) else None,h[:,1].max().round(3) if len(h) else None)
EOF

# openrua op 107
cat eih_cloud.py | head -40

# openrua op 108
python3 - <<'EOF'
from rb import *
import eih_cloud
r=Robot('look')
zh=np.array([0,0,-1.0]); yh=np.array([1.0,0,0]); R=R_from(zh,yh)
q=r.ik(np.array([-0.104,0.15,1.32])-0.1034*zh,R)
print('ik',q is not None)
print(r.move(q,4))
print('tcp',(r.fk()[0]+0.1034*zh).round(3))
eih_cloud.cloud(r,'eih_look')
EOF
python3 - <<'EOF'
import numpy as np
P=np.load('eih_look_xyz.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,2]>1.0)&(P[:,2]<1.03)&(P[:,0]<-0.03)&(P[:,0]>-0.18)&(P[:,1]>0.05)&(P[:,1]<0.24)
Q=P[m]; print(len(Q),'x',Q[:,0].min().round(3),Q[:,0].max().round(3),'y',Q[:,1].min().round(3),Q[:,1].max().round(3))
x,y=Q[:,0],Q[:,1]; A=np.c_[2*x,2*y,np.ones_like(x)]; b=x**2+y**2
c=np.linalg.lstsq(A,b,rcond=None)[0]; cx,cy=c[0],c[1]; rr=np.sqrt(c[2]+cx**2+cy**2); print('circle',round(cx,3),round(cy,3),round(rr,3))
bx=Q[np.abs(Q[:,1]-cy)<0.006]; by=Q[np.abs(Q[:,0]-cx)<0.006]
print('x ext',bx[:,0].min().round(3),bx[:,0].max().round(3),'y ext',by[:,1].min().round(3),by[:,1].max().round(3))
print('rim z', np.percentile(Q[:,2],[50,90,99]).round(3))
# handle
h=P[(P[:,2]>0.93)&(P[:,2]<1.0)&(P[:,0]<-0.03)&(P[:,0]>-0.18)&(P[:,1]>cy+0.05)&(P[:,1]<0.26)]
print('handle pts',len(h),'y max',h[:,1].max().round(3) if len(h) else None,'x',h[:,0].min().round(3) if len(h) else None,h[:,0].max().round(3) if len(h) else None,'z',h[:,2].min().round(3) if len(h) else None,h[:,2].max().round(3) if len(h) else None)
EOF

# openrua op 109
python3 - <<'EOF'
import numpy as np
P=np.load('eih_look_xyz.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,2]>1.005)&(P[:,2]<1.02)&(P[:,0]<-0.03)&(P[:,0]>-0.18)&(P[:,1]>0.05)&(P[:,1]<0.3)
Q=P[m]; print(len(Q),'x',Q[:,0].min().round(3),Q[:,0].max().round(3),'y',Q[:,1].min().round(3),Q[:,1].max().round(3))
x,y=Q[:,0],Q[:,1]; A=np.c_[2*x,2*y,np.ones_like(x)]; b=x**2+y**2
c=np.linalg.lstsq(A,b,rcond=None)[0]; cx,cy=c[0],c[1]; rr=np.sqrt(c[2]+cx**2+cy**2); print('circle',round(cx,3),round(cy,3),round(rr,3))
for cyy in (cy,):
    bx=Q[np.abs(Q[:,1]-cyy)<0.006]; print('x ext',bx[:,0].min().round(3),bx[:,0].max().round(3))
for cxx in (cx,-0.105):
    by=Q[np.abs(Q[:,0]-cxx)<0.006]; print('y ext at x',round(cxx,3),by[:,1].min().round(3),by[:,1].max().round(3))
# angular histogram of rim points around fitted center
ang=np.degrees(np.arctan2(y-cy,x-cx)); print('angles present', np.histogram(ang,bins=12,range=(-180,180))[0])
h=P[(P[:,2]>0.92)&(P[:,2]<1.0)&(P[:,0]<-0.03)&(P[:,0]>-0.18)&(P[:,1]>cy+0.05)&(P[:,1]<0.3)]
print('handle pts',len(h),'y max',h[:,1].max().round(3),'x',h[:,0].min().round(3),h[:,0].max().round(3),'z',h[:,2].min().round(3),h[:,2].max().round(3))
# where is anything else at z>1.0 in y 0.24-0.30 (ledge/door?)
EOF

# openrua op 110
python3 - <<'EOF'
import re
s=open('insert4.py').read()
s=s.replace("BETA = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0","BETA = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0")
s=s.replace("xm, ym, rim = -0.104, 0.150, 1.012","xm, ym, rim = -0.106, 0.191, 1.012")
s=s.replace("D = 0.016","D = float(sys.argv[2]) if len(sys.argv) > 2 else 0.014")
s=s.replace("XT, YT = -0.07, 0.285","XT, YT = -0.075, 0.287")
s=s.replace("""_, f = go([XT, YT - 0.042 - 0.07, ZH], 1.6, 'approach', fmax=12)
r.snap('frontview', 'front_appr.png')""","""_, f = go([XT, YT - 0.042 - 0.07, ZH], 1.6, 'approach', fmax=12)
r.snap('frontview', 'front_appr.png')
if f[0] < 0.0012:
    print('SLIPPED during approach -> set down here'); go([XT, YT - 0.042 - 0.07, 1.012], 1.2, 'setdown2'); r.grip(0.08); go([XT, YT - 0.042 - 0.07, 1.09], 3, 'up'); raise SystemExit""")
open('insert4.py','w').write(s)
EOF
grep -n "BETA =\|xm, ym\|D =\|XT, YT\|ZH =" insert4.py

# openrua op 111
python3 - <<'EOF'
s=open('insert4.py').read()
s=s.replace("""    print('SLIPPED to lip -> set down'); go(g, 1.2, 'setdown'); r.grip(0.08); go([g[0], g[1], 1.09], 3, 'up'); raise SystemExit""",
"""    if f[0] > 0.0003:
        print('SLIPPED to lip -> set down'); go([g[0], g[1], 1.010], 1.2, 'setdown')
    else:
        print('mug lost'); 
    r.grip(0.08); go([g[0], g[1], 1.09], 3, 'up'); raise SystemExit""")
open('insert4.py','w').write(s)
EOF
python3 insert4.py 20 0.014 2>&1 | grep -v "^\[" 

# openrua op 112
python3 -c "
from rb import *; r=Robot('s'); r.snap('agentview','agent_col.png'); r.snap('frontview','front_col.png'); print(r.fingers(), r.wrench().round(1))
from PIL import Image
for n in ('front_appr','agent_col','front_col'):
    im=Image.open(n+'.png'); im.crop((150,80,640,400)).resize((980,640)).save(n+'_c.png')"

# openrua op 113
python3 - <<'EOF'
from rb import *
r=Robot('rec')
b=np.radians(20); zh=np.array([0,np.sin(b),-np.cos(b)]); yh=np.array([0,np.cos(b),np.sin(b)]); R=R_from(zh,yh)
def flange(t): return t-0.1034*zh
w0=r.wrench()
for t,sec,tag in (([-0.075,0.175,1.06],1.5,'back'),):
    q=r.ik(flange(t),R); print(r.move(q,sec)); p=r.fk()[0]; print(tag,(p+0.1034*zh).round(3),'W',r.wrench().round(1),'fing',np.round(r.fingers(),4))
r.snap('frontview','front_back.png'); r.snap('agentview','agent_back.png')
from PIL import Image
for n in ('front_back','agent_back'):
    im=Image.open(n+'.png'); im.crop((150,80,640,400)).resize((980,640)).save(n+'_c.png')
EOF

# openrua op 114
cat > insert5.py <<'EOF'
from rb import *
from clear import hand_clearance
import sys
r = Robot('ins5')
BETA = 25.0; b = np.radians(BETA)
zh = np.array([0, np.sin(b), -np.cos(b)]); yh = np.array([0, np.cos(b), np.sin(b)]); RF = R_from(zh, yh)       # final frame
Rz90 = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1.0]])
R0 = Rz90 @ RF                                                                                                 # grasp frame (fingers down & -x)
w0 = None
def go(t, sec, R, tag='', fmax=8, seed=None, check=True):
    global w0
    zh_ = R[:, 2]
    q = r.ik(np.asarray(t) - 0.1034 * zh_, R, seed=seed)
    if q is None: raise SystemExit('ik fail ' + tag)
    jump = np.abs(q - r.arm_q()).max()
    code, err = r.move(q, sec)
    if err > 0.01: code, err = r.move(q, 2)
    p, Rc = r.fk(); w = r.wrench()
    if w0 is None: w0 = w
    r.spin(0.2); f = r.fingers()
    print(tag, 'tcp', (p + 0.1034 * zh_).round(3), 'code', code, 'err', round(err, 3), 'jump', round(jump, 2), 'dW', (w - w0).round(1), 'fing', np.round(f, 4), 'clr', round(hand_clearance(p + 0.1034 * zh_, R, 0.006), 3), flush=True)
    if check and np.abs(w - w0)[:3].max() > fmax: raise SystemExit('force! ' + tag)
    return q, f, w - w0

stage = sys.argv[1] if len(sys.argv) > 1 else 'all'
if stage in ('setdown', 'all'):
    # currently hanging mug at beta=20 frame; set down at (-0.09,0.172)
    b2 = np.radians(20); z2 = np.array([0, np.sin(b2), -np.cos(b2)]); y2 = np.array([0, np.cos(b2), np.sin(b2)]); R2 = R_from(z2, y2)
    go([-0.09, 0.130, 1.06], 1.5, R2, 'move back', fmax=12)
    go([-0.09, 0.130, 1.012], 1.5, R2, 'lower', fmax=12, check=False)
    print('open', r.grip(0.08))
    go([-0.09, 0.130, 1.10], 2, R2, 'up')
    w0 = None
    r.snap('agentview', 'agent_sd5.png')
EOF
python3 insert5.py setdown 2>&1 | grep -v "^\["

# openrua op 115
python3 - <<'EOF'
from rb import *
import eih_cloud
r=Robot('look')
zh=np.array([0,0,-1.0]); yh=np.array([1.0,0,0]); R=R_from(zh,yh)
q=r.ik(np.array([-0.09,0.17,1.33])-0.1034*zh,R); r.move(q,3); print('tcp',(r.fk()[0]+0.1034*zh).round(3))
eih_cloud.cloud(r,'eih_look')
import numpy as np
P=np.load('eih_look_xyz.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,2]>1.005)&(P[:,2]<1.02)&(P[:,0]<-0.02)&(P[:,0]>-0.18)&(P[:,1]>0.05)&(P[:,1]<0.26)
Q=P[m]; print(len(Q),'x',Q[:,0].min().round(3),Q[:,0].max().round(3),'y',Q[:,1].min().round(3),Q[:,1].max().round(3))
cx=(Q[:,0].min()+Q[:,0].max())/2; cy=(Q[:,1].min()+Q[:,1].max())/2
bx=Q[np.abs(Q[:,1]-cy)<0.006]; by=Q[np.abs(Q[:,0]-cx)<0.006]
print('x ext',bx[:,0].min().round(3),bx[:,0].max().round(3),'y ext',by[:,1].min().round(3),by[:,1].max().round(3))
print('rim z pct', np.percentile(Q[:,2],[10,50,90]).round(3))
h=P[(P[:,2]>0.92)&(P[:,2]<1.0)&(P[:,0]<0.0)&(P[:,0]>-0.2)&(P[:,1]>0.05)&(P[:,1]<0.27)]
h=h[np.hypot(h[:,0]-cx,h[:,1]-cy)>0.05]
print('handle pts',len(h),'x',h[:,0].min().round(3),h[:,0].max().round(3),'y',h[:,1].min().round(3),h[:,1].max().round(3))
EOF

# openrua op 116
python3 - <<'EOF'
import numpy as np
P=np.load('eih_look_xyz.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
cx,cy=-0.0945,0.1875
d=np.hypot(P[:,0]-cx,P[:,1]-cy)
h=P[(P[:,2]>0.925)&(P[:,2]<0.995)&(d>0.052)&(d<0.09)]
print(len(h)); 
if len(h):
    ang=np.degrees(np.arctan2(h[:,1]-cy,h[:,0]-cx)); print(np.histogram(ang,bins=12,range=(-180,180))[0])
    print('x',h[:,0].min().round(3),h[:,0].max().round(3),'y',h[:,1].min().round(3),h[:,1].max().round(3))
EOF

# openrua op 117
python3 - <<'EOF'
import numpy as np
from rb import R_from
from clear import hand_clearance
b=np.radians(25); zh=np.array([0,np.sin(b),-np.cos(b)]); yh=np.array([0,np.cos(b),np.sin(b)]); R=R_from(zh,yh)
for tcp in ([-0.075,0.183,1.012],[-0.075,0.183,1.057],[-0.075,0.213,1.057],[-0.075,0.248,1.05],[-0.075,0.248,1.046],[-0.075,0.258,1.048]):
    print(tcp, round(hand_clearance(tcp,R,0.006),3))
EOF

# openrua op 118
cat >> insert5.py <<'EOF'
if stage in ('grasp', 'all'):
    cx, cy, rim = -0.0945, 0.1875, 1.012
    D = 0.014
    g = np.array([cx + 0.046 - 0.004, cy, rim - D])
    print('open', r.grip(0.08))
    go([g[0], g[1], 1.10], 4, R0, 'above')
    go(g, 3, R0, 'grasp pose')
    print('close', r.grip(0.0)); r.spin(0.3); print('fingers', np.round(r.fingers(), 4))
    _, f, _ = go([g[0], g[1], 1.06], 1.2, R0, 'lift', fmax=12)
    if f[0] < 0.0015:
        print('slipped'); 
        if f[0] > 0.0003: go([g[0], g[1], 1.010], 1.2, R0, 'setdown')
        r.grip(0.08); go([g[0], g[1], 1.10], 3, R0, 'up'); raise SystemExit
    # yaw -90 while moving: mug centre -> (-0.075, 0.17); TCP = mug - 0.042 y
    q, f, _ = go([-0.075, 0.170 - 0.042, 1.06], 2.5, RF, 'yaw+move', fmax=12)
    r.snap('frontview', 'front_yaw.png'); r.snap('agentview', 'agent_yaw.png')
    if f[0] < 0.0012: print('slipped after yaw'); go([-0.075, 0.128, 1.010], 1.5, RF, 'setdown'); r.grip(0.08); go([-0.075, 0.128, 1.10], 3, RF, 'up'); raise SystemExit
    go([-0.075, 0.128, 1.012], 1.5, RF, 'lower to table', fmax=15, check=False)
    _, f, dw = go([-0.075, 0.175, 1.012], 1.5, RF, 'slide y1', fmax=15, check=False)
    _, f, dw = go([-0.075, 0.186, 1.012], 0.8, RF, 'slide y2', fmax=15, check=False)
    r.snap('frontview', 'front_slide.png'); r.snap('agentview', 'agent_slide.png')
    _, f, dw = go([-0.075, 0.186, 1.058], 1.5, RF, 'lift along face', fmax=15, check=False)
    print('fingers', np.round(f, 4))
    if f[0] < 0.0010: print('slipped on lift'); go([-0.075, 0.186, 1.012], 1.2, RF, 'setdown'); r.grip(0.08); go([-0.075, 0.186, 1.10], 3, RF, 'up'); raise SystemExit
    _, f, dw = go([-0.075, 0.216, 1.058], 1.0, RF, 'over ledge', fmax=15, check=False)
    r.snap('frontview', 'front_ledge.png'); r.snap('agentview', 'agent_ledge.png')
    _, f, dw = go([-0.075, 0.250, 1.056], 1.2, RF, 'in', fmax=20, check=False)
    _, f, dw = go([-0.075, 0.250, 1.048], 0.6, RF, 'down', fmax=20, check=False)
    print('open', r.grip(0.08))
    go([-0.075, 0.190, 1.06], 1.5, RF, 'back', check=False)
    go([-0.075, 0.12, 1.15], 3, RF, 'up', check=False)
    r.snap('agentview', 'agent_ins5.png'); r.snap('frontview', 'front_ins5.png')
EOF
python3 insert5.py grasp 2>&1 | grep -v "^\["

# openrua op 119
python3 -c "
from rb import *; r=Robot('s'); r.snap('agentview','agent_x.png'); r.snap('frontview','front_x.png'); print(r.arm_q().round(2)); print(r.wrench().round(1)); p,R=r.fk(); print(p.round(3)); print(R.round(2))"

# openrua op 120
python3 - <<'EOF'
from rb import *
r=Robot('rec')
p,R=r.fk(); q0=r.arm_q()
for dz in (0.06,0.1):
    q=r.ik(p+np.array([0,0,dz]),R,seed=q0)
    print(dz, None if q is None else np.abs(q-q0).max().round(2))
    if q is not None and np.abs(q-q0).max()<0.6:
        print(r.move(q,2)); print('W',r.wrench().round(1),'p',r.fk()[0].round(3)); break
EOF

# openrua op 121
python3 - <<'EOF'
from rb import *
r=Robot('t')
q0=r.arm_q(); print('q0',q0.round(2))
b=np.radians(25)
zh=np.array([0,np.sin(b),-np.cos(b)]); yh=np.array([0,np.cos(b),np.sin(b)]); RF=R_from(zh,yh)
Rz90=np.array([[0,-1,0],[1,0,0],[0,0,1.0]]); R0=Rz90@RF
flip=np.diag([-1,-1,1.0])
g=np.array([-0.0525,0.1875,0.998])
for name,R in (('R0',R0),('R0flip',R0@flip),('RF',RF),('RFflip',RF@flip)):
    for seed in (q0, BASE if 'BASE' in dir() else None):
        q=r.ik(g+np.array([0,0,0.1])-0.1034*R[:,2],R,seed=seed)
        print(name, 'seed', 'cur' if seed is q0 else 'base', None if q is None else (q.round(2), np.abs(q-q0).max().round(2)))
EOF

# openrua op 122
python3 - <<'EOF'
s=open('insert5.py').read()
s=s.replace("R0 = Rz90 @ RF ","flip = np.diag([-1, -1, 1.0])\nRF = RF @ flip\nR0 = Rz90 @ RF ")
s=s.replace("""    jump = np.abs(q - r.arm_q()).max()
    code, err = r.move(q, sec)""","""    jump = np.abs(q - r.arm_q()).max()
    if jump > 1.6: raise SystemExit(f'jump too big {jump:.2f} ' + tag)
    code, err = r.move(q, sec)""")
open('insert5.py','w').write(s)
EOF
python3 - <<'EOF'
from rb import *
r=Robot('t')
b=np.radians(25)
zh=np.array([0,np.sin(b),-np.cos(b)]); yh=np.array([0,np.cos(b),np.sin(b)]); RF=R_from(zh,yh)@np.diag([-1,-1,1.0])
Rz90=np.array([[0,-1,0],[1,0,0],[0,0,1.0]]); R0=Rz90@RF
g=np.array([-0.0525,0.1875,0.998])
plan=[(R0,[g[0],g[1],1.10]),(R0,g),(R0,[g[0],g[1],1.06]),(RF,[-0.075,0.128,1.06]),(RF,[-0.075,0.128,1.012]),(RF,[-0.075,0.186,1.012]),(RF,[-0.075,0.186,1.058]),(RF,[-0.075,0.216,1.058]),(RF,[-0.075,0.25,1.056]),(RF,[-0.075,0.25,1.048]),(RF,[-0.075,0.19,1.06]),(RF,[-0.075,0.12,1.15])]
qp=r.arm_q()
for R,t in plan:
    q=r.ik(np.array(t)-0.1034*R[:,2],R,seed=qp)
    print(t, None if q is None else (np.abs(q-qp).max().round(2), q.round(2)))
    if q is not None: qp=q
EOF

# openrua op 123
python3 cloud.py agentview >/dev/null 2>&1; python3 - <<'EOF'
import numpy as np
P=np.load('agentview_xyz.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,2]>1.0)&(P[:,2]<1.03)&(P[:,0]<-0.02)&(P[:,0]>-0.17)&(P[:,1]>0.1)&(P[:,1]<0.25)
Q=P[m]; print(len(Q),'x',Q[:,0].min().round(3),Q[:,0].max().round(3),'y',Q[:,1].min().round(3),Q[:,1].max().round(3),'z',np.percentile(Q[:,2],[10,50,90]).round(3))
by=Q[np.abs(Q[:,0]+0.0945)<0.008]; print('y ext',by[:,1].min().round(3),by[:,1].max().round(3))
bx=Q[np.abs(Q[:,1]-0.1875)<0.008]; print('x ext',bx[:,0].min().round(3),bx[:,0].max().round(3))
EOF

# openrua op 124
python3 insert5.py grasp 2>&1 | grep -v "^\["

# openrua op 125
python3 -c "
from rb import *; r=Robot('s'); r.snap('agentview','agent_r.png'); r.snap('frontview','front_r.png')
from PIL import Image
for n in ('agent_r','front_r','front_ledge','agent_ledge'):
    im=Image.open(n+'.png'); im.crop((250,100,640,400)).resize((1040,800)).save(n+'_c.png')"

# openrua op 126
python3 cloud.py agentview >/dev/null 2>&1; python3 cloud.py frontview >/dev/null 2>&1; python3 - <<'EOF'
import numpy as np
for cam in ('agentview','frontview'):
    P=np.load(f'{cam}_xyz.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
    m=(P[:,2]>1.0)&(P[:,2]<1.03)&(P[:,0]<0.0)&(P[:,0]>-0.17)&(P[:,1]>0.1)&(P[:,1]<0.30)
    Q=P[m]; print(cam,len(Q),'x',Q[:,0].min().round(3),Q[:,0].max().round(3),'y',Q[:,1].min().round(3),Q[:,1].max().round(3),'z',np.percentile(Q[:,2],[10,50,90]).round(3))
    cx=(Q[:,0].min()+Q[:,0].max())/2
    by=Q[np.abs(Q[:,0]-cx)<0.01]; print(' y ext at cx',round(cx,3),by[:,1].min().round(3),by[:,1].max().round(3))
    # handle: z 0.93-0.99 outside radius
    cy=(by[:,1].min()+by[:,1].max())/2
    d=np.hypot(P[:,0]-cx,P[:,1]-cy); h=P[(P[:,2]>0.925)&(P[:,2]<0.995)&(d>0.052)&(d<0.1)&(P[:,1]<0.268)]
    if len(h): print(' handle', len(h), 'ang hist', np.histogram(np.degrees(np.arctan2(h[:,1]-cy,h[:,0]-cx)),bins=8,range=(-180,180))[0], 'x max', h[:,0].max().round(3))
EOF

# openrua op 127
python3 - <<'EOF'
import numpy as np
for cam in ('agentview','frontview'):
    P=np.load(f'{cam}_xyz.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
    m=(P[:,2]>1.003)&(P[:,2]<1.02)&(P[:,0]<0.0)&(P[:,0]>-0.17)&(np.abs(P[:,1]-0.213)<0.01)
    Q=P[m]; print(cam,'x ext at y~0.213',Q[:,0].min().round(3),Q[:,0].max().round(3))
    # circle fit on rim ring
    m=(P[:,2]>1.003)&(P[:,2]<1.02)&(P[:,0]<-0.03)&(P[:,0]>-0.16)&(P[:,1]>0.16)&(P[:,1]<0.262)
    Q=P[m]; x,y=Q[:,0],Q[:,1]; A=np.c_[2*x,2*y,np.ones_like(x)]; b=x**2+y**2
    c=np.linalg.lstsq(A,b,rcond=None)[0]; print(' circle',round(c[0],3),round(c[1],3),round(np.sqrt(c[2]+c[0]**2+c[1]**2),3))
EOF

# openrua op 128
cat >> insert5.py <<'EOF'
if stage == 'final':
    cx, cy, rim = -0.075, 0.213, 1.012
    D = 0.014
    g = np.array([cx, cy - 0.046 + 0.004, rim - D])
    print('open', r.grip(0.08))
    go([g[0], g[1], 1.10], 3, RF, 'above')
    go(g, 3, RF, 'grasp pose')
    print('close', r.grip(0.0)); r.spin(0.3); print('fingers', np.round(r.fingers(), 4))
    _, f, dw = go([g[0], 0.184, g[2]], 1.0, RF, 'slide to face', fmax=15, check=False)
    _, f, dw = go([g[0], 0.186, 1.072], 1.5, RF, 'lift along face', fmax=15, check=False)
    if f[0] < 0.0010: print('slipped on lift'); go([g[0], 0.186, 1.010], 1.2, RF, 'setdown'); r.grip(0.08); go([g[0], 0.186, 1.10], 3, RF, 'up'); raise SystemExit
    _, f, dw = go([g[0], 0.216, 1.072], 0.8, RF, 'over ledge', fmax=15, check=False)
    r.snap('frontview', 'front_ledge.png')
    _, f, dw = go([g[0], 0.250, 1.062], 1.0, RF, 'in', fmax=20, check=False)
    r.snap('frontview', 'front_in.png')
    _, f, dw = go([g[0], 0.250, 1.056], 0.5, RF, 'down', fmax=20, check=False)
    print('open partial', r.grip(0.02)); r.spin(0.3); print('fingers', np.round(r.fingers(), 4))
    go([g[0], 0.190, 1.062], 1.5, RF, 'back', check=False)
    print('open', r.grip(0.08))
    go([g[0], 0.12, 1.15], 3, RF, 'up', check=False)
    r.snap('agentview', 'agent_ins5.png'); r.snap('frontview', 'front_ins5.png')
EOF
python3 insert5.py final 2>&1 | grep -v "^\["

# openrua op 129
python3 -c "
from rb import *; r=Robot('s'); r.snap('agentview','agent_r.png'); r.snap('frontview','front_r.png')
from PIL import Image
for n in ('agent_r','front_r','front_ledge','front_in'):
    im=Image.open(n+'.png'); im.crop((250,100,640,400)).resize((1040,800)).save(n+'_c.png')"

# openrua op 130
python3 - <<'EOF'
import numpy as np
from rb import R_from
from clear import hand_clearance
for beta in (25,30,35,40,45):
    b=np.radians(beta); zh=np.array([0,np.sin(b),-np.cos(b)]); yh=np.array([0,np.cos(b),np.sin(b)]); R=R_from(zh,yh)
    for ym in (0.29,0.30):
        ok=[z for z in np.arange(1.00,1.10,0.002) if hand_clearance([-0.075,ym-0.040,z],R,g=0.006)>=0.006]
        print(f'beta {beta} y_mug {ym}: TCP z min {min(ok):.3f} -> rim(D=.03) {min(ok)+0.03:.3f} bottom {min(ok)+0.03-0.108:.3f}')
EOF

# openrua op 131
python3 cloud.py agentview >/dev/null 2>&1; python3 cloud.py frontview >/dev/null 2>&1; python3 - <<'EOF'
import numpy as np
for cam in ('agentview','frontview'):
    P=np.load(f'{cam}_xyz.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
    # points inside cavity region
    m=(P[:,1]>0.272)&(P[:,1]<0.43)&(P[:,0]>-0.16)&(P[:,0]<0.05)&(P[:,2]>0.95)&(P[:,2]<1.085)
    Q=P[m]; print(cam,len(Q))
    if len(Q):
        print(' x',Q[:,0].min().round(3),Q[:,0].max().round(3),'y',Q[:,1].min().round(3),Q[:,1].max().round(3),'z',Q[:,2].min().round(3),Q[:,2].max().round(3))
        hist=np.histogram(Q[:,2],bins=np.arange(0.95,1.09,0.01)); print(' z hist',hist[0])
        top=Q[Q[:,2]>1.045]; 
        if len(top): print(' top pts x',top[:,0].min().round(3),top[:,0].max().round(3),'y',top[:,1].min().round(3),top[:,1].max().round(3),'z med',np.median(top[:,2]).round(3))
EOF

# openrua op 132
python3 - <<'EOF'
import numpy as np
for cam in ('agentview','frontview'):
    P=np.load(f'{cam}_xyz.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
    m=(P[:,1]>0.20)&(P[:,1]<0.272)&(P[:,0]>-0.16)&(P[:,0]<0.05)&(P[:,2]>0.95)&(P[:,2]<1.07)
    Q=P[m]; print(cam,len(Q))
    if len(Q):
        print(' x',Q[:,0].min().round(3),Q[:,0].max().round(3),'y',Q[:,1].min().round(3),Q[:,1].max().round(3),'z',Q[:,2].min().round(3),Q[:,2].max().round(3))
        for zlo in (0.95,1.0,1.04):
            s=Q[(Q[:,2]>zlo)&(Q[:,2]<zlo+0.02)]
            if len(s): print(f'  z {zlo}: y min {s[:,1].min():.3f} x range {s[:,0].min():.3f}..{s[:,0].max():.3f} n {len(s)}')
    # rim ring inside: z 1.045-1.06, all y
    R=P[(P[:,2]>1.045)&(P[:,2]<1.062)&(P[:,0]>-0.16)&(P[:,0]<0.05)&(P[:,1]>0.2)&(P[:,1]<0.45)]
    print(' rim-height pts: y',R[:,1].min().round(3),R[:,1].max().round(3),'x',R[:,0].min().round(3),R[:,0].max().round(3))
    bx=R[np.abs(R[:,0]+0.075)<0.01]; print('  y at x~-0.075',bx[:,1].min().round(3) if len(bx) else None,bx[:,1].max().round(3) if len(bx) else None)
EOF

# openrua op 133
python3 - <<'EOF'
from rb import *
import eih_cloud
r=Robot('look')
zh=np.array([0,1.0,0]); yh=np.array([0,0,1.0]); R=R_from(zh,yh)
q0=r.arm_q()
for t in ([-0.075,0.06,1.02],[-0.06,0.05,1.03],[-0.09,0.04,1.02]):
    q=r.ik(np.array(t)-0.1034*zh,R,seed=q0)
    print(t, None if q is None else np.abs(q-q0).max().round(2))
    if q is not None and np.abs(q-q0).max()<2.0:
        print(r.move(q,4)); print('tcp',(r.fk()[0]+0.1034*zh).round(3)); break
eih_cloud.cloud(r,'eih_cav')
EOF

# openrua op 134
python3 - <<'EOF'
from rb import *
import eih_cloud
r=Robot('look')
zh=np.array([0,1.0,0]); yh=np.array([0,0,-1.0]); R=R_from(zh,yh)
q0=r.arm_q()
for t in ([-0.075,0.06,1.02],[-0.06,0.05,1.03],[-0.09,0.04,1.02]):
    q=r.ik(np.array(t)-0.1034*zh,R,seed=q0)
    print(t, None if q is None else np.abs(q-q0).max().round(2))
    if q is not None and np.abs(q-q0).max()<2.0:
        print(r.move(q,4)); print('tcp',(r.fk()[0]+0.1034*zh).round(3)); break
eih_cloud.cloud(r,'eih_cav')
EOF

# openrua op 135
python3 - <<'EOF'
import numpy as np
P=np.load('eih_cav_xyz.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,1]>0.26)&(P[:,1]<0.42)&(P[:,0]>-0.17)&(P[:,0]<0.06)&(P[:,2]>0.94)&(P[:,2]<1.09)
Q=P[m]; print(len(Q),'x',Q[:,0].min().round(3),Q[:,0].max().round(3),'y',Q[:,1].min().round(3),Q[:,1].max().round(3),'z',Q[:,2].min().round(3),Q[:,2].max().round(3))
print('z hist',np.histogram(Q[:,2],bins=np.arange(0.94,1.10,0.01))[0])
# near wall: for z slices, min y at central x
for zlo in np.arange(0.95,1.07,0.02):
    s=Q[(Q[:,2]>zlo)&(Q[:,2]<zlo+0.02)&(Q[:,1]<0.40)]
    if len(s):
        cx=np.median(s[:,0]); c=s[np.abs(s[:,0]-cx)<0.01]
        print(f'z {zlo:.2f}: n {len(s)} x {s[:,0].min():.3f}..{s[:,0].max():.3f} y min {s[:,1].min():.3f}  y at median x({cx:.3f}) {c[:,1].min():.3f}')
top=Q[Q[:,2]>1.045]; print('top x',top[:,0].min().round(3),top[:,0].max().round(3),'y',top[:,1].min().round(3),top[:,1].max().round(3))
EOF

# openrua op 136
python3 - <<'EOF'
import numpy as np
P=np.load('eih_cav_xyz.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,1]>0.25)&(P[:,1]<0.41)&(P[:,0]>-0.15)&(P[:,0]<0.04)&(P[:,2]>0.955)&(P[:,2]<1.08)
Q=P[m]; print(len(Q))
print('z hist',np.histogram(Q[:,2],bins=np.arange(0.95,1.09,0.01))[0])
for zlo in np.arange(0.96,1.07,0.01):
    s=Q[(Q[:,2]>zlo)&(Q[:,2]<zlo+0.01)]
    if len(s): print(f'z {zlo:.2f}: n {len(s)} x {s[:,0].min():.3f}..{s[:,0].max():.3f} y {s[:,1].min():.3f}..{s[:,1].max():.3f}')
# rim ring: top 
top=Q[Q[:,2]>1.048]
x,y=top[:,0],top[:,1]; A=np.c_[2*x,2*y,np.ones_like(x)]; b=x**2+y**2
c=np.linalg.lstsq(A,b,rcond=None)[0]; print('rim circle',round(c[0],3),round(c[1],3),round(np.sqrt(c[2]+c[0]**2+c[1]**2),3),'n',len(top))
EOF

# openrua op 137
python3 - <<'EOF'
from rb import *
r=Robot('t')
q0=r.arm_q(); print('q0',q0.round(2))
for yh in (np.array([1.0,0,0]),np.array([-1.0,0,0])):
    zh=np.array([0,1.0,0]); R=R_from(zh,yh)
    qp=q0
    for t in ([-0.06,0.20,0.985],[-0.06,0.29,0.985],[-0.06,0.15,0.985],[-0.06,0.15,1.10]):
        q=r.ik(np.array(t)-0.1034*zh,R,seed=qp)
        print('yh',yh, t, None if q is None else (np.abs(q-qp).max().round(2), q.round(2)))
        if q is not None: qp=q
EOF

# openrua op 138
cat > push2.py <<'EOF'
from rb import *
r = Robot('push2')
zh = np.array([0, 1.0, 0]); yh = np.array([-1.0, 0, 0]); R = R_from(zh, yh)
print('close', r.grip(0.0))
w0 = None
def go(t, sec, tag, fmax=15):
    global w0
    q = r.ik(np.asarray(t) - 0.1034 * zh, R, seed=r.arm_q())
    if q is None: raise SystemExit('ik fail ' + tag)
    jump = np.abs(q - r.arm_q()).max()
    if jump > 1.7: raise SystemExit(f'jump {jump:.2f} ' + tag)
    code, err = r.move(q, sec)
    if err > 0.01: code, err = r.move(q, 2)
    p = r.fk()[0]; w = r.wrench()
    if w0 is None: w0 = w
    print(tag, 'tcp', (p + 0.1034 * zh).round(3), 'code', code, 'err', round(err, 3), 'jump', round(jump, 2), 'dW', (w - w0).round(1), flush=True)
    if np.abs(w - w0)[:3].max() > fmax: raise SystemExit('force! ' + tag)
go([-0.06, 0.15, 0.985], 5, 'pre')
go([-0.06, 0.20, 0.985], 1.0, 'approach')
r.snap('agentview', 'agent_push_a.png')
go([-0.06, 0.29, 0.985], 2.5, 'push', fmax=25)
go([-0.06, 0.15, 0.985], 1.5, 'back', fmax=25)
go([-0.06, 0.15, 1.10], 2, 'up', fmax=25)
r.snap('agentview', 'agent_push_b.png')
EOF
python3 push2.py 2>&1 | grep -v "^\["

# openrua op 139
python3 -c "
from PIL import Image
for n in ('agent_push_a','agent_push_b'):
    im=Image.open(n+'.png'); im.crop((150,80,640,420)).resize((980,680)).save(n+'_c.png')"

# openrua op 140
python3 - <<'EOF'
from rb import *
import eih_cloud
r=Robot('look')
zh=np.array([0,1.0,0]); yh=np.array([-1.0,0,0]); R=R_from(zh,yh)
q=r.ik(np.array([-0.07,0.13,1.06])-0.1034*zh,R,seed=r.arm_q()); print(np.abs(q-r.arm_q()).max().round(2)); print(r.move(q,2))
eih_cloud.cloud(r,'eih_cav2')
import numpy as np
P=np.load('eih_cav2_xyz.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,1]>0.25)&(P[:,1]<0.42)&(P[:,0]>-0.15)&(P[:,0]<0.04)&(P[:,2]>0.955)&(P[:,2]<1.08)
Q=P[m]; print(len(Q))
for zlo in np.arange(0.96,1.07,0.01):
    s=Q[(Q[:,2]>zlo)&(Q[:,2]<zlo+0.01)]
    if len(s): print(f'z {zlo:.2f}: n {len(s)} x {s[:,0].min():.3f}..{s[:,0].max():.3f} y {s[:,1].min():.3f}..{s[:,1].max():.3f}')
print('zmax',Q[:,2].max().round(3))
EOF

# openrua op 141
python3 -c "
from PIL import Image; im=Image.open('eih_cav2.png'); print(im.size); im.save('eih_cav2_v.png')"

# openrua op 142
python3 - <<'EOF'
s=open('push2.py').read()
s=s.replace("go([-0.06, 0.15, 0.985], 5, 'pre')\n","")
s=s.replace("go([-0.06, 0.29, 0.985], 2.5, 'push', fmax=25)","go([-0.06, 0.305, 0.985], 2.5, 'push', fmax=25)")
open('push2.py','w').write(s)
EOF
python3 push2.py 2>&1 | grep -v "^\["

# openrua op 143
python3 - <<'EOF'
from rb import *
import eih_cloud
r=Robot('look')
zh=np.array([0,1.0,0]); yh=np.array([-1.0,0,0]); R=R_from(zh,yh)
q=r.ik(np.array([-0.07,0.13,1.06])-0.1034*zh,R,seed=r.arm_q()); print(r.move(q,2))
eih_cloud.cloud(r,'eih_cav3')
import numpy as np
P=np.load('eih_cav3_xyz.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,1]>0.25)&(P[:,1]<0.42)&(P[:,0]>-0.15)&(P[:,0]<0.04)&(P[:,2]>0.955)&(P[:,2]<1.08)
Q=P[m]; print(len(Q))
for zlo in np.arange(0.96,1.07,0.02):
    s=Q[(Q[:,2]>zlo)&(Q[:,2]<zlo+0.02)]
    if len(s): print(f'z {zlo:.2f}: n {len(s)} x {s[:,0].min():.3f}..{s[:,0].max():.3f} y {s[:,1].min():.3f}..{s[:,1].max():.3f}')
print('zmax',Q[:,2].max().round(3))
EOF

# openrua op 144
python3 - <<'EOF'
import numpy as np
P=np.load('eih_cav3_xyz.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
# frame face: region above opening (z 1.09-1.10) and below (z 0.91-0.94), x in opening range
for name,m in (('above',(P[:,2]>1.092)&(P[:,2]<1.105)&(P[:,0]>-0.15)&(P[:,0]<0.04)),('below',(P[:,2]>0.905)&(P[:,2]<0.94)&(P[:,0]>-0.15)&(P[:,0]<0.04)),('right side',(P[:,0]>0.055)&(P[:,0]<0.1)&(P[:,2]>0.95)&(P[:,2]<1.08))):
    s=P[m]; s=s[(s[:,1]>0.2)&(s[:,1]<0.5)]
    print(name,len(s),'y pct',np.percentile(s[:,1],[5,50,95]).round(3))
# cavity floor / back wall
s=P[(P[:,0]>-0.14)&(P[:,0]<0.03)&(P[:,1]>0.3)&(P[:,1]<0.5)&(P[:,2]>0.93)&(P[:,2]<0.96)]; print('floor z pct',np.percentile(s[:,2],[5,50,95]).round(3),'y range',s[:,1].min().round(3),s[:,1].max().round(3))
s=P[(P[:,0]>-0.14)&(P[:,0]<0.03)&(P[:,1]>0.40)&(P[:,2]>0.96)&(P[:,2]<1.08)]; print('back wall y pct',np.percentile(s[:,1],[5,50,95]).round(3))
EOF

# openrua op 145
sed -i "s/go(\[-0.06, 0.305, 0.985\], 2.5, 'push', fmax=25)/go([-0.062, 0.31, 0.985], 2.5, 'push', fmax=25)/" push2.py && python3 push2.py 2>&1 | grep -v "^\[" && python3 - <<'EOF'
from rb import *
import eih_cloud
r=Robot('look')
zh=np.array([0,1.0,0]); yh=np.array([-1.0,0,0]); R=R_from(zh,yh)
q=r.ik(np.array([-0.07,0.13,1.06])-0.1034*zh,R,seed=r.arm_q()); print(r.move(q,2))
eih_cloud.cloud(r,'eih_cav4')
P=np.load('eih_cav4_xyz.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,1]>0.25)&(P[:,1]<0.43)&(P[:,0]>-0.15)&(P[:,0]<0.04)&(P[:,2]>0.955)&(P[:,2]<1.08)
Q=P[m]; print(len(Q))
for zlo in np.arange(0.96,1.07,0.02):
    s=Q[(Q[:,2]>zlo)&(Q[:,2]<zlo+0.02)]
    if len(s): print(f'z {zlo:.2f}: n {len(s)} x {s[:,0].min():.3f}..{s[:,0].max():.3f} y {s[:,1].min():.3f}..{s[:,1].max():.3f}')
print('zmax',Q[:,2].max().round(3))
EOF

# openrua op 146
python3 -c "
from PIL import Image; im=Image.open('eih_cav4.png').rotate(180); im.crop((160,60,480,340)).resize((960,840)).save('eih_cav4_r.png')"

# openrua op 147
python3 - <<'EOF'
from rb import *
r=Robot('cal')
zh=np.array([0,0,-1.0]); yh=np.array([1.0,0,0]); R=R_from(zh,yh)
def flange(t): return np.asarray(t)-0.1034*zh
q=r.ik(flange([-0.25,0.05,1.0]),R,seed=r.arm_q()); print('jump',np.abs(q-r.arm_q()).max().round(2)); print(r.move(q,4))
w0=r.wrench()
for z in (0.95,0.93,0.92,0.91,0.90,0.89,0.88,0.87):
    q=r.ik(flange([-0.25,0.05,z]),R,seed=r.arm_q()); r.move(q,1.0)
    p=r.fk()[0]; w=r.wrench()-w0
    print('target',z,'tcp z',round(p[2]+0.1034,4),'dW',w[:3].round(1))
    if abs(w[2])>5: break
q=r.ik(flange([-0.25,0.05,1.0]),R,seed=r.arm_q()); r.move(q,2)
EOF

# openrua op 148
python3 -c "
from rb import *; r=Robot('s'); print(r.arm_q().round(2)); print(r.wrench().round(1)); p,R=r.fk(); print(p.round(3)); print(R.round(2)); r.snap('agentview','agent_x.png'); r.snap('frontview','front_x.png')"

# openrua op 149
python3 - <<'EOF'
from rb import *
r=Robot('rec')
p,R=r.fk(); q0=r.arm_q()
q=r.ik(p+np.array([0,0,0.12]),R,seed=q0); print('jump',np.abs(q-q0).max().round(2))
if np.abs(q-q0).max()<0.8:
    print(r.move(q,2)); print('W',r.wrench().round(1),'p',r.fk()[0].round(3))
r.snap('agentview','agent_x.png')
EOF

# openrua op 150
cat > door.py <<'EOF'
from rb import *
import sys
r = Robot('door')
zh = np.array([0, 0, -1.0]); yh = np.array([0, -1.0, 0]); R = R_from(zh, yh)
H = np.array([-0.19, 0.27]); RHO = 0.16; OFF = float(sys.argv[1]) if len(sys.argv) > 1 else 0.033
Z = 1.075
def tcp_at(th, off):
    d = np.array([np.sin(th), -np.cos(th)]); n = np.array([-np.cos(th), -np.sin(th)])
    p = H + RHO * d + off * n
    return np.array([p[0], p[1], Z])
w0 = None
def go(t, sec, tag, fmax=20):
    global w0
    q = r.ik(np.asarray(t) - 0.1034 * zh, R, seed=r.arm_q())
    if q is None: raise SystemExit('ik fail ' + tag)
    jump = np.abs(q - r.arm_q()).max()
    if jump > 1.2: raise SystemExit(f'jump {jump:.2f} ' + tag)
    code, err = r.move(q, sec)
    if err > 0.01: code, err = r.move(q, 2)
    p = r.fk()[0]; w = r.wrench()
    if w0 is None: w0 = w
    print(tag, 'tcp', (p + 0.1034 * zh).round(3), 'code', code, 'err', round(err, 3), 'jump', round(jump, 2), 'dW', (w - w0).round(1), flush=True)
    if np.abs(w - w0)[:3].max() > fmax: raise SystemExit('force! ' + tag)
print('close', r.grip(0.0))
go([-0.26, 0.11, 1.20], 4, 'above')
go([-0.26, 0.11, Z], 3, 'down')
go(tcp_at(0, 0.045), 1.5, 'near door')
for th in np.radians(np.arange(10, 91, 10)):
    go(tcp_at(th, OFF), 1.0, f'th {np.degrees(th):.0f}')
r.snap('agentview', 'agent_door.png'); r.snap('birdview', 'bird_door.png')
go(tcp_at(np.radians(90), 0.08), 1.5, 'release')
go([-0.05, 0.15, 1.25], 3, 'up')
r.snap('agentview', 'agent_door2.png'); r.snap('frontview', 'front_door2.png')
EOF
python3 - <<'EOF'
# dry-run IK feasibility & jumps
from rb import *
r=Robot('t')
zh=np.array([0,0,-1.0]); yh=np.array([0,-1.0,0]); R=R_from(zh,yh)
H=np.array([-0.19,0.27]); RHO=0.16; Z=1.075
def tcp_at(th,off):
    d=np.array([np.sin(th),-np.cos(th)]); n=np.array([-np.cos(th),-np.sin(th)]); p=H+RHO*d+off*n; return np.array([p[0],p[1],Z])
qp=r.arm_q()
pts=[[-0.26,0.11,1.20],[-0.26,0.11,Z],tcp_at(0,0.045)]+[tcp_at(np.radians(a),0.033) for a in range(10,91,10)]+[tcp_at(np.radians(90),0.08),[-0.05,0.15,1.25]]
for t in pts:
    q=r.ik(np.asarray(t)-0.1034*zh,R,seed=qp)
    print(np.round(t,3), None if q is None else np.abs(q-qp).max().round(2))
    if q is not None: qp=q
EOF

# openrua op 151
python3 cloud.py birdview >/dev/null 2>&1; python3 - <<'EOF'
import numpy as np
P=np.load('birdview_xyz.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,2]>1.09)&(P[:,2]<1.12)&(P[:,0]>-0.30)&(P[:,0]<-0.10)&(P[:,1]>-0.05)&(P[:,1]<0.27)
Q=P[m]; print(len(Q),'x',Q[:,0].min().round(3),Q[:,0].max().round(3),'y',Q[:,1].min().round(3),Q[:,1].max().round(3))
for ylo in np.arange(0.0,0.27,0.05):
    s=Q[(Q[:,1]>ylo)&(Q[:,1]<ylo+0.05)]
    if len(s): print(f'y {ylo:.2f}: x {s[:,0].min():.3f}..{s[:,0].max():.3f} n {len(s)}')
# handle region protruding -x?
h=P[(P[:,2]>0.95)&(P[:,2]<1.09)&(P[:,0]>-0.30)&(P[:,0]<-0.215)&(P[:,1]>-0.05)&(P[:,1]<0.27)]
print('outer-side pts (x<-0.215)',len(h), 'x',h[:,0].min().round(3) if len(h) else None,'y',h[:,1].min().round(3) if len(h) else None,h[:,1].max().round(3) if len(h) else None,'z',h[:,2].min().round(3) if len(h) else None,h[:,2].max().round(3) if len(h) else None)
EOF

# openrua op 152
python3 - <<'EOF'
import re
s=open('door.py').read()
s=s.replace("""go([-0.26, 0.11, 1.20], 4, 'above')
go([-0.26, 0.11, Z], 3, 'down')
go(tcp_at(0, 0.045), 1.5, 'near door')
for th in np.radians(np.arange(10, 91, 10)):
    go(tcp_at(th, OFF), 1.0, f'th {np.degrees(th):.0f}')""",
"""TH0 = np.radians(-35)
p0 = tcp_at(TH0, 0.045)
go([p0[0], p0[1], 1.20], 4, 'above')
go(p0, 3, 'down')
for th in np.radians(np.arange(-30, 91, 5)):
    go(tcp_at(th, OFF), 0.7, f'th {np.degrees(th):.0f}')""")
open('door.py','w').write(s)
EOF
python3 - <<'EOF'
from rb import *
import numpy as np
r=Robot('dry')
zh=np.array([0,0,-1.0]); yh=np.array([0,-1.0,0]); R=R_from(zh,yh)
H=np.array([-0.19,0.27]); RHO=0.16; Z=1.075
def tcp_at(th,off):
    d=np.array([np.sin(th),-np.cos(th)]); n=np.array([-np.cos(th),-np.sin(th)]); p=H+RHO*d+off*n; return np.array([p[0],p[1],Z])
q=r.arm_q(); 
pts=[('above',tcp_at(np.radians(-35),0.045)+[0,0,0.125]),('down',tcp_at(np.radians(-35),0.045))]+[(f'th{t}',tcp_at(np.radians(t),0.033)) for t in range(-30,91,5)]+[('rel',tcp_at(np.radians(90),0.08)),('up',[-0.05,0.15,1.25])]
for tag,t in pts:
    qq=r.ik(np.asarray(t)-0.1034*zh,R,seed=q)
    if qq is None: print(tag,'IK FAIL',np.round(t,3)); continue
    print(tag,np.round(t,3),'jump',round(float(np.abs(qq-q).max()),2)); q=qq
EOF

# openrua op 153
timeout 900 python3 door.py 2>&1 | grep -v "^\["

# openrua op 154
python3 cloud.py birdview >/dev/null 2>&1; python3 - <<'EOF'
import numpy as np
P=np.load('birdview_xyz.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,2]>1.09)&(P[:,2]<1.12)&(P[:,0]>-0.30)&(P[:,0]<0.10)&(P[:,1]>-0.05)&(P[:,1]<0.30)
Q=P[m]
for ylo in np.arange(0.0,0.30,0.025):
    s=Q[(Q[:,1]>ylo)&(Q[:,1]<ylo+0.025)]
    if len(s): print(f'y {ylo:.3f}: x {s[:,0].min():.3f}..{s[:,0].max():.3f} n {len(s)}')
# front region in front of face: anything at y<0.271 z 0.95-1.09 near x -0.19..0.07?
f=P[(P[:,2]>0.95)&(P[:,2]<1.09)&(P[:,1]>0.20)&(P[:,1]<0.271)&(P[:,0]>-0.25)&(P[:,0]<0.10)]
print('front pts', len(f), 'y', f[:,1].min().round(3) if len(f) else None, f[:,1].max().round(3) if len(f) else None)
EOF

# openrua op 155
python3 cloud.py frontview >/dev/null 2>&1; python3 - <<'EOF'
import numpy as np
P=np.load('frontview_xyz.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
# door panel: points in front of face plane y=0.271 within microwave x range, z 0.92-1.10
d=P[(P[:,1]<0.271)&(P[:,1]>0.15)&(P[:,0]>-0.22)&(P[:,0]<0.10)&(P[:,2]>0.93)&(P[:,2]<1.10)]
print('panel pts',len(d),'x',d[:,0].min().round(3),d[:,0].max().round(3),'z',d[:,2].min().round(3),d[:,2].max().round(3))
for xlo in np.arange(-0.22,0.10,0.04):
    s=d[(d[:,0]>xlo)&(d[:,0]<xlo+0.04)]
    if len(s): print(f'x {xlo:.2f}: y {np.percentile(s[:,1],5):.3f}..{np.percentile(s[:,1],95):.3f} n {len(s)}')
# anything still sticking out further than y<0.20 in front (open door would be at x -0.215..-0.19, y<0.27)
o=P[(P[:,1]<0.20)&(P[:,1]>-0.05)&(P[:,0]>-0.30)&(P[:,0]<-0.17)&(P[:,2]>0.95)&(P[:,2]<1.12)]
print('open-door-zone pts',len(o))
EOF

# openrua op 156
python3 - <<'EOF' 2>&1 | grep -v "^\["
from rb import *
r=Robot('fin'); import numpy as np
p,_=r.fk(); print('flange',np.round(p,3),'fingers',np.round(r.fingers(),4))
j=r.js(); v=getattr(j,'velocity',None); print('max |qdot|', np.abs(v).max().round(4) if v is not None and len(v) else 'n/a')
EOF
