#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat tools/perception/cam_snap.py tools/perception/px2world.py

# openrua op 2
cat tools/action/fjt_send.py tools/action/gripper_cmd.py tools/action/ik_move.py

# openrua op 3
timeout 30 ros2 topic list; echo ---; timeout 20 ros2 action list; echo ---; timeout 20 ros2 node list

# openrua op 4
timeout 60 python3 tools/perception/cam_snap.py agentview && timeout 60 python3 tools/perception/cam_snap.py birdview && timeout 60 python3 tools/perception/cam_snap.py frontview && timeout 30 ros2 topic echo /joint_states --once

# openrua op 5
for p in "80 290" "550 300" "200 235" "380 235" "315 185"; do echo -n "$p -> "; timeout 60 python3 tools/perception/px2world.py agentview $p; done

# openrua op 6
timeout 20 ros2 topic echo /tf_static --once 2>&1 | head -80; echo ---; timeout 15 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -12

# openrua op 7
timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png; timeout 20 ros2 topic echo /birdview/color/camera_info --once | grep -A12 "^k:"; timeout 15 ros2 run tf2_ros tf2_echo world birdview_optical_frame 2>&1 | grep -A3 Translation | head -5

# openrua op 8
python3 -c "
import numpy as np, cv2
d=np.load('birdview_depth.npy'); img=cv2.imread('birdview.png')
fx=579.4112549695428; cx=320; cy=240
# camera at world (-0.2,0,3.0), optical z down. Rotation q=(0.707,0.707,0,0): R maps cam->world
# cam x -> world y, cam y -> world x, cam z -> world -z   (check)
q=np.array([0.707107,0.707107,0,0]); x,y,z,w=q
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
print(R.round(3))
t=np.array([-0.2,0,3.0])
def world(u,v):
    Z=d[v,u]; p=np.array([(u-cx)*Z/fx,(v-cy)*Z/fx,Z]); return R@p+t
# height map: world z
H=np.zeros_like(d)
for v in range(0,480,1):
    Z=d[v]; 
    # vectorized
    us=np.arange(640); P=np.stack([(us-cx)*Z/fx,(v-cy)*Z/fx,Z],0); W=R@P+t[:,None]; H[v]=W[2]
np.save('birdview_H.npy',H)
tab=H[(H>0.47)&(H<0.50)]
print('table z median',np.median(tab))
mask=(H>0.50)&(H<0.7)
# exclude robot region: find connected components
m8=(mask*255).astype('uint8')
n,lab,stats,cent=cv2.connectedComponentsWithStats(m8)
for i in range(1,n):
    a=stats[i,cv2.CC_STAT_AREA]
    if a<30: continue
    u,v=cent[i]; ys,xs=np.where(lab==i)
    hz=H[lab==i]
    print(i,'area',a,'px',round(u),round(v),'world',world(int(u),int(v)).round(3),'zmax',hz.max().round(3),'z90',np.percentile(hz,90).round(3), 'color',img[int(v),int(u)])
"


# openrua op 9
python3 -c "
import numpy as np, cv2
H=np.load('birdview_H.npy'); img=cv2.imread('birdview.png')
mask=(H>0.50)
m8=(mask*255).astype('uint8')
n,lab,stats,cent=cv2.connectedComponentsWithStats(m8)
for i in range(1,n):
    a=stats[i,cv2.CC_STAT_AREA]
    if a<30: continue
    x,y,w,h=stats[i,:4]
    print(i,'area',a,'bbox',x,y,w,h,'zmax',H[lab==i].max().round(3))
# crop and upscale region around mugs
crop=img[220:320,220:420]; cv2.imwrite('bird_crop.png',cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST))
"; timeout 15 ros2 run tf2_ros tf2_echo world agentview_optical_frame 2>&1 | grep -A2 Translation | head -3

# openrua op 10
python3 -c "
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
rclpy.init(); n=rclpy.create_node('rd')
qos=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
got=[]
n.create_subscription(String,'/robot_description',got.append,qos)
import time
while not got: rclpy.spin_once(n,timeout_sec=0.5)
open('panda.urdf','w').write(got[0].data); print(len(got[0].data))
" && grep -o 'name="panda_[a-z0-9_]*"' panda.urdf | sort -u | head -40 && python3 -c "import PyKDL; print('kdl ok')"; python3 -c "import kdl_parser_py" 2>&1 | tail -1

# openrua op 11
grep -B2 -A12 '<joint name="panda_joint[1-8]"\|<joint name="panda_hand_joint"' panda.urdf | grep -E 'joint name|origin|axis|parent|child'

# openrua op 12
mkdir -p "$(dirname /workspace/rob.py)"
cat > /workspace/rob.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Helper library for this Panda workstation: FK, IK, trajectories, gripper,
joint state, world<->base conversion. Import from scripts in /workspace."""
import math
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
JOINTS = FJT["joints"]
LIMITS = FJT["limits_rad"]
TCP_OFF = float(M["hand"]["tcp_offset_m"])
# world -> panda_link0 (measured via tf2_echo): pure translation
W2B_T = np.array([-0.510, 0.0, 0.420])


def world_to_base(p):
    return np.asarray(p, float) - W2B_T


def base_to_world(p):
    return np.asarray(p, float) + W2B_T


# ---------------- kinematics ----------------
def _rot(rpy):
    r, p, y = rpy
    Rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    Ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    Rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def _T(xyz, rpy):
    T = np.eye(4)
    T[:3, :3] = _rot(rpy)
    T[:3, 3] = xyz
    return T


def _rz(q):
    T = np.eye(4)
    T[:3, :3] = _rot((0, 0, q))
    return T


# joint origins from URDF (parent->child), axis z for all revolute joints
_ORIGINS = [
    ((0, 0, 0.333), (0, 0, 0)),
    ((0, 0, 0), (-math.pi / 2, 0, 0)),
    ((0, -0.316, 0), (math.pi / 2, 0, 0)),
    ((0.0825, 0, 0), (math.pi / 2, 0, 0)),
    ((-0.0825, 0.384, 0), (-math.pi / 2, 0, 0)),
    ((0, 0, 0), (math.pi / 2, 0, 0)),
    ((0.088, 0, 0), (math.pi / 2, 0, 0)),
]
_T8 = _T((0, 0, 0.107), (0, 0, 0))
_THAND = _T((0, 0, 0), (0, 0, -math.pi / 4))


def fk_hand(q):
    """4x4 pose of panda_hand in panda_link0 frame."""
    T = np.eye(4)
    for (xyz, rpy), qi in zip(_ORIGINS, q):
        T = T @ _T(xyz, rpy) @ _rz(qi)
    return T @ _T8 @ _THAND


def fk_tcp(q):
    T = fk_hand(q)
    T = T.copy()
    T[:3, 3] += TCP_OFF * T[:3, 2]
    return T


def quat_from_R(R):
    """(x,y,z,w) from rotation matrix."""
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return np.array([(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s,
                         (R[1, 0] - R[0, 1]) / s, 0.25 * s])
    i = int(np.argmax(np.diag(R)))
    if i == 0:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return np.array([0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s])
    if i == 1:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        return np.array([(R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s])
    s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
    return np.array([(R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s, (R[1, 0] - R[0, 1]) / s])


def R_from_quat(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def top_down_R(yaw):
    """Hand rotation: z axis pointing down (-Z base), hand x axis rotated by
    yaw about base z. Fingers open along hand y."""
    c, s = math.cos(yaw), math.sin(yaw)
    # columns: x_hand, y_hand, z_hand in base frame
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, -1]]) @ np.array([[1, 0, 0], [0, -1, 0], [0, 0, 1]])
    # (the second matrix flips y so the frame stays right-handed with z down)


# ---------------- ROS client ----------------
class Robot:
    def __init__(self, name="rob"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])

    def _on_js(self, m):
        self._js = m

    def spin(self, t=0.05):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self, fresh=True):
        if fresh:
            self._js = None
        end = time.time() + 20
        while self._js is None and time.time() < end:
            self.spin(0.1)
        if self._js is None:
            raise RuntimeError("no /joint_states")
        d = dict(zip(self._js.name, self._js.position))
        return d

    def q(self, fresh=True):
        d = self.joints(fresh)
        return np.array([d[j] for j in JOINTS])

    def fingers(self, fresh=True):
        d = self.joints(fresh)
        return d["panda_finger_joint1"], d["panda_finger_joint2"]

    def tcp_world(self, q=None):
        if q is None:
            q = self.q()
        T = fk_tcp(q)
        return base_to_world(T[:3, 3])

    # ---- IK ----
    def ik_solve(self, pos_base, quat_xyzw, seed=None, timeout=60.0, attempts=1):
        if not self.ik.wait_for_service(timeout_sec=10):
            raise RuntimeError("IK service unavailable")
        if seed is None:
            seed = self.q()
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pos_base)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat_xyzw)
        js = JointState()
        js.name = list(JOINTS)
        js.position = [float(v) for v in seed]
        req.ik_request.robot_state.joint_state = js
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = 1
        for _ in range(attempts):
            fut = self.ik.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
            res = fut.result()
            if res is None:
                raise RuntimeError("IK timeout")
            if res.error_code.val == 1:
                sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
                return np.array([sol[j] for j in JOINTS])
        return None

    def ik_tcp(self, tcp_world_pos, R, seed=None, **kw):
        """IK for a TCP position (world frame) with hand rotation R (base frame)."""
        R = np.asarray(R, float)
        hand_base = world_to_base(tcp_world_pos) - TCP_OFF * R[:, 2]
        return self.ik_solve(hand_base, quat_from_R(R), seed=seed, **kw)

    # ---- motion ----
    def move_traj(self, points, times, wait=True):
        """points: list of joint vectors; times: cumulative seconds."""
        if not self.fjt.wait_for_server(timeout_sec=10):
            raise RuntimeError("no FJT server")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        for pt, t in zip(points, times):
            jp = JointTrajectoryPoint(positions=[float(v) for v in pt])
            jp.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(jp)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=30)
        gh = send.result()
        if gh is None or not gh.accepted:
            raise RuntimeError("trajectory goal rejected")
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        r = res.result()
        code = r.result.error_code if r else None
        qf = self.q()
        err = np.abs(qf - np.asarray(points[-1])).max()
        print(f"traj done code={code} max_joint_err={err:.4f}")
        return code, qf

    def move_to(self, qtarget, seconds=3.0):
        return self.move_traj([qtarget], [seconds])

    def gripper(self, width, timeout=120):
        if not self.grip.wait_for_server(timeout_sec=10):
            raise RuntimeError("no gripper server")
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = float(GRIP.get("max_effort", 30.0))
        fut = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        gh = fut.result()
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=timeout)
        r = res.result().result
        f = self.fingers()
        print(f"gripper -> reached={r.reached_goal} stalled={r.stalled} fingers={f}")
        return f

    def open(self):
        return self.gripper(GRIP["open_m"])

    def close(self):
        return self.gripper(GRIP["closed_m"])
OPENRUA_EOF

# openrua op 13
python3 - <<'EOF'
import re
s=open('rob.py').read()
s=s.replace('''    c, s = math.cos(yaw), math.sin(yaw)
    # columns: x_hand, y_hand, z_hand in base frame
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, -1]]) @ np.array([[1, 0, 0], [0, -1, 0], [0, 0, 1]])
    # (the second matrix flips y so the frame stays right-handed with z down)''','''    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, s, 0], [s, -c, 0], [0, 0, -1]])  # Rz(yaw) @ Rx(pi)''')
open('rob.py','w').write(s)
EOF
python3 -c "
import rob, numpy as np
q=np.array([0.0,-0.161037389,0.0,-2.44459747,0.0,2.2267522,0.7853981633974483])
T=rob.fk_hand(q); print('hand base', T[:3,3].round(4)); print(T[:3,:3].round(3)); print('quat', rob.quat_from_R(T[:3,:3]).round(4))
print('tcp world', rob.base_to_world(rob.fk_tcp(q)[:3,3]).round(4))
"; timeout 15 ros2 run tf2_ros tf2_echo panda_link0 panda_hand 2>&1 | grep -A1 Translation | head -2

# openrua op 14
python3 -c "
import rob, numpy as np
for q in [[0,-0.8,0,-2.2,0,1.4,0.785],[0,-1.0,0,-2.0,0,1.0,0.785],[0,-0.5,0,-2.0,0,1.5,0.785]]:
    print(q, 'tcp world', rob.base_to_world(rob.fk_tcp(np.array(q))[:3,3]).round(3))
"

# openrua op 15
timeout 600 python3 -u -c "
import rob, numpy as np
r=rob.Robot()
print('q0',r.q().round(3))
r.move_to(np.array([0,-1.0,0,-2.0,0,1.0,0.785]),4.0)
print('tcp world',r.tcp_world().round(3))
" && timeout 60 python3 tools/perception/cam_snap.py birdview && timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png

# openrua op 16
timeout 600 python3 -u -c "
import rob, numpy as np
r=rob.Robot()
print('q',r.q().round(3))
r.move_to(np.array([0,-1.0,0,-2.0,0,1.0,0.785]),4.0)
print('q',r.q().round(3),'tcp world',r.tcp_world().round(3))
"

# openrua op 17
timeout 60 python3 tools/perception/cam_snap.py birdview && timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png && python3 -c "
import numpy as np, cv2
d=np.load('birdview_depth.npy'); img=cv2.imread('birdview.png')
fx=579.4112549695428; cx=320; cy=240
R=np.array([[0,1,0],[1,0,0],[0,0,-1]]); t=np.array([-0.2,0,3.0])
us=np.arange(640)[None,:].repeat(480,0); vs=np.arange(480)[:,None].repeat(640,1)
P=np.stack([(us-cx)*d/fx,(vs-cy)*d/fx,d],0).reshape(3,-1); W=(R@P+t[:,None]).reshape(3,480,640)
X,Y,H=W
np.save('bird_W.npy',W)
print('table z', np.median(H[(H>0.47)&(H<0.50)]))
mask=(H>0.50)&(H<0.8)
n,lab,stats,cent=cv2.connectedComponentsWithStats((mask*255).astype('uint8'))
for i in range(1,n):
    a=stats[i,cv2.CC_STAT_AREA]
    if a<30: continue
    sel=lab==i
    # use the rim (top 1.5cm) for the centroid to avoid perspective bias
    hz=H[sel]; top=sel&(H>hz.max()-0.015)
    cxw,cyw=X[top].mean(),Y[top].mean()
    print(i,'area',a,'px',cent[i].round(0),'rim center world',round(cxw,3),round(cyw,3),'zmax',hz.max().round(3),'bbox',stats[i,:4], 'mean color BGR',img[sel].mean(0).round(0))
# plates: slight height ~ 0.49-0.50 and whitish
pm=(H>0.489)&(H<0.505)&(img.mean(2)>150)
n,lab,stats,cent=cv2.connectedComponentsWithStats((pm*255).astype('uint8'))
for i in range(1,n):
    a=stats[i,cv2.CC_STAT_AREA]
    if a<100: continue
    sel=lab==i
    print('plate?',i,'area',a,'px',cent[i].round(0),'world',round(X[sel].mean(),3),round(Y[sel].mean(),3),'z',round(H[sel].mean(),3))
"

# openrua op 18
python3 -c "
import numpy as np, cv2
img=cv2.imread('birdview.png'); W=np.load('bird_W.npy'); X,Y,H=W
hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
# plates: bright (V high), low saturation, on table (H around table height), outside mug regions
pm=(hsv[:,:,2]>140)&(hsv[:,:,1]<90)&(H<0.52)&(H>0.45)
pm=cv2.morphologyEx(pm.astype('uint8')*255,cv2.MORPH_OPEN,np.ones((3,3),np.uint8))
n,lab,stats,cent=cv2.connectedComponentsWithStats(pm)
for i in range(1,n):
    a=stats[i,cv2.CC_STAT_AREA]
    if a<80: continue
    sel=lab==i
    print(i,'area',a,'px',cent[i].round(0),'world',round(X[sel].mean(),3),round(Y[sel].mean(),3),'z',round(H[sel].mean(),3),'bbox',stats[i,:4])
cv2.imwrite('plates_mask.png',pm)
"

# openrua op 19
python3 -c "
import numpy as np, cv2
img=cv2.imread('birdview.png'); W=np.load('bird_W.npy'); X,Y,H=W
for (u,v) in [(243,283),(390,290),(300,300)]:
    print((u,v),'BGR',img[v,u],'world',X[v,u].round(3),Y[v,u].round(3),H[v,u].round(3))
hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
print('hsv plate',hsv[283,243],hsv[290,390],'table',hsv[300,300])
"

# openrua op 20
python3 -c "
import numpy as np
W=np.load('bird_W.npy'); X,Y,H=W
sel=(np.abs(X)<0.4)&(np.abs(Y)<0.5)
h,e=np.histogram(H[sel],bins=np.arange(0.35,0.7,0.01))
for hh,ee in zip(h,e): print(round(ee,2),hh)
"

# openrua op 21
python3 -c "
import numpy as np, cv2
img=cv2.imread('birdview.png'); W=np.load('bird_W.npy'); X,Y,H=W
hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
pm=((hsv[:,:,2]>150)&(hsv[:,:,1]<60)&(H<0.47)&(H>0.40)).astype('uint8')*255
pm=cv2.morphologyEx(pm,cv2.MORPH_CLOSE,np.ones((5,5),np.uint8))
n,lab,stats,cent=cv2.connectedComponentsWithStats(pm)
for i in range(1,n):
    a=stats[i,cv2.CC_STAT_AREA]
    if a<80: continue
    sel=lab==i
    print(i,'area',a,'px',cent[i].round(0),'world',round(X[sel].mean(),3),round(Y[sel].mean(),3),'z',round(H[sel].mean(),3),'bbox',stats[i,:4])
"

# openrua op 22
python3 -c "
import numpy as np, cv2
img=cv2.imread('birdview.png'); W=np.load('bird_W.npy'); X,Y,H=W
mask=(H>0.50)&(H<0.8)
n,lab,stats,cent=cv2.connectedComponentsWithStats((mask*255).astype('uint8'))
for i in range(1,n):
    a=stats[i,cv2.CC_STAT_AREA]
    if a<100 or stats[i,1]<200: continue
    sel=lab==i; hz=H[sel]
    for name,thr in [('all>0.50',0.50),('top2cm',hz.max()-0.02)]:
        s2=sel&(H>thr)
        xs,ys=X[s2],Y[s2]
        print(i,name,'n',s2.sum(),'x range',xs.min().round(3),xs.max().round(3),'y range',ys.min().round(3),ys.max().round(3),'center',xs.mean().round(3),ys.mean().round(3))
crop=img[220:320,240:380]; cv2.imwrite('bird_crop2.png',cv2.resize(crop,None,fx=6,fy=6,interpolation=cv2.INTER_NEAREST))
"

# openrua op 23
timeout 300 python3 -u -c "
import rob, numpy as np
r=rob.Robot()
R=rob.top_down_R(0.0)
tests={'white_pre':(-0.0945,-0.2235,0.70),'white_grasp':(-0.0945,-0.2235,0.52),'right_place':(0.015,0.25,0.56),'yellow_grasp':(-0.079,0.138,0.512),'left_place':(-0.024,-0.37,0.56)}
q0=r.q()
for k,p in tests.items():
    q=r.ik_tcp(p,R,seed=q0)
    if q is None: print(k,'IK FAIL'); continue
    print(k,q.round(3),'fk tcp',rob.base_to_world(rob.fk_tcp(q)[:3,3]).round(3))
"

# openrua op 24
timeout 300 python3 -u -c "
import rob, numpy as np
r=rob.Robot()
q0=r.q(); T=rob.fk_hand(q0)
print('current hand base',T[:3,3].round(3), rob.quat_from_R(T[:3,:3]).round(3))
q=r.ik_solve(T[:3,3], rob.quat_from_R(T[:3,:3]), seed=q0); print('ik current ->',None if q is None else q.round(3))
# home-ish pose
qh=np.array([0,-0.161,0,-2.445,0,2.227,0.785]); Th=rob.fk_hand(qh)
q=r.ik_solve(Th[:3,3], rob.quat_from_R(Th[:3,:3]), seed=q0); print('ik home ->',None if q is None else q.round(3))
q=r.ik_solve(Th[:3,3], (1,0,0,0), seed=qh); print('ik home straight down ->',None if q is None else q.round(3))
"

# openrua op 25
timeout 300 python3 -u -c "
import rob, numpy as np, rclpy
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
r=rob.Robot()
qh=np.array([0,-0.161,0,-2.445,0,2.227,0.785]); Th=rob.fk_hand(qh)
def call(pos,quat,seed,timeout_s=1,avoid=False,link=''):
    req=GetPositionIK.Request(); ik=req.ik_request
    ik.group_name='panda_arm'; ik.pose_stamped.header.frame_id=''
    ik.ik_link_name=link
    p=ik.pose_stamped.pose; p.position.x,p.position.y,p.position.z=map(float,pos)
    p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=map(float,quat)
    js=JointState(); js.name=list(rob.JOINTS); js.position=[float(v) for v in seed]
    ik.robot_state.joint_state=js; ik.avoid_collisions=avoid; ik.timeout.sec=timeout_s
    fut=r.ik.call_async(req); rclpy.spin_until_future_complete(r.node,fut,timeout_sec=60)
    res=fut.result(); print('code',res.error_code.val, dict(zip(res.solution.joint_state.name,np.round(res.solution.joint_state.position,3))) if res.error_code.val==1 else '')
call(Th[:3,3],rob.quat_from_R(Th[:3,:3]),qh)
call(Th[:3,3],rob.quat_from_R(Th[:3,:3]),qh,timeout_s=5)
call(Th[:3,3],rob.quat_from_R(Th[:3,:3]),qh,link='panda_hand')
call(Th[:3,3],rob.quat_from_R(Th[:3,:3]),qh,link='panda_link8')
"

# openrua op 26
timeout 300 python3 -u -c "
import rob, numpy as np
q0=np.array([0,-1.0,0,-2.001,0,1.002,0.785]); sol=np.array([-0.554,0.52,0.64,-1.748,-0.362,2.148,0.222])
T0=rob.fk_hand(q0); Ts=rob.fk_hand(sol)
print('target hand', T0[:3,3].round(3), rob.quat_from_R(T0[:3,:3]).round(3))
print('sol hand   ', Ts[:3,3].round(3), rob.quat_from_R(Ts[:3,:3]).round(3))
T8=rob.fk_hand(sol)@np.linalg.inv(rob._THAND)
print('sol link8  ', T8[:3,3].round(3), rob.quat_from_R(T8[:3,:3]).round(3))
"; timeout 20 ros2 param get /move_group robot_description_kinematics.panda_arm.kinematics_solver 2>&1 | tail -1; timeout 20 ros2 param list /move_group 2>&1 | grep -i kinematics | head

# openrua op 27
python3 - <<'EOF'
s=open('/workspace/rob.py').read()
s=s.replace('''    def ik_tcp(self, tcp_world_pos, R, seed=None, **kw):
        """IK for a TCP position (world frame) with hand rotation R (base frame)."""
        R = np.asarray(R, float)
        hand_base = world_to_base(tcp_world_pos) - TCP_OFF * R[:, 2]
        return self.ik_solve(hand_base, quat_from_R(R), seed=seed, **kw)''',
'''    def ik_tcp(self, tcp_world_pos, R, seed=None, **kw):
        """IK for a TCP position (world frame) with hand rotation R.
        Machine fact (measured): /compute_ik takes the pose in the WORLD
        frame and solves for panda_link8 (= panda_hand rotated +45deg about z)."""
        R = np.asarray(R, float)
        hand_world = np.asarray(tcp_world_pos, float) - TCP_OFF * R[:, 2]
        R8 = R @ _rot((0, 0, math.pi / 4))
        return self.ik_solve(hand_world, quat_from_R(R8), seed=seed, **kw)''')
open('/workspace/rob.py','w').write(s)
EOF
timeout 300 python3 -u -c "
import rob, numpy as np
r=rob.Robot()
qh=np.array([0,-0.161,0,-2.445,0,2.227,0.785]); Th=rob.fk_tcp(qh)
q=r.ik_tcp(rob.base_to_world(Th[:3,3]), Th[:3,:3], seed=r.q()); print('home via ik_tcp ->', None if q is None else q.round(3), 'expected', qh.round(3))
R=rob.top_down_R(0.0); q0=r.q()
tests={'white_pre':(-0.0945,-0.2235,0.70),'white_grasp':(-0.0945,-0.2235,0.52),'right_place':(0.015,0.25,0.56),'yellow_grasp':(-0.079,0.138,0.512),'left_place':(-0.024,-0.37,0.56)}
for k,p in tests.items():
    q=r.ik_tcp(p,R,seed=q0)
    if q is None: print(k,'IK FAIL'); continue
    print(k,q.round(3),'fk tcp world',rob.base_to_world(rob.fk_tcp(q)[:3,3]).round(3))
"

# openrua op 28
mkdir -p "$(dirname /workspace/pick_place.py)"
cat > /workspace/pick_place.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Rim-grasp pick and place for one mug.

Usage: python3 pick_place.py <grasp_x> <grasp_y> <grasp_z> <place_x> <place_y> <place_z>
All in world frame; grasp = TCP position on the mug rim wall, place = TCP
position at release. Fingers open along world y (hand yaw 0).
"""
import sys

import numpy as np

import rob

Z_TRAVEL = 0.75
R = rob.top_down_R(0.0)


def move_tcp(r, p, seed, seconds, tag):
    q = r.ik_tcp(p, R, seed=seed)
    if q is None:
        raise SystemExit(f"{tag}: IK failed for {p}")
    for attempt in range(3):
        code, qf = r.move_traj([q], [seconds])
        if np.abs(qf - q).max() < 0.02:
            break
        print(f"{tag}: residual too big, resending")
    tcp = r.tcp_world(qf)
    print(f"{tag}: tcp world {tcp.round(3)} (target {np.round(p, 3)})")
    return qf


def main():
    gx, gy, gz, px, py, pz = map(float, sys.argv[1:7])
    r = rob.Robot("pick_place")
    q = r.q()
    print("start q", q.round(3), "tcp", r.tcp_world(q).round(3))

    r.open()
    q = move_tcp(r, (gx, gy, Z_TRAVEL), q, 4.0, "pre-grasp")
    q = move_tcp(r, (gx, gy, gz + 0.06), q, 2.5, "approach")
    q = move_tcp(r, (gx, gy, gz), q, 2.0, "grasp-height")
    f = r.close()
    gap = f[0] - f[1]
    print(f"finger gap after close: {gap:.4f}")
    if gap < 0.004:
        raise SystemExit("closed on air; aborting")
    q = move_tcp(r, (gx, gy, Z_TRAVEL), q, 3.0, "lift")
    f = r.fingers()
    print(f"finger gap after lift: {f[0]-f[1]:.4f}")
    q = move_tcp(r, (px, py, Z_TRAVEL), q, 4.0, "transport")
    q = move_tcp(r, (px, py, pz + 0.05), q, 2.5, "pre-place")
    q = move_tcp(r, (px, py, pz), q, 2.0, "place")
    r.open()
    q = move_tcp(r, (px, py, Z_TRAVEL), q, 3.0, "retreat")
    print("done")


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 29
nohup python3 -u pick_place.py -0.0945 -0.221 0.52 0.015 0.250 0.555 > white.log 2>&1 &
sleep 45; cat /workspace/white.log

# openrua op 30
sleep 40; cat /workspace/white.log

# openrua op 31
sleep 15; tail -3 /workspace/white.log; timeout 60 python3 tools/perception/cam_snap.py agentview && timeout 60 python3 tools/perception/cam_snap.py birdview

# openrua op 32
python3 - <<'EOF'
s=open('/workspace/pick_place.py').read()
s=s.replace('''    for attempt in range(3):
        code, qf = r.move_traj([q], [seconds])
        if np.abs(qf - q).max() < 0.02:
            break
        print(f"{tag}: residual too big, resending")''','''    for attempt in range(6):
        code, qf = r.move_traj([q], [seconds])
        if np.abs(qf - q).max() < 0.02:
            break
        print(f"{tag}: residual too big, resending")
    else:
        raise SystemExit(f"{tag}: could not converge")''')
s=s.replace('q = move_tcp(r, (gx, gy, Z_TRAVEL), q, 4.0, "pre-grasp")','q = move_tcp(r, (gx, gy, Z_TRAVEL), q, 6.0, "pre-grasp")')
s=s.replace('q = move_tcp(r, (px, py, Z_TRAVEL), q, 4.0, "transport")','q = move_tcp(r, (px, py, Z_TRAVEL), q, 6.0, "transport")')
open('/workspace/pick_place.py','w').write(s)
EOF
timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png >/dev/null && python3 -c "
import numpy as np, cv2
d=np.load('birdview_depth.npy'); img=cv2.imread('birdview.png')
fx=579.4112549695428; cx=320; cy=240
R=np.array([[0,1,0],[1,0,0],[0,0,-1]]); t=np.array([-0.2,0,3.0])
us=np.arange(640)[None,:].repeat(480,0); vs=np.arange(480)[:,None].repeat(640,1)
P=np.stack([(us-cx)*d/fx,(vs-cy)*d/fx,d],0).reshape(3,-1); W=(R@P+t[:,None]).reshape(3,480,640); X,Y,H=W
mask=(H>0.50)&(H<0.8)
n,lab,stats,cent=cv2.connectedComponentsWithStats((mask*255).astype('uint8'))
for i in range(1,n):
    a=stats[i,cv2.CC_STAT_AREA]
    if a<100: continue
    sel=lab==i; hz=H[sel]; s2=sel&(H>hz.max()-0.02)
    print(i,'area',a,'x',X[s2].min().round(3),X[s2].max().round(3),'y',Y[s2].min().round(3),Y[s2].max().round(3),'center',X[s2].mean().round(3),Y[s2].mean().round(3),'ztop',hz.max().round(3))
"

# openrua op 33
nohup python3 -u pick_place.py -0.0745 0.137 0.512 -0.024 -0.2785 0.545 > yellow.log 2>&1 &
sleep 60; cat /workspace/yellow.log

# openrua op 34
sleep 20; tail -4 /workspace/yellow.log; timeout 60 python3 tools/perception/cam_snap.py agentview && timeout 60 python3 tools/perception/cam_snap.py birdview && timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png

# openrua op 35
python3 -c "
import numpy as np, cv2
d=np.load('birdview_depth.npy'); img=cv2.imread('birdview.png')
fx=579.4112549695428; cx=320; cy=240
R=np.array([[0,1,0],[1,0,0],[0,0,-1]]); t=np.array([-0.2,0,3.0])
us=np.arange(640)[None,:].repeat(480,0); vs=np.arange(480)[:,None].repeat(640,1)
P=np.stack([(us-cx)*d/fx,(vs-cy)*d/fx,d],0).reshape(3,-1); W=(R@P+t[:,None]).reshape(3,480,640); X,Y,H=W
mask=(H>0.50)&(H<0.8)
n,lab,stats,cent=cv2.connectedComponentsWithStats((mask*255).astype('uint8'))
plates={'left':(-0.024,-0.323),'right':(0.015,0.296)}
for i in range(1,n):
    a=stats[i,cv2.CC_STAT_AREA]
    if a<100 or stats[i,1]<200: continue
    sel=lab==i; hz=H[sel]; s2=sel&(H>hz.max()-0.02)
    cxw,cyw=X[s2].mean(),Y[s2].mean()
    col=img[sel].mean(0).round(0)
    near=min(plates,key=lambda k:np.hypot(plates[k][0]-cxw,plates[k][1]-cyw))
    dist=np.hypot(plates[near][0]-cxw,plates[near][1]-cyw)
    print(f'obj{i} area {a} rim-center ({cxw:.3f},{cyw:.3f}) ztop {hz.max():.3f} mean BGR {col}  nearest plate: {near} at {dist*100:.1f} cm (plate radius ~6.8cm)')
crop=img[230:330,200:440]; cv2.imwrite('bird_final.png',cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST))
"

# openrua op 36
timeout 600 python3 -u -c "
import rob, numpy as np
r=rob.Robot()
q=np.array([0,-1.0,0,-2.0,0,1.0,0.785])
for i in range(4):
    code,qf=r.move_to(q,6.0)
    if np.abs(qf-q).max()<0.02: break
print('tcp',r.tcp_world().round(3))
" && timeout 60 python3 tools/perception/cam_snap.py birdview && timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png && python3 -c "
import numpy as np, cv2
d=np.load('birdview_depth.npy'); img=cv2.imread('birdview.png')
fx=579.4112549695428; cx=320; cy=240
R=np.array([[0,1,0],[1,0,0],[0,0,-1]]); t=np.array([-0.2,0,3.0])
us=np.arange(640)[None,:].repeat(480,0); vs=np.arange(480)[:,None].repeat(640,1)
P=np.stack([(us-cx)*d/fx,(vs-cy)*d/fx,d],0).reshape(3,-1); W=(R@P+t[:,None]).reshape(3,480,640); X,Y,H=W
mask=(H>0.50)&(H<0.8)
n,lab,stats,cent=cv2.connectedComponentsWithStats((mask*255).astype('uint8'))
plates={'left':(-0.024,-0.323),'right':(0.015,0.296)}
for i in range(1,n):
    a=stats[i,cv2.CC_STAT_AREA]
    if a<100 or stats[i,1]<200: continue
    sel=lab==i; hz=H[sel]; s2=sel&(H>hz.max()-0.02)
    cxw,cyw=X[s2].mean(),Y[s2].mean()
    col=img[sel].mean(0).round(0)
    near=min(plates,key=lambda k:np.hypot(plates[k][0]-cxw,plates[k][1]-cyw))
    dist=np.hypot(plates[near][0]-cxw,plates[near][1]-cyw)
    print(f'obj{i} area {a} rim-center ({cxw:.3f},{cyw:.3f}) ztop {hz.max():.3f} mean BGR {col}  nearest plate: {near} at {dist*100:.1f} cm')
crop=img[230:330,200:440]; cv2.imwrite('bird_final.png',cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST))
"

# openrua op 37
mkdir -p "$(dirname /claude-config/projects/-workspace/memory/panda-ik-frame-convention.md)"
cat > /claude-config/projects/-workspace/memory/panda-ik-frame-convention.md <<'OPENRUA_EOF'
---
name: panda-ik-frame-convention
description: On the Panda workstation, /compute_ik takes poses in the WORLD frame and solves for panda_link8, contrary to docs/30-action.md
metadata:
  type: project
---

On the /workspace Panda robot (machine.yaml, Sep 2026), `/compute_ik` with empty
`frame_id` interprets the pose in the **world** frame (not panda_link0 as
docs/30-action.md "Planning facts" claims) and its tip link is **panda_link8**
(= panda_hand rotated +45° about z). World→panda_link0 is a pure translation
(-0.51, 0, 0.42).

**Why:** Sending a base-frame panda_hand pose returned NO_IK_SOLUTION (-31)
even for the reachable home pose; FK-checking a returned solution showed it
offset by exactly the world→base translation and rotated 45°.

**How to apply:** Convert TCP/hand targets to world coords and post-multiply
the hand rotation by Rz(+45°) before calling IK; verify with FK on the returned
joints. `/workspace/rob.py` (`ik_tcp`) already does this if it still exists.
Also: table top is world z≈0.42–0.43 (birdview depth); oblique agentview
overestimates plate height by ~5 cm. Mug bodies ~9 cm > 8 cm gripper gap, so
rim grasps work (finger gap 8–12 mm when closed on the wall).
OPENRUA_EOF

# openrua op 38
cd /claude-config/projects/-workspace/memory && (test -f MEMORY.md && cat MEMORY.md || echo "(no index yet)")

# openrua op 39
mkdir -p "$(dirname /claude-config/projects/-workspace/memory/MEMORY.md)"
cat > /claude-config/projects/-workspace/memory/MEMORY.md <<'OPENRUA_EOF'
- [Panda IK frame convention](panda-ik-frame-convention.md) — /compute_ik wants WORLD-frame poses for panda_link8, not base-frame panda_hand as docs say; rim grasps for 9 cm mugs
OPENRUA_EOF
