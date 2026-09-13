#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la && find . -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
cat tools/action/fjt_send.py; echo ----; cat tools/action/gripper_cmd.py; echo ----; cat tools/action/ik_move.py

# openrua op 3
timeout 30 ros2 topic list; echo ---; timeout 20 ros2 action list; echo ---; timeout 20 ros2 service list | grep -iv parameter | head -40; echo ---; timeout 20 ros2 node list

# openrua op 4
for c in agentview frontview birdview sideview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c; done; timeout 20 ros2 topic echo /joint_states --once

# openrua op 5
timeout 20 ros2 topic echo /tf_static --once 2>&1 | head -80; echo ----; timeout 10 ros2 topic echo /tf --once 2>&1 | head -40

# openrua op 6
timeout 30 python3 -c "
import rclpy
from tf2_msgs.msg import TFMessage
rclpy.init(); n=rclpy.create_node('tfdump')
got=[]
n.create_subscription(TFMessage,'/tf',got.append,10)
import time
for _ in range(20): rclpy.spin_once(n,timeout_sec=0.5)
seen={}
for m in got:
  for t in m.transforms:
    seen[(t.header.frame_id,t.child_frame_id)]=t.transform
for k,v in seen.items():
  print(k, 'T=(%.3f %.3f %.3f)'%(v.translation.x,v.translation.y,v.translation.z), 'Q=(%.3f %.3f %.3f %.3f)'%(v.rotation.x,v.rotation.y,v.rotation.z,v.rotation.w))
"

# openrua op 7
cat > /workspace/px_batch.py <<'EOF'
#!/usr/bin/env python3
"""px_batch.py <camera> u,v [u,v ...] -> world xyz per pixel (uses TF from /tf)."""
import struct, sys, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

def grab(node, topic, T):
    got={}
    s=node.create_subscription(T, topic, lambda m: got.setdefault('m',m), 1)
    while 'm' not in got: rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(s); return got['m']

def qR(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

cam=sys.argv[1]
rclpy.init(); node=rclpy.create_node('pxb')
buf=Buffer(); TransformListener(buf,node)
depth=grab(node,f'/{cam}/depth/image_raw',Image)
info=grab(node,f'/{cam}/color/camera_info',CameraInfo)
D=np.frombuffer(depth.data,dtype=np.float32).reshape(depth.height,depth.width)
frame=f'{cam}_optical_frame'
while not buf.can_transform('world',frame,rclpy.time.Time()): rclpy.spin_once(node,timeout_sec=0.2)
t=buf.lookup_transform('world',frame,rclpy.time.Time()); q=t.transform.rotation
R=qR(q.x,q.y,q.z,q.w); tr=np.array([t.transform.translation.x,t.transform.translation.y,t.transform.translation.z])
fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
for a in sys.argv[2:]:
    u,v=map(int,a.split(','))
    z=float(D[v,u])
    p=np.array([(u-cx)*z/fx,(v-cy)*z/fy,z])
    w=R@p+tr
    print(f'({u},{v}) depth={z:.3f} -> world {w[0]:.3f} {w[1]:.3f} {w[2]:.3f}')
rclpy.shutdown()
EOF
timeout 60 python3 /workspace/px_batch.py agentview 318,200 318,150 215,230 215,190 395,285 395,275 310,390 260,390 360,390 310,360 310,420 100,300 500,300

# openrua op 8
timeout 60 python3 /workspace/px_batch.py agentview 200,400 450,400 310,460 310,300 150,250 480,250 400,350 220,350 320,30 320,470

# openrua op 9
timeout 300 python3 tools/action/fjt_send.py 0.0,-0.161037389,0.0,-2.44459747,0.0,2.2267522,0.7853981633974483 2 && timeout 300 python3 tools/action/gripper_cmd.py 0.04 && timeout 20 ros2 topic echo /joint_states --once | grep -A9 position

# openrua op 10
timeout 60 python3 tools/perception/cam_snap.py agentview && timeout 60 python3 tools/perception/cam_snap.py birdview && timeout 60 python3 /workspace/px_batch.py agentview 310,390 310,360 310,420 395,285 318,200 318,150 215,230

# openrua op 11
cat > /workspace/segment.py <<'EOF'
#!/usr/bin/env python3
"""Segment objects above the table in a camera's depth image; print world-frame blobs."""
import sys, numpy as np, rclpy, cv2
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge

def grab(node, topic, T):
    got={}
    s=node.create_subscription(T, topic, lambda m: got.setdefault('m',m), 1)
    while 'm' not in got: rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(s); return got['m']
def qR(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
cam=sys.argv[1]; table_z=float(sys.argv[2]) if len(sys.argv)>2 else 0.426
rclpy.init(); node=rclpy.create_node('seg')
buf=Buffer(); TransformListener(buf,node)
depth=grab(node,f'/{cam}/depth/image_raw',Image)
color=grab(node,f'/{cam}/color/image_raw',Image)
info=grab(node,f'/{cam}/color/camera_info',CameraInfo)
D=np.frombuffer(depth.data,dtype=np.float32).reshape(depth.height,depth.width)
C=CvBridge().imgmsg_to_cv2(color,'bgr8')
frame=f'{cam}_optical_frame'
while not buf.can_transform('world',frame,rclpy.time.Time()): rclpy.spin_once(node,timeout_sec=0.2)
t=buf.lookup_transform('world',frame,rclpy.time.Time()); q=t.transform.rotation
R=qR(q.x,q.y,q.z,q.w); tr=np.array([t.transform.translation.x,t.transform.translation.y,t.transform.translation.z])
fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
H,W=D.shape
vv,uu=np.mgrid[0:H,0:W]
P=np.stack([(uu-cx)*D/fx,(vv-cy)*D/fy,D],-1).reshape(-1,3)
Wp=(P@R.T+tr).reshape(H,W,3)
np.save(f'{cam}_world.npy',Wp)
mask=((Wp[...,2]>table_z+0.008)&(Wp[...,2]<table_z+0.35)&np.isfinite(D)).astype(np.uint8)
# limit to table area
mask&=((np.abs(Wp[...,0])<0.6)&(np.abs(Wp[...,1])<0.6)).astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,cv2.CC_STAT_AREA]<15: continue
    m=lab==i
    pts=Wp[m]; col=C[m].mean(0)
    x,y,w,h=stats[i,:4]
    print(f'blob{i}: px area={stats[i,4]} bbox=({x},{y},{w},{h}) ctr_px=({cent[i][0]:.0f},{cent[i][1]:.0f}) '
          f'world x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] '
          f'zmax={pts[:,2].max():.3f} mean=({pts[:,0].mean():.3f},{pts[:,1].mean():.3f}) bgr={col.astype(int)}')
rclpy.shutdown()
EOF
timeout 120 python3 /workspace/segment.py birdview

# openrua op 12
python3 -c "
import numpy as np
W=np.load('birdview_world.npy')
np.set_printoptions(precision=3, linewidth=250, suppress=True)
print('red mug z rows 225-244, cols 300-334:')
print(W[225:245,300:334,2])
print('pudding z rows 268-288, cols 328-352:')
print(W[268:288,328:352,2])
print('plate z row 322 cols 300-334:', W[322,300:334,2])
"

# openrua op 13
mkdir -p "$(dirname /workspace/rob.py)"
cat > /workspace/rob.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Reusable helpers for this Panda: joint state, FK, IK, trajectories, gripper.

All poses are in the arm base frame (panda_link0) unless noted; world =
base + (-0.51, 0, 0.42) (from /tf). Import and use, or run as a module
for quick one-offs.
"""
import math
import time
from pathlib import Path

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

ROOT = Path(__file__).resolve().parent
M = yaml.safe_load((ROOT / "machine.yaml").read_text())
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]
LIMITS = FJT["limits_rad"]
BASE_IN_WORLD = np.array([-0.51, 0.0, 0.42])
TCP_OFF = float(M["hand"]["tcp_offset_m"])


def world_to_base(p):
    return np.asarray(p, float) - BASE_IN_WORLD


def base_to_world(p):
    return np.asarray(p, float) + BASE_IN_WORLD


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def quat_mul(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array([
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ])


def down_quat(yaw=0.0):
    """Hand pointing straight down (hand +Z = world -Z); `yaw` rotates the
    finger-opening axis about vertical. yaw=0 -> fingers open along base Y."""
    q_down = np.array([1.0, 0.0, 0.0, 0.0])  # 180 deg about X
    q_yaw = np.array([0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)])
    return quat_mul(q_yaw, q_down)


class Robot:
    def __init__(self, name="rob"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self._js.__setitem__("m", m), 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt.wait_for_server(10)
        self.grip.wait_for_server(10)
        self.ik.wait_for_service(10)
        self.fk.wait_for_service(10)

    # ---------------- sensing ----------------
    def spin(self, t=0.05):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self, fresh=True):
        if fresh:
            self._js.pop("m", None)
        while "m" not in self._js:
            self.spin(0.2)
        m = self._js["m"]
        return dict(zip(m.name, m.position))

    def arm_q(self):
        j = self.joints()
        return [j[n] for n in ARM]

    def finger_gap(self):
        j = self.joints()
        return abs(j["panda_finger_joint1"]) + abs(j["panda_finger_joint2"])

    def fk_pose(self, q=None, link="panda_hand"):
        """Returns (pos[3], quat[4]) of `link` in base frame."""
        if q is None:
            q = self.arm_q()
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {None if res is None else res.error_code.val}")
        p = res.pose_stamped[0].pose
        return (np.array([p.position.x, p.position.y, p.position.z]),
                np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]))

    def tcp_world(self, q=None):
        pos, quat = self.fk_pose(q)
        R = quat_to_R(*quat)
        return base_to_world(pos + TCP_OFF * R[:, 2])

    # ---------------- IK ----------------
    def solve_ik(self, pos_base, quat, seed=None, at_tcp=False, timeout=20.0):
        """IK for the hand (or TCP if at_tcp) pose in base frame. Returns q list or None."""
        pos = np.asarray(pos_base, float)
        if at_tcp:
            R = quat_to_R(*quat)
            pos = pos - TCP_OFF * R[:, 2]
        if seed is None:
            seed = self.arm_q()
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pos)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.robot_state.joint_state.name = list(ARM)
        req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=int(timeout), nanosec=int((timeout % 1) * 1e9))
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout + 30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[n] for n in ARM]

    def ik_world(self, pos_world, quat, seed=None, at_tcp=True, tries=3):
        q = None
        s = seed
        for _ in range(tries):
            q = self.solve_ik(world_to_base(pos_world), quat, seed=s, at_tcp=at_tcp)
            if q is not None:
                break
        return q

    # ---------------- motion ----------------
    def move_joints(self, points, times):
        """points: list of q lists; times: cumulative seconds per point."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        for q, t in zip(points, times):
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
        r = res.result()
        code = r.result.error_code if r else None
        q_now = self.arm_q()
        err = max(abs(a - b) for a, b in zip(q_now, points[-1]))
        return code, err

    def move_q(self, q, seconds=3.0):
        return self.move_joints([q], [seconds])

    def move_world(self, pos_world, quat, seconds=3.0, at_tcp=True, seed=None):
        q = self.ik_world(pos_world, quat, seed=seed, at_tcp=at_tcp)
        if q is None:
            raise RuntimeError(f"IK failed for world {pos_world}")
        code, err = self.move_q(q, seconds)
        return q, code, err

    def move_line_world(self, p_from, p_to, quat, n=6, seconds=3.0, at_tcp=True):
        """Straight TCP line via IK waypoints in one trajectory."""
        pts, times = [], []
        seed = self.arm_q()
        for i in range(1, n + 1):
            p = np.asarray(p_from) + (np.asarray(p_to) - np.asarray(p_from)) * i / n
            q = self.ik_world(p, quat, seed=seed, at_tcp=at_tcp)
            if q is None:
                raise RuntimeError(f"IK failed on line at {p}")
            seed = q
            pts.append(q)
            times.append(seconds * i / n)
        return self.move_joints(pts, times)

    def gripper(self, width, timeout=120):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP.get("max_effort", 30.0))
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=timeout)
        r = res.result().result
        return r.reached_goal, r.stalled, self.finger_gap()

    def open(self):
        return self.gripper(GRIP["open_m"])

    def close(self):
        return self.gripper(GRIP["closed_m"])
OPENRUA_EOF

# openrua op 14
timeout 120 python3 -c "
from rob import *
r=Robot('t1')
q=r.arm_q(); print('q',np.round(q,3))
pos,quat=r.fk_pose(); print('hand base',np.round(pos,3),'quat',np.round(quat,3)); print('hand world',np.round(base_to_world(pos),3))
print('tcp world',np.round(r.tcp_world(),3))
R=quat_to_R(*quat); print('hand z axis in base', np.round(R[:,2],3), 'hand y (finger axis)', np.round(R[:,1],3))
print('down_quat(0)', down_quat(0), 'down_quat(pi/4)', np.round(down_quat(math.pi/4),3))
"

# openrua op 15
timeout 120 python3 -c "
from rob import *
r=Robot('t2')
for l in ['panda_link0','panda_link1','panda_hand','panda_leftfinger']:
    pos,quat=r.fk_pose(link=l); print(l, np.round(pos,4), np.round(quat,3))
" ; timeout 30 ros2 service call /get_planning_scene moveit_msgs/srv/GetPlanningScene "{components: {components: 0}}" 2>/dev/null | grep -E "frame_id|planning_frame" | head

# openrua op 16
sed -i 's/^BASE_IN_WORLD = np.array(\[-0.51, 0.0, 0.42\])/BASE_IN_WORLD = np.array([0.0, 0.0, 0.0])  # FK(panda_link0) = (-0.51,0,0.42): model frame IS world/' rob.py && grep -n BASE_IN_WORLD rob.py | head -2 && timeout 120 python3 -c "
from rob import *
r=Robot('t3')
q0=r.arm_q()
pos,quat=r.fk_pose(); print('hand world',np.round(pos,4))
print('tcp world', np.round(r.tcp_world(),4))
q=r.solve_ik(pos,quat,seed=q0); print('ik roundtrip q', None if q is None else np.round(q,3))
# IK for a pose above the red mug: tcp at (-0.224,-0.003,0.70) pointing down, fingers along y
q1=r.ik_world([-0.224,-0.003,0.70], down_quat(0)); print('above mug q', None if q1 is None else np.round(q1,3))
if q1 is not None: print('check tcp', np.round(r.tcp_world(q1),4))
q2=r.ik_world([-0.224,-0.003,0.70], down_quat(math.pi/2)); print('above mug yaw90 q', None if q2 is None else np.round(q2,3))
if q2 is not None: print('check tcp', np.round(r.tcp_world(q2),4))
"

# openrua op 17
python3 -c "
import numpy as np
W=np.load('birdview_world.npy')
sub=W[220:250,308:336]
m=(sub[...,2]>0.50)&(sub[...,2]<0.60)
pts=sub[m]
print('rim pts',len(pts),'centroid',pts[:,:2].mean(0),'x range',pts[:,0].min(),pts[:,0].max(),'y range',pts[:,1].min(),pts[:,1].max())
# fit circle least squares
x,y=pts[:,0],pts[:,1]
A=np.c_[2*x,2*y,np.ones_like(x)]; b=x**2+y**2
c=np.linalg.lstsq(A,b,rcond=None)[0]; r=np.sqrt(c[2]+c[0]**2+c[1]**2)
print('circle fit center',c[:2],'radius',r)
# inner floor
m2=(sub[...,2]>0.44)&(sub[...,2]<0.46); p2=sub[m2]; print('inner floor centroid',p2[:,:2].mean(0),'n',len(p2), 'z', p2[:,2].mean())
"

# openrua op 18
timeout 900 python3 -u -c "
from rob import *
r=Robot('grasp1')
cx,cy,rim=-0.224,0.002,0.573
wall=0.037
pre=[cx,cy+wall,0.70]; grasp=[cx,cy+wall,rim-0.025]
Q=down_quat(0)
print('open', r.open())
q,code,err=r.move_world(pre,Q,seconds=4); print('pre-grasp code',code,'err',round(err,4),'tcp',np.round(r.tcp_world(),4))
code,err=r.move_line_world(pre,grasp,Q,n=5,seconds=3); print('descend code',code,'err',round(err,4),'tcp',np.round(r.tcp_world(),4))
print('close', r.close())
print('gap after close', round(r.finger_gap(),4))
" 2>&1 | tail -20

# openrua op 19
timeout 60 python3 tools/perception/cam_snap.py agentview && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand && timeout 60 python3 tools/perception/cam_snap.py sideview

# openrua op 20
timeout 900 python3 -u -c "
from rob import *
r=Robot('lift1')
t=r.tcp_world(); print('tcp now',np.round(t,4))
code,err=r.move_line_world(t,[t[0],t[1],t[2]+0.12],down_quat(0),n=4,seconds=3); print('lift code',code,'err',round(err,4),'tcp',np.round(r.tcp_world(),4),'gap',round(r.finger_gap(),4))
" 2>&1 | tail -5 && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 21
timeout 120 python3 /workspace/segment.py birdview 0.426 2>&1 | grep -v "^blob[0-9]*: px area=1[0-9] " ; python3 -c "
import numpy as np
W=np.load('birdview_world.npy')
Z=W[...,2]
m=(Z>0.66)&(Z<0.72)&(np.abs(W[...,0]+0.23)<0.08)&(np.abs(W[...,1])<0.1)
pts=W[m]; print('rim-height pts',len(pts))
if len(pts):
  x,y=pts[:,0],pts[:,1]
  A=np.c_[2*x,2*y,np.ones_like(x)]; b=x**2+y**2
  c=np.linalg.lstsq(A,b,rcond=None)[0]; r=np.sqrt(c[2]+c[0]**2+c[1]**2)
  print('circle fit center',np.round(c[:2],4),'radius',round(r,4),'xr',x.min(),x.max(),'yr',y.min(),y.max())
"

# openrua op 22
timeout 60 python3 tools/perception/cam_snap.py frontview && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 23
timeout 1200 python3 -u -c "
from rob import *
r=Robot('carry1')
Q=down_quat(0)
t=r.tcp_world(); print('tcp',np.round(t,4))
hi=[t[0],t[1],0.76]
code,err=r.move_line_world(t,hi,Q,n=3,seconds=3); print('up code',code,round(err,4),np.round(r.tcp_world(),4))
plate=np.array([0.160,-0.012])
off=np.array([0.008,-0.037])   # mug center - tcp
tgt_xy=plate-off
above=[tgt_xy[0],tgt_xy[1],0.76]
code,err=r.move_line_world(hi,above,Q,n=6,seconds=6); print('traverse code',code,round(err,4),np.round(r.tcp_world(),4),'gap',round(r.finger_gap(),4))
" 2>&1 | tail -5 && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 24
timeout 1200 python3 -u -c "
from rob import *
r=Robot('carry2')
Q=down_quat(0)
t=r.tcp_world(); print('tcp',np.round(t,4))
above=[0.152,0.025,0.76]
for i in range(3):
    code,err=r.move_line_world(r.tcp_world(),above,Q,n=4,seconds=5); t=r.tcp_world(); print('traverse',i,'code',code,round(err,4),np.round(t,4),'gap',round(r.finger_gap(),4))
    if code==0 and np.linalg.norm(t-above)<0.01: break
" 2>&1 | tail -5

# openrua op 25
timeout 1200 python3 -u -c "
from rob import *
r=Robot('place1')
Q=down_quat(0)
t=r.tcp_world()
low=[t[0],t[1],0.585]
code,err=r.move_line_world(t,low,Q,n=5,seconds=4); t=r.tcp_world(); print('lower code',code,round(err,4),np.round(t,4),'gap',round(r.finger_gap(),4))
" 2>&1 | tail -3 && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 26
timeout 1200 python3 -u -c "
from rob import *
r=Robot('release1')
Q=down_quat(0)
print('open',r.open())
t=r.tcp_world()
code,err=r.move_line_world(t,[t[0],t[1],0.78],Q,n=3,seconds=3); print('retreat code',code,round(err,4),np.round(r.tcp_world(),4))
" 2>&1 | tail -3 && timeout 60 python3 tools/perception/cam_snap.py agentview && timeout 120 python3 /workspace/segment.py birdview 0.426 | grep -v "px area=[1-9][0-9] "

# openrua op 27
timeout 1200 python3 -u -c "
from rob import *
r=Robot('park1')
q,code,err=r.move_world([-0.05,-0.25,0.80],down_quat(0),seconds=5); print('park code',code,round(err,4),np.round(r.tcp_world(),4))
" 2>&1 | tail -2 && timeout 120 python3 /workspace/segment.py birdview 0.426 | grep -v "px area=[1-9][0-9] " && timeout 60 python3 tools/perception/cam_snap.py birdview

# openrua op 28
timeout 60 find / -iname "*.bddl" 2>/dev/null | head; timeout 60 find / -path /proc -prune -o \( -iname "*libero*" -o -iname "*robosuite*" \) -print 2>/dev/null | head

# openrua op 29
timeout 1200 python3 -u -c "
from rob import *
r=Robot('pud1')
Q=down_quat(0)
px,py,ptop=-0.042,0.105,0.465
print('open',r.open())
q,code,err=r.move_world([px,py,0.60],Q,seconds=4); print('pre code',code,round(err,4),np.round(r.tcp_world(),4))
code,err=r.move_line_world([px,py,0.60],[px,py,0.44],Q,n=5,seconds=3); print('descend code',code,round(err,4),np.round(r.tcp_world(),4))
print('close',r.close()); print('gap',round(r.finger_gap(),4))
t=r.tcp_world()
code,err=r.move_line_world(t,[t[0],t[1],0.60],Q,n=4,seconds=3); print('lift code',code,round(err,4),np.round(r.tcp_world(),4),'gap',round(r.finger_gap(),4))
" 2>&1 | tail -6 && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 30
python3 - <<'EOF'
import re
s=open('rob.py').read()
old=s[s.index('    def move_line_world'):s.index('    def gripper')]
new='''    def move_line_world(self, p_from, p_to, quat, n=6, seconds=3.0, at_tcp=True,
                        max_jump=0.4, settle=True):
        """Straight TCP line via IK waypoints in one trajectory. Rejects
        waypoints whose joint solution jumps branches; re-sends the final
        point if the controller lagged (tolerance violation)."""
        pts, times = [], []
        seed = self.arm_q()
        prev = seed
        for i in range(1, n + 1):
            p = np.asarray(p_from) + (np.asarray(p_to) - np.asarray(p_from)) * i / n
            q = None
            for _ in range(5):
                cand = self.ik_world(p, quat, seed=prev, at_tcp=at_tcp, tries=1)
                if cand is not None and max(abs(a - b) for a, b in zip(cand, prev)) <= max_jump:
                    q = cand
                    break
            if q is None:
                raise RuntimeError(f"IK failed / branch jump on line at {p}")
            prev = q
            pts.append(q)
            times.append(seconds * i / n)
        code, err = self.move_joints(pts, times)
        if settle and (code != 0 or err > 0.02):
            for _ in range(3):
                code, err = self.move_joints([pts[-1]], [2.0])
                if code == 0 and err <= 0.02:
                    break
        return code, err

'''
s=s.replace(old,new)
# also make move_q settle
s=s.replace('''    def move_q(self, q, seconds=3.0):
        return self.move_joints([q], [seconds])''','''    def move_q(self, q, seconds=3.0, settle=True):
        code, err = self.move_joints([q], [seconds])
        if settle and (code != 0 or err > 0.02):
            for _ in range(3):
                code, err = self.move_joints([q], [2.0])
                if code == 0 and err <= 0.02:
                    break
        return code, err''')
open('rob.py','w').write(s)
EOF
python3 -c "import ast;ast.parse(open('/workspace/rob.py').read());print('ok')"

# openrua op 31
timeout 1200 python3 -u -c "
from rob import *
r=Robot('park2')
q,code,err=r.move_world([-0.05,-0.25,0.80],down_quat(0),seconds=5); print('park code',code,round(err,4),np.round(r.tcp_world(),4))
" 2>&1 | tail -1 && timeout 120 python3 /workspace/segment.py birdview 0.426 | grep -v "px area=[1-9][0-9] "

# openrua op 32
timeout 1500 python3 -u -c "
from rob import *
r=Robot('pud2')
Q=down_quat(0)
px,py=-0.042,0.096
print('open',r.open())
q,code,err=r.move_world([px,py,0.60],Q,seconds=5); t=r.tcp_world(); print('pre code',code,round(err,4),np.round(t,4))
assert np.linalg.norm(t-[px,py,0.60])<0.01, 'pre-grasp off'
code,err=r.move_line_world(t,[px,py,0.44],Q,n=4,seconds=3); t=r.tcp_world(); print('descend code',code,round(err,4),np.round(t,4))
assert np.linalg.norm(t-[px,py,0.44])<0.01, 'descend off'
print('close',r.close()); print('gap',round(r.finger_gap(),4))
t=r.tcp_world()
code,err=r.move_line_world(t,[t[0],t[1],0.60],Q,n=3,seconds=3); print('lift code',code,round(err,4),np.round(r.tcp_world(),4),'gap',round(r.finger_gap(),4))
" 2>&1 | tail -6 && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 33
timeout 1500 python3 -u -c "
from rob import *
r=Robot('pud3')
Q=down_quat(0)
tx,ty=0.156,0.135
t=r.tcp_world()
code,err=r.move_line_world(t,[tx,ty,0.60],Q,n=5,seconds=5); t=r.tcp_world(); print('traverse code',code,round(err,4),np.round(t,4),'gap',round(r.finger_gap(),4))
assert np.linalg.norm(t-[tx,ty,0.60])<0.01, 'traverse off'
code,err=r.move_line_world(t,[tx,ty,0.447],Q,n=4,seconds=3); t=r.tcp_world(); print('lower code',code,round(err,4),np.round(t,4),'gap',round(r.finger_gap(),4))
print('open',r.open())
code,err=r.move_line_world(t,[tx,ty,0.65],Q,n=3,seconds=3); print('retreat code',code,round(err,4),np.round(r.tcp_world(),4))
" 2>&1 | tail -5 && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 34
timeout 1200 python3 -u -c "
from rob import *
r=Robot('home')
home=[0.0,-0.161037389,0.0,-2.44459747,0.0,2.2267522,0.7853981633974483]
code,err=r.move_q(home,5); print('home code',code,round(err,4),np.round(r.tcp_world(),4))
code,err=r.move_q(home,2); print('settle code',code,round(err,4))
" 2>&1 | tail -2 && timeout 120 python3 /workspace/segment.py birdview 0.426 | grep -v "px area=[1-9][0-9] " && timeout 60 python3 tools/perception/cam_snap.py agentview && timeout 60 python3 tools/perception/cam_snap.py birdview

# openrua op 35
python3 -c "
import numpy as np
W=np.load('birdview_world.npy'); Z=W[...,2]
m=(Z>0.55)&(Z<0.62)&(np.abs(W[...,0]-0.156)<0.1)&(np.abs(W[...,1])<0.1)
pts=W[m]; x,y=pts[:,0],pts[:,1]
A=np.c_[2*x,2*y,np.ones_like(x)]; b=x**2+y**2
c=np.linalg.lstsq(A,b,rcond=None)[0]; print('mug rim center',np.round(c[:2],4),'r',round(float(np.sqrt(c[2]+c[0]**2+c[1]**2)),4),'n',len(pts),'rim z',round(pts[:,2].mean(),3))
mp=(Z>0.435)&(Z<0.47)&(np.abs(W[...,0]-0.156)<0.1)&(np.abs(W[...,1]+0.012)<0.1)
p=W[mp]; print('plate visible pts',len(p),'x',round(p[:,0].min(),3),round(p[:,0].max(),3),'y',round(p[:,1].min(),3),round(p[:,1].max(),3))
"

# openrua op 36
mkdir -p "$(dirname /claude-config/projects/-workspace/memory/panda-workstation-quirks.md)"
cat > /claude-config/projects/-workspace/memory/panda-workstation-quirks.md <<'OPENRUA_EOF'
---
name: panda-workstation-quirks
description: Verified facts about the /workspace Panda sim workstation that differ from or extend the generic docs (MoveIt frame, camera axes, controller lag, mug grasp)
metadata:
  type: project
---

Verified on the /workspace Franka Panda workstation (session 2026-09-12):

- MoveIt `/compute_fk` / `/compute_ik` (frame_id "") work in the **world** frame, not the arm base:
  FK(panda_link0) returns (-0.51, 0, 0.42). docs/30-action.md's "model frame is the arm base" note is wrong here.
- agentview / birdview: image-right = world +y; table surface z = 0.425; robot faces +x.
- Long (>30 cm) FollowJointTrajectory moves often return error -5 (tolerance) mid-way — re-send the final
  point and it converges (docs are right about this). Also check IK waypoints for branch jumps (>0.4 rad).
- Gripper max gap ~8 cm; mugs (~8 cm dia) can't be body-grasped — pinch the rim wall (TCP at the wall,
  ~2.5 cm below the rim) with one finger inside; gap settles ~1.2 cm when holding.
- `/workspace/rob.py` (Robot class: FK/IK/line moves/gripper) and `segment.py` (birdview height-map blobs)
  were written for this and are reusable.

**Why:** these cost several turns to discover and aren't in the shipped docs.
**How to apply:** trust FK/IK poses as world coords; verify every long move; use birdview height map to localize objects.
OPENRUA_EOF

# openrua op 37
cd /claude-config/projects/-workspace/memory && (test -f MEMORY.md && cat MEMORY.md || true) && echo "- [Panda workstation quirks](panda-workstation-quirks.md) — MoveIt frame is world, camera axes, controller lag, rim-grasp for mugs" >> MEMORY.md
