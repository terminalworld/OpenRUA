#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la && find . -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
cat /workspace/tools/action/fjt_send.py; echo ----; cat /workspace/tools/action/gripper_cmd.py; echo ----; cat /workspace/tools/action/ik_move.py

# openrua op 3
timeout 30 ros2 topic list; echo ----; timeout 20 ros2 action list; echo ----; timeout 20 ros2 service list | grep -iv parameter | head -40; echo ----; timeout 20 ros2 node list

# openrua op 4
mkdir -p img && for c in agentview birdview frontview sideview galleryview paperview robot0_robotview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c img/$c.png & done; wait; ls -la img; timeout 20 ros2 topic echo /joint_states --once

# openrua op 5
python3 -c "
import cv2
im=cv2.imread('img/birdview.png'); crop=im[140:300,180:420]; cv2.imwrite('img/bird_crop.png', cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('img/agentview.png'); crop=im[140:420,90:500]; cv2.imwrite('img/agent_crop.png', cv2.resize(crop,None,fx=2,fy=2,interpolation=cv2.INTER_CUBIC))
"

# openrua op 6
timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw img/birdview_depth.png && timeout 20 ros2 topic echo /birdview/color/camera_info --once && timeout 20 ros2 run tf2_ros tf2_echo world birdview_optical_frame 2>&1 | head -20

# openrua op 7
timeout 15 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | grep -A3 "Translation" | head -5; timeout 15 ros2 run tf2_ros tf2_echo panda_link0 panda_hand 2>&1 | grep -A6 "Translation" | head -8

# openrua op 8
python3 -c "
import numpy as np
d=np.load('img/birdview_depth.npy')
fx=579.4112549695428; cx=320; cy=240
H,W=d.shape
v,u=np.mgrid[0:H,0:W]
X=(u-cx)*d/fx; Y=(v-cy)*d/fx; Z=d
wx=Y-0.2; wy=X; wz=3.0-Z
print('table z (median center region):', np.median(wz[300:400,200:440]))
# caddy region approx birdview pixels x 210-310, y 160-215
np.set_printoptions(linewidth=250, precision=3, suppress=True)
sub=wz[150:225:3,200:320:4]
print('heights in caddy region rows v=150..225 step3, cols u=200..320 step 4')
for i,row in enumerate(sub): print(150+3*i, ' '.join(f'{h:.2f}' for h in row))
"


# openrua op 9
cat > /workspace/cloud.py <<'EOF'
import sys, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
import struct

def grab(node, topic, T, timeout=20):
    got={}
    sub=node.create_subscription(T, topic, lambda m: got.setdefault('m',m), 1)
    import time; t0=time.time()
    while 'm' not in got and time.time()-t0<timeout: rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    return got.get('m')

def quat_R(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

def cloud(node, tfbuf, cam):
    depth=grab(node, f'/{cam}/depth/image_raw', Image)
    info=grab(node, f'/{cam}/color/camera_info', CameraInfo)
    d=np.frombuffer(depth.data, dtype=np.float32).reshape(depth.height, depth.width)
    fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
    frame=f'{cam}_optical_frame'
    import time; t0=time.time()
    while time.time()-t0<10:
        rclpy.spin_once(node, timeout_sec=0.2)
        if tfbuf.can_transform('world', frame, rclpy.time.Time()): break
    t=tfbuf.lookup_transform('world', frame, rclpy.time.Time())
    q=t.transform.rotation; R=quat_R(q.x,q.y,q.z,q.w)
    tr=np.array([t.transform.translation.x,t.transform.translation.y,t.transform.translation.z])
    v,u=np.mgrid[0:depth.height,0:depth.width]
    ok=np.isfinite(d)&(d>0)
    P=np.stack([(u-cx)*d/fx,(v-cy)*d/fy,d],-1)[ok]
    W=P@R.T+tr
    return W, tr

if __name__=='__main__':
    rclpy.init(); node=rclpy.create_node('cloud'); tfbuf=Buffer(); TransformListener(tfbuf,node)
    for cam in sys.argv[1:]:
        W,tr=cloud(node,tfbuf,cam)
        np.save(f'img/{cam}_cloud.npy', W.astype(np.float32))
        print(cam, 'cam pos', tr, 'pts', len(W))
    rclpy.shutdown()
EOF
timeout 120 python3 cloud.py birdview agentview frontview sideview galleryview paperview robot0_robotview robot0_eye_in_hand

# openrua op 10
python3 -c "
import numpy as np
cams=['birdview','agentview','frontview','sideview','robot0_robotview','robot0_eye_in_hand','paperview']
pts=np.concatenate([np.load(f'img/{c}_cloud.npy') for c in cams])
# region of interest: x -0.7..0.1, y -0.4..0.4, z 0.87..1.2
m=(pts[:,0]>-0.7)&(pts[:,0]<0.1)&(pts[:,1]>-0.4)&(pts[:,1]<0.4)&(pts[:,2]>0.85)&(pts[:,2]<1.15)
p=pts[m]
res=0.01
xs=np.arange(-0.7,0.1,res); ys=np.arange(-0.4,0.4,res)
H=np.full((len(xs),len(ys)),np.nan)
ix=((p[:,0]+0.7)/res).astype(int); iy=((p[:,1]+0.4)/res).astype(int)
ok=(ix>=0)&(ix<len(xs))&(iy>=0)&(iy<len(ys))
for a,b,z in zip(ix[ok],iy[ok],p[ok,2]):
    if np.isnan(H[a,b]) or z>H[a,b]: H[a,b]=z
np.save('img/H.npy',H)
print('     y: '+' '.join(f'{y:+.2f}'[1:] for y in ys[::2]))
for i,x in enumerate(xs):
    if i%1==0:
        row=H[i]
        s=''.join((' . ' if np.isnan(h) else f'{int(round((h-0.88)*100)):2d} ') for h in row[::2])
        print(f'x={x:+.2f} {s}')
" 

# openrua op 11
python3 -c "
import numpy as np
cams=['birdview','agentview','frontview','sideview','robot0_robotview','robot0_eye_in_hand','paperview','galleryview']
pts=np.concatenate([np.load(f'img/{c}_cloud.npy') for c in cams])
res=0.005
x0,x1,y0,y1=-0.50,-0.30,-0.40,0.10
m=(pts[:,0]>x0)&(pts[:,0]<x1)&(pts[:,1]>y0)&(pts[:,1]<y1)&(pts[:,2]>0.85)&(pts[:,2]<1.15)
p=pts[m]
xs=np.arange(x0,x1,res); ys=np.arange(y0,y1,res)
H=np.full((len(xs),len(ys)),np.nan)
ix=((p[:,0]-x0)/res).astype(int); iy=((p[:,1]-y0)/res).astype(int)
ok=(ix>=0)&(ix<len(xs))&(iy>=0)&(iy<len(ys))
for a,b,z in zip(ix[ok],iy[ok],p[ok,2]):
    if np.isnan(H[a,b]) or z>H[a,b]: H[a,b]=z
print('       y:'+''.join(f'{y:+.3f}'[-3:].rjust(3) if i%2==0 else '   ' for i,y in enumerate(ys)))
for i,x in enumerate(xs):
    row=H[i]
    s=''.join((' . ' if np.isnan(h) else f'{int(round((h-0.88)*100)):2d} ') for h in row)
    print(f'x={x:+.3f} {s}')
" 

# openrua op 12
python3 -c "
import numpy as np
cams=['birdview','agentview','frontview','sideview','robot0_robotview','paperview','galleryview']
pts=np.concatenate([np.load(f'img/{c}_cloud.npy') for c in cams])
m=(pts[:,0]>-0.25)&(pts[:,0]<0.0)&(pts[:,1]>-0.1)&(pts[:,1]<0.15)&(pts[:,2]>0.885)&(pts[:,2]<1.2)
p=pts[m]
print('n',len(p),'z range',p[:,2].min(),p[:,2].max())
# top face: z>1.07
top=p[p[:,2]>1.07]
print('top pts',len(top),'x',top[:,0].min(),top[:,0].max(),'y',top[:,1].min(),top[:,1].max(), 'zmean',top[:,2].mean())
xy=top[:,:2]; c=xy.mean(0); u,s,vt=np.linalg.svd(xy-c,full_matrices=False)
print('center',c,'axes',vt,'extent along axes', (xy-c)@vt[0], )
proj=(xy-c)@vt.T
print('len along major',proj[:,0].min(),proj[:,0].max(),' minor',proj[:,1].min(),proj[:,1].max())
print('yaw of major axis deg', np.degrees(np.arctan2(vt[0,1],vt[0,0])))
# all book pts footprint
for zl in [0.9,0.95,1.0,1.05]:
    s=p[(p[:,2]>zl)&(p[:,2]<zl+0.05)]
    print(zl, 'x',s[:,0].min(),s[:,0].max(),'y',s[:,1].min(),s[:,1].max())
"


# openrua op 13
python3 -c "
import numpy as np
p=np.load('img/birdview_cloud.npy')
m=(p[:,0]>-0.25)&(p[:,0]<0.0)&(p[:,1]>-0.1)&(p[:,1]<0.15)&(p[:,2]>1.06)
t=p[m]; print(len(t), 'z', t[:,2].min(), t[:,2].max())
xy=t[:,:2]; c=xy.mean(0); u,s,vt=np.linalg.svd(xy-c,full_matrices=False)
proj=(xy-c)@vt.T
print('center',c,'yaw deg',np.degrees(np.arctan2(vt[0,1],vt[0,0])))
print('major',proj[:,0].min(),proj[:,0].max(),'minor',proj[:,1].min(),proj[:,1].max())
# also the base (table-level footprint) from non-top points to check verticality: points z in 0.9..1.0 from sideview/front
q=np.concatenate([np.load(f'img/{c}_cloud.npy') for c in ['agentview','frontview','sideview','robot0_robotview']])
m=(q[:,0]>-0.2)&(q[:,0]<-0.03)&(q[:,1]>-0.05)&(q[:,1]<0.06)&(q[:,2]>0.89)&(q[:,2]<1.06)
b=q[m]
for zl in np.arange(0.9,1.06,0.03):
    s=b[(b[:,2]>=zl)&(b[:,2]<zl+0.03)]
    if len(s): print(f'z {zl:.2f}: x {s[:,0].min():.3f}..{s[:,0].max():.3f}  y {s[:,1].min():.3f}..{s[:,1].max():.3f} n={len(s)}')
"


# openrua op 14
python3 -c "
import numpy as np
p=np.load('img/birdview_cloud.npy')
m=(p[:,0]>-0.25)&(p[:,0]<0.0)&(p[:,1]>-0.1)&(p[:,1]<0.15)&(p[:,2]>1.06)&(p[:,2]<1.12)
t=p[m]; print(len(t), 'z', t[:,2].min(), t[:,2].max())
xy=t[:,:2]; c=xy.mean(0); u,s,vt=np.linalg.svd(xy-c,full_matrices=False)
proj=(xy-c)@vt.T
print('center',c,'yaw deg',np.degrees(np.arctan2(vt[0,1],vt[0,0])))
print('major',proj[:,0].min(),proj[:,0].max(),'minor',proj[:,1].min(),proj[:,1].max())
print(sorted(set(np.round(xy[:,0],3)))[::5]); print(sorted(set(np.round(xy[:,1],3))))
"


# openrua op 15
timeout 30 python3 - <<'EOF'
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
rclpy.init(); n=rclpy.create_node('rd')
qos=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
got=[]
n.create_subscription(String,'/robot_description',got.append,qos)
import time; t=time.time()
while not got and time.time()-t<10: rclpy.spin_once(n,timeout_sec=0.2)
if got:
    open('img/robot.urdf','w').write(got[0].data); print(len(got[0].data))
rclpy.shutdown()
EOF
grep -n -A12 'name="panda_hand_joint"\|name="panda_finger_joint1"\|name="panda_hand_tcp_joint"\|panda_leftfinger" type\|link name="panda_leftfinger"' img/robot.urdf | head -80

# openrua op 16
mkdir -p "$(dirname /workspace/lib.py)"
cat > /workspace/lib.py <<'OPENRUA_EOF'
"""Helpers: IK / FK / trajectory / gripper / joint-state on one node."""
import math
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]
BASE_IN_WORLD = np.array([-0.75, 0.0, 0.912])  # world -> panda_link0 (TF, static)
TCP = float(M["hand"]["tcp_offset_m"])


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def down_quat(yaw):
    """Hand pointing straight down (hand z = -world z), hand x rotated by yaw
    about world z. Fingers close along hand y."""
    c, s = math.cos(yaw / 2), math.sin(yaw / 2)
    return (c, s, 0.0, 0.0)  # qz(yaw) * qx(pi)


class Robot:
    def __init__(self, name="robot_lib"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = None
        self.node.create_subscription(JointState, "/joint_states",
                                      self._on_js, 1)
        self.ik_cli = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])

    def _on_js(self, m):
        self._js = m

    def spin(self, t=0.1):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self, fresh=True):
        if fresh:
            self._js = None
        t0 = time.time()
        while self._js is None and time.time() - t0 < 10:
            self.spin(0.2)
        d = dict(zip(self._js.name, self._js.position))
        return d

    def arm_q(self):
        d = self.joints()
        return [d[j] for j in ARM]

    def fingers(self):
        d = self.joints()
        return d["panda_finger_joint1"], d["panda_finger_joint2"]

    # ---- kinematics (base frame = panda_link0) ----
    def ik(self, pos_base, quat, seed=None, timeout=60):
        self.ik_cli.wait_for_service(timeout_sec=10)
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pos_base)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        seed = seed if seed is not None else self.arm_q()
        js = JointState()
        js.name = list(ARM)
        js.position = [float(v) for v in seed]
        req.ik_request.robot_state.joint_state = js
        req.ik_request.avoid_collisions = False
        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        res = fut.result()
        if res is None:
            raise RuntimeError("IK timeout")
        if res.error_code.val != 1:
            return None
        sol = dict(zip(res.solution.joint_state.name,
                       res.solution.joint_state.position))
        return [sol[j] for j in ARM]

    def ik_tcp_world(self, pos_world, yaw, seed=None, tcp_z_below=0.0):
        """IK for the TCP point (fingertip centre) at a WORLD position with the
        hand pointing down and yaw about world z."""
        q = down_quat(yaw)
        R = quat_R(*q)
        hand_world = np.array(pos_world) - TCP * R[:, 2]
        hand_base = hand_world - BASE_IN_WORLD
        return self.ik(hand_base, q, seed)

    def fk(self, q, link="panda_hand"):
        self.fk_cli.wait_for_service(timeout_sec=10)
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = [link]
        js = JointState()
        js.name = list(ARM)
        js.position = [float(v) for v in q]
        req.robot_state.joint_state = js
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {res}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z])
        quat = (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        return pos, quat

    def tcp_world(self, q=None):
        q = q if q is not None else self.arm_q()
        pos, quat = self.fk(q)
        R = quat_R(*quat)
        return pos + BASE_IN_WORLD + TCP * R[:, 2], quat

    # ---- motion ----
    def move(self, waypoints, times):
        """waypoints: list of 7-vectors; times: cumulative seconds."""
        self.fjt.wait_for_server(timeout_sec=10)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        for q, t in zip(waypoints, times):
            pt = JointTrajectoryPoint(positions=[float(v) for v in q])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res)
        code = res.result().result.error_code
        q_now = self.arm_q()
        err = float(np.max(np.abs(np.array(q_now) - np.array(waypoints[-1]))))
        print(f"move: error_code={code} max_joint_err={err:.4f}")
        return code, err

    def gripper(self, width):
        self.grip.wait_for_server(timeout_sec=10)
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=120)
        r = rf.result().result
        f = self.fingers()
        print(f"gripper({width}): reached={r.reached_goal} stalled={r.stalled} fingers={f}")
        return f
OPENRUA_EOF

# openrua op 17
timeout 200 python3 - <<'EOF'
import numpy as np, math
from lib import *
r=Robot()
q0=r.arm_q(); print('q0',np.round(q0,3))
pos,quat=r.tcp_world(q0); print('tcp world now',np.round(pos,3), np.round(quat,3))
book_yaw=math.radians(-13.4)
targets={
 'pre_grasp': ((-0.101,0.004,1.25), book_yaw),
 'grasp':     ((-0.101,0.004,1.04), book_yaw),
 'lift':      ((-0.101,0.004,1.30), book_yaw),
 'above_slot':((-0.425,-0.145,1.30), math.pi/2),
 'place':     ((-0.425,-0.145,1.09), math.pi/2),
}
seed=q0
for k,(p,yaw) in targets.items():
    q=r.ik_tcp_world(p,yaw,seed)
    if q is None: print(k,'IK FAILED'); continue
    chk,_=r.tcp_world(q)
    print(k, np.round(q,3), 'fk tcp', np.round(chk,3), 'delta from seed', np.round(np.array(q)-np.array(seed),2))
    seed=q
EOF

# openrua op 18
timeout 200 python3 - <<'EOF'
import numpy as np, math
from lib import *
r=Robot()
q0=r.arm_q()
# raw FK
req=GetPositionFK.Request(); req.header.frame_id=""; req.fk_link_names=["panda_hand","panda_link0"]
js=JointState(); js.name=list(ARM); js.position=[float(v) for v in q0]; req.robot_state.joint_state=js
r.fk_cli.wait_for_service(5); fut=r.fk_cli.call_async(req); rclpy.spin_until_future_complete(r.node,fut,timeout_sec=30)
res=fut.result()
for ps in res.pose_stamped: print(ps.header.frame_id, ps.pose.position)
# try IK with the current hand pose in both frames
for label,pos in [('base',(0.457,0,0.358)),('world',(-0.293,0,1.27))]:
    q=r.ik(pos,(1,0,-0.028,0),q0)
    print(label, None if q is None else np.round(q,3))
EOF

# openrua op 19
python3 - <<'EOF'
import re
s=open('lib.py').read()
s=s.replace('''        hand_world = np.array(pos_world) - TCP * R[:, 2]
        hand_base = hand_world - BASE_IN_WORLD
        return self.ik(hand_base, q, seed)''','''        hand_world = np.array(pos_world) - TCP * R[:, 2]
        # machine fact (probed): MoveIt's model frame here IS world
        return self.ik(hand_world, q, seed)''')
s=s.replace('''        return pos + BASE_IN_WORLD + TCP * R[:, 2], quat''','''        return pos + TCP * R[:, 2], quat  # FK already answers in world''')
open('lib.py','w').write(s)
EOF
timeout 300 python3 - <<'EOF'
import numpy as np, math
from lib import *
r=Robot()
q0=r.arm_q()
pos,quat=r.tcp_world(q0); print('tcp world now',np.round(pos,3), np.round(quat,3))
qs=r.ik((-0.293,0,1.27),(1,0,-0.028,0),q0); print('sol',np.round(qs,3)); print('fk of sol', np.round(r.fk(qs)[0],3), np.round(r.fk(qs)[1],3))
book_yaw=math.radians(-13.4)
targets={
 'pre_grasp': ((-0.101,0.004,1.25), book_yaw),
 'grasp':     ((-0.101,0.004,1.04), book_yaw),
 'lift':      ((-0.101,0.004,1.30), book_yaw),
 'above_slot':((-0.425,-0.145,1.30), math.pi/2),
 'place':     ((-0.425,-0.145,1.09), math.pi/2),
}
seed=q0
for k,(p,yaw) in targets.items():
    q=r.ik_tcp_world(p,yaw,seed)
    if q is None: print(k,'IK FAILED'); continue
    chk,cq=r.tcp_world(q)
    print(k, np.round(q,3), 'fk tcp', np.round(chk,3), np.round(cq,3), 'delta', np.round(np.array(q)-np.array(seed),2))
    seed=q
EOF

# openrua op 20
python3 - <<'EOF'
s=open('lib.py').read()
s=s.replace('''        req.ik_request.group_name = M["planning"]["group"]
''','''        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.ik_link_name = "panda_hand"  # group tip is link8 (45 deg off)
''')
open('lib.py','w').write(s)
EOF
timeout 300 python3 - <<'EOF'
import numpy as np, math
from lib import *
r=Robot()
q0=r.arm_q()
book_yaw=math.radians(-13.4)
targets={
 'pre_grasp': ((-0.101,0.004,1.25), book_yaw),
 'grasp':     ((-0.101,0.004,1.04), book_yaw),
 'lift':      ((-0.101,0.004,1.30), book_yaw),
 'above_slot':((-0.425,-0.145,1.30), math.pi/2),
 'place':     ((-0.425,-0.145,1.09), math.pi/2),
}
seed=q0
for k,(p,yaw) in targets.items():
    q=r.ik_tcp_world(p,yaw,seed)
    if q is None: print(k,'IK FAILED'); continue
    chk,cq=r.tcp_world(q)
    print(k, np.round(q,3), 'fk tcp', np.round(chk,3), 'want quat', np.round(down_quat(yaw),3), 'got', np.round(cq,3), 'delta', np.round(np.array(q)-np.array(seed),2))
    seed=q
EOF

# openrua op 21
timeout 300 python3 - <<'EOF'
import numpy as np, math
from lib import *
r=Robot()
lift=[-0.011,0.287,0.02,-1.532,-0.006,1.819,1.029]
for yaw in [math.pi/2, -math.pi/2]:
  for z in [1.30, 1.25]:
    q=r.ik_tcp_world((-0.425,-0.145,z),yaw,lift)
    if q is None: print(yaw,z,'FAIL'); continue
    print(f'yaw {math.degrees(yaw):.0f} z {z}', np.round(q,3), 'delta', np.round(np.array(q)-np.array(lift),2))
    # intermediate midpoint in joint space -> where's the tcp?
    mid=(np.array(q)+np.array(lift))/2
    print('   midpoint tcp', np.round(r.tcp_world(mid)[0],3))
    q2=r.ik_tcp_world((-0.425,-0.145,1.09),yaw,q)
    print('   place', None if q2 is None else np.round(q2,3))
EOF

# openrua op 22
timeout 600 python3 - <<'EOF'
import numpy as np, math
from lib import *
r=Robot()
r.gripper(0.04)
q0=r.arm_q()
book_yaw=math.radians(-13.4)
pre=r.ik_tcp_world((-0.101,0.004,1.25),book_yaw,q0)
print('pre',np.round(pre,3))
r.move([pre],[4.0])
print('tcp now',np.round(r.tcp_world()[0],3))
print('fingers',r.fingers())
EOF
timeout 60 python3 tools/perception/cam_snap.py agentview img/agent_pre.png

# openrua op 23
timeout 100 python3 -c "
from lib import *; import numpy as np
r=Robot(); print(np.round(r.arm_q(),3)); print('tcp',np.round(r.tcp_world()[0],4))
"

# openrua op 24
timeout 300 python3 - <<'EOF'
import numpy as np
from lib import *
r=Robot()
pre=[-0.009,0.303,0.017,-1.637,-0.005,1.94,1.029]
r.move([pre],[2.0])
print(np.round(r.arm_q(),3)); print('tcp',np.round(r.tcp_world()[0],4))
EOF

# openrua op 25
python3 - <<'EOF'
s=open('lib.py').read()
s=s.replace('''        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res)
        code = res.result().result.error_code
        q_now = self.arm_q()
        err = float(np.max(np.abs(np.array(q_now) - np.array(waypoints[-1]))))
        print(f"move: error_code={code} max_joint_err={err:.4f}")
        return code, err''','''        # hold the last point briefly: the controller settles late
        pt = JointTrajectoryPoint(positions=[float(v) for v in waypoints[-1]])
        t = times[-1] + 1.0
        pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
        goal.trajectory.points.append(pt)
        for attempt in range(3):
            send = self.fjt.send_goal_async(goal)
            rclpy.spin_until_future_complete(self.node, send)
            res = send.result().get_result_async()
            rclpy.spin_until_future_complete(self.node, res)
            code = res.result().result.error_code
            q_now = self.arm_q()
            err = float(np.max(np.abs(np.array(q_now) - np.array(waypoints[-1]))))
            print(f"move: error_code={code} max_joint_err={err:.4f}")
            if err < 0.01:
                break
            # resend only the final point (short) to converge
            goal.trajectory.points = goal.trajectory.points[-1:]
            goal.trajectory.points[0].time_from_start = Duration(sec=1, nanosec=500000000)
        return code, err''')
open('lib.py','w').write(s)
EOF
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand img/eih_pre.png

# openrua op 26
timeout 600 python3 - <<'EOF'
import numpy as np, math
from lib import *
r=Robot()
book_yaw=math.radians(-13.4)
q0=r.arm_q()
g=r.ik_tcp_world((-0.101,0.004,1.04),book_yaw,q0)
print('grasp q',np.round(g,3),'delta',np.round(np.array(g)-np.array(q0),2))
r.move([g],[2.5])
print('tcp now',np.round(r.tcp_world()[0],4))
r.gripper(0.0)
print('fingers',r.fingers())
EOF

# openrua op 27
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand img/eih_grasp.png; timeout 60 python3 tools/perception/cam_snap.py agentview img/agent_grasp.png; timeout 20 ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once | head -20

# openrua op 28
timeout 600 python3 - <<'EOF'
import numpy as np, math
from lib import *
r=Robot()
book_yaw=math.radians(-13.4)
q0=r.arm_q()
g=r.ik_tcp_world((-0.101,0.004,1.20),book_yaw,q0)
r.move([g],[2.5])
print('tcp now',np.round(r.tcp_world()[0],4)); print('fingers',r.fingers())
EOF
timeout 60 python3 tools/perception/cam_snap.py agentview img/agent_lift.png

# openrua op 29
timeout 60 python3 tools/perception/cam_snap.py birdview img/bird2.png; timeout 60 python3 tools/perception/cam_snap.py frontview img/front2.png; python3 -c "
import cv2
a=cv2.imread('img/birdview.png')[200:320,240:400]; b=cv2.imread('img/bird2.png')[200:320,240:400]
import numpy as np
cv2.imwrite('img/bird_cmp.png', cv2.resize(np.hstack([a,b]),None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
a=cv2.imread('img/frontview.png')[240:400,240:400]; b=cv2.imread('img/front2.png')[240:400,240:400]
cv2.imwrite('img/front_cmp.png', cv2.resize(np.hstack([a,b]),None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
"

# openrua op 30
python3 -c "
import cv2, numpy as np
a=cv2.imread('img/frontview.png'); b=cv2.imread('img/front2.png')
cv2.imwrite('img/front_full_cmp.png', np.hstack([a,b]))
"

# openrua op 31
timeout 120 python3 cloud.py robot0_eye_in_hand agentview birdview && python3 - <<'EOF'
import numpy as np
for cam in ['robot0_eye_in_hand','agentview','birdview']:
    p=np.load(f'img/{cam}_cloud.npy')
    m=(p[:,0]>-0.25)&(p[:,0]<0.0)&(p[:,1]>-0.1)&(p[:,1]<0.12)&(p[:,2]>0.95)&(p[:,2]<1.10)
    t=p[m]
    if not len(t): print(cam,'none'); continue
    print(cam, 'n',len(t),'z',t[:,2].min().round(3),t[:,2].max().round(3))
    for zl in [0.95,1.0,1.03,1.05,1.06,1.07]:
        s=t[(t[:,2]>=zl)&(t[:,2]<zl+0.01)]
        if len(s): print(f'  z{zl:.2f}: x {s[:,0].min():.3f}..{s[:,0].max():.3f} y {s[:,1].min():.3f}..{s[:,1].max():.3f} n={len(s)} cx={s[:,0].mean():.3f} cy={s[:,1].mean():.3f}')
EOF

# openrua op 32
timeout 200 python3 cloud.py birdview agentview frontview sideview robot0_robotview paperview galleryview robot0_eye_in_hand >/dev/null && python3 - <<'EOF'
import numpy as np
cams=['birdview','agentview','frontview','sideview','robot0_robotview','paperview','galleryview','robot0_eye_in_hand']
pts=np.concatenate([np.load(f'img/{c}_cloud.npy') for c in cams])
def hmap(x0,x1,y0,y1,res,zmax=1.15):
    m=(pts[:,0]>x0)&(pts[:,0]<x1)&(pts[:,1]>y0)&(pts[:,1]<y1)&(pts[:,2]>0.85)&(pts[:,2]<zmax)
    p=pts[m]; xs=np.arange(x0,x1,res); ys=np.arange(y0,y1,res)
    H=np.full((len(xs),len(ys)),np.nan)
    ix=((p[:,0]-x0)/res).astype(int); iy=((p[:,1]-y0)/res).astype(int)
    ok=(ix>=0)&(ix<len(xs))&(iy>=0)&(iy<len(ys))
    for a,b,z in zip(ix[ok],iy[ok],p[ok,2]):
        if np.isnan(H[a,b]) or z>H[a,b]: H[a,b]=z
    print('       y:'+''.join(f'{y:+.3f}'[-3:].rjust(3) if i%2==0 else '   ' for i,y in enumerate(ys)))
    for i,x in enumerate(xs):
        print(f'x={x:+.3f} '+''.join((' . ' if np.isnan(h) else f'{int(round((h-0.88)*100)):2d} ') for h in H[i]))
print('table z', np.median(pts[(pts[:,0]>0.0)&(pts[:,0]<0.2)&(abs(pts[:,1])<0.2)&(pts[:,2]>0.8)&(pts[:,2]<0.95),2]))
hmap(-0.49,-0.31,-0.38,0.08,0.01)
print()
hmap(-0.18,-0.02,-0.04,0.06,0.005)
EOF

# openrua op 33
python3 - <<'EOF'
import numpy as np
p=np.load('img/robot0_eye_in_hand_cloud.npy')
m=(p[:,0]>-0.2)&(p[:,0]<0.0)&(p[:,1]>-0.06)&(p[:,1]<0.08)&(p[:,2]>1.005)&(p[:,2]<1.05)
t=p[m]; print(len(t),'z',t[:,2].min(),t[:,2].max())
xy=t[:,:2]; c=xy.mean(0); u,s,vt=np.linalg.svd(xy-c,full_matrices=False)
proj=(xy-c)@vt.T
print('center',c.round(4),'yaw deg',np.degrees(np.arctan2(vt[0,1],vt[0,0])).round(2))
print('major',proj[:,0].min().round(4),proj[:,0].max().round(4),'minor',proj[:,1].min().round(4),proj[:,1].max().round(4))
# thickness via percentiles
print('minor 5/95 pct', np.percentile(proj[:,1],[2,98]).round(4))
EOF

# openrua op 34
timeout 600 python3 - <<'EOF'
import numpy as np, math
from lib import *
r=Robot()
r.gripper(0.04)
yaw=math.radians(-12.9)
q0=r.arm_q()
pre=r.ik_tcp_world((-0.098,0.006,1.10),yaw,q0); print('pre',np.round(pre,3))
r.move([pre],[2.5]); print('tcp',np.round(r.tcp_world()[0],4))
g=r.ik_tcp_world((-0.098,0.006,0.98),yaw,pre); print('grasp',np.round(g,3),'delta',np.round(np.array(g)-np.array(pre),2))
r.move([g],[2.5]); print('tcp',np.round(r.tcp_world()[0],4))
EOF
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand img/eih_grasp2.png; timeout 60 python3 tools/perception/cam_snap.py agentview img/agent_grasp2.png

# openrua op 35
timeout 300 python3 - <<'EOF'
import numpy as np
from lib import *
r=Robot()
r.gripper(0.0)
import time
for i in range(3):
    r.spin(0.3); print('fingers',r.fingers())
EOF
timeout 20 ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once | grep -A3 force

# openrua op 36
timeout 600 python3 - <<'EOF'
import numpy as np, math
from lib import *
r=Robot()
yaw=math.radians(-12.9)
q0=r.arm_q()
lift=r.ik_tcp_world((-0.098,0.006,1.25),yaw,q0); print('lift',np.round(lift,3),'delta',np.round(np.array(lift)-np.array(q0),2))
r.move([lift],[3.0]); print('tcp',np.round(r.tcp_world()[0],4)); print('fingers',r.fingers())
EOF
timeout 60 python3 tools/perception/cam_snap.py agentview img/agent_lift2.png

# openrua op 37
timeout 600 python3 - <<'EOF'
import numpy as np, math
from lib import *
r=Robot()
q0=r.arm_q()
best=None
for yaw in [math.pi/2, -math.pi/2]:
    q=r.ik_tcp_world((-0.42,-0.145,1.25),yaw,q0)
    if q is None: print(yaw,'fail'); continue
    d=np.array(q)-np.array(q0)
    print(f'yaw {math.degrees(yaw):.0f}', np.round(q,3),'delta',np.round(d,2))
    # sample the joint-space path
    zs=[]
    for s in np.linspace(0,1,11):
        qi=np.array(q0)+s*d
        p,_=r.tcp_world(qi); zs.append(p.round(3))
    print('  tcp path:', [tuple(z) for z in zs[::2]])
    minz=min(z[2] for z in zs); print('  min tcp z',minz)
    if best is None or (minz>1.15 and np.abs(d).sum()<np.abs(np.array(best[1])-np.array(q0)).sum()): best=(yaw,q)
print('chosen yaw',best[0])
np.save('img/above_slot_q.npy',np.array(best[1])); open('img/above_slot_yaw.txt','w').write(str(best[0]))
EOF

# openrua op 38
timeout 600 python3 - <<'EOF'
import numpy as np, math
from lib import *
r=Robot()
q=list(np.load('img/above_slot_q.npy'))
r.move([q],[4.0]); p,quat=r.tcp_world(); print('tcp',np.round(p,4),'quat',np.round(quat,3)); print('fingers',r.fingers())
EOF
timeout 60 python3 tools/perception/cam_snap.py agentview img/agent_above.png; timeout 60 python3 tools/perception/cam_snap.py sideview img/side_above.png

# openrua op 39
timeout 600 python3 - <<'EOF'
import numpy as np, math
from lib import *
r=Robot()
q=list(np.load('img/above_slot_q.npy'))
print('now',np.round(r.arm_q(),3)); print('target',np.round(q,3))
for i in range(4):
    code,err=r.move([q],[3.0])
    if err<0.01: break
p,quat=r.tcp_world(); print('tcp',np.round(p,4),'yaw deg',round(math.degrees(2*math.atan2(quat[1],quat[0])),1)); print('fingers',r.fingers())
EOF

# openrua op 40
timeout 60 python3 tools/perception/cam_snap.py agentview img/agent_above.png; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand img/eih_above.png

# openrua op 41
timeout 600 python3 - <<'EOF'
import numpy as np, math
from lib import *
r=Robot()
q0=r.arm_q()
q=r.ik_tcp_world((-0.42,-0.145,1.10),math.pi/2,q0); print('delta',np.round(np.array(q)-np.array(q0),2))
r.move([q],[2.5]); p,quat=r.tcp_world(); print('tcp',np.round(p,4),'yaw',round(math.degrees(2*math.atan2(quat[1],quat[0])),1)); print('fingers',r.fingers())
EOF
timeout 120 python3 cloud.py robot0_eye_in_hand sideview agentview > /dev/null; python3 - <<'EOF'
import numpy as np
pts=np.concatenate([np.load(f'img/{c}_cloud.npy') for c in ['robot0_eye_in_hand','sideview','agentview']])
x0,x1,y0,y1,res=-0.48,-0.33,-0.25,-0.04,0.005
m=(pts[:,0]>x0)&(pts[:,0]<x1)&(pts[:,1]>y0)&(pts[:,1]<y1)&(pts[:,2]>0.85)&(pts[:,2]<1.30)
p=pts[m]; xs=np.arange(x0,x1,res); ys=np.arange(y0,y1,res)
H=np.full((len(xs),len(ys)),np.nan)
ix=((p[:,0]-x0)/res).astype(int); iy=((p[:,1]-y0)/res).astype(int)
ok=(ix>=0)&(ix<len(xs))&(iy>=0)&(iy<len(ys))
for a,b,z in zip(ix[ok],iy[ok],p[ok,2]):
    if np.isnan(H[a,b]) or z>H[a,b]: H[a,b]=z
print('       y:'+''.join(f'{y:+.3f}'[-3:].rjust(3) if i%2==0 else '   ' for i,y in enumerate(ys)))
for i,x in enumerate(xs):
    print(f'x={x:+.3f} '+''.join((' . ' if np.isnan(h) else f'{int(round((h-0.88)*100)):2d} ') for h in H[i]))
EOF

# openrua op 42
python3 - <<'EOF'
import numpy as np
for cam in ['robot0_eye_in_hand','sideview','agentview']:
    p=np.load(f'img/{cam}_cloud.npy')
    m=(p[:,0]>-0.47)&(p[:,0]<-0.37)&(p[:,1]>-0.24)&(p[:,1]<-0.05)&(p[:,2]>1.11)&(p[:,2]<1.16)
    t=p[m]
    if len(t)<20: print(cam,'few',len(t)); continue
    xy=t[:,:2]; c=xy.mean(0); u,s,vt=np.linalg.svd(xy-c,full_matrices=False)
    proj=(xy-c)@vt.T
    yaw=np.degrees(np.arctan2(vt[0,1],vt[0,0]))
    print(cam,'n',len(t),'z',t[:,2].min().round(3),t[:,2].max().round(3),'center',c.round(4),'yaw',yaw.round(2),'major',proj[:,0].min().round(4),proj[:,0].max().round(4),'minor pct',np.percentile(proj[:,1],[2,98]).round(4))
    print('   y extent', xy[:,1].min().round(4), xy[:,1].max().round(4), 'x extent', xy[:,0].min().round(4), xy[:,0].max().round(4))
EOF

# openrua op 43
python3 - <<'EOF'
import numpy as np
for cam in ['sideview','agentview','robot0_eye_in_hand']:
    p=np.load(f'img/{cam}_cloud.npy')
    m=(p[:,0]>-0.45)&(p[:,0]<-0.39)&(p[:,1]>-0.215)&(p[:,1]<-0.075)&(p[:,2]>1.125)&(p[:,2]<1.15)
    t=p[m]
    if len(t)<20: print(cam,'few',len(t)); continue
    xy=t[:,:2]; c=xy.mean(0); u,s,vt=np.linalg.svd(xy-c,full_matrices=False)
    proj=(xy-c)@vt.T
    yaw=np.degrees(np.arctan2(vt[0,1],vt[0,0]))
    print(cam,'n',len(t),'center',c.round(4),'yaw',yaw.round(2),'major',proj[:,0].min().round(4),proj[:,0].max().round(4),'minor pct',np.percentile(proj[:,1],[2,98]).round(4))
    # per-y-bin mean x to see the tilt
    for yb in np.arange(-0.21,-0.08,0.02):
        s=t[(t[:,1]>=yb)&(t[:,1]<yb+0.02)]
        if len(s): print(f'   y {yb:.2f}: x mean {s[:,0].mean():.4f} min {s[:,0].min():.4f} max {s[:,0].max():.4f} n={len(s)}')
EOF

# openrua op 44
timeout 60 python3 tools/perception/cam_snap.py agentview img/agent_110.png; timeout 60 python3 tools/perception/cam_snap.py sideview img/side_110.png; timeout 20 ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once | grep -A3 force; python3 -c "
import cv2
im=cv2.imread('img/agent_110.png'); cv2.imwrite('img/agent_110_crop.png', cv2.resize(im[80:300,150:420],None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('img/side_110.png'); cv2.imwrite('img/side_110_crop.png', cv2.resize(im[120:340,330:560],None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
"

# openrua op 45
python3 - <<'EOF'
import struct, numpy as np
f='/opt/ros/jazzy/share/moveit_resources_panda_description/meshes/collision/finger.stl'
d=open(f,'rb').read()
if d[:5]==b'solid' and b'facet' in d[:300]:
    import re
    v=np.array([list(map(float,m.groups())) for m in re.finditer(rb'vertex\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)',d)])
else:
    n=struct.unpack_from('<I',d,80)[0]
    arr=np.frombuffer(d[84:84+n*50],dtype=np.dtype([('n','<3f4'),('v','<9f4'),('a','<u2')]))
    v=arr['v'].reshape(-1,3)
print('finger mesh bbox min',v.min(0).round(4),'max',v.max(0).round(4))
# pad region: z > 0.04 (near tip)
tip=v[v[:,2]>0.04]; print('tip region (z>0.04) bbox min',tip.min(0).round(4),'max',tip.max(0).round(4))
EOF

# openrua op 46
find / -iname "finger*.stl" -o -iname "finger*.dae" -o -iname "finger*.obj" 2>/dev/null | head; 

# openrua op 47
timeout 600 python3 - <<'EOF'
import numpy as np, math
from lib import *
r=Robot()
q0=r.arm_q()
q=r.ik_tcp_world((-0.42,-0.145,1.085),math.pi/2,q0); print('delta',np.round(np.array(q)-np.array(q0),2))
r.move([q],[2.0]); p,quat=r.tcp_world(); print('tcp',np.round(p,4),'yaw',round(math.degrees(2*math.atan2(quat[1],quat[0])),1)); print('fingers',r.fingers())
EOF
timeout 20 ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once | grep -A3 force

# openrua op 48
timeout 60 python3 tools/perception/cam_snap.py agentview img/agent_release_pre.png; python3 -c "
import cv2
im=cv2.imread('img/agent_release_pre.png'); cv2.imwrite('img/agent_release_pre_crop.png', cv2.resize(im[80:300,150:420],None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))"

# openrua op 49
timeout 300 python3 - <<'EOF'
import numpy as np, math
from lib import *
r=Robot()
r.gripper(0.04)
q0=r.arm_q()
q=r.ik_tcp_world((-0.42,-0.145,1.25),math.pi/2,q0)
r.move([q],[2.5]); print('tcp',np.round(r.tcp_world()[0],4))
EOF
timeout 60 python3 tools/perception/cam_snap.py agentview img/agent_after.png; timeout 60 python3 tools/perception/cam_snap.py sideview img/side_after.png

# openrua op 50
python3 -c "
import cv2
im=cv2.imread('img/agent_after.png'); cv2.imwrite('img/agent_after_crop.png', cv2.resize(im[140:260,180:380],None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('img/side_after.png'); cv2.imwrite('img/side_after_crop.png', cv2.resize(im[180:340,360:540],None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))"
timeout 200 python3 cloud.py birdview agentview sideview frontview robot0_robotview robot0_eye_in_hand >/dev/null; python3 - <<'EOF'
import numpy as np
cams=['birdview','agentview','sideview','frontview','robot0_robotview','robot0_eye_in_hand']
pts=np.concatenate([np.load(f'img/{c}_cloud.npy') for c in cams])
x0,x1,y0,y1,res=-0.48,-0.32,-0.25,-0.04,0.005
m=(pts[:,0]>x0)&(pts[:,0]<x1)&(pts[:,1]>y0)&(pts[:,1]<y1)&(pts[:,2]>0.85)&(pts[:,2]<1.12)
p=pts[m]; xs=np.arange(x0,x1,res); ys=np.arange(y0,y1,res)
H=np.full((len(xs),len(ys)),np.nan)
ix=((p[:,0]-x0)/res).astype(int); iy=((p[:,1]-y0)/res).astype(int)
ok=(ix>=0)&(ix<len(xs))&(iy>=0)&(iy<len(ys))
for a,b,z in zip(ix[ok],iy[ok],p[ok,2]):
    if np.isnan(H[a,b]) or z>H[a,b]: H[a,b]=z
print('       y:'+''.join(f'{y:+.3f}'[-3:].rjust(3) if i%2==0 else '   ' for i,y in enumerate(ys)))
for i,x in enumerate(xs):
    print(f'x={x:+.3f} '+''.join((' . ' if np.isnan(h) else f'{int(round((h-0.88)*100)):2d} ') for h in H[i]))
# book anywhere else? check the original book spot
m2=(pts[:,0]>-0.2)&(pts[:,0]<0.0)&(abs(pts[:,1])<0.08)&(pts[:,2]>0.9)&(pts[:,2]<1.1)
print('points at old book location above table:', m2.sum())
EOF

# openrua op 51
mkdir -p "$(dirname /claude-config/projects/-workspace/memory/panda-workstation-quirks.md)"
cat > /claude-config/projects/-workspace/memory/panda-workstation-quirks.md <<'OPENRUA_EOF'
---
name: panda-workstation-quirks
description: Verified machine quirks of the /workspace Panda arm sim (MoveIt frame, IK tip link, unsettled scene, controller lag)
metadata:
  type: project
---

Facts probed on the Franka Panda workstation (/workspace, Sept 2026) that
differ from or go beyond docs/30-action.md:

- MoveIt `/compute_ik` and `/compute_fk` answer in the WORLD frame (panda_link0
  sits at world (-0.75, 0, 0.912)), not the arm base as the docs claim.
  Leave `frame_id` empty and pass world coordinates.
- The `panda_arm` group tip is `panda_link8`, which is yawed 45° from
  `panda_hand`. Set `ik_request.ik_link_name = "panda_hand"` or the fingers
  end up 45° off the intended closing direction.
- At session start objects may float above their supports (a book sat 5 cm
  high); the scene settles only once the first motion advances the clock.
  Re-measure object heights after the first move before planning a grasp.
- FollowJointTrajectory settles late: max joint error ~0.05 rad right after the
  result; append a 1 s hold point and/or resend the final point until the error
  is < 0.01 rad. Large joint7 rotations return error_code -5 but converge on resend.
- Gripper `reached_goal=True` at 0.004 m does NOT mean an object is held
  (goal tolerance ~1 cm); check fingers stay well above closed_m after lifting.

**Why:** each of these cost a failed step during the book-into-caddy task.
**How to apply:** use /workspace/lib.py (Robot class) which already encodes them.
OPENRUA_EOF

# openrua op 52
cd /claude-config/projects/-workspace/memory && (test -f MEMORY.md && cat MEMORY.md || echo "(no index yet)")

# openrua op 53
mkdir -p "$(dirname /claude-config/projects/-workspace/memory/MEMORY.md)"
cat > /claude-config/projects/-workspace/memory/MEMORY.md <<'OPENRUA_EOF'
- [Panda workstation quirks](panda-workstation-quirks.md) — MoveIt answers in world frame, IK tip is link8 (45° off hand), scene unsettled at start, controller settles late
OPENRUA_EOF
