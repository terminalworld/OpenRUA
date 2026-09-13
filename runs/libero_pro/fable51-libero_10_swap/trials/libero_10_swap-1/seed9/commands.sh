#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 2 -type f | head -50

# openrua op 1
cat README.md machine.yaml docs/10-machine.md docs/20-perception.md docs/30-action.md docs/40-patterns.md tools/README.md; ls -la tools .ros

# openrua op 2
ls -la tools/action tools/perception; cat tools/action/*.py tools/perception/*.py

# openrua op 3
timeout 30 ros2 topic list; echo ---; timeout 20 ros2 action list; echo ---; timeout 20 ros2 service list | head -40; echo ---; timeout 20 ros2 node list

# openrua op 4
timeout 60 ros2 topic echo /joint_states --once | head -40; for c in agentview birdview frontview sideview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c; done

# openrua op 5
timeout 60 python3 tools/perception/cam_snap.py robot0_robotview && python3 -c "
import cv2
im=cv2.imread('agentview.png'); crop=im[220:420,100:420]; cv2.imwrite('agent_crop.png', cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('birdview.png'); crop=im[200:330,230:420]; cv2.imwrite('bird_crop.png', cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))
"

# openrua op 6
timeout 60 python3 -c "
import rclpy, time
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
rclpy.init(); n=rclpy.create_node('tfdump')
qos=QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
got=[]
n.create_subscription(TFMessage,'/tf_static',lambda m:got.extend(m.transforms),qos)
n.create_subscription(TFMessage,'/tf',lambda m:got.extend(m.transforms),10)
t0=time.time()
while time.time()-t0<5: rclpy.spin_once(n,timeout_sec=0.2)
seen=set()
for t in got:
    k=(t.header.frame_id,t.child_frame_id)
    if k in seen: continue
    seen.add(k); tr=t.transform.translation; q=t.transform.rotation
    print(f'{k[0]:>28} -> {k[1]:<32} t=({tr.x:.3f},{tr.y:.3f},{tr.z:.3f}) q=({q.x:.3f},{q.y:.3f},{q.z:.3f},{q.w:.3f})')
"

# openrua op 7
mkdir -p /workspace/work && cat > /workspace/work/cloud.py <<'EOF'
"""Grab color+depth+info for a camera, return world-frame XYZ per pixel."""
import numpy as np, rclpy, yaml
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

def quat_R(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

def grab(node, topic, typ, timeout=30):
    got={}
    s=node.create_subscription(typ, topic, lambda m: got.setdefault('m',m), 1)
    import time; t0=time.time()
    while 'm' not in got and time.time()-t0<timeout: rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(s)
    if 'm' not in got: raise SystemExit('no msg on '+topic)
    return got['m']

def cam_tf(node, cam, timeout=10):
    qos=QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    got={}
    def cb(m):
        for t in m.transforms:
            if t.child_frame_id==f'{cam}_optical_frame' and t.header.frame_id=='world': got['t']=t
    s=node.create_subscription(TFMessage,'/tf_static',cb,qos)
    s2=node.create_subscription(TFMessage,'/tf',cb,10)
    import time; t0=time.time()
    while 't' not in got and time.time()-t0<timeout: rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(s); node.destroy_subscription(s2)
    t=got['t'].transform; q=t.rotation
    T=np.eye(4); T[:3,:3]=quat_R(q.x,q.y,q.z,q.w); T[:3,3]=[t.translation.x,t.translation.y,t.translation.z]
    return T

def cloud(cam, node=None):
    own = node is None
    if own:
        rclpy.init(); node=rclpy.create_node('cloud_'+cam)
    br=CvBridge()
    color=br.imgmsg_to_cv2(grab(node,f'/{cam}/color/image_raw',Image),'bgr8')
    depth=br.imgmsg_to_cv2(grab(node,f'/{cam}/depth/image_raw',Image),'passthrough').astype(np.float32)
    info=grab(node,f'/{cam}/color/camera_info',CameraInfo)
    T=cam_tf(node,cam)
    fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
    h,w=depth.shape
    u,v=np.meshgrid(np.arange(w),np.arange(h))
    pc=np.stack([(u-cx)*depth/fx,(v-cy)*depth/fy,depth,np.ones_like(depth)],-1)
    W=(pc.reshape(-1,4)@T.T)[:,:3].reshape(h,w,3)
    if own: node.destroy_node(); rclpy.shutdown()
    return color, depth, W, (fx,fy,cx,cy), T
EOF
cd /workspace/work && timeout 120 python3 -c "
from cloud import cloud
import numpy as np, cv2
color,depth,W,K,T=cloud('birdview')
np.save('bird_W.npy',W); cv2.imwrite('bird_color.png',color)
z=W[:,:,2]
print('depth range',np.nanmin(depth),np.nanmax(depth))
# table height histogram
vals=z[np.isfinite(z)]
hist,edges=np.histogram(vals,bins=np.arange(0,1.2,0.01))
for h,e in zip(hist,edges):
    if h>200: print(f'{e:.2f} {h}')
"

# openrua op 8
python3 -c "
import numpy as np, cv2
W=np.load('bird_W.npy'); color=cv2.imread('bird_color.png')
z=W[:,:,2]
mask=((z>0.435)&(z<0.75)).astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,4]<15: continue
    m=lab==i
    pts=W[m]; c=color[m].mean(0)
    x0,y0,w,h,a=stats[i]
    print(f'comp{i} px={a} bbox=({x0},{y0},{w},{h}) cen_px=({cent[i][0]:.0f},{cent[i][1]:.0f}) world x={pts[:,0].mean():.3f} y={pts[:,1].mean():.3f} zmax={pts[:,2].max():.3f} zmed={np.median(pts[:,2]):.3f} xr=({pts[:,0].min():.3f},{pts[:,0].max():.3f}) yr=({pts[:,1].min():.3f},{pts[:,1].max():.3f}) bgr={c.astype(int)}')
"


# openrua op 9
timeout 120 python3 -c "
from cloud import cloud
import numpy as np, cv2
color,depth,W,K,T=cloud('agentview')
np.save('agent_W.npy',W); cv2.imwrite('agent_color.png',color)
z=W[:,:,2]
mask=((z>0.435)&(z<0.75)&(W[:,:,0]<0.3)).astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,4]<30: continue
    m=lab==i; pts=W[m]; c=color[m].mean(0); x0,y0,w,h,a=stats[i]
    print(f'comp{i} px={a} bbox=({x0},{y0},{w},{h}) world x={pts[:,0].mean():.3f} y={pts[:,1].mean():.3f} zmax={pts[:,2].max():.3f} xr=({pts[:,0].min():.3f},{pts[:,0].max():.3f}) yr=({pts[:,1].min():.3f},{pts[:,1].max():.3f}) bgr={c.astype(int)}')
"


# openrua op 10
python3 -c "
import numpy as np, cv2
W=np.load('agent_W.npy'); color=cv2.imread('agent_color.png')
z=W[:,:,2]
m=(z>0.435)&(z<0.75)&(W[:,:,0]<-0.04)&(W[:,:,0]>-0.2)&(W[:,:,1]>0.0)&(W[:,:,1]<0.15)
pts=W[m]; print('butter cand n=',m.sum())
print('x',pts[:,0].min(),pts[:,0].max(),'y',pts[:,1].min(),pts[:,1].max(),'z',pts[:,2].min(),pts[:,2].max())
print('mean',pts.mean(0), 'median', np.median(pts,0))
vs,us=np.where(m); print('px bbox u',us.min(),us.max(),'v',vs.min(),vs.max())
# also cream cheese from agentview
m2=(z>0.435)&(z<0.75)&(W[:,:,0]<0.03)&(W[:,:,0]>-0.08)&(W[:,:,1]>-0.27)&(W[:,:,1]<-0.21)
pts=W[m2]; print('cc n=',m2.sum(),'x',pts[:,0].min(),pts[:,0].max(),'y',pts[:,1].min(),pts[:,1].max(),'z',pts[:,2].min(),pts[:,2].max())
"


# openrua op 11
cd /workspace && timeout 300 python3 tools/action/fjt_send.py 0,-0.9,0,-2.5,0,1.6,0.785 3 && timeout 60 ros2 topic echo /joint_states --once | grep -A9 position

# openrua op 12
timeout 300 python3 tools/action/fjt_send.py 0,-0.9,0,-2.5,0,1.6,0.785 3 && timeout 60 ros2 topic echo /joint_states --once | grep -A9 position && cd work && timeout 120 python3 -c "
from cloud import cloud
import numpy as np, cv2
color,depth,W,K,T=cloud('birdview')
np.save('bird_W.npy',W); cv2.imwrite('bird_color.png',color)
z=W[:,:,2]
mask=((z>0.435)&(z<0.75)).astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,4]<15: continue
    m=lab==i; pts=W[m]; c=color[m].mean(0); x0,y0,w,h,a=stats[i]
    print(f'comp{i} px={a} bbox=({x0},{y0},{w},{h}) world x={pts[:,0].mean():.3f} y={pts[:,1].mean():.3f} zmax={pts[:,2].max():.3f} zmed={np.median(pts[:,2]):.3f} xr=({pts[:,0].min():.3f},{pts[:,0].max():.3f}) yr=({pts[:,1].min():.3f},{pts[:,1].max():.3f}) bgr={c.astype(int)}')
"

# openrua op 13
timeout 60 ros2 param get /move_group robot_description_semantic 2>/dev/null | grep -iE "group name|chain|tip|end_effector|link name" | head -20

# openrua op 14
python3 -c "
from scipy.spatial.transform import Rotation as R
import numpy as np
# hand: z down, fingers (hand y) along world y
Rh=R.from_quat([1,0,0,0])
Rz45=R.from_euler('z',45,degrees=True)
R8=Rh*Rz45   # link8 = hand * Rz(+45) since hand = link8 * Rz(-45)
print('q_link8 xyzw',R8.as_quat())
print('hand from link8:', (R8*R.from_quat([0,0,-0.383,0.924])).as_matrix().round(3))
"

# openrua op 15
mkdir -p "$(dirname /workspace/work/ctl.py)"
cat > /workspace/work/ctl.py <<'OPENRUA_EOF'
"""Reusable controller: IK/FK, trajectory, gripper, joint state. World-frame API."""
import sys, time
import numpy as np, rclpy, yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open('/workspace/machine.yaml'))
FJT = next(a for a in M['actuators'] if a['kind'] == 'joint_trajectory')
GRIP = next(a for a in M['actuators'] if a['kind'] == 'gripper')
ARM = FJT['joints']
BASE_IN_WORLD = np.array([-0.51, 0.0, 0.42])  # world -> panda_link0 (from TF)
TCP = float(M['hand']['tcp_offset_m'])
# link8 orientation for a top-down grasp with fingers closing along world y
Q_DOWN_Y = (0.92387953, -0.38268343, 0.0, 0.0)
# fingers closing along world x (yaw 90 deg)
Q_DOWN_X = (0.38268343, -0.92387953, 0.0, 0.0)


class Ctl:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node('ctl')
        self._js = {}
        self.node.create_subscription(JointState, '/joint_states',
                                      lambda m: self._js.__setitem__('m', m), 10)
        self.ik = self.node.create_client(GetPositionIK, '/compute_ik')
        self.fk = self.node.create_client(GetPositionFK, '/compute_fk')
        self.traj = ActionClient(self.node, FollowJointTrajectory, FJT['port'])
        self.grip = ActionClient(self.node, GripperCommand, GRIP['port'])
        self.ik.wait_for_service(20); self.fk.wait_for_service(20)
        self.traj.wait_for_server(20); self.grip.wait_for_server(20)

    def spin(self, t=0.2):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self, fresh=True):
        if fresh:
            self._js.pop('m', None)
        t0 = time.time()
        while 'm' not in self._js and time.time() - t0 < 30:
            self.spin(0.2)
        m = self._js['m']
        return dict(zip(m.name, m.position))

    def arm_q(self):
        j = self.joints()
        return [j[n] for n in ARM]

    def fingers(self):
        j = self.joints()
        return j['panda_finger_joint1'], j['panda_finger_joint2']

    def _seed(self, q=None):
        q = q or self.arm_q()
        js = JointState(); js.name = list(ARM); js.position = [float(v) for v in q]
        return js

    def solve_ik(self, xyz_world, quat, at_tcp=True, seed=None):
        p = np.array(xyz_world, float)
        if at_tcp:
            from scipy.spatial.transform import Rotation as R
            p = p - TCP * R.from_quat(quat).as_matrix()[:, 2]
        p = p - BASE_IN_WORLD
        req = GetPositionIK.Request()
        req.ik_request.group_name = M['planning']['group']
        req.ik_request.pose_stamped.header.frame_id = ''
        ps = req.ik_request.pose_stamped.pose
        ps.position.x, ps.position.y, ps.position.z = map(float, p)
        ps.orientation.x, ps.orientation.y, ps.orientation.z, ps.orientation.w = map(float, quat)
        req.ik_request.robot_state.joint_state = self._seed(seed)
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = 2
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            print('IK failed', None if res is None else res.error_code.val)
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in ARM]

    def fk_world(self, q=None, link='panda_link8'):
        req = GetPositionFK.Request()
        req.header.frame_id = ''
        req.fk_link_names = [link]
        req.robot_state.joint_state = self._seed(q)
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        ps = res.pose_stamped[0].pose
        p = np.array([ps.position.x, ps.position.y, ps.position.z]) + BASE_IN_WORLD
        q = (ps.orientation.x, ps.orientation.y, ps.orientation.z, ps.orientation.w)
        return p, q

    def tcp_world(self):
        from scipy.spatial.transform import Rotation as R
        p, q = self.fk_world()
        return p + TCP * R.from_quat(q).as_matrix()[:, 2]

    def move(self, q, seconds=3.0, retries=2, tol=0.02):
        """Send a single-point trajectory; verify; resend on tolerance violation."""
        for attempt in range(retries + 1):
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = list(ARM)
            pt = JointTrajectoryPoint(positions=[float(v) for v in q])
            pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
            goal.trajectory.points = [pt]
            f = self.traj.send_goal_async(goal)
            rclpy.spin_until_future_complete(self.node, f, timeout_sec=120)
            rf = f.result().get_result_async()
            rclpy.spin_until_future_complete(self.node, rf, timeout_sec=600)
            code = rf.result().result.error_code
            cur = self.arm_q()
            err = max(abs(a - b) for a, b in zip(cur, q))
            print(f'  move: code={code} max_err={err:.4f}')
            if err < tol:
                return True
        return False

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP['max_effort'])
        f = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, f, timeout_sec=60)
        rf = f.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result
        fg = self.fingers()
        print(f'  gripper({width}): reached={r.reached_goal} stalled={r.stalled} fingers={fg[0]:.4f},{fg[1]:.4f}')
        return fg

    def goto(self, xyz, quat=Q_DOWN_Y, seconds=3.0, at_tcp=True):
        q = self.solve_ik(xyz, quat, at_tcp)
        if q is None:
            return False
        ok = self.move(q, seconds)
        tcp = self.tcp_world()
        print(f'  tcp now {tcp.round(4)} target {np.round(xyz,4)}')
        return ok
OPENRUA_EOF

# openrua op 16
timeout 200 python3 -c "
from ctl import *
c=Ctl()
print('arm q',np.round(c.arm_q(),3),'fingers',c.fingers())
p,q=c.fk_world(); print('link8 world',p.round(4),np.round(q,4))
ph,qh=c.fk_world(link='panda_hand'); print('hand world',ph.round(4),np.round(qh,4))
print('tcp world',c.tcp_world().round(4))
for name,xyz in [('butter_pre',(-0.117,0.039,0.56)),('butter_grasp',(-0.117,0.039,0.44)),('cc_pre',(-0.020,-0.237,0.56)),('cc_grasp',(-0.020,-0.237,0.44)),('basket',(-0.009,0.259,0.62))]:
    s=c.solve_ik(xyz,Q_DOWN_Y)
    if s: 
        pp,qq=c.fk_world(s); tcp=pp+TCP*R.from_quat(qq).as_matrix()[:,2] if False else None
        print(name,np.round(s,3))
" 2>&1 | grep -v Warning

# openrua op 17
timeout 300 python3 -c "
from ctl import *
c=Ctl()
ready=[0,-0.785,0,-2.356,0,1.571,0.785]
p,q=c.fk_world(ready); print('ready link8 world',p.round(3),np.round(q,3))
for name,xyz in [('butter_pre',(-0.117,0.039,0.56)),('butter_grasp',(-0.117,0.039,0.44)),('cc_pre',(-0.020,-0.237,0.56)),('cc_grasp',(-0.020,-0.237,0.44)),('basket',(-0.009,0.259,0.62))]:
    s=c.solve_ik(xyz,Q_DOWN_Y,seed=ready)
    if s:
        pp,qq=c.fk_world(s); tcp=pp+TCP*R.from_quat(qq).as_matrix()[:,2]
        print(name,np.round(s,3),'-> tcp',tcp.round(4))
" 2>&1 | grep -v Warning

# openrua op 18
timeout 300 python3 -c "
from ctl import *
import ctl
c=Ctl()
ready=[0,-0.785,0,-2.356,0,1.571,0.785]
req=GetPositionFK.Request(); req.header.frame_id=''; req.fk_link_names=['panda_link8','panda_link0']; req.robot_state.joint_state=c._seed(ready)
f=c.fk.call_async(req); rclpy.spin_until_future_complete(c.node,f,timeout_sec=60); r=f.result()
for ps in r.pose_stamped: print(ps.header.frame_id, ps.pose.position)
# Try IK with world coords directly (disable base offset)
ctl.BASE_IN_WORLD=np.zeros(3)
for name,xyz in [('butter_pre',(-0.117,0.039,0.56)),('butter_grasp',(-0.117,0.039,0.44)),('cc_pre',(-0.020,-0.237,0.56)),('cc_grasp',(-0.020,-0.237,0.44)),('basket',(-0.009,0.259,0.62))]:
    s=c.solve_ik(xyz,Q_DOWN_Y,seed=ready)
    if s:
        pp,qq=c.fk_world(s); tcp=pp+TCP*R.from_quat(qq).as_matrix()[:,2]
        print(name,np.round(s,3),'-> tcp',tcp.round(4), 'q',np.round(qq,3))
" 2>&1 | grep -v Warning

# openrua op 19
sed -i 's|^BASE_IN_WORLD = np.array(\[-0.51, 0.0, 0.42\]).*|BASE_IN_WORLD = np.zeros(3)  # verified: /compute_fk and /compute_ik poses are WORLD frame here|' ctl.py && sed -i 's|^import numpy as np, rclpy, yaml|import numpy as np, rclpy, yaml\nfrom scipy.spatial.transform import Rotation as R|' ctl.py && timeout 300 python3 -c "
from ctl import *
c=Ctl()
ready=[0,-0.785,0,-2.356,0,1.571,0.785]
print('current tcp world', c.tcp_world().round(3))
for name,xyz in [('butter_pre',(-0.117,0.039,0.56)),('butter_grasp',(-0.117,0.039,0.44)),('cc_pre',(-0.020,-0.237,0.56)),('cc_grasp',(-0.020,-0.237,0.44)),('basket',(-0.009,0.259,0.62))]:
    s=c.solve_ik(xyz,Q_DOWN_Y,seed=ready)
    if s:
        pp,qq=c.fk_world(s); tcp=pp+TCP*R.from_quat(qq).as_matrix()[:,2]
        print(name,np.round(s,3),'-> tcp',tcp.round(4), 'q',np.round(qq,3))
" 2>&1 | grep -v Warning

# openrua op 20
python3 -c "
import numpy as np
W=np.load('bird_W.npy')
def near(x,y,r=0.03):
    m=(abs(W[:,:,0]-x)<r)&(abs(W[:,:,1]-y)<r)
    z=W[m][:,2]; return np.percentile(z,[5,50,95]).round(4)
print('table near butter (x-0.117,y 0.12):',near(-0.117,0.12))
print('table near cc (x-0.02,y-0.31):',near(-0.02,-0.31))
print('butter top:',near(-0.117,0.039,0.012))
print('cc top:',near(-0.02,-0.237,0.012))
print('basket floor?', near(-0.009,0.259,0.03))
"

# openrua op 21
timeout 600 python3 -u -c "
from ctl import *
c=Ctl()
print('fingers',c.fingers())
print('-> butter pre-grasp'); c.goto((-0.117,0.039,0.58),Q_DOWN_Y,4.0)
" 2>&1 | grep -v Warning && cd /workspace && timeout 60 python3 tools/perception/cam_snap.py agentview && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 22
timeout 600 python3 -u -c "
from ctl import *
c=Ctl()
print('-> descend'); c.goto((-0.117,0.039,0.437),Q_DOWN_Y,2.5)
print('-> close'); fg=c.gripper(0.0)
print('-> lift'); c.goto((-0.117,0.039,0.65),Q_DOWN_Y,2.5)
print('fingers after lift',c.fingers())
" 2>&1 | grep -v Warning && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 23
cd /workspace/work && timeout 600 python3 -u -c "
from ctl import *
c=Ctl()
print('-> descend'); c.goto((-0.117,0.039,0.437),Q_DOWN_Y,2.5)
print('-> close'); fg=c.gripper(0.0)
print('-> lift'); c.goto((-0.117,0.039,0.65),Q_DOWN_Y,2.5)
print('fingers after lift',c.fingers())
" 2>&1 | grep -v Warning && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 24
timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 25
cd /workspace/work && timeout 600 python3 -u -c "
from ctl import *
c=Ctl()
print('-> above basket'); c.goto((-0.009,0.259,0.72),Q_DOWN_Y,3.5)
print('fingers',c.fingers())
print('-> lower'); c.goto((-0.009,0.259,0.62),Q_DOWN_Y,2.0)
print('-> open'); c.gripper(0.04)
print('-> retreat'); c.goto((-0.009,0.259,0.75),Q_DOWN_Y,2.0)
" 2>&1 | grep -v Warning && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 26
timeout 60 python3 tools/perception/cam_snap.py agentview && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 27
cd /workspace/work && timeout 600 python3 -u -c "
from ctl import *
c=Ctl()
print('-> cc pre-grasp'); c.goto((-0.020,-0.237,0.60),Q_DOWN_Y,4.0)
" 2>&1 | grep -v Warning && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 28
timeout 60 python3 /workspace/tools/perception/cam_snap.py robot0_eye_in_hand /workspace/robot0_eye_in_hand.png && timeout 60 python3 /workspace/tools/perception/cam_snap.py agentview /workspace/agentview.png

# openrua op 29
cd /workspace/work && timeout 600 python3 -u -c "
from ctl import *
c=Ctl()
print('-> descend'); c.goto((-0.020,-0.237,0.437),Q_DOWN_Y,2.5)
print('-> close'); c.gripper(0.0)
print('-> lift'); c.goto((-0.020,-0.237,0.65),Q_DOWN_Y,2.5)
print('fingers after lift',c.fingers())
" 2>&1 | grep -v Warning && timeout 60 python3 /workspace/tools/perception/cam_snap.py agentview /workspace/agentview.png

# openrua op 30
timeout 600 python3 -u -c "
from ctl import *
c=Ctl()
print('-> above basket'); c.goto((0.01,0.259,0.72),Q_DOWN_Y,3.5)
print('fingers',c.fingers())
print('-> lower'); c.goto((0.01,0.259,0.62),Q_DOWN_Y,2.0)
print('-> open'); c.gripper(0.04)
print('-> retreat'); c.goto((0.01,0.259,0.78),Q_DOWN_Y,2.0)
" 2>&1 | grep -v Warning && timeout 60 python3 /workspace/tools/perception/cam_snap.py robot0_eye_in_hand /workspace/robot0_eye_in_hand.png

# openrua op 31
timeout 600 python3 -u -c "
from ctl import *
c=Ctl()
print('-> retract'); c.move([0,-0.9,0,-2.5,0,1.6,0.785],3.0)
" 2>&1 | grep -v Warning && timeout 120 python3 -c "
from cloud import cloud
import numpy as np, cv2
color,depth,W,K,T=cloud('birdview')
cv2.imwrite('/workspace/birdview_after.png',color)
z=W[:,:,2]
def near(x,y,r):
    m=(abs(W[:,:,0]-x)<r)&(abs(W[:,:,1]-y)<r); return np.percentile(W[m][:,2],[50,95]).round(4)
print('old butter spot z:',near(-0.117,0.039,0.012),'(table=0.425)')
print('old cream cheese spot z:',near(-0.020,-0.237,0.012),'(table=0.425)')
# basket interior
m=(abs(W[:,:,0]+0.009)<0.05)&(abs(W[:,:,1]-0.259)<0.055)
zi=W[m][:,2]; print('basket interior z pct 5/50/95:',np.percentile(zi,[5,50,95]).round(4))
inside=m&(z>0.455)&(z<0.56)
pts=W[inside]; print('objects in basket: n=',inside.sum(),'x',pts[:,0].min().round(3),pts[:,0].max().round(3),'y',pts[:,1].min().round(3),pts[:,1].max().round(3),'zmax',pts[:,2].max().round(3))
"
python3 -c "
import cv2; im=cv2.imread('/workspace/birdview_after.png'); cv2.imwrite('/workspace/bird_after_crop.png', cv2.resize(im[200:330,230:420],None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))"
timeout 60 python3 /workspace/tools/perception/cam_snap.py agentview /workspace/agentview.png
