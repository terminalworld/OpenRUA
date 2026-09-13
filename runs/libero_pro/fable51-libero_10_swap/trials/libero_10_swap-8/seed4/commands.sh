#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
cat /workspace/tools/action/fjt_send.py; echo ----; cat /workspace/tools/action/gripper_cmd.py; echo ----; cat /workspace/tools/action/ik_move.py

# openrua op 3
timeout 30 ros2 topic list; echo ---; timeout 20 ros2 action list; echo ---; timeout 20 ros2 node list

# openrua op 4
for c in agentview birdview frontview sideview robot0_eye_in_hand robot0_robotview; do timeout 60 python3 tools/perception/cam_snap.py $c; done; timeout 30 ros2 topic echo /joint_states --once

# openrua op 5
timeout 30 ros2 run tf2_ros tf2_echo world birdview_optical_frame 2>&1 | head -12; timeout 30 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -12; timeout 20 ros2 topic echo /birdview/color/camera_info --once | grep -A12 '^k:'

# openrua op 6
cat > scene.py <<'EOF'
import numpy as np, struct, rclpy, sys
from sensor_msgs.msg import Image, CameraInfo
from tf2_ros import Buffer, TransformListener
import cv2

def grab(node, topic, T, timeout=20):
    got={}
    s=node.create_subscription(T, topic, lambda m: got.setdefault('m',m),1)
    import time; t0=time.time()
    while 'm' not in got and time.time()-t0<timeout: rclpy.spin_once(node,timeout_sec=0.2)
    node.destroy_subscription(s); return got['m']

def quat_R(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

def cloud(cam):
    rclpy.init(); node=rclpy.create_node('scene'); buf=Buffer(); TransformListener(buf,node)
    d=grab(node,f'/{cam}/depth/image_raw',Image); info=grab(node,f'/{cam}/color/camera_info',CameraInfo)
    col=grab(node,f'/{cam}/color/image_raw',Image)
    import time; t0=time.time()
    while not buf.can_transform('world',f'{cam}_optical_frame',rclpy.time.Time()) and time.time()-t0<10: rclpy.spin_once(node,timeout_sec=0.2)
    t=buf.lookup_transform('world',f'{cam}_optical_frame',rclpy.time.Time())
    depth=np.frombuffer(d.data,dtype=np.float32).reshape(d.height,d.width)
    fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
    v,u=np.mgrid[0:d.height,0:d.width]
    X=(u-cx)*depth/fx; Y=(v-cy)*depth/fy
    P=np.stack([X,Y,depth],-1)
    q=t.transform.rotation; R=quat_R(q.x,q.y,q.z,q.w); tr=np.array([t.transform.translation.x,t.transform.translation.y,t.transform.translation.z])
    W=P@R.T+tr
    rgb=np.frombuffer(col.data,dtype=np.uint8).reshape(col.height,col.width,-1)
    rclpy.shutdown()
    return W,depth,rgb

if __name__=='__main__':
    cam=sys.argv[1] if len(sys.argv)>1 else 'birdview'
    W,depth,rgb=cloud(cam)
    np.save(f'{cam}_world.npy',W)
    z=W[...,2]
    print('z range',np.nanmin(z),np.nanmax(z))
    # table height: mode of z in the middle region
    hist,edges=np.histogram(z[np.isfinite(z)],bins=200)
    i=np.argmax(hist); print('table z ~',edges[i],edges[i+1])
EOF
timeout 60 python3 scene.py birdview

# openrua op 7
cat > analyze.py <<'EOF'
import numpy as np, cv2
W=np.load('birdview_world.npy'); z=W[...,2]
# table region: find z histogram between 0.5 and 1.2
zz=z[(z>0.5)&(z<1.2)]
h,e=np.histogram(zz,bins=140); i=np.argmax(h); table=(e[i]+e[i+1])/2; print('table z',table)
# objects above table
mask=(z>table+0.01)&(z<table+0.4)
mask=mask.astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
for k in range(1,n):
    if stats[k,4]<30: continue
    m=lab==k
    pts=W[m]
    print(f'comp {k}: px={stats[k,4]} centroid px=({cent[k][0]:.0f},{cent[k][1]:.0f}) world x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] ztop={pts[:,2].max():.3f} zmed={np.median(pts[:,2]):.3f}')
EOF
python3 analyze.py

# openrua op 8
timeout 60 python3 scene.py frontview; timeout 60 python3 scene.py sideview; cat > profile.py <<'EOF'
import numpy as np
for cam in ['frontview','sideview','birdview']:
    W=np.load(f'{cam}_world.npy')
    for name,(x0,x1,y0,y1) in {'pot1':(-0.30,-0.10,-0.32,-0.10),'pot2':(-0.14,0.04,0.13,0.35),'stove':(0.09,0.33,-0.09,0.15),'knob':(0.0,0.11,-0.03,0.09)}.items():
        m=(W[...,0]>x0)&(W[...,0]<x1)&(W[...,1]>y0)&(W[...,1]<y1)&(W[...,2]>0.905)&(W[...,2]<1.2)
        P=W[m]
        if len(P)<10: continue
        print(cam,name,len(P))
        for zl in np.arange(0.91,1.07,0.01):
            s=P[(P[:,2]>=zl)&(P[:,2]<zl+0.01)]
            if len(s): print(f'  z={zl:.2f} n={len(s):4d} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]')
EOF
python3 profile.py

# openrua op 9
python3 -c "
import cv2
im=cv2.imread('frontview.png'); c=im[270:420,170:290]; cv2.imwrite('crop_pot1.png',cv2.resize(c,None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST))
im=cv2.imread('sideview.png'); c=im[270:400,250:340]; cv2.imwrite('crop_pot1s.png',cv2.resize(c,None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST))
"

# openrua op 10
cat > profile2.py <<'EOF'
import numpy as np
W=np.load('frontview_world.npy')
for name,(x0,x1,y0,y1) in {'pot1':(-0.30,-0.10,-0.32,-0.10),'pot2':(-0.14,0.04,0.13,0.35)}.items():
    m=(W[...,0]>x0)&(W[...,0]<x1)&(W[...,1]>y0)&(W[...,1]<y1)&(W[...,2]>0.905)&(W[...,2]<1.2)
    P=W[m]
    print(name)
    for zl in np.arange(0.905,1.06,0.005):
        s=P[(P[:,2]>=zl)&(P[:,2]<zl+0.005)]
        if len(s):
            ys=np.sort(s[:,1])
            # cluster gaps
            print(f'  z={zl:.3f} n={len(s):3d} y[{ys[0]:.3f},{ys[-1]:.3f}] w={ys[-1]-ys[0]:.3f} x[{s[:,0].min():.3f},{s[:,0].max():.3f}]')
EOF
python3 profile2.py

# openrua op 11
python3 -c "
import cv2,numpy as np
im=cv2.imread('birdview.png'); c=im[300:400,280:380]; cv2.imwrite('crop_stove.png',cv2.resize(c,None,fx=5,fy=5,interpolation=cv2.INTER_NEAREST))
W=np.load('birdview_world.npy')
hsv=cv2.cvtColor(im,cv2.COLOR_BGR2HSV)
red=((hsv[...,0]<10)|(hsv[...,0]>170))&(hsv[...,1]>120)&(hsv[...,2]>80)
ys,xs=np.where(red); print('red px', len(xs), 'u range',xs.min(),xs.max(),'v range',ys.min(),ys.max())
P=W[red]; print('red world x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]'%(P[:,0].min(),P[:,0].max(),P[:,1].min(),P[:,1].max(),P[:,2].min(),P[:,2].max()))
print('center', P[:,0].mean(), P[:,1].mean())
# stove plate (gray) region z~0.92
m=(W[...,2]>0.915)&(W[...,2]<0.925)&(W[...,0]>0.1)
P=W[m]; print('plate x[%.3f,%.3f] y[%.3f,%.3f]'%(P[:,0].min(),P[:,0].max(),P[:,1].min(),P[:,1].max()))
"


# openrua op 12
timeout 20 ros2 service list | grep -iE "ik|fk|state|scene"; timeout 20 ros2 interface show moveit_msgs/srv/GetPositionFK | head -30

# openrua op 13
mkdir -p "$(dirname /workspace/robot.py)"
cat > /workspace/robot.py <<'OPENRUA_EOF'
"""Reusable helpers: joint state, FK, IK, trajectory, gripper, TF.
All poses are WORLD frame unless noted; the planner works in panda_link0
which sits at BASE = (-0.66, 0, 0.912) with identity rotation."""
import time
import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from moveit_msgs.msg import RobotState
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open('/workspace/machine.yaml'))
FJT = next(a for a in M['actuators'] if a['kind'] == 'joint_trajectory')
GRIP = next(a for a in M['actuators'] if a['kind'] == 'gripper')
ARM = FJT['joints']
BASE = np.array([-0.66, 0.0, 0.912])
TCP = M['hand']['tcp_offset_m']


def quat_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def R_quat(R):
    # from rotation matrix to xyzw
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1) * 2
        return np.array([(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s,
                         (R[1, 0] - R[0, 1]) / s, 0.25 * s])
    i = np.argmax(np.diag(R))
    if i == 0:
        s = np.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return np.array([0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s])
    if i == 1:
        s = np.sqrt(1 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        return np.array([(R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s])
    s = np.sqrt(1 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
    return np.array([(R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s, (R[1, 0] - R[0, 1]) / s])


def rot_x(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_y(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rot_z(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def grasp_R(yaw=0.0, pitch=0.0):
    """Hand rotation: z down; fingers open along world y rotated by yaw
    about z; then pitched by `pitch` about the world y axis (positive =
    hand leans toward -x / the robot)."""
    R = rot_x(np.pi)          # z down, hand x = world x, hand y = -world y
    R = rot_z(yaw) @ R
    R = rot_y(pitch) @ R
    return R


class Robot:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node('robot_helper')
        self._js = {}
        self.node.create_subscription(JointState, '/joint_states',
                                      lambda m: self._js.__setitem__('m', m), 1)
        self.ik_cli = self.node.create_client(GetPositionIK, '/compute_ik')
        self.fk_cli = self.node.create_client(GetPositionFK, '/compute_fk')
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT['port'])
        self.grip = ActionClient(self.node, GripperCommand, GRIP['port'])
        self.ik_cli.wait_for_service(10); self.fk_cli.wait_for_service(10)
        self.fjt.wait_for_server(10); self.grip.wait_for_server(10)

    def spin(self, t=0.1):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self, fresh=True):
        if fresh:
            self._js.pop('m', None)
        t0 = time.time()
        while 'm' not in self._js and time.time() - t0 < 10:
            self.spin(0.2)
        m = self._js['m']
        return dict(zip(m.name, m.position))

    def arm_q(self):
        j = self.joints()
        return np.array([j[n] for n in ARM])

    def finger_gap(self):
        j = self.joints()
        return j['panda_finger_joint1'] - j['panda_finger_joint2']

    def _seed(self, q):
        js = JointState(); js.name = list(ARM); js.position = [float(v) for v in q]
        rs = RobotState(); rs.joint_state = js
        return rs

    def fk(self, q, link='panda_hand'):
        """World-frame (pos, R) of link for arm config q."""
        req = GetPositionFK.Request()
        req.fk_link_names = [link]
        req.robot_state = self._seed(q)
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f'FK failed: {res and res.error_code.val}')
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE
        q_ = [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
        return pos, quat_R(q_)

    def ik(self, pos, R, seed=None, timeout=5.0, attempts=1):
        """IK for panda_hand at world pos with rotation R. Returns q or None."""
        if seed is None:
            seed = self.arm_q()
        req = GetPositionIK.Request()
        req.ik_request.group_name = M['planning']['group']
        req.ik_request.pose_stamped.header.frame_id = ''
        req.ik_request.ik_link_name = 'panda_hand'
        p = req.ik_request.pose_stamped.pose
        pb = np.asarray(pos, float) - BASE
        p.position.x, p.position.y, p.position.z = map(float, pb)
        qx, qy, qz, qw = R_quat(R)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, (qx, qy, qz, qw))
        req.ik_request.robot_state = self._seed(seed)
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=int(timeout), nanosec=int((timeout % 1) * 1e9))
        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return np.array([sol[j] for j in ARM])

    def ik_tcp(self, tcp_pos, R, **kw):
        """IK with the fingertip centre (TCP) at tcp_pos."""
        hand = np.asarray(tcp_pos, float) - TCP * R[:, 2]
        return self.ik(hand, R, **kw)

    def move(self, q_list, durations):
        """Send a multi-point trajectory; returns error_code."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        for q, t in zip(q_list, durations):
            pt = JointTrajectoryPoint(positions=[float(v) for v in q])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        fut = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        h = fut.result()
        rf = h.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=600)
        return rf.result().result.error_code

    def move_to(self, q, t=3.0):
        code = self.move([q], [t])
        err = np.abs(self.arm_q() - q).max()
        return code, err

    def gripper(self, width):
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = float(GRIP['max_effort'])
        fut = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result
        return r.reached_goal, r.stalled, r.position
OPENRUA_EOF

# openrua op 14
cat > t1.py <<'EOF'
from robot import *
r=Robot()
q=r.arm_q(); print('q',q.round(3))
for link in ['panda_hand','panda_link8','panda_hand_tcp','panda_leftfinger']:
    try:
        p,R=r.fk(q,link); print(link,'pos',p.round(4),'z-axis',R[:,2].round(3),'y-axis',R[:,1].round(3))
    except Exception as e: print(link,e)
# IK check: does IK of current hand pose return current q?
p,R=r.fk(q,'panda_hand')
s=r.ik(p,R); print('ik back',None if s is None else (s-q).round(3))
EOF
timeout 120 python3 t1.py

# openrua op 15
timeout 20 ros2 run tf2_ros tf2_echo panda_link0 panda_hand 2>&1 | grep -m1 Translation; timeout 20 ros2 run tf2_ros tf2_echo world panda_hand 2>&1 | grep -m1 -E "Translation|Invalid|unconnected"; timeout 20 ros2 topic echo /robot_description --once --qos-durability transient_local 2>/dev/null | grep -oE 'name="[a-z_0-9]*"' | head -5

# openrua op 16
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/robot.py", "old_string": "BASE = np.array([-0.66, 0.0, 0.912])", "new_string": "# Verified: FK/IK poses on this machine are already WORLD-frame (TF\\n# panda_link0->panda_hand == FK - (-0.66,0,0.912)); no offset to apply.\\nBASE = np.zeros(3)\\nLINK0 = np.array([-0.66, 0.0, 0.912])", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 17
cat > t2.py <<'EOF'
from robot import *
r=Robot()
q0=r.arm_q()
# reachability of straight-down grasp (fingers along y) at TCP z=0.94 for various x, y=0.03
for pitch in [0.0, 0.3, 0.5]:
  for x in [0.10,0.14,0.17,0.20,0.23]:
    R=grasp_R(yaw=0, pitch=pitch)
    s=r.ik_tcp([x,0.03,0.94],R,seed=q0,timeout=2)
    print(f'pitch={pitch} x={x}: {"OK "+str(s.round(2)) if s is not None else "fail"}')
# reachability at pot positions
for name,(x,y) in {'pot1':(-0.193,-0.200),'pot2':(-0.040,0.253)}.items():
    for yaw in [0, np.pi/2]:
        s=r.ik_tcp([x,y,0.94],grasp_R(yaw=yaw),seed=q0,timeout=2)
        print(name,'yaw',yaw, None if s is None else s.round(2))
EOF
timeout 300 python3 t2.py

# openrua op 18
cat > t3.py <<'EOF'
from robot import *
r=Robot()
q0=r.arm_q()
for pitch in [-0.25,-0.4,-0.55]:
  for x in [0.14,0.17,0.20,0.23]:
    R=grasp_R(yaw=0, pitch=pitch)
    s=r.ik_tcp([x,0.03,0.94],R,seed=q0,timeout=2)
    print(f'pitch={pitch} x={x}: {"OK "+str(s.round(2)) if s is not None else "fail"}')
EOF
timeout 300 python3 t3.py

# openrua op 19
cat > t4.py <<'EOF'
from robot import *
r=Robot()
print('open', r.gripper(0.04))
j=r.joints(); print('fingers', j['panda_finger_joint1'], j['panda_finger_joint2'])
q=r.arm_q()
for link in ['panda_leftfinger','panda_rightfinger','panda_hand']:
    p,R=r.fk(q,link); print(link,p.round(4))
EOF
timeout 120 python3 t4.py

# openrua op 20
timeout 30 python3 - <<'EOF'
import rclpy
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy
rclpy.init(); n=rclpy.create_node('rd')
got=[]
qos=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
n.create_subscription(String,'/robot_description',got.append,qos)
import time; t0=time.time()
while not got and time.time()-t0<15: rclpy.spin_once(n,timeout_sec=0.2)
open('robot.urdf','w').write(got[0].data); print(len(got[0].data))
EOF
grep -n -A12 'link name="panda_hand"' robot.urdf | head -60; grep -n -B2 -A14 'link name="panda_leftfinger"' robot.urdf | head -40

# openrua op 21
python3 - <<'EOF'
import numpy as np, struct
def stl_bbox(fn):
    d=open(fn,'rb').read()
    if d[:5]==b'solid' and b'facet' in d[:300]:
        import re
        v=np.array([list(map(float,m)) for m in re.findall(rb'vertex\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)',d)])
    else:
        n=struct.unpack('<I',d[80:84])[0]
        arr=np.frombuffer(d[84:84+n*50],dtype=np.dtype([('n','<3f'),('v','<9f'),('a','<H')]))
        v=arr['v'].reshape(-1,3)
    return v.min(0),v.max(0)
for f in ['hand','finger']:
    print(f, stl_bbox(f'/opt/ros/jazzy/share/moveit_resources_panda_description/meshes/collision/{f}.stl'))
EOF

# openrua op 22
find / -iname "*hand*.stl" -o -iname "*finger*.stl" -o -iname "hand*.dae" -o -iname "hand*.obj" 2>/dev/null | grep -v proc | head; find / -type d -iname "*robosuite*" -o -type d -iname "*robocasa*" 2>/dev/null | head

# openrua op 23
cat > handle.py <<'EOF'
import numpy as np
W=np.load('frontview_world.npy')
# pot1 handle side: y<-0.20 ; list clusters in y per z slice
m=(W[...,0]>-0.30)&(W[...,0]<-0.10)&(W[...,1]>-0.32)&(W[...,1]<-0.10)&(W[...,2]>0.94)&(W[...,2]<1.06)
P=W[m]
for zl in np.arange(0.94,1.06,0.005):
    s=P[(P[:,2]>=zl)&(P[:,2]<zl+0.005)]
    if len(s)==0: continue
    ys=np.sort(s[:,1]); gaps=np.where(np.diff(ys)>0.006)[0]
    segs=[]; start=0
    for g in gaps: segs.append((ys[start],ys[g])); start=g+1
    segs.append((ys[start],ys[-1]))
    print(f'z={zl:.3f} '+' | '.join(f'[{a:.3f},{b:.3f}]' for a,b in segs))
EOF
python3 handle.py

# openrua op 24
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih.png && timeout 60 python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw eih_d.png && python3 -c "
import numpy as np, cv2
d=np.load('eih_d.npy'); print(d.shape, np.nanmin(d), np.nanmax(d))
# bottom rows: fingers close to camera
for v in [360,380,400,420,440,460]:
    row=d[v]; print(v, ' '.join(f'{x:.2f}' for x in row[::20]))
"; timeout 20 ros2 topic echo /robot0_eye_in_hand/color/camera_info --once | grep -A3 '^k:'

# openrua op 25
python3 -c "
import numpy as np
d=np.load('eih_d.npy'); fx=312.774
# rows 380-410: find inner edges of the two fingers (depth ~0.08-0.10 vs background 0.38)
for v in range(376,416,4):
    row=d[v]; m=row<0.2
    idx=np.where(m)[0]
    # left finger: indices <320, right: >320
    L=idx[idx<320]; R=idx[idx>320]
    if len(L) and len(R):
        zl=row[L[-1]]; zr=row[R[0]]
        gap=(R[0]-L[-1])*((zl+zr)/2)/fx
        print(v,'left inner px',L[-1],'z',round(zl,3),'right inner px',R[0],'z',round(zr,3),'gap m',round(gap,4), 'finger widths px',len(L),len(R))
"

# openrua op 26
cat > t5.py <<'EOF'
from robot import *
r=Robot()
q0=r.arm_q()
def R_tilt(tilt, yaw=0.0):
    # hand z-axis pointing down-forward (+x) by `tilt` below horizontal; fingers along world y
    # start from straight-down (rot_x(pi)): z=-Z, x=X, y=-Y. Rotate about world y by +(90-tilt) so z tilts toward +x?
    # rot_y(a) maps (0,0,-1) -> (-sin a, 0, -cos a). want (cos t, 0, -sin t) => -sin a = cos t => a = -(90-t)
    a = -(np.pi/2 - tilt)
    return rot_z(yaw) @ rot_y(a) @ rot_x(np.pi)
for tilt in [np.radians(30), np.radians(45), np.radians(60)]:
    R=R_tilt(tilt); print('tilt',np.degrees(tilt),'z-axis',R[:,2].round(3),'y-axis',R[:,1].round(3))
    for (x,y) in [(0.17,-0.02),(0.20,0.08),(0.23,0.08),(0.25,0.08)]:
        s=r.ik_tcp([x,y,0.97],R,seed=q0,timeout=2)
        print(f'  ({x},{y}): {"OK "+str(s.round(2)) if s is not None else "fail"}')
EOF
timeout 300 python3 t5.py

# openrua op 27
python3 - <<'EOF'
import numpy as np
for cam in ['sideview','frontview','birdview']:
    W=np.load(f'{cam}_world.npy')
    m=(W[...,2]>1.10)&(W[...,2]<1.30)&(np.abs(W[...,1])<0.15)&(W[...,0]>-0.35)&(W[...,0]<-0.05)
    P=W[m]
    print(cam)
    for zl in np.arange(1.10,1.30,0.01):
        s=P[(P[:,2]>=zl)&(P[:,2]<zl+0.01)]
        if len(s): print(f'  z={zl:.2f} n={len(s):4d} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]')
EOF

# openrua op 28
mkdir -p "$(dirname /workspace/pp.py)"
cat > /workspace/pp.py <<'OPENRUA_EOF'
"""Pick-and-place primitives for the moka pots (tilted side grasp)."""
import sys, time
import numpy as np
from robot import *

TILT = np.radians(20)          # hand z-axis below horizontal, pointing +x
GRASP_Z = 0.938                # TCP height for the lower-chamber grasp
TABLE_Z = 0.90


def R_tilt(tilt=TILT):
    a = -(np.pi / 2 - tilt)
    return rot_y(a) @ rot_x(np.pi)


R_G = R_tilt()
ZH = R_G[:, 2]                  # hand approach axis in world


def hand_from_tcp(tcp):
    return np.asarray(tcp) - TCP * ZH


class PP:
    def __init__(self):
        self.r = Robot()

    def ik_path(self, tcps, seed=None, max_jump=0.8):
        """IK for a list of TCP positions (orientation R_G); seeds chain."""
        q = self.r.arm_q() if seed is None else seed
        out = []
        for p in tcps:
            s = self.r.ik_tcp(p, R_G, seed=q, timeout=3)
            if s is None:
                raise RuntimeError(f'IK failed at TCP {np.round(p, 3)}')
            jump = np.abs(s - q).max()
            if jump > max_jump and out:
                # try again seeded from previous once more
                s2 = self.r.ik_tcp(p, R_G, seed=q, timeout=3)
                if s2 is not None and np.abs(s2 - q).max() < jump:
                    s = s2
                    jump = np.abs(s - q).max()
                if jump > max_jump:
                    print(f'  warn: joint jump {jump:.2f} at {np.round(p,3)}')
            out.append(s)
            q = s
        return out

    def exec(self, qs, total_t, label=''):
        n = len(qs)
        ts = [total_t * (i + 1) / n for i in range(n)]
        code = self.r.move(qs, ts)
        err = np.abs(self.r.arm_q() - qs[-1]).max()
        print(f'  [{label}] code={code} final joint err={err:.4f}')
        if code != 0 or err > 0.05:
            print('  retrying last point')
            code = self.r.move([qs[-1]], [2.0])
            err = np.abs(self.r.arm_q() - qs[-1]).max()
            print(f'  [{label}] retry code={code} err={err:.4f}')
        return code, err

    def line(self, p0, p1, n=None):
        p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
        d = np.linalg.norm(p1 - p0)
        if n is None:
            n = max(2, int(np.ceil(d / 0.03)))
        return [p0 + (p1 - p0) * (i + 1) / n for i in range(n)]

    def tcp_now(self):
        p, R = self.r.fk(self.r.arm_q(), 'panda_hand')
        return p + TCP * R[:, 2], R

    def move_tcp_line(self, p1, t=None, label='line'):
        p0, _ = self.tcp_now()
        pts = self.line(p0, p1)
        qs = self.ik_path(pts)
        if t is None:
            t = max(1.5, np.linalg.norm(np.asarray(p1) - p0) / 0.08)
        return self.exec(qs, t, label)

    def goto_tcp(self, p1, t=3.0, label='goto'):
        qs = self.ik_path([p1])
        return self.exec(qs, t, label)

    def pick(self, center):
        cx, cy = center
        g = np.array([cx, cy, GRASP_Z])
        pre = g - 0.10 * ZH                      # back along approach axis
        high = pre + np.array([0, 0, 0.12])
        print('open gripper', self.r.gripper(0.04))
        print('-> high pre-grasp', np.round(high, 3))
        self.goto_tcp(high, 4.0, 'high')
        print('-> pre-grasp', np.round(pre, 3))
        self.move_tcp_line(pre, 2.5, 'pre')
        print('-> grasp', np.round(g, 3))
        self.move_tcp_line(g, 3.0, 'approach')
        p, _ = self.tcp_now(); print('  tcp at', np.round(p, 4))
        res = self.r.gripper(0.0)
        gap = self.r.finger_gap()
        print(f'close gripper {res} gap={gap:.4f}')
        return gap

    def lift(self, dz=0.12):
        p, _ = self.tcp_now()
        self.move_tcp_line(p + np.array([0, 0, dz]), 3.0, 'lift')

    def place(self, target, z_bottom=0.935, carry_z=1.06):
        tx, ty = target
        rel_z = z_bottom + (GRASP_Z - TABLE_Z)     # TCP z when pot bottom at z_bottom
        above = np.array([tx, ty, carry_z])
        down = np.array([tx, ty, rel_z])
        p0, _ = self.tcp_now()
        print('-> carry to above target', np.round(above, 3))
        pts = self.line(p0, above, n=6)
        qs = self.ik_path(pts)
        self.exec(qs, 6.0, 'carry')
        print('-> lower', np.round(down, 3))
        self.move_tcp_line(down, 3.0, 'lower')
        print('open gripper', self.r.gripper(0.04))
        gap = self.r.finger_gap(); print(f'  gap={gap:.4f}')
        back = down - 0.10 * ZH
        print('-> retreat', np.round(back, 3))
        self.move_tcp_line(back, 2.5, 'retreat')
        self.move_tcp_line(back + np.array([0, 0, 0.10]), 2.0, 'up')
OPENRUA_EOF

# openrua op 29
cat > t6.py <<'EOF'
from pp import *
pp=PP(); r=pp.r
print('ZH',ZH.round(3),'y-axis',R_G[:,1].round(3))
q0=r.arm_q()
tests={'B_high':(-0.042-0.094,0.254,1.092),'B_pre':(-0.042-0.094,0.254,0.972),'B_grasp':(-0.042,0.254,0.938),
'A_pre':(-0.195-0.094,-0.2005,0.972),'A_grasp':(-0.195,-0.2005,0.938),
'B_place_above':(0.23,0.08,1.06),'B_place':(0.23,0.08,0.973),'B_retreat':(0.23-0.094,0.08,1.007),
'A_place_above':(0.17,-0.02,1.06),'A_place':(0.17,-0.02,0.973)}
for k,p in tests.items():
    s=r.ik_tcp(p,R_G,seed=q0,timeout=3)
    print(k, None if s is None else s.round(2))
    if s is not None:
        hp,hR=r.fk(s,'panda_hand'); print('   fk tcp',(hp+TCP*hR[:,2]).round(4),'z-axis',hR[:,2].round(3))
EOF
timeout 300 python3 t6.py

# openrua op 30
cat > t7.py <<'EOF'
from pp import *
pp=PP(); r=pp.r
def seed_for(p):
    j1=np.arctan2(p[1], p[0]+0.66)
    return np.array([j1, 0.6, 0.0, -1.9, 0.0, 2.8, 0.785+j1])
tests={'B_high':(-0.136,0.254,1.092),'B_pre':(-0.136,0.254,0.972),'B_grasp':(-0.042,0.254,0.938),
'B_lift':(-0.042,0.254,1.06),'B_place_above':(0.23,0.08,1.06),'B_place':(0.23,0.08,0.973),'B_retreat':(0.136,0.08,1.007),'B_retreat_up':(0.136,0.08,1.107),
'A_high':(-0.289,-0.2005,1.092),'A_pre':(-0.289,-0.2005,0.972),'A_grasp':(-0.195,-0.2005,0.938),'A_lift':(-0.195,-0.2005,1.06),
'A_place_above':(0.17,-0.02,1.06),'A_place':(0.17,-0.02,0.973),'A_retreat':(0.076,-0.02,1.007)}
prev=None
for k,p in tests.items():
    p=np.array(p)
    best=None
    for sd in ([prev] if prev is not None else [])+[seed_for(p)]:
        s=r.ik_tcp(p,R_G,seed=sd,timeout=3)
        if s is None: continue
        d=np.abs(s-(prev if prev is not None else seed_for(p))).max()
        if best is None or d<best[1]: best=(s,d)
    if best is None: print(k,'FAIL'); continue
    s,d=best; prev=s
    print(f'{k:15s} jump={d:.2f}', s.round(2))
EOF
timeout 300 python3 t7.py

# openrua op 31
cat > t8.py <<'EOF'
from pp import *
r=Robot()
for j6 in [1.571, 2.0, 2.4, 2.79, 3.2]:
    q=np.array([0,-0.785,0,-2.356,0,j6,0.785])
    p,R=r.fk(q,'panda_hand'); print('j6',j6,'hand',p.round(3),'z',R[:,2].round(2),'y',R[:,1].round(2))
for j6 in [0.35, 0.8]:
    q=np.array([0,-0.785,0,-2.356,0,j6,0.785])
    p,R=r.fk(q,'panda_hand'); print('j6',j6,'hand',p.round(3),'z',R[:,2].round(2),'y',R[:,1].round(2))
# vary j2, j4 with j6 fixed at 2.79
for j2,j4 in [(0.0,-2.0),(0.5,-1.8),(0.8,-1.5),(1.0,-1.2),(0.3,-2.4)]:
    q=np.array([0,j2,0,j4,0,2.79- ( (j2-(-0.785)) + (j4-(-2.356)) ) ,0.785])
    p,R=r.fk(q,'panda_hand'); print('j2',j2,'j4',j4,'j6',round(q[5],3),'hand',p.round(3),'z',R[:,2].round(2))
EOF
timeout 120 python3 t8.py

# openrua op 32
cat > t9.py <<'EOF'
from pp import *
pp=PP(); r=pp.r
LIM=np.array(FJT['limits_rad'])
tests={'B_high':(-0.136,0.254,1.092),'B_pre':(-0.136,0.254,0.972),'B_grasp':(-0.042,0.254,0.938),
'B_lift':(-0.042,0.254,1.06),'B_place_above':(0.23,0.08,1.06),'B_place':(0.23,0.08,0.973),'B_retreat':(0.136,0.08,1.007),'B_retreat_up':(0.136,0.08,1.107),
'A_high':(-0.289,-0.2005,1.092),'A_pre':(-0.289,-0.2005,0.972),'A_grasp':(-0.195,-0.2005,0.938),'A_lift':(-0.195,-0.2005,1.06),
'A_place_above':(0.17,-0.02,1.06),'A_place':(0.17,-0.02,0.973),'A_retreat':(0.076,-0.02,1.007)}
for seedname,base in {'wristflip':np.array([0.4,0.8,-0.7,-2.5,2.8,1.7,-1.9])}.items():
    print('=== seed',seedname)
    prev=None
    for k,p in tests.items():
        p=np.array(p)
        cands=[]
        for sd in ([prev] if prev is not None else [])+[base]:
            s=r.ik_tcp(p,R_G,seed=sd,timeout=3)
            if s is not None: cands.append(s)
        if not cands: print(f'{k:15s} FAIL'); continue
        ref=prev if prev is not None else base
        s=min(cands,key=lambda s:np.abs(s-ref).max()); d=np.abs(s-ref).max()
        margin=np.minimum(s-LIM[:,0],LIM[:,1]-s).min()
        prev=s
        print(f'{k:15s} jump={d:.2f} margin={margin:.2f}', s.round(2))
EOF
timeout 300 python3 t9.py

# openrua op 33
cat > t10.py <<'EOF'
from pp import *
import pp as P
pp_=PP(); r=pp_.r
LIM=np.array(FJT['limits_rad'])
tests={'B_high':(-0.136,0.254,1.092),'B_pre':(-0.136,0.254,0.972),'B_grasp':(-0.042,0.254,0.938),
'B_lift':(-0.042,0.254,1.06),'B_place_above':(0.23,0.08,1.06),'B_place':(0.23,0.08,0.973),'B_retreat':(0.136,0.08,1.007),'B_retreat_up':(0.136,0.08,1.107),
'A_high':(-0.289,-0.2005,1.092),'A_pre':(-0.289,-0.2005,0.972),'A_grasp':(-0.195,-0.2005,0.938),'A_lift':(-0.195,-0.2005,1.06),
'A_place_above':(0.17,-0.02,1.06),'A_place':(0.17,-0.02,0.973),'A_retreat':(0.076,-0.02,1.007)}
def run(R, seeds):
    prev=None; ok=0
    for k,p in tests.items():
        p=np.array(p); cands=[]
        for sd in ([prev] if prev is not None else [])+seeds:
            s=r.ik_tcp(p,R,seed=sd,timeout=2)
            if s is not None: cands.append(s)
        if not cands: print(f'{k:15s} FAIL'); continue
        ref=prev if prev is not None else seeds[0]
        s=min(cands,key=lambda s:np.abs(s-ref).max()); d=np.abs(s-ref).max()
        margin=np.minimum(s-LIM[:,0],LIM[:,1]-s).min(); prev=s; ok+=1
        print(f'{k:15s} jump={d:.2f} margin={margin:.2f}', s.round(2))
    return ok
R2=R_tilt(np.radians(20)) @ rot_z(np.pi)
print('R2 z',R2[:,2].round(2),'y',R2[:,1].round(2),'x',R2[:,0].round(2))
q0=r.arm_q()
seeds=[q0, np.array([0,0.5,0,-2.0,0,2.5,0.785]), np.array([0,0.5,0,-2.0,0,0.5,0.785]), np.array([0,0.5,0,-2.0,3.0,1.5,-2.3])]
print('=== roll180 tilt20'); run(R2, seeds)
EOF
timeout 600 python3 t10.py

# openrua op 34
cat > t11.py <<'EOF'
from pp import *
import time
pp_=PP(); r=pp_.r
q0=r.arm_q()
t=time.time(); s=r.ik_tcp([-0.042,0.254,0.938],R_G,seed=q0,timeout=0.5); print('call1',time.time()-t, s is not None)
t=time.time(); s=r.ik_tcp([0.5,0.5,0.5],R_G,seed=q0,timeout=0.5); print('fail call',time.time()-t, s is not None)
t=time.time(); s=r.ik_tcp([0.5,0.5,0.5],R_G,seed=q0,timeout=0.1); print('fail call 0.1',time.time()-t, s is not None)
EOF
timeout 120 python3 t11.py

# openrua op 35
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/robot.py", "old_string": "    def ik_tcp(self, tcp_pos, R, **kw):\\n        \\"\\"\\"IK with the fingertip centre (TCP) at tcp_pos.\\"\\"\\"\\n        hand = np.asarray(tcp_pos, float) - TCP * R[:, 2]\\n        return self.ik(hand, R, **kw)", "new_string": "    def ik_tcp(self, tcp_pos, R, **kw):\\n        \\"\\"\\"IK with the fingertip centre (TCP) at tcp_pos.\\"\\"\\"\\n        hand = np.asarray(tcp_pos, float) - TCP * R[:, 2]\\n        return self.ik(hand, R, **kw)\\n\\n    def ik_best(self, tcp_pos, R, prev=None, n_random=60, w_cont=1.0,\\n                min_margin=0.15, rng=np.random.default_rng(0)):\\n        \\"\\"\\"Multi-start IK: many seeds, keep the solution with good joint\\n        limit margin and (if prev given) small joint-space distance.\\"\\"\\"\\n        lim = np.array(FJT[\'limits_rad\'])\\n        seeds = []\\n        if prev is not None:\\n            seeds.append(np.asarray(prev))\\n        seeds.append(self.arm_q())\\n        for _ in range(n_random):\\n            seeds.append(rng.uniform(lim[:, 0] + 0.2, lim[:, 1] - 0.2))\\n        best, best_cost = None, None\\n        for sd in seeds:\\n            s = self.ik_tcp(tcp_pos, R, seed=sd, timeout=0.05)\\n            if s is None:\\n                continue\\n            margin = np.minimum(s - lim[:, 0], lim[:, 1] - s).min()\\n            cost = max(0.0, min_margin - margin) * 20.0\\n            if prev is not None:\\n                cost += w_cont * np.abs(s - prev).max()\\n            if best is None or cost < best_cost:\\n                best, best_cost = s, cost\\n        return best", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 36
cat > t12.py <<'EOF'
from pp import *
import time
pp_=PP(); r=pp_.r
LIM=np.array(FJT['limits_rad'])
ZH20=R_G[:,2]
def seq(cx,cy,tx,ty,zb):
    g=np.array([cx,cy,GRASP_Z]); pre=g-0.10*ZH20; high=pre+[0,0,0.12]
    lift=g+[0,0,0.12]; above=np.array([tx,ty,1.06]); down=np.array([tx,ty,zb+GRASP_Z-TABLE_Z]); back=down-0.10*ZH20; up=back+[0,0,0.10]
    return {'high':high,'pre':pre,'grasp':g,'lift':lift,'above':above,'down':down,'back':back,'up':up}
for name,R in [('R_G',R_G),('R2',R_G@rot_z(np.pi))]:
  for pot,(cx,cy,tx,ty,zb) in {'B':(-0.042,0.254,0.20,-0.02,0.935),'A':(-0.195,-0.2005,0.14,0.085,0.945)}.items():
    print(f'=== {name} pot {pot}')
    prev=None; t0=time.time()
    for k,p in seq(cx,cy,tx,ty,zb).items():
        s=r.ik_best(p,R,prev=prev)
        if s is None: print(f'  {k:6s} FAIL'); continue
        margin=np.minimum(s-LIM[:,0],LIM[:,1]-s).min(); d=0 if prev is None else np.abs(s-prev).max(); prev=s
        print(f'  {k:6s} jump={d:.2f} margin={margin:.2f}', s.round(2))
    print('  time',round(time.time()-t0,1))
EOF
timeout 600 python3 t12.py

# openrua op 37
cat > t13.py <<'EOF'
from pp import *
pp_=PP(); r=pp_.r
LIM=np.array(FJT['limits_rad'])
for pitch in [0.0,-0.2,-0.35,-0.5]:
    R=grasp_R(yaw=0,pitch=pitch)
    print('pitch',pitch,'z',R[:,2].round(2))
    for x in [0.14,0.16,0.18,0.20,0.21]:
        for y in [-0.015,0.085]:
            s=r.ik_best([x,y,1.09],R,n_random=40)
            if s is None: print(f'  ({x},{y}) FAIL'); continue
            margin=np.minimum(s-LIM[:,0],LIM[:,1]-s).min()
            print(f'  ({x},{y}) margin={margin:.2f}',s.round(2))
EOF
timeout 600 python3 t13.py

# openrua op 38
timeout 20 ros2 topic echo /tf_static --once 2>/dev/null | grep -E "frame_id" | sort | uniq; echo ---; timeout 15 ros2 topic echo /tf --once 2>/dev/null | grep -E "frame_id" | sort | uniq -c; echo ---; timeout 20 ros2 run tf2_ros tf2_echo panda_hand robot0_eye_in_hand_optical_frame 2>&1 | head -8

# openrua op 39
mkdir -p "$(dirname /workspace/knob.py)"
cat > /workspace/knob.py <<'OPENRUA_EOF'
"""Knob grasp pick-and-place: pinch the lid knob from above, pot hangs plumb.
Usage: python3 knob.py <pot: A|B> [dry]"""
import sys
import numpy as np
import rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from robot import *
from pp import PP

PITCH = -0.40                    # hand leans toward the robot (extends reach)
KNOB_Z = 1.056                   # TCP height for the pinch (knob spans ~1.045-1.06)
HOVER_Z = 1.25
CARRY_Z = 1.20
PLACE_Z = KNOB_Z + 0.040         # pot bottom ~0.94, 1 cm above the burner (0.93)

POTS = {  # knob centre from the birdview cloud, and target on the stove
    'A': dict(knob=(-0.1965, -0.201), target=(0.22, -0.01)),
    'B': dict(knob=(-0.0425, 0.255), target=(0.16, 0.085)),
}


def R_of(pitch=PITCH):
    return grasp_R(0.0, pitch)


class Knob(PP):
    def __init__(self):
        super().__init__()
        self.tfbuf = Buffer()
        TransformListener(self.tfbuf, self.r.node)

    # ---- IK along a path with a fixed orientation, chained seeds -------
    def path(self, tcps, R, seed):
        q = np.asarray(seed)
        out = []
        for p in tcps:
            s = self.r.ik_tcp(p, R, seed=q, timeout=0.5)
            if s is None or np.abs(s - q).max() > 0.6:
                s2 = self.r.ik_best(p, R, prev=q, n_random=40)
                if s2 is not None and (s is None or np.abs(s2 - q).max() < np.abs(s - q).max()):
                    s = s2
            if s is None:
                raise RuntimeError(f'IK failed at {np.round(p, 3)}')
            out.append(s)
            q = s
        return out

    def go_line(self, p1, R, t, label, dry=False, seed=None):
        p0 = self.tcp_now()[0] if seed is None else self.tcp_of(seed, R)
        pts = self.line(p0, p1)
        qs = self.path(pts, R, self.r.arm_q() if seed is None else seed)
        jump = max(np.abs(np.diff(np.array([qs[0] if seed is None else seed] + qs), axis=0)).max(1))
        print(f'  {label}: {len(qs)} pts, max step {jump:.2f}, last q {np.round(qs[-1], 2)}')
        if not dry:
            self.exec(qs, t, label)
        return qs[-1]

    def tcp_of(self, q, R=None):
        p, Rq = self.r.fk(q)
        return p + TCP * Rq[:, 2]

    # ---- eye-in-hand cloud -------------------------------------------
    def grab(self, topic, typ, timeout=15.0):
        got = {}
        sub = self.r.node.create_subscription(typ, topic, lambda m: got.setdefault('m', m), 1)
        import time
        t0 = time.time()
        while 'm' not in got and time.time() - t0 < timeout:
            rclpy.spin_once(self.r.node, timeout_sec=0.2)
        self.r.node.destroy_subscription(sub)
        return got.get('m')

    def eih_cloud(self):
        cam = 'robot0_eye_in_hand'
        d = self.grab(f'/{cam}/depth/image_raw', Image)
        info = self.grab(f'/{cam}/color/camera_info', CameraInfo)
        depth = np.frombuffer(d.data, dtype=np.float32).reshape(d.height, d.width)
        fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
        v, u = np.mgrid[0:d.height, 0:d.width]
        z = depth
        ok = np.isfinite(z) & (z > 0.05)
        pc = np.stack([(u - cx) * z / fx, (v - cy) * z / fy, z], -1)[ok]
        frame = f'{cam}_optical_frame'
        import time
        t0 = time.time()
        while time.time() - t0 < 10 and not self.tfbuf.can_transform('world', frame, rclpy.time.Time()):
            rclpy.spin_once(self.r.node, timeout_sec=0.2)
        t = self.tfbuf.lookup_transform('world', frame, rclpy.time.Time())
        q = t.transform.rotation
        R = quat_R([q.x, q.y, q.z, q.w])
        tr = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
        return pc @ R.T + tr

    def find_knob(self, guess, rad=0.05):
        P = self.eih_cloud()
        near = P[(np.abs(P[:, 0] - guess[0]) < rad) & (np.abs(P[:, 1] - guess[1]) < rad)]
        if len(near) == 0:
            print('  no points near guess'); return None
        zmax = near[:, 2].max()
        top = near[near[:, 2] > zmax - 0.008]
        c = top[:, :2].mean(0)
        print(f'  knob: zmax={zmax:.4f} n_top={len(top)} centre={np.round(c, 4)} '
              f'extent x{np.round(np.ptp(top[:,0]),3)} y{np.round(np.ptp(top[:,1]),3)}')
        return c, zmax

    # ---- the routine ---------------------------------------------------
    def run(self, name, dry=False):
        pot = POTS[name]
        R = R_of()
        kx, ky = pot['knob']
        tx, ty = pot['target']
        q0 = self.r.arm_q()
        print('start q', np.round(q0, 2), 'gap', round(self.r.finger_gap(), 4))
        if not dry:
            print('open', self.r.gripper(0.04))
        hover = np.array([kx, ky, HOVER_Z])
        # move to hover via a single IK (free-space move)
        qh = self.r.ik_best(hover, R, prev=q0, n_random=40)
        if qh is None:
            raise RuntimeError('no IK for hover')
        print('  hover q', np.round(qh, 2))
        if not dry:
            self.exec([qh], 4.0, 'hover')
            res = self.find_knob((kx, ky))
            if res is not None:
                (kx, ky), zmax = res
                # re-hover exactly above the refined knob
                self.go_line([kx, ky, HOVER_Z], R, 1.5, 'rehover')
        seed = qh
        # descend
        g = np.array([kx, ky, KNOB_Z])
        seed = self.go_line(g, R, 4.0, 'descend', dry, seed if dry else None)
        if not dry:
            p = self.tcp_now()[0]; print('  tcp at pinch', np.round(p, 4))
            print('close', self.r.gripper(0.0))
            gap = self.r.finger_gap(); print(f'  gap after close {gap:.4f}')
            if gap < 0.004:
                print('GRASP FAILED (gap too small); opening and stopping')
                self.r.gripper(0.04)
                self.go_line([kx, ky, HOVER_Z], R, 3.0, 'abort-up')
                return False
        # lift
        seed = self.go_line([kx, ky, CARRY_Z], R, 3.0, 'lift', dry, seed if dry else None)
        if not dry:
            gap = self.r.finger_gap(); print(f'  gap after lift {gap:.4f}')
        # carry
        seed = self.go_line([tx, ty, CARRY_Z], R, 6.0, 'carry', dry, seed if dry else None)
        # lower
        seed = self.go_line([tx, ty, PLACE_Z], R, 4.0, 'lower', dry, seed if dry else None)
        if not dry:
            print('open', self.r.gripper(0.04))
        # retreat straight up
        seed = self.go_line([tx, ty, CARRY_Z], R, 3.0, 'retreat', dry, seed if dry else None)
        return True


if __name__ == '__main__':
    name = sys.argv[1]
    dry = len(sys.argv) > 2 and sys.argv[2] == 'dry'
    k = Knob()
    ok = k.run(name, dry)
    print('done', ok)
OPENRUA_EOF

# openrua op 40
timeout 300 python3 knob.py A dry 2>&1 | tail -20; echo ======; timeout 300 python3 knob.py B dry 2>&1 | tail -20

# openrua op 41
python3 - <<'EOF'
import re
s = open('knob.py').read()
s = s.replace("""    def path(self, tcps, R, seed):
        q = np.asarray(seed)
        out = []
        for p in tcps:
            s = self.r.ik_tcp(p, R, seed=q, timeout=0.5)""", """    def path(self, tcps, R, seed):
        q = np.asarray(seed)
        out = []
        Rs = R if isinstance(R, list) else [R] * len(tcps)
        for p, R in zip(tcps, Rs):
            s = self.r.ik_tcp(p, R, seed=q, timeout=0.5)""")
s = s.replace("""    def go_line(self, p1, R, t, label, dry=False, seed=None):
        p0 = self.tcp_now()[0] if seed is None else self.tcp_of(seed, R)
        pts = self.line(p0, p1)
        qs = self.path(pts, R, self.r.arm_q() if seed is None else seed)""",
"""    def go_line(self, p1, R, t, label, dry=False, seed=None, pitch_from=None, pitch_to=None):
        p0 = self.tcp_now()[0] if seed is None else self.tcp_of(seed)
        pts = self.line(p0, p1)
        if pitch_from is not None:      # interpolate the hand pitch along the line
            n = len(pts)
            R = [R_of(pitch_from + (pitch_to - pitch_from) * (i + 1) / n) for i in range(n)]
        qs = self.path(pts, R, self.r.arm_q() if seed is None else seed)""")
s = s.replace("""    def run(self, name, dry=False):
        pot = POTS[name]
        R = R_of()""", """    def run(self, name, dry=False):
        pot = POTS[name]
        Rp = R_of(pot['pick_pitch'])
        R = R_of()""")
# pickup phases use Rp
s = s.replace("qh = self.r.ik_best(hover, R, prev=q0, n_random=40)", "qh = self.r.ik_best(hover, Rp, prev=q0, n_random=40)")
s = s.replace("self.go_line([kx, ky, HOVER_Z], R, 1.5, 'rehover')", "self.go_line([kx, ky, HOVER_Z], Rp, 1.5, 'rehover')")
s = s.replace("seed = self.go_line(g, R, 4.0, 'descend', dry, seed if dry else None)", "seed = self.go_line(g, Rp, 4.0, 'descend', dry, seed if dry else None)")
s = s.replace("self.go_line([kx, ky, HOVER_Z], R, 3.0, 'abort-up')", "self.go_line([kx, ky, HOVER_Z], Rp, 3.0, 'abort-up')")
s = s.replace("seed = self.go_line([kx, ky, CARRY_Z], R, 3.0, 'lift', dry, seed if dry else None)", "seed = self.go_line([kx, ky, CARRY_Z], Rp, 3.0, 'lift', dry, seed if dry else None)")
s = s.replace("seed = self.go_line([tx, ty, CARRY_Z], R, 6.0, 'carry', dry, seed if dry else None)",
 "seed = self.go_line([tx, ty, CARRY_END_Z], R, 6.0, 'carry', dry, seed if dry else None,\n                            pitch_from=pot['pick_pitch'], pitch_to=PITCH)")
s = s.replace("seed = self.go_line([tx, ty, CARRY_Z], R, 3.0, 'retreat', dry, seed if dry else None)", "seed = self.go_line([tx, ty, CARRY_END_Z], R, 3.0, 'retreat', dry, seed if dry else None)")
s = s.replace("CARRY_Z = 1.20\n", "CARRY_Z = 1.20\nCARRY_END_Z = 1.14               # above the target: pot bottom ~0.98, clears the dial (0.959)\n")
s = s.replace("'A': dict(knob=(-0.1965, -0.201), target=(0.22, -0.01)),", "'A': dict(knob=(-0.1965, -0.201), target=(0.22, -0.01), pick_pitch=float(sys.argv[3]) if len(sys.argv) > 3 else 0.0),")
s = s.replace("'B': dict(knob=(-0.0425, 0.255), target=(0.16, 0.085)),", "'B': dict(knob=(-0.0425, 0.255), target=(0.16, 0.085), pick_pitch=PITCH),")
open('knob.py','w').write(s)
EOF
for p in 0.0 -0.2 -0.4; do echo "=== pick_pitch $p"; timeout 300 python3 knob.py A dry $p 2>&1 | grep -vE "Traceback|File|\^" | tail -8; done

# openrua op 42
sed -i "s/'carry', dry, seed if dry else None,/'carry', dry, seed if dry else None,/; s/R, 6.0, 'carry'/R, 8.0, 'carry'/" knob.py && timeout 900 python3 knob.py A 2>&1 | grep -v "^\[" 

# openrua op 43
python3 tools/perception/cam_snap.py birdview bird2.png && python3 tools/perception/cam_snap.py agentview agent2.png && python3 tools/perception/cam_snap.py frontview front2.png && timeout 60 python3 scene.py birdview 2>&1 | tail -2 && python3 - <<'EOF'
import numpy as np
P=np.load('birdview_world.npy'); P=P[np.isfinite(P).all(1)]
# objects above table (z>0.93) excluding robot region x<-0.3
Q=P[(P[:,2]>0.925)&(P[:,0]>-0.35)&(np.abs(P[:,1])<0.45)&(P[:,0]<0.45)]
print('n',len(Q))
# cluster crudely by grid
from collections import defaultdict
g=defaultdict(list)
for p in Q: g[(round(p[0]/0.02),round(p[1]/0.02))].append(p)
# print cells with zmax>1.0
cells=[(k,len(v),max(q[2] for q in v)) for k,v in g.items()]
for k,n,zm in sorted(cells):
    if zm>0.98: print(k[0]*0.02,k[1]*0.02,n,round(zm,3))
EOF

# openrua op 44
timeout 120 python3 - <<'EOF' 2>&1 | grep -v "^\["
import numpy as np
from knob import *
k = Knob()
p, R = k.tcp_now(); print('tcp', np.round(p,3), 'hand z axis', np.round(R[:,2],2))
P = k.eih_cloud()
print('n', len(P), 'z range', P[:,2].min().round(3), P[:,2].max().round(3))
# exclude finger points: z above tcp-0.02
Q = P[P[:,2] < p[2]-0.03]
print('below tcp:', len(Q))
for lo,hi in [(0.88,0.91),(0.91,0.925),(0.925,0.94),(0.94,0.955),(0.955,0.97),(0.97,1.0),(1.0,1.1)]:
    S = Q[(Q[:,2]>=lo)&(Q[:,2]<hi)]
    if len(S): print(f'z[{lo},{hi}) n={len(S)} x[{S[:,0].min():.3f},{S[:,0].max():.3f}] y[{S[:,1].min():.3f},{S[:,1].max():.3f}]')
EOF

# openrua op 45
python3 - <<'EOF'
s = open('knob.py').read()
s = s.replace("""    def find_knob(self, guess, rad=0.05):
        P = self.eih_cloud()
        near = P[(np.abs(P[:, 0] - guess[0]) < rad) & (np.abs(P[:, 1] - guess[1]) < rad)]""",
"""    def find_knob(self, guess, rad=0.05):
        P = self.eih_cloud()
        tcp_z = self.tcp_now()[0][2]
        # drop the gripper's own fingers (near the lens) and anything not pot-top height
        P = P[(P[:, 2] < tcp_z - 0.05) & (P[:, 2] > 1.0) & (P[:, 2] < 1.10)]
        near = P[(np.abs(P[:, 0] - guess[0]) < rad) & (np.abs(P[:, 1] - guess[1]) < rad)]""")
s = s.replace("""    def go_line(self, p1, R, t, label, dry=False, seed=None, pitch_from=None, pitch_to=None):""",
"""    VMAX = 0.4                      # rad/s cap for the slowest joint step

    def exec(self, qs, total_t, label=''):
        # time each segment by its largest joint step so the controller can track
        q0 = self.r.arm_q()
        steps = [np.abs(b - a).max() for a, b in zip([q0] + list(qs[:-1]), qs)]
        segs = [max(0.3, st / self.VMAX) for st in steps]
        scale = max(1.0, total_t / sum(segs))
        ts = list(np.cumsum(segs) * scale)
        code = self.r.move(qs, ts)
        err = np.abs(self.r.arm_q() - qs[-1]).max()
        print(f'  [{label}] {len(qs)} pts in {ts[-1]:.1f}s code={code} final joint err={err:.4f}')
        if code != 0 or err > 0.05:
            print('  retrying last point')
            code = self.r.move([qs[-1]], [2.0])
            err = np.abs(self.r.arm_q() - qs[-1]).max()
            print(f'  [{label}] retry code={code} err={err:.4f}')
        return code, err

    def go_line(self, p1, R, t, label, dry=False, seed=None, pitch_from=None, pitch_to=None):""")
open('knob.py','w').write(s)
EOF
timeout 900 python3 knob.py A 2>&1 | grep -v "^\["

# openrua op 46
grep -n -i -E "veloc|speed|slow|fast|rad/s|duration|tolerance|-5" docs/30-action.md docs/40-patterns.md docs/10-machine.md | head -40

# openrua op 47
sed -n 105,125p docs/30-action.md; sed -n 10,30p docs/30-action.md

# openrua op 48
timeout 120 python3 - <<'EOF' 2>&1 | grep -v "^\["
import numpy as np, time, rclpy
from robot import *
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
r = Robot()
q0 = r.arm_q(); print('q0', np.round(q0,3))
# move j1 by +0.6 rad in 3 s, log progress
q1 = q0.copy(); q1[0] += 0.6
goal = FollowJointTrajectory.Goal(); goal.trajectory.joint_names = list(ARM)
pt = JointTrajectoryPoint(positions=[float(v) for v in q1]); pt.time_from_start = Duration(sec=3)
goal.trajectory.points.append(pt)
fut = r.fjt.send_goal_async(goal)
rclpy.spin_until_future_complete(r.node, fut, timeout_sec=10)
h = fut.result(); rf = h.get_result_async()
t0 = time.time(); log=[]
while not rf.done():
    rclpy.spin_once(r.node, timeout_sec=0.05)
    if 'm' in r._js:
        m = r._js['m']; j = dict(zip(m.name, m.position))
        log.append((time.time()-t0, m.header.stamp.sec + m.header.stamp.nanosec*1e-9, j['panda_joint1']))
print('code', rf.result().result.error_code)
last=None
for wall, st, j1 in log:
    if last is None or st-last>=0.25: print(f'wall {wall:.2f} sim {st:.2f} j1 {j1:.3f}'); last=st
print('final', round(r.arm_q()[0],3), 'target', round(q1[0],3))
# move back
print(r.move_to(q0, 4.0))
EOF

# openrua op 49
timeout 200 python3 - <<'EOF' 2>&1 | grep -v "^\["
import numpy as np, time, rclpy
from robot import *
r = Robot()
q0 = r.arm_q(); print('q0', np.round(q0,3))
for dq, T in [(0.9, 3.0), (0.9, 6.0), (1.5, 6.0)]:
    q1 = q0.copy(); q1[0] += dq
    code, err = r.move_to(q1, T)
    print(f'dq={dq} T={T} -> code {code} err {err:.3f}  ({dq/T:.2f} rad/s requested)')
    code, err = r.move_to(q0, 8.0)
    print(f'   back: code {code} err {err:.3f}')
EOF

# openrua op 50
python3 - <<'EOF'
s = open('knob.py').read()
s = s.replace("    VMAX = 0.4                      # rad/s cap for the slowest joint step", "    VMAX = 0.15                     # rad/s; controller tracks ~0.2 rad/s at most")
s = s.replace("""        if code != 0 or err > 0.05:
            print('  retrying last point')
            code = self.r.move([qs[-1]], [2.0])
            err = np.abs(self.r.arm_q() - qs[-1]).max()
            print(f'  [{label}] retry code={code} err={err:.4f}')
        return code, err""", """        tries = 0
        while (code != 0 or err > 0.02) and tries < 4:
            tries += 1
            code = self.r.move([qs[-1]], [max(1.0, err / self.VMAX)])
            err = np.abs(self.r.arm_q() - qs[-1]).max()
            print(f'  [{label}] retry {tries} code={code} err={err:.4f}')
        return code, err

    def settle(self, t=1.0):
        q = self.r.arm_q()
        self.r.move([q], [t])""")
s = s.replace("""        P = P[(P[:, 2] < tcp_z - 0.05) & (P[:, 2] > 1.0) & (P[:, 2] < 1.10)]
        near = P[(np.abs(P[:, 0] - guess[0]) < rad) & (np.abs(P[:, 1] - guess[1]) < rad)]
        if len(near) == 0:
            print('  no points near guess'); return None
        zmax = near[:, 2].max()
        top = near[near[:, 2] > zmax - 0.008]
        c = top[:, :2].mean(0)
        print(f'  knob: zmax={zmax:.4f} n_top={len(top)} centre={np.round(c, 4)} '
              f'extent x{np.round(np.ptp(top[:,0]),3)} y{np.round(np.ptp(top[:,1]),3)}')
        return c, zmax""", """        P = P[(P[:, 2] < tcp_z - 0.05) & (P[:, 2] > 1.04) & (P[:, 2] < 1.10)]
        near = P[(np.abs(P[:, 0] - guess[0]) < rad) & (np.abs(P[:, 1] - guess[1]) < rad)]
        if len(near) < 20:
            print('  too few knob points near guess', len(near)); return None
        zmax = near[:, 2].max()
        top = near[near[:, 2] > zmax - 0.006]
        c = top[:, :2].mean(0)
        ext = np.ptp(top[:, 0]), np.ptp(top[:, 1])
        print(f'  knob: zmax={zmax:.4f} n_top={len(top)} centre={np.round(c, 4)} '
              f'extent {np.round(ext, 3)}  (birdview guess {guess})')
        if max(ext) > 0.03 or np.linalg.norm(c - np.asarray(guess)) > 0.025:
            print('  refinement rejected, keeping the guess'); return None
        return c, zmax""")
s = s.replace("            res = self.find_knob((kx, ky))", "            res = self.find_knob((kx, ky), rad=0.03)")
s = s.replace("""            print('close', self.r.gripper(0.0))
            gap = self.r.finger_gap(); print(f'  gap after close {gap:.4f}')""", """            print('close', self.r.gripper(0.0))
            self.settle(1.0)
            gap = self.r.finger_gap(); print(f'  gap after close {gap:.4f}')""")
open('knob.py','w').write(s)
EOF
timeout 1500 python3 knob.py A 2>&1 | grep -v "^\["

# openrua op 51
python3 -c "
import cv2; im=cv2.imread('front2.png'); print(im.shape)
c=im[150:330, 380:560]; c=cv2.resize(c,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC); cv2.imwrite('crop_front_pot.png',c)"

# openrua op 52
python3 -c "
import cv2; im=cv2.imread('front2.png')
c=im[300:360, 410:500]; c=cv2.resize(c,None,fx=6,fy=6,interpolation=cv2.INTER_CUBIC); cv2.imwrite('crop_knob.png',c)"

# openrua op 53
python3 - <<'EOF'
s = open('knob.py').read()
s = s.replace("KNOB_Z = 1.056                   # TCP height for the pinch (knob spans ~1.045-1.06)",
              "KNOB_Z = 1.044                   # fingertip (TCP) height for the pinch: knob spans ~1.041-1.053\nKNOB_TOP_DROP = 0.009            # pinch = measured knob top - this")
s = s.replace("PLACE_Z = KNOB_Z + 0.040         # pot bottom ~0.94, 1 cm above the burner (0.93)",
              "PLACE_DZ = 0.040                 # release TCP = pinch height + this: pot bottom ~0.94, 1 cm above the burner")
s = s.replace("""        seed = qh
        # descend
        g = np.array([kx, ky, KNOB_Z])""", """        seed = qh
        knob_z = KNOB_Z
        if not dry and res is not None:
            knob_z = zmax - KNOB_TOP_DROP
        print(f'  pinch height {knob_z:.4f}')
        # descend
        g = np.array([kx, ky, knob_z])""")
s = s.replace("seed = self.go_line([tx, ty, PLACE_Z], R, 4.0, 'lower', dry, seed if dry else None)",
              "seed = self.go_line([tx, ty, knob_z + PLACE_DZ], R, 4.0, 'lower', dry, seed if dry else None)")
open('knob.py','w').write(s)
EOF
grep -n "res = None\|res = self.find" knob.py

# openrua op 54
python3 - <<'EOF'
s = open('knob.py').read()
s = s.replace("""        print('  hover q', np.round(qh, 2))
        if not dry:""", """        print('  hover q', np.round(qh, 2))
        res = None
        if not dry:""")
open('knob.py','w').write(s)
EOF
timeout 1500 python3 knob.py A 2>&1 | grep -v "^\["

# openrua op 55
python3 tools/perception/cam_snap.py birdview bird3.png >/dev/null && python3 tools/perception/cam_snap.py agentview agent3.png >/dev/null && timeout 60 python3 scene.py birdview >/dev/null 2>&1; python3 - <<'EOF'
import numpy as np
P=np.load('birdview_world.npy'); print(P.shape)
P=P.reshape(-1,3) if P.ndim==3 else P
P=P[np.isfinite(P).all(1)]
Q=P[(P[:,2]>0.945)&(P[:,0]>-0.4)&(P[:,0]<0.45)&(np.abs(P[:,1])<0.45)]
from collections import defaultdict
g=defaultdict(list)
for p in Q: g[(round(p[0]/0.03),round(p[1]/0.03))].append(p[2])
for k,v in sorted(g.items()):
    if max(v)>1.0 and len(v)>5: print(f'x{k[0]*0.03:+.2f} y{k[1]*0.03:+.2f} n={len(v)} zmax={max(v):.3f}')
EOF

# openrua op 56
timeout 300 python3 - <<'EOF' 2>&1 | grep -v "^\["
import numpy as np
from knob import *
k = Knob()
q_home = np.array([0, -0.161, 0, -2.445, 0, 2.227, 0.785])
k.exec([q_home], 14.0, 'home')
EOF
python3 tools/perception/cam_snap.py birdview bird4.png >/dev/null && timeout 60 python3 scene.py birdview >/dev/null 2>&1; python3 - <<'EOF'
import numpy as np
P=np.load('birdview_world.npy'); P=P.reshape(-1,3); P=P[np.isfinite(P).all(1)]
# pot A region
for name,(x0,x1,y0,y1) in {'potA':(0.12,0.32,-0.1,0.12),'potB':(-0.12,0.03,0.18,0.33)}.items():
    Q=P[(P[:,0]>x0)&(P[:,0]<x1)&(P[:,1]>y0)&(P[:,1]<y1)]
    print(name, 'zmax', Q[:,2].max().round(4))
    for lo,hi in [(0.925,0.95),(0.95,1.0),(1.0,1.04),(1.04,1.06),(1.06,1.075),(1.075,1.10)]:
        S=Q[(Q[:,2]>=lo)&(Q[:,2]<hi)]
        if len(S)>3: print(f'  z[{lo},{hi}) n={len(S)} x[{S[:,0].min():.3f},{S[:,0].max():.3f}] y[{S[:,1].min():.3f},{S[:,1].max():.3f}]')
    top=Q[Q[:,2]>Q[:,2].max()-0.008]; print('  top centre', top[:,:2].mean(0).round(4), 'n', len(top))
EOF

# openrua op 57
timeout 300 python3 knob.py B dry 2>&1 | grep -v "^\["

# openrua op 58
timeout 1500 python3 knob.py B 2>&1 | grep -v "^\["

# openrua op 59
timeout 300 python3 - <<'EOF' 2>&1 | grep -v "^\["
import numpy as np
from knob import *
k = Knob()
q_home = np.array([0, -0.161, 0, -2.445, 0, 2.227, 0.785])
k.exec([q_home], 10.0, 'home')
print('final q', np.round(k.r.arm_q(),3))
EOF
for c in birdview agentview frontview sideview; do python3 tools/perception/cam_snap.py $c final_$c.png >/dev/null; done; timeout 60 python3 scene.py birdview >/dev/null 2>&1; python3 - <<'EOF'
import numpy as np
P=np.load('birdview_world.npy'); P=P.reshape(-1,3); P=P[np.isfinite(P).all(1)]
# anything above the table in the whole workspace (excluding robot x<-0.35)
Q=P[(P[:,2]>0.945)&(P[:,0]>-0.35)&(P[:,0]<0.45)&(np.abs(P[:,1])<0.45)]
print('points above z=0.945 outside robot column:', len(Q), 'x range', Q[:,0].min().round(3), Q[:,0].max().round(3), 'y range', Q[:,1].min().round(3), Q[:,1].max().round(3))
for name,(x0,x1,y0,y1) in {'potA':(0.17,0.30,-0.10,0.04),'potB':(0.10,0.21,0.04,0.14)}.items():
    Q=P[(P[:,0]>x0)&(P[:,0]<x1)&(P[:,1]>y0)&(P[:,1]<y1)]
    zmax=Q[:,2].max(); top=Q[Q[:,2]>zmax-0.008]
    rim=Q[(Q[:,2]>zmax-0.025)&(Q[:,2]<zmax-0.01)]
    print(f'{name}: knob top z={zmax:.4f} -> bottom at {zmax-0.1532:.4f}; knob centre {top[:,:2].mean(0).round(4)}; lid rim x[{rim[:,0].min():.3f},{rim[:,0].max():.3f}] y[{rim[:,1].min():.3f},{rim[:,1].max():.3f}]')
# old spots empty?
for name,(x0,x1,y0,y1) in {'oldA':(-0.26,-0.13,-0.27,-0.13),'oldB':(-0.11,0.03,0.18,0.33)}.items():
    Q=P[(P[:,0]>x0)&(P[:,0]<x1)&(P[:,1]>y0)&(P[:,1]<y1)]
    print(name, 'zmax', Q[:,2].max().round(3))
EOF

# openrua op 60
mkdir -p "$(dirname /claude-config/projects/-workspace/memory/panda-sim-machine-facts.md)"
cat > /claude-config/projects/-workspace/memory/panda-sim-machine-facts.md <<'OPENRUA_EOF'
---
name: panda-sim-machine-facts
description: Hard-won facts about the /workspace Panda robosuite sim (frames, speed cap, gripper, cameras) that the docs get wrong or omit
metadata:
  type: project
---

Facts about the Franka Panda sim at /workspace (verified 2026-09-13):
- MoveIt `/compute_fk` and `/compute_ik` poses are already WORLD frame (docs claim arm-base frame). panda_link0 is at world (-0.66, 0, 0.912). Leave `frame_id` empty.
- Joint trajectory controller tracks at most ~0.2 rad/s per joint; faster goals return error -5 with the arm short of target. Time segments at <=0.15 rad/s and resend the last point until error < 0.02.
- TCP offset 0.1034 from `panda_hand` is at the FINGERTIP; pads extend ~2 cm upward from it. Open finger gap reads 0.0775 (finger_joint2 negative sign). Gap readings need a settle tick (send a 1 s hold trajectory) before trusting them.
- Eye-in-hand depth cloud includes the gripper's own fingers ~5 cm from the lens; filter points above TCP z - 0.05. TF `world -> robot0_eye_in_hand_optical_frame` is published directly; `world -> panda_*` is not.
- IK is fast but seed-sensitive; multi-start (`ik_best` in /workspace/robot.py) avoids joint-limit branches.

**Why:** Each of these cost a failed attempt to learn; the starter docs are generic.
**How to apply:** Reuse /workspace/robot.py and /workspace/knob.py (knob-pinch pick-and-place worked for both moka pots) before rewriting.
OPENRUA_EOF

# openrua op 61
cd /claude-config/projects/-workspace/memory && (test -f MEMORY.md && cat MEMORY.md || true); echo "- [Panda sim machine facts](panda-sim-machine-facts.md) — world-frame FK/IK, 0.2 rad/s speed cap, TCP at fingertip, eye-in-hand sees own fingers" >> MEMORY.md
