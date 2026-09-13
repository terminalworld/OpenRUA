#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat tools/perception/cam_snap.py tools/perception/px2world.py

# openrua op 2
cat tools/action/fjt_send.py tools/action/gripper_cmd.py tools/action/ik_move.py

# openrua op 3
ros2 topic list 2>&1; echo ---; ros2 action list 2>&1; echo ---; ros2 service list 2>&1 | head -50; echo ---; ros2 node list 2>&1

# openrua op 4
for c in agentview frontview sideview birdview robot0_eye_in_hand robot0_robotview; do timeout 60 python3 tools/perception/cam_snap.py $c; done; ros2 topic echo /joint_states --once

# openrua op 5
timeout 30 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -8; timeout 30 ros2 run tf2_ros tf2_echo world birdview_optical_frame 2>&1 | head -8; ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id|child" | head -40

# openrua op 6
mkdir -p "$(dirname /workspace/cloud.py)"
cat > /workspace/cloud.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Dump a camera's depth frame as a world-frame point cloud (.npy, HxWx3)
plus the color image. Usage: python3 cloud.py <camera>"""
import sys
import numpy as np
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
import cv2


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
    br = CvBridge()
    depth = br.imgmsg_to_cv2(grab(node, f"/{cam}/depth/image_raw", Image), "passthrough").astype(np.float64)
    color = br.imgmsg_to_cv2(grab(node, f"/{cam}/color/image_raw", Image), "bgr8")
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    frame = f"{cam}_optical_frame"
    import time
    end = time.time() + 15
    while time.time() < end and not tfbuf.can_transform("world", frame, rclpy.time.Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    R = quat_R(q.x, q.y, q.z, q.w)
    T = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    h, w = depth.shape
    u, v = np.meshgrid(np.arange(w), np.arange(h))
    X = (u - cx) * depth / fx
    Y = (v - cy) * depth / fy
    P = np.stack([X, Y, depth], -1) @ R.T + T
    np.save(f"{cam}_cloud.npy", P)
    cv2.imwrite(f"{cam}.png", color)
    print(f"{cam}: {w}x{h}, cam at {T}, saved {cam}_cloud.npy")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 7
timeout 120 python3 cloud.py birdview && timeout 120 python3 cloud.py agentview && timeout 120 python3 cloud.py sideview

# openrua op 8
python3 -c "
import numpy as np, cv2
P = np.load('birdview_cloud.npy'); img = cv2.imread('birdview.png')
# table height: mode of z in center
z = P[...,2]
print('z range', np.nanmin(z), np.nanmax(z))
h = np.histogram(z[np.isfinite(z)], bins=100)
for c,e in zip(*h):
    if c>500: print(f'{e:.3f} {c}')
# yellow mug pixel in birdview ~ (330, 292); gray mug ~ (420, 295); microwave ~ (245, 265)
for name,(u,v) in {'yellow_mug':(330,292),'gray_mug':(420,295),'microwave_top':(245,265),'table':(400,400),'door':(180,190)}.items():
    print(name, P[v,u], img[v,u])
"


# openrua op 9
python3 -c "
import numpy as np, cv2
P = np.load('birdview_cloud.npy'); img = cv2.imread('birdview.png')
z=P[...,2]
m = (z>1.09)&(z<1.12)
pts = P[m]
print('microwave-top-ish points', len(pts), 'x', pts[:,0].min(), pts[:,0].max(), 'y', pts[:,1].min(), pts[:,1].max())
# split: body vs door. body is roughly rectangular; look at y distribution
import collections
xs = np.round(pts[:,0],2); ys=np.round(pts[:,1],2)
# print occupancy grid coarse
gx = np.arange(-0.5,0.3,0.05); gy=np.arange(-0.6,0.0,0.05)
H,_,_ = np.histogram2d(pts[:,0],pts[:,1],bins=[gx,gy])
print('rows x from -0.5 step .05, cols y from -0.6 step .05')
for i,row in enumerate(H):
    print(f'{gx[i]:+.2f}', ''.join('#' if c>20 else ('.' if c>0 else ' ') for c in row))
# mug
m2 = (z>0.95)&(z<1.0)&(P[...,1]>-0.1)&(P[...,1]<0.15)
pts2=P[m2]; print('yellow mug rim pts', len(pts2), pts2[:,0].min(), pts2[:,0].max(), pts2[:,1].min(), pts2[:,1].max(), 'center', pts2[:,:2].mean(0))
m3 = (z>0.95)&(z<1.0)&(P[...,1]>0.25)&(P[...,1]<0.45)
pts3=P[m3]; print('gray mug rim pts', len(pts3), pts3[:,0].min(), pts3[:,0].max(), pts3[:,1].min(), pts3[:,1].max(), 'center', pts3[:,:2].mean(0))
"


# openrua op 10
python3 -c "
import numpy as np
pts = np.concatenate([np.load(f'{c}_cloud.npy').reshape(-1,3) for c in ['birdview','agentview','sideview']])
pts = pts[np.isfinite(pts).all(1)]
sel = pts[(pts[:,2]>0.93)&(pts[:,2]<1.3)&(pts[:,0]>-0.55)&(pts[:,0]<0.15)&(pts[:,1]>-0.7)&(pts[:,1]<0.0)]
gx = np.arange(-0.55,0.15,0.02); gy=np.arange(-0.7,0.0,0.02)
H,_,_ = np.histogram2d(sel[:,0],sel[:,1],bins=[gx,gy])
print('      y: ' + ''.join(f'{y:+.1f}'.ljust(5) if i%5==0 else '' for i,y in enumerate(gy[:-1])))
for i,row in enumerate(H):
    print(f'{gx[i]:+.2f}', ''.join('#' if c>10 else ('.' if c>0 else ' ') for c in row))
"


# openrua op 11
python3 -c "
import numpy as np
for c in ['agentview','sideview','birdview']:
    P=np.load(f'{c}_cloud.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
    inside = P[(P[:,0]>-0.27)&(P[:,0]<0.04)&(P[:,1]>-0.34)&(P[:,1]<-0.16)&(P[:,2]>0.91)&(P[:,2]<1.08)]
    print(c, 'points inside body volume:', len(inside))
    if len(inside):
        print('  x',inside[:,0].min(),inside[:,0].max(),'y',inside[:,1].min(),inside[:,1].max(),'z',inside[:,2].min(),inside[:,2].max())
"


# openrua op 12
timeout 120 python3 cloud.py robot0_robotview && python3 -c "
import numpy as np
P=np.load('robot0_robotview_cloud.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
inside = P[(P[:,0]>-0.27)&(P[:,0]<0.04)&(P[:,1]>-0.34)&(P[:,1]<-0.16)&(P[:,2]>0.91)&(P[:,2]<1.08)]
print('points inside body volume:', len(inside))
if len(inside): print('  x',inside[:,0].min(),inside[:,0].max(),'y',inside[:,1].min(),inside[:,1].max(),'z',inside[:,2].min(),inside[:,2].max())
face = P[(P[:,0]>-0.33)&(P[:,0]<-0.25)&(P[:,1]>-0.36)&(P[:,1]<-0.14)&(P[:,2]>0.91)&(P[:,2]<1.08)]
print('points on -x face plane:', len(face))
"

# openrua op 13
cat > /workspace/fk.py <<'EOF'
#!/usr/bin/env python3
"""Print the current hand pose (panda_hand in panda_link0 and world) via /compute_fk."""
import rclpy, yaml, numpy as np
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState

M = yaml.safe_load(open('/workspace/machine.yaml'))
ARM = M['actuators'][0]['joints']
BASE = np.array([-0.66, 0.0, 0.912])  # world -> panda_link0 (from TF)

def main():
    rclpy.init(); node = rclpy.create_node('fk')
    js = {}
    node.create_subscription(JointState, '/joint_states', lambda m: js.setdefault('m', m), 1)
    while 'm' not in js: rclpy.spin_once(node, timeout_sec=0.2)
    cli = node.create_client(GetPositionFK, '/compute_fk'); cli.wait_for_service()
    req = GetPositionFK.Request(); req.fk_link_names = ['panda_hand']
    d = dict(zip(js['m'].name, js['m'].position))
    req.robot_state.joint_state.name = ARM
    req.robot_state.joint_state.position = [d[j] for j in ARM]
    fut = cli.call_async(req); rclpy.spin_until_future_complete(node, fut, timeout_sec=30)
    r = fut.result()
    p = r.pose_stamped[0].pose
    print('joints', [round(d[j],4) for j in ARM], 'fingers', round(d['panda_finger_joint1'],4), round(d['panda_finger_joint2'],4))
    print('hand in base: %.4f %.4f %.4f  q %.4f %.4f %.4f %.4f' % (p.position.x,p.position.y,p.position.z,p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w))
    w = BASE + np.array([p.position.x,p.position.y,p.position.z])
    print('hand in world: %.4f %.4f %.4f' % tuple(w))
    rclpy.shutdown()
main()
EOF
timeout 60 python3 /workspace/fk.py

# openrua op 14
timeout 20 ros2 run tf2_ros tf2_echo panda_link0 panda_hand 2>&1 | grep -A2 "At time" | head -4; timeout 20 ros2 run tf2_ros tf2_echo world robot0_eye_in_hand_optical_frame 2>&1 | grep -A3 "At time" | head -5; ros2 topic echo /tf --once 2>&1 | head -30

# openrua op 15
python3 -c "
import cv2
img=cv2.imread('birdview.png'); crop=img[140:340,130:300]; cv2.imwrite('bird_crop.png', cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))
img=cv2.imread('agentview.png'); crop=img[80:360,0:260]; cv2.imwrite('agent_crop.png', cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
"

# openrua op 16
python3 -c "
import numpy as np, cv2
P=np.load('birdview_cloud.npy'); img=cv2.imread('birdview.png')
# map around mug at pixel (330,292)
for v in range(270,315,3):
    print(v, ' '.join(f'{P[v,u,2]-0.9:.2f}' if P[v,u,2]>0.91 else ' .  ' for u in range(300,360,3)))
print('u from 300 to 360 step 3; x,y at (300,270)=',P[270,300,:2],' (360,315)=',P[315,360,:2])
"

# openrua op 17
python3 -c "
import numpy as np
for c in ['agentview','sideview','frontview_x']:
    try: P=np.load(f'{c}_cloud.npy').reshape(-1,3)
    except Exception as e: continue
    P=P[np.isfinite(P).all(1)]
    mug = P[(P[:,0]>-0.08)&(P[:,0]<0.03)&(P[:,1]>-0.06)&(P[:,1]<0.1)&(P[:,2]>0.905)]
    print(c,'mug pts',len(mug),'z',mug[:,2].min(),mug[:,2].max(),'x',mug[:,0].min(),mug[:,0].max(),'y',mug[:,1].min(),mug[:,1].max())
    hist=np.histogram(mug[:,2],bins=np.arange(0.9,1.12,0.01))
    print('  z hist', [(round(e,2),int(n)) for e,n in zip(hist[1],hist[0]) if n>0])
    mw = P[(P[:,0]>-0.3)&(P[:,0]<0.07)&(P[:,1]>-0.37)&(P[:,1]<-0.13)&(P[:,2]>0.905)]
    print(c,'microwave z max',mw[:,2].max(), 'x max', mw[:,0].max())
    table = P[(P[:,0]>0.1)&(P[:,0]<0.3)&(abs(P[:,1])<0.2)]
    print(c,'table z', np.median(table[:,2]))
"

# openrua op 18
python3 -c "
import numpy as np
P=np.load('agentview_cloud.npy')
for v in range(270,300,2):
    print(v, ' '.join(f'{P[v,u,2]:.3f}' for u in range(320,360,5)))
P2=P.reshape(-1,3); P2=P2[np.isfinite(P2).all(1)]
g = P2[(P2[:,0]>-0.08)&(P2[:,0]<0.05)&(P2[:,1]>0.28)&(P2[:,1]<0.42)&(P2[:,2]>0.905)]
print('gray mug z', g[:,2].min(), g[:,2].max())
"

# openrua op 19
timeout 300 python3 tools/action/gripper_cmd.py 0.04; ros2 topic echo /joint_states --once | grep -A9 position | tail -2

# openrua op 20
timeout 120 python3 cloud.py agentview && timeout 120 python3 cloud.py birdview && python3 -c "
import numpy as np
P=np.load('agentview_cloud.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
mug = P[(P[:,0]>-0.1)&(P[:,0]<0.05)&(P[:,1]>-0.08)&(P[:,1]<0.12)&(P[:,2]>0.905)]
print('mug z',mug[:,2].min(),mug[:,2].max(),'x',mug[:,0].min(),mug[:,0].max(),'y',mug[:,1].min(),mug[:,1].max())
B=np.load('birdview_cloud.npy')
for v in range(276,310,3):
    print(v, ' '.join(f'{B[v,u,2]-0.9:.2f}' if B[v,u,2]>0.91 else ' .  ' for u in range(300,360,3)))
"

# openrua op 21
python3 -c "
import numpy as np
B=np.load('birdview_cloud.npy').reshape(-1,3); B=B[np.isfinite(B).all(1)]
rim = B[(B[:,2]>0.985)&(B[:,2]<1.02)&(B[:,0]>-0.1)&(B[:,0]<0.05)&(B[:,1]>-0.08)&(B[:,1]<0.12)]
print('rim+handle pts',len(rim),'x',rim[:,0].min(),rim[:,0].max(),'y',rim[:,1].min(),rim[:,1].max())
# handle is at -y; body rim: fit circle to points with y > -0.02
body = rim[rim[:,1]>-0.015]
# least squares circle fit
x,y=body[:,0],body[:,1]
A=np.c_[2*x,2*y,np.ones_like(x)]; b=x**2+y**2
c=np.linalg.lstsq(A,b,rcond=None)[0]; cx,cy=c[0],c[1]; r=np.sqrt(c[2]+cx**2+cy**2)
print('circle center %.4f %.4f radius %.4f'%(cx,cy,r))
A2=np.load('agentview_cloud.npy').reshape(-1,3); A2=A2[np.isfinite(A2).all(1)]
mug=A2[(A2[:,0]>-0.1)&(A2[:,0]<0.05)&(A2[:,1]>-0.08)&(A2[:,1]<0.12)&(A2[:,2]>0.92)]
for zlo in [0.92,0.95,0.98]:
    s=mug[(mug[:,2]>zlo)&(mug[:,2]<zlo+0.02)&(mug[:,1]>-0.02)]
    print('z',zlo,'y range body', s[:,1].min(), s[:,1].max(), 'x min', s[:,0].min())
"

# openrua op 22
python3 -c "
import numpy as np
B=np.load('birdview_cloud.npy').reshape(-1,3); B=B[np.isfinite(B).all(1)]
h = B[(B[:,2]>0.93)&(B[:,0]>-0.1)&(B[:,0]<0.05)&(B[:,1]>-0.06)&(B[:,1]<-0.005)]
print('handle-ish pts',len(h))
for p in sorted(h.tolist(), key=lambda p:p[1])[:40]: print('%.3f %.3f %.3f'%tuple(p))
" | head -45

# openrua op 23
cat > /workspace/iktest.py <<'EOF'
import rclpy, yaml, numpy as np
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
M = yaml.safe_load(open('/workspace/machine.yaml')); ARM = M['actuators'][0]['joints']
rclpy.init(); node = rclpy.create_node('iktest')
js = {}
node.create_subscription(JointState, '/joint_states', lambda m: js.setdefault('m', m), 1)
while 'm' not in js: rclpy.spin_once(node, timeout_sec=0.2)
d = dict(zip(js['m'].name, js['m'].position))
cli = node.create_client(GetPositionIK, '/compute_ik'); cli.wait_for_service()
for label, pos in [('world', (-0.2030, 0.0, 1.2696)), ('base', (0.457, 0.0, 0.358))]:
    req = GetPositionIK.Request(); req.ik_request.group_name = 'panda_arm'
    req.ik_request.pose_stamped.header.frame_id = ''
    p = req.ik_request.pose_stamped.pose
    p.position.x, p.position.y, p.position.z = pos
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = 0.9996, 0.0, -0.0284, 0.0
    req.ik_request.robot_state.joint_state.name = ARM
    req.ik_request.robot_state.joint_state.position = [d[j] for j in ARM]
    req.ik_request.timeout.sec = 5
    fut = cli.call_async(req); rclpy.spin_until_future_complete(node, fut, timeout_sec=60)
    r = fut.result()
    print(label, 'err', r.error_code.val, [round(v,3) for n,v in zip(r.solution.joint_state.name, r.solution.joint_state.position) if n in ARM])
print('current', [round(d[j],3) for j in ARM])
rclpy.shutdown()
EOF
timeout 120 python3 /workspace/iktest.py

# openrua op 24
mkdir -p "$(dirname /workspace/rob.py)"
cat > /workspace/rob.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Small helper library for this Panda workstation (world-frame poses).

IK/FK on this machine take/return WORLD coordinates (verified: FK of the
current state equals TF world->panda_hand; IK with world coords returns the
current joints).
"""
import sys
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import WrenchStamped
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open('/workspace/machine.yaml'))
ARM = M['actuators'][0]['joints']
LIMITS = M['actuators'][0]['limits_rad']
TCP = M['hand']['tcp_offset_m']


def quat_to_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def R_to_quat(R):
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1) * 2
        return np.array([(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s])
    i = np.argmax(np.diag(R))
    if i == 0:
        s = np.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return np.array([0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s])
    if i == 1:
        s = np.sqrt(1 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        return np.array([(R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s])
    s = np.sqrt(1 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
    return np.array([(R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s, (R[1, 0] - R[0, 1]) / s])


def quat_from_axes(hx, hy, hz):
    """Quaternion whose rotation has the given hand axes (world vectors) as columns."""
    R = np.stack([np.asarray(hx, float), np.asarray(hy, float), np.asarray(hz, float)], 1)
    return R_to_quat(R)


class Robot:
    def __init__(self, name='rob'):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = {}
        self._wr = {}
        self.node.create_subscription(JointState, '/joint_states', self._on_js, 1)
        self.node.create_subscription(WrenchStamped, M['sensors'][1]['port'], self._on_wr, 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, M['actuators'][0]['port'])
        self.grip = ActionClient(self.node, GripperCommand, M['actuators'][2]['port'])
        self.ik_cli = self.node.create_client(GetPositionIK, '/compute_ik')
        self.fk_cli = self.node.create_client(GetPositionFK, '/compute_fk')
        self.fjt.wait_for_server(10); self.grip.wait_for_server(10)
        self.ik_cli.wait_for_service(10); self.fk_cli.wait_for_service(10)

    def _on_js(self, m):
        self._js['m'] = m

    def _on_wr(self, m):
        self._wr['m'] = m

    def spin(self, t=0.05):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self, fresh=True):
        if fresh:
            self._js.clear()
        while 'm' not in self._js:
            self.spin(0.2)
        d = dict(zip(self._js['m'].name, self._js['m'].position))
        return d

    def arm_q(self):
        d = self.joints()
        return np.array([d[j] for j in ARM])

    def fingers(self):
        d = self.joints()
        return d['panda_finger_joint1'], d['panda_finger_joint2']

    def wrench(self):
        self._wr.clear()
        while 'm' not in self._wr:
            self.spin(0.2)
        w = self._wr['m'].wrench
        return np.array([w.force.x, w.force.y, w.force.z, w.torque.x, w.torque.y, w.torque.z])

    def fk(self, q=None):
        req = GetPositionFK.Request()
        req.fk_link_names = ['panda_hand']
        req.robot_state.joint_state.name = ARM
        req.robot_state.joint_state.position = list(self.arm_q() if q is None else q)
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        p = fut.result().pose_stamped[0].pose
        return (np.array([p.position.x, p.position.y, p.position.z]),
                np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]))

    def ik(self, pos, quat, seed=None, timeout=5.0, attempts=3):
        for _ in range(attempts):
            req = GetPositionIK.Request()
            req.ik_request.group_name = 'panda_arm'
            req.ik_request.pose_stamped.header.frame_id = ''
            p = req.ik_request.pose_stamped.pose
            p.position.x, p.position.y, p.position.z = map(float, pos)
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
            req.ik_request.robot_state.joint_state.name = ARM
            req.ik_request.robot_state.joint_state.position = list(map(float, self.arm_q() if seed is None else seed))
            req.ik_request.timeout.sec = int(timeout)
            req.ik_request.timeout.nanosec = int((timeout % 1) * 1e9)
            req.ik_request.avoid_collisions = False
            fut = self.ik_cli.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=120)
            r = fut.result()
            if r is not None and r.error_code.val == 1:
                sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
                return np.array([sol[j] for j in ARM])
        return None

    def move_q(self, q, seconds=3.0, via=None):
        """Send one trajectory (optionally through intermediate points `via`,
        list of (q, t)) and wait; returns error_code."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM
        pts = []
        for qq, t in (via or []) + [(q, seconds)]:
            pt = JointTrajectoryPoint(positions=[float(v) for v in qq])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=120)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        err = np.abs(self.arm_q() - np.asarray(q)).max()
        print(f'  move_q: error_code={code} max_joint_err={err:.4f}', flush=True)
        return code

    def move_pose(self, pos, quat, seconds=3.0, seed=None):
        q = self.ik(pos, quat, seed=seed)
        if q is None:
            print(f'  IK FAILED for {np.round(pos, 3)}', flush=True)
            return None
        code = self.move_q(q, seconds)
        p, _ = self.fk()
        print(f'  now at {np.round(p, 4)} (target {np.round(pos, 4)})', flush=True)
        return q

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = 30.0
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=300)
        r = res.result().result
        f = self.fingers()
        print(f'  gripper({width}): reached={r.reached_goal} stalled={r.stalled} fingers={f[0]:.4f},{f[1]:.4f}', flush=True)
        return f
OPENRUA_EOF

# openrua op 25
cat > step_check.py <<'EOF'
from rob import *
r = Robot('chk')
q_side = quat_from_axes([0,0,1],[1,0,0],[0,1,0])   # hand z -> +y, fingers along x, hand x up
print('q_side', np.round(q_side,4))
poses = {
 'pregrasp': ([-0.030, -0.22, 0.955], q_side),
 'grasp':    ([-0.030, -0.133, 0.955], q_side),
 'lift':     ([-0.030, -0.133, 1.05], q_side),
 'pre_mw':   ([-0.155, -0.52, 1.00], q_side),
 'in_mw':    ([-0.155, -0.403, 0.99], q_side),
}
seed = r.arm_q()
for k,(p,q) in poses.items():
    sol = r.ik(p, q, seed=seed)
    print(k, None if sol is None else np.round(sol,3))
    if sol is not None:
        fp, fq = r.fk(sol); print('   fk', np.round(fp,3), np.round(fq,3))
        seed = sol
EOF
timeout 600 python3 step_check.py

# openrua op 26
cat >> rob.py <<'EOF'


def quat_mul(a, b):
    x1, y1, z1, w1 = a
    x2, y2, z2, w2 = b
    return np.array([w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                     w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
                     w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2])


# IK on this machine solves for panda_link8; panda_hand = link8 rotated -45deg
# about z. Convert a desired HAND quaternion into the link8 quaternion to ask for.
Q_Z45 = np.array([0.0, 0.0, np.sin(np.pi / 8), np.cos(np.pi / 8)])


def hand_to_link8(q_hand):
    return quat_mul(np.asarray(q_hand, float), Q_Z45)


_orig_ik = Robot.ik


def _ik_hand(self, pos, quat, **kw):
    return _orig_ik(self, pos, hand_to_link8(quat), **kw)


Robot.ik = _ik_hand
EOF
cat > step_check.py <<'EOF'
from rob import *
r = Robot('chk')
q_side = quat_from_axes([0,0,1],[1,0,0],[0,1,0])   # hand z -> +y, fingers along x, hand x up
q_side2 = quat_from_axes([0,0,-1],[-1,0,0],[0,1,0])
for name, qs in [('x-up', q_side), ('x-down', q_side2)]:
    print('====', name, np.round(qs,4))
    poses = {
     'pregrasp': ([-0.030, -0.22, 0.955], qs),
     'grasp':    ([-0.030, -0.133, 0.955], qs),
     'lift':     ([-0.030, -0.133, 1.05], qs),
     'pre_mw':   ([-0.155, -0.52, 1.00], qs),
     'pre_mw2':  ([-0.155, -0.48, 1.00], qs),
     'in_mw':    ([-0.155, -0.403, 0.99], qs),
    }
    seed = r.arm_q()
    for k,(p,q) in poses.items():
        sol = r.ik(p, q, seed=seed)
        print(k, None if sol is None else np.round(sol,3))
        if sol is not None:
            fp, fq = r.fk(sol); print('   fk', np.round(fp,3), np.round(fq,3))
            seed = sol
EOF
timeout 900 python3 step_check.py

# openrua op 27
cat > view_check.py <<'EOF'
from rob import *
r = Robot('chk')
s=np.sqrt(0.5)
q_view45 = quat_from_axes([0,s,s],[1,0,0],[0,s,-s])   # hand z down-forward(+y) 45deg
q_view60 = quat_from_axes([0,np.cos(np.radians(30)),np.sin(np.radians(30))],[1,0,0],[0,np.sin(np.radians(30)),-np.cos(np.radians(30))])  # 60deg down
q_view30 = quat_from_axes([0,np.cos(np.radians(60)),np.sin(np.radians(60))],[1,0,0],[0,np.sin(np.radians(60)),-np.cos(np.radians(60))])  # 30deg down
cands = [
 ('v45 a', [-0.155,-0.50,1.05], q_view45), ('v45 b', [-0.155,-0.46,1.08], q_view45), ('v45 c', [-0.155,-0.44,1.10], q_view45),
 ('v60 a', [-0.155,-0.46,1.15], q_view60), ('v60 b', [-0.155,-0.42,1.20], q_view60),
 ('v30 a', [-0.155,-0.50,1.02], q_view30), ('v30 b', [-0.155,-0.46,1.03], q_view30),
]
seed = r.arm_q()
for name,p,q in cands:
    sol = r.ik(p,q,seed=seed)
    print(name, p, None if sol is None else np.round(sol,3), flush=True)
EOF
timeout 900 python3 view_check.py

# openrua op 28
cat >> rob.py <<'EOF'


def path_check(r, q0, q1, n=15, verbose=True):
    """FK-sample the straight joint-space path q0->q1; report hand & fingertip
    positions and flag intrusions into the microwave/door/table volumes."""
    bad = 0
    for i in range(n + 1):
        q = q0 + (q1 - q0) * i / n
        p, quat = r.fk(q)
        tip = p + quat_to_R(quat)[:, 2] * TCP
        flags = []
        for name, pt in (('hand', p), ('tip', tip)):
            x, y, z = pt
            if z < 0.93:
                flags.append(f'{name} below table')
            if -0.31 < x < 0.08 and -0.38 < y < -0.12 and z < 1.13:
                flags.append(f'{name} in microwave box')
            if -0.48 < x < -0.28 and -0.62 < y < -0.34 and z < 1.13:
                flags.append(f'{name} in door zone')
        bad += bool(flags)
        if verbose or flags:
            print(f'   {i:2d} hand {np.round(p,3)} tip {np.round(tip,3)} {" ".join(flags)}', flush=True)
    return bad
EOF
cat > step1.py <<'EOF'
from rob import *
r = Robot('s1')
s=np.sqrt(0.5)
q_view45 = quat_from_axes([0,s,s],[1,0,0],[0,s,-s])
q0 = r.arm_q()
q1 = r.ik([-0.155,-0.50,1.05], q_view45, seed=q0)
print('target q', np.round(q1,3))
print('bad steps:', path_check(r, q0, q1))
EOF
timeout 600 python3 step1.py

# openrua op 29
cat > step1.py <<'EOF'
from rob import *
r = Robot('s1')
s=np.sqrt(0.5)
q_view45 = quat_from_axes([0,s,s],[1,0,0],[0,s,-s])
q0 = r.arm_q()
qa = r.ik([-0.155,-0.40,1.32], q_view45, seed=q0)
print('via q', np.round(qa,3)); print('bad A:', path_check(r, q0, qa, verbose=False))
qb = r.ik([-0.155,-0.50,1.05], q_view45, seed=qa)
print('target q', np.round(qb,3)); print('bad B:', path_check(r, qa, qb, verbose=False))
import sys
if '--go' in sys.argv:
    r.move_q(qa, 4.0)
    r.move_q(qb, 3.0)
    print('fk', r.fk())
EOF
timeout 600 python3 step1.py

# openrua op 30
timeout 900 python3 step1.py --go && timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 31
python3 -c "
from rob import *
r=Robot('c'); q=r.arm_q(); print(np.round(q,3)); print(r.fk())
print(np.round(q-np.array([-0.762,1.154,-0.178,-0.95,0.63,1.43,-1.803]),3))
"

# openrua op 32
python3 -c "
from rob import *
r=Robot('c')
qb=np.array([-0.762,1.154,-0.178,-0.95,0.63,1.43,-1.803])
r.move_q(qb, 4.0)
print(np.round(r.arm_q()-qb,3)); print(r.fk())
"

# openrua op 33
python3 -c "
from rob import *
r=Robot('c')
qb=np.array([-0.762,1.154,-0.178,-0.95,0.63,1.43,-1.803])
r.move_q(qb, 4.0)
print(np.round(r.arm_q()-qb,3)); print(r.fk())
"

# openrua op 34
timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 35
timeout 120 python3 cloud.py robot0_eye_in_hand && python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_cloud.npy'); 
Q=P.reshape(-1,3); Q=Q[np.isfinite(Q).all(1)&(Q[:,2]>0.5)]
print('cam pts', len(Q))
# points inside microwave footprint
inside = Q[(Q[:,0]>-0.29)&(Q[:,0]<0.06)&(Q[:,1]>-0.36)&(Q[:,1]<-0.14)]
print('inside footprint: z range', inside[:,2].min(), inside[:,2].max())
h=np.histogram(inside[:,2],bins=np.arange(0.88,1.12,0.01)); print([(round(e,2),int(n)) for e,n in zip(h[1],h[0])])
# floor: z between 0.9 and 0.96 -> x,y extent
for lo,hi in [(0.90,0.925),(0.925,0.95),(0.95,0.98)]:
    f=inside[(inside[:,2]>lo)&(inside[:,2]<hi)]
    if len(f): print('z',lo,hi,'n',len(f),'x',f[:,0].min(),f[:,0].max(),'y',f[:,1].min(),f[:,1].max())
# back wall: y max of points
bw = inside[(inside[:,2]>0.95)&(inside[:,2]<1.05)]
print('walls z.95-1.05: y max', bw[:,1].max(), 'x range', bw[:,0].min(), bw[:,0].max())
# ceiling
ce = inside[inside[:,2]>1.05]; print('ceiling pts', len(ce), 'z', ce[:,2].min() if len(ce) else None, ce[:,2].max() if len(ce) else None)
"

# openrua op 36
python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_cloud.npy'); Q=P.reshape(-1,3); Q=Q[np.isfinite(Q).all(1)&(Q[:,2]>0.5)]
fl = Q[(Q[:,2]>0.935)&(Q[:,2]<0.95)&(Q[:,1]>-0.5)&(Q[:,1]<-0.1)]
print('floor z median', np.median(fl[:,2]))
h=np.histogram(fl[:,0],bins=np.arange(-0.35,0.1,0.01)); print('floor x hist', [(round(e,2),int(n)) for e,n in zip(h[1],h[0]) if n>0])
h=np.histogram(fl[:,1],bins=np.arange(-0.5,-0.1,0.01)); print('floor y hist', [(round(e,2),int(n)) for e,n in zip(h[1],h[0]) if n>0])
# side walls: points with x near constant for y in [-0.33,-0.2], z in [0.95,1.05]
w = Q[(Q[:,1]>-0.33)&(Q[:,1]<-0.2)&(Q[:,2]>0.95)&(Q[:,2]<1.05)]
h=np.histogram(w[:,0],bins=np.arange(-0.4,0.1,0.01)); print('wall x hist', [(round(e,2),int(n)) for e,n in zip(h[1],h[0]) if n>100])
# ceiling: points with z>1.04 inside x range
c = Q[(Q[:,1]>-0.36)&(Q[:,1]<-0.17)&(Q[:,0]>-0.28)&(Q[:,0]<-0.03)&(Q[:,2]>1.03)]
h=np.histogram(c[:,2],bins=np.arange(1.03,1.12,0.005)); print('ceiling z hist', [(round(e,3),int(n)) for e,n in zip(h[1],h[0]) if n>0])
# front face / lip: points at y ~ -0.36 .. -0.37 : z range
lip = Q[(Q[:,1]>-0.375)&(Q[:,1]<-0.355)&(Q[:,0]>-0.28)&(Q[:,0]<-0.03)]
h=np.histogram(lip[:,2],bins=np.arange(0.88,1.12,0.01)); print('front plane z hist', [(round(e,2),int(n)) for e,n in zip(h[1],h[0]) if n>0])
"

# openrua op 37
cat > reach_scan.py <<'EOF'
from rob import *
r = Robot('scan')
q_side = quat_from_axes([0,0,1],[1,0,0],[0,1,0])
seed = r.arm_q()
print('hand +y horizontal, z=1.00 ; rows x, cols y')
ys = np.arange(-0.40,-0.66,-0.04)
print('      '+' '.join(f'{y:+.2f}' for y in ys))
for x in [-0.30,-0.25,-0.20,-0.155,-0.10,-0.05]:
    row=[]
    for y in ys:
        sol = r.ik([x,y,1.0], q_side, seed=seed, timeout=1.0, attempts=2)
        row.append('  ok ' if sol is not None else '  -- ')
    print(f'{x:+.3f}', ' '.join(row), flush=True)
EOF
timeout 1500 python3 reach_scan.py

# openrua op 38
cat > reach_scan2.py <<'EOF'
from rob import *
r = Robot('scan')
seed = r.arm_q()
center = np.array([-0.16,-0.27,1.0])
for psi in [15,30,45,60]:
    d = np.array([np.sin(np.radians(psi)), np.cos(np.radians(psi)), 0])
    hx = np.array([0,0,1.0]); hy = np.cross(d, hx)  # hand_y = z x x
    q = quat_from_axes(hx, hy, d)
    row=[]
    for s in [0.20,0.25,0.30,0.35,0.40,0.45,0.50,0.55]:
        p = center - s*d
        sol = r.ik(p, q, seed=seed, timeout=1.0, attempts=2)
        row.append(f's={s:.2f}@({p[0]:+.2f},{p[1]:+.2f}):' + ('ok' if sol is not None else '--'))
    print(f'psi={psi}', ' '.join(row), flush=True)
EOF
timeout 1500 python3 reach_scan2.py

# openrua op 39
cat > plan_check.py <<'EOF'
from rob import *
r = Robot('plan')
def Q(psi_deg):
    d = np.array([np.sin(np.radians(psi_deg)), np.cos(np.radians(psi_deg)), 0.0])
    hx = np.array([0,0,1.0]); hy = np.cross(d, hx)
    return quat_from_axes(hx, hy, d), d
BAR = np.array([-0.030, -0.038])   # handle outer bar centre (x,y)
PINCH_Z = 0.96
def hand_for_mug(cx, cy, psi, z):
    q, d = Q(psi); c = np.array([cx, cy, 0]); return c - 0.168*d + np.array([0,0,z]), q
wps = [
 ('pregrasp', [-0.030, -0.235, PINCH_Z], Q(0)[0]),
 ('grasp',    [-0.030, -0.131, PINCH_Z], Q(0)[0]),
 ('lift',     [-0.030, -0.131, 1.03], Q(0)[0]),
 ('high',     [-0.030, -0.131, 1.25], Q(0)[0]),
]
for name,(cx,cy,psi,z) in [('over_a',(-0.16,-0.43,45,1.25)),('pre_ins',(-0.16,-0.43,45,1.006)),
                            ('ins1',(-0.16,-0.38,35,1.006)),('ins2',(-0.16,-0.33,22,1.006)),
                            ('ins3',(-0.16,-0.29,10,1.006)),('ins4',(-0.16,-0.25,0,1.006)),('ins5',(-0.16,-0.23,0,1.006)),
                            ('lower',(-0.16,-0.23,0,0.997)),('retreat',(-0.16,-0.33,0,0.997))]:
    p,q = hand_for_mug(cx,cy,psi,z); wps.append((name, p, q))
seed = r.arm_q(); sols = {}
for name,p,q in wps:
    sol = r.ik(p,q,seed=seed, timeout=2.0)
    print(f'{name:9s} hand {np.round(p,3)} ->', None if sol is None else np.round(sol,3), flush=True)
    if sol is not None: seed = sol; sols[name]=sol
np.save('plan_sols.npy', sols, allow_pickle=True)
EOF
timeout 1500 python3 plan_check.py

# openrua op 40
cat >> rob.py <<'EOF'


def move_q_retry(r, q, seconds=3.0, tries=4, tol=0.01):
    for i in range(tries):
        code = r.move_q(q, seconds)
        err = np.abs(r.arm_q() - np.asarray(q)).max()
        if err < tol:
            return True
        print(f'  retry {i+1}: max_joint_err={err:.3f}', flush=True)
    return False


def Q_yaw(psi_deg):
    """Hand pointing horizontally along d=(sin psi, cos psi, 0), camera (hand x) up."""
    d = np.array([np.sin(np.radians(psi_deg)), np.cos(np.radians(psi_deg)), 0.0])
    hx = np.array([0, 0, 1.0]); hy = np.cross(d, hx)
    return quat_from_axes(hx, hy, d), d
EOF
cat > phase1.py <<'EOF'
import sys
from rob import *
r = Robot('p1')
q0 = r.arm_q(); print('start q', np.round(q0,3)); print('hand', np.round(r.fk()[0],3))
qy0,_ = Q_yaw(0)
PZ = 0.96
targets = [('high_view', [-0.155,-0.50,1.30], None),   # straight up from view pose (same orientation)
           ('high', [-0.03,-0.131,1.25], qy0),
           ('pregrasp', [-0.03,-0.235,PZ], qy0),
           ('grasp', [-0.03,-0.131,PZ], qy0)]
# current orientation for the first lift
_, qcur = r.fk()
sols = []; seed = q0
for name,p,q in targets:
    if q is None: q = qcur
    s = r.ik(p, q, seed=seed, timeout=2.0)
    print(name, np.round(p,3), '->', None if s is None else np.round(s,3), flush=True)
    if s is None: sys.exit('IK fail')
    print('  bad steps:', path_check(r, seed, s, verbose=False), flush=True)
    sols.append((name,s)); seed = s
if '--go' not in sys.argv: sys.exit(0)
for name,s in sols:
    print('>>', name, flush=True)
    ok = move_q_retry(r, s, 3.0 if name!='grasp' else 2.5)
    print('   reached' if ok else '   NOT reached', 'hand', np.round(r.fk()[0],4), flush=True)
    if not ok: sys.exit('move failed')
print('closing gripper'); r.gripper(0.0)
print('fingers', r.fingers())
EOF
timeout 1500 python3 phase1.py

# openrua op 41
cat >> rob.py <<'EOF'


def ik_near(r, pos, quat, seed, tries=6, timeout=1.0, max_dist=None):
    """IK, several attempts, return the solution closest (L-inf) to seed."""
    best = None
    for _ in range(tries):
        s = _orig_ik(r, pos, hand_to_link8(quat), seed=seed, timeout=timeout, attempts=1)
        if s is None:
            continue
        d = np.abs(s - seed).max()
        if best is None or d < best[0]:
            best = (d, s)
        if d < 0.5:
            break
    if best is None:
        return None
    if max_dist is not None and best[0] > max_dist:
        print(f'  ik_near: nearest solution is far (dist {best[0]:.2f})', flush=True)
    return best[1]
EOF
cat > phase1.py <<'EOF'
import sys
from rob import *
r = Robot('p1')
q0 = r.arm_q(); p0, qcur = r.fk(); print('start q', np.round(q0,3), 'hand', np.round(p0,3))
MUG = np.array([-0.029, 0.039]); R_MUG = 0.047
q_down_a = quat_from_axes([0,-1,0],[-1,0,0],[0,0,-1])   # hand z down, fingers along x
q_down_b = quat_from_axes([0,1,0],[1,0,0],[0,0,-1])
gx, gy = MUG[0] + R_MUG, MUG[1]            # straddle wall at +x point
DROP = np.array([-0.15, 0.20])             # where to set the mug down (open table)
Z_GRASP = 1.082                            # fingertips ~3cm below rim (rim 1.006)
plan = [('up',       [-0.155,-0.50,1.30], qcur),
        ('over_mug', [gx, gy, 1.25], None),
        ('descend',  [gx, gy, Z_GRASP], None),
        ('CLOSE',),
        ('lift',     [gx, gy, 1.22], None),
        ('over_drop',[DROP[0]+R_MUG, DROP[1], 1.22], None),
        ('setdown',  [DROP[0]+R_MUG, DROP[1], Z_GRASP+0.004], None),
        ('OPEN',),
        ('lift2',    [DROP[0]+R_MUG, DROP[1], 1.20], None)]
sols = []; seed = q0; qd = None
for step in plan:
    if len(step) == 1: sols.append(step); continue
    name, p, q = step
    if q is None:
        if qd is None:
            ca = ik_near(r, p, q_down_a, seed); cb = ik_near(r, p, q_down_b, seed)
            cands = [(np.abs(c-seed).max(), c, qq) for c, qq in ((ca,q_down_a),(cb,q_down_b)) if c is not None]
            if not cands: sys.exit(f'IK fail {name}')
            cands.sort(key=lambda t: t[0]); s, qd = cands[0][1], cands[0][2]
            print('  chose orientation', 'a' if qd is q_down_a else 'b')
        else:
            s = ik_near(r, p, qd, seed)
    else:
        s = ik_near(r, p, q, seed)
    if s is None: sys.exit(f'IK fail {name}')
    print(f'{name:9s} {np.round(p,3)} -> {np.round(s,3)}  dist {np.abs(s-seed).max():.2f}', flush=True)
    print('  bad steps:', path_check(r, seed, s, verbose=False), flush=True)
    sols.append((name, s)); seed = s
if '--go' not in sys.argv: sys.exit(0)
for step in sols:
    if step[0] == 'CLOSE':
        r.gripper(0.0); f = r.fingers(); print('   fingers after close', f, flush=True); continue
    if step[0] == 'OPEN':
        r.gripper(0.04); continue
    name, s = step
    print('>>', name, flush=True)
    ok = move_q_retry(r, s, 3.0)
    print('   reached' if ok else '   NOT reached', 'hand', np.round(r.fk()[0],4), flush=True)
    if not ok: sys.exit('move failed')
    if name in ('lift',):
        f = r.fingers(); print('   fingers while lifted', f, flush=True)
        if f[0] < 0.002: sys.exit('grasp lost')
EOF
timeout 1500 python3 phase1.py

# openrua op 42
sed -i 's/^Z_GRASP = 1.082 .*/Z_GRASP = 1.072                            # fingertips ~3.7cm below rim (rim 1.006)/' phase1.py && timeout 1700 python3 phase1.py --go 2>&1 | tee phase1.log

# openrua op 43
timeout 120 python3 cloud.py birdview && timeout 120 python3 tools/perception/cam_snap.py agentview && python3 -c "
import numpy as np
B=np.load('birdview_cloud.npy').reshape(-1,3); B=B[np.isfinite(B).all(1)]
rim = B[(B[:,2]>0.985)&(B[:,2]<1.02)&(B[:,0]>-0.3)&(B[:,0]<0.1)&(B[:,1]>0.05)&(B[:,1]<0.3)]
print('rim pts',len(rim),'x',rim[:,0].min(),rim[:,0].max(),'y',rim[:,1].min(),rim[:,1].max())
body = rim[rim[:,1]>rim[:,1].min()+0.03]
x,y=body[:,0],body[:,1]; A=np.c_[2*x,2*y,np.ones_like(x)]; b=x**2+y**2
c=np.linalg.lstsq(A,b,rcond=None)[0]; cx,cy=c[0],c[1]; r=np.sqrt(c[2]+cx**2+cy**2)
print('circle center %.4f %.4f radius %.4f'%(cx,cy,r))
h = B[(B[:,2]>0.93)&(B[:,2]<0.995)&(B[:,0]>cx-0.03)&(B[:,0]<cx+0.03)&(B[:,1]>cy-0.1)&(B[:,1]<cy-0.045)]
print('handle pts', len(h), 'x', h[:,0].min(), h[:,0].max(), 'y', h[:,1].min(), h[:,1].max(), 'ztop', h[:,2].max())
"

# openrua op 44
python3 -c "
from rob import *
r=Robot('m'); q0=r.arm_q(); _,qc=r.fk()
s=ik_near(r,[-0.30,-0.05,1.40],qc,q0); print(np.round(s,3)); print('bad',path_check(r,q0,s,verbose=False))
move_q_retry(r,s,3.0); print(r.fk()[0])
" && timeout 120 python3 cloud.py birdview && python3 -c "
import numpy as np
B=np.load('birdview_cloud.npy').reshape(-1,3); B=B[np.isfinite(B).all(1)]
rim = B[(B[:,2]>0.985)&(B[:,2]<1.02)&(B[:,0]>-0.3)&(B[:,0]<0.1)&(B[:,1]>0.05)&(B[:,1]<0.3)]
print('rim pts',len(rim),'x',rim[:,0].min(),rim[:,0].max(),'y',rim[:,1].min(),rim[:,1].max())
body = rim[rim[:,1]>rim[:,1].min()+0.03]
x,y=body[:,0],body[:,1]; A=np.c_[2*x,2*y,np.ones_like(x)]; b=x**2+y**2
c=np.linalg.lstsq(A,b,rcond=None)[0]; cx,cy=c[0],c[1]; r=np.sqrt(c[2]+cx**2+cy**2)
print('circle center %.4f %.4f radius %.4f'%(cx,cy,r))
h = B[(B[:,2]>0.93)&(B[:,2]<0.995)&(B[:,0]>cx-0.03)&(B[:,0]<cx+0.03)&(B[:,1]>cy-0.1)&(B[:,1]<cy-0.045)]
print('handle pts', len(h), 'x', h[:,0].min(), h[:,0].max(), 'y', h[:,1].min(), h[:,1].max(), 'ztop', h[:,2].max())
for p in sorted(h.tolist(), key=lambda p:p[1])[:12]: print('  %.3f %.3f %.3f'%tuple(p))
"

# openrua op 45
python3 -c "
import numpy as np, cv2
B=np.load('birdview_cloud.npy'); img=cv2.imread('birdview.png')
# find pixel region for world x in [-0.25,0.05], y in [0.05,0.35]
X=B[...,0]; Y=B[...,1]; Z=B[...,2]
m=(X>-0.25)&(X<0.05)&(Y>0.05)&(Y<0.35)
vs,us=np.where(m); print('pixel box v',vs.min(),vs.max(),'u',us.min(),us.max())
for v in range(vs.min(), vs.max()+1, 3):
    print(f'{v:3d}', ''.join(('#' if Z[v,u]>0.98 else ('+' if Z[v,u]>0.95 else ('-' if Z[v,u]>0.91 else '.'))) for u in range(us.min(), us.max()+1, 2)))
print('u->y:', B[vs.min(),us.min(),1], B[vs.min(),us.max(),1], ' v->x:', B[vs.min(),us.min(),0], B[vs.max(),us.min(),0])
"

# openrua op 46
python3 -c "
import numpy as np
B=np.load('birdview_cloud.npy'); Z=B[...,2]
sub=B[243:272, 349:372]; z=sub[...,2]
pts=sub[(z>0.985)&(z<1.02)].reshape(-1,3)
x,y=pts[:,0],pts[:,1]; A=np.c_[2*x,2*y,np.ones_like(x)]; b=x**2+y**2
c=np.linalg.lstsq(A,b,rcond=None)[0]; cx,cy=c[0],c[1]; r=np.sqrt(c[2]+cx**2+cy**2)
print('n',len(pts),'circle center %.4f %.4f radius %.4f'%(cx,cy,r), 'rim z', pts[:,2].mean())
h=B[250:266, 340:352].reshape(-1,3); h=h[h[:,2]>0.93]
print('handle: x',h[:,0].min(),h[:,0].max(),'y',h[:,1].min(),h[:,1].max(),'z',h[:,2].min(),h[:,2].max())
hb=h[h[:,2]>0.98]; print('handle top: x %.4f..%.4f y %.4f..%.4f'%(hb[:,0].min(),hb[:,0].max(),hb[:,1].min(),hb[:,1].max()))
"

# openrua op 47
python3 -c "
import numpy as np
B=np.load('birdview_cloud.npy')
sub=B[243:272, 366:396]; z=sub[...,2]
pts=sub[(z>0.985)&(z<1.02)].reshape(-1,3)
x,y=pts[:,0],pts[:,1]; A=np.c_[2*x,2*y,np.ones_like(x)]; b=x**2+y**2
c=np.linalg.lstsq(A,b,rcond=None)[0]; cx,cy=c[0],c[1]; r=np.sqrt(c[2]+cx**2+cy**2)
print('n',len(pts),'circle center %.4f %.4f radius %.4f'%(cx,cy,r), 'rim z', pts[:,2].mean())
h=B[250:266, 350:368].reshape(-1,3); h=h[(h[:,2]>0.93)&(h[:,1]<cy-0.046)]
print('handle: x',h[:,0].min(),h[:,0].max(),'y',h[:,1].min(),h[:,1].max(),'z',h[:,2].min(),h[:,2].max())
hb=h[h[:,2]>0.98]; print('handle top: x %.4f..%.4f y %.4f..%.4f'%(hb[:,0].min(),hb[:,0].max(),hb[:,1].min(),hb[:,1].max()))
"

# openrua op 48
timeout 120 python3 cloud.py agentview && python3 -c "
import numpy as np
A=np.load('agentview_cloud.npy').reshape(-1,3); A=A[np.isfinite(A).all(1)]
m=A[(A[:,0]>-0.2)&(A[:,0]<-0.08)&(A[:,1]>0.10)&(A[:,1]<0.26)&(A[:,2]>0.905)]
print('mug z', m[:,2].min(), m[:,2].max())
for lo in [0.91,0.93,0.95,0.97,0.99]:
    s=m[(m[:,2]>lo)&(m[:,2]<lo+0.02)&(m[:,1]>0.155)]
    if len(s): print(f'z {lo}: x {s[:,0].min():.3f}..{s[:,0].max():.3f}  y {s[:,1].min():.3f}..{s[:,1].max():.3f}')
hd=m[(m[:,1]<0.152)]
print('handle from agentview: z', hd[:,2].min(), hd[:,2].max(), 'y', hd[:,1].min(), hd[:,1].max(), 'x', hd[:,0].min(), hd[:,0].max())
"

# openrua op 49
cat > phase2.py <<'EOF'
import sys
from rob import *
r = Robot('p2')
q0 = r.arm_q(); p0, qcur = r.fk(); print('start q', np.round(q0,3), 'hand', np.round(p0,3))
BAR = np.array([-0.138, 0.120]); PZ = 0.958
qy0, _ = Q_yaw(0)
plan = [('pregrasp', [BAR[0], BAR[1]+0.01-TCP-0.08, PZ], qy0),
        ('grasp',    [BAR[0], BAR[1]+0.01-TCP, PZ], qy0),
        ('CLOSE',),
        ('lift',     [BAR[0], BAR[1]+0.01-TCP, 1.03], qy0)]
sols=[]; seed=q0
for step in plan:
    if len(step)==1: sols.append(step); continue
    name,p,q = step
    s = ik_near(r, p, q, seed)
    if s is None: sys.exit(f'IK fail {name}')
    print(f'{name:9s} {np.round(p,3)} -> {np.round(s,3)}  dist {np.abs(s-seed).max():.2f}', flush=True)
    print('  bad steps:', path_check(r, seed, s, verbose=False), flush=True)
    sols.append((name,s)); seed=s
if '--go' not in sys.argv: sys.exit(0)
for step in sols:
    if step[0]=='CLOSE':
        r.gripper(0.0); print('   fingers after close', r.fingers(), flush=True); continue
    name,s = step
    print('>>', name, flush=True)
    ok = move_q_retry(r, s, 3.0)
    print('   reached' if ok else '   NOT reached', 'hand', np.round(r.fk()[0],4), flush=True)
    if not ok: sys.exit('move failed')
print('fingers after lift', r.fingers())
EOF
timeout 1500 python3 phase2.py

# openrua op 50
python3 -c "
from rob import *
r=Robot('t'); qy0,_=Q_yaw(0)
for seed in [np.array([-0.627,1.387,0.165,-0.94,1.037,1.247,-1.578]), np.array([0,-0.5,0,-2.0,0,1.5,0.785]), np.array([0.3,0.8,0.0,-1.2,0.5,1.5,-1.5])]:
    for p in [[-0.138,-0.053,0.958],[-0.138,0.027,0.958],[-0.138,0.027,1.03]]:
        s=ik_near(r,p,qy0,seed,tries=8)
        print(np.round(seed,2), p, None if s is None else np.round(s,3))
"

# openrua op 51
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/phase2.py", "old_string": "sols=[]; seed=q0", "new_string": "SEED = np.array([-0.627,1.387,0.165,-0.94,1.037,1.247,-1.578])  # phase-1 grasp branch\\nplan.insert(0, (\'high\', [BAR[0], BAR[1]+0.01-TCP-0.08, 1.15], qy0))\\nsols=[]; seed=SEED; prev=q0", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 52
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/phase2.py", "old_string": "    s = ik_near(r, p, q, seed)\\n    if s is None: sys.exit(f\'IK fail {name}\')\\n    print(f\'{name:9s} {np.round(p,3)} -> {np.round(s,3)}  dist {np.abs(s-seed).max():.2f}\', flush=True)\\n    print(\'  bad steps:\', path_check(r, seed, s, verbose=False), flush=True)\\n    sols.append((name,s)); seed=s", "new_string": "    s = ik_near(r, p, q, seed, tries=8)\\n    if s is None: sys.exit(f\'IK fail {name}\')\\n    print(f\'{name:9s} {np.round(p,3)} -> {np.round(s,3)}  dist {np.abs(s-prev).max():.2f}\', flush=True)\\n    print(\'  bad steps:\', path_check(r, prev, s, verbose=False), flush=True)\\n    sols.append((name,s)); seed=s; prev=s", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 53
timeout 1500 python3 phase2.py 2>&1 | tail -20

# openrua op 54
timeout 1700 python3 -u phase2.py --go 2>&1 | tee phase2.log | tail -30

# openrua op 55
python3 -c "
from rob import *
r=Robot('t'); q=r.arm_q(); print('q',np.round(q,3)); print('target diff', np.round(q-np.array([-0.504,0.939,0.203,-1.858,1.156,1.582,-1.967]),3)); print('wrench',np.round(r.wrench(),2)); p,qq=r.fk(); print(p, quat_to_R(qq)[:,2])
"; ros2 run tf2_ros tf2_echo world panda_link7 2>/dev/null | head -4; ros2 run tf2_ros tf2_echo world panda_link6 2>/dev/null | head -4

# openrua op 56
python3 -c "
from rob import *
from moveit_msgs.srv import GetPositionFK
import rclpy
r=Robot('t')
def fkl(q, links=('panda_link4','panda_link5','panda_link6','panda_link7','panda_hand')):
    req=GetPositionFK.Request(); req.fk_link_names=list(links)
    req.robot_state.joint_state.name=ARM; req.robot_state.joint_state.position=list(map(float,q))
    f=r.fk_cli.call_async(req); rclpy.spin_until_future_complete(r.node,f,timeout_sec=60)
    return {l:np.round([p.pose.position.x,p.pose.position.y,p.pose.position.z],3) for l,p in zip(links,f.result().pose_stamped)}
for name,q in [('grasp',[-0.424,0.923,0.277,-1.926,1.26,1.695,-1.997]),('lift',[-0.451,0.734,0.306,-2.04,1.288,1.669,-1.928]),('pregrasp',[-0.504,0.939,0.203,-1.858,1.156,1.582,-1.967])]:
    print(name, fkl(q))
"

# openrua op 57
sed -e "s/MUG = np.array(\[-0.029, 0.039\])/MUG = np.array([-0.1409, 0.1993])/" -e "s/DROP = np.array(\[-0.15, 0.20\])/DROP = np.array([-0.22, 0.32])/" -e "s/plan = \[('up',       \[-0.155,-0.50,1.30\], qcur),/plan = [('up', None, np.array([-0.546,0.468,0.239,-2.084,1.237,1.469,-1.754])),/" -e "s/Robot('p1')/Robot('p1b')/" phase1.py > phase1b.py
python3 - <<'EOF'
s=open('/workspace/phase1b.py').read()
s=s.replace("""    name, p, q = step
    if q is None:""","""    name, p, q = step
    if p is None:
        s = q
    elif q is None:""")
open('/workspace/phase1b.py','w').write(s)
EOF
grep -n "MUG =\|DROP =\|'up'\|p is None" phase1b.py

# openrua op 58
sed -i "s/print(f'{name:9s} {np.round(p,3)} -> /print(f'{name:9s} {p if p is None else np.round(p,3)} -> /" phase1b.py && timeout 1500 python3 -u phase1b.py 2>&1 | tail -20

# openrua op 59
python3 - <<'EOF'
s=open('/workspace/phase1b.py').read()
s=s.replace("""plan = [('up', None, np.array([-0.546,0.468,0.239,-2.084,1.237,1.469,-1.754])),""",
"""qy0,_ = Q_yaw(0)
plan = [('up', None, np.array([-0.546,0.468,0.239,-2.084,1.237,1.469,-1.754])),
        ('up2', [-0.138, 0.0, 1.32], qy0),""")
open('/workspace/phase1b.py','w').write(s)
EOF
timeout 1500 python3 -u phase1b.py 2>&1 | grep -v "bad steps: 0"

# openrua op 60
timeout 1700 python3 -u phase1b.py --go 2>&1 | tee phase1b.log | grep -v "bad steps: 0\|^  move_q\|retry"

# openrua op 61
python3 -c "
from rob import *
r=Robot('t'); q0=r.arm_q(); p,qc=r.fk()
s=ik_near(r,[-0.45,0.10,1.38],qc,q0,tries=8); print(np.round(s,3), np.abs(s-q0).max()); print(path_check(r,q0,s,verbose=False)); print(move_q_retry(r,s,3.0)); print(r.fk()[0])
" && python3 cloud.py birdview && python3 -c "
import numpy as np
P=np.load('birdview_cloud.npy'); X,Y,Z=P[...,0],P[...,1],P[...,2]
m=(X>-0.32)&(X<-0.12)&(Y>0.22)&(Y<0.42)&(Z>0.985)&(Z<1.02)
pts=P[m][:,:2]; print('n',len(pts))
A=np.c_[2*pts,np.ones(len(pts))]; b=(pts**2).sum(1); c=np.linalg.lstsq(A,b,rcond=None)[0]; cx,cy=c[:2]; rr=np.sqrt(c[2]+cx**2+cy**2); print('center',cx,cy,'r',rr)
m2=(X>-0.32)&(X<-0.12)&(Y>0.15)&(Y<0.42)&(Z>0.92)&(Z<0.99)&(Y<cy-0.05)
print('handle pts', m2.sum(), P[m2].mean(0) if m2.sum() else None, P[m2][:,0].min() if m2.sum() else None, P[m2][:,0].max() if m2.sum() else None, P[m2][:,1].min() if m2.sum() else None)
"

# openrua op 62
python3 -c "
import numpy as np
P=np.load('birdview_cloud.npy'); X,Y,Z=P[...,0],P[...,1],P[...,2]
for zlo,zhi in [(0.985,1.02),(0.995,1.02),(0.99,1.01)]:
    m=(X>-0.32)&(X<-0.12)&(Y>0.27)&(Y<0.42)&(Z>zlo)&(Z<zhi)
    pts=P[m][:,:2]
    A=np.c_[2*pts,np.ones(len(pts))]; b=(pts**2).sum(1); c=np.linalg.lstsq(A,b,rcond=None)[0]; cx,cy=c[:2]; rr=np.sqrt(c[2]+cx**2+cy**2)
    res=np.hypot(pts[:,0]-cx,pts[:,1]-cy)-rr
    print(zlo,zhi,'n',len(pts),'center %.4f %.4f r %.4f rms %.4f'%(cx,cy,rr,res.std()))
m=(X>-0.32)&(X<-0.12)&(Y>0.2)&(Y<0.42)&(Z>0.92)
print('extent x',X[m].min(),X[m].max(),'y',Y[m].min(),Y[m].max(),'zmax',Z[m].max())
# handle bar: points with y<0.25
mh=m&(Y<0.25)
for yl in np.arange(0.22,0.26,0.005):
    mm=mh&(Y>=yl)&(Y<yl+0.005)
    if mm.sum(): print('y %.3f n %d x %.3f..%.3f z %.3f..%.3f'%(yl,mm.sum(),X[mm].min(),X[mm].max(),Z[mm].min(),Z[mm].max()))
"

# openrua op 63
sed -i "s/BAR = np.array(\[-0.138, 0.120\])/BAR = np.array([-0.208, 0.242])/" phase2.py && timeout 1500 python3 -u phase2.py 2>&1 | grep -v "bad steps: 0"

# openrua op 64
timeout 1700 python3 -u phase2.py --go 2>&1 | tee phase2.log | grep -v "bad steps: 0\|^  move_q\|retry\|XMLPARSER"

# openrua op 65
python3 cloud.py birdview >/dev/null 2>&1; python3 cloud.py agentview 2>&1 | grep -v XMLPARSER; python3 -c "
import numpy as np
P=np.load('birdview_cloud.npy'); X,Y,Z=P[...,0],P[...,1],P[...,2]
m=(X>-0.32)&(X<-0.12)&(Y>0.26)&(Y<0.42)&(Z>0.92)
print('mug region zmax',Z[m].max(),'n',m.sum())
for zlo in [1.0,1.03,1.05,1.06,1.07,1.08]:
    mm=m&(Z>zlo)&(Z<zlo+0.01)
    if mm.sum(): print(zlo, mm.sum(), 'x %.3f..%.3f y %.3f..%.3f'%(X[mm].min(),X[mm].max(),Y[mm].min(),Y[mm].max()))
"

# openrua op 66
mkdir -p "$(dirname /workspace/phase3.py)"
cat > /workspace/phase3.py <<'OPENRUA_EOF'
import sys
from rob import *
from moveit_msgs.srv import GetPositionFK

r = Robot('p3')
q0 = r.arm_q(); p0, qcur = r.fk(); print('start q', np.round(q0, 3), 'hand', np.round(p0, 3))

L = 0.172                      # mug centre is this far in front of the hand (along d)
CX = -0.14                     # mug track x inside the cavity
Z_IN = 1.006                   # hand z while inserting (rim ~1.054, bottom ~0.955)
Z_LOW = 0.997                  # set-down (bottom ~0.946 vs floor 0.944)
HINGE = np.array([-0.29, -0.36]); ANG = np.radians(236)
U = np.array([np.cos(ANG), np.sin(ANG)]); N = np.array([-np.sin(ANG), np.cos(ANG)])   # N -> (+x,-y) side
if N[0] < 0: N = -N


def hand_for(cy, psi, z):
    q, d = Q_yaw(psi)
    c = np.array([CX, cy, 0.0])
    return list(c - L * d + np.array([0, 0, z])), q


def fk_links(q, links=('panda_link5', 'panda_link6', 'panda_link7', 'panda_hand')):
    req = GetPositionFK.Request(); req.fk_link_names = list(links)
    req.robot_state.joint_state.name = ARM; req.robot_state.joint_state.position = list(map(float, q))
    f = r.fk_cli.call_async(req); rclpy.spin_until_future_complete(r.node, f, timeout_sec=60)
    out = {}
    for l, p in zip(links, f.result().pose_stamped):
        pp = p.pose
        out[l] = (np.array([pp.position.x, pp.position.y, pp.position.z]),
                  np.array([pp.orientation.x, pp.orientation.y, pp.orientation.z, pp.orientation.w]))
    return out


def door_clearance(q):
    """min signed distance (m) of hand ends / wrist links to the open door plane,
    only for points within the door's radial extent (<0.30 from hinge) and below z 1.12."""
    lk = fk_links(q)
    hp, hq = lk['panda_hand']; R = quat_to_R(hq)
    pts = [('hand+', hp + 0.10 * R[:, 1], 0.0), ('hand-', hp - 0.10 * R[:, 1], 0.0),
           ('tip+', hp + 0.10 * R[:, 1] + TCP * R[:, 2], 0.0), ('tip-', hp - 0.10 * R[:, 1] + TCP * R[:, 2], 0.0),
           ('l7', lk['panda_link7'][0], 0.05), ('l6', lk['panda_link6'][0], 0.06), ('l5', lk['panda_link5'][0], 0.06)]
    worst = (9, '')
    for name, p, rad in pts:
        rel = p[:2] - HINGE
        along = rel @ U; dist = rel @ N - rad
        if p[2] < 1.12 and -0.05 < along < 0.30 and dist < worst[0]:
            worst = (dist, name)
    return worst


# mug-centre waypoints (cy, psi)
WPS = [(-0.40, 40), (-0.36, 32), (-0.32, 22), (-0.29, 12), (-0.25, 0), (-0.23, 0)]
plan = [('lift_hi', [p0[0], p0[1], 1.25], qcur)]
h, q = hand_for(*WPS[0], 1.25); plan.append(('over_pre', h, q))
h, q = hand_for(*WPS[0], Z_IN); plan.append(('pre_ins', h, q))
for i, (cy, psi) in enumerate(WPS[1:], 1):
    h, q = hand_for(cy, psi, Z_IN); plan.append((f'ins{i}', h, q))
h, q = hand_for(*WPS[-1], Z_LOW); plan.append(('lower', h, q))
plan.append(('OPEN',))
for i, (cy, psi) in [(4, WPS[4]), (3, WPS[3]), (2, WPS[2]), (0, WPS[0])]:
    h, q = hand_for(cy, psi, Z_LOW if i == 4 else Z_IN); plan.append((f'ret{i}', h, q))
h, q = hand_for(*WPS[0], 1.25); plan.append(('ret_hi', h, q))

sols = []; seed = q0; prev = q0
for step in plan:
    if len(step) == 1: sols.append(step); continue
    name, p, qq = step
    s = ik_near(r, p, qq, seed, tries=8)
    if s is None: sys.exit(f'IK fail {name} {np.round(p,3)}')
    bad = path_check(r, prev, s, verbose=False)
    dc = door_clearance(s)
    print(f'{name:9s} {np.round(p,3)} -> {np.round(s,3)}  dist {np.abs(s-prev).max():.2f} bad {bad} door {dc[0]:.3f} {dc[1]}', flush=True)
    sols.append((name, s)); seed = s; prev = s
np.save('phase3_sols.npy', np.array([s[1] for s in sols if len(s) == 2]))
if '--go' not in sys.argv: sys.exit(0)
for step in sols:
    if step[0] == 'OPEN':
        r.gripper(0.04); continue
    name, s = step
    print('>>', name, flush=True)
    ok = move_q_retry(r, s, 3.0)
    f = r.fingers()
    print('   reached' if ok else '   NOT reached', 'hand', np.round(r.fk()[0], 4), 'fingers %.4f' % f[0], 'wrench', np.round(r.wrench()[:3], 1), flush=True)
    if not ok: sys.exit('move failed')
    if name.startswith('ins') or name in ('pre_ins', 'lower', 'over_pre'):
        if f[0] < 0.004: sys.exit('grasp lost')
OPENRUA_EOF

# openrua op 67
timeout 1700 python3 -u phase3.py 2>&1 | grep -v XMLPARSER

# openrua op 68
python3 - <<'EOF'
s=open('phase3.py').read()
s=s.replace("CX = -0.14 ","CX = -0.13 ")
s=s.replace("WPS = [(-0.40, 40), (-0.36, 32), (-0.32, 22), (-0.29, 12), (-0.25, 0), (-0.23, 0)]",
            "WPS = [(-0.40, 35), (-0.36, 28), (-0.32, 20), (-0.29, 10), (-0.25, 0)]")
s=s.replace("for i, (cy, psi) in [(4, WPS[4]), (3, WPS[3]), (2, WPS[2]), (0, WPS[0])]:\n    h, q = hand_for(cy, psi, Z_LOW if i == 4 else Z_IN); plan.append((f'ret{i}', h, q))",
            "for i in (3, 2, 1, 0):\n    h, q = hand_for(*WPS[i], Z_IN); plan.append((f'ret{i}', h, q))")
open('phase3.py','w').write(s)
EOF
timeout 1700 python3 -u phase3.py 2>&1 | grep -v "XMLPARSER\|in microwave box"

# openrua op 69
cat > sweep3.py <<'EOF'
import sys
sys.argv=['x']
exec(open('phase3.py').read().split('# mug-centre waypoints')[0])
CX=-0.15
seed=np.array([-1.004,1.296,-0.139,-0.984,1.374,1.163,-1.561])
for cy,psi in [(-0.40,35),(-0.40,30),(-0.38,30),(-0.37,28),(-0.36,25),(-0.33,20),(-0.31,15),(-0.29,10),(-0.27,5),(-0.25,0),(-0.24,0)]:
    h,q=hand_for(cy,psi,Z_IN)
    s=ik_near(r,h,q,seed,tries=8)
    if s is None: print(cy,psi,'IK fail'); continue
    dc=door_clearance(s); print(cy,psi,np.round(h,3),np.round(s,3),'door %.3f %s'%dc, flush=True)
EOF
timeout 1500 python3 -u sweep3.py 2>&1 | grep -v XMLPARSER

# openrua op 70
python3 - <<'EOF'
s=open('phase3.py').read()
s=s.replace("CX = -0.13 ","CX = -0.15 ")
s=s.replace("WPS = [(-0.40, 35), (-0.36, 28), (-0.32, 20), (-0.29, 10), (-0.25, 0)]",
            "WPS = [(-0.40, 30), (-0.36, 25), (-0.33, 20), (-0.29, 10), (-0.27, 5), (-0.25, 0)]")
s=s.replace("for i in (3, 2, 1, 0):","for i in (4, 3, 2, 1, 0):")
open('phase3.py','w').write(s)
EOF
timeout 1700 python3 -u phase3.py 2>&1 | grep -v "XMLPARSER\|in microwave box"

# openrua op 71
timeout 1750 python3 -u phase3.py --go 2>&1 | tee phase3.log | grep -v "XMLPARSER\|in microwave box\|^  move_q\|retry"

# openrua op 72
for c in agentview frontview sideview robot0_eye_in_hand; do python3 cloud.py $c 2>&1 | grep -v XMLPARSER; done; python3 -c "
from rob import *
r=Robot('t'); print(np.round(r.arm_q(),3)); print(np.round(r.wrench(),2)); print(r.fingers())"

# openrua op 73
python3 cloud.py birdview 2>&1 | grep -v XMLP; python3 -c "
import numpy as np, cv2
P=np.load('birdview_cloud.npy'); X,Y,Z=P[...,0],P[...,1],P[...,2]
img=cv2.imread('birdview.png'); b,g,r=img[...,0].astype(int),img[...,1].astype(int),img[...,2].astype(int)
yellow=(r>150)&(g>120)&(b<100)
m=yellow&(Z>0.9)
print('yellow pts',m.sum()); 
if m.sum(): print('x %.3f..%.3f y %.3f..%.3f z %.3f..%.3f'%(X[m].min(),X[m].max(),Y[m].min(),Y[m].max(),Z[m].min(),Z[m].max()))
# microwave front face: points near y=-0.36 with z between 0.95 and 1.1
m2=(X>-0.30)&(X<0.06)&(Y<-0.30)&(Y>-0.45)&(Z>1.0)&(Z<1.12)
print('front region pts', m2.sum(), 'ymin %.3f'%Y[m2].min() if m2.sum() else '')
for yl in np.arange(-0.45,-0.30,0.01):
    mm=(X>-0.30)&(X<0.06)&(Y>=yl)&(Y<yl+0.01)&(Z>0.95)
    if mm.sum(): print('y %.2f n %d z %.3f..%.3f x %.3f..%.3f'%(yl,mm.sum(),Z[mm].min(),Z[mm].max(),X[mm].min(),X[mm].max()))
"

# openrua op 74
grep -A12 ">> ins1" phase3.log | grep -v XMLP; python3 -c "
import numpy as np
q=np.array([-0.974,1.309,-0.167,-0.934,1.298,1.045,-1.588]); t=np.array([-0.968,1.315,-0.161,-0.943,1.288,1.044,-1.592]); print(np.round(q-t,4))"

# openrua op 75
python3 -c "
import numpy as np, cv2
for cam in ['frontview','agentview']:
    P=np.load(cam+'_cloud.npy'); X,Y,Z=P[...,0],P[...,1],P[...,2]
    img=cv2.imread(cam+'.png'); b,g,r=img[...,0].astype(int),img[...,1].astype(int),img[...,2].astype(int)
    yellow=(r>140)&(g>110)&(b<110)&(r-b>60)
    m=yellow&(Z>0.9)&(Y<-0.3)
    print(cam,'yellow pts',m.sum())
    if m.sum():
        print(' x %.3f..%.3f y %.3f..%.3f z %.3f..%.3f'%(X[m].min(),X[m].max(),Y[m].min(),Y[m].max(),Z[m].min(),Z[m].max()))
        # rim: top points per y-slice
        for yl in np.arange(-0.45,-0.35,0.01):
            mm=m&(Y>=yl)&(Y<yl+0.01)
            if mm.sum(): print('  y %.2f n %d zmax %.3f zmin %.3f'%(yl,mm.sum(),Z[mm].max(),Z[mm].min()))
"

# openrua op 76
python3 - <<'EOF'
s=open('rob.py').read()
s=s.replace('''def move_q_retry(r, q, seconds=3.0, tries=4, tol=0.01):
    for i in range(tries):
        code = r.move_q(q, seconds)
        err = np.abs(r.arm_q() - np.asarray(q)).max()
        if err < tol:
            return True
        print(f'  retry {i+1}: max_joint_err={err:.3f}', flush=True)
    return False''','''def move_q_retry(r, q, seconds=3.0, tries=4, tol=0.01):
    """Move to q; resend on lag. If a small static offset remains (payload
    droop), command q minus the residual so the arm lands on q."""
    q = np.asarray(q, float); cmd = q.copy()
    for i in range(tries):
        code = r.move_q(cmd, seconds)
        res = r.arm_q() - q
        err = np.abs(res).max()
        if err < tol:
            return True
        if err < 0.05:
            cmd = cmd - res
        print(f'  retry {i+1}: max_joint_err={err:.3f}', flush=True)
    return False''')
open('rob.py','w').write(s)
s=open('phase3.py').read()
s=s.replace("Z_IN = 1.006 ","Z_IN = 1.016 ").replace("Z_LOW = 0.997 ","Z_LOW = 1.003 ")
s=s.replace("if '--go' not in sys.argv: sys.exit(0)\nfor step in sols:",
"if '--go' not in sys.argv: sys.exit(0)\nif '--from' in sys.argv:\n    k = [i for i, s in enumerate(sols) if s[0] == sys.argv[sys.argv.index('--from') + 1]][0]; sols = sols[k:]\nfor step in sols:")
s=s.replace("ok = move_q_retry(r, s, 3.0)","ok = move_q_retry(r, s, 3.0, tries=5, tol=0.006)")
open('phase3.py','w').write(s)
EOF
grep -n "Z_IN =\|Z_LOW =\|--from\|move_q_retry" phase3.py

# openrua op 77
timeout 1750 python3 -u phase3.py --go --from pre_ins 2>&1 | tee phase3b.log | grep -v "XMLPARSER\|in microwave box\|^  move_q"

# openrua op 78
python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_cloud.npy'); X,Y,Z=P[...,0],P[...,1],P[...,2]
m=(Y>-0.375)&(Y<-0.345)&(X>-0.30)&(X<0.06)&(Z>0.90)&(Z<1.12)
print('front face pts',m.sum())
zs=Z[m]; xs=X[m]
h,e=np.histogram(zs,bins=np.arange(0.90,1.121,0.01)); 
for hh,ee in zip(h,e): print('z %.2f n %d'%(ee,hh), 'x %.3f..%.3f'%(xs[(zs>=ee)&(zs<ee+0.01)].min(),xs[(zs>=ee)&(zs<ee+0.01)].max()) if hh else '')
print('--- x histogram at z 0.98-1.03 on the face')
mm=m&(Z>0.98)&(Z<1.03)
h,e=np.histogram(X[mm],bins=np.arange(-0.30,0.061,0.01))
for hh,ee in zip(h,e): print('x %.2f n %d'%(ee,hh))
# interior points (deeper than face)
mi=(Y>-0.34)&(Y<-0.18)&(X>-0.28)&(X<-0.04)&(Z>0.90)&(Z<1.12)
print('interior: z %.3f..%.3f  x %.3f..%.3f'%(Z[mi].min(),Z[mi].max(),X[mi].min(),X[mi].max()))
h,e=np.histogram(Z[mi],bins=np.arange(0.90,1.121,0.01))
for hh,ee in zip(h,e): print('  z %.2f n %d'%(ee,hh))
"

# openrua op 79
python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_cloud.npy'); X,Y,Z=P[...,0],P[...,1],P[...,2]
m=(Y>-0.38)&(Y<-0.34)&(X>-0.25)&(X<-0.07)&(Z>0.90)&(Z<1.12)
print('aperture-column face pts',m.sum())
h,e=np.histogram(Z[m],bins=np.arange(0.90,1.121,0.005))
for hh,ee in zip(h,e):
    if hh: print('z %.3f n %d y %.3f..%.3f'%(ee,hh,Y[m&(Z>=ee)&(Z<ee+0.005)].min(),Y[m&(Z>=ee)&(Z<ee+0.005)].max()))
# also: any points between y -0.36 and -0.30 in the aperture column at z<1.0 (bottom lip)?
m2=(Y>-0.40)&(Y<-0.30)&(X>-0.25)&(X<-0.07)&(Z>0.90)&(Z<0.99)
print('low pts near opening', m2.sum())
if m2.sum():
    for yl in np.arange(-0.40,-0.30,0.01):
        mm=m2&(Y>=yl)&(Y<yl+0.01)
        if mm.sum(): print('  y %.2f n %d z %.3f..%.3f x %.3f..%.3f'%(yl,mm.sum(),Z[mm].min(),Z[mm].max(),X[mm].min(),X[mm].max()))
"

# openrua op 80
python3 tools/perception/cam_snap.py robot0_eye_in_hand eih2.png 2>&1 | grep -v XMLP; python3 tools/perception/cam_snap.py agentview agent2.png 2>&1 | grep -v XMLP

# openrua op 81
python3 -c "
from rob import *
r=Robot('t'); q=np.array([-1.009,1.354,-0.185,-0.855,1.371,1.051,-1.522]); print(move_q_retry(r,q,3.0,tries=5,tol=0.006)); print(np.round(r.fk()[0],4), r.fingers(), np.round(r.wrench()[:3],1))" 2>&1 | grep -v XMLP; python3 cloud.py agentview 2>&1 | grep -v XMLP; python3 -c "
import numpy as np, cv2
P=np.load('agentview_cloud.npy'); X,Y,Z=P[...,0],P[...,1],P[...,2]
img=cv2.imread('agentview.png'); b,g,r=img[...,0].astype(int),img[...,1].astype(int),img[...,2].astype(int)
yellow=(r>140)&(g>110)&(b<110)&(r-b>60)
m=yellow&(Z>0.9)&(Y<-0.3)&(Y>-0.5)
print('yellow n',m.sum(),'x %.3f..%.3f y %.3f..%.3f z %.3f..%.3f'%(X[m].min(),X[m].max(),Y[m].min(),Y[m].max(),Z[m].min(),Z[m].max()))
h,e=np.histogram(Z[m],bins=np.arange(0.90,1.10,0.01))
for hh,ee in zip(h,e):
    if hh: print('  z %.2f n %d y %.3f..%.3f'%(ee,hh,Y[m&(Z>=ee)&(Z<ee+0.01)].min(),Y[m&(Z>=ee)&(Z<ee+0.01)].max()))
"

# openrua op 82
python3 -c "
import numpy as np
P=np.load('agentview_cloud.npy'); X,Y,Z=P[...,0],P[...,1],P[...,2]
m=(X>-0.21)&(X<-0.09)&(Y>-0.47)&(Y<-0.355)&(Z>0.90)&(Z<1.06)
print('n',m.sum())
h,e=np.histogram(Z[m],bins=np.arange(0.90,1.06,0.005))
for hh,ee in zip(h,e):
    if hh: print('  z %.3f n %d x %.3f..%.3f y %.3f..%.3f'%(ee,hh,X[m&(Z>=ee)&(Z<ee+0.005)].min(),X[m&(Z>=ee)&(Z<ee+0.005)].max(),Y[m&(Z>=ee)&(Z<ee+0.005)].min(),Y[m&(Z>=ee)&(Z<ee+0.005)].max()))
"

# openrua op 83
python3 - <<'EOF'
s=open('phase3.py').read()
s=s.replace("Z_IN = 1.016 ","Z_IN = 1.038 ")
s=s.replace("h, q = hand_for(*WPS[-1], Z_LOW); plan.append(('lower', h, q))",
"for j, z in enumerate((1.028, 1.020, 1.012, 1.004)):\n    h, q = hand_for(*WPS[-1], z); plan.append((f'lower{j}', h, q))")
s=s.replace("""for step in sols:
    if step[0] == 'OPEN':""","""skip_lower = False
for step in sols:
    if step[0].startswith('lower') and skip_lower: continue
    if step[0] == 'OPEN':""")
s=s.replace("""    if not ok: sys.exit('move failed')
    if name.startswith('ins')""","""    if name.startswith('lower'):
        F = np.linalg.norm(r.wrench()[:3])
        print('   |F| = %.1f' % F, flush=True)
        if F > 9.0 or not ok:
            print('   contact with floor -> stop lowering', flush=True); skip_lower = True
        continue
    if not ok: sys.exit('move failed')
    if name.startswith('ins')""")
open('phase3.py','w').write(s)
EOF
timeout 1750 python3 -u phase3.py --go --from pre_ins 2>&1 | tee phase3c.log | grep -v "XMLPARSER\|in microwave box\|^  move_q"

# openrua op 84
python3 -c "
from rob import *
r=Robot('t'); q=np.array([-0.991,1.29,-0.212,-0.921,1.388,1.037,-1.525]); print(move_q_retry(r,q,3.0,tries=5,tol=0.006)); print(np.round(r.fk()[0],4))" 2>&1 | grep -v XMLP; python3 cloud.py robot0_eye_in_hand 2>&1 | grep -v XMLP; cp robot0_eye_in_hand.png eih3.png; python3 -c "
import numpy as np, cv2
P=np.load('robot0_eye_in_hand_cloud.npy'); X,Y,Z=P[...,0],P[...,1],P[...,2]
img=cv2.imread('robot0_eye_in_hand.png'); b,g,r=img[...,0].astype(int),img[...,1].astype(int),img[...,2].astype(int)
yellow=(r>120)&(g>90)&(b<110)&(r-b>50)
m=yellow&(Z>0.9)&(Y>-0.37)&(Y<-0.15)
print('yellow inside n',m.sum())
if m.sum(): print(' x %.3f..%.3f y %.3f..%.3f z %.3f..%.3f'%(X[m].min(),X[m].max(),Y[m].min(),Y[m].max(),Z[m].min(),Z[m].max()))
pts=P[m]
# fit circle to rim points (top 1cm)
top=pts[pts[:,2]>pts[:,2].max()-0.01][:,:2]
A=np.c_[2*top,np.ones(len(top))]; bb=(top**2).sum(1); c=np.linalg.lstsq(A,bb,rcond=None)[0]; print('rim circle center %.3f %.3f r %.3f'%(c[0],c[1],np.sqrt(c[2]+c[0]**2+c[1]**2)))
# bottom
for zl in np.arange(0.94,1.06,0.01):
    mm=m&(Z>=zl)&(Z<zl+0.01)
    if mm.sum(): print('  z %.2f n %d x %.3f..%.3f y %.3f..%.3f'%(zl,mm.sum(),X[mm].min(),X[mm].max(),Y[mm].min(),Y[mm].max()))
"

# openrua op 85
python3 -c "
from rob import *
r=Robot('t'); q0=r.arm_q(); qy,_=Q_yaw(0)
seed=np.array([-0.783,1.339,-0.19,-0.808,1.155,0.814,-1.607])
for z in [0.99,1.0]:
  for y in [-0.42,-0.43,-0.44,-0.45,-0.46]:
    s=ik_near(r,[-0.15,y,z],qy,seed,tries=6)
    print(z,y, None if s is None else np.round(s,2))
" 2>&1 | grep -v XMLP

# openrua op 86
python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_cloud.npy'); X,Y,Z=P[...,0],P[...,1],P[...,2]
m=(X>-0.25)&(X<-0.07)&(Y>-0.40)&(Y<-0.18)&(Z>0.95)&(Z<1.075)
print('n',m.sum())
for zl in np.arange(0.95,1.07,0.01):
    mm=m&(Z>=zl)&(Z<zl+0.01)
    if mm.sum():
        ys=Y[mm]; xs=X[mm]
        # nearest wall y as function of x bins
        line=[]
        for xl in np.arange(-0.25,-0.07,0.02):
            k=mm&(X>=xl)&(X<xl+0.02)
            line.append('%.3f'%Y[k].min() if k.sum() else '  -  ')
        print('z %.2f n %4d ymin %.3f | '%(zl,mm.sum(),ys.min()),' '.join(line))
"

# openrua op 87
python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_cloud.npy'); X,Y,Z=P[...,0],P[...,1],P[...,2]
m=(X>-0.25)&(X<-0.07)&(Y>-0.42)&(Y<-0.375)&(Z>0.95)&(Z<1.05)
print('handle pts',m.sum(),'x %.3f..%.3f mean %.3f  y %.3f..%.3f  z %.3f..%.3f'%(X[m].min(),X[m].max(),X[m].mean(),Y[m].min(),Y[m].max(),Z[m].min(),Z[m].max()))
for zl in np.arange(0.95,1.05,0.01):
    mm=m&(Z>=zl)&(Z<zl+0.01)
    if mm.sum(): print(' z %.2f n %d x %.3f..%.3f y %.3f..%.3f'%(zl,mm.sum(),X[mm].min(),X[mm].max(),Y[mm].min(),Y[mm].max()))
# body rim circle from all points z 1.03-1.05 with y>-0.37
mb=(X>-0.25)&(X<-0.07)&(Y>-0.375)&(Y<-0.2)&(Z>1.035)&(Z<1.05)
pts=P[mb][:,:2]; A=np.c_[2*pts,np.ones(len(pts))]; b=(pts**2).sum(1); c=np.linalg.lstsq(A,b,rcond=None)[0]; print('rim fit center %.3f %.3f r %.3f n %d'%(c[0],c[1],np.sqrt(c[2]+c[0]**2+c[1]**2),len(pts)))
"

# openrua op 88
mkdir -p "$(dirname /workspace/push.py)"
cat > /workspace/push.py <<'OPENRUA_EOF'
import sys
from rob import *
exec(open('/workspace/phase3.py').read().split('# mug-centre waypoints')[0].split("r = Robot('p3')")[1].replace("q0 = r.arm_q(); p0, qcur = r.fk(); print('start q', np.round(q0, 3), 'hand', np.round(p0, 3))", ""))  # helpers: fk_links, door_clearance

TH = np.radians(25)
D = np.array([0, np.cos(TH), -np.sin(TH)]); HY = np.array([1.0, 0, 0]); HX = np.cross(HY, D)
Q_PUSH = quat_from_axes(HX, HY, D)
XT = -0.17                     # handle bar x
TIP_Z = 0.975                  # fingertip height while pushing


def hand_for_tips(ty, tz=TIP_Z):
    return list(np.array([XT, ty, tz]) - TCP * D)


q0 = r.arm_q(); p0, qcur = r.fk(); print('start q', np.round(q0, 3), 'hand', np.round(p0, 3))
plan = [('hi', [p0[0], p0[1], 1.25], qcur),
        ('p0', hand_for_tips(-0.43, TIP_Z + 0.13), Q_PUSH),
        ('p1', hand_for_tips(-0.43), Q_PUSH)]
for ty in (-0.41, -0.39, -0.37, -0.355, -0.345):
    plan.append((f'push{ty}', hand_for_tips(ty), Q_PUSH))
plan += [('back', hand_for_tips(-0.43), Q_PUSH), ('up', hand_for_tips(-0.43, TIP_Z + 0.13), Q_PUSH)]

sols = []; seed = q0; prev = q0
for name, p, qq in plan:
    s = ik_near(r, p, qq, seed, tries=8)
    if s is None: sys.exit(f'IK fail {name} {np.round(p,3)}')
    bad = path_check(r, prev, s, verbose=False)
    dc = door_clearance(s)
    lk = fk_links(s)
    print(f'{name:10s} {np.round(p,3)} -> {np.round(s,3)} dist {np.abs(s-prev).max():.2f} bad {bad} door {dc[0]:.3f} {dc[1]} l7 {np.round(lk["panda_link7"][0],3)}', flush=True)
    sols.append((name, s)); seed = s; prev = s
if '--go' not in sys.argv: sys.exit(0)
r.gripper(0.0)
for name, s in sols:
    print('>>', name, flush=True)
    ok = move_q_retry(r, s, 3.0, tries=4, tol=0.006)
    F = r.wrench()[:3]
    print('   reached' if ok else '   NOT reached', 'hand', np.round(r.fk()[0], 4), 'F', np.round(F, 1), '|F| %.1f' % np.linalg.norm(F), flush=True)
    if name.startswith('push') and (np.linalg.norm(F) > 25 or not ok):
        print('   large force / blocked -> stop pushing', flush=True)
        # fall through to retreat steps only
        sols_rest = [x for x in sols if x[0] in ('back', 'up')]
        for n2, s2 in sols_rest:
            print('>>', n2, flush=True); move_q_retry(r, s2, 3.0, tries=4, tol=0.006)
        break
OPENRUA_EOF

# openrua op 89
python3 - <<'EOF'
s=open('/workspace/push.py').read()
start=s.index("exec("); end=s.index("\n", start)
helpers=open('/workspace/phase3.py').read()
h0=helpers.index("HINGE ="); h1=helpers.index("# mug-centre waypoints")
helper_code=helpers[h0:h1].replace("def hand_for(cy, psi, z):\n    q, d = Q_yaw(psi)\n    c = np.array([CX, cy, 0.0])\n    return list(c - L * d + np.array([0, 0, z]))\n\n\n","")
s=s[:start]+"from moveit_msgs.srv import GetPositionFK\nr = Robot('push')\n"+helper_code+s[end+1:]
open('/workspace/push.py','w').write(s)
EOF
sed -n 1,50p push.py

# openrua op 90
timeout 1500 python3 -u push.py 2>&1 | grep -v "XMLPARSER\|in microwave box"

# openrua op 91
python3 -c "
from rob import *
r=Robot('t'); qy,_=Q_yaw(0)
TH=np.radians(25); D=np.array([0,np.cos(TH),-np.sin(TH)]); HY=np.array([1.0,0,0]); HX=np.cross(HY,D); QP=quat_from_axes(HX,HY,D)
rng=np.random.default_rng(0)
lo=np.array([l[0] for l in LIMITS]); hi=np.array([l[1] for l in LIMITS])
for name,p,q in [('flat -0.44',[-0.15,-0.44,1.0],qy),('flat -0.46',[-0.15,-0.46,1.0],qy),('pitch p1',[-0.17,-0.524,1.019],QP),('pitch p0',[-0.17,-0.524,1.149],QP)]:
    found=[]
    for i in range(12):
        seed=lo+(hi-lo)*rng.random(7)
        s=_orig_ik(r,p,hand_to_link8(q),seed=seed,timeout=0.5,attempts=1)
        if s is not None: found.append(np.round(s,2))
    print(name,len(found)); [print('   ',f) for f in found[:4]]
" 2>&1 | grep -v XMLP

# openrua op 92
python3 -c "
from rob import *
import rob
r=Robot('t'); qy,_=Q_yaw(0)
TH=np.radians(25); D=np.array([0,np.cos(TH),-np.sin(TH)]); HY=np.array([1.0,0,0]); HX=np.cross(HY,D); QP=quat_from_axes(HX,HY,D)
rng=np.random.default_rng(0)
lo=np.array([l[0] for l in LIMITS]); hi=np.array([l[1] for l in LIMITS])
for name,p,q in [('flat -0.44',[-0.15,-0.44,1.0],qy),('flat -0.46',[-0.15,-0.46,1.0],qy),('pitch p1',[-0.17,-0.524,1.019],QP),('pitch p0',[-0.17,-0.524,1.149],QP)]:
    found=[]
    for i in range(12):
        seed=lo+(hi-lo)*rng.random(7)
        s=rob._orig_ik(r,p,hand_to_link8(q),seed=seed,timeout=0.5,attempts=1)
        if s is not None: found.append(np.round(s,2))
    print(name,len(found)); [print('   ',f) for f in found[:4]]
" 2>&1 | grep -v XMLP

# openrua op 93
python3 - <<'EOF'
s=open('/workspace/push.py').read()
head=s[:s.index("TH = np.radians(25)")]
body='''
TIP_Z = 0.98


def hand_for_tips(tx, ty, psi, z=TIP_Z):
    q, d = Q_yaw(psi)
    return list(np.array([tx, ty, z]) - TCP * d), q


q0 = r.arm_q(); p0, qcur = r.fk(); print('start q', np.round(q0, 3), 'hand', np.round(p0, 3))
# (tip x, tip y, yaw)
TIPS = [('start', -0.198, -0.41, 30), ('behind', -0.17, -0.41, 30),
        ('push1', -0.17, -0.39, 25), ('push2', -0.17, -0.375, 20), ('push3', -0.168, -0.365, 15), ('push4', -0.165, -0.355, 10),
        ('back1', -0.17, -0.39, 15), ('back2', -0.198, -0.43, 30)]
plan = [('hi', [p0[0], p0[1], 1.25], qcur)]
h, q = hand_for_tips(-0.198, -0.41, 30, 1.20); plan.append(('over', h, q))
for name, tx, ty, psi in TIPS:
    h, q = hand_for_tips(tx, ty, psi); plan.append((name, h, q))
h, q = hand_for_tips(-0.198, -0.43, 30, 1.20); plan.append(('up', h, q))

sols = []; seed = q0; prev = q0
for name, p, qq in plan:
    s = ik_near(r, p, qq, seed, tries=8)
    if s is None: sys.exit(f'IK fail {name} {np.round(p,3)}')
    bad = path_check(r, prev, s, verbose=False)
    dc = door_clearance(s)
    print(f'{name:8s} hand {np.round(p,3)} -> {np.round(s,3)} dist {np.abs(s-prev).max():.2f} bad {bad} door {dc[0]:.3f} {dc[1]}', flush=True)
    sols.append((name, s)); seed = s; prev = s
if '--go' not in sys.argv: sys.exit(0)
r.gripper(0.0)
abort = False
for name, s in sols:
    if abort and not name.startswith(('back', 'up')): continue
    print('>>', name, flush=True)
    ok = move_q_retry(r, s, 3.0, tries=4, tol=0.006)
    F = r.wrench()[:3]
    print('   reached' if ok else '   NOT reached', 'hand', np.round(r.fk()[0], 4), 'F', np.round(F, 1), '|F| %.1f' % np.linalg.norm(F), flush=True)
    if name.startswith('push') and (np.linalg.norm(F) > 25 or not ok):
        print('   large force / blocked -> stop pushing', flush=True); abort = True
'''
open('/workspace/push2.py','w').write(head+body)
EOF
timeout 1500 python3 -u push2.py 2>&1 | grep -v "XMLPARSER\|in microwave box"

# openrua op 94
python3 - <<'EOF'
s=open('/workspace/push2.py').read()
s=s.replace("TIPS = [('start', -0.198, -0.41, 30), ('behind', -0.17, -0.41, 30),","TIPS = [('behind', -0.17, -0.41, 30),")
s=s.replace("('back1', -0.17, -0.39, 15), ('back2', -0.198, -0.43, 30)]","('back1', -0.17, -0.39, 15), ('back2', -0.18, -0.43, 30)]")
s=s.replace("h, q = hand_for_tips(-0.198, -0.41, 30, 1.20); plan.append(('over', h, q))","h, q = hand_for_tips(-0.17, -0.41, 30, 1.20); plan.append(('over', h, q))")
s=s.replace("h, q = hand_for_tips(-0.198, -0.43, 30, 1.20); plan.append(('up', h, q))","h, q = hand_for_tips(-0.18, -0.43, 30, 1.20); plan.append(('up', h, q))")
open('/workspace/push2.py','w').write(s)
EOF
timeout 1700 python3 -u push2.py --go 2>&1 | tee push2.log | grep -v "XMLPARSER\|in microwave box\|^  move_q"

# openrua op 95
python3 -c "
from rob import *
r=Robot('t'); q=np.array([-0.991,1.29,-0.212,-0.921,1.388,1.037,-1.525]); print(move_q_retry(r,q,3.0,tries=5,tol=0.006)); print(np.round(r.fk()[0],4))" 2>&1 | grep -v XMLP; python3 cloud.py robot0_eye_in_hand 2>&1 | grep -v XMLP; cp robot0_eye_in_hand.png eih4.png; python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_cloud.npy'); X,Y,Z=P[...,0],P[...,1],P[...,2]
m=(X>-0.25)&(X<-0.07)&(Y>-0.42)&(Y<-0.18)&(Z>0.95)&(Z<1.075)
for zl in np.arange(0.95,1.07,0.01):
    mm=m&(Z>=zl)&(Z<zl+0.01)
    if mm.sum():
        line=[]
        for xl in np.arange(-0.25,-0.07,0.02):
            k=mm&(X>=xl)&(X<xl+0.02)
            line.append('%.3f'%Y[k].min() if k.sum() else '  -  ')
        print('z %.2f n %4d ymin %.3f | '%(zl,mm.sum(),Y[mm].min()),' '.join(line))
mb=(X>-0.25)&(X<-0.07)&(Y>-0.40)&(Y<-0.2)&(Z>1.035)&(Z<1.05)
pts=P[mb][:,:2]; A=np.c_[2*pts,np.ones(len(pts))]; b=(pts**2).sum(1); c=np.linalg.lstsq(A,b,rcond=None)[0]; print('rim fit center %.3f %.3f r %.3f n %d'%(c[0],c[1],np.sqrt(c[2]+c[0]**2+c[1]**2),len(pts)))
"

# openrua op 96
python3 - <<'EOF'
s=open('/workspace/push2.py').read()
s=s.replace("""TIPS = [('behind', -0.17, -0.41, 30),
        ('push1', -0.17, -0.39, 25), ('push2', -0.17, -0.375, 20), ('push3', -0.168, -0.365, 15), ('push4', -0.165, -0.355, 10),
        ('back1', -0.17, -0.39, 15), ('back2', -0.18, -0.43, 30)]""",
"""TIPS = [('behind', -0.17, -0.41, 30),
        ('push1', -0.17, -0.39, 25), ('push2', -0.17, -0.375, 20), ('push3', -0.168, -0.365, 15), ('push4', -0.165, -0.355, 10),
        ('push5', -0.163, -0.345, 7), ('push6', -0.16, -0.335, 5),
        ('back1', -0.163, -0.365, 5)]""")
s=s.replace("h, q = hand_for_tips(-0.18, -0.43, 30, 1.20); plan.append(('up', h, q))","h, q = hand_for_tips(-0.163, -0.365, 5, 1.20); plan.append(('up', h, q))")
s=s.replace("r = Robot('push')","r = Robot('push3')")
open('/workspace/push3.py','w').write(s)
EOF
timeout 1700 python3 -u push3.py --go 2>&1 | tee push3.log | grep -v "XMLPARSER\|in microwave box\|^  move_q"

# openrua op 97
sed -i "s/('back1', -0.163, -0.365, 5)\]/('back1', -0.17, -0.39, 15)]/; s/h, q = hand_for_tips(-0.163, -0.365, 5, 1.20); plan.append(('up', h, q))/h, q = hand_for_tips(-0.17, -0.39, 15, 1.20); plan.append(('up', h, q))/" push3.py && timeout 1700 python3 -u push3.py --go 2>&1 | tee push3.log | grep -v "XMLPARSER\|in microwave box\|^  move_q"

# openrua op 98
sed -i "s/if name.startswith('push') and (np.linalg.norm(F) > 25 or not ok):/if name.startswith('push') and np.linalg.norm(F) > 25:/; s/ok = move_q_retry(r, s, 3.0, tries=4, tol=0.006)/ok = move_q_retry(r, s, 3.0, tries=3, tol=0.012)/" push3.py && timeout 1700 python3 -u push3.py --go 2>&1 | tee push3b.log | grep -v "XMLPARSER\|in microwave box\|^  move_q\|-> \["

# openrua op 99
python3 -c "
from rob import *
r=Robot('t'); q=np.array([-0.991,1.29,-0.212,-0.921,1.388,1.037,-1.525]); print(move_q_retry(r,q,3.0,tries=5,tol=0.006)); print(np.round(r.fk()[0],4))" 2>&1 | grep -v XMLP; python3 cloud.py robot0_eye_in_hand 2>&1 | grep -v XMLP; cp robot0_eye_in_hand.png eih5.png; python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_cloud.npy'); X,Y,Z=P[...,0],P[...,1],P[...,2]
m=(X>-0.25)&(X<-0.07)&(Y>-0.42)&(Y<-0.18)&(Z>0.95)&(Z<1.075)
for zl in np.arange(0.95,1.07,0.01):
    mm=m&(Z>=zl)&(Z<zl+0.01)
    if mm.sum():
        line=[]
        for xl in np.arange(-0.25,-0.07,0.02):
            k=mm&(X>=xl)&(X<xl+0.02)
            line.append('%.3f'%Y[k].min() if k.sum() else '  -  ')
        print('z %.2f n %4d ymin %.3f | '%(zl,mm.sum(),Y[mm].min()),' '.join(line))
mb=(X>-0.25)&(X<-0.07)&(Y>-0.40)&(Y<-0.2)&(Z>1.035)&(Z<1.05)
pts=P[mb][:,:2]; A=np.c_[2*pts,np.ones(len(pts))]; b=(pts**2).sum(1); c=np.linalg.lstsq(A,b,rcond=None)[0]; print('rim fit center %.3f %.3f r %.3f n %d'%(c[0],c[1],np.sqrt(c[2]+c[0]**2+c[1]**2),len(pts)))
"

# openrua op 100
mkdir -p "$(dirname /workspace/door.py)"
cat > /workspace/door.py <<'OPENRUA_EOF'
import sys
from rob import *
r = Robot('door')
HINGE = np.array([-0.29, -0.36]); TIP_Z = 1.0


def door_pose(a_deg, R, pitch_deg=30.0):
    """Hand pose whose closed fingertips touch the OUTER face of the door
    (door angle a, hinge distance R), hand pointing at the door and down."""
    a = np.radians(a_deg); c, s = np.cos(np.radians(pitch_deg)), np.sin(np.radians(pitch_deg))
    contact = np.array([HINGE[0] + R * np.cos(a), HINGE[1] + R * np.sin(a), TIP_Z])
    n_out = np.array([np.sin(a), -np.cos(a), 0.0])
    d = -n_out * c + np.array([0, 0, -s])
    hy = np.array([np.cos(a), np.sin(a), 0.0])
    hx = np.cross(hy, d)
    if hx[2] < 0:           # keep the camera side up
        hy = -hy; hx = -hx
    q = quat_from_axes(hx, hy, d)
    return list(contact - TCP * d), q, contact


q0 = r.arm_q(); p0, qcur = r.fk(); print('start q', np.round(q0, 3), 'hand', np.round(p0, 3))
ARC = [(236, 0.16), (250, 0.16), (265, 0.16), (280, 0.16), (295, 0.16), (310, 0.15), (325, 0.14),
       (338, 0.12), (348, 0.10), (355, 0.10)]
plan = []
h, q, c = door_pose(236, 0.16); plan.append(('over_a', [h[0], h[1], 1.25], q))
h, q, c = door_pose(230, 0.16); plan.append(('pre', h, q))          # 6 deg behind the door
for a, R in ARC:
    h, q, c = door_pose(a, R); plan.append((f'a{a}', h, q))
h, q, c = door_pose(360, 0.10, 45); plan.append(('a360', h, q))
h, q, c = door_pose(355, 0.10); plan.append(('back', [h[0], h[1] - 0.05, h[2]], q))
h, q, c = door_pose(355, 0.10); plan.append(('up', [h[0], h[1] - 0.05, 1.25], q))

sols = []; seed = q0; prev = q0
for name, p, qq in plan:
    s = ik_near(r, p, qq, seed, tries=8)
    if s is None:
        print(f'IK fail {name} {np.round(p,3)}'); sols.append((name, None)); continue
    bad = path_check(r, prev, s, verbose=False)
    print(f'{name:7s} hand {np.round(p,3)} -> {np.round(s,3)} dist {np.abs(s-prev).max():.2f} bad {bad}', flush=True)
    sols.append((name, s)); seed = s; prev = s
if '--go' not in sys.argv: sys.exit(0)
r.gripper(0.0)
for name, s in sols:
    if s is None: print('skip', name); continue
    print('>>', name, flush=True)
    ok = move_q_retry(r, s, 3.0, tries=3, tol=0.012)
    F = r.wrench()[:3]
    print('   reached' if ok else '   NOT reached', 'hand', np.round(r.fk()[0], 4), 'F', np.round(F, 1), '|F| %.1f' % np.linalg.norm(F), flush=True)
    if name.startswith('a') and np.linalg.norm(F) > 40:
        print('   large force -> abort sweep', flush=True); break
OPENRUA_EOF

# openrua op 101
timeout 600 python3 -u door.py > door.log 2>&1; cat door.log

# openrua op 102
ls *.npy | head -30 && python3 - <<'EOF'
import numpy as np
for cam in ['birdview','agentview']:
    try: P=np.load(f'{cam}_cloud.npy').reshape(-1,3)
    except Exception as e: print(cam,e); continue
    P=P[np.isfinite(P).all(1)]
    # door panel region: along 236deg from hinge
    H=np.array([-0.29,-0.36]); a=np.radians(236); u=np.array([np.cos(a),np.sin(a)]); n=np.array([-np.sin(a),np.cos(a)])
    rel=P[:,:2]-H; al=rel@u; dn=rel@n
    m=(al>0.02)&(al<0.30)&(abs(dn)<0.04)&(P[:,2]>0.92)
    D=P[m]; print(cam,'door pts',m.sum())
    if m.sum():
        print(' z max %.3f  z pct 99 %.3f'%(D[:,2].max(),np.percentile(D[:,2],99)))
        for lo in np.arange(0.02,0.30,0.04):
            s=D[(al[m]>=lo)&(al[m]<lo+0.04)]
            if len(s): print('  along %.2f-%.2f: n %d dn mean %.3f min %.3f max %.3f zmax %.3f'%(lo,lo+0.04,len(s),s.__len__() and (s[:,:2]-H)@n and ((s[:,:2]-H)@n).mean(),((s[:,:2]-H)@n).min(),((s[:,:2]-H)@n).max(),s[:,2].max()))
EOF

# openrua op 103
timeout 300 python3 cloud.py birdview > /dev/null 2>&1; timeout 300 python3 cloud.py agentview > /dev/null 2>&1; python3 - <<'EOF'
import numpy as np
H=np.array([-0.29,-0.36]); a=np.radians(236); u=np.array([np.cos(a),np.sin(a)]); n=np.array([-np.sin(a),np.cos(a)])
for cam in ['birdview','agentview']:
    P=np.load(f'{cam}_cloud.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
    rel=P[:,:2]-H; al=rel@u; dn=rel@n
    m=(al>0.02)&(al<0.32)&(abs(dn)<0.05)&(P[:,2]>0.92)&(P[:,2]<1.2)
    D=P[m]; print(cam,'door pts',m.sum())
    zs=D[:,2]; print(' z hist', np.histogram(zs,bins=np.arange(0.92,1.2,0.02))[0])
    for lo in np.arange(0.02,0.32,0.03):
        s=(al>=lo)&(al<lo+0.03)&m
        if s.sum(): print('  along %.2f: n %4d dn mean %.3f min %.3f max %.3f zmax %.3f'%(lo,s.sum(),dn[s].mean(),dn[s].min(),dn[s].max(),P[s,2].max()))
EOF

# openrua op 104
mkdir -p "$(dirname /workspace/door2.py)"
cat > /workspace/door2.py <<'OPENRUA_EOF'
import sys
from rob import *
from moveit_msgs.srv import GetPositionFK
r = Robot('door2')
HINGE = np.array([-0.29, -0.36]); TIP_Z = 1.08      # tips 2.8cm below door top (1.108); hand body bottom ~1.125
OUT = 0.022                                        # tip centre offset to the outer side of the door model line (1cm penetration)


def fk_links(q, links=('panda_link5', 'panda_link6', 'panda_link7', 'panda_hand')):
    req = GetPositionFK.Request(); req.fk_link_names = list(links)
    req.robot_state.joint_state.name = ARM; req.robot_state.joint_state.position = list(map(float, q))
    f = r.fk_cli.call_async(req); rclpy.spin_until_future_complete(r.node, f, timeout_sec=60)
    return {l: np.array([p.pose.position.x, p.pose.position.y, p.pose.position.z]) for l, p in zip(links, f.result().pose_stamped)}


def door_pose(a_deg, R, z=TIP_Z):
    """Vertical hand, closed fingertips just outside the door's OUTER face at hinge distance R."""
    a = np.radians(a_deg)
    u = np.array([np.cos(a), np.sin(a), 0.0]); n_out = np.array([np.sin(a), -np.cos(a), 0.0])
    tip = np.array([HINGE[0], HINGE[1], 0.0]) + R * u + OUT * n_out + np.array([0, 0, z])
    d = np.array([0, 0, -1.0])
    qa = quat_from_axes(np.cross(u, d), u, d); qb = quat_from_axes(np.cross(-u, d), -u, d)
    return list(tip - TCP * d), (qa, qb)


q0 = r.arm_q(); p0, qcur = r.fk(); print('start q', np.round(q0, 3), 'hand', np.round(p0, 3))
ARC = [(228, 0.16), (240, 0.16), (252, 0.16), (264, 0.16), (276, 0.16), (288, 0.16), (300, 0.16), (312, 0.15),
       (324, 0.14), (336, 0.13), (346, 0.12), (354, 0.11), (362, 0.11)]
plan = [('hi', [p0[0], p0[1], 1.25], (qcur,))]
h, qq = door_pose(228, 0.16, 1.25); plan.append(('over', h, qq))
for a, R in ARC:
    h, qq = door_pose(a, R); plan.append((f'a{a}', h, qq))
h, qq = door_pose(354, 0.11); plan.append(('back', [h[0], h[1] - 0.04, h[2]], qq))
h, qq = door_pose(354, 0.11, 1.25); plan.append(('up', [h[0], h[1] - 0.04, h[2]], qq))

sols = []; seed = q0; prev = q0; chosen = None
for name, p, qs in plan:
    if len(qs) == 2 and chosen is not None: qs = (qs[chosen],)
    cands = []
    for i, qq in enumerate(qs):
        s = ik_near(r, p, qq, seed, tries=8)
        if s is not None: cands.append((np.abs(s - seed).max(), i, s))
    if not cands: print(f'IK fail {name} {np.round(p,3)}'); sols.append((name, None)); continue
    cands.sort(key=lambda t: t[0]); _, i, s = cands[0]
    if len(qs) == 2 and chosen is None: chosen = i; print('  yaw branch', i)
    bad = path_check(r, prev, s, verbose=False)
    lk = fk_links(s)
    print(f'{name:5s} hand {np.round(p,3)} -> {np.round(s,3)} dist {np.abs(s-prev).max():.2f} bad {bad} l7 {np.round(lk["panda_link7"],3)} l6 {np.round(lk["panda_link6"],3)}', flush=True)
    sols.append((name, s)); seed = s; prev = s
if '--go' not in sys.argv: sys.exit(0)
r.gripper(0.0)
for name, s in sols:
    if s is None: print('skip', name); continue
    print('>>', name, flush=True)
    ok = move_q_retry(r, s, 3.0, tries=3, tol=0.012)
    F = r.wrench()[:3]
    print('   reached' if ok else '   NOT reached', 'hand', np.round(r.fk()[0], 4), 'F', np.round(F, 1), '|F| %.1f' % np.linalg.norm(F), flush=True)
    if name.startswith('a') and np.linalg.norm(F) > 30:
        print('   large force -> stop sweep here', flush=True)
        rest = [x for x in sols if x[0] in ('back', 'up')]
        for n2, s2 in rest:
            print('>>', n2, flush=True); move_q_retry(r, s2, 3.0, tries=3, tol=0.012)
            print('   hand', np.round(r.fk()[0], 4), flush=True)
        break
OPENRUA_EOF

# openrua op 105
timeout 900 python3 -u door2.py > door2.log 2>&1; grep -v '^ *[0-9]* hand' door2.log; echo ---; grep -c 'below table' door2.log; grep 'below table\|microwave box' door2.log | head

# openrua op 106
python3 - <<'EOF'
import re
s=open('door2.py').read()
s=s.replace("""    d = np.array([0, 0, -1.0])
    qa = quat_from_axes(np.cross(u, d), u, d); qb = quat_from_axes(np.cross(-u, d), -u, d)
    return list(tip - TCP * d), (qa, qb)""","""    d = np.array([0, 0, -1.0])
    return list(tip - TCP * d), (Q_DOWN[YAW],)""")
s=s.replace("r = Robot('door2')","""r = Robot('door2')
Q_DOWN = {'a': quat_from_axes([0,-1,0],[-1,0,0],[0,0,-1]), 'b': quat_from_axes([0,1,0],[1,0,0],[0,0,-1])}
YAW = sys.argv[sys.argv.index('--yaw')+1] if '--yaw' in sys.argv else 'b'""")
open('door2.py','w').write(s)
EOF
(timeout 900 python3 -u door2.py --yaw a > door2a.log 2>&1; timeout 900 python3 -u door2.py --yaw b > door2b.log 2>&1); for f in door2a.log door2b.log; do echo "== $f"; grep -v '^ *[0-9]* hand' $f; done

# openrua op 107
python3 - <<'EOF'
s=open('door2.py').read()
s=s.replace("""    sols.append((name, s)); seed = s; prev = s
if '--go'""","""    # link sweep check along the joint-space interpolation
    worst = (9, ''); inbox = []
    for t in np.linspace(0, 1, 11):
        lk = fk_links(prev + t * (s - prev), ('panda_link5', 'panda_link6', 'panda_link7', 'panda_hand'))
        for l, p in lk.items():
            rad = 0.06 if l != 'panda_hand' else 0.10
            if p[2] - rad < worst[0]: worst = (p[2] - rad, l)
            if -0.29 - rad < p[0] < 0.06 + rad and -0.36 - rad < p[1] < -0.14 + rad and p[2] - rad < 1.108: inbox.append(l)
    print(f'      sweep: lowest link-bottom z {worst[0]:.3f} ({worst[1]})  over-microwave hits: {sorted(set(inbox))}', flush=True)
    sols.append((name, s)); seed = s; prev = s
np.save('door2_sols.npy', np.array([x[1] for x in sols if x[1] is not None]))
if '--go'""")
open('door2.py','w').write(s)
EOF
timeout 900 python3 -u door2.py --yaw b > door2b.log 2>&1; grep -v '^ *[0-9]* hand' door2b.log

# openrua op 108
timeout 1500 python3 -u door2.py --yaw b --go > door2go.log 2>&1; grep -v '^ *[0-9]* hand\|sweep:' door2go.log | grep -v '^[a-z0-9]* *hand'

# openrua op 109
timeout 300 python3 cloud.py birdview > /dev/null 2>&1; timeout 300 python3 cloud.py agentview > /dev/null 2>&1; python3 - <<'EOF'
import numpy as np
H=np.array([-0.29,-0.36])
for cam in ['birdview','agentview']:
    P=np.load(f'{cam}_cloud.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
    # points in front of the microwave face, above table, near door heights
    m=(P[:,2]>0.93)&(P[:,2]<1.15)&(P[:,1]<-0.34)&(P[:,1]>-0.70)&(P[:,0]>-0.6)&(P[:,0]<0.1)
    D=P[m]; print(cam,'pts in front zone',m.sum())
    # angle of each point about hinge
    ang=np.degrees(np.arctan2(D[:,1]-H[1],D[:,0]-H[0]))%360; rad=np.hypot(D[:,0]-H[0],D[:,1]-H[1])
    hist,edges=np.histogram(ang[rad>0.05],bins=np.arange(180,372,12)); print(' angle hist (12deg bins from 180):',hist)
    # y profile for x in [-0.28,-0.02]
    s=D[(D[:,0]>-0.28)&(D[:,0]<-0.02)]
    print(' y hist for x in door span:',np.histogram(s[:,1],bins=np.arange(-0.46,-0.33,0.01))[0])
    print(' z max in that band %.3f'%(s[:,2].max() if len(s) else -1))
EOF

# openrua op 110
python3 - <<'EOF'
import numpy as np
H=np.array([-0.29,-0.36])
P=np.load('agentview_cloud.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,2]>0.95)&(P[:,2]<1.10)&(P[:,1]<-0.355)&(P[:,1]>-0.50)&(P[:,0]>-0.30)&(P[:,0]<0.08)
D=P[m]
for lo in np.arange(-0.30,0.08,0.04):
    s=D[(D[:,0]>=lo)&(D[:,0]<lo+0.04)]
    if len(s): print('x %.2f..%.2f n %4d  y: min %.3f  p10 %.3f median %.3f  max %.3f'%(lo,lo+0.04,len(s),s[:,1].min(),np.percentile(s[:,1],10),np.median(s[:,1]),s[:,1].max()))
EOF

# openrua op 111
timeout 300 python3 cloud.py frontview > /dev/null 2>&1; timeout 300 python3 cloud.py sideview > /dev/null 2>&1; ls -la frontview.png sideview.png

# openrua op 112
python3 - <<'EOF'
import numpy as np
H=np.array([-0.29,-0.36])
for cam in ['birdview','frontview','sideview','agentview']:
    P=np.load(f'{cam}_cloud.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
    m=(P[:,2]>0.95)&(P[:,2]<1.115)&(P[:,1]<-0.362)&(P[:,1]>-0.55)&(P[:,0]>-0.32)&(P[:,0]<0.0)
    D=P[m]
    if len(D)==0: print(cam,'no pts'); continue
    ang=np.degrees(np.arctan2(D[:,1]-H[1],D[:,0]-H[0]))%360; rad=np.hypot(D[:,0]-H[0],D[:,1]-H[1])
    k=rad>0.08
    print(cam,'n',k.sum(),'angle p5 %.1f median %.1f p95 %.1f'%tuple(np.percentile(ang[k],[5,50,95])) if k.sum() else '')
    for lo in np.arange(-0.30,0.0,0.04):
        s=D[(D[:,0]>=lo)&(D[:,0]<lo+0.04)]
        if len(s): print('   x %.2f n %4d y med %.3f min %.3f  z %.3f-%.3f'%(lo,len(s),np.median(s[:,1]),s[:,1].min(),s[:,2].min(),s[:,2].max()))
EOF

# openrua op 113
mkdir -p "$(dirname /claude-config/projects/-workspace/memory/panda-sim-quirks.md)"
cat > /claude-config/projects/-workspace/memory/panda-sim-quirks.md <<'OPENRUA_EOF'
---
name: panda-sim-quirks
description: Hard-won quirks of the /workspace Panda + microwave simulation (IK frames, tracking, wrench, reach limits, helper scripts)
metadata:
  type: project
---

Mug-in-microwave task (Sept 2026) was completed; helper library `/workspace/rob.py`
(Robot class, `Q_yaw`, `ik_near`, `move_q_retry`, `path_check`) and `cloud.py` exist.

- MoveIt `/compute_ik` here solves for panda_link8 in WORLD coords; hand = link8 rotated −45° about z
  (`hand_to_link8` patch). TCP offset 0.1034 from hand frame; hand body ~0.2 wide along hand-y.
- Joint tracking: joint 7 lags (error −5); with payload a static ~0.01 rad residual stays → compensate
  by commanding q − residual (`move_q_retry`). Tolerance 0.006 is too strict, 0.012 works.
- Wrench magnitude baseline ≈5.1 N regardless of payload; contact shows only as a change in |F|.
- Reach: hand pointing +y at x≈−0.15 only reachable for y ≥ ≈−0.42; pitched-down poses near y −0.5
  are unreachable; vertical (down-pointing) hand with a FIXED yaw (fingers along x) is reachable
  over the whole y −0.38…−0.52 region and gave smooth IK branches (`door2.py`).
- Sim clock only advances with commands; verify by re-reading sensors, never by waiting.

**Why:** these took many failed attempts to discover and are not in the starter docs.
**How to apply:** reuse rob.py; check `path_check` + link FK sweeps before large joint-space jumps.
OPENRUA_EOF

# openrua op 114
cd /claude-config/projects/-workspace/memory && (test -f MEMORY.md && cat MEMORY.md; echo "- [Panda sim quirks](panda-sim-quirks.md) — IK frame/tracking/wrench/reach quirks of the /workspace Panda microwave sim" >> MEMORY.md; cat MEMORY.md)
