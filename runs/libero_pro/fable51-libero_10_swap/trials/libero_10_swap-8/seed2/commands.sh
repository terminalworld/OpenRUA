#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace; find /workspace -maxdepth 2 -type f | head -50

# openrua op 1
cat README.md machine.yaml docs/10-machine.md docs/20-perception.md docs/30-action.md docs/40-patterns.md tools/README.md; ls -la tools .ros

# openrua op 2
ls -la tools/action tools/perception; cat tools/perception/*.py tools/action/*.py

# openrua op 3
ros2 topic list; echo ---; ros2 action list; echo ---; ros2 service list | grep -v -E "parameter|describe|list_|get_type" ; echo ---; ros2 node list

# openrua op 4
mkdir -p snaps && for c in agentview birdview frontview sideview galleryview paperview robot0_robotview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c snaps/$c.png; done; ros2 topic echo /joint_states --once

# openrua op 5
ros2 topic echo /tf_static --once 2>/dev/null | grep -E "frame_id|translation|rotation" -A0 | head -80; echo ---; timeout 20 ros2 topic echo /tf --once 2>/dev/null | grep -E "frame_id" | head

# openrua op 6
cat > tf_dump.py <<'EOF'
import rclpy, sys
from tf2_msgs.msg import TFMessage
rclpy.init(); node = rclpy.create_node("tfd")
seen = {}
def cb(m):
    for t in m.transforms:
        seen[(t.header.frame_id, t.child_frame_id)] = t.transform
node.create_subscription(TFMessage, "/tf", cb, 50)
from rclpy.qos import QoSProfile, DurabilityPolicy
qos = QoSProfile(depth=50); qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
node.create_subscription(TFMessage, "/tf_static", cb, qos)
import time
t0=time.time()
while time.time()-t0 < 5: rclpy.spin_once(node, timeout_sec=0.2)
for k,v in sorted(seen.items()):
    tr, q = v.translation, v.rotation
    print(f"{k[0]:>22} -> {k[1]:<28} t=({tr.x:.4f},{tr.y:.4f},{tr.z:.4f}) q=({q.x:.4f},{q.y:.4f},{q.z:.4f},{q.w:.4f})")
EOF
python3 tf_dump.py

# openrua op 7
for p in "258 240" "390 285" "330 355" "330 308" "320 240" "200 300"; do echo -n "$p -> "; timeout 60 python3 tools/perception/px2world.py birdview $p; done

# openrua op 8
cat > scene.py <<'EOF'
"""Segment objects above the table from birdview depth; print world-frame clusters."""
import rclpy, numpy as np, sys
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge
from scipy import ndimage

def grab(node, topic, T):
    got={}
    s=node.create_subscription(T, topic, lambda m: got.setdefault("m",m), 1)
    while "m" not in got: rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(s); return got["m"]

rclpy.init(); node=rclpy.create_node("scene")
cam = sys.argv[1] if len(sys.argv)>1 else "birdview"
d = CvBridge().imgmsg_to_cv2(grab(node, f"/{cam}/depth/image_raw", Image), "passthrough").astype(float)
info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
col = CvBridge().imgmsg_to_cv2(grab(node, f"/{cam}/color/image_raw", Image), "bgr8")
fx,fy,cx,cy = info.k[0],info.k[4],info.k[2],info.k[5]
H,W = d.shape
v,u = np.mgrid[0:H,0:W]
X=(u-cx)*d/fx; Y=(v-cy)*d/fy; Z=d
# birdview: world = (-0.2,0,3.0), q=(0.7071,0.7071,0,0): R maps cam->world
q=np.array([0.7071,0.7071,0,0]); x,y,z,w=q
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
P = np.stack([X,Y,Z],-1) @ R.T + np.array([-0.2,0,3.0])
wz = P[...,2]
np.save("birdview_world.npy", P)
mask = (wz > 0.905) & (wz < 1.3) & np.isfinite(wz)
lab, n = ndimage.label(mask)
for i in range(1,n+1):
    m = lab==i
    if m.sum() < 15: continue
    pts = P[m]
    c = pts.mean(0); mn=pts.min(0); mx=pts.max(0)
    us,vs = u[m], v[m]
    print(f"cluster {i}: n={m.sum()} px=({us.mean():.0f},{vs.mean():.0f}) centroid=({c[0]:.3f},{c[1]:.3f}) "
          f"x[{mn[0]:.3f},{mx[0]:.3f}] y[{mn[1]:.3f},{mx[1]:.3f}] ztop={mx[2]:.3f} color={col[m].mean(0).astype(int)}")
EOF
python3 scene.py

# openrua op 9
cat > cloud.py <<'EOF'
"""World-frame point cloud from a camera: python3 cloud.py <cam> -> <cam>_cloud.npy (H,W,3) + <cam>_rgb.npy"""
import rclpy, numpy as np, sys
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge
from tf2_msgs.msg import TFMessage

def grab(node, topic, T):
    got={}
    s=node.create_subscription(T, topic, lambda m: got.setdefault("m",m), 1)
    while "m" not in got: rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(s); return got["m"]

def quat_R(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

def main():
    cam=sys.argv[1]
    rclpy.init(); node=rclpy.create_node("cloud")
    tfs={}
    node.create_subscription(TFMessage,"/tf",lambda m:[tfs.setdefault(t.child_frame_id,t.transform) for t in m.transforms],50)
    d = CvBridge().imgmsg_to_cv2(grab(node, f"/{cam}/depth/image_raw", Image), "passthrough").astype(float)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    col = CvBridge().imgmsg_to_cv2(grab(node, f"/{cam}/color/image_raw", Image), "bgr8")
    fr=f"{cam}_optical_frame"
    while fr not in tfs: rclpy.spin_once(node, timeout_sec=0.2)
    t=tfs[fr]; q=t.rotation; R=quat_R(q.x,q.y,q.z,q.w); o=np.array([t.translation.x,t.translation.y,t.translation.z])
    fx,fy,cx,cy = info.k[0],info.k[4],info.k[2],info.k[5]
    H,W=d.shape; v,u=np.mgrid[0:H,0:W]
    P=np.stack([(u-cx)*d/fx,(v-cy)*d/fy,d],-1)@R.T+o
    np.save(f"{cam}_cloud.npy",P); np.save(f"{cam}_rgb.npy",col)
    print(cam, P.shape, "cam at", o)
main()
EOF
python3 cloud.py agentview && python3 cloud.py sideview && python3 cloud.py birdview

# openrua op 10
cat > profile.py <<'EOF'
import numpy as np, sys
P=np.load(f"{sys.argv[1]}_cloud.npy").reshape(-1,3)
P=P[np.isfinite(P).all(1)]
# pot A region and pot B region boxes (xy), profile width per z-bin
boxes={"potA":(-0.30,-0.10,-0.32,-0.10),"potB":(-0.15,0.05,0.12,0.35),"stove":(0.09,0.33,-0.08,0.15)}
for name,(x0,x1,y0,y1) in boxes.items():
    m=(P[:,0]>x0)&(P[:,0]<x1)&(P[:,1]>y0)&(P[:,1]<y1)&(P[:,2]>0.903)
    Q=P[m]
    print(name, "n=",len(Q), "ztop=%.3f"%Q[:,2].max() if len(Q) else "")
    for z0 in np.arange(0.90,1.08,0.01):
        s=Q[(Q[:,2]>=z0)&(Q[:,2]<z0+0.01)]
        if len(s)<3: continue
        print(f"  z {z0:.2f}: n={len(s):4d} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}] cx={s[:,0].mean():.3f} cy={s[:,1].mean():.3f}")
EOF
python3 profile.py agentview; echo ======; python3 profile.py sideview

# openrua op 11
python3 -c "
import cv2
im=cv2.imread('snaps/sideview.png'); crop=im[200:400,240:420]; cv2.imwrite('snaps/side_crop.png', cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('snaps/agentview.png'); crop=im[160:280,150:260]; cv2.imwrite('snaps/agent_crop.png', cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))
"

# openrua op 12
ros2 node info /robot_bridge 2>&1 | head -80; ros2 param list /robot_bridge 2>&1 | head -30

# openrua op 13
cat > handle.py <<'EOF'
import numpy as np
for cam in ["birdview","agentview","sideview"]:
    P=np.load(f"{cam}_cloud.npy").reshape(-1,3); P=P[np.isfinite(P).all(1)]
    print("==",cam)
    for name,(x0,x1,y0,y1,yb) in {"A":(-0.26,-0.13,-0.32,-0.243,-0.243),"B":(-0.10,0.0,0.14,0.215,0.215)}.items():
        m=(P[:,0]>x0)&(P[:,0]<x1)&(P[:,1]>y0)&(P[:,1]<y1)&(P[:,2]>0.905)
        Q=P[m]
        print(f" handle {name}: n={len(Q)}")
        if len(Q)==0: continue
        for z0 in np.arange(0.94,1.05,0.01):
            s=Q[(Q[:,2]>=z0)&(Q[:,2]<z0+0.01)]
            if len(s)<2: continue
            print(f"   z {z0:.2f}: n={len(s):3d} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]")
EOF
python3 handle.py

# openrua op 14
mkdir -p "$(dirname /workspace/rob.py)"
cat > /workspace/rob.py <<'OPENRUA_EOF'
"""Persistent helper for this Panda: joint state, FK, IK, trajectory, gripper, servo."""
import math
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
TW = next(a for a in M["actuators"] if a["kind"] == "cartesian_twist")
JOINTS = FJT["joints"]
BASE = np.array([-0.66, 0.0, 0.912])  # world -> panda_link0
TCP = M["hand"]["tcp_offset_m"]

# hand pointing down, fingers closing along world x / world y
Q_DOWN_FX = (0.7071068, 0.7071068, 0.0, 0.0)
Q_DOWN_FY = (0.0, 1.0, 0.0, 0.0)


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def R_quat(R):
    w = math.sqrt(max(0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = math.sqrt(max(0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2
    y = math.sqrt(max(0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2
    z = math.sqrt(max(0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2
    x = math.copysign(x, R[2, 1] - R[1, 2])
    y = math.copysign(y, R[0, 2] - R[2, 0])
    z = math.copysign(z, R[1, 0] - R[0, 1])
    return (x, y, z, w)


def q_down_yaw(yaw):
    """Hand pointing down; finger axis rotated by yaw from world x."""
    fy = np.array([math.cos(yaw), math.sin(yaw), 0.0])  # hand y in world
    fz = np.array([0.0, 0.0, -1.0])
    fx = np.cross(fy, fz)
    return R_quat(np.column_stack([fx, fy, fz]))


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
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.tw = self.node.create_publisher(TwistStamped, TW["port"], 10)
        self.fjt.wait_for_server(20)
        self.grip.wait_for_server(20)
        self.ik.wait_for_service(20)
        self.fk.wait_for_service(20)

    def _on_js(self, m):
        self._js = m

    def spin(self, t=0.05):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self, fresh=True):
        if fresh:
            self._js = None
        while self._js is None:
            self.spin(0.1)
        d = dict(zip(self._js.name, self._js.position))
        return d

    def arm(self):
        d = self.joints()
        return [d[j] for j in JOINTS]

    def fingers(self):
        d = self.joints()
        return d["panda_finger_joint1"], d["panda_finger_joint2"]

    # ---- kinematics (planning frame = panda_link0) ----
    def fk_world(self, q=None):
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = list(JOINTS)
        req.robot_state.joint_state.position = list(q if q is not None else self.arm())
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        r = fut.result()
        p = r.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE
        return pos, (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)

    def ik_world(self, pos, quat, seed=None, at_tcp=False, timeout=20.0):
        pos = np.asarray(pos, float)
        if at_tcp:
            R = quat_R(*quat)
            pos = pos - TCP * R[:, 2]
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        req.ik_request.ik_link_name = "panda_hand"
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = (pos - BASE).tolist()
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = quat
        req.ik_request.robot_state.joint_state.name = list(JOINTS)
        req.ik_request.robot_state.joint_state.position = list(seed if seed is not None else self.arm())
        req.ik_request.timeout = Duration(sec=int(timeout), nanosec=int((timeout % 1) * 1e9))
        req.ik_request.avoid_collisions = False
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout + 30)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            return None
        sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
        return [sol[j] for j in JOINTS]

    # ---- motion ----
    def move_joints(self, q, seconds=3.0, via=None):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        pts = []
        wps = (via or []) + [q]
        for i, w in enumerate(wps):
            t = seconds * (i + 1) / len(wps)
            pt = JointTrajectoryPoint(positions=[float(x) for x in w])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res)
        code = res.result().result.error_code
        err = np.abs(np.array(self.arm()) - np.array(q)).max()
        return code, err

    def move_pose(self, pos, quat, seconds=3.0, at_tcp=False, seed=None):
        q = self.ik_world(pos, quat, seed=seed, at_tcp=at_tcp)
        if q is None:
            return None
        return self.move_joints(q, seconds)

    def gripper(self, width, timeout=120):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=timeout)
        r = res.result().result
        return r.reached_goal, r.stalled, self.fingers()

    def servo(self, v, n=20, dt=0.05):
        """Stream n twist messages with linear velocity v (m/s, base frame)."""
        msg = TwistStamped()
        msg.header.frame_id = TW["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = [float(x) for x in v]
        for _ in range(n):
            self.tw.publish(msg)
            self.spin(dt)
OPENRUA_EOF

# openrua op 15
timeout 300 python3 -u -c "
from rob import *
r=Robot('probe')
q=r.arm(); print('q',np.round(q,3))
pos,quat=r.fk_world(); print('hand world',np.round(pos,4),'quat',np.round(quat,4))
R=quat_R(*quat); print('hand x,y,z axes in world:\n',np.round(R.T,3))
print('q_down_yaw(0)=',np.round(q_down_yaw(0),4),' q_down_yaw(pi/2)=',np.round(q_down_yaw(math.pi/2),4))
" 2>&1 | grep -v Warning

# openrua op 16
timeout 600 python3 -u -c "
from rob import *
r=Robot('probe')
q=r.arm()
req = GetPositionFK.Request(); req.fk_link_names=['panda_hand','panda_link0']
req.robot_state.joint_state.name=list(JOINTS); req.robot_state.joint_state.position=q
fut=r.fk.call_async(req); rclpy.spin_until_future_complete(r.node,fut,timeout_sec=30); res=fut.result()
for ps in res.pose_stamped: print(ps.header.frame_id, np.round([ps.pose.position.x,ps.pose.position.y,ps.pose.position.z],4))
p=res.pose_stamped[0].pose
# IK round trip with raw FK pose (no offset)
raw=np.array([p.position.x,p.position.y,p.position.z]); quat=(p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w)
for label,pos in [('raw',raw),('raw-BASE',raw-BASE)]:
    sol=r.ik_world(pos+BASE,quat)   # ik_world subtracts BASE
    print(label, None if sol is None else np.round(sol,3))
" 2>&1 | grep -v Warning

# openrua op 17
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rob.py", "old_string": "BASE = np.array([-0.66, 0.0, 0.912])  # world -> panda_link0", "new_string": "BASE = np.zeros(3)  # verified: FK/IK poses are already in the world frame", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 18
timeout 900 python3 -u -c "
from rob import *
r=Robot('probe')
seed=r.arm()
tests={
 'potA knob top-down (TCP 1.05)':((-0.194,-0.205,1.05),Q_DOWN_FX),
 'potB knob top-down':((-0.048,0.251,1.05),Q_DOWN_FX),
 'potA handle top-down (TCP 1.00)':((-0.195,-0.275,1.00),Q_DOWN_FX),
 'place near x=0.165 knob z1.09':((0.165,0.036,1.09),Q_DOWN_FX),
 'place x=0.12 knob z1.09':((0.12,0.036,1.09),Q_DOWN_FX),
 'place x=0.10 handle z1.03':((0.10,0.0,1.03),Q_DOWN_FY),
 'place x=0.13 handle z1.03':((0.13,0.0,1.03),Q_DOWN_FY),
 'place x=0.16 handle z1.03':((0.16,0.0,1.03),Q_DOWN_FY),
 'place x=0.20 handle z1.03':((0.20,0.0,1.03),Q_DOWN_FY),
 'place x=0.20 z0.95':((0.20,0.0,0.95),Q_DOWN_FY),
 'place x=0.25 z0.95':((0.25,0.0,0.95),Q_DOWN_FY),
 'place x=0.30 z0.95':((0.30,0.0,0.95),Q_DOWN_FY),
}
for k,(p,q) in tests.items():
    sol=r.ik_world(p,q,seed=seed,at_tcp=True,timeout=5)
    print(f'{k:40s}', 'OK '+str(np.round(sol,2)) if sol else 'NO')
" 2>&1 | grep -v Warning

# openrua op 19
timeout 900 python3 -u -c "
from rob import *
r=Robot('probe')
seed=r.arm()
for yaw in (-math.pi/2, math.pi/2):
  q=q_down_yaw(yaw)
  for p in [(0.105,-0.012,1.04),(0.105,0.084,1.04),(0.105,-0.012,1.15),(0.105,0.084,1.15),(0.115,-0.012,1.04),(0.115,0.084,1.04)]:
    sol=r.ik_world(p,q,seed=seed,at_tcp=True,timeout=5)
    print(f'yaw={yaw:+.2f} {p}', 'OK '+str(np.round(sol,2)) if sol else 'NO')
# grasp poses for handle on table
for yaw in (0.0, math.pi):
  q=q_down_yaw(yaw)
  for p in [(-0.195,-0.279,1.00),(-0.195,-0.279,1.15),(-0.048,0.177,1.00),(-0.048,0.177,1.15)]:
    sol=r.ik_world(p,q,seed=seed,at_tcp=True,timeout=5)
    print(f'grasp yaw={yaw:+.2f} {p}', 'OK '+str(np.round(sol,2)) if sol else 'NO')
" 2>&1 | grep -v Warning

# openrua op 20
python3 - <<'EOF'
import re
s=open('rob.py').read()
old=s[s.index('def R_quat(R):'):s.index('def q_down_yaw')]
new='''def R_quat(R):
    from scipy.spatial.transform import Rotation
    return tuple(Rotation.from_matrix(R).as_quat())  # (x, y, z, w)


'''
s=s.replace(old,new)
open('rob.py','w').write(s)
EOF
python3 -c "
from rob import *
for y in (0,math.pi/2,math.pi,-math.pi/2): print(y, np.round(q_down_yaw(y),4), np.round(quat_R(*q_down_yaw(y)),2).tolist())"

# openrua op 21
mkdir -p "$(dirname /workspace/pick.py)"
cat > /workspace/pick.py <<'OPENRUA_EOF'
"""Pick a moka pot by its handle bar (top-down grasp) and lift.
Usage: python3 -u pick.py <hx> <hy> [grasp_z=1.00]
hx,hy = world xy of the handle's outer bar; hand yaw 0 (fingers close along x)."""
import sys
from rob import *

hx, hy = float(sys.argv[1]), float(sys.argv[2])
gz = float(sys.argv[3]) if len(sys.argv) > 3 else 1.00
Q = q_down_yaw(0.0)
r = Robot("pick")
print("start q", np.round(r.arm(), 3), flush=True)

print("open gripper ->", r.gripper(0.04), flush=True)

hover = (hx, hy, 1.15)
q1 = r.ik_world(hover, Q, at_tcp=True)
print("hover ik", None if q1 is None else np.round(q1, 3), flush=True)
print("move hover ->", r.move_joints(q1, 4.0), flush=True)
pos, _ = r.fk_world(); print("hand now", np.round(pos, 4), "tcp z", round(pos[2] - TCP, 4), flush=True)

q2 = r.ik_world((hx, hy, gz), Q, seed=q1, at_tcp=True)
print("grasp ik", None if q2 is None else np.round(q2, 3), flush=True)
print("descend ->", r.move_joints(q2, 3.0), flush=True)
pos, _ = r.fk_world(); print("hand now", np.round(pos, 4), "tcp z", round(pos[2] - TCP, 4), flush=True)

print("close gripper ->", r.gripper(0.0), flush=True)
print("fingers", r.fingers(), flush=True)

q3 = r.ik_world((hx, hy, gz + 0.08), Q, seed=q2, at_tcp=True)
print("lift ->", r.move_joints(q3, 2.5), flush=True)
pos, _ = r.fk_world(); print("hand now", np.round(pos, 4), "tcp z", round(pos[2] - TCP, 4), flush=True)
print("fingers after lift", r.fingers(), flush=True)
print("DONE", flush=True)
OPENRUA_EOF

# openrua op 22
nohup python3 -u pick.py -0.195 -0.279 1.00 > pickA.log 2>&1 &
sleep 45; cat /workspace/pickA.log | grep -v Warning

# openrua op 23
sleep 20; grep -v Warning /workspace/pickA.log; timeout 60 python3 tools/perception/cam_snap.py agentview snaps/a1.png; timeout 60 python3 tools/perception/cam_snap.py sideview snaps/s1.png

# openrua op 24
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rob.py", "old_string": "        code = res.result().result.error_code\\n        err = np.abs(np.array(self.arm()) - np.array(q)).max()\\n        return code, err", "new_string": "        code = res.result().result.error_code\\n        return code, self.settle(q)\\n\\n    def settle(self, q, tol=0.01, max_reads=200, resend=True):\\n        \\"\\"\\"Poll joint states until within tol of q; resend the goal once if stuck.\\"\\"\\"\\n        last = None\\n        stuck = 0\\n        for i in range(max_reads):\\n            cur = np.array(self.arm())\\n            err = np.abs(cur - np.array(q)).max()\\n            if err < tol:\\n                return err\\n            if last is not None and np.abs(cur - last).max() < 1e-4:\\n                stuck += 1\\n                if stuck >= 15:\\n                    if resend:\\n                        print(f\\"  settle: stuck at err={err:.3f}, resending goal\\", flush=True)\\n                        return self.move_joints(q, 2.0)[1] if False else self._resend(q)\\n                    return err\\n            else:\\n                stuck = 0\\n            last = cur\\n        return err\\n\\n    def _resend(self, q):\\n        goal = FollowJointTrajectory.Goal()\\n        goal.trajectory.joint_names = list(JOINTS)\\n        pt = JointTrajectoryPoint(positions=[float(x) for x in q])\\n        pt.time_from_start = Duration(sec=2, nanosec=0)\\n        goal.trajectory.points = [pt]\\n        send = self.fjt.send_goal_async(goal)\\n        rclpy.spin_until_future_complete(self.node, send)\\n        res = send.result().get_result_async()\\n        rclpy.spin_until_future_complete(self.node, res)\\n        return self.settle(q, resend=False)", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 25
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rob.py", "old_string": "                        return self.move_joints(q, 2.0)[1] if False else self._resend(q)", "new_string": "                        return self._resend(q)", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 26
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rob.py", "old_string": "        r = res.result().result\\n        return r.reached_goal, r.stalled, self.fingers()", "new_string": "        r = res.result().result\\n        # fingers keep moving after the result; poll until stable\\n        last, stable = None, 0\\n        for _ in range(100):\\n            f = self.fingers()\\n            if last is not None and abs(f[0] - last[0]) < 1e-5:\\n                stable += 1\\n                if stable >= 8:\\n                    break\\n            else:\\n                stable = 0\\n            last = f\\n        return r.reached_goal, r.stalled, f", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 27
mkdir -p "$(dirname /workspace/regrasp.py)"
cat > /workspace/regrasp.py <<'OPENRUA_EOF'
"""Re-attempt grasp from the current hover position: open, descend, close, lift.
Usage: python3 -u regrasp.py <hx> <hy> <grasp_z> [yaw=0]"""
import sys
from rob import *

hx, hy, gz = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])
yaw = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0
Q = q_down_yaw(yaw)
r = Robot("regrasp")
pos, _ = r.fk_world(); print("hand now", np.round(pos, 4), "tcp z", round(pos[2] - TCP, 4), flush=True)
print("open gripper ->", r.gripper(0.04), flush=True)

q0 = r.arm()
q2 = r.ik_world((hx, hy, gz), Q, seed=q0, at_tcp=True)
print("grasp ik", None if q2 is None else np.round(q2, 3), flush=True)
print("descend ->", r.move_joints(q2, 4.0), flush=True)
pos, _ = r.fk_world(); print("hand now", np.round(pos, 4), "tcp z", round(pos[2] - TCP, 4), flush=True)

print("close gripper ->", r.gripper(0.0), flush=True)
pos, _ = r.fk_world(); print("hand now", np.round(pos, 4), "tcp z", round(pos[2] - TCP, 4), flush=True)

q3 = r.ik_world((hx, hy, gz + 0.08), Q, seed=q2, at_tcp=True)
print("lift ->", r.move_joints(q3, 3.0), flush=True)
pos, _ = r.fk_world(); print("hand now", np.round(pos, 4), "tcp z", round(pos[2] - TCP, 4), flush=True)
print("fingers after lift", r.fingers(), flush=True)
print("DONE", flush=True)
OPENRUA_EOF

# openrua op 28
nohup python3 -u regrasp.py -0.195 -0.279 1.00 > regraspA.log 2>&1 &
for i in $(seq 1 24); do sleep 10; if grep -q DONE /workspace/regraspA.log; then break; fi; done; grep -v Warning /workspace/regraspA.log

# openrua op 29
timeout 60 python3 tools/perception/cam_snap.py agentview snaps/a2.png; timeout 60 python3 tools/perception/cam_snap.py sideview snaps/s2.png

# openrua op 30
mkdir -p "$(dirname /workspace/place.py)"
cat > /workspace/place.py <<'OPENRUA_EOF'
"""Carry the grasped pot to a place spot and release.
Usage: python3 -u place.py <hx> <hy> <place_tcp_z> <yaw_from> <yaw_to>
(hx,hy) = target world xy of the handle bar (TCP); descends to place_tcp_z, opens, retreats."""
import sys
from rob import *

hx, hy, pz = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])
yaw0, yaw1 = float(sys.argv[4]), float(sys.argv[5])
r = Robot("place")
pos, _ = r.fk_world(); print("hand now", np.round(pos, 4), "tcp z", round(pos[2] - TCP, 4), flush=True)
print("fingers", r.fingers(), flush=True)
start = pos - TCP * quat_R(*q_down_yaw(yaw0))[:, 2]  # current TCP
CARRY_Z = 1.20

def go(p, yaw, secs, seed):
    q = r.ik_world(p, q_down_yaw(yaw), seed=seed, at_tcp=True)
    if q is None:
        print("IK FAILED for", p, yaw, flush=True); sys.exit(2)
    code, err = r.move_joints(q, secs)
    pos, _ = r.fk_world()
    print(f"  -> {np.round(p,3)} yaw={yaw:.2f} code={code} err={err:.4f} tcp={np.round(pos - TCP*quat_R(*q_down_yaw(yaw))[:,2],4)} fingers={np.round(r.fingers(),4)}", flush=True)
    return q

seed = r.arm()
# 1. up to carry height
seed = go((start[0], start[1], CARRY_Z), yaw0, 3.0, seed)
# 2. interpolate xy + yaw in 4 steps at carry height
for k in range(1, 5):
    a = k / 4
    p = (start[0] + a * (hx - start[0]), start[1] + a * (hy - start[1]), CARRY_Z)
    seed = go(p, yaw0 + a * (yaw1 - yaw0), 3.0, seed)
# 3. descend
seed = go((hx, hy, pz + 0.06), yaw1, 3.0, seed)
seed = go((hx, hy, pz), yaw1, 3.0, seed)
print("open gripper ->", r.gripper(0.04), flush=True)
seed = go((hx, hy, pz + 0.10), yaw1, 3.0, seed)
print("DONE", flush=True)
OPENRUA_EOF

# openrua op 31
nohup python3 -u place.py 0.106 -0.012 1.034 0 -1.5708 > placeA.log 2>&1 &
for i in $(seq 1 60); do sleep 10; if grep -qE "DONE|FAILED|Error|Traceback" /workspace/placeA.log; then break; fi; done; grep -v Warning /workspace/placeA.log

# openrua op 32
timeout 60 python3 tools/perception/cam_snap.py agentview snaps/a3.png; timeout 60 python3 tools/perception/cam_snap.py sideview snaps/s3.png; timeout 60 python3 tools/perception/cam_snap.py birdview snaps/b3.png

# openrua op 33
timeout 120 python3 cloud.py birdview >/dev/null && python3 - <<'EOF'
import numpy as np
P=np.load("birdview_cloud.npy").reshape(-1,3); P=P[np.isfinite(P).all(1)]
# stove region objects above plate
m=(P[:,0]>0.10)&(P[:,0]<0.32)&(P[:,1]>-0.07)&(P[:,1]<0.14)&(P[:,2]>0.94)
Q=P[m]; print("on-plate pts", len(Q), "ztop=%.3f"%Q[:,2].max())
top=Q[Q[:,2]>1.07]; print("lid knob (z>1.07) centroid", np.round(top.mean(0),3), "n",len(top))
body=Q[(Q[:,2]>1.04)&(Q[:,2]<1.07)]; print("rim ring x[%.3f,%.3f] y[%.3f,%.3f]"%(body[:,0].min(),body[:,0].max(),body[:,1].min(),body[:,1].max()))
# pot B still on table?
m=(P[:,0]>-0.15)&(P[:,0]<0.05)&(P[:,1]>0.12)&(P[:,1]<0.35)&(P[:,2]>1.04)
Q=P[m]; print("potB knob centroid", np.round(Q[Q[:,2]>1.045].mean(0),3), "ztop %.3f"%Q[:,2].max())
EOF

# openrua op 34
python3 - <<'EOF'
import numpy as np
P=np.load("birdview_cloud.npy").reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,0]>0.10)&(P[:,0]<0.32)&(P[:,1]>-0.07)&(P[:,1]<0.14)&(P[:,2]>0.94)&(P[:,2]<1.12)
Q=P[m]; print("on-plate pts", len(Q), "ztop=%.3f"%Q[:,2].max())
for z0 in np.arange(0.94,1.12,0.01):
    s=Q[(Q[:,2]>=z0)&(Q[:,2]<z0+0.01)]
    if len(s)<3: continue
    print(f"  z {z0:.2f}: n={len(s):4d} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}] c=({s[:,0].mean():.3f},{s[:,1].mean():.3f})")
m=(P[:,0]>-0.15)&(P[:,0]<0.05)&(P[:,1]>0.12)&(P[:,1]<0.35)&(P[:,2]>1.04)&(P[:,2]<1.10)
Q=P[m]; print("potB knob centroid", np.round(Q[Q[:,2]>1.045].mean(0),3), "ztop %.3f"%Q[:,2].max())
EOF

# openrua op 35
timeout 120 python3 cloud.py sideview >/dev/null && python3 - <<'EOF'
import numpy as np
P=np.load("sideview_cloud.npy").reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,0]>0.08)&(P[:,0]<0.32)&(P[:,1]>-0.10)&(P[:,1]<0.14)&(P[:,2]>0.935)&(P[:,2]<1.12)
Q=P[m]; print("on-plate pts", len(Q), "ztop=%.3f"%Q[:,2].max())
for z0 in np.arange(0.93,1.12,0.01):
    s=Q[(Q[:,2]>=z0)&(Q[:,2]<z0+0.01)]
    if len(s)<3: continue
    print(f"  z {z0:.2f}: n={len(s):4d} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}] c=({s[:,0].mean():.3f},{s[:,1].mean():.3f})")
EOF

# openrua op 36
nohup python3 -u pick.py -0.048 0.177 1.00 > pickB.log 2>&1 &
for i in $(seq 1 60); do sleep 10; if grep -qE "DONE|FAILED|Error|Traceback" /workspace/pickB.log; then break; fi; done; grep -v Warning /workspace/pickB.log

# openrua op 37
nohup python3 -u place.py 0.118 0.084 1.038 0 -1.5708 > placeB.log 2>&1 &
for i in $(seq 1 60); do sleep 10; if grep -qE "DONE|FAILED|Error|Traceback" /workspace/placeB.log; then break; fi; done; grep -v Warning /workspace/placeB.log

# openrua op 38
timeout 600 python3 -u -c "
from rob import *
r=Robot('home')
q=[0.0,-0.161,0.0,-2.445,0.0,2.227,0.785]
print('home ->', r.move_joints(q,4.0))
pos,_=r.fk_world(); print('hand', np.round(pos,3))
" 2>&1 | grep -v Warning; for c in agentview sideview birdview frontview; do timeout 60 python3 tools/perception/cam_snap.py $c snaps/${c}_final.png; done

# openrua op 39
timeout 120 python3 cloud.py birdview >/dev/null && python3 - <<'EOF'
import numpy as np
from scipy import ndimage
P=np.load("birdview_cloud.npy"); 
Z=P[...,2]
plate=(Z>0.92)&(Z<0.94)&(P[...,0]>0.05)&(P[...,0]<0.35)&(np.abs(P[...,1]-0.036)<0.15)
pp=P[plate]; print("plate top: x[%.3f,%.3f] y[%.3f,%.3f] z=%.3f"%(pp[:,0].min(),pp[:,0].max(),pp[:,1].min(),pp[:,1].max(),pp[:,2].mean()))
mask=(Z>1.04)&(Z<1.10)&np.isfinite(Z)
lab,n=ndimage.label(mask)
for i in range(1,n+1):
    m=lab==i
    if m.sum()<20: continue
    q=P[m]; top=q[q[:,2]>q[:,2].max()-0.012]
    print(f"pot: n={m.sum()} knob=({top[:,0].mean():.3f},{top[:,1].mean():.3f}) ztop={q[:,2].max():.3f} body x[{q[:,0].min():.3f},{q[:,0].max():.3f}] y[{q[:,1].min():.3f},{q[:,1].max():.3f}]")
# anything left on the table at the original pot spots?
for name,(x,y) in {"A_orig":(-0.194,-0.205),"B_orig":(-0.052,0.251)}.items():
    m=(np.abs(P[...,0]-x)<0.06)&(np.abs(P[...,1]-y)<0.06)&(Z>0.905)
    print(name,"points above table:",int(m.sum()))
EOF
