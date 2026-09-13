#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
cat /workspace/tools/action/fjt_send.py; echo ----; cat /workspace/tools/action/gripper_cmd.py; echo ----; cat /workspace/tools/action/ik_move.py

# openrua op 3
ros2 topic list 2>&1; echo ----; ros2 action list 2>&1; echo ----; ros2 service list 2>&1 | head -40; echo ----; ros2 node list 2>&1

# openrua op 4
for c in agentview frontview birdview sideview robot0_eye_in_hand robot0_robotview; do timeout 60 python3 tools/perception/cam_snap.py $c; done; ros2 topic echo /joint_states --once 2>&1 | head -40

# openrua op 5
python3 -c "
import cv2
im=cv2.imread('robot0_robotview.png')
cv2.imwrite('crop1.png', cv2.resize(im[60:200,0:250],None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('agentview.png')
cv2.imwrite('crop2.png', cv2.resize(im[190:340,70:270],None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
"

# openrua op 6
timeout 30 ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id|x:|y:|z:|w:" | head -80

# openrua op 7
cat > tfdump.py <<'EOF'
import rclpy, yaml
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
rclpy.init(); node=rclpy.create_node("tfdump")
seen={}
def cb(m):
    for t in m.transforms:
        seen[(t.header.frame_id,t.child_frame_id)]=t.transform
qos=QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)
node.create_subscription(TFMessage,"/tf_static",cb,qos)
node.create_subscription(TFMessage,"/tf",cb,10)
import time
for _ in range(40): rclpy.spin_once(node,timeout_sec=0.1)
for k,v in sorted(seen.items()):
    print(k, f"t=({v.translation.x:.4f},{v.translation.y:.4f},{v.translation.z:.4f}) q=({v.rotation.x:.4f},{v.rotation.y:.4f},{v.rotation.z:.4f},{v.rotation.w:.4f})")
EOF
timeout 60 python3 tfdump.py

# openrua op 8
cat > pxw.py <<'EOF'
"""pxw.py <camera> u,v [u,v ...]  -> world xyz per pixel (reuses one node)."""
import sys, struct, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge
cam=sys.argv[1]; pts=[tuple(map(int,a.split(','))) for a in sys.argv[2:]]
rclpy.init(); node=rclpy.create_node("pxw"); buf=Buffer(); TransformListener(buf,node)
got={}
node.create_subscription(Image,f"/{cam}/depth/image_raw",lambda m:got.setdefault('d',m),1)
node.create_subscription(CameraInfo,f"/{cam}/color/camera_info",lambda m:got.setdefault('i',m),1)
while not('d' in got and 'i' in got): rclpy.spin_once(node,timeout_sec=0.2)
depth=CvBridge().imgmsg_to_cv2(got['d'],"passthrough").astype(float)
np.save(f"{cam}_depth.npy",depth)
k=got['i'].k; fx,fy,cx,cy=k[0],k[4],k[2],k[5]
frame=f"{cam}_optical_frame"
while not buf.can_transform("world",frame,rclpy.time.Time()): rclpy.spin_once(node,timeout_sec=0.2)
t=buf.lookup_transform("world",frame,rclpy.time.Time()); q=t.transform.rotation
x,y,z,w=q.x,q.y,q.z,q.w
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
T=np.array([t.transform.translation.x,t.transform.translation.y,t.transform.translation.z])
print("depth shape",depth.shape,"fx",fx,"cx",cx,"cy",cy)
for u,v in pts:
    Z=depth[v,u]
    p=R@np.array([(u-cx)*Z/fx,(v-cy)*Z/fy,Z])+T
    print(f"({u},{v}) depth={Z:.3f} -> world {p[0]:.4f} {p[1]:.4f} {p[2]:.4f}")
rclpy.shutdown()
EOF
timeout 90 python3 pxw.py agentview 115,275 115,300 228,210 228,240 530,270 530,190 460,300 600,300 320,420 240,300

# openrua op 9
python3 -c "
import cv2
im=cv2.imread('birdview.png')
c=im[200:340,220:420]
c=cv2.resize(c,None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC)
# grid lines every 10 px original
for i in range(0,c.shape[1],40): cv2.line(c,(i,0),(i,c.shape[0]),(0,255,0),1); cv2.putText(c,str(220+i//4),(i,12),0,0.35,(0,255,0),1)
for j in range(0,c.shape[0],40): cv2.line(c,(0,j),(c.shape[1],j),(0,255,0),1); cv2.putText(c,str(200+j//4),(0,j+12),0,0.35,(0,255,0),1)
cv2.imwrite('bird_crop.png',c)
"

# openrua op 10
timeout 90 python3 pxw.py birdview 280,228 257,280 380,305 362,305 398,305 380,275 380,335 320,330 280,257

# openrua op 11
python3 -c "
import numpy as np
d=np.load('birdview_depth.npy')
fx=579.4112549695428; cx=320; cy=240
# camera at (-0.2,0,3) looking down; world x = -0.2 - (v-cy)*Z/fx ? verify with known: (320,330)->x=0.2  => x = -0.2 + (v-cy)*Z/fx ... (330-240)*2.575/579=0.40?? no.
# use the rotation: compute properly
q=np.array([0.7071,0.7071,0,0]); x,y,z,w=q
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
T=np.array([-0.2,0,3.0])
def w(u,v):
    Z=d[v,u]; return R@np.array([(u-cx)*Z/fx,(v-cy)*Z/fx,Z])+T
# basket region
reg=d[255:350,345:415]
mask=reg<2.45
vs,us=np.where(mask); vs+=255; us+=345
pts=np.array([w(u,v) for u,v in zip(us,vs)])
print('basket rim/wall pts: x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]'%(pts[:,0].min(),pts[:,0].max(),pts[:,1].min(),pts[:,1].max(),pts[:,2].min(),pts[:,2].max()))
print('center approx', (pts[:,0].min()+pts[:,0].max())/2, (pts[:,1].min()+pts[:,1].max())/2)
# alphabet soup
for name,(u0,u1,v0,v1) in {'alpha':(268,295,215,242),'tomato':(243,272,265,295)}.items():
    reg=d[v0:v1,u0:u1]; m=reg<2.53
    vs,us=np.where(m); vs+=v0; us+=u0
    pts=np.array([w(u,v) for u,v in zip(us,vs)])
    print(name,'x[%.3f,%.3f] y[%.3f,%.3f] ztop %.3f center (%.3f,%.3f) n=%d'%(pts[:,0].min(),pts[:,0].max(),pts[:,1].min(),pts[:,1].max(),pts[:,2].max(),pts[:,0].mean(),pts[:,1].mean(),len(pts)))
"


# openrua op 12
python3 -c "
import numpy as np
d=np.load('birdview_depth.npy')
fx=579.4112549695428; cx=320; cy=240
q=np.array([0.7071,0.7071,0,0]); x,y,z,w=q
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
T=np.array([-0.2,0,3.0])
def w(u,v):
    Z=d[v,u]; return R@np.array([(u-cx)*Z/fx,(v-cy)*Z/fx,Z])+T
reg=d[255:350,345:415]
vs,us=np.where((reg<2.45)&(reg>2.3)); vs+=255; us+=345
pts=np.array([w(u,v) for u,v in zip(us,vs)])
pts=pts[pts[:,1]>0.12]
print('basket rim pts: x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]'%(pts[:,0].min(),pts[:,0].max(),pts[:,1].min(),pts[:,1].max(),pts[:,2].min(),pts[:,2].max()))
print('center approx', (pts[:,0].min()+pts[:,0].max())/2, (pts[:,1].min()+pts[:,1].max())/2)
# floor of basket: deeper pixels inside
reg=d[280:335,362:400]; print('inside depth min/max', reg.min(), reg.max())
vs,us=np.where(reg>2.5); vs+=280; us+=362
pts=np.array([w(u,v) for u,v in zip(us,vs)]); print('basket floor z', pts[:,2].min(), pts[:,2].max(), 'x',pts[:,0].min(),pts[:,0].max(),'y',pts[:,1].min(),pts[:,1].max())
"


# openrua op 13
mkdir -p "$(dirname /workspace/rob.py)"
cat > /workspace/rob.py <<'OPENRUA_EOF'
"""Small helper library: one node, reusable clients. World-frame API.

Usage from a script:
    from rob import Robot
    r = Robot()
    r.gripper(0.04)
    r.move_tcp(x, y, z, yaw=0.0, secs=3)   # top-down grasp orientation
    print(r.hand_pose(), r.fingers())
"""
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
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]
TCP = float(M["hand"]["tcp_offset_m"])
BASE_IN_WORLD = np.array([-0.51, 0.0, 0.42])  # world -> panda_link0 (from /tf)


def log(*a):
    print(time.strftime("%H:%M:%S"), *a, flush=True)


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def topdown_quat(yaw):
    """Hand z pointing down (-world z), fingers closing along world y
    rotated by yaw about z. q = Rz(yaw) * (1,0,0,0)."""
    c, s = math.cos(yaw / 2), math.sin(yaw / 2)
    # Rz(yaw) = (0,0,s,c); (1,0,0,0) ; product (w1w2 - v1.v2, w1 v2 + w2 v1 + v1 x v2)
    # q1=(0,0,s,c) q2=(1,0,0,0)
    w = c * 0 - (0 * 1 + 0 * 0 + s * 0)
    v = c * np.array([1, 0, 0]) + 0 * np.array([0, 0, s]) + np.cross([0, 0, s], [1, 0, 0])
    return (v[0], v[1], v[2], w)


class Robot:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("rob")
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self._js.__setitem__("m", m), 1)
        self.tfbuf = Buffer()
        TransformListener(self.tfbuf, self.node)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        assert self.fjt.wait_for_server(timeout_sec=20), "no FJT server"
        assert self.grip.wait_for_server(timeout_sec=20), "no gripper server"
        assert self.ik.wait_for_service(timeout_sec=20), "no IK service"
        self.spin(0.5)
        log("robot ready")

    def spin(self, secs):
        end = time.time() + secs
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    # ---- sensing ----
    def joints(self, fresh=True):
        if fresh:
            self._js.pop("m", None)
            while "m" not in self._js:
                rclpy.spin_once(self.node, timeout_sec=0.2)
        m = self._js["m"]
        return dict(zip(m.name, m.position))

    def arm_q(self):
        j = self.joints()
        return [j[n] for n in ARM]

    def fingers(self):
        j = self.joints()
        return j.get("panda_finger_joint1"), j.get("panda_finger_joint2")

    def hand_pose(self):
        """world -> panda_hand (t, q) from TF; TCP point also returned."""
        for _ in range(50):
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.tfbuf.can_transform("world", "panda_hand", rclpy.time.Time()):
                break
        t = self.tfbuf.lookup_transform("world", "panda_hand", rclpy.time.Time())
        tr, q = t.transform.translation, t.transform.rotation
        p = np.array([tr.x, tr.y, tr.z])
        R = quat_to_R(q.x, q.y, q.z, q.w)
        tcp = p + TCP * R[:, 2]
        return p, (q.x, q.y, q.z, q.w), tcp

    # ---- IK ----
    def solve_ik(self, x, y, z, quat, at_tcp=True, seed=None, timeout=60):
        """x,y,z in WORLD; returns joint list (manifest order) or None."""
        p = np.array([x, y, z], float)
        if at_tcp:
            R = quat_to_R(*quat)
            p = p - TCP * R[:, 2]
        p = p - BASE_IN_WORLD
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        ps = req.ik_request.pose_stamped.pose
        ps.position.x, ps.position.y, ps.position.z = map(float, p)
        ps.orientation.x, ps.orientation.y, ps.orientation.z, ps.orientation.w = map(float, quat)
        req.ik_request.avoid_collisions = False
        js = JointState()
        seed = seed if seed is not None else self.arm_q()
        js.name = list(ARM)
        js.position = [float(v) for v in seed]
        req.ik_request.robot_state.joint_state = js
        req.ik_request.timeout.sec = 2
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        res = fut.result()
        if res is None:
            log("IK: no answer")
            return None
        if res.error_code.val != 1:
            log(f"IK failed code={res.error_code.val} for world {x:.3f},{y:.3f},{z:.3f}")
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[n] for n in ARM]

    # ---- motion ----
    def move_joints(self, q, secs=3.0, retries=1):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pt = JointTrajectoryPoint(positions=[float(v) for v in q])
        pt.time_from_start = Duration(sec=int(secs), nanosec=int((secs % 1) * 1e9))
        goal.trajectory.points = [pt]
        for attempt in range(retries + 1):
            send = self.fjt.send_goal_async(goal)
            rclpy.spin_until_future_complete(self.node, send)
            res = send.result().get_result_async()
            rclpy.spin_until_future_complete(self.node, res)
            code = res.result().result.error_code
            cur = self.arm_q()
            err = max(abs(a - b) for a, b in zip(cur, q))
            log(f"traj code={code} max_joint_err={err:.4f}")
            if code == 0 and err < 0.02:
                return True
            if attempt < retries:
                log("retrying trajectory")
        return err < 0.05

    def move_tcp(self, x, y, z, yaw=0.0, secs=3.0, quat=None):
        quat = quat or topdown_quat(yaw)
        q = self.solve_ik(x, y, z, quat)
        if q is None:
            return False
        ok = self.move_joints(q, secs)
        _, _, tcp = self.hand_pose()
        log(f"tcp now {tcp.round(4)} target {np.array([x, y, z]).round(4)} "
            f"err={np.linalg.norm(tcp - [x, y, z]):.4f}")
        return ok

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP.get("max_effort", 30.0))
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=300)
        r = res.result().result
        self.spin(0.3)
        f = self.fingers()
        log(f"gripper -> {width}: reached={r.reached_goal} stalled={r.stalled} fingers={f}")
        return f
OPENRUA_EOF

# openrua op 14
cat > dry.py <<'EOF'
from rob import *
r = Robot()
p, q, tcp = r.hand_pose()
log("hand", p.round(3), "quat", np.round(q,3), "tcp", tcp.round(3))
log("fingers", r.fingers())
for name,(x,y,z) in {"soup_pre":(-0.244,-0.173,0.60),"soup_grasp":(-0.244,-0.173,0.45),
                     "tom_pre":(-0.023,-0.274,0.62),"tom_grasp":(-0.023,-0.274,0.46),
                     "basket_a":(0.0,0.215,0.72),"basket_b":(0.0,0.285,0.72)}.items():
    for yaw in (0.0, 0.785, -0.785, 1.571):
        sol = r.solve_ik(x,y,z,topdown_quat(yaw))
        log(name, "yaw",yaw, None if sol is None else np.round(sol,3))
        if sol is not None: break
EOF
timeout 600 python3 -u dry.py 2>&1 | tail -30

# openrua op 15
timeout 600 python3 -u dry.py 2>&1 | head -12

# openrua op 16
cat > iktest.py <<'EOF'
from rob import *
r = Robot()
p, q, tcp = r.hand_pose()
log("hand", p.round(4), "quat", np.round(q,4))
# 1) exact current hand pose, at hand (not tcp)
log("A current hand pose:", r.solve_ik(*p, q, at_tcp=False))
# 2) with tcp flag
log("B current tcp pose:", r.solve_ik(*tcp, q, at_tcp=True))
# 3) link8-style quat (rotate by -45deg about z): q_hand * (0,0,0.3827,0.9239)
import math
def qmul(a,b):
    x1,y1,z1,w1=a; x2,y2,z2,w2=b
    return (w1*x2+x1*w2+y1*z2-z1*y2, w1*y2-x1*z2+y1*w2+z1*x2, w1*z2+x1*y2-y1*x2+z1*w2, w1*w2-x1*x2-y1*y2-z1*z2)
q8 = qmul(q,(0,0,0.38268343,0.92387953))
log("C link8 quat:", r.solve_ik(*p, q8, at_tcp=False))
# 4) frame_id variants
for fid in ("panda_link0","world"):
    req_frame = fid
    from moveit_msgs.srv import GetPositionIK
    req = GetPositionIK.Request(); req.ik_request.group_name="panda_arm"
    req.ik_request.pose_stamped.header.frame_id=fid
    pb = p - BASE_IN_WORLD if fid=="panda_link0" else p
    ps=req.ik_request.pose_stamped.pose
    ps.position.x,ps.position.y,ps.position.z=map(float,pb)
    ps.orientation.x,ps.orientation.y,ps.orientation.z,ps.orientation.w=map(float,q)
    js=JointState(); js.name=list(ARM); js.position=[float(v) for v in r.arm_q()]
    req.ik_request.robot_state.joint_state=js
    fut=r.ik.call_async(req); rclpy.spin_until_future_complete(r.node,fut,timeout_sec=60)
    res=fut.result(); log("D frame",fid,"code",res.error_code.val if res else None, dict(zip(res.solution.joint_state.name,np.round(res.solution.joint_state.position,3))) if res and res.error_code.val==1 else "")
EOF
timeout 600 python3 -u iktest.py 2>&1

# openrua op 17
cat > iktest2.py <<'EOF'
from rob import *
r = Robot()
p, q, tcp = r.hand_pose()
def ik(fid, timeout=None, avoid=None, pos=None, quat=None):
    req = GetPositionIK.Request(); req.ik_request.group_name="panda_arm"
    req.ik_request.pose_stamped.header.frame_id=fid
    pb = pos if pos is not None else p
    ps=req.ik_request.pose_stamped.pose
    ps.position.x,ps.position.y,ps.position.z=map(float,pb)
    ps.orientation.x,ps.orientation.y,ps.orientation.z,ps.orientation.w=map(float,quat or q)
    js=JointState(); js.name=list(ARM); js.position=[float(v) for v in r.arm_q()]
    req.ik_request.robot_state.joint_state=js
    if timeout is not None: req.ik_request.timeout.sec=timeout
    if avoid is not None: req.ik_request.avoid_collisions=avoid
    fut=r.ik.call_async(req); rclpy.spin_until_future_complete(r.node,fut,timeout_sec=60)
    res=fut.result(); 
    return res.error_code.val, (np.round([dict(zip(res.solution.joint_state.name,res.solution.joint_state.position))[n] for n in ARM],3) if res.error_code.val==1 else None)
log("empty frame, world pos:", ik(""))
log("empty frame, base pos:", ik("", pos=p-BASE_IN_WORLD))
log("empty frame, timeout=2:", ik("", timeout=2))
log("world, timeout=2:", ik("world", timeout=2))
log("world, avoid=False:", ik("world", avoid=False))
# check link8 hypothesis: give q8 with world frame -> expect joint7 ~ 0.785 (current)
def qmul(a,b):
    x1,y1,z1,w1=a; x2,y2,z2,w2=b
    return (w1*x2+x1*w2+y1*z2-z1*y2, w1*y2-x1*z2+y1*w2+z1*x2, w1*z2+x1*y2-y1*x2+z1*w2, w1*w2-x1*x2-y1*y2-z1*z2)
log("world, q8:", ik("world", quat=qmul(q,(0,0,0.38268343,0.92387953))))
log("current q:", np.round(r.arm_q(),3))
EOF
timeout 600 python3 -u iktest2.py 2>&1

# openrua op 18
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rob.py", "old_string": "        p = np.array([x, y, z], float)\\n        if at_tcp:\\n            R = quat_to_R(*quat)\\n            p = p - TCP * R[:, 2]\\n        p = p - BASE_IN_WORLD\\n        req = GetPositionIK.Request()\\n        req.ik_request.group_name = M[\\"planning\\"][\\"group\\"]\\n        req.ik_request.pose_stamped.header.frame_id = \\"\\"\\n        ps = req.ik_request.pose_stamped.pose\\n        ps.position.x, ps.position.y, ps.position.z = map(float, p)\\n        ps.orientation.x, ps.orientation.y, ps.orientation.z, ps.orientation.w = map(float, quat)\\n        req.ik_request.avoid_collisions = False", "new_string": "        p = np.array([x, y, z], float)\\n        if at_tcp:\\n            R = quat_to_R(*quat)\\n            p = p - TCP * R[:, 2]\\n        # machine facts (measured): IK poses are in WORLD on this machine,\\n        # and the group\'s tip link is panda_link8 = hand rotated +45deg\\n        # about z (link8->hand static TF is Rz(-45deg)).\\n        q8 = qmul(quat, (0.0, 0.0, 0.38268343, 0.92387953))\\n        req = GetPositionIK.Request()\\n        req.ik_request.group_name = M[\\"planning\\"][\\"group\\"]\\n        req.ik_request.pose_stamped.header.frame_id = \\"world\\"\\n        ps = req.ik_request.pose_stamped.pose\\n        ps.position.x, ps.position.y, ps.position.z = map(float, p)\\n        ps.orientation.x, ps.orientation.y, ps.orientation.z, ps.orientation.w = map(float, q8)", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 19
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rob.py", "old_string": "def topdown_quat(yaw):", "new_string": "def qmul(a, b):\\n    x1, y1, z1, w1 = a\\n    x2, y2, z2, w2 = b\\n    return (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,\\n            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,\\n            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,\\n            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)\\n\\n\\ndef topdown_quat(yaw):", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 20
timeout 600 python3 -u dry.py 2>&1

# openrua op 21
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rob.py", "old_string": "    def gripper(self, width):", "new_string": "    def move_tcp_line(self, x, y, z, yaw=0.0, secs=3.0, n=4, quat=None):\\n        \\"\\"\\"Straight-line TCP motion from the current TCP to (x,y,z): IK on\\n        n waypoints (each seeded with the previous) in one trajectory.\\"\\"\\"\\n        quat = quat or topdown_quat(yaw)\\n        _, _, tcp0 = self.hand_pose()\\n        target = np.array([x, y, z], float)\\n        seed = self.arm_q()\\n        qs = []\\n        for i in range(1, n + 1):\\n            wp = tcp0 + (target - tcp0) * i / n\\n            q = self.solve_ik(*wp, quat, seed=seed)\\n            if q is None:\\n                return False\\n            qs.append(q)\\n            seed = q\\n        goal = FollowJointTrajectory.Goal()\\n        goal.trajectory.joint_names = list(ARM)\\n        for i, q in enumerate(qs, 1):\\n            pt = JointTrajectoryPoint(positions=[float(v) for v in q])\\n            t = secs * i / n\\n            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))\\n            goal.trajectory.points.append(pt)\\n        send = self.fjt.send_goal_async(goal)\\n        rclpy.spin_until_future_complete(self.node, send)\\n        res = send.result().get_result_async()\\n        rclpy.spin_until_future_complete(self.node, res)\\n        code = res.result().result.error_code\\n        cur = self.arm_q()\\n        err = max(abs(a - b) for a, b in zip(cur, qs[-1]))\\n        _, _, tcp = self.hand_pose()\\n        log(f\\"line traj code={code} max_joint_err={err:.4f} tcp now {tcp.round(4)} \\"\\n            f\\"target {target.round(4)} err={np.linalg.norm(tcp - target):.4f}\\")\\n        return code == 0 and err < 0.05\\n\\n    def gripper(self, width):", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 22
cat > phase1.py <<'EOF'
# Pick the alphabet soup: open, pre-grasp above, straight descent, close, lift.
from rob import *
r = Robot()
SX, SY = -0.244, -0.173
r.gripper(0.04)
assert r.move_tcp(SX, SY, 0.62, yaw=0.0, secs=4), "pre-grasp failed"
assert r.move_tcp_line(SX, SY, 0.455, yaw=0.0, secs=3, n=4), "descent failed"
f = r.gripper(0.0)
gap = f[0] - f[1]
log(f"finger gap after close = {gap:.4f} (closed_m=0 -> nothing held)")
assert r.move_tcp_line(SX, SY, 0.72, yaw=0.0, secs=3, n=3), "lift failed"
log("fingers after lift", r.fingers())
log("PHASE1 DONE")
EOF
nohup python3 -u phase1.py > phase1.log 2>&1 &
echo started

# openrua op 23
sleep 45; cat phase1.log

# openrua op 24
sleep 40; cat phase1.log

# openrua op 25
timeout 60 python3 tools/perception/cam_snap.py agentview a1.png; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand e1.png; timeout 60 python3 tools/perception/cam_snap.py sideview s1.png

# openrua op 26
cat > diag.py <<'EOF'
from rob import *
r = Robot()
q0 = r.arm_q(); log("current q", np.round(q0,3))
_,_,tcp = r.hand_pose(); log("tcp", tcp.round(4))
seed=q0
for z in (0.50, 0.485, 0.47, 0.455):
    q = r.solve_ik(-0.244,-0.173,z, topdown_quat(0.0), seed=seed)
    log(z, None if q is None else np.round(q,3), "jump", None if q is None else np.round(max(abs(a-b) for a,b in zip(q,seed)),3))
    if q: seed=q
# also try alternative yaws for the grasp height, seeded from current
for yaw in (0.4, -0.4, 0.785, -0.785, 1.571):
    q = r.solve_ik(-0.244,-0.173,0.455, topdown_quat(yaw), seed=q0)
    log("yaw",yaw, None if q is None else np.round(q,3), "jump", None if q is None else np.round(max(abs(a-b) for a,b in zip(q,q0)),3))
EOF
timeout 300 python3 -u diag.py 2>&1

# openrua op 27
cat > phase1b.py <<'EOF'
from rob import *
r = Robot()
SX, SY = -0.244, -0.173
log("fingers", r.fingers())
ok = r.move_tcp_line(SX, SY, 0.455, yaw=0.0, secs=4, n=3)
if not ok:
    log("retry descent")
    ok = r.move_tcp_line(SX, SY, 0.455, yaw=0.0, secs=4, n=2)
_,_,tcp = r.hand_pose(); log("tcp before close", tcp.round(4))
f = r.gripper(0.0)
log(f"finger gap after close = {f[0]-f[1]:.4f}")
ok2 = r.move_tcp_line(SX, SY, 0.72, yaw=0.0, secs=4, n=3)
if not ok2:
    log("retry lift"); r.move_tcp_line(SX, SY, 0.72, yaw=0.0, secs=4, n=2)
log("fingers after lift", r.fingers())
log("PHASE1B DONE")
EOF
nohup python3 -u phase1b.py > phase1b.log 2>&1 &
sleep 60; cat phase1b.log

# openrua op 28
sleep 20; cat phase1b.log | tail -4; timeout 60 python3 tools/perception/cam_snap.py agentview a2.png; timeout 60 python3 tools/perception/cam_snap.py robot0_robotview r2.png

# openrua op 29
cat > diag2.py <<'EOF'
from rob import *
from geometry_msgs.msg import WrenchStamped
r = Robot()
wr={}
r.node.create_subscription(WrenchStamped, "/franka_robot_state_broadcaster/external_wrench", lambda m: wr.__setitem__('m',m), 1)
def wrench():
    wr.pop('m',None)
    for _ in range(50):
        rclpy.spin_once(r.node,timeout_sec=0.1)
        if 'm' in wr: break
    if 'm' not in wr: return None
    f=wr['m'].wrench.force; t=wr['m'].wrench.torque
    return np.round([f.x,f.y,f.z,t.x,t.y,t.z],2)
log("wrench at start", wrench())
q0=r.arm_q(); log("q0",np.round(q0,3))
_,_,tcp=r.hand_pose(); log("tcp",tcp.round(4))
for z in (0.60, 0.55, 0.52, 0.50):
    q = r.solve_ik(-0.244,-0.173,z, topdown_quat(0.0))
    log("target z",z,"q",np.round(q,3))
    ok = r.move_joints(q, secs=3, retries=0)
    cur=r.arm_q()
    log("per-joint err", np.round(np.array(cur)-np.array(q),3))
    _,_,tcp=r.hand_pose(); log("tcp",tcp.round(4), "wrench", wrench())
    if not ok: break
log("DIAG2 DONE")
EOF
nohup python3 -u diag2.py > diag2.log 2>&1 &
sleep 75; cat diag2.log

# openrua op 30
cat > refine.py <<'EOF'
"""refine.py x y zlo zhi -> centroid of eye-in-hand depth points near (x,y) with world z in [zlo,zhi]."""
import sys
from rob import *
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge
def refine(r, x0, y0, zlo, zhi, rad=0.06, cam="robot0_eye_in_hand"):
    got={}
    s1=r.node.create_subscription(Image,f"/{cam}/depth/image_raw",lambda m:got.setdefault('d',m),1)
    s2=r.node.create_subscription(CameraInfo,f"/{cam}/color/camera_info",lambda m:got.setdefault('i',m),1)
    while not('d' in got and 'i' in got): rclpy.spin_once(r.node,timeout_sec=0.2)
    r.node.destroy_subscription(s1); r.node.destroy_subscription(s2)
    d=CvBridge().imgmsg_to_cv2(got['d'],"passthrough").astype(float)
    k=got['i'].k; fx,fy,cx,cy=k[0],k[4],k[2],k[5]
    frame=f"{cam}_optical_frame"
    for _ in range(50):
        rclpy.spin_once(r.node,timeout_sec=0.1)
        if r.tfbuf.can_transform("world",frame,rclpy.time.Time()): break
    t=r.tfbuf.lookup_transform("world",frame,rclpy.time.Time()); q=t.transform.rotation
    R=quat_to_R(q.x,q.y,q.z,q.w); T=np.array([t.transform.translation.x,t.transform.translation.y,t.transform.translation.z])
    log("cam at",T.round(4))
    vs,us=np.mgrid[0:d.shape[0],0:d.shape[1]]
    Z=d; X=(us-cx)*Z/fx; Y=(vs-cy)*Z/fy
    P=np.stack([X,Y,Z],-1).reshape(-1,3)@R.T+T
    ok=np.isfinite(P).all(1)&(P[:,2]>zlo)&(P[:,2]<zhi)&(np.hypot(P[:,0]-x0,P[:,1]-y0)<rad)
    pts=P[ok]
    if len(pts)==0: log("no points"); return None
    c=pts.mean(0); 
    log(f"n={len(pts)} centroid {c.round(4)} x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] ztop {pts[:,2].max():.4f}")
    return c, pts
if __name__=="__main__":
    r=Robot(); x0,y0,zlo,zhi=map(float,sys.argv[1:5]); refine(r,x0,y0,zlo,zhi)
EOF
cat > phase1c.py <<'EOF'
from rob import *
from refine import refine
r = Robot()
SX, SY = -0.244, -0.173
r.move_tcp_line(SX, SY, 0.62, yaw=0.0, secs=3, n=2)
r.gripper(0.04)
res = refine(r, SX, SY, 0.47, 0.53)
if res is not None:
    c, pts = res
    # use the extent midpoint (robust to partial views) 
    SX = (pts[:,0].min()+pts[:,0].max())/2; SY = (pts[:,1].min()+pts[:,1].max())/2
    log(f"refined center {SX:.4f},{SY:.4f}")
log("PHASE1C DONE")
EOF
nohup python3 -u phase1c.py > phase1c.log 2>&1 &
sleep 45; cat phase1c.log

# openrua op 31
timeout 120 python3 -u refine.py -0.238 -0.176 0.505 0.53 2>&1 | grep -v XMLPARSER; timeout 120 python3 -u refine.py -0.238 -0.176 0.44 0.505 2>&1 | grep -v XMLPARSER

# openrua op 32
cat > phase1d.py <<'EOF'
from rob import *
from geometry_msgs.msg import WrenchStamped
r = Robot()
wr={}
r.node.create_subscription(WrenchStamped, "/franka_robot_state_broadcaster/external_wrench", lambda m: wr.__setitem__('m',m), 1)
def wrench():
    wr.pop('m',None)
    for _ in range(50):
        rclpy.spin_once(r.node,timeout_sec=0.1)
        if 'm' in wr: break
    f=wr['m'].wrench.force; return np.round([f.x,f.y,f.z],2)
SX, SY = -0.238, -0.1765
r.spin(0.5); log("fingers", r.fingers())
assert r.move_tcp_line(SX, SY, 0.56, yaw=0.0, secs=3, n=2)
log("wrench", wrench())
assert r.move_tcp_line(SX, SY, 0.50, yaw=0.0, secs=3, n=2)
log("wrench", wrench())
ok = r.move_tcp_line(SX, SY, 0.45, yaw=0.0, secs=3, n=2)
log("wrench", wrench(), "ok", ok)
_,_,tcp=r.hand_pose(); log("tcp before close", tcp.round(4))
f = r.gripper(0.0)
r.spin(0.5); f=r.fingers()
log(f"finger gap after close = {f[0]-f[1]:.4f}")
ok2 = r.move_tcp_line(SX, SY, 0.75, yaw=0.0, secs=4, n=3)
if not ok2:
    log("retry lift"); r.move_tcp_line(SX, SY, 0.75, yaw=0.0, secs=4, n=2)
r.spin(0.5); f=r.fingers(); log("fingers after lift", f, "gap", f[0]-f[1], "wrench", wrench())
log("PHASE1D DONE")
EOF
nohup python3 -u phase1d.py > phase1d.log 2>&1 &
sleep 90; grep -v XMLPARSER phase1d.log

# openrua op 33
timeout 60 python3 tools/perception/cam_snap.py agentview a3.png 2>&1 | grep -v XMLPARSER

# openrua op 34
cat > phase2.py <<'EOF'
from rob import *
r = Robot()
r.spin(0.5); f=r.fingers(); log("fingers", f, "gap", f[0]-f[1])
BX, BY = 0.0, 0.215
assert r.move_tcp_line(-0.238, -0.1765, 0.82, yaw=0.0, secs=3, n=2)
ok = r.move_tcp(BX, BY, 0.82, yaw=0.0, secs=5)
if not ok: log("retry"); ok = r.move_tcp(BX, BY, 0.82, yaw=0.0, secs=5)
f=r.fingers(); log("fingers over basket", f, "gap", f[0]-f[1])
ok = r.move_tcp_line(BX, BY, 0.66, yaw=0.0, secs=4, n=3)
if not ok: log("retry"); r.move_tcp_line(BX, BY, 0.66, yaw=0.0, secs=4, n=2)
r.gripper(0.04)
r.spin(0.5); log("fingers after open", r.fingers())
ok = r.move_tcp_line(BX, BY, 0.85, yaw=0.0, secs=4, n=2)
if not ok: log("retry"); r.move_tcp_line(BX, BY, 0.85, yaw=0.0, secs=4, n=2)
log("PHASE2 DONE")
EOF
nohup python3 -u phase2.py > phase2.log 2>&1 &
sleep 100; grep -v XMLPARSER phase2.log

# openrua op 35
timeout 60 python3 tools/perception/cam_snap.py agentview a4.png 2>&1 | grep -v XMLPARSER; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand e4.png 2>&1 | grep -v XMLPARSER

# openrua op 36
cat > phase3.py <<'EOF'
from rob import *
from refine import refine
from geometry_msgs.msg import WrenchStamped
r = Robot()
wr={}
r.node.create_subscription(WrenchStamped, "/franka_robot_state_broadcaster/external_wrench", lambda m: wr.__setitem__('m',m), 1)
def wrench():
    wr.pop('m',None)
    for _ in range(50):
        rclpy.spin_once(r.node,timeout_sec=0.1)
        if 'm' in wr: break
    f=wr['m'].wrench.force; return np.round([f.x,f.y,f.z],2)
TX, TY = -0.023, -0.274
ok = r.move_tcp(TX, TY, 0.66, yaw=0.0, secs=5)
if not ok: log("retry"); r.move_tcp(TX, TY, 0.66, yaw=0.0, secs=5)
res = refine(r, TX, TY, 0.49, 0.56)
if res is not None:
    c, pts = res
    TX = (pts[:,0].min()+pts[:,0].max())/2; TY = (pts[:,1].min()+pts[:,1].max())/2
    ztop = np.percentile(pts[:,2], 90)
    log(f"refined center {TX:.4f},{TY:.4f} ztop {ztop:.4f}")
r.gripper(0.04); r.spin(0.5); log("fingers", r.fingers())
assert r.move_tcp_line(TX, TY, 0.56, yaw=0.0, secs=3, n=2)
log("wrench", wrench())
ok = r.move_tcp_line(TX, TY, 0.465, yaw=0.0, secs=4, n=3)
log("wrench", wrench(), "ok", ok)
_,_,tcp=r.hand_pose(); log("tcp before close", tcp.round(4))
r.gripper(0.0); r.spin(0.5); f=r.fingers()
log(f"finger gap after close = {f[0]-f[1]:.4f}")
ok2 = r.move_tcp_line(TX, TY, 0.82, yaw=0.0, secs=4, n=3)
if not ok2: log("retry lift"); r.move_tcp_line(TX, TY, 0.82, yaw=0.0, secs=4, n=2)
r.spin(0.5); f=r.fingers(); log("fingers after lift", f, "gap", f[0]-f[1], "wrench", wrench())
log("PHASE3 DONE")
EOF
nohup python3 -u phase3.py > phase3.log 2>&1 &
sleep 110; grep -v XMLPARSER phase3.log

# openrua op 37
cat > phase4.py <<'EOF'
from rob import *
r = Robot()
r.spin(0.5); f=r.fingers(); log("fingers", f, "gap", f[0]-f[1])
BX, BY = 0.02, 0.29
ok = r.move_tcp(BX, BY, 0.84, yaw=0.0, secs=5)
if not ok: log("retry"); ok = r.move_tcp(BX, BY, 0.84, yaw=0.0, secs=5)
f=r.fingers(); log("fingers over basket", f, "gap", f[0]-f[1])
ok = r.move_tcp_line(BX, BY, 0.67, yaw=0.0, secs=4, n=3)
if not ok: log("retry"); r.move_tcp_line(BX, BY, 0.67, yaw=0.0, secs=4, n=2)
r.gripper(0.04)
r.spin(0.5); log("fingers after open", r.fingers())
ok = r.move_tcp_line(BX, BY, 0.88, yaw=0.0, secs=4, n=2)
if not ok: log("retry"); r.move_tcp_line(BX, BY, 0.88, yaw=0.0, secs=4, n=2)
# retreat to a neutral pose away from the basket so cameras see it
r.move_tcp(-0.15, 0.0, 0.85, yaw=0.0, secs=5)
log("PHASE4 DONE")
EOF
nohup python3 -u phase4.py > phase4.log 2>&1 &
sleep 110; grep -v XMLPARSER phase4.log

# openrua op 38
for c in agentview birdview frontview robot0_robotview; do timeout 60 python3 tools/perception/cam_snap.py $c final_$c.png 2>&1 | grep -v XMLPARSER; done

# openrua op 39
python3 -c "
import numpy as np, cv2
im=cv2.imread('final_birdview.png'); c=im[315:365,370:430]; c=cv2.resize(c,None,fx=8,fy=8,interpolation=cv2.INTER_CUBIC)
for i in range(0,c.shape[1],80): cv2.line(c,(i,0),(i,c.shape[0]),(0,255,0),1); cv2.putText(c,str(370+i//8),(i,12),0,0.4,(0,255,0),1)
for j in range(0,c.shape[0],80): cv2.line(c,(0,j),(c.shape[1],j),(0,255,0),1); cv2.putText(c,str(315+j//8),(0,j+12),0,0.4,(0,255,0),1)
cv2.imwrite('bird_tom.png',c)"
timeout 90 python3 pxw.py birdview 398,340 390,335 406,345 2>&1 | grep -v XMLPARSER

# openrua op 40
cat >> rob.py <<'EOF'


def quat_from_axes(zaxis, yaxis):
    """Quaternion (x,y,z,w) of a hand frame with the given world-frame
    approach axis (hand z) and finger-closing axis (hand y)."""
    z = np.asarray(zaxis, float); z /= np.linalg.norm(z)
    y = np.asarray(yaxis, float); y -= y.dot(z) * z; y /= np.linalg.norm(y)
    x = np.cross(y, z)
    R = np.column_stack([x, y, z])
    tr = np.trace(R)
    if tr > 0:
        s = math.sqrt(tr + 1) * 2
        return ((R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s)
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(1 + R[i, i] - R[j, j] - R[k, k]) * 2
    q = [0.0] * 4
    q[i] = 0.25 * s
    q[j] = (R[j, i] + R[i, j]) / s
    q[k] = (R[k, i] + R[i, k]) / s
    q[3] = (R[k, j] - R[j, k]) / s
    return tuple(q)
EOF
cat > diag3.py <<'EOF'
from rob import *
r = Robot()
CX, CY, ang = 0.219, 0.337, math.radians(30.6)
c = np.array([math.cos(ang), math.sin(ang), 0.0])
f = np.array([-math.sin(ang), math.cos(ang), 0.0])
for th in (0, 15, 25, 35, 45, 55):
    t = math.radians(th)
    a = math.sin(t) * c + np.array([0, 0, -math.cos(t)])
    q = quat_from_axes(a, f)
    for z in (0.60, 0.445):
        sol = r.solve_ik(CX, CY, z, q)
        log(f"tilt {th} z {z}:", None if sol is None else np.round(sol, 3))
    # also flipped finger axis
    q2 = quat_from_axes(a, -f)
    sol = r.solve_ik(CX, CY, 0.445, q2); log(f"tilt {th} z 0.445 flipped:", None if sol is None else np.round(sol, 3))
EOF
timeout 300 python3 -u diag3.py 2>&1 | grep -v XMLPARSER

# openrua op 41
cat > phase5.py <<'EOF'
from rob import *
from refine import refine
from geometry_msgs.msg import WrenchStamped
r = Robot()
wr={}
r.node.create_subscription(WrenchStamped, "/franka_robot_state_broadcaster/external_wrench", lambda m: wr.__setitem__('m',m), 1)
def wrench():
    wr.pop('m',None)
    for _ in range(50):
        rclpy.spin_once(r.node,timeout_sec=0.1)
        if 'm' in wr: break
    f=wr['m'].wrench.force; return np.round([f.x,f.y,f.z],2)
CX, CY, ang = 0.219, 0.337, math.radians(30.6)
def grasp_quat(ang, tilt_deg):
    c = np.array([math.cos(ang), math.sin(ang), 0.0])
    f = np.array([-math.sin(ang), math.cos(ang), 0.0])
    t = math.radians(tilt_deg)
    a = math.sin(t) * c + np.array([0, 0, -math.cos(t)])
    return quat_from_axes(a, -f)
q = grasp_quat(ang, 35)
r.gripper(0.04)
ok = r.move_tcp(CX, CY, 0.60, quat=q, secs=5)
if not ok: log("retry"); ok = r.move_tcp(CX, CY, 0.60, quat=q, secs=5)
res = refine(r, CX, CY, 0.44, 0.505, rad=0.08)
if res is not None:
    c, pts = res
    xy = pts[:, :2] - pts[:, :2].mean(0)
    w, v = np.linalg.eigh(xy.T @ xy)
    axis = v[:, 1]  # principal direction = can axis
    ang2 = math.atan2(axis[1], axis[0])
    if math.cos(ang2 - ang) < 0: ang2 += math.pi  # keep pointing away from base
    log(f"refined center {c[0]:.4f},{c[1]:.4f} axis angle {math.degrees(ang2):.1f} deg (was {math.degrees(ang):.1f})")
    CX, CY, ang = c[0], c[1], ang2
    q = grasp_quat(ang, 35)
    ok = r.move_tcp(CX, CY, 0.60, quat=q, secs=3)
log("wrench", wrench())
ok = r.move_tcp_line(CX, CY, 0.50, quat=q, secs=3, n=2)
log("wrench", wrench(), ok)
ok = r.move_tcp_line(CX, CY, 0.445, quat=q, secs=3, n=2)
log("wrench", wrench(), ok)
_,_,tcp=r.hand_pose(); log("tcp before close", tcp.round(4))
r.gripper(0.0); r.spin(0.5); f=r.fingers()
log(f"finger gap after close = {f[0]-f[1]:.4f}")
ok2 = r.move_tcp_line(CX, CY, 0.70, quat=q, secs=4, n=2)
if not ok2: log("retry lift"); r.move_tcp_line(CX, CY, 0.70, quat=q, secs=4, n=2)
r.spin(0.5); f=r.fingers(); log("fingers after lift", f, "gap", f[0]-f[1], "wrench", wrench())
log("PHASE5 DONE")
EOF
nohup python3 -u phase5.py > phase5.log 2>&1 &
sleep 120; grep -v XMLPARSER phase5.log

# openrua op 42
cat > phase5b.py <<'EOF'
from rob import *
from refine import refine
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
r = Robot()
r.gripper(0.04)
ok = r.move_tcp(0.19, 0.31, 0.66, yaw=0.0, secs=5)
if not ok: log("retry"); ok = r.move_tcp(0.19, 0.31, 0.66, yaw=0.0, secs=5)
got={}
s=r.node.create_subscription(Image,"/robot0_eye_in_hand/color/image_raw",lambda m:got.setdefault('c',m),1)
while 'c' not in got: rclpy.spin_once(r.node,timeout_sec=0.2)
cv2.imwrite("eih_tom.png", CvBridge().imgmsg_to_cv2(got['c'],"bgr8"))
res = refine(r, 0.22, 0.34, 0.435, 0.51, rad=0.13)
c, pts = res
xy = pts[:, :2] - pts[:, :2].mean(0)
w, v = np.linalg.eigh(xy.T @ xy)
axis = v[:, 1]; ang2 = math.degrees(math.atan2(axis[1], axis[0]))
log(f"center {c.round(4)} extents x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] axis {ang2:.1f} deg eig {np.sqrt(w/len(pts)).round(4)}")
# top-ridge points only (z > 0.48) give the axis line cleanly
top = pts[pts[:,2] > 0.48]
xy = top[:, :2] - top[:, :2].mean(0); w, v = np.linalg.eigh(xy.T @ xy); axis = v[:, 1]
log(f"ridge n={len(top)} center {top.mean(0).round(4)} axis {math.degrees(math.atan2(axis[1], axis[0])):.1f} deg")
np.save("tom_pts.npy", pts)
log("PHASE5B DONE")
EOF
nohup python3 -u phase5b.py > phase5b.log 2>&1 &
sleep 50; grep -v XMLPARSER phase5b.log

# openrua op 43
cat > diag4.py <<'EOF'
from rob import *
r = Robot()
CX, CY, ang = 0.228, 0.346, math.radians(31.0)
c = np.array([math.cos(ang), math.sin(ang), 0.0]); f = np.array([-math.sin(ang), math.cos(ang), 0.0])
lim = np.array(FJT["limits_rad"])
cur = r.arm_q()
seeds = {"cur": cur, "stretch": [0.4, 1.0, 0.0, -1.2, 0.0, 2.2, 0.8], "stretch2": [0.4, 1.2, 0.0, -1.0, 0.0, 2.6, 0.8]}
best=[]
for th in (30, 40, 50, 60, 70):
    t = math.radians(th); a = math.sin(t) * c + np.array([0, 0, -math.cos(t)])
    for sgn in (1, -1):
        q = quat_from_axes(a, sgn * f)
        for sn, sd in seeds.items():
            sol = r.solve_ik(CX, CY, 0.445, q, seed=sd)
            if sol is None: continue
            sol = np.array(sol); margin = np.min(np.minimum(sol - lim[:,0], lim[:,1] - sol))
            # pre-grasp 12cm back along approach, seeded from sol
            pre = np.array([CX, CY, 0.445]) - 0.12 * a
            sol2 = r.solve_ik(*pre, q, seed=sol)
            jump = None if sol2 is None else np.max(np.abs(np.array(sol2) - sol))
            log(f"tilt {th} sgn {sgn} seed {sn}: margin {margin:.2f} pre_ok {sol2 is not None} jump {jump} sol {sol.round(2)}")
            if sol2 is not None: best.append((margin, th, sgn, sn, sol.round(3).tolist(), np.round(sol2,3).tolist()))
best.sort(reverse=True)
log("BEST", best[:3])
EOF
timeout 600 python3 -u diag4.py 2>&1 | grep -v XMLPARSER | grep -v "IK failed"

# openrua op 44
cat > phase6.py <<'EOF'
from rob import *
from geometry_msgs.msg import WrenchStamped
r = Robot()
wr={}
r.node.create_subscription(WrenchStamped, "/franka_robot_state_broadcaster/external_wrench", lambda m: wr.__setitem__('m',m), 1)
def wrench():
    wr.pop('m',None)
    for _ in range(50):
        rclpy.spin_once(r.node,timeout_sec=0.1)
        if 'm' in wr: break
    f=wr['m'].wrench.force; return np.round([f.x,f.y,f.z],2)
CX, CY, ang = 0.228, 0.346, math.radians(31.0)
c = np.array([math.cos(ang), math.sin(ang), 0.0]); f = np.array([-math.sin(ang), math.cos(ang), 0.0])
t = math.radians(30); a = math.sin(t) * c + np.array([0, 0, -math.cos(t)])
q = quat_from_axes(a, -f)
PRE = [0.324, 0.724, 0.115, -1.747, -0.001, 2.986, 0.635]
GZ = 0.45
r.gripper(0.04)
ok = r.move_joints(PRE, secs=5)
_,_,tcp = r.hand_pose(); log("pre tcp", tcp.round(4), "expected", (np.array([CX,CY,0.445]) - 0.12*a).round(4))
log("wrench", wrench())
mid = np.array([CX, CY, GZ]) - 0.05 * a
ok = r.move_tcp_line(*mid, quat=q, secs=3, n=2); log("wrench", wrench(), ok)
ok = r.move_tcp_line(CX, CY, GZ, quat=q, secs=3, n=2); log("wrench", wrench(), ok)
_,_,tcp=r.hand_pose(); log("tcp before close", tcp.round(4))
r.gripper(0.0); r.spin(0.5); fg=r.fingers()
log(f"finger gap after close = {fg[0]-fg[1]:.4f}")
# retreat back along the approach axis, then up
back = np.array([CX, CY, GZ]) - 0.15 * a
ok2 = r.move_tcp_line(*back, quat=q, secs=4, n=2)
if not ok2: log("retry"); r.move_tcp_line(*back, quat=q, secs=4, n=2)
r.spin(0.5); fg=r.fingers(); log("fingers after lift", fg, "gap", fg[0]-fg[1], "wrench", wrench())
log("PHASE6 DONE")
EOF
nohup python3 -u phase6.py > phase6.log 2>&1 &
sleep 100; grep -v XMLPARSER phase6.log

# openrua op 45
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand e6.png 2>&1 | grep -v XMLPARSER; timeout 60 python3 tools/perception/cam_snap.py birdview b6.png 2>&1 | grep -v XMLPARSER; timeout 90 python3 pxw.py birdview 398,340 2>&1 | grep -v XMLPARSER | tail -1

# openrua op 46
cat >> rob.py <<'EOF'


def move_tcp_iter(r, x, y, z, quat, secs=3.0, tries=4, tol=0.005):
    """Repeat IK->trajectory with error compensation until TCP within tol."""
    target = np.array([x, y, z], float)
    goal = target.copy()
    for i in range(tries):
        q = r.solve_ik(*goal, quat)
        if q is None:
            log("iter: IK failed"); return False
        r.move_joints(q, secs, retries=0)
        cur = np.array(r.arm_q())
        _, _, tcp = r.hand_pose()
        err = tcp - target
        log(f"iter {i}: tcp {tcp.round(4)} err {err.round(4)} |err|={np.linalg.norm(err):.4f} "
            f"joint err {np.round(cur - np.array(q), 3)}")
        if np.linalg.norm(err) < tol:
            return True
        goal = goal - err  # compensate the residual
    return False
EOF
cat > phase6b.py <<'EOF'
from rob import *
from geometry_msgs.msg import WrenchStamped
r = Robot()
wr={}
r.node.create_subscription(WrenchStamped, "/franka_robot_state_broadcaster/external_wrench", lambda m: wr.__setitem__('m',m), 1)
def wrench():
    wr.pop('m',None)
    for _ in range(50):
        rclpy.spin_once(r.node,timeout_sec=0.1)
        if 'm' in wr: break
    f=wr['m'].wrench.force; return np.round([f.x,f.y,f.z],2)
CX, CY, ang = 0.230, 0.342, math.radians(31.0)
c = np.array([math.cos(ang), math.sin(ang), 0.0]); f = np.array([-math.sin(ang), math.cos(ang), 0.0])
t = math.radians(30); a = math.sin(t) * c + np.array([0, 0, -math.cos(t)])
q = quat_from_axes(a, -f)
GZ = 0.45
r.gripper(0.04)
mid = np.array([CX, CY, GZ]) - 0.06 * a
ok = move_tcp_iter(r, *mid, q, secs=3); log("wrench", wrench(), ok)
ok = move_tcp_iter(r, CX, CY, GZ, q, secs=3); log("wrench", wrench(), ok)
_,_,tcp=r.hand_pose(); log("tcp before close", tcp.round(4))
r.gripper(0.0); r.spin(0.5); fg=r.fingers()
log(f"finger gap after close = {fg[0]-fg[1]:.4f}")
back = np.array([CX, CY, GZ]) - 0.15 * a
ok2 = r.move_tcp_line(*back, quat=q, secs=4, n=2)
if not ok2: log("retry"); r.move_tcp_line(*back, quat=q, secs=4, n=2)
r.spin(0.5); fg=r.fingers(); log("fingers after lift", fg, "gap", fg[0]-fg[1], "wrench", wrench())
log("PHASE6B DONE")
EOF
nohup python3 -u phase6b.py > phase6b.log 2>&1 &
sleep 120; grep -v XMLPARSER phase6b.log

# openrua op 47
cat > look.py <<'EOF'
from rob import *
r = Robot()
CX, CY, ang = 0.230, 0.342, math.radians(31.0)
c = np.array([math.cos(ang), math.sin(ang), 0.0]); f = np.array([-math.sin(ang), math.cos(ang), 0.0])
t = math.radians(30); a = math.sin(t) * c + np.array([0, 0, -math.cos(t)])
q = quat_from_axes(a, -f)
sol = r.solve_ik(CX, CY, 0.45, q)
log("target q", np.round(sol,3))
r.move_joints(sol, 3, retries=0)
log("now q", np.round(r.arm_q(),3))
EOF
timeout 200 python3 -u look.py 2>&1 | grep -v XMLPARSER; for c in frontview sideview agentview; do timeout 60 python3 tools/perception/cam_snap.py $c blk_$c.png 2>&1 | grep -v XMLPARSER; done

# openrua op 48
cat > scene.py <<'EOF'
"""Publish basket + table collision boxes into the MoveIt planning scene."""
from rob import *
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

def box(name, center, size):
    co = CollisionObject()
    co.header.frame_id = "world"
    co.id = name
    sp = SolidPrimitive(); sp.type = SolidPrimitive.BOX; sp.dimensions = [float(s) for s in size]
    p = Pose(); p.position.x, p.position.y, p.position.z = map(float, center); p.orientation.w = 1.0
    co.primitives = [sp]; co.primitive_poses = [p]
    co.operation = CollisionObject.ADD
    return co

def publish_scene(r):
    cli = r.node.create_client(ApplyPlanningScene, "/apply_planning_scene")
    assert cli.wait_for_service(timeout_sec=20)
    req = ApplyPlanningScene.Request()
    req.scene = PlanningScene(); req.scene.is_diff = True
    req.scene.world.collision_objects = [
        box("basket", (0.003, 0.251, 0.525), (0.17, 0.18, 0.20)),
        box("table", (0.0, 0.0, 0.40), (1.6, 1.2, 0.05)),
    ]
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(r.node, fut, timeout_sec=30)
    log("apply_planning_scene ->", fut.result().success if fut.result() else None)

if __name__ == "__main__":
    r = Robot(); publish_scene(r)
EOF
timeout 120 python3 -u scene.py 2>&1 | grep -v XMLPARSER
timeout 60 ros2 service call /get_planning_scene moveit_msgs/srv/GetPlanningScene "{components: {components: 4}}" 2>&1 | grep -E "id:|success" | head

# openrua op 49
timeout 60 ros2 service call /get_planning_scene moveit_msgs/srv/GetPlanningScene "{components: {components: 12}}" 2>&1 | grep -E "^\s*id:" | head

# openrua op 50
timeout 60 ros2 service call /get_planning_scene moveit_msgs/srv/GetPlanningScene "{components: {components: 1023}}" 2>&1 | grep -iE "id: |collision_objects|basket|table" | head

# openrua op 51
cat > diag5.py <<'EOF'
from rob import *
from moveit_msgs.srv import GetStateValidity
r = Robot()
sv = r.node.create_client(GetStateValidity, "/check_state_validity"); assert sv.wait_for_service(timeout_sec=20)
def valid(q):
    req = GetStateValidity.Request(); req.group_name = "panda_arm"
    req.robot_state.joint_state.name = list(ARM); req.robot_state.joint_state.position = [float(v) for v in q]
    fut = sv.call_async(req); rclpy.spin_until_future_complete(r.node, fut, timeout_sec=30)
    res = fut.result()
    return res.valid, [(c.contact_body_1, c.contact_body_2) for c in res.contacts]
log("current state valid?", valid(r.arm_q()))
log("blocked target valid?", valid([0.35, 1.103, 0.103, -1.321, -0.039, 2.94, 0.653]))

def ik_col(x, y, z, quat, seed):
    p = np.array([x, y, z]) - TCP * quat_to_R(*quat)[:, 2]
    q8 = qmul(quat, (0.0, 0.0, 0.38268343, 0.92387953))
    req = GetPositionIK.Request(); req.ik_request.group_name = "panda_arm"
    req.ik_request.pose_stamped.header.frame_id = "world"
    ps = req.ik_request.pose_stamped.pose
    ps.position.x, ps.position.y, ps.position.z = map(float, p)
    ps.orientation.x, ps.orientation.y, ps.orientation.z, ps.orientation.w = map(float, q8)
    req.ik_request.robot_state.joint_state.name = list(ARM)
    req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
    req.ik_request.avoid_collisions = True
    req.ik_request.timeout.sec = 3
    fut = r.ik.call_async(req); rclpy.spin_until_future_complete(r.node, fut, timeout_sec=60)
    res = fut.result()
    if res is None or res.error_code.val != 1: return None
    sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
    return [sol[n] for n in ARM]

CX, CY, ang = 0.230, 0.342, math.radians(31.0)
c = np.array([math.cos(ang), math.sin(ang), 0.0]); f = np.array([-math.sin(ang), math.cos(ang), 0.0])
lim = np.array(FJT["limits_rad"])
rng = np.random.default_rng(0)
found = []
for th in (20, 30, 40, 50, 60):
    t = math.radians(th); a = math.sin(t) * c + np.array([0, 0, -math.cos(t)])
    for sgn in (-1, 1):
        q = quat_from_axes(a, sgn * f)
        seeds = [r.arm_q()] + [rng.uniform(lim[:, 0], lim[:, 1]) for _ in range(6)]
        for i, sd in enumerate(seeds):
            sol = ik_col(CX, CY, 0.45, q, sd)
            if sol is None: continue
            sol = np.array(sol); margin = np.min(np.minimum(sol - lim[:, 0], lim[:, 1] - sol))
            v = valid(sol)
            log(f"tilt {th} sgn {sgn} seed {i}: margin {margin:.2f} valid {v[0]} sol {sol.round(2)}")
            found.append((margin, th, sgn, sol.round(4).tolist()))
found.sort(reverse=True)
log("FOUND", found[:5])
EOF
timeout 900 python3 -u diag5.py 2>&1 | grep -v XMLPARSER

# openrua op 52
cat > phase7.py <<'EOF'
from rob import *
from scene import box
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.msg import PlanningScene, CollisionObject
r = Robot()
# drop the basket collision object (we're about to move it)
cli = r.node.create_client(ApplyPlanningScene, "/apply_planning_scene"); cli.wait_for_service(timeout_sec=20)
req = ApplyPlanningScene.Request(); req.scene.is_diff = True
co = CollisionObject(); co.header.frame_id = "world"; co.id = "basket"; co.operation = CollisionObject.REMOVE
req.scene.world.collision_objects = [co]
fut = cli.call_async(req); rclpy.spin_until_future_complete(r.node, fut, timeout_sec=30); log("removed basket CO", fut.result().success)
# back off from the can region first (straight up along current approach)
r.gripper(0.04)
ok = r.move_tcp(0.0, 0.125, 0.75, yaw=math.pi/2, secs=6)
if not ok: log("retry"); r.move_tcp(0.0, 0.125, 0.75, yaw=math.pi/2, secs=6)
ok = r.move_tcp_line(0.0, 0.125, 0.50, yaw=math.pi/2, secs=4, n=3); log("down ok", ok)
ok = r.move_tcp_line(0.0, 0.37, 0.50, yaw=math.pi/2, secs=6, n=4); log("push ok", ok)
if not ok: log("retry push"); r.move_tcp_line(0.0, 0.37, 0.50, yaw=math.pi/2, secs=6, n=2)
ok = r.move_tcp_line(0.0, 0.30, 0.78, yaw=math.pi/2, secs=4, n=2); log("up ok", ok)
log("PHASE7 DONE")
EOF
nohup python3 -u phase7.py > phase7.log 2>&1 &
sleep 120; grep -v XMLPARSER phase7.log

# openrua op 53
sleep 25; grep -v XMLPARSER phase7.log | tail -3; for c in birdview agentview; do timeout 60 python3 tools/perception/cam_snap.py $c p7_$c.png 2>&1 | grep -v XMLPARSER; done

# openrua op 54
for c in robot0_robotview sideview frontview; do timeout 60 python3 tools/perception/cam_snap.py $c p7_$c.png 2>&1 | grep -v XMLPARSER; done

# openrua op 55
cat > park.py <<'EOF'
from rob import *
r = Robot()
ok = r.move_tcp(-0.30, -0.15, 0.85, yaw=0.0, secs=5)
if not ok: r.move_tcp(-0.30, -0.15, 0.85, yaw=0.0, secs=5)
EOF
timeout 300 python3 -u park.py 2>&1 | grep -v XMLPARSER | tail -2; timeout 60 python3 tools/perception/cam_snap.py birdview p7b_birdview.png 2>&1 | grep -v XMLPARSER

# openrua op 56
python3 -c "
import cv2
im=cv2.imread('p7b_birdview.png'); c=im[230:340,300:470]; c=cv2.resize(c,None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC)
for i in range(0,c.shape[1],40): cv2.line(c,(i,0),(i,c.shape[0]),(0,255,0),1); cv2.putText(c,str(300+i//4),(i,12),0,0.35,(0,255,0),1)
for j in range(0,c.shape[0],40): cv2.line(c,(0,j),(c.shape[1],j),(0,255,0),1); cv2.putText(c,str(230+j//4),(0,j+12),0,0.35,(0,255,0),1)
cv2.imwrite('bird_p7.png',c)"

# openrua op 57
timeout 90 python3 pxw.py birdview 425,292 2>&1 | grep -v XMLPARSER | tail -1; python3 -c "
import numpy as np
d=np.load('birdview_depth.npy')
fx=579.4112549695428; cx=320; cy=240
q=np.array([0.7071,0.7071,0,0]); x,y,z,w=q
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
T=np.array([-0.2,0,3.0])
reg=d[255:335,395:460]
vs,us=np.where((reg<2.45)&(reg>2.3)); vs+=255; us+=395
P=np.array([R@np.array([(u-cx)*d[v,u]/fx,(v-cy)*d[v,u]/fx,d[v,u]])+T for u,v in zip(us,vs)])
print('rim pts n',len(P),'x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]'%(P[:,0].min(),P[:,0].max(),P[:,1].min(),P[:,1].max(),P[:,2].min(),P[:,2].max()))
c=P[:,:2].mean(0); print('rim centroid',c.round(4))
# orientation via minimum-area bounding box search
best=None
for a in np.arange(0,90,1):
    t=np.radians(a); Rz=np.array([[np.cos(t),np.sin(t)],[-np.sin(t),np.cos(t)]])
    Q=(P[:,:2]-c)@Rz.T; ext=Q.max(0)-Q.min(0); area=ext[0]*ext[1]
    if best is None or area<best[0]: best=(area,a,ext,(Q.max(0)+Q.min(0))/2)
print('best angle',best[1],'extents',best[2].round(3),'offset',best[3].round(4))
# soup can inside: pixels deeper than rim but shallower than floor
reg=d[270:310,405:445]; vs,us=np.where((reg>2.45)&(reg<2.53)); vs+=270; us+=405
P2=np.array([R@np.array([(u-cx)*d[v,u]/fx,(v-cy)*d[v,u]/fx,d[v,u]])+T for u,v in zip(us,vs)])
if len(P2): print('soup pts n',len(P2),'center',P2[:,:2].mean(0).round(4),'ztop',P2[:,2].max().round(3))
"


# openrua op 58
cat > diag6.py <<'EOF'
from rob import *
from scene import box
from moveit_msgs.srv import ApplyPlanningScene, GetStateValidity
from moveit_msgs.msg import PlanningScene
r = Robot()
cli = r.node.create_client(ApplyPlanningScene, "/apply_planning_scene"); cli.wait_for_service(timeout_sec=20)
req = ApplyPlanningScene.Request(); req.scene.is_diff = True
b = box("basket", (0.027, 0.447, 0.525), (0.17, 0.18, 0.20))
t = math.radians(60); b.pose.orientation.z = math.sin(t/2); b.pose.orientation.w = math.cos(t/2)
req.scene.world.collision_objects = [b]
fut = cli.call_async(req); rclpy.spin_until_future_complete(r.node, fut, timeout_sec=30); log("basket CO", fut.result().success)
sv = r.node.create_client(GetStateValidity, "/check_state_validity"); sv.wait_for_service(timeout_sec=20)
def valid(q):
    rq = GetStateValidity.Request(); rq.group_name = "panda_arm"
    rq.robot_state.joint_state.name = list(ARM); rq.robot_state.joint_state.position = [float(v) for v in q]
    f = sv.call_async(rq); rclpy.spin_until_future_complete(r.node, f, timeout_sec=30)
    return f.result().valid, [(c.contact_body_1, c.contact_body_2) for c in f.result().contacts]
CX, CY, ang = 0.230, 0.342, math.radians(31.0)
c = np.array([math.cos(ang), math.sin(ang), 0.0]); f = np.array([-math.sin(ang), math.cos(ang), 0.0])
for th in (30, 40):
    tt = math.radians(th); a = math.sin(tt) * c + np.array([0, 0, -math.cos(tt)])
    q = quat_from_axes(a, -f)
    seed = [0.369, 1.114, 0.087, -1.314, -0.009, 2.945, 0.64]
    g = r.solve_ik(CX, CY, 0.45, q, seed=seed)
    pre = np.array([CX, CY, 0.45]) - 0.12 * a
    p = r.solve_ik(*pre, q, seed=g) if g else None
    log(f"tilt {th}: grasp {None if g is None else np.round(g,3)} valid {valid(g) if g else None}")
    log(f"tilt {th}: pre   {None if p is None else np.round(p,3)} valid {valid(p) if p else None}")
EOF
timeout 300 python3 -u diag6.py 2>&1 | grep -v XMLPARSER

# openrua op 59
grep -n "def \|wrench\|Wrench" rob.py | head -40; grep -ln "wrench" *.py

# openrua op 60
grep -n -B2 -A8 "wrench" phase6b.py | head -60

# openrua op 61
cat > phase8.py <<'EOF'
from rob import *
from geometry_msgs.msg import WrenchStamped
r = Robot()
wr = {}
r.node.create_subscription(WrenchStamped, "/franka_robot_state_broadcaster/external_wrench", lambda m: wr.__setitem__('m', m), 1)
def wrench():
    wr.pop('m', None)
    for _ in range(50):
        rclpy.spin_once(r.node, timeout_sec=0.1)
        if 'm' in wr: break
    f = wr['m'].wrench.force; return np.round([f.x, f.y, f.z], 2)
CX, CY, GZ, ang = 0.230, 0.342, 0.45, math.radians(31.0)
c = np.array([math.cos(ang), math.sin(ang), 0.0]); f = np.array([-math.sin(ang), math.cos(ang), 0.0])
t = math.radians(30); a = math.sin(t) * c + np.array([0, 0, -math.cos(t)])
q = quat_from_axes(a, -f)
# place-pose feasibility (can horizontal, axis along 60 deg, over basket)
ang2 = math.radians(60); c2 = np.array([math.cos(ang2), math.sin(ang2), 0]); f2 = np.array([-math.sin(ang2), math.cos(ang2), 0])
a2 = math.sin(t) * c2 + np.array([0, 0, -math.cos(t)]); q2 = quat_from_axes(a2, -f2)
for z in (0.72, 0.68, 0.66):
    s = r.solve_ik(0.027, 0.447, z, q2, seed=[0.369, 1.114, 0.087, -1.314, -0.009, 2.945, 0.64])
    log("place IK z", z, None if s is None else np.round(s, 3))
PRE = [0.312, 0.714, 0.121, -1.749, -0.0, 2.978, 0.627]
r.gripper(0.04)
ok = r.move_joints(PRE, secs=5); log("at PRE", ok, "wrench", wrench())
mid = np.array([CX, CY, GZ]) - 0.05 * a
ok = move_tcp_iter(r, *mid, q, secs=3, tol=0.004); log("at mid", ok, "wrench", wrench())
ok = move_tcp_iter(r, CX, CY, GZ, q, secs=3, tol=0.004); log("at grasp", ok, "wrench", wrench())
_, _, tcp = r.hand_pose(); log("tcp before close", tcp.round(4))
if not ok:
    log("grasp pose not reached; stopping for inspection"); raise SystemExit
r.gripper(0.0); r.spin(0.5); fg = r.fingers(); gap = fg[0] - fg[1]
log(f"finger gap after close = {gap:.4f}")
if gap < 0.05:
    log("missed the can; reopening"); r.gripper(0.04); raise SystemExit
back = np.array([CX, CY, GZ]) - 0.12 * a
ok2 = move_tcp_iter(r, *back, q, secs=4, tries=2, tol=0.01); log("retreat", ok2)
up = np.array([CX, CY, 0.70]) - 0.10 * a
ok3 = move_tcp_iter(r, *up, q, secs=4, tries=2, tol=0.01); log("lift", ok3)
r.spin(0.5); fg = r.fingers(); log("fingers after lift", fg, "gap", round(fg[0] - fg[1], 4), "wrench", wrench())
_, _, tcp = r.hand_pose(); log("tcp", tcp.round(4))
log("PHASE8 DONE")
EOF
nohup python3 -u phase8.py > phase8.log 2>&1 &
sleep 45; grep -v XMLPARSER phase8.log

# openrua op 62
sleep 60; grep -v XMLPARSER phase8.log | tail -n +8

# openrua op 63
cat > tfl.py <<'EOF'
from rob import *
r = Robot()
for l in ["panda_link1","panda_link2","panda_link3","panda_link4","panda_link5","panda_link6","panda_link7","panda_hand","panda_leftfinger","panda_rightfinger"]:
    for _ in range(50):
        rclpy.spin_once(r.node, timeout_sec=0.1)
        if r.tf.can_transform("world", l, rclpy.time.Time()): break
    t = r.tf.lookup_transform("world", l, rclpy.time.Time()).transform.translation
    log(l, round(t.x,3), round(t.y,3), round(t.z,3))
log("q", np.round(r.arm_q(),3))
EOF
timeout 120 python3 -u tfl.py 2>&1 | grep -v XMLPARSER; python3 tools/perception/cam_snap.py agentview p8_agent.png; python3 tools/perception/cam_snap.py birdview p8_bird.png

# openrua op 64
sed -n 60,120p rob.py

# openrua op 65
sed -i 's/r\.tf\./r.tfbuf./g' tfl.py; timeout 120 python3 -u tfl.py 2>&1 | grep -v XMLPARSER

# openrua op 66
grep -v "^#" phase7.py | head -60

# openrua op 67
cat > phase9.py <<'EOF'
from rob import *
r = Robot()
PRE = [0.312, 0.714, 0.121, -1.749, -0.0, 2.978, 0.627]
ok = r.move_joints(PRE, secs=4); log("back at PRE", ok)
r.gripper(0.04)
ok = r.move_tcp(0.03, 0.30, 0.78, yaw=math.pi/2, secs=6)
if not ok: log("retry"); r.move_tcp(0.03, 0.30, 0.78, yaw=math.pi/2, secs=6)
ok = r.move_tcp_line(0.03, 0.30, 0.52, yaw=math.pi/2, secs=4, n=3); log("down ok", ok)
_, _, tcp = r.hand_pose(); log("tcp", tcp.round(3))
ok = r.move_tcp_line(0.03, 0.52, 0.52, yaw=math.pi/2, secs=7, n=4); log("push ok", ok)
if not ok: log("retry push"); r.move_tcp_line(0.03, 0.52, 0.52, yaw=math.pi/2, secs=6, n=2)
_, _, tcp = r.hand_pose(); log("tcp", tcp.round(3))
ok = r.move_tcp_line(0.03, 0.45, 0.80, yaw=math.pi/2, secs=4, n=2); log("up ok", ok)
ok = r.move_tcp(-0.30, -0.15, 0.85, yaw=0.0, secs=6); log("parked", ok)
log("PHASE9 DONE")
EOF
nohup python3 -u phase9.py > phase9.log 2>&1 &
sleep 75; grep -v XMLPARSER phase9.log

# openrua op 68
nohup python3 -u phase9.py > phase9.log 2>&1 &
sleep 80; grep -v XMLPARSER phase9.log

# openrua op 69
sleep 40; grep -v XMLPARSER phase9.log | tail -4; python3 tools/perception/cam_snap.py birdview p9_bird.png; python3 tools/perception/cam_snap.py frontview p9_front.png

# openrua op 70
cat > canmeas.py <<'EOF'
from refine import *
import sys
r = Robot()
cen, pts = refine(r, 0.24, 0.34, 0.44, 0.50, rad=0.12, cam="birdview")
pts = np.asarray(pts)
ridge = pts[pts[:, 2] > 0.475]
c0 = ridge[:, :2].mean(0)
u, s, vt = np.linalg.svd(ridge[:, :2] - c0)
ax = vt[0]; ang = math.degrees(math.atan2(ax[1], ax[0]))
proj = (ridge[:, :2] - c0) @ ax
log("ridge n", len(ridge), "center", c0.round(4), "axis deg", round(ang, 1), "length", round(proj.max() - proj.min(), 3), "ztop", round(pts[:, 2].max(), 4))
c1 = pts[:, :2].mean(0); log("all pts center", c1.round(4))
EOF
timeout 120 python3 -u canmeas.py 2>&1 | grep -v XMLPARSER

# openrua op 71
cat > diag7.py <<'EOF'
from rob import *
r = Robot()
CX, CY, ang = 0.268, 0.353, math.radians(31.0)
c = np.array([math.cos(ang), math.sin(ang), 0.0]); f = np.array([-math.sin(ang), math.cos(ang), 0.0])
seed = [0.369, 1.114, 0.087, -1.314, -0.009, 2.945, 0.64]
for th in (30, 40, 50, 60):
    tt = math.radians(th); a = math.sin(tt) * c + np.array([0, 0, -math.cos(tt)])
    for sgn in (1, -1):
        q = quat_from_axes(a, sgn * f)
        for z in (0.45, 0.455):
            g = r.solve_ik(CX, CY, z, q, seed=seed)
            log(f"tilt {th} sgn {sgn} z {z}: {None if g is None else np.round(g,3)}")
EOF
timeout 300 python3 -u diag7.py 2>&1 | grep -v XMLPARSER | grep -v "IK failed"

# openrua op 72
cat > phase10.py <<'EOF'
from refine import *
from geometry_msgs.msg import WrenchStamped
r = Robot()
wr = {}
r.node.create_subscription(WrenchStamped, "/franka_robot_state_broadcaster/external_wrench", lambda m: wr.__setitem__('m', m), 1)
def wrench():
    wr.pop('m', None)
    for _ in range(50):
        rclpy.spin_once(r.node, timeout_sec=0.1)
        if 'm' in wr: break
    f = wr['m'].wrench.force; return np.round([f.x, f.y, f.z], 2)
CX, CY, GZ, ang = 0.268, 0.353, 0.455, math.radians(31.0)
def frames(ang):
    c = np.array([math.cos(ang), math.sin(ang), 0.0]); f = np.array([-math.sin(ang), math.cos(ang), 0.0])
    t = math.radians(30); a = math.sin(t) * c + np.array([0, 0, -math.cos(t)])
    return c, f, a, quat_from_axes(a, -f)
c, f, a, q = frames(ang)
r.gripper(0.04)
pre = np.array([CX, CY, GZ]) - 0.14 * a
PRE = r.solve_ik(*pre, q, seed=[0.366, 1.26, 0.093, -1.01, -0.011, 2.786, 0.624]); log("PRE", np.round(PRE, 3))
ok = r.move_joints(PRE, secs=6); log("at PRE", ok, "wrench", wrench())
# wrist-camera refinement of the can pose from here
try:
    cen, pts = refine(r, CX, CY, 0.44, 0.50, rad=0.13, cam="robot0_eye_in_hand")
    pts = np.asarray(pts); ridge = pts[pts[:, 2] > 0.475]
    c0 = ridge[:, :2].mean(0); u, s, vt = np.linalg.svd(ridge[:, :2] - c0); ax = vt[0]
    ang2 = math.atan2(ax[1], ax[0])
    if math.cos(ang2 - ang) < 0: ang2 += math.pi
    log("refined center", c0.round(4), "axis deg", round(math.degrees(ang2), 1), "n", len(ridge), "ztop", round(pts[:, 2].max(), 4))
    if len(ridge) > 50 and np.linalg.norm(c0 - [CX, CY]) < 0.03 and abs(math.degrees(ang2) - 31) < 15:
        CX, CY = float(c0[0]), float(c0[1]); ang = ang2; c, f, a, q = frames(ang); log("using refined pose")
except Exception as e:
    log("refine failed", e)
mid = np.array([CX, CY, GZ]) - 0.05 * a
ok = move_tcp_iter(r, *mid, q, secs=3, tol=0.004); log("at mid", ok, "wrench", wrench())
if not ok: log("mid not reached; stopping"); raise SystemExit
ok = move_tcp_iter(r, CX, CY, GZ, q, secs=3, tries=3, tol=0.004); log("at grasp", ok, "wrench", wrench())
_, _, tcp = r.hand_pose(); log("tcp before close", tcp.round(4))
if not ok:
    log("grasp pose not reached; stopping for inspection"); raise SystemExit
r.gripper(0.0); r.spin(0.5); fg = r.fingers(); gap = fg[0] - fg[1]
log(f"finger gap after close = {gap:.4f}")
if gap < 0.05:
    log("missed the can; reopening"); r.gripper(0.04); raise SystemExit
back = np.array([CX, CY, GZ]) - 0.12 * a
ok2 = move_tcp_iter(r, *back, q, secs=4, tries=2, tol=0.01); log("retreat", ok2)
up = np.array([CX - 0.05, CY, 0.70]) - 0.10 * a
ok3 = move_tcp_iter(r, *up, q, secs=4, tries=2, tol=0.01); log("lift", ok3)
r.spin(0.5); fg = r.fingers(); log("fingers after lift", fg, "gap", round(fg[0] - fg[1], 4), "wrench", wrench())
_, _, tcp = r.hand_pose(); log("tcp", tcp.round(4))
log("PHASE10 DONE")
EOF
nohup python3 -u phase10.py > phase10.log 2>&1 &
sleep 60; grep -v XMLPARSER phase10.log

# openrua op 73
sleep 45; grep -v XMLPARSER phase10.log | tail -n +14

# openrua op 74
python3 tools/perception/cam_snap.py robot0_eye_in_hand p10_eih.png; python3 tools/perception/cam_snap.py agentview p10_agent.png

# openrua op 75
python3 tools/perception/cam_snap.py frontview p10_front.png; python3 tools/perception/cam_snap.py birdview p10_bird.png; python3 -c "
import cv2; im=cv2.imread('p10_bird.png'); cv2.imwrite('p10_bird_crop.png', cv2.resize(im[300:400,340:460],None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))"

# openrua op 76
python3 -c "
import cv2; im=cv2.imread('p10_front.png'); cv2.imwrite('p10_front_crop.png', cv2.resize(im[200:320,400:540],None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))"

# openrua op 77
cat > canwidth.py <<'EOF'
from refine import *
r = Robot()
cen, pts = refine(r, 0.269, 0.3535, 0.435, 0.51, rad=0.10, cam="birdview")
pts = np.asarray(pts)
ang = math.radians(31.1); c = np.array([math.cos(ang), math.sin(ang)]); f = np.array([-math.sin(ang), math.cos(ang)])
rel = pts[:, :2] - [0.269, 0.3535]
pc = rel @ c; pf = rel @ f
mid = np.abs(pc) < 0.025
log("along-axis extent", round(pc.min(), 3), round(pc.max(), 3))
for zlo in (0.435, 0.45, 0.46, 0.47):
    m = mid & (pts[:, 2] > zlo)
    log(f"z>{zlo}: n={m.sum()} f-extent [{pf[m].min():.4f},{pf[m].max():.4f}] width={pf[m].max()-pf[m].min():.4f} f-center={(pf[m].max()+pf[m].min())/2:.4f}")
# hand/finger points? (exclude z>0.51 already)
EOF
timeout 120 python3 -u canwidth.py 2>&1 | grep -v XMLPARSER

# openrua op 78
cat > phase10b.py <<'EOF'
from rob import *
from geometry_msgs.msg import WrenchStamped
r = Robot()
wr = {}
r.node.create_subscription(WrenchStamped, "/franka_robot_state_broadcaster/external_wrench", lambda m: wr.__setitem__('m', m), 1)
def wrench():
    wr.pop('m', None)
    for _ in range(50):
        rclpy.spin_once(r.node, timeout_sec=0.1)
        if 'm' in wr: break
    f = wr['m'].wrench.force; return np.round([f.x, f.y, f.z], 2)
CX, CY, GZ, ang = 0.2691, 0.3535, 0.455, math.radians(31.1)
c = np.array([math.cos(ang), math.sin(ang), 0.0]); f = np.array([-math.sin(ang), math.cos(ang), 0.0])
t = math.radians(30); a = math.sin(t) * c + np.array([0, 0, -math.cos(t)]); q = quat_from_axes(a, -f)
log("wrench before", wrench())
r.gripper(0.0); r.spin(0.5); fg = r.fingers(); gap = fg[0] - fg[1]
log(f"finger gap after close = {gap:.4f}", "wrench", wrench())
if gap < 0.04:
    log("missed the can; reopening"); r.gripper(0.04); raise SystemExit
_, _, tcp = r.hand_pose(); log("tcp", tcp.round(4))
back = tcp - 0.12 * a
ok2 = move_tcp_iter(r, *back, q, secs=4, tries=2, tol=0.01); log("retreat", ok2)
r.spin(0.5); fg = r.fingers(); log("fingers after retreat", fg, "gap", round(fg[0] - fg[1], 4), "wrench", wrench())
up = np.array([CX - 0.05, CY, 0.70]) - 0.10 * a
ok3 = move_tcp_iter(r, *up, q, secs=4, tries=2, tol=0.01); log("lift", ok3)
r.spin(0.5); fg = r.fingers(); log("fingers after lift", fg, "gap", round(fg[0] - fg[1], 4), "wrench", wrench())
_, _, tcp = r.hand_pose(); log("tcp", tcp.round(4))
log("PHASE10B DONE")
EOF
nohup python3 -u phase10b.py > phase10b.log 2>&1 &
sleep 50; grep -v XMLPARSER phase10b.log

# openrua op 79
python3 tools/perception/cam_snap.py frontview p10b_front.png; python3 -c "
import cv2; im=cv2.imread('p10b_front.png'); cv2.imwrite('p10b_front_crop.png', cv2.resize(im[100:300,330:560],None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))"

# openrua op 80
cat > bmeas.py <<'EOF'
from refine import *
import cv2
r = Robot()
cen, pts = refine(r, 0.03, 0.63, 0.56, 0.68, rad=0.16, cam="birdview")
pts = np.asarray(pts)
rim = pts[pts[:, 2] > 0.60]
log("rim n", len(rim), "z range", rim[:, 2].min().round(3), rim[:, 2].max().round(3), "centroid", rim[:, :2].mean(0).round(4))
(cx, cy), (w, h), th = cv2.minAreaRect(rim[:, :2].astype(np.float32))
log("bbox center", round(cx, 4), round(cy, 4), "size", round(w, 3), round(h, 3), "angle", round(th, 1))
cen2, pts2 = refine(r, cx, cy, 0.48, 0.56, rad=0.09, cam="birdview")
pts2 = np.asarray(pts2); log("soup-top pts n", len(pts2), "center", pts2[:, :2].mean(0).round(4) if len(pts2) else None, "ztop", pts2[:, 2].max().round(3) if len(pts2) else None)
EOF
timeout 120 python3 -u bmeas.py 2>&1 | grep -v XMLPARSER

# openrua op 81
sed -i 's/rim = pts\[pts\[:, 2\] > 0.60\]/rim = pts[pts[:, 2] > 0.555]/' bmeas.py; timeout 120 python3 -u bmeas.py 2>&1 | grep -v XMLPARSER

# openrua op 82
python3 tools/perception/cam_snap.py birdview p10c_bird.png; python3 -c "
import cv2; im=cv2.imread('p10c_bird.png'); cv2.imwrite('p10c_bird_crop.png', cv2.resize(im[250:340,420:520],None,fx=5,fy=5,interpolation=cv2.INTER_CUBIC))"

# openrua op 83
cat > phase11.py <<'EOF'
from rob import *
from geometry_msgs.msg import WrenchStamped
r = Robot()
wr = {}
r.node.create_subscription(WrenchStamped, "/franka_robot_state_broadcaster/external_wrench", lambda m: wr.__setitem__('m', m), 1)
def wrench():
    wr.pop('m', None)
    for _ in range(50):
        rclpy.spin_once(r.node, timeout_sec=0.1)
        if 'm' in wr: break
    f = wr['m'].wrench.force; return np.round([f.x, f.y, f.z], 2)
PX, PY, PZ = 0.070, 0.607, 0.565
t = math.radians(30)
cands = []
for ang2 in (math.pi / 2, -math.pi / 2):
    c2 = np.array([math.cos(ang2), math.sin(ang2), 0.0]); f2 = np.array([-math.sin(ang2), math.cos(ang2), 0.0])
    a2 = math.sin(t) * c2 + np.array([0, 0, -math.cos(t)]); q2 = quat_from_axes(a2, -f2)
    s_lo = r.solve_ik(PX, PY, PZ, q2, seed=r.arm_q())
    s_hi = r.solve_ik(PX, PY, 0.76, q2, seed=s_lo if s_lo else r.arm_q())
    log(f"ang2 {math.degrees(ang2):.0f}: low {None if s_lo is None else np.round(s_lo,3)} high {None if s_hi is None else np.round(s_hi,3)}")
    if s_lo is not None and s_hi is not None: cands.append((ang2, q2, a2, s_hi, s_lo))
if not cands: log("no place IK; holding"); raise SystemExit
ang2, q2, a2, s_hi, s_lo = cands[0]; log("using ang2", math.degrees(ang2))
fg = r.fingers(); log("gap before transport", round(fg[0] - fg[1], 4))
ok = r.move_joints(s_hi, secs=6); log("above basket", ok)
_, _, tcp = r.hand_pose(); log("tcp", tcp.round(4), "wrench", wrench())
fg = r.fingers(); log("gap", round(fg[0] - fg[1], 4))
if round(fg[0] - fg[1], 4) < 0.04: log("lost the can!"); raise SystemExit
ok = move_tcp_iter(r, PX, PY, PZ, q2, secs=4, tries=3, tol=0.006); log("lowered", ok, "wrench", wrench())
_, _, tcp = r.hand_pose(); log("tcp", tcp.round(4))
if tcp[2] > PZ + 0.03: log("descent blocked; stopping for inspection"); raise SystemExit
r.gripper(0.04); r.spin(1.0); log("released, fingers", r.fingers(), "wrench", wrench())
ok = move_tcp_iter(r, PX, PY, 0.80, q2, secs=4, tries=2, tol=0.01); log("lifted", ok)
ok = r.move_tcp(-0.30, -0.15, 0.85, yaw=0.0, secs=6); log("parked", ok)
if not ok: r.move_tcp(-0.30, -0.15, 0.85, yaw=0.0, secs=6)
log("PHASE11 DONE")
EOF
nohup python3 -u phase11.py > phase11.log 2>&1 &
sleep 50; grep -v XMLPARSER phase11.log

# openrua op 84
sleep 45; grep -v XMLPARSER phase11.log | tail -n +12

# openrua op 85
sleep 25; grep -v XMLPARSER phase11.log | tail -3; python3 tools/perception/cam_snap.py birdview p11_bird.png; python3 tools/perception/cam_snap.py frontview p11_front.png; python3 -c "
import cv2; im=cv2.imread('p11_bird.png'); cv2.imwrite('p11_bird_crop.png', cv2.resize(im[250:340,420:520],None,fx=5,fy=5,interpolation=cv2.INTER_CUBIC))"

# openrua op 86
sleep 10; grep -v XMLPARSER phase11.log | tail -2; python3 tools/perception/cam_snap.py frontview p11b_front.png; python3 -c "
import cv2; im=cv2.imread('p11b_front.png'); cv2.imwrite('p11b_front_crop.png', cv2.resize(im[180:320,480:620],None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))"

# openrua op 87
cat > final_check.py <<'EOF'
from refine import *
r = Robot()
BX, BY = 0.032, 0.607
cen, pts = refine(r, BX, BY, 0.44, 0.545, rad=0.075, cam="birdview")
pts = np.asarray(pts)
inner = pts[(np.abs(pts[:, 0] - BX) < 0.07) & (np.abs(pts[:, 1] - BY) < 0.065)]
soup = inner[(inner[:, 2] > 0.505)]
tom = inner[(inner[:, 2] > 0.46) & (inner[:, 2] < 0.505)]
log("soup-lid pts", len(soup), "center", soup[:, :2].mean(0).round(3) if len(soup) else None, "z", soup[:, 2].max().round(3) if len(soup) else None)
log("tomato pts", len(tom), "center", tom[:, :2].mean(0).round(3) if len(tom) else None, "ztop", tom[:, 2].max().round(3) if len(tom) else None,
    "extent x", tom[:, 0].min().round(3), tom[:, 0].max().round(3), "y", tom[:, 1].min().round(3), tom[:, 1].max().round(3))
# anything left at the old can spot?
cen2, pts2 = refine(r, 0.269, 0.3535, 0.44, 0.52, rad=0.12, cam="birdview")
log("points at old can location:", len(pts2))
_, _, tcp = r.hand_pose(); log("arm tcp", tcp.round(3), "fingers", r.fingers())
EOF
timeout 120 python3 -u final_check.py 2>&1 | grep -v XMLPARSER

# openrua op 88
ls /claude-config/projects/-workspace/memory/ 2>/dev/null; cat /claude-config/projects/-workspace/memory/MEMORY.md 2>/dev/null

# openrua op 89
mkdir -p "$(dirname /claude-config/projects/-workspace/memory/panda-sim-machine-quirks.md)"
cat > /claude-config/projects/-workspace/memory/panda-sim-machine-quirks.md <<'OPENRUA_EOF'
---
name: panda-sim-machine-quirks
description: Measured quirks of the /workspace Panda ROS2 sim that contradict machine.yaml/docs (IK frame, tip link, gripper span, basket height, forearm-collision trap)
metadata:
  type: project
---

Facts measured on the /workspace Franka Panda sim (Sept 2026) that the starter docs get wrong or omit:
- /compute_ik wants poses in frame_id "world" (empty frame / base-relative fails with -31); its tip link is panda_link8 = panda_hand rotated +45° about z, so right-multiply the hand quaternion by (0,0,0.38268,0.92388).
- world->panda_link0 = (-0.51,0,0.42); table top z≈0.425; the robot base sits ON the table. Reach limit ~0.85 m; objects beyond r≈0.78 need a tilted (30°) grasp.
- Gripper opens to ~0.037-0.040 per finger (72-80 mm); cans are ~62-67 mm, so lateral error must be <5 mm — refine with the wrist cam (robot0_eye_in_hand) before the final descent.
- The wicker basket rim is at z≈0.58 (walls ~0.155 tall), not 0.62. Hand body must stay above the rim; release with TCP≈0.565 and the can horizontal, over free floor, not over another can (dropping from 0.67 onto the soup can bounced the tomato can out).
- Trajectory result -5 with joint2 capped ≈1.1 rad and NO wrist-wrench change = forearm (link5/6) resting on the basket rim; the wrist wrench sensor does not see upstream link contacts. Fix by pushing the basket out of the base->target corridor (push at TCP z≈0.52 with open fingers, yaw π/2).
- Large joint moves often return -5 first time (controller lag); a retry converges.

**Why:** these cost many iterations to discover; the docs' "leave frame_id empty" and rim-height assumptions were wrong.
**How to apply:** reuse /workspace/rob.py (Robot, move_tcp_iter, quat_from_axes), refine.py, scene.py; run phase scripts detached with `nohup python3 -u ... &` and poll the log.
OPENRUA_EOF

# openrua op 90
mkdir -p "$(dirname /claude-config/projects/-workspace/memory/MEMORY.md)"
cat > /claude-config/projects/-workspace/memory/MEMORY.md <<'OPENRUA_EOF'
- [Panda sim machine quirks](panda-sim-machine-quirks.md) — IK world frame/link8 tip, rim z≈0.58, forearm-on-basket trap, gripper span
OPENRUA_EOF
