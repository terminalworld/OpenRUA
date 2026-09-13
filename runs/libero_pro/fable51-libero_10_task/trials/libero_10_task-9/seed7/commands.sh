#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
cd /workspace; cat tools/action/fjt_send.py; echo ----; cat tools/action/gripper_cmd.py; echo ----; cat tools/action/ik_move.py

# openrua op 3
cd /workspace; ros2 topic list; echo ----; ros2 action list; echo ----; ros2 service list | grep -v -E "parameters|describe|get_type" | head -40; echo ----; ros2 node list

# openrua op 4
cd /workspace; mkdir -p snaps; for c in agentview frontview sideview birdview robot0_robotview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c snaps/$c.png; done; ros2 topic echo /joint_states --once

# openrua op 5
cd /workspace; timeout 20 ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id|child" | head -40; echo ---; timeout 15 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -8

# openrua op 6
cd /workspace; cat > tf_dump.py <<'EOF'
import rclpy, time
from tf2_ros import Buffer, TransformListener
rclpy.init(); n=rclpy.create_node("tfd"); b=Buffer(); TransformListener(b,n)
end=time.time()+5
while time.time()<end: rclpy.spin_once(n,timeout_sec=0.1)
print(b.all_frames_as_yaml())
for f in ["agentview_optical_frame","birdview_optical_frame","frontview_optical_frame","sideview_optical_frame","panda_hand","panda_link0"]:
    try:
        t=b.lookup_transform("world",f,rclpy.time.Time()).transform
        print(f, [round(t.translation.x,3),round(t.translation.y,3),round(t.translation.z,3)], [round(t.rotation.x,3),round(t.rotation.y,3),round(t.rotation.z,3),round(t.rotation.w,3)])
    except Exception as e: print(f, "ERR", e)
EOF
timeout 60 python3 tf_dump.py

# openrua op 7
cd /workspace; cat > pxw.py <<'EOF'
"""Usage: python3 pxw.py <cam> u,v u,v ...   -> world xyz per pixel; also saves depth npy"""
import sys, struct, numpy as np, rclpy, time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge
cam=sys.argv[1]; pts=[tuple(map(int,a.split(","))) for a in sys.argv[2:]]
rclpy.init(); n=rclpy.create_node("pxw"); b=Buffer(); TransformListener(b,n)
got={}
n.create_subscription(Image,f"/{cam}/depth/image_raw",lambda m:got.setdefault("d",m),1)
n.create_subscription(CameraInfo,f"/{cam}/color/camera_info",lambda m:got.setdefault("i",m),1)
end=time.time()+30
while time.time()<end and not ("d" in got and "i" in got and b.can_transform("world",f"{cam}_optical_frame",rclpy.time.Time())):
    rclpy.spin_once(n,timeout_sec=0.1)
d=CvBridge().imgmsg_to_cv2(got["d"],"passthrough").astype(np.float32); np.save(f"snaps/{cam}_depth.npy",d)
k=got["i"].k; fx,fy,cx,cy=k[0],k[4],k[2],k[5]
t=b.lookup_transform("world",f"{cam}_optical_frame",rclpy.time.Time()).transform
q=t.rotation; x,y,z,w=q.x,q.y,q.z,q.w
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
T=np.array([t.translation.x,t.translation.y,t.translation.z])
print("cam",cam,"size",d.shape,"fx",fx,"cx",cx,"cy",cy)
for u,v in pts:
    Z=d[v,u]; p=R@np.array([(u-cx)*Z/fx,(v-cy)*Z/fy,Z])+T
    print(f"px({u},{v}) depth={Z:.3f} world=({p[0]:.3f},{p[1]:.3f},{p[2]:.3f})")
rclpy.shutdown()
EOF
timeout 90 python3 pxw.py birdview 243,265 325,295 420,300 360,255 330,265 395,250 300,400 200,200

# openrua op 8
cd /workspace; cat > seg.py <<'EOF'
import numpy as np, cv2
d=np.load("snaps/birdview_depth.npy"); fx=579.4112549695428; cx=320; cy=240; camz=3.0; camx=-0.2
h=camz-d  # world z
def w(u,v): Z=d[v,u]; return (camx+(v-cy)*Z/fx, (u-cx)*Z/fx, camz-Z)  # x=down in image, y=right
# objects above table (z>0.93) in region rows 150-420
mask=(h>0.93)&(h<1.5); mask[:150,:]=0; mask[:, :100]=0
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask.astype(np.uint8))
for i in range(1,n):
    x,y,ww,hh,a=stats[i]
    if a<30: continue
    ys,xs=np.where(lab==i); zs=h[ys,xs]
    print(f"comp{i}: px bbox u[{x},{x+ww}] v[{y},{y+hh}] area={a} zmax={zs.max():.3f} zmed={np.median(zs):.3f} centroid_world={w(int(cent[i][0]),int(cent[i][1]))}")
    print("   corners world:", w(x,y)[:2], w(x+ww-1,y+hh-1)[:2])
EOF
python3 seg.py

# openrua op 9
cd /workspace; cat > seg2.py <<'EOF'
import numpy as np
d=np.load("snaps/birdview_depth.npy"); fx=579.4112549695428; cx=320; cy=240; camz=3.0; camx=-0.2
h=camz-d
np.set_printoptions(linewidth=250, precision=2)
print("white mug region heights (rows 248-282, cols 224-270, step 2):")
print(h[248:283:2, 224:271:2])
print("\nmicrowave region rows 236-360 step 4, cols 300-464 step 4:")
print(h[236:361:4, 300:465:4])
EOF
python3 seg2.py

# openrua op 10
cd /workspace; timeout 90 python3 pxw.py sideview 325,212 325,240 285,245 285,262 300,300 200,320 300,235; timeout 90 python3 pxw.py agentview 160,185 160,240 335,215 335,290 400,160 300,400

# openrua op 11
cd /workspace; python3 -c "
import cv2
im=cv2.imread('snaps/agentview.png'); crop=im[140:400,280:640]; cv2.imwrite('snaps/agent_mw.png', cv2.resize(crop,None,fx=2,fy=2,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('snaps/frontview.png'); crop=im[240:450,280:640]; cv2.imwrite('snaps/front_mw.png', cv2.resize(crop,None,fx=2,fy=2,interpolation=cv2.INTER_CUBIC))
"

# openrua op 12
cd /workspace; cat > cav.py <<'EOF'
import numpy as np, rclpy, time
from tf2_ros import Buffer, TransformListener
d=np.load("snaps/agentview_depth.npy"); fx=579.4112549695428; cx=320; cy=240
rclpy.init(); n=rclpy.create_node("c"); b=Buffer(); TransformListener(b,n)
end=time.time()+5
while time.time()<end and not b.can_transform("world","agentview_optical_frame",rclpy.time.Time()): rclpy.spin_once(n,timeout_sec=0.1)
t=b.lookup_transform("world","agentview_optical_frame",rclpy.time.Time()).transform
q=t.rotation; x,y,z,w=q.x,q.y,q.z,q.w
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
T=np.array([t.translation.x,t.translation.y,t.translation.z])
def W(u,v): Z=d[v,u]; return R@np.array([(u-cx)*Z/fx,(v-cy)*Z/fx,Z])+T
np.set_printoptions(precision=3, suppress=True, linewidth=200)
for v in range(160,340,10):
    print(v, " ".join(f"[{u}:{W(u,v)[0]:.2f},{W(u,v)[1]:.2f},{W(u,v)[2]:.2f}]" for u in range(440,530,10)))
rclpy.shutdown()
EOF
timeout 60 python3 cav.py

# openrua op 13
cd /workspace; cat > side.py <<'EOF'
import numpy as np, rclpy, time
from tf2_ros import Buffer, TransformListener
cam="sideview"
d=np.load(f"snaps/{cam}_depth.npy"); fx=579.4112549695428; cx=320; cy=240
rclpy.init(); n=rclpy.create_node("c"); b=Buffer(); TransformListener(b,n)
end=time.time()+5
while time.time()<end and not b.can_transform("world",f"{cam}_optical_frame",rclpy.time.Time()): rclpy.spin_once(n,timeout_sec=0.1)
t=b.lookup_transform("world",f"{cam}_optical_frame",rclpy.time.Time()).transform
q=t.rotation; x,y,z,w=q.x,q.y,q.z,q.w
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
T=np.array([t.translation.x,t.translation.y,t.translation.z])
def W(u,v): Z=d[v,u]; return R@np.array([(u-cx)*Z/fx,(v-cy)*Z/fx,Z])+T
for v in range(200,270,4):
    print(v, " ".join(f"[{u}:{W(u,v)[0]:.2f},{W(u,v)[1]:.2f},{W(u,v)[2]:.2f}]" for u in range(310,350,6)))
rclpy.shutdown()
EOF
timeout 60 python3 side.py

# openrua op 14
cd /workspace; python3 -c "
import numpy as np
d=np.load('snaps/birdview_depth.npy'); h=3.0-d
np.set_printoptions(linewidth=300, precision=2)
fx=579.4112549695428
for v in [246,250,256,262,270,280,290,300,310,320,330,340,350,356,360]:
    row=h[v,380:470:3]
    print(f'v={v} x={-0.2+(v-240)*1.9/fx:.3f}:', ' '.join(f'{z:.2f}' for z in row))
print('cols->y:', ' '.join(f'{(u-320)*1.9/fx:.2f}' for u in range(380,470,3)))
"


# openrua op 15
cd /workspace; python3 -c "
import numpy as np
d=np.load('snaps/sideview_depth.npy'); fx=579.4112549695428
for v in range(206,258,2):
    row=d[v,300:360]; m=(row>1.5)&(row<1.7)
    idx=np.where(m)[0]
    if len(idx): 
        Z=row[idx].mean(); print(v, 'cols',300+idx[0],300+idx[-1],'width_m=%.3f'%((idx[-1]-idx[0]+1)*Z/fx),'Z=%.3f'%Z, 'z_world~%.3f'%(1.488-(v-240)*Z/fx*0.999))
"


# openrua op 16
cd /workspace; cat > get_urdf.py <<'EOF'
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
rclpy.init(); n=rclpy.create_node("u")
qos=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
got=[]
n.create_subscription(String,"/robot_description",lambda m:got.append(m.data),qos)
import time; end=time.time()+10
while not got and time.time()<end: rclpy.spin_once(n,timeout_sec=0.2)
open("robot.urdf","w").write(got[0] if got else "")
print(len(got[0]) if got else "none")
EOF
timeout 30 python3 get_urdf.py; grep -n -A12 'link name="panda_hand"' robot.urdf | head -60; grep -n -B2 -A8 'link name="panda_leftfinger"' robot.urdf | head -40

# openrua op 17
cd /workspace; python3 -c "
import numpy as np, struct
def stl_bbox(p):
    data=open(p,'rb').read()
    if data[:5]==b'solid' and b'facet' in data[:300]:
        import re
        v=np.array([list(map(float,m)) for m in re.findall(rb'vertex\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)',data)])
    else:
        n=struct.unpack('<I',data[80:84])[0]
        arr=np.frombuffer(data[84:84+n*50],dtype=np.dtype([('n','<3f4'),('v','<9f4'),('a','<u2')]))
        v=arr['v'].reshape(-1,3)
    return v.min(0),v.max(0)
for f in ['hand','finger','link7']:
    print(f, stl_bbox(f'/opt/ros/jazzy/share/moveit_resources_panda_description/meshes/collision/{f}.stl'))
"


# openrua op 18
find / -iname "hand*.stl" -o -iname "finger*.stl" 2>/dev/null | head; find / -path "*panda*" -name "*.stl" 2>/dev/null | head

# openrua op 19
cd /workspace; timeout 300 python3 tools/action/gripper_cmd.py 0.04; ros2 topic echo /joint_states --once | grep -A9 position

# openrua op 20
cd /workspace; timeout 60 python3 tools/perception/cam_snap.py birdview snaps/birdview2.png; timeout 90 python3 pxw.py birdview 243,265 325,295 >/dev/null; python3 -c "
import numpy as np
d=np.load('snaps/birdview_depth.npy'); h=3.0-d; fx=579.4112549695428
np.set_printoptions(linewidth=250, precision=2)
print(h[248:283:2, 224:271:2])
# mug rim center estimate
m=(h>1.0)&(h<1.03); m[:,280:]=0; m[:240,:]=0
ys,xs=np.where(m); print('rim px center',xs.mean(),ys.mean(), 'z rim', h[ys,xs].max())
Z=d[ys,xs].mean(); print('world', -0.2+(ys.mean()-240)*Z/fx, (xs.mean()-320)*Z/fx)
"

# openrua op 21
cd /workspace; timeout 90 python3 pxw.py frontview 320,240 > /dev/null; cat > cav2.py <<'EOF'
import numpy as np, rclpy, time, sys
from tf2_ros import Buffer, TransformListener
cam=sys.argv[1]; u0,u1,du,v0,v1,dv=map(int,sys.argv[2:8])
d=np.load(f"snaps/{cam}_depth.npy"); fx=579.4112549695428; cx=320; cy=240
rclpy.init(); n=rclpy.create_node("c"); b=Buffer(); TransformListener(b,n)
end=time.time()+5
while time.time()<end and not b.can_transform("world",f"{cam}_optical_frame",rclpy.time.Time()): rclpy.spin_once(n,timeout_sec=0.1)
t=b.lookup_transform("world",f"{cam}_optical_frame",rclpy.time.Time()).transform
q=t.rotation; x,y,z,w=q.x,q.y,q.z,q.w
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
T=np.array([t.translation.x,t.translation.y,t.translation.z])
def W(u,v): Z=d[v,u]; return R@np.array([(u-cx)*Z/fx,(v-cy)*Z/fx,Z])+T
for v in range(v0,v1,dv):
    print(v, " ".join(f"[{u}:{W(u,v)[0]:.2f},{W(u,v)[1]:.2f},{W(u,v)[2]:.2f}]" for u in range(u0,u1,du)))
rclpy.shutdown()
EOF
timeout 60 python3 cav2.py frontview 425 475 5 260 340 5

# openrua op 22
cd /workspace; cat > rlib.py <<'EOF'
"""Robot helper library: persistent node, FK/IK, trajectory, gripper, TF."""
import time, math, numpy as np, rclpy, yaml
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from geometry_msgs.msg import Pose
from tf2_ros import Buffer, TransformListener

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
JOINTS = FJT["joints"]
BASE = np.array([-0.66, 0.0, 0.912])   # world -> panda_link0 translation (TF)

class Robot:
    def __init__(self, name="rlib"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self.js = {}
        self.node.create_subscription(JointState, "/joint_states", self._js_cb, 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.tfbuf = Buffer(); TransformListener(self.tfbuf, self.node)
        self.fjt.wait_for_server(10); self.grip.wait_for_server(10)
        self.ik.wait_for_service(10); self.fk.wait_for_service(10)
        self.spin(0.5)

    def _js_cb(self, m):
        self.js = dict(zip(m.name, m.position))

    def spin(self, t):
        end = time.time() + t
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def joints(self):
        self.js = {}
        while not self.js:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return [self.js[j] for j in JOINTS]

    def fingers(self):
        self.joints()
        return self.js.get("panda_finger_joint1"), self.js.get("panda_finger_joint2")

    # ---- kinematics (poses in WORLD frame; converted to base for MoveIt) ----
    def fk_world(self, q=None, link="panda_hand"):
        q = q if q is not None else self.joints()
        req = GetPositionFK.Request()
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = list(JOINTS)
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            raise RuntimeError(f"FK failed {r.error_code.val if r else None}")
        p = r.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE
        quat = np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        return pos, quat

    def ik_world(self, pos, quat, seed=None, attempts=3, timeout=5.0):
        """pos: world xyz of panda_hand frame; quat xyzw. Returns joint list or None."""
        seed = seed if seed is not None else self.joints()
        pb = np.array(pos) - BASE
        for _ in range(attempts):
            req = GetPositionIK.Request()
            req.ik_request.group_name = M["planning"]["group"]
            req.ik_request.pose_stamped.header.frame_id = ""
            p = req.ik_request.pose_stamped.pose
            p.position.x, p.position.y, p.position.z = map(float, pb)
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
            req.ik_request.robot_state.joint_state.name = list(JOINTS)
            req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
            req.ik_request.timeout.sec = int(timeout)
            req.ik_request.timeout.nanosec = int((timeout % 1) * 1e9)
            req.ik_request.avoid_collisions = False
            fut = self.ik.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            r = fut.result()
            if r is not None and r.error_code.val == 1:
                sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
                return [sol[j] for j in JOINTS]
        return None

    # ---- motion ----
    def move_joints(self, points, seconds, wait=True):
        """points: list of joint lists (waypoints), evenly timed to `seconds` total."""
        if not isinstance(points[0], (list, tuple, np.ndarray)):
            points = [points]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        n = len(points)
        for i, q in enumerate(points):
            t = seconds * (i + 1) / n
            pt = JointTrajectoryPoint(positions=[float(v) for v in q])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        gh = send.result()
        if not gh.accepted:
            raise RuntimeError("trajectory goal rejected")
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, res)
        code = res.result().result.error_code
        q = self.joints()
        err = max(abs(a - b) for a, b in zip(q, points[-1]))
        print(f"  traj done code={code} max_joint_err={err:.4f}")
        return code, err

    def gripper(self, width):
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = float(GRIP.get("max_effort", 30.0))
        send = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=30)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=120)
        r = res.result().result
        f = self.fingers()
        print(f"  gripper -> reached={r.reached_goal} stalled={r.stalled} fingers={f}")
        return f

    def tf(self, target, source):
        end = time.time() + 5
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.tfbuf.can_transform(target, source, rclpy.time.Time()):
                break
        t = self.tfbuf.lookup_transform(target, source, rclpy.time.Time()).transform
        return (np.array([t.translation.x, t.translation.y, t.translation.z]),
                np.array([t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w]))

def quat_from_R(R):
    from scipy.spatial.transform import Rotation
    return Rotation.from_matrix(R).as_quat()  # xyzw

def R_from_axes(xh, yh, zh):
    return np.column_stack([xh, yh, zh])

def hand_pose_from_tcp(tcp, R, tcp_offset=M["hand"]["tcp_offset_m"]):
    """world position of panda_hand frame such that the TCP (fingertip centre) lands on `tcp`."""
    return np.array(tcp) - tcp_offset * R[:, 2]
EOF
cat > t_fk.py <<'EOF'
from rlib import *
r = Robot()
print("joints", np.round(r.joints(),3))
print("FK hand world", r.fk_world())
print("TF  hand world", r.tf("world","panda_hand"))
EOF
timeout 120 python3 t_fk.py

# openrua op 23
cd /workspace; cat > t_ik.py <<'EOF'
from rlib import *
import rlib
r = Robot()
q0 = r.joints()
pos, quat = r.tf("world","panda_hand")
rlib.BASE = np.zeros(3)   # try raw world coords
s = r.ik_world(pos, quat, seed=q0, attempts=1)
print("IK(world coords):", None if s is None else np.round(s,3))
rlib.BASE = np.array([-0.66,0,0.912])
s2 = r.ik_world(pos, quat, seed=q0, attempts=1)
print("IK(base coords):", None if s2 is None else np.round(s2,3))
print("current:", np.round(q0,3))
EOF
timeout 200 python3 t_ik.py

# openrua op 24
cd /workspace; sed -i 's/^BASE = np.array(\[-0.66, 0.0, 0.912\]).*/BASE = np.zeros(3)   # verified: FK\/IK model frame == world here/' rlib.py; grep -n "^BASE" rlib.py

# openrua op 25
cd /workspace; cat > s1_hover.py <<'EOF'
from rlib import *
r = Robot()
R = np.array([[-1,0,0],[0,1,0],[0,0,-1]], float)
quat = quat_from_R(R); print("quat", quat)
mug = np.array([-0.118, -0.268]); rim_z = 1.01
tcp = np.array([mug[0], mug[1]-0.044, 1.12])
hand = hand_pose_from_tcp(tcp, R)
q = r.ik_world(hand, quat)
print("IK:", None if q is None else np.round(q,3))
if q is not None:
    r.move_joints([q], 4.0)
    print("hand now", np.round(r.fk_world()[0],3), "target", np.round(hand,3))
EOF
timeout 600 python3 s1_hover.py

# openrua op 26
cd /workspace; ros2 topic echo /joint_states --once | grep -A7 position: | tr -d ' -' | tr '\n' ' '; echo

# openrua op 27
cd /workspace; cat > resend.py <<'EOF'
import sys
from rlib import *
r = Robot()
q = [float(x) for x in sys.argv[1].split(",")]
r.move_joints([q], float(sys.argv[2]) if len(sys.argv)>2 else 3.0)
print(np.round(r.joints(),3))
EOF
timeout 600 python3 resend.py -0.395,0.364,-0.138,-1.9,0.063,2.26,2.578 3

# openrua op 28
cd /workspace; timeout 600 python3 resend.py -0.395,0.364,-0.138,-1.9,0.063,2.26,2.578 4

# openrua op 29
cd /workspace; timeout 60 python3 tools/perception/cam_snap.py birdview snaps/bv_hover.png; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih_hover.png; python3 -c "
import cv2; im=cv2.imread('snaps/bv_hover.png'); cv2.imwrite('snaps/bv_hover_crop.png', cv2.resize(im[200:330,160:330],None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))"

# openrua op 30
cd /workspace; timeout 90 python3 pxw.py robot0_eye_in_hand 365,300 365,330 300,340 430,340 365,370 100,380 560,380 && python3 -c "
import numpy as np
d=np.load('snaps/robot0_eye_in_hand_depth.npy'); print(d.shape, np.nanmin(d), np.nanmax(d))
np.set_printoptions(linewidth=250, precision=2)
print(d[280:420:10, 280:460:10])
"

# openrua op 31
cd /workspace; cat > rimfit.py <<'EOF'
import numpy as np, rclpy, time, sys
from tf2_ros import Buffer, TransformListener
cam="robot0_eye_in_hand"
d=np.load(f"snaps/{cam}_depth.npy"); fx=312.77408948188935; cx=320; cy=240
rclpy.init(); n=rclpy.create_node("c"); b=Buffer(); TransformListener(b,n)
end=time.time()+5
while time.time()<end and not b.can_transform("world",f"{cam}_optical_frame",rclpy.time.Time()): rclpy.spin_once(n,timeout_sec=0.1)
t=b.lookup_transform("world",f"{cam}_optical_frame",rclpy.time.Time()).transform
q=t.rotation; x,y,z,w=q.x,q.y,q.z,q.w
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
T=np.array([t.translation.x,t.translation.y,t.translation.z])
print("cam pos",T)
vs,us=np.mgrid[0:480,0:640]
Z=d; P=np.stack([(us-cx)*Z/fx,(vs-cy)*Z/fx,Z],-1)@R.T+T
zw=P[...,2]
m=(zw>1.0)&(zw<1.02)&(vs<400)
pts=P[m][:,:2]; print("rim pts",len(pts))
# algebraic circle fit
A=np.c_[2*pts, np.ones(len(pts))]; bb=(pts**2).sum(1)
c=np.linalg.lstsq(A,bb,rcond=None)[0]; cx_,cy_=c[0],c[1]; r_=np.sqrt(c[2]+cx_**2+cy_**2)
print(f"rim circle centre=({cx_:.4f},{cy_:.4f}) r={r_:.4f}")
EOF
timeout 60 python3 rimfit.py

# openrua op 32
cd /workspace; cat > s2_descend.py <<'EOF'
import sys
from rlib import *
r = Robot()
R = np.array([[-1,0,0],[0,1,0],[0,0,-1]], float); quat = quat_from_R(R)
c = np.array([-0.1174, -0.2675])
z = float(sys.argv[1])
tcp = np.array([c[0], c[1]-0.044, z])
hand = hand_pose_from_tcp(tcp, R)
q = r.ik_world(hand, quat)
print("IK:", None if q is None else np.round(q,3))
if q is not None:
    for i in range(3):
        code, err = r.move_joints([q], 3.0)
        if err < 0.01: break
    pos, qt = r.fk_world()
    print("hand", np.round(pos,4), "tcp", np.round(pos + 0.1034*R[:,2],4), "quat", np.round(qt,3))
EOF
timeout 900 python3 s2_descend.py 1.03

# openrua op 33
cd /workspace; sed -i 's/            req.ik_request.group_name = M\["planning"\]\["group"\]/            req.ik_request.group_name = M["planning"]["group"]\n            req.ik_request.ik_link_name = "panda_hand"/' rlib.py; grep -n ik_link_name rlib.py; timeout 900 python3 s2_descend.py 1.03

# openrua op 34
cd /workspace; cat > s2b.py <<'EOF'
import sys
from rlib import *
r = Robot()
R = np.array([[1,0,0],[0,-1,0],[0,0,-1]], float); quat = quat_from_R(R)   # closing axis -y (same grasp, j7 far from limit)
c = np.array([-0.1174, -0.2675])
z = float(sys.argv[1])
tcp = np.array([c[0], c[1]-0.044, z])
hand = hand_pose_from_tcp(tcp, R)
seed = [-0.405, 0.515, -0.122, -1.927, 0.093, 2.437, 0.2]
q = r.ik_world(hand, quat, seed=seed)
print("IK:", None if q is None else np.round(q,3))
if q is not None:
    for i in range(4):
        code, err = r.move_joints([q], 4.0)
        if err < 0.01: break
    pos, qt = r.fk_world()
    print("hand", np.round(pos,4), "tcp", np.round(pos + 0.1034*R[:,2],4), "quat", np.round(qt,3))
EOF
timeout 900 python3 s2b.py 1.03

# openrua op 35
cd /workspace; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih2.png

# openrua op 36
cd /workspace; timeout 60 python3 pxw.py robot0_eye_in_hand 320,240 >/dev/null; timeout 60 python3 rimfit.py; python3 - <<'EOF'
from rlib import *
r=Robot()
for f in ["panda_hand","panda_leftfinger","panda_rightfinger"]:
    print(f, np.round(r.tf("world",f)[0],4))
EOF

# openrua op 37
cd /workspace; sed -i 's/c = np.array(\[-0.1174, -0.2675\])/c = np.array([-0.114, -0.271])/' s2b.py; timeout 900 python3 s2b.py 0.98

# openrua op 38
cd /workspace; timeout 60 python3 tools/perception/cam_snap.py agentview snaps/agent_grasp.png; python3 -c "
import cv2; im=cv2.imread('snaps/agent_grasp.png'); cv2.imwrite('snaps/agent_grasp_crop.png', cv2.resize(im[100:330,60:300],None,fx=2.5,fy=2.5,interpolation=cv2.INTER_CUBIC))"

# openrua op 39
cd /workspace; timeout 300 python3 -c "
from rlib import *
r=Robot(); r.gripper(0.0)
"

# openrua op 40
cd /workspace; timeout 900 python3 s2b.py 1.10 && timeout 60 python3 tools/perception/cam_snap.py agentview snaps/agent_lift.png && python3 -c "
import cv2; im=cv2.imread('snaps/agent_lift.png'); cv2.imwrite('snaps/agent_lift_crop.png', cv2.resize(im[60:330,40:300],None,fx=2.5,fy=2.5,interpolation=cv2.INTER_CUBIC))"

# openrua op 41
cd /workspace; cat > cart.py <<'EOF'
"""Cartesian-interpolated move for the TCP: waypoints via IK (seeded sequentially)."""
from rlib import *
from scipy.spatial.transform import Rotation, Slerp

def cart_path(r, tcp0, R0, tcp1, R1, n):
    """returns list of joint solutions along a straight TCP line with slerp rotation."""
    key = Rotation.from_matrix([R0, R1]); sl = Slerp([0, 1], key)
    seed = r.joints(); qs = []
    for s in np.linspace(0, 1, n + 1)[1:]:
        Rs = sl(s).as_matrix()
        tcp = (1 - s) * np.asarray(tcp0) + s * np.asarray(tcp1)
        hand = hand_pose_from_tcp(tcp, Rs)
        q = r.ik_world(hand, quat_from_R(Rs), seed=seed, attempts=2)
        if q is None:
            raise RuntimeError(f"IK failed at s={s:.2f} tcp={tcp}")
        jump = max(abs(a - b) for a, b in zip(q, seed))
        print(f"  s={s:.2f} tcp={np.round(tcp,3)} jump={jump:.3f} q={np.round(q,2)}")
        seed = q; qs.append(q)
    return qs

def current_tcp(r, R):
    pos, _ = r.fk_world()
    return pos + M["hand"]["tcp_offset_m"] * R[:, 2]

def run(qs, r, seconds, retries=3):
    for i in range(retries):
        code, err = r.move_joints(qs, seconds)
        if err < 0.01: return True
        qs = [qs[-1]]; seconds = max(3.0, seconds/2)
    return False
EOF
cat > s3_reorient.py <<'EOF'
from cart import *
r = Robot()
Rg = np.array([[1,0,0],[0,-1,0],[0,0,-1]], float)
Rf = np.array([[-1,0,0],[0,0,1],[0,1,0]], float)
tcp0 = current_tcp(r, Rg); print("tcp now", np.round(tcp0,3))
# (a) straight up
qs = cart_path(r, tcp0, Rg, [tcp0[0], tcp0[1], 1.32], Rg, 2)
run(qs, r, 3.0)
# (b) reorient in place
tcpA = current_tcp(r, Rg)
qs = cart_path(r, tcpA, Rg, tcpA, Rf, 6)
print("max jump ok?"); run(qs, r, 8.0)
pos, qt = r.fk_world(); print("hand", np.round(pos,3), "quat", np.round(qt,3), "want", np.round(quat_from_R(Rf),3))
print("fingers", r.fingers())
EOF
timeout 1500 python3 s3_reorient.py

# openrua op 42
cd /workspace; timeout 900 python3 resend.py -0.84,0.97,-0.02,-0.83,1.33,0.75,-2.7 4; timeout 600 python3 resend.py -0.84,0.97,-0.02,-0.83,1.33,0.75,-2.7 3; timeout 60 python3 tools/perception/cam_snap.py frontview snaps/front_reorient.png

# openrua op 43
cd /workspace; cat > s4_preinsert.py <<'EOF'
import sys
from cart import *
r = Robot()
Rf = np.array([[-1,0,0],[0,0,1],[0,1,0]], float)
tcp0 = current_tcp(r, Rf); print("tcp now", np.round(tcp0,3))
goal = np.array([float(v) for v in sys.argv[1:4]])
via = np.array([goal[0], min(goal[1], -0.03), 1.25]) if len(sys.argv) > 4 else None
if via is not None:
    qs = cart_path(r, tcp0, Rf, via, Rf, 3); run(qs, r, 4.0)
    tcp0 = current_tcp(r, Rf)
qs = cart_path(r, tcp0, Rf, goal, Rf, 4); run(qs, r, 4.0)
pos, qt = r.fk_world(); print("tcp", np.round(current_tcp(r, Rf),4), "quat", np.round(qt,3), "fingers", r.fingers())
EOF
timeout 1500 python3 s4_preinsert.py -0.05 0.13 1.06 via

# openrua op 44
cd /workspace; cat > ikscan.py <<'EOF'
import sys
from cart import *
r = Robot()
Rf = np.array([[-1,0,0],[0,0,1],[0,1,0]], float)
tcp = np.array([float(v) for v in sys.argv[1:4]])
hand = hand_pose_from_tcp(tcp, Rf); quat = quat_from_R(Rf)
lim = np.array(FJT["limits_rad"])
rng = np.random.default_rng(0)
sols = []
for i in range(25):
    seed = rng.uniform(lim[:,0], lim[:,1])
    q = r.ik_world(hand, quat, seed=seed, attempts=1, timeout=1.0)
    if q is not None:
        q = np.round(q, 2)
        if not any(np.allclose(q, s, atol=0.05) for s in sols):
            sols.append(q); print(q, "margin j7", round(2.9-abs(q[6]),2))
print(len(sols), "distinct")
EOF
timeout 600 python3 ikscan.py -0.05 0.13 1.06

# openrua op 45
cd /workspace; cat > rollscan.py <<'EOF'
import sys
from cart import *
r = Robot()
Rf = np.array([[-1,0,0],[0,0,1],[0,1,0]], float)
mugc = np.array([float(v) for v in sys.argv[1:4]])   # desired mug centre (world)
lim = np.array(FJT["limits_rad"]); rng = np.random.default_rng(1)
for deg in range(-90, 91, 15):
    Ry = Rotation.from_euler('y', deg, degrees=True).as_matrix()
    R = Ry @ Rf
    tcp = mugc + 0.044 * R[:,1]          # mug centre = tcp - 0.044*y_h
    hand = hand_pose_from_tcp(tcp, R); quat = quat_from_R(R)
    best = None
    seeds = [r.joints()] + [rng.uniform(lim[:,0], lim[:,1]) for _ in range(6)]
    for seed in seeds:
        q = r.ik_world(hand, quat, seed=seed, attempts=1, timeout=0.5)
        if q is not None:
            m = min(min(qi - lo, hi - qi) for qi, (lo, hi) in zip(q, lim))
            if best is None or m > best[0]: best = (m, np.round(q,2))
    print(f"roll {deg:4d}: tcp={np.round(tcp,3)} y_h={np.round(R[:,1],2)} ", "none" if best is None else f"margin={best[0]:.2f} q={best[1]}")
EOF
timeout 900 python3 rollscan.py -0.05 0.13 1.013

# openrua op 46
cd /workspace; timeout 60 python3 tools/perception/cam_snap.py sideview snaps/side_hand.png; timeout 60 python3 tools/perception/cam_snap.py frontview snaps/front_hand.png; python3 -c "
from rlib import *
r=Robot(); pos,qt=r.fk_world(); print('hand',np.round(pos,3),np.round(qt,3)); print(np.round(r.joints(),3))"

# openrua op 47
cd /workspace; timeout 90 python3 pxw.py frontview 355,222 355,228 355,235 355,300 355,325 355,332 355,338 355,345 400,290 400,300 375,300 2>&1 | tail -15

# openrua op 48
cd /workspace; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih_front.png; timeout 60 python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw snaps/eih_front_d.png

# openrua op 49
cd /workspace; timeout 90 python3 pxw.py robot0_eye_in_hand 150,100 150,108 150,114 150,118 150,122 150,128 150,140 300,180 305,180 290,180 280,180 35,180 25,180 20,180 15,180 150,200 200,60 2>&1 | tail -18

# openrua op 50
cd /workspace; cat > cav3.py <<'EOF'
import numpy as np, sys
from rlib import *
import rclpy
r=Robot()
d=np.load("snaps/robot0_eye_in_hand_depth.npy")
fx=312.77408948188935; cx=320; cy=240
pos,quat=r.tf("world","robot0_eye_in_hand_optical_frame")
from scipy.spatial.transform import Rotation as Ro
R=Ro.from_quat(quat).as_matrix()
v,u=np.mgrid[0:480,0:640]
z=d; X=(u-cx)*z/fx; Y=(v-cy)*z/fx
P=np.stack([X,Y,z],-1).reshape(-1,3)@R.T+pos
ok=np.isfinite(z.reshape(-1))&(z.reshape(-1)>0.05)
P=P[ok]
print("cam",np.round(pos,3))
face=P[(P[:,1]>0.235)&(P[:,1]<0.27)]
print("front face pts",len(face))
# histogram in z, x
for zlo in np.arange(0.88,1.20,0.02):
    s=face[(face[:,2]>=zlo)&(face[:,2]<zlo+0.02)]
    if len(s): print(f"face z[{zlo:.2f},{zlo+0.02:.2f}) n={len(s)} x[{s[:,0].min():.3f},{s[:,0].max():.3f}]")
inside=P[(P[:,1]>0.29)&(P[:,1]<0.46)]
print("inside pts",len(inside))
for zlo in np.arange(0.88,1.20,0.02):
    s=inside[(inside[:,2]>=zlo)&(inside[:,2]<zlo+0.02)]
    if len(s): print(f"in z[{zlo:.2f},{zlo+0.02:.2f}) n={len(s)} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]")
for xlo in np.arange(-0.25,0.15,0.02):
    s=inside[(inside[:,0]>=xlo)&(inside[:,0]<xlo+0.02)]
    if len(s): print(f"in x[{xlo:.2f},{xlo+0.02:.2f}) n={len(s)} z[{s[:,2].min():.3f},{s[:,2].max():.3f}]")
EOF
timeout 60 python3 cav3.py

# openrua op 51
cd /workspace; cat cart.py | head -60; cat rollscan.py | head -40

# openrua op 52
cd /workspace; cat > s5_roll.py <<'EOF'
"""Step A: roll hand +60deg about world y while moving TCP to a safe pre-insert pose."""
import sys
from cart import *
r = Robot()
Rf = np.array([[-1,0,0],[0,0,1],[0,1,0]], float)
deg = float(sys.argv[1]); tgt = np.array([float(v) for v in sys.argv[2:5]])
pos, quat = r.fk_world(); R0 = Rotation.from_quat(quat).as_matrix()
tcp0 = pos + 0.1034 * R0[:, 2]
R1 = Rotation.from_euler('y', deg, degrees=True).as_matrix() @ Rf
print("tcp0", np.round(tcp0,3), "-> ", tgt, "q0", np.round(r.joints(),2))
qs = cart_path(r, tcp0, R0, tgt, R1, 8)
if "--go" in sys.argv:
    q0 = np.array(r.joints()); tot = max(abs(np.array(qs[-1]) - q0))
    secs = max(6.0, 6.0 * abs(qs[-1][6] - q0[6]) + 2.0, tot * 4)
    print("running", secs, "s"); ok = run(qs, r, secs)
    print("ok", ok, "q", np.round(r.joints(),3), "fingers", np.round(r.fingers(),4))
    pos, quat = r.fk_world(); R = Rotation.from_quat(quat).as_matrix()
    print("tcp", np.round(pos + 0.1034*R[:,2],3), "y_h", np.round(R[:,1],2), "z_h", np.round(R[:,2],2))
EOF
timeout 120 python3 s5_roll.py 60 0.0 0.16 1.10

# openrua op 53
cd /workspace; timeout 300 python3 s5_roll.py 60 0.0 0.16 1.10 --go 2>&1 | tail -5

# openrua op 54
cd /workspace; timeout 60 python3 tools/perception/cam_snap.py frontview snaps/front_roll60.png

# openrua op 55
cd /workspace; timeout 60 python3 tools/perception/cam_snap.py birdview snaps/bird_roll60.png; timeout 60 python3 tools/perception/cam_snap.py agentview snaps/agent_roll60.png

# openrua op 56
cd /workspace; cat > cav4.py <<'EOF'
import numpy as np
from rlib import *
from scipy.spatial.transform import Rotation as Ro
r=Robot()
d=np.load("snaps/robot0_eye_in_hand_depth.npy")
fx=312.77408948188935; cx=320; cy=240
pos,quat=r.tf("world","robot0_eye_in_hand_optical_frame")
R=Ro.from_quat(quat).as_matrix()
v,u=np.mgrid[0:480,0:640]
z=d; X=(u-cx)*z/fx; Y=(v-cy)*z/fx
P=np.stack([X,Y,z],-1).reshape(-1,3)@R.T+pos
ok=np.isfinite(z.reshape(-1))&(z.reshape(-1)>0.05); P=P[ok]
sel=P[(P[:,0]>-0.13)&(P[:,0]<-0.10)]
for ylo in np.arange(0.20,0.46,0.01):
    s=sel[(sel[:,1]>=ylo)&(sel[:,1]<ylo+0.01)]
    if len(s): print(f"y[{ylo:.2f}) n={len(s)} z[{s[:,2].min():.3f},{s[:,2].max():.3f}]")
EOF
timeout 60 python3 cav4.py

# openrua op 57
cd /workspace; timeout 60 python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw snaps/eih_r60_d.png && cp snaps/eih_r60_d.npy snaps/robot0_eye_in_hand_depth.npy && timeout 60 python3 cav4.py && timeout 60 python3 cav3.py | head -40; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih_r60.png

# openrua op 58
cd /workspace; timeout 90 python3 pxw.py birdview 350,285 360,285 365,285 370,285 375,285 380,285 385,285 390,285 395,285 400,285 375,275 375,295 375,300 2>&1 | tail -13

# openrua op 59
cd /workspace; timeout 60 python3 tools/perception/cam_snap.py sideview snaps/side2.png >/dev/null; timeout 90 python3 pxw.py sideview 160,340 180,340 200,340 220,340 240,340 260,340 280,340 300,340 320,340 340,340 360,340 370,340 240,300 240,320 240,360 240,380 240,400 240,410 2>&1 | tail -18

# openrua op 60
cd /workspace; timeout 90 python3 pxw.py agentview 478,200 478,250 478,300 478,350 478,380 485,300 490,300 495,300 500,300 505,300 510,300 490,360 490,390 500,380 470,300 465,300 2>&1 | tail -16

# openrua op 61
cd /workspace; timeout 60 python3 -c "
from rlib import *
r=Robot()
pos,quat=r.fk_world(); from scipy.spatial.transform import Rotation as Ro
R=Ro.from_quat(quat).as_matrix()
print('hand',np.round(pos,3),'z_h',np.round(R[:,2],2),'y_h',np.round(R[:,1],2),'x_h',np.round(R[:,0],2))
for l in ['panda_link4','panda_link5','panda_link6','panda_link7','panda_link8']:
    p,q=r.fk_world(link=l); print(l,np.round(p,3), 'rel hand', np.round(R.T@(p-pos),3))
"

# openrua op 62
cd /workspace; timeout 90 python3 pxw.py frontview 340,190 340,200 340,210 340,220 340,230 340,240 340,250 340,260 340,270 340,280 345,215 335,215 2>&1 | tail -12

# openrua op 63
cd /workspace; cat > cloud.py <<'EOF'
import numpy as np, sys
from rlib import *
from scipy.spatial.transform import Rotation as Ro
import rclpy
from sensor_msgs.msg import Image
def cloud(r, cam):
    got=[]
    sub=r.node.create_subscription(Image, f"/{cam}/depth/image_raw", lambda m: got.append(m), 1)
    while not got: rclpy.spin_once(r.node, timeout_sec=0.2)
    r.node.destroy_subscription(sub)
    m=got[0]; d=np.frombuffer(m.data, dtype=np.float32).reshape(m.height, m.width)
    fx = 312.77408948188935 if cam=="robot0_eye_in_hand" else 579.4112549695428
    pos,quat=r.tf("world", f"{cam}_optical_frame"); R=Ro.from_quat(quat).as_matrix()
    v,u=np.mgrid[0:m.height,0:m.width]
    X=(u-320)*d/fx; Y=(v-240)*d/fx
    P=np.stack([X,Y,d],-1).reshape(-1,3)@R.T+pos
    ok=np.isfinite(d.reshape(-1))&(d.reshape(-1)>0.05)
    return P[ok]
if __name__=="__main__":
    r=Robot(); P=cloud(r, sys.argv[1])
    lo=np.array([float(v) for v in sys.argv[2].split(",")]); hi=np.array([float(v) for v in sys.argv[3].split(",")])
    S=P[np.all((P>=lo)&(P<=hi),axis=1)]
    print("n",len(S))
    if len(S):
        print("min",np.round(S.min(0),3),"max",np.round(S.max(0),3))
        ax=int(sys.argv[4]) if len(sys.argv)>4 else 2
        for b in np.arange(lo[ax],hi[ax],0.01):
            s=S[(S[:,ax]>=b)&(S[:,ax]<b+0.01)]
            if len(s): print(f"{'xyz'[ax]}[{b:.2f}) n={len(s)} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}] z[{s[:,2].min():.3f},{s[:,2].max():.3f}]")
EOF
timeout 60 python3 cloud.py frontview -0.1,-0.06,0.95 0.15,0.03,1.25 1

# openrua op 64
cd /workspace; timeout 60 python3 cloud.py frontview -0.1,-0.05,1.02 0.15,0.0,1.25 2 | tail -25

# openrua op 65
cd /workspace; cat > geo.py <<'EOF'
"""Pose geometry checks for the lying mug held by the rim (rim at TCP-0.035 z_h, mug centre line = TCP - 0.044 y_h)."""
from cart import *
Rf = np.array([[-1,0,0],[0,0,1],[0,1,0]], float)
def Rrp(roll, pitch):
    return Rotation.from_euler('x', -pitch, degrees=True).as_matrix() @ Rotation.from_euler('y', roll, degrees=True).as_matrix() @ Rf
def report(tcp, R, tag=""):
    tcp=np.asarray(tcp); xh,yh,zh=R[:,0],R[:,1],R[:,2]
    rimc = tcp - 0.035*zh - 0.044*yh; nosec = rimc + 0.115*zh
    # ring extreme points: centre +- 0.0465 in plane perpendicular to zh -> vertical extent 0.0465*sqrt(1-zh_z^2)
    rz = 0.0465*np.sqrt(1-zh[2]**2); ry = 0.0465*np.sqrt(1-zh[1]**2)
    hand = tcp - 0.1034*zh; wrist = tcp - 0.21*zh; bodyfront = tcp - 0.045*zh
    fo = tcp + 0.05*yh  # outer finger (open) centre, tip at +0.01 zh
    print(f"{tag} tcp={np.round(tcp,3)} yh={np.round(yh,2)} zh={np.round(zh,2)}")
    print(f"   nose: front y={nosec[1]+ry:.3f} low z={nosec[2]-rz:.3f} top z={nosec[2]+rz:.3f} x[{nosec[0]-0.0465:.3f},{nosec[0]+0.0465:.3f}]")
    print(f"   rim : y={rimc[1]-ry:.3f} top z={rimc[2]+rz:.3f} low z={rimc[2]-rz:.3f}")
    print(f"   body front y={bodyfront[1]:.3f} z={bodyfront[2]:.3f}; hand={np.round(hand,3)} body ends {np.round(hand+0.1*yh,3)} {np.round(hand-0.1*yh,3)}")
    print(f"   wrist centre={np.round(wrist,3)} bottom z={wrist[2]-0.043:.3f}; outer finger open {np.round(fo,3)}")
if __name__=="__main__":
    import sys
    for a in sys.argv[1:]:
        x,y,z,roll,pitch=[float(v) for v in a.split(",")]
        report((x,y,z), Rrp(roll,pitch), f"roll{roll:.0f} pitch{pitch:.0f}")
EOF
python3 geo.py 0,0.16,1.10,60,0 0,0.11,1.10,60,0 0.044,0.13,1.07,90,40 0.044,0.20,1.04,90,30 0.044,0.26,1.015,90,15

# openrua op 66
cd /workspace; cat > s6.py <<'EOF'
"""Execute a sequence of TCP waypoints (x,y,z,roll,pitch) via Cartesian interpolation."""
import sys
from geo import *
r = Robot()
go = "--go" in sys.argv
specs = [a for a in sys.argv[1:] if not a.startswith("--")]
n = 6
pos, quat = r.fk_world(); R0 = Rotation.from_quat(quat).as_matrix(); tcp0 = pos + 0.1034*R0[:,2]
print("start tcp", np.round(tcp0,3), "q", np.round(r.joints(),2))
for a in specs:
    x,y,z,roll,pitch = [float(v) for v in a.split(",")]
    R1 = Rrp(roll,pitch); tcp1 = np.array([x,y,z])
    report(tcp1, R1, f"--> roll{roll:.0f} pitch{pitch:.0f}")
    qs = cart_path(r, tcp0, R0, tcp1, R1, n)
    if go:
        q0 = np.array(r.joints()); dq = np.abs(np.array(qs[-1]) - q0)
        secs = max(5.0, 6.0*dq[6] + 2.0, dq.max()*4)
        ok = run(qs, r, secs)
        pos, quat = r.fk_world(); Rn = Rotation.from_quat(quat).as_matrix()
        print("  ok", ok, "tcp", np.round(pos + 0.1034*Rn[:,2],3), "fingers", np.round(r.fingers(),4), "q", np.round(r.joints(),2))
        if not ok: sys.exit("move failed")
    tcp0, R0 = tcp1, R1
EOF
timeout 200 python3 s6.py 0,0.11,1.10,60,0 -0.006,0.13,1.07,90,40 -0.006,0.20,1.04,90,30 -0.006,0.26,1.015,90,15 2>&1 | grep -v "^   "

# openrua op 67
cd /workspace; timeout 400 python3 s6.py 0,0.11,1.10,60,0 -0.006,0.13,1.07,90,40 --go 2>&1 | grep -E "ok|-->|failed|code"

# openrua op 68
cd /workspace; timeout 60 python3 tools/perception/cam_snap.py frontview snaps/front_p40.png; timeout 60 python3 tools/perception/cam_snap.py agentview snaps/agent_p40.png

# openrua op 69
cd /workspace; timeout 400 python3 s6.py -0.006,0.20,1.04,90,30 -0.006,0.26,1.015,90,15 --go 2>&1 | grep -E "ok|-->|failed|code"

# openrua op 70
cd /workspace; timeout 60 python3 tools/perception/cam_snap.py agentview snaps/agent_ins.png; timeout 60 python3 tools/perception/cam_snap.py frontview snaps/front_ins.png

# openrua op 71
cd /workspace; timeout 60 python3 -c "
from rlib import *
r=Robot(); print('fingers after open', r.gripper(0.04)); print('q', np.round(r.joints(),3))"

# openrua op 72
cd /workspace; timeout 60 python3 tools/perception/cam_snap.py agentview snaps/agent_rel.png; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih_rel.png

# openrua op 73
cd /workspace; timeout 60 python3 cloud.py robot0_eye_in_hand -0.16,0.20,0.93 0.08,0.42,1.07 1 | tail -25

# openrua op 74
cd /workspace; timeout 60 python3 cloud.py robot0_eye_in_hand -0.13,0.23,0.95 0.06,0.42,1.06 0 | tail -22

# openrua op 75
cd /workspace; timeout 400 python3 s6.py -0.006,0.20,1.03,90,15 -0.006,0.12,1.10,90,15 --go 2>&1 | grep -E "ok|-->|failed|code"

# openrua op 76
cd /workspace; timeout 60 python3 tools/perception/cam_snap.py agentview snaps/agent_ret.png; timeout 60 python3 tools/perception/cam_snap.py birdview snaps/bird_ret.png

# openrua op 77
cd /workspace; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih_ret.png; timeout 60 python3 tools/perception/cam_snap.py frontview snaps/front_ret.png

# openrua op 78
cd /workspace; timeout 60 python3 cloud.py robot0_eye_in_hand -0.16,0.05,0.95 0.03,0.30,1.16 1 | tail -22; timeout 60 python3 cloud.py robot0_eye_in_hand -0.16,0.05,0.95 0.03,0.30,1.16 0 | tail -18

# openrua op 79
cd /workspace; timeout 60 python3 -c "
from rlib import *
r=Robot(); print('fingers after close', r.gripper(0.0))"; timeout 60 python3 tools/perception/cam_snap.py frontview snaps/front_regrasp.png

# openrua op 80
cd /workspace; timeout 60 python3 -c "
from rlib import *
r=Robot(); pos,q=r.fk_world(); from scipy.spatial.transform import Rotation as Ro
R=Ro.from_quat(q).as_matrix(); print('tcp',np.round(pos+0.1034*R[:,2],3),'yh',np.round(R[:,1],2),'zh',np.round(R[:,2],2))"; timeout 60 python3 cloud.py robot0_eye_in_hand -0.16,0.05,0.95 0.03,0.25,1.128 1 | tail -16

# openrua op 81
cd /workspace; timeout 60 python3 cloud.py robot0_eye_in_hand -0.08,0.05,0.95 -0.02,0.25,1.128 1 | tail -14

# openrua op 82
cd /workspace; timeout 500 python3 s6.py -0.006,0.13,1.07,90,40 -0.006,0.20,1.04,90,30 -0.006,0.26,1.015,90,15 --go 2>&1 | grep -E "ok|-->|failed|code"

# openrua op 83
cd /workspace; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih_ins2.png; timeout 60 python3 tools/perception/cam_snap.py agentview snaps/agent_ins2.png

# openrua op 84
cd /workspace; timeout 60 python3 -c "
from rlib import *
r=Robot(); print('fingers after open', r.gripper(0.04))" && timeout 300 python3 s6.py -0.006,0.26,1.0,90,25 -0.006,0.17,1.0,90,25 --go 2>&1 | grep -E "ok|-->|failed|code"

# openrua op 85
cd /workspace; timeout 60 python3 tools/perception/cam_snap.py agentview snaps/agent_ret2.png; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih_ret2.png

# openrua op 86
cd /workspace; timeout 60 python3 cloud.py robot0_eye_in_hand -0.13,0.15,0.95 0.06,0.42,1.06 1 | tail -24

# openrua op 87
cd /workspace; timeout 60 python3 cloud.py birdview -0.20,0.0,0.95 -0.12,0.26,1.15 1 | tail -28

# openrua op 88
cd /workspace; cat > door.py <<'EOF'
"""Close the microwave door: hand vertical (fingers down, closed), fingertips push the door's outer face along the hinge arc."""
import sys
from cart import *
r = Robot()
H = np.array([-0.19, 0.25]); rho = float(sys.argv[1]); press = float(sys.argv[2]); zt = float(sys.argv[3])
th0, th1 = float(sys.argv[4]), float(sys.argv[5])
def pose(deg):
    t = np.radians(deg); d = np.array([np.cos(t), -np.sin(t)]); n = np.array([-np.sin(t), -np.cos(t)])
    p = H + rho*d + (0.01 - press)*n
    tcp = np.array([p[0], p[1], zt])
    yh = np.array([d[0], d[1], 0]); zh = np.array([0,0,-1.0]); xh = np.cross(yh, zh)
    return tcp, R_from_axes(xh, yh, zh)
degs = np.arange(th0, th1 - 1e-6, -8.0 if th1 < th0 else 8.0); degs = np.append(degs, th1)
seed = r.joints(); qs = []
for dg in degs:
    tcp, R = pose(dg); hand = hand_pose_from_tcp(tcp, R)
    q = r.ik_world(hand, quat_from_R(R), seed=seed, attempts=3)
    if q is None: sys.exit(f"IK fail at {dg}")
    print(f"th={dg:5.1f} tcp={np.round(tcp,3)} jump={max(abs(a-b) for a,b in zip(q,seed)):.3f} q={np.round(q,2)}")
    seed = q; qs.append(q)
if "--go" in sys.argv:
    q0 = np.array(r.joints()); tot = sum(np.abs(np.diff(np.array([q0]+qs), axis=0)), 0)
    secs = max(6.0, 6.0*tot[6] + 2.0, tot.max()*3)
    print("secs", secs); ok = run(qs, r, secs); print("ok", ok, "q", np.round(r.joints(),2))
    pos, quat = r.fk_world(); Rn = Rotation.from_quat(quat).as_matrix(); print("tcp", np.round(pos + 0.1034*Rn[:,2],3))
EOF
timeout 120 python3 door.py 0.19 0.0 1.085 97 97

# openrua op 89
cd /workspace; cat > doorik.py <<'EOF'
import sys
from door import *  # noqa (door.py runs its own main; avoid) 
EOF
cat > door2.py <<'EOF'
import sys
from cart import *
r = Robot()
H = np.array([-0.19, 0.25]); lim = np.array(FJT["limits_rad"])
def pose(deg, rho, press, zt):
    t = np.radians(deg); d = np.array([np.cos(t), -np.sin(t)]); n = np.array([-np.sin(t), -np.cos(t)])
    p = H + rho*d + (0.01 - press)*n
    tcp = np.array([p[0], p[1], zt]); yh = np.array([d[0], d[1], 0]); zh = np.array([0,0,-1.0]); xh = np.cross(yh, zh)
    return tcp, R_from_axes(xh, yh, zh)
def margin(q): return min(min(qi - lo, hi - qi) for qi, (lo, hi) in zip(q, lim))
if __name__ == "__main__":
    rho, press, zt = 0.19, 0.0, 1.085
    rng = np.random.default_rng(3)
    tcp, R = pose(97, rho, press, zt); hand = hand_pose_from_tcp(tcp, R)
    best = None
    for i in range(30):
        seed = rng.uniform(lim[:,0], lim[:,1])
        q = r.ik_world(hand, quat_from_R(R), seed=seed, attempts=1, timeout=0.5)
        if q is not None:
            m = margin(q)
            if best is None or m > best[0]: best = (m, q)
    print("best start margin", round(best[0],2), np.round(best[1],2))
    # follow arc
    seed = best[1]
    for dg in list(np.arange(97, 0, -8)) + [0]:
        tcp, R = pose(dg, rho, press, zt); q = r.ik_world(hand_pose_from_tcp(tcp, R), quat_from_R(R), seed=seed, attempts=3)
        if q is None: print("fail", dg); break
        print(f"th={dg:5.1f} tcp={np.round(tcp,3)} jump={max(abs(a-b) for a,b in zip(q,seed)):.3f} margin={margin(q):.2f} q={np.round(q,2)}"); seed = q
EOF
timeout 200 python3 door2.py

# openrua op 90
cd /workspace; cat > door3.py <<'EOF'
import sys
from door2 import *
rho, press, zt = 0.19, 0.0, 1.085
rng = np.random.default_rng(5)
tcp, R = pose(97, rho, press, zt); hand = hand_pose_from_tcp(tcp, R)
sols = []
base = np.array([0.1, 0.3, 0.0, -2.5, 0.0, 2.8, 0.8])
for i in range(40):
    seed = np.clip(base + rng.normal(0, 0.6, 7), lim[:,0]+0.05, lim[:,1]-0.05)
    q = r.ik_world(hand, quat_from_R(R), seed=seed, attempts=1, timeout=0.5)
    if q is not None and q[1] > 0: sols.append((margin(q), np.round(q,2)))
sols.sort(key=lambda s: -s[0])
for s in sols[:6]: print(s)
if sols:
    seed = list(sols[0][1])
    for dg in list(np.arange(97, 0, -8)) + [0]:
        tcp, R = pose(dg, rho, press, zt); q = r.ik_world(hand_pose_from_tcp(tcp, R), quat_from_R(R), seed=seed, attempts=3)
        if q is None: print("fail", dg); break
        print(f"th={dg:5.1f} tcp={np.round(tcp,3)} jump={max(abs(a-b) for a,b in zip(q,seed)):.3f} margin={margin(q):.2f} q={np.round(q,2)}"); seed = q
EOF
timeout 200 python3 door3.py

# openrua op 91
cd /workspace; cat > door4.py <<'EOF'
import sys
from door2 import *
rho, press, zt = 0.19, 0.0, 1.085
cur = r.joints(); print("cur", np.round(cur,2))
for dg in [97, 81, 65, 49, 33, 17, 0]:
    tcp, R = pose(dg, rho, press, zt)
    best=None
    for k in range(6):
        seed = np.array(cur) + np.random.default_rng(k).normal(0, 0.3, 7)*(k>0)
        q = r.ik_world(hand_pose_from_tcp(tcp, R), quat_from_R(R), seed=list(seed), attempts=1, timeout=0.5)
        if q is not None:
            d = max(abs(a-b) for a,b in zip(q,cur))
            if best is None or d < best[0]: best=(d, margin(q), np.round(q,2))
    print(dg, np.round(tcp,3), best)
EOF
timeout 200 python3 door4.py

# openrua op 92
cd /workspace; cat > door5.py <<'EOF'
import sys
from door2 import *
rho, press, zt = 0.19, 0.0, 1.085
rng = np.random.default_rng(7)
dg0 = float(sys.argv[1]) if len(sys.argv)>1 else 97
tcp, R = pose(dg0, rho, press, zt); hand = hand_pose_from_tcp(tcp, R)
sols=[]
for base in ([0.1,0.3,0.0,-2.7,0.0,3.0,0.8],[0.1,0.1,0.0,-2.9,0.0,3.1,0.8],[0.0,0.6,0.0,-2.4,0.0,3.0,0.8],[-0.3,0.2,0.3,-2.8,0.0,3.0,0.0]):
    for i in range(15):
        seed = np.clip(np.array(base) + rng.normal(0, 0.3, 7)*(i>0), lim[:,0]+0.05, lim[:,1]-0.05)
        q = r.ik_world(hand, quat_from_R(R), seed=list(seed), attempts=1, timeout=0.5)
        if q is not None and abs(q[0])<1.0 and abs(q[2])<1.0: sols.append((round(margin(q),2), np.round(q,2)))
sols.sort(key=lambda s:-s[0])
for s in sols[:5]: print(s)
print("n", len(sols))
EOF
timeout 200 python3 door5.py 97

# openrua op 93
cd /workspace; cat > door_go.py <<'EOF'
import sys
from door2 import *
rho, press, zt = 0.19, 0.0, 1.085
def seg(tcp1, R1, n=6):
    global tcp0, R0
    qs = cart_path(r, tcp0, R0, tcp1, R1, n)
    q0 = np.array(r.joints()); tot = np.abs(np.diff(np.array([q0]+qs), axis=0)).sum(0)
    secs = max(5.0, 6.0*tot[6] + 2.0, tot.max()*3)
    ok = run(qs, r, secs); pos, quat = r.fk_world(); Rn = Rotation.from_quat(quat).as_matrix()
    print("  ok", ok, "tcp", np.round(pos + 0.1034*Rn[:,2],3), "q", np.round(r.joints(),2))
    if not ok: sys.exit("fail")
    tcp0, R0 = np.asarray(tcp1, float), R1
print("fingers", r.gripper(0.0))
pos, quat = r.fk_world(); R0 = Rotation.from_quat(quat).as_matrix(); tcp0 = pos + 0.1034*R0[:,2]
tcpd, Rd = pose(97, rho, press, zt)
stage = sys.argv[1]
if stage == "approach":
    seg(tcp0 + [0,0,0.30], R0, 4)
    seg([tcpd[0], tcpd[1], 1.30], Rd, 8)
    seg(tcpd, Rd, 4)
EOF
timeout 400 python3 door_go.py approach 2>&1 | grep -E "ok|fail|fingers|jump=[1-9]|jump=0\.[3-9]"

# openrua op 94
cd /workspace; timeout 60 python3 tools/perception/cam_snap.py birdview snaps/bird_door0.png; timeout 60 python3 tools/perception/cam_snap.py agentview snaps/agent_door0.png

# openrua op 95
cd /workspace; cat > door_arc.py <<'EOF'
import sys
from door2 import *
rho, press, zt = 0.19, float(sys.argv[3]), 1.085
th0, th1 = float(sys.argv[1]), float(sys.argv[2])
degs = list(np.arange(th0, th1, -6.0)) + [th1]
seed = r.joints(); qs = []
for dg in degs:
    tcp, R = pose(dg, rho, press, zt)
    q = r.ik_world(hand_pose_from_tcp(tcp, R), quat_from_R(R), seed=seed, attempts=3)
    if q is None: sys.exit(f"IK fail {dg}")
    j = max(abs(a-b) for a,b in zip(q, seed)); print(f"th={dg:5.1f} tcp={np.round(tcp,3)} jump={j:.3f} q={np.round(q,2)}")
    if j > 0.5: sys.exit("jump too large")
    seed = q; qs.append(q)
if "--go" in sys.argv:
    q0 = np.array(r.joints()); tot = np.abs(np.diff(np.array([q0]+qs), axis=0)).sum(0)
    secs = max(8.0, 6.0*tot[6] + 2.0, tot.max()*3); print("secs", secs)
    ok = run(qs, r, secs); pos, quat = r.fk_world(); Rn = Rotation.from_quat(quat).as_matrix()
    print("ok", ok, "tcp", np.round(pos + 0.1034*Rn[:,2],3), "q", np.round(r.joints(),2))
EOF
timeout 400 python3 door_arc.py 97 0 0.005 --go 2>&1 | grep -vE "^  s="

# openrua op 96
cd /workspace; timeout 60 python3 tools/perception/cam_snap.py birdview snaps/bird_door1.png; timeout 60 python3 tools/perception/cam_snap.py agentview snaps/agent_door1.png

# openrua op 97
timeout 90 python3 cloud.py birdview -0.25,0.15,1.09 0.10,0.30,1.13 y 2>&1 | tail -25; echo ---; timeout 90 python3 cloud.py birdview -0.25,0.10,0.92 0.10,0.24,1.06 y 2>&1 | tail -25

# openrua op 98
timeout 90 python3 cloud.py birdview -0.25,0.15,1.09 0.10,0.30,1.13 1 2>&1 | tail -20; echo ---; timeout 90 python3 cloud.py birdview -0.25,0.10,0.92 0.10,0.24,1.06 1 2>&1 | tail -20

# openrua op 99
ros2 topic list | grep color/image_raw; timeout 60 python3 pxw.py birdview 345,315 400,300 440,300 395,330 2>&1 | tail -5

# openrua op 100
timeout 60 python3 tools/perception/cam_snap.py frontview snaps/front_door1.png; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih_door1.png

# openrua op 101
cat > /workspace/lift.py <<'EOF'
import sys, numpy as np
from rlib import Robot
from cart import cart_path, current_tcp, run
r = Robot()
dz = float(sys.argv[1])
tcp0, R0 = current_tcp(r)
tcp1 = tcp0 + np.array([0, 0, dz])
qs = cart_path(r, tcp0, R0, tcp1, R0, 4)
if qs is None: raise SystemExit("ik fail")
q0 = np.array(r.joints()); dq = np.abs(np.array(qs[-1]) - q0)
secs = max(5, 6*dq[6]+2, dq.max()*4)
ok = run(qs, r, secs)
print("ok", ok, "tcp", np.round(current_tcp(r)[0], 3), "q", np.round(r.joints(), 2))
EOF
timeout 300 python3 -u lift.py 0.20 2>&1 | tail -8

# openrua op 102
grep -n "def current_tcp" -A6 /workspace/cart.py; grep -n "def run" -A3 /workspace/cart.py; grep -n "def fk_world" -A3 /workspace/rlib.py

# openrua op 103
python3 - <<'EOF'
import re
s = open('lift.py').read()
s = s.replace("tcp0, R0 = current_tcp(r)", "pos, quat = r.fk_world()\nfrom scipy.spatial.transform import Rotation\nR0 = Rotation.from_quat(quat).as_matrix()\ntcp0 = current_tcp(r, R0)")
s = s.replace("current_tcp(r)[0]", "current_tcp(r, R0)")
open('lift.py','w').write(s)
EOF
grep -n "quat" rlib.py | head -5

# openrua op 104
timeout 300 python3 -u lift.py 0.20 2>&1 | tail -8

# openrua op 105
timeout 60 python3 tools/perception/cam_snap.py frontview snaps/front_door2.png; timeout 60 python3 tools/perception/cam_snap.py birdview snaps/bird_door2.png; timeout 90 python3 cloud.py birdview -0.25,0.15,1.09 0.10,0.30,1.13 1 2>&1 | tail -12

# openrua op 106
timeout 90 python3 cloud.py frontview -0.22,0.10,0.93 0.10,0.40,1.10 0 2>&1 | tail -34
