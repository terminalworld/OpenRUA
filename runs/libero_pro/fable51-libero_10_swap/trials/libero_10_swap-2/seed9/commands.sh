#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la && find . -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/action/fjt_send.py; echo ----; cat /workspace/tools/action/ik_move.py; echo ----; cat /workspace/tools/action/gripper_cmd.py; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
timeout 30 ros2 topic list; echo ---; timeout 30 ros2 action list; echo ---; timeout 30 ros2 service list | grep -v -E "parameter|describe|get_type" ; echo ---; timeout 20 ros2 node list

# openrua op 3
timeout 20 ros2 topic echo /joint_states --once; echo ---; timeout 20 ros2 topic info /pipeline_state -v | head -30; timeout 10 ros2 topic echo /pipeline_state --once

# openrua op 4
for c in agentview frontview sideview birdview robot0_robotview robot0_eye_in_hand galleryview paperview; do timeout 60 python3 tools/perception/cam_snap.py $c || echo "FAIL $c"; done

# openrua op 5
timeout 60 python3 - <<'EOF'
import rclpy, numpy as np
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time
rclpy.init(); node = rclpy.create_node("tfq")
buf = Buffer(); TransformListener(buf, node)
import time
for _ in range(40): rclpy.spin_once(node, timeout_sec=0.1)
print(buf.all_frames_as_string())
for a,b in [("world","panda_link0"),("world","panda_hand"),("panda_link0","panda_hand"),("world","birdview_optical_frame"),("world","agentview_optical_frame")]:
    try:
        t = buf.lookup_transform(a,b,Time())
        tr=t.transform.translation; q=t.transform.rotation
        print(a,"->",b, round(tr.x,4),round(tr.y,4),round(tr.z,4), "q", round(q.x,4),round(q.y,4),round(q.z,4),round(q.w,4))
    except Exception as e: print(a,"->",b,"ERR",e)
EOF

# openrua op 6
cat > px_batch.py <<'EOF'
#!/usr/bin/env python3
"""px_batch.py <camera> u,v [u,v ...]  -> world xyz per pixel (one grab)."""
import sys, struct, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time
cam = sys.argv[1]; pts=[tuple(map(int,a.split(","))) for a in sys.argv[2:]]
rclpy.init(); node=rclpy.create_node("pxb"); buf=Buffer(); TransformListener(buf,node)
def grab(topic,T):
    got={}; s=node.create_subscription(T,topic,lambda m:got.setdefault("m",m),1)
    while "m" not in got: rclpy.spin_once(node,timeout_sec=0.2)
    node.destroy_subscription(s); return got["m"]
d=grab(f"/{cam}/depth/image_raw",Image); info=grab(f"/{cam}/color/camera_info",CameraInfo)
depth=np.frombuffer(d.data,dtype=np.float32).reshape(d.height,d.width)
np.save(f"{cam}_depth.npy",depth)
fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
frame=f"{cam}_optical_frame"
while not buf.can_transform("world",frame,Time()): rclpy.spin_once(node,timeout_sec=0.2)
t=buf.lookup_transform("world",frame,Time()); q=t.transform.rotation
x,y,z,w=q.x,q.y,q.z,q.w
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
T=np.eye(4);T[:3,:3]=R;T[:3,3]=[t.transform.translation.x,t.transform.translation.y,t.transform.translation.z]
for u,v in pts:
    Z=float(depth[v,u]); p=T@np.array([(u-cx)*Z/fx,(v-cy)*Z/fy,Z,1.0])
    print(f"({u},{v}) depth={Z:.4f} world= {p[0]:.4f} {p[1]:.4f} {p[2]:.4f}")
rclpy.shutdown()
EOF
timeout 60 python3 px_batch.py birdview 250,285 320,310 375,285 378,237 200,380 450,380 320,240 150,300 340,235

# openrua op 7
timeout 60 python3 - <<'EOF'
import numpy as np, cv2
d=np.load("birdview_depth.npy")
# camera at z=3.0 looking down; world z = 3.0 - depth (approx for nadir camera)
h=3.0-d
img=cv2.imread("birdview.png")
# Table region roughly rows 160..420, cols 155..485
mask=(h>0.905)&(h<1.3)
mask[:150,:]=False; mask[:, :150]=False; mask[:,490:]=False; mask[420:,:]=False
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask.astype(np.uint8))
fx=fy=None
import yaml
for i in range(1,n):
    x,y,w,hh,a=stats[i]
    if a<30: continue
    ys,xs=np.where(lab==i)
    print(f"comp {i}: bbox u[{x},{x+w}] v[{y},{y+hh}] area={a} centroid=({cent[i][0]:.1f},{cent[i][1]:.1f}) zmax={h[ys,xs].max():.3f} zmed={np.median(h[ys,xs]):.3f}")
# crop images for a closer look
cv2.imwrite("bird_crop.png", cv2.resize(img[200:340,200:420],None,fx=3,fy=3,interpolation=cv2.INTER_NEAREST))
s=cv2.imread("sideview.png"); cv2.imwrite("side_crop.png", cv2.resize(s[200:400,200:440],None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
EOF

# openrua op 8
timeout 60 python3 - <<'EOF'
import numpy as np, cv2
d=np.load("birdview_depth.npy"); h=3.0-d
sub=h[222:256,358:392]
np.set_printoptions(linewidth=250, precision=2, suppress=True)
print("knob heights (rows v=222..255, cols u=358..391), cm above table:")
print(np.round((sub-0.9)*100).astype(int))
EOF

# openrua op 9
timeout 60 python3 - <<'EOF'
import numpy as np, sys
d=np.load("birdview_depth.npy"); h=3.0-d
sub=h[224:256,360:392]
a=np.round((sub-0.9)*100).astype(int)
for r,row in enumerate(a): print(f"v={224+r:3d} "+" ".join(f"{x:2d}" for x in row))
print("cols u=360..391")
EOF

# openrua op 10
cat > cloud.py <<'EOF'
#!/usr/bin/env python3
"""cloud.py <camera> -> saves <camera>_cloud.npy (H,W,3) world xyz + <camera>_depth.npy"""
import sys, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time
cam=sys.argv[1]
rclpy.init(); node=rclpy.create_node("cloud"); buf=Buffer(); TransformListener(buf,node)
def grab(topic,T):
    got={}; s=node.create_subscription(T,topic,lambda m:got.setdefault("m",m),1)
    while "m" not in got: rclpy.spin_once(node,timeout_sec=0.2)
    node.destroy_subscription(s); return got["m"]
d=grab(f"/{cam}/depth/image_raw",Image); info=grab(f"/{cam}/color/camera_info",CameraInfo)
depth=np.frombuffer(d.data,dtype=np.float32).reshape(d.height,d.width)
fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
frame=f"{cam}_optical_frame"
while not buf.can_transform("world",frame,Time()): rclpy.spin_once(node,timeout_sec=0.2)
t=buf.lookup_transform("world",frame,Time()); q=t.transform.rotation
x,y,z,w=q.x,q.y,q.z,q.w
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
tt=np.array([t.transform.translation.x,t.transform.translation.y,t.transform.translation.z])
v,u=np.mgrid[0:d.height,0:d.width]
P=np.stack([(u-cx)*depth/fx,(v-cy)*depth/fy,depth],-1)
W=P@R.T+tt
np.save(f"{cam}_cloud.npy",W.astype(np.float32)); np.save(f"{cam}_depth.npy",depth)
print(cam, "cam pos", tt, "fx",fx, "size", d.width, d.height)
rclpy.shutdown()
EOF
for c in frontview agentview sideview birdview; do timeout 60 python3 cloud.py $c; done

# openrua op 11
timeout 60 python3 - <<'EOF'
import numpy as np
np.set_printoptions(precision=3, suppress=True, linewidth=200)
pts=[]
for c in ["frontview","agentview","sideview","birdview"]:
    W=np.load(f"{c}_cloud.npy").reshape(-1,3)
    pts.append(W)
W=np.concatenate(pts)
def region(name, xr, yr, zmin=0.903):
    m=(W[:,0]>xr[0])&(W[:,0]<xr[1])&(W[:,1]>yr[0])&(W[:,1]<yr[1])&(W[:,2]>zmin)&(W[:,2]<1.5)
    P=W[m]
    print(f"== {name}: n={len(P)} x[{P[:,0].min():.3f},{P[:,0].max():.3f}] y[{P[:,1].min():.3f},{P[:,1].max():.3f}] z[{P[:,2].min():.3f},{P[:,2].max():.3f}]")
    for z0 in np.arange(0.905, P[:,2].max(), 0.01):
        S=P[(P[:,2]>=z0)&(P[:,2]<z0+0.01)]
        if len(S)<3: continue
        print(f"  z={z0:.3f}-{z0+0.01:.3f} n={len(S):4d} x[{S[:,0].min():.3f},{S[:,0].max():.3f}] cx={S[:,0].mean():.3f}  y[{S[:,1].min():.3f},{S[:,1].max():.3f}] cy={S[:,1].mean():.3f}")
region("moka", (-0.15,0.05), (-0.35,-0.12))
region("stove", (-0.15,0.06), (0.09,0.31))
region("knob", (-0.30,-0.14), (0.14,0.27))
region("pan", (-0.06,0.16), (-0.12,0.13))
EOF

# openrua op 12
timeout 60 python3 - <<'EOF'
import cv2
f=cv2.imread("frontview.png"); cv2.imwrite("front_crop.png", cv2.resize(f[290:410,130:260],None,fx=5,fy=5,interpolation=cv2.INTER_CUBIC))
a=cv2.imread("agentview.png"); cv2.imwrite("agent_crop.png", cv2.resize(a[200:340,90:230],None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))
EOF

# openrua op 13
timeout 60 python3 - <<'EOF'
import numpy as np
W=np.load("birdview_cloud.npy")
for (u,v) in [(375,239),(376,238),(376,226),(376,251)]:
    print((u,v), W[v,u])
# nadir scale
print("m/px at table:", W[300,301]-W[300,300])
# moka pot silhouette from birdview at rim height
h=W[:,:,2]; m=(h>1.02)&(h<1.05); m[:, :200]=False; m[:,300:]=False
ys,xs=np.where(m); P=W[ys,xs]
print("rim x[%.3f,%.3f] y[%.3f,%.3f]"%(P[:,0].min(),P[:,0].max(),P[:,1].min(),P[:,1].max()))
# exclude handle: y>-0.29
Q=P[P[:,1]>-0.29]; print("rim excl handle center", Q[:,0].mean(), Q[:,1].mean(), "x mid", (Q[:,0].min()+Q[:,0].max())/2, "y mid", (Q[:,1].min()+Q[:,1].max())/2)
EOF

# openrua op 14
mkdir -p "$(dirname /workspace/rob.py)"
cat > /workspace/rob.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Reusable robot helpers: joint state, IK, FK, trajectory, gripper, TF."""
import time
import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from rclpy.action import ActionClient
from rclpy.time import Time
from scipy.spatial.transform import Rotation as Rot
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]
TCP_OFF = float(M["hand"]["tcp_offset_m"])
BASE_IN_WORLD = None


class Robot:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("rob_helper")
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self._js.__setitem__("m", m), 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik_cli = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.tfbuf = Buffer(); TransformListener(self.tfbuf, self.node)
        self.fjt.wait_for_server(10); self.grip.wait_for_server(10)
        self.ik_cli.wait_for_service(10); self.fk_cli.wait_for_service(10)
        self.spin(0.5)

    def spin(self, sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    # ---------- sensing ----------
    def js(self):
        self._js.pop("m", None)
        while "m" not in self._js:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        m = self._js["m"]
        return dict(zip(m.name, m.position))

    def arm_q(self):
        j = self.js(); return [j[n] for n in ARM]

    def fingers(self):
        j = self.js()
        return j.get("panda_finger_joint1"), j.get("panda_finger_joint2")

    def tf(self, a, b, tries=50):
        for _ in range(tries):
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.tfbuf.can_transform(a, b, Time()):
                t = self.tfbuf.lookup_transform(a, b, Time())
                tr, q = t.transform.translation, t.transform.rotation
                return np.array([tr.x, tr.y, tr.z]), np.array([q.x, q.y, q.z, q.w])
        raise RuntimeError(f"no tf {a}->{b}")

    def base_in_world(self):
        global BASE_IN_WORLD
        if BASE_IN_WORLD is None:
            BASE_IN_WORLD = self.tf("world", M["frames"]["base"])[0]
        return BASE_IN_WORLD

    def hand_pose_world(self, q=None):
        """FK of the hand frame, in the world frame (base offset added)."""
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = [M["frames"]["hand"]]
        req.robot_state.joint_state.name = ARM
        req.robot_state.joint_state.position = q if q is not None else self.arm_q()
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            raise RuntimeError(f"FK failed {r and r.error_code.val}")
        p = r.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + self.base_in_world()
        quat = np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        return pos, quat

    def tcp_pose_world(self, q=None):
        pos, quat = self.hand_pose_world(q)
        R = Rot.from_quat(quat).as_matrix()
        return pos + TCP_OFF * R[:, 2], quat

    # ---------- IK ----------
    def ik(self, pos_world, quat, at_tcp=True, seed=None, timeout=60):
        """pos_world: target in world; quat xyzw of hand. Returns arm joints or None."""
        pos = np.array(pos_world, float)
        R = Rot.from_quat(quat).as_matrix()
        if at_tcp:
            pos = pos - TCP_OFF * R[:, 2]
        pos = pos - self.base_in_world()  # planner frame = arm base
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pos)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.robot_state.joint_state.name = ARM
        req.ik_request.robot_state.joint_state.position = list(seed) if seed is not None else self.arm_q()
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = 2
        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        r = fut.result()
        if r is None:
            print("IK: no answer"); return None
        if r.error_code.val != 1:
            print(f"IK failed code={r.error_code.val}"); return None
        sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
        return [sol[j] for j in ARM]

    # ---------- acting ----------
    def move_joints(self, q_list, secs, verbose=True):
        """q_list: list of joint vectors (waypoints); secs: total or per-point list."""
        if not isinstance(q_list[0], (list, tuple, np.ndarray)):
            q_list = [q_list]
        if not isinstance(secs, (list, tuple)):
            n = len(q_list); secs = [secs * (i + 1) / n for i in range(n)]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM
        for q, t in zip(q_list, secs):
            pt = JointTrajectoryPoint(positions=[float(x) for x in q])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        gh = send.result()
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, res)
        code = res.result().result.error_code
        err = np.abs(np.array(self.arm_q()) - np.array(q_list[-1])).max()
        if verbose:
            print(f"traj done code={code} max_joint_err={err:.4f}")
        return code, err

    def move_tcp(self, pos_world, quat, secs=3.0, seed=None):
        q = self.ik(pos_world, quat, at_tcp=True, seed=seed)
        if q is None:
            return None
        code, err = self.move_joints(q, secs)
        p, _ = self.tcp_pose_world()
        print(f"  tcp now {np.round(p,4)} target {np.round(pos_world,4)}")
        return q

    def gripper(self, width):
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = float(GRIP.get("max_effort", 30.0))
        fut = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=120)
        r = res.result().result
        f = self.fingers()
        print(f"gripper -> {width}: reached={r.reached_goal} stalled={r.stalled} fingers={f}")
        return f

    def shutdown(self):
        self.node.destroy_node(); rclpy.shutdown()


def quat_from_axes(hand_x, hand_y, hand_z):
    R = np.column_stack([hand_x, hand_y, hand_z])
    return Rot.from_matrix(R).as_quat()  # xyzw


Q_DOWN = np.array([1.0, 0.0, 0.0, 0.0])  # hand z down, fingers along world y
OPENRUA_EOF

# openrua op 15
timeout 120 python3 -u - <<'EOF'
from rob import *
r=Robot()
print("arm q", np.round(r.arm_q(),4))
print("fingers", r.fingers())
print("hand FK world", r.hand_pose_world())
print("tcp FK world", r.tcp_pose_world())
print("TF world->hand", r.tf("world","panda_hand"))
q=r.ik([-0.207,0.199,1.06], Q_DOWN)
print("IK pregrasp knob:", None if q is None else np.round(q,4))
if q is not None: print("  FK check", np.round(r.tcp_pose_world(q)[0],4))
r.shutdown()
EOF

# openrua op 16
timeout 120 python3 -u - <<'EOF'
from rob import *
import rob
r=Robot()
hp,hq=r.hand_pose_world()
print("hand raw FK", np.round(hp - r.base_in_world(),4))
# test IK at current hand pose, expressed in world and in base
for label, off in [("world", np.zeros(3)), ("base", r.base_in_world())]:
    rob.BASE_IN_WORLD = off
    q=r.ik(hp - r.base_in_world() , hq, at_tcp=False)   # hp-raw... 
    print(label, "IK:", None if q is None else np.round(q,4))
r.shutdown()
EOF

# openrua op 17
timeout 300 python3 -u - <<'EOF'
import rclpy, numpy as np
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
rclpy.init(); node=rclpy.create_node("iktest")
cli=node.create_client(GetPositionIK,"/compute_ik"); cli.wait_for_service(10)
seedq=[0.0,-0.161037389,0.0,-2.44459747,0.0,2.2267522,0.7853981633974483]
ARM=[f"panda_joint{i}" for i in range(1,8)]
hq=[9.99596635e-01,0,-2.84001213e-02,0]
def test(pos, quat, link="", tmo=None, avoid=None):
    req=GetPositionIK.Request(); req.ik_request.group_name="panda_arm"
    req.ik_request.pose_stamped.header.frame_id=""
    if link: req.ik_request.ik_link_name=link
    p=req.ik_request.pose_stamped.pose
    p.position.x,p.position.y,p.position.z=map(float,pos)
    p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=map(float,quat)
    req.ik_request.robot_state.joint_state.name=ARM
    req.ik_request.robot_state.joint_state.position=seedq
    if tmo is not None: req.ik_request.timeout.sec=tmo
    if avoid is not None: req.ik_request.avoid_collisions=avoid
    fut=cli.call_async(req); rclpy.spin_until_future_complete(node,fut,timeout_sec=60)
    r=fut.result()
    if r is None: print(pos,link,tmo,avoid,"-> no answer"); return
    sol=dict(zip(r.solution.joint_state.name,r.solution.joint_state.position))
    print(pos,link,tmo,avoid,"-> code",r.error_code.val, [round(sol.get(j,9),3) for j in ARM] if r.error_code.val==1 else "")
test([-0.203,0,1.2696],hq)
test([0.457,0,0.3576],hq)
test([-0.203,0,1.2696],hq,link="panda_hand")
test([0.457,0,0.3576],hq,link="panda_hand")
test([0.457,0,0.3576],hq,link="panda_link8")
test([-0.203,0,1.2696],hq,link="panda_link8")
EOF

# openrua op 18
python3 - <<'EOF'
s=open("rob.py").read()
s=s.replace("""        pos = np.array(pos_world, float)
        R = Rot.from_quat(quat).as_matrix()
        if at_tcp:
            pos = pos - TCP_OFF * R[:, 2]
        pos = pos - self.base_in_world()  # planner frame = arm base
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
""","""        pos = np.array(pos_world, float)
        R = Rot.from_quat(quat).as_matrix()
        if at_tcp:
            pos = pos - TCP_OFF * R[:, 2]
        # machine fact (verified): this planner's model frame IS world
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.ik_link_name = M["frames"]["hand"]
""")
s=s.replace("""        pos = np.array([p.position.x, p.position.y, p.position.z]) + self.base_in_world()
""","""        pos = np.array([p.position.x, p.position.y, p.position.z])  # already world
""")
s=s.replace("""        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = 2
""","")
open("rob.py","w").write(s)
EOF
timeout 120 python3 -u - <<'EOF'
from rob import *
r=Robot()
print("tcp FK world", np.round(r.tcp_pose_world()[0],4))
q=r.ik([-0.207,0.199,1.06], Q_DOWN)
print("IK pregrasp knob:", None if q is None else np.round(q,4))
if q is not None: print("  FK check tcp", np.round(r.tcp_pose_world(q)[0],4), "hand", np.round(r.hand_pose_world(q)[0],4), np.round(r.hand_pose_world(q)[1],4))
r.shutdown()
EOF

# openrua op 19
timeout 600 python3 -u - <<'EOF' 2>&1 | tee knob1.log
from rob import *
r=Robot()
r.gripper(0.04)
KNOB=np.array([-0.207,0.199])
q1=r.move_tcp([KNOB[0],KNOB[1],1.06], Q_DOWN, 4.0)
q2=r.move_tcp([KNOB[0],KNOB[1],0.945], Q_DOWN, 3.0, seed=q1)
print("arm q", np.round(r.arm_q(),4), "fingers", r.fingers())
r.shutdown()
EOF
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih1.png; timeout 60 python3 tools/perception/cam_snap.py sideview side1.png

# openrua op 20
timeout 600 python3 -u - <<'EOF' 2>&1 | tee knob2.log
from rob import *
r=Robot()
f=r.gripper(0.01)
q=r.arm_q(); print("before", np.round(q,4))
q2=list(q); q2[6]=q[6]-np.pi/2
r.move_joints(q2, 3.0)
print("after", np.round(r.arm_q(),4), "fingers", r.fingers())
print("tcp", np.round(r.tcp_pose_world()[0],4))
r.shutdown()
EOF
timeout 60 python3 tools/perception/cam_snap.py birdview bird2.png; timeout 60 python3 tools/perception/cam_snap.py agentview agent2.png

# openrua op 21
timeout 60 python3 - <<'EOF'
import cv2
for n in ["bird2","birdview"]:
    img=cv2.imread(f"{n}.png"); cv2.imwrite(f"{n}_knob.png", cv2.resize(img[200:290,320:430],None,fx=5,fy=5,interpolation=cv2.INTER_CUBIC))
EOF

# openrua op 22
timeout 600 python3 -u - <<'EOF' 2>&1 | tee knob3.log
from rob import *
r=Robot()
r.gripper(0.04)
p,qh=r.tcp_pose_world()
q=r.move_tcp([p[0],p[1],1.10], Q_DOWN, 3.0)
print("arm q", np.round(r.arm_q(),4))
r.shutdown()
EOF
timeout 60 python3 tools/perception/cam_snap.py birdview bird3.png; timeout 60 python3 tools/perception/cam_snap.py sideview side3.png
timeout 60 python3 - <<'EOF'
import cv2
img=cv2.imread("bird3.png"); cv2.imwrite("bird3_knob.png", cv2.resize(img[200:290,320:430],None,fx=5,fy=5,interpolation=cv2.INTER_CUBIC))
EOF

# openrua op 23
timeout 300 python3 -u - <<'EOF' 2>&1 | tee moka_ik.log
from rob import *
r=Robot()
C=np.array([-0.047,-0.237]); ZG=0.972
a=np.array([1,-1,0])/np.sqrt(2); c=np.array([1,1,0])/np.sqrt(2)
for label,hy in [("A", c),("B",-c)]:
    hx=np.cross(hy,a); q=quat_from_axes(hx,hy,a)
    print(label,"quat",np.round(q,4),"hand_x",hx)
    seed=r.arm_q()
    for d in [0.12,0.06,0.0]:
        tcp=np.array([C[0],C[1],ZG])-d*a
        s=r.ik(tcp,q,seed=seed)
        print(f"  d={d} tcp={np.round(tcp,3)} ->", None if s is None else np.round(s,3))
        if s is not None: seed=s
    # lifted above stove
    s=r.ik([-0.05,0.196,1.10],q,seed=seed); print("  above stove:", None if s is None else np.round(s,3))
    s=r.ik([-0.05,0.196,1.012],q,seed=seed); print("  place stove:", None if s is None else np.round(s,3))
# burner center px->world
W=np.load("birdview_cloud.npy"); print("burner center world", W[282,375])
r.shutdown()
EOF

# openrua op 24
timeout 300 python3 -u - <<'EOF'
from rob import *
from moveit_msgs.srv import GetPositionFK
r=Robot()
links=["panda_link3","panda_link4","panda_link5","panda_link6","panda_link7","panda_hand","panda_leftfinger","panda_rightfinger"]
def fk_all(q):
    req=GetPositionFK.Request(); req.fk_link_names=links
    req.robot_state.joint_state.name=ARM; req.robot_state.joint_state.position=q
    fut=r.fk_cli.call_async(req); rclpy.spin_until_future_complete(r.node,fut,timeout_sec=30)
    res=fut.result()
    for n,ps in zip(res.fk_link_names,res.pose_stamped):
        p=ps.pose.position; print(f"   {n:18s} {p.x:7.3f} {p.y:7.3f} {p.z:7.3f}")
for label,q in [("pre d=0.12",[0.025,0.69,-0.037,-2.374,-2.358,1.643,0.747]),("grasp d=0",[-0.085,0.803,-0.119,-2.067,-2.5,1.843,0.692]),("above stove",[0.406,0.642,0.251,-1.948,-1.8,1.542,0.203]),("place",[0.397,0.857,0.264,-1.844,-1.85,1.491,0.295])]:
    print(label); fk_all(q)
r.shutdown()
EOF

# openrua op 25
timeout 900 python3 -u - <<'EOF' 2>&1 | tee moka1.log
from rob import *
r=Robot()
C=np.array([-0.047,-0.237]); ZG=0.972
a=np.array([1,-1,0])/np.sqrt(2); c=np.array([1,1,0])/np.sqrt(2)
QD=quat_from_axes(np.cross(c,a),c,a)
np.save("QD.npy",QD)
r.gripper(0.04)
# waypoint: horizontal hand, high, behind the pot
q=r.move_tcp(np.r_[C-0.12*a,1.15], QD, 4.0)
q=r.move_tcp(np.r_[C-0.12*a,ZG], QD, 3.0, seed=q)
q=r.move_tcp(np.r_[C-0.06*a,ZG], QD, 2.0, seed=q)
print("arm q", np.round(r.arm_q(),4), "fingers", r.fingers())
r.shutdown()
EOF
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih2.png; timeout 60 python3 tools/perception/cam_snap.py frontview front2.png; timeout 60 python3 tools/perception/cam_snap.py birdview bird4.png

# openrua op 26
timeout 900 python3 -u - <<'EOF' 2>&1 | tee moka1.log
from rob import *
r=Robot()
C=np.array([-0.047,-0.237,0.972])
a=np.array([1,-1,0])/np.sqrt(2); c=np.array([1,1,0])/np.sqrt(2)
QD=quat_from_axes(np.cross(c,a),c,a)
np.save("QD.npy",QD)
q=r.move_tcp(C-0.12*a+[0,0,0.178], QD, 4.0)
q=r.move_tcp(C-0.12*a, QD, 3.0, seed=q)
q=r.move_tcp(C-0.06*a, QD, 2.0, seed=q)
print("arm q", np.round(r.arm_q(),4), "fingers", r.fingers())
r.shutdown()
EOF
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih2.png; timeout 60 python3 tools/perception/cam_snap.py frontview front2.png; timeout 60 python3 tools/perception/cam_snap.py birdview bird4.png

# openrua op 27
timeout 900 python3 -u - <<'EOF' 2>&1 | tee moka1b.log
from rob import *
r=Robot()
C=np.array([-0.047,-0.237,0.972]); a=np.array([1,-1,0])/np.sqrt(2); QD=np.load("QD.npy")
tgt=[-0.03,0.738,-0.086,-2.233,-2.439,1.74,0.719]
for i in range(3):
    code,err=r.move_joints(tgt, 4.0)
    print("  q", np.round(r.arm_q(),3))
    if err<0.02: break
print("tcp", np.round(r.tcp_pose_world()[0],4))
r.shutdown()
EOF

# openrua op 28
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih2.png; timeout 60 python3 tools/perception/cam_snap.py sideview side4.png

# openrua op 29
timeout 900 python3 -u - <<'EOF' 2>&1 | tee moka2.log
from rob import *
r=Robot()
C=np.array([-0.047,-0.237,0.972]); a=np.array([1,-1,0])/np.sqrt(2); QD=np.load("QD.npy")
q=r.ik(C, QD, seed=r.arm_q()); print("grasp q", np.round(q,3))
for i in range(3):
    code,err=r.move_joints(q, 3.0)
    if err<0.02: break
print("tcp", np.round(r.tcp_pose_world()[0],4))
f=r.gripper(0.02)
print("fingers after close", f, "gap", f[0]-f[1])
r.shutdown()
EOF
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih3.png

# openrua op 30
timeout 600 python3 -u - <<'EOF' 2>&1 | tee moka2b.log
from rob import *
r=Robot()
f=r.gripper(0.0)
r.spin(0.5); f=r.fingers()
print("fingers after close", f, "gap", f[0]-f[1])
r.shutdown()
EOF
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih3.png

# openrua op 31
timeout 60 python3 tools/perception/cam_snap.py frontview front3.png; timeout 60 python3 tools/perception/cam_snap.py sideview side5.png; timeout 60 python3 tools/perception/cam_snap.py agentview agent3.png
timeout 60 python3 - <<'EOF'
import cv2
f=cv2.imread("front3.png"); cv2.imwrite("front3_crop.png", cv2.resize(f[270:420,100:330],None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
s=cv2.imread("side5.png"); cv2.imwrite("side5_crop.png", cv2.resize(s[190:330,240:440],None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
EOF

# openrua op 32
timeout 300 python3 -u - <<'EOF'
from rob import *
from moveit_msgs.srv import GetPositionFK
r=Robot()
links=["panda_link7","panda_hand","panda_leftfinger","panda_rightfinger"]
req=GetPositionFK.Request(); req.fk_link_names=links
js=r.js()
req.robot_state.joint_state.name=ARM; req.robot_state.joint_state.position=[js[j] for j in ARM]
fut=r.fk_cli.call_async(req); rclpy.spin_until_future_complete(r.node,fut,timeout_sec=30)
for n,ps in zip(fut.result().fk_link_names,fut.result().pose_stamped):
    p=ps.pose.position; print(f"{n:18s} {p.x:7.4f} {p.y:7.4f} {p.z:7.4f}")
# TF-based, includes finger joints
for f in ["panda_leftfinger","panda_rightfinger","panda_hand"]:
    print("TF", f, np.round(r.tf("world",f)[0],4))
# point cloud from frontview: what's at the finger heights near the pot
import subprocess; subprocess.run(["python3","cloud.py","frontview"]); subprocess.run(["python3","cloud.py","sideview"])
W=np.concatenate([np.load("frontview_cloud.npy").reshape(-1,3),np.load("sideview_cloud.npy").reshape(-1,3)])
m=(W[:,0]>-0.15)&(W[:,0]<0.05)&(W[:,1]>-0.35)&(W[:,1]<-0.12)&(W[:,2]>0.903)&(W[:,2]<1.1)
P=W[m]
for z0 in np.arange(0.905,1.07,0.01):
    S=P[(P[:,2]>=z0)&(P[:,2]<z0+0.01)]
    if len(S)<3: continue
    print(f"  z={z0:.3f} n={len(S):4d} x[{S[:,0].min():.3f},{S[:,0].max():.3f}] y[{S[:,1].min():.3f},{S[:,1].max():.3f}]")
r.shutdown()
EOF

# openrua op 33
timeout 600 python3 -u - <<'EOF' 2>&1 | tee moka2c.log
from rob import *
r=Robot()
m=None
r._js.pop("m",None)
while "m" not in r._js: rclpy.spin_once(r.node,timeout_sec=0.2)
m=r._js["m"]; print("efforts", dict(zip(m.name, np.round(m.effort,2))))
print("fingers before", r.fingers())
r.gripper(0.0)
for i in range(5):
    r.spin(0.3); print("fingers", r.fingers())
r.shutdown()
EOF

# openrua op 34
timeout 120 python3 - <<'EOF'
import numpy as np
W=np.concatenate([np.load("frontview_cloud.npy").reshape(-1,3),np.load("sideview_cloud.npy").reshape(-1,3)])
# points along the +c side of the pot near the expected finger
c=np.array([1,1,0])/np.sqrt(2); C=np.array([-0.047,-0.237,0])
d=(W-C)@c            # coordinate along closing axis
t=(W-C)@np.array([1,-1,0])/np.sqrt(2)  # along approach axis
m=(np.abs(t)<0.03)&(d>0.02)&(d<0.07)&(W[:,2]>0.903)&(W[:,2]<1.1)
P=W[m]; D=d[m]
for z0 in np.arange(0.905,1.07,0.01):
    s=(P[:,2]>=z0)&(P[:,2]<z0+0.01)
    if s.sum()<3: continue
    print(f"z={z0:.3f} n={s.sum():4d} d[{D[s].min():.3f},{D[s].max():.3f}]  t[{((P[s]-C)@np.array([1,-1,0])/np.sqrt(2)).min():.3f},{((P[s]-C)@np.array([1,-1,0])/np.sqrt(2)).max():.3f}]")
EOF

# openrua op 35
timeout 60 python3 - <<'EOF'
import cv2
a=cv2.imread("agent3.png"); cv2.imwrite("agent3_crop.png", cv2.resize(a[180:340,60:260],None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
EOF

# openrua op 36
timeout 600 python3 -u - <<'EOF' 2>&1 | tee moka3.log
from rob import *
r=Robot()
QD=np.load("QD.npy")
p,_=r.tcp_pose_world()
q=r.move_tcp(p+[0,0,0.03], QD, 2.0)
print("fingers", r.fingers())
r.shutdown()
EOF
timeout 60 python3 tools/perception/cam_snap.py agentview agent4.png
timeout 60 python3 - <<'EOF'
import cv2
a=cv2.imread("agent4.png"); cv2.imwrite("agent4_crop.png", cv2.resize(a[180:340,60:260],None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
EOF

# openrua op 37
timeout 600 python3 -u - <<'EOF' 2>&1 | tee moka3.log
from rob import *
r=Robot()
QD=np.load("QD.npy")
p,_=r.tcp_pose_world()
q=r.move_tcp(p+[0,0,0.03], QD, 2.0)
print("fingers", r.fingers())
r.shutdown()
EOF
timeout 60 python3 tools/perception/cam_snap.py agentview agent4.png
timeout 60 python3 - <<'EOF'
import cv2
a=cv2.imread("agent4.png"); cv2.imwrite("agent4_crop.png", cv2.resize(a[180:340,60:260],None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
EOF

# openrua op 38
timeout 120 python3 cloud.py birdview >/dev/null && timeout 60 python3 - <<'EOF'
import numpy as np
W=np.load("birdview_cloud.npy")
m=(W[:,:,0]>-0.15)&(W[:,:,0]<0.05)&(W[:,:,1]>-0.35)&(W[:,:,1]<-0.20)&(W[:,:,2]>1.03)
P=W[m]; print("pot top points:", len(P), "zmax", P[:,2].max(), "top xy at zmax", P[P[:,2].argmax()])
lid=P[P[:,2]>P[:,2].max()-0.012]; print("lid knob center", lid.mean(0))
EOF

# openrua op 39
timeout 1200 python3 -u - <<'EOF' 2>&1 | tee moka4.log
from rob import *
r=Robot()
QD=np.load("QD.npy"); a=np.array([1,-1,0])/np.sqrt(2)
BURNER=np.array([-0.050,0.196])
def go(tcp, secs, seed):
    q=r.ik(tcp, QD, seed=seed)
    if q is None: raise SystemExit("IK fail at %s"%tcp)
    for i in range(3):
        code,err=r.move_joints(q, secs)
        if err<0.02: break
    p,_=r.tcp_pose_world(); print("  tcp", np.round(p,4), "fingers", np.round(r.fingers(),4))
    return q
q=r.arm_q()
q=go([-0.048,-0.235,1.18], 3.0, q)          # lift
q=go([-0.05,-0.02,1.18], 4.0, q)            # mid
q=go([BURNER[0],BURNER[1],1.18], 4.0, q)    # above burner
q=go([BURNER[0],BURNER[1],1.06], 3.0, q)    # partway down
r.shutdown()
EOF
timeout 60 python3 tools/perception/cam_snap.py sideview side6.png; timeout 60 python3 tools/perception/cam_snap.py birdview bird6.png

# openrua op 40
timeout 60 python3 tools/perception/cam_snap.py frontview front6.png; timeout 60 python3 tools/perception/cam_snap.py agentview agent6.png; timeout 60 python3 - <<'EOF'
import cv2
a=cv2.imread("agent6.png"); cv2.imwrite("agent6_crop.png", cv2.resize(a[150:340,300:520],None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
b=cv2.imread("bird6.png"); cv2.imwrite("bird6_crop.png", cv2.resize(b[180:330,280:460],None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
EOF

# openrua op 41
timeout 120 python3 cloud.py frontview >/dev/null; timeout 120 python3 cloud.py agentview >/dev/null; timeout 60 python3 - <<'EOF'
import numpy as np
W=np.concatenate([np.load("frontview_cloud.npy").reshape(-1,3),np.load("agentview_cloud.npy").reshape(-1,3)])
# pot region above the burner: within 5cm of burner center, z above stove plate
m=(np.abs(W[:,0]+0.05)<0.06)&(np.abs(W[:,1]-0.196)<0.06)&(W[:,2]>0.94)&(W[:,2]<1.2)
P=W[m]
for z0 in np.arange(0.93,1.12,0.01):
    s=(P[:,2]>=z0)&(P[:,2]<z0+0.01)
    if s.sum()<3: continue
    S=P[s]; print(f"z={z0:.3f} n={s.sum():4d} x[{S[:,0].min():.3f},{S[:,0].max():.3f}] y[{S[:,1].min():.3f},{S[:,1].max():.3f}]")
EOF

# openrua op 42
timeout 1200 python3 -u - <<'EOF' 2>&1 | tee moka5.log
from rob import *
r=Robot()
QD=np.load("QD.npy"); a=np.array([1,-1,0])/np.sqrt(2)
def go(tcp, secs, seed):
    q=r.ik(tcp, QD, seed=seed)
    if q is None: raise SystemExit("IK fail at %s"%tcp)
    for i in range(3):
        code,err=r.move_joints(q, secs)
        if err<0.02: break
    p,_=r.tcp_pose_world(); print("  tcp", np.round(p,4), "fingers", np.round(r.fingers(),4))
    return q
q=r.arm_q()
p,_=r.tcp_pose_world()
tgt=np.array([p[0]-0.02, p[1]+0.019, p[2]])
q=go(tgt, 2.0, q)
q=go([tgt[0],tgt[1],1.02], 2.5, q)
q=go([tgt[0],tgt[1],1.003], 1.5, q)
r.gripper(0.04)
r.spin(0.5); print("fingers", r.fingers())
q=go(np.array([tgt[0],tgt[1],1.003])-0.09*a, 2.5, q)   # retreat along -approach
q=go(np.array([tgt[0],tgt[1],1.15])-0.12*a, 2.5, q)     # lift
r.shutdown()
EOF
timeout 60 python3 tools/perception/cam_snap.py agentview agent7.png; timeout 60 python3 tools/perception/cam_snap.py sideview side7.png; timeout 60 python3 tools/perception/cam_snap.py birdview bird7.png

# openrua op 43
timeout 120 python3 cloud.py birdview >/dev/null; timeout 60 python3 - <<'EOF'
import numpy as np, cv2
W=np.load("birdview_cloud.npy"); img=cv2.imread("bird7.png")
m=(np.abs(W[:,:,0]+0.05)<0.08)&(np.abs(W[:,:,1]-0.196)<0.08)&(W[:,:,2]>0.95)
P=W[m]; print("pot points on stove:", len(P), "zmax %.3f"%P[:,2].max())
lid=P[P[:,2]>P[:,2].max()-0.012]; print("lid knob center xy", np.round(lid[:,:2].mean(0),4), "(burner center -0.050 0.196)")
rim=P[(P[:,2]>1.05)&(P[:,2]<1.08)]; print("rim silhouette x[%.3f,%.3f] y[%.3f,%.3f] -> center (%.3f, %.3f)"%(rim[:,0].min(),rim[:,0].max(),rim[:,1].min(),rim[:,1].max(),(rim[:,0].min()+rim[:,0].max())/2,(rim[:,1].min()+rim[:,1].max())/2))
# burner red check: pixels on the stove plate that are red
ys,xs=np.where((np.abs(W[:,:,0]+0.05)<0.09)&(np.abs(W[:,:,1]-0.196)<0.09)&(W[:,:,2]<0.94))
b,g,r=img[ys,xs,0].astype(int),img[ys,xs,1].astype(int),img[ys,xs,2].astype(int)
red=(r>150)&(g<100)&(b<100); print("red burner pixels visible:", red.sum(), "of", len(ys))
# original moka pot location now empty?
m2=(np.abs(W[:,:,0]+0.047)<0.06)&(np.abs(W[:,:,1]+0.237)<0.06)&(W[:,:,2]>0.91)
print("points above table at old pot location:", m2.sum())
EOF

# openrua op 44
timeout 60 python3 - <<'EOF'
import numpy as np, cv2
for name,roi in [("bird7.png",(250,320,340,410)),("agent7.png",(270,330,380,500)),("birdview.png",(250,320,340,410)),("agentview.png",(240,330,380,500))]:
    img=cv2.imread(name); y0,y1,x0,x1=roi; c=img[y0:y1,x0:x1].astype(int)
    red=(c[:,:,2]>c[:,:,1]+60)&(c[:,:,2]>c[:,:,0]+60)
    print(name, "reddish pixels in stove ROI:", red.sum())
EOF
