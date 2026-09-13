#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la && find . -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
cat tools/action/fjt_send.py; echo ----; cat tools/action/gripper_cmd.py; echo ----; cat tools/action/ik_move.py

# openrua op 3
timeout 30 ros2 topic list; echo ----; timeout 20 ros2 action list; echo ----; timeout 20 ros2 node list

# openrua op 4
for c in agentview birdview frontview sideview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c; done; timeout 20 ros2 topic echo /joint_states --once

# openrua op 5
python3 -c "
import cv2
im=cv2.imread('agentview.png')
crop=im[150:420,80:420]
cv2.imwrite('agent_crop.png', cv2.resize(crop,None,fx=2.5,fy=2.5,interpolation=cv2.INTER_CUBIC))
im2=cv2.imread('robot0_eye_in_hand.png')
cv2.imwrite('eih_crop.png', cv2.resize(im2[60:360,180:560],None,fx=2,fy=2,interpolation=cv2.INTER_CUBIC))
"

# openrua op 6
timeout 20 ros2 topic echo /tf_static --once 2>/dev/null | grep -E "frame_id|translation|x:|y:|z:|w:" | head -80; echo ----; timeout 10 ros2 topic echo /birdview/color/camera_info --once 2>/dev/null | grep -A12 "^k:"

# openrua op 7
cat > tfdump.py <<'EOF'
import rclpy, yaml
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
rclpy.init(); n=rclpy.create_node("tfd")
got=[]
qos=QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
n.create_subscription(TFMessage,"/tf_static",lambda m:got.append(("static",m)),qos)
n.create_subscription(TFMessage,"/tf",lambda m:got.append(("dyn",m)),10)
import time
t=time.time()
while time.time()-t<4: rclpy.spin_once(n,timeout_sec=0.2)
seen={}
for k,m in got:
    for tr in m.transforms:
        seen[(k,tr.header.frame_id,tr.child_frame_id)]=tr.transform
for (k,p,c),t in seen.items():
    print(k,p,"->",c,"t=(%.3f %.3f %.3f) q=(%.3f %.3f %.3f %.3f)"%(t.translation.x,t.translation.y,t.translation.z,t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w))
EOF
timeout 30 python3 tfdump.py 2>/dev/null

# openrua op 8
cat > scene.py <<'EOF'
import numpy as np, rclpy, struct, sys
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
cam = sys.argv[1] if len(sys.argv)>1 else "birdview"
rclpy.init(); n=rclpy.create_node("scene")
got={}
n.create_subscription(Image,f"/{cam}/depth/image_raw",lambda m:got.setdefault("d",m),1)
n.create_subscription(Image,f"/{cam}/color/image_raw",lambda m:got.setdefault("c",m),1)
n.create_subscription(CameraInfo,f"/{cam}/color/camera_info",lambda m:got.setdefault("i",m),1)
while len(got)<3: rclpy.spin_once(n,timeout_sec=0.2)
br=CvBridge()
d=br.imgmsg_to_cv2(got["d"],"passthrough").astype(np.float32)
c=br.imgmsg_to_cv2(got["c"],"bgr8")
k=got["i"].k; fx,fy,cx,cy=k[0],k[4],k[2],k[5]
np.save(f"{cam}_depth.npy",d)
print("depth shape",d.shape,"min",np.nanmin(d),"max",np.nanmax(d))
# for birdview: cam at (-0.2,0,3.0), optical z down. q=(0.707,0.707,0,0): R = rotation
# compute world from quaternion
q=np.array([0.707107,0.707107,0,0]); x,y,z,w=q
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
t=np.array([-0.2,0,3.0])
H,W=d.shape
us,vs=np.meshgrid(np.arange(W),np.arange(H))
X=(us-cx)*d/fx; Y=(vs-cy)*d/fy
P=np.stack([X,Y,d],-1)@R.T+t
np.save(f"{cam}_world.npy",P)
zw=P[...,2]
# table height: mode of z in the central region
cen=zw[200:300,200:450]
hist,edges=np.histogram(cen[np.isfinite(cen)],bins=200)
tz=edges[np.argmax(hist)]
print("table z ~",tz)
mask=(zw>tz+0.01)&np.isfinite(zw)
mask[:, :]&=True
num,lab,stats,cents=cv2.connectedComponentsWithStats(mask.astype(np.uint8))
for i in range(1,num):
    if stats[i,4]<15: continue
    m=lab==i
    pts=P[m]
    col=c[m].mean(0)
    print(f"blob {i}: px area {stats[i,4]} centroid px ({cents[i][0]:.0f},{cents[i][1]:.0f}) world x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] zmax {pts[:,2].max():.3f} mean xyz ({pts[:,0].mean():.3f},{pts[:,1].mean():.3f},{pts[:,2].mean():.3f}) bgr {col.astype(int)}")
EOF
timeout 60 python3 scene.py birdview 2>/dev/null

# openrua op 9
cat > scene2.py <<'EOF'
import numpy as np, cv2
P=np.load("birdview_world.npy"); c=cv2.imread("birdview.png")
zw=P[...,2]
for (u,v) in [(200,200),(450,350),(250,330),(400,200),(330,330)]:
    print("table sample",u,v,P[v,u].round(3))
tz=np.median(zw[np.isfinite(zw)&(zw>0.40)&(zw<0.47)])
print("table median z",tz)
mask=(zw>tz+0.012)&np.isfinite(zw)&(zw<0.9)
num,lab,stats,cents=cv2.connectedComponentsWithStats(mask.astype(np.uint8))
for i in range(1,num):
    if stats[i,4]<10: continue
    m=lab==i; pts=P[m]; col=c[m].mean(0)
    top=pts[pts[:,2]>pts[:,2].max()-0.01]
    print(f"blob {i}: area {stats[i,4]} px({cents[i][0]:.0f},{cents[i][1]:.0f}) x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] ztop {pts[:,2].max():.3f} topmean ({top[:,0].mean():.3f},{top[:,1].mean():.3f}) bgr {col.astype(int)}")
EOF
python3 scene2.py

# openrua op 10
cat > handpose.py <<'EOF'
import rclpy, time
from tf2_ros import Buffer, TransformListener
rclpy.init(); n=rclpy.create_node("hp"); b=Buffer(); TransformListener(b,n)
t0=time.time()
while time.time()-t0<5: rclpy.spin_once(n,timeout_sec=0.1)
for a,c in [("panda_link0","panda_hand"),("world","panda_hand"),("world","panda_link0")]:
    try:
        t=b.lookup_transform(a,c,rclpy.time.Time()).transform
        print(a,"->",c,"t=(%.4f %.4f %.4f) q=(%.4f %.4f %.4f %.4f)"%(t.translation.x,t.translation.y,t.translation.z,t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w))
    except Exception as e: print(a,c,"ERR",e)
EOF
timeout 30 python3 handpose.py 2>/dev/null; python3 -c "
import numpy as np; P=np.load('birdview_world.npy'); print(P[284,318], P[280,320], P[290,315])"

# openrua op 11
cat > scene3.py <<'EOF'
import numpy as np, rclpy, sys, time, cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener
cam=sys.argv[1]; zthr=float(sys.argv[2]) if len(sys.argv)>2 else 0.44
rclpy.init(); n=rclpy.create_node("scene3"); b=Buffer(); TransformListener(b,n)
got={}
n.create_subscription(Image,f"/{cam}/depth/image_raw",lambda m:got.setdefault("d",m),1)
n.create_subscription(Image,f"/{cam}/color/image_raw",lambda m:got.setdefault("c",m),1)
n.create_subscription(CameraInfo,f"/{cam}/color/camera_info",lambda m:got.setdefault("i",m),1)
t0=time.time()
while len(got)<3 or not b.can_transform("world",f"{cam}_optical_frame",rclpy.time.Time()):
    rclpy.spin_once(n,timeout_sec=0.2)
    if time.time()-t0>30: raise SystemExit("timeout")
br=CvBridge()
d=br.imgmsg_to_cv2(got["d"],"passthrough").astype(np.float32)
c=br.imgmsg_to_cv2(got["c"],"bgr8")
k=got["i"].k; fx,fy,cx,cy=k[0],k[4],k[2],k[5]
tr=b.lookup_transform("world",f"{cam}_optical_frame",rclpy.time.Time()).transform
q=tr.rotation; x,y,z,w=q.x,q.y,q.z,q.w
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
t=np.array([tr.translation.x,tr.translation.y,tr.translation.z])
H,W=d.shape; us,vs=np.meshgrid(np.arange(W),np.arange(H))
P=np.stack([(us-cx)*d/fx,(vs-cy)*d/fy,d],-1)@R.T+t
np.save(f"{cam}_world.npy",P); cv2.imwrite(f"{cam}.png",c)
zw=P[...,2]
mask=(zw>zthr)&np.isfinite(zw)&(zw<0.75)&(d<2.5)
num,lab,stats,cents=cv2.connectedComponentsWithStats(mask.astype(np.uint8))
for i in range(1,num):
    if stats[i,4]<10: continue
    m=lab==i; pts=P[m]; col=c[m].mean(0)
    print(f"blob {i}: area {stats[i,4]} px({cents[i][0]:.0f},{cents[i][1]:.0f}) x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] z[{pts[:,2].min():.3f},{pts[:,2].max():.3f}] bgr {col.astype(int)}")
EOF
timeout 60 python3 scene3.py agentview 0.44 2>/dev/null

# openrua op 12
python3 -c "
import numpy as np
for cam,thr in [('birdview',0.437),('agentview',0.443)]:
    P=np.load(cam+'_world.npy'); 
    m=(P[...,0]>0.02)&(P[...,0]<0.12)&(P[...,1]>0.02)&(P[...,1]<0.09)&(P[...,2]>thr)&(P[...,2]<0.5)
    pts=P[m][:,:2]; c=pts.mean(0); u,s,vt=np.linalg.svd(pts-c)
    print(cam,'butter n',len(pts),'center',c.round(3),'axis',vt[0].round(3),'angle deg',np.degrees(np.arctan2(vt[0][1],vt[0][0])).round(1),'extent',(s/np.sqrt(len(pts))*2*np.sqrt(3)).round(3))
"


# openrua op 13
mkdir -p "$(dirname /workspace/ctl.py)"
cat > /workspace/ctl.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Small verified-motion controller for this Panda.

Usage:
  python3 ctl.py state                       # joints, hand & TCP pose (world)
  python3 ctl.py goto X Y Z [yaw_deg] [secs] # TCP (fingertip point) to world pose, top-down
  python3 ctl.py line X Y Z [yaw_deg] [secs] # same, but as a multi-waypoint straight-ish path
  python3 ctl.py grip open|close
World -> panda_link0 offset comes from TF; IK is asked in the planner's
model frame (link0) with an empty frame_id, seeded with the arm joints.
"""
import math
import sys
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
W2B = np.array([-0.51, 0.0, 0.42])  # world -> panda_link0 (verified via TF)


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def topdown_quat(yaw_deg):
    """Hand z down; yaw rotates fingers (they close along hand y)."""
    h = math.radians(yaw_deg) / 2
    # qz(yaw) * (1,0,0,0)  -> (cos h, sin h, 0, 0) in (x,y,z,w)? compute properly
    # qz = (0,0,sin h,cos h); q0 = (1,0,0,0)
    # product qz*q0: w = -0, x = cos h, y = sin h, z = 0
    return (math.cos(h), math.sin(h), 0.0, 0.0)


class Ctl:
    def __init__(self):
        rclpy.init()
        self.n = rclpy.create_node("ctl")
        self.js = {}
        self.n.create_subscription(JointState, "/joint_states",
                                   lambda m: self.js.update(zip(m.name, m.position)), 10)
        self.buf = Buffer()
        TransformListener(self.buf, self.n)
        self.ik = self.n.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fjt = ActionClient(self.n, FollowJointTrajectory, FJT["port"])
        self.gr = ActionClient(self.n, GripperCommand, GRIP["port"])
        t0 = time.time()
        while (not self.js or time.time() - t0 < 1.0) and time.time() - t0 < 10:
            rclpy.spin_once(self.n, timeout_sec=0.1)

    def spin(self, s=0.5):
        t0 = time.time()
        while time.time() - t0 < s:
            rclpy.spin_once(self.n, timeout_sec=0.05)

    def arm_q(self):
        return [self.js[j] for j in ARM]

    def hand_pose(self):
        self.spin(0.3)
        t = self.buf.lookup_transform("world", "panda_hand", rclpy.time.Time()).transform
        p = np.array([t.translation.x, t.translation.y, t.translation.z])
        R = quat_to_R(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w)
        return p, R, (t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w)

    def state(self):
        self.spin(0.5)
        q = self.arm_q()
        p, R, quat = self.hand_pose()
        tcp = p + TCP * R[:, 2]
        print("joints:", np.round(q, 4).tolist())
        print("fingers:", round(self.js.get("panda_finger_joint1", -1), 4),
              round(self.js.get("panda_finger_joint2", -1), 4))
        print("hand world: %s quat %s" % (np.round(p, 4), np.round(quat, 4)))
        print("tcp  world: %s" % np.round(tcp, 4))

    def solve_ik(self, tcp_world, yaw_deg, seed=None):
        if not self.ik.wait_for_service(timeout_sec=10):
            raise SystemExit("IK service unavailable")
        qx, qy, qz, qw = topdown_quat(yaw_deg)
        R = quat_to_R(qx, qy, qz, qw)
        hand_world = np.array(tcp_world) - TCP * R[:, 2]
        hb = hand_world - W2B
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        pp = req.ik_request.pose_stamped.pose
        pp.position.x, pp.position.y, pp.position.z = map(float, hb)
        pp.orientation.x, pp.orientation.y, pp.orientation.z, pp.orientation.w = qx, qy, qz, qw
        s = JointState()
        s.name = list(ARM)
        s.position = [float(v) for v in (seed if seed is not None else self.arm_q())]
        req.ik_request.robot_state.joint_state = s
        req.ik_request.timeout.sec = 2
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.n, fut, timeout_sec=60)
        res = fut.result()
        if res is None:
            raise SystemExit("IK no answer")
        if res.error_code.val != 1:
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in ARM]

    def send_traj(self, points, secs):
        if not self.fjt.wait_for_server(timeout_sec=10):
            raise SystemExit("no FJT server")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        n = len(points)
        for i, q in enumerate(points):
            pt = JointTrajectoryPoint(positions=[float(v) for v in q])
            t = secs * (i + 1) / n
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.n, send)
        gh = send.result()
        if not gh.accepted:
            raise SystemExit("FJT goal rejected")
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.n, rf, timeout_sec=600)
        code = rf.result().result.error_code
        self.spin(0.5)
        err = np.abs(np.array(self.arm_q()) - np.array(points[-1])).max()
        print(f"traj done error_code={code} max_joint_err={err:.4f}")
        return code, err

    def goto(self, x, y, z, yaw=0.0, secs=3.0, waypoints=1):
        target = np.array([x, y, z])
        p0, R0, _ = self.hand_pose()
        tcp0 = p0 + TCP * R0[:, 2]
        seed = self.arm_q()
        pts = []
        for i in range(1, waypoints + 1):
            a = i / waypoints
            wp = tcp0 + a * (target - tcp0)
            q = self.solve_ik(wp, yaw, seed)
            if q is None:
                print(f"IK FAILED at waypoint {i}/{waypoints} {np.round(wp,3)}; no motion")
                return False
            pts.append(q)
            seed = q
        code, err = self.send_traj(pts, secs)
        p, R, _ = self.hand_pose()
        tcp = p + TCP * R[:, 2]
        print("tcp now %s (target %s) dist %.4f" % (np.round(tcp, 4), np.round(target, 4),
                                                   np.linalg.norm(tcp - target)))
        return code == 0

    def grip(self, what):
        if not self.gr.wait_for_server(timeout_sec=10):
            raise SystemExit("no gripper server")
        g = GripperCommand.Goal()
        g.command.position = float(GRIP["open_m"] if what == "open" else GRIP["closed_m"])
        g.command.max_effort = float(GRIP["max_effort"])
        f = self.gr.send_goal_async(g)
        rclpy.spin_until_future_complete(self.n, f, timeout_sec=60)
        rf = f.result().get_result_async()
        rclpy.spin_until_future_complete(self.n, rf, timeout_sec=300)
        r = rf.result().result
        self.spin(1.0)
        print(f"grip {what}: reached={r.reached_goal} stalled={r.stalled} pos={r.position:.4f} "
              f"fingers={self.js.get('panda_finger_joint1'):.4f},{self.js.get('panda_finger_joint2'):.4f}")


def main():
    a = sys.argv[1:]
    if not a:
        raise SystemExit(__doc__)
    c = Ctl()
    if a[0] == "state":
        c.state()
    elif a[0] in ("goto", "line"):
        x, y, z = map(float, a[1:4])
        yaw = float(a[4]) if len(a) > 4 else 0.0
        secs = float(a[5]) if len(a) > 5 else 3.0
        c.goto(x, y, z, yaw, secs, waypoints=1 if a[0] == "goto" else 4)
        c.state()
    elif a[0] == "grip":
        c.grip(a[1])
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 14
timeout 60 python3 ctl.py state 2>/dev/null

# openrua op 15
timeout 300 python3 ctl.py grip open 2>/dev/null && timeout 600 python3 ctl.py goto -0.084 -0.162 0.60 0 3 2>/dev/null

# openrua op 16
timeout 600 python3 ctl.py goto -0.084 -0.162 0.60 0 3 2>&1 | grep -v XMLPARSER | tail -20

# openrua op 17
cat > iktest.py <<'EOF'
import sys, numpy as np
sys.argv=[sys.argv[0]]
import ctl
c=ctl.Ctl()
tests=[(-0.0589,0.0,0.6744,0),(-0.084,-0.162,0.60,0),(-0.084,-0.162,0.60,10),(-0.084,-0.162,0.60,-10),(-0.084,-0.162,0.60,45),(-0.084,-0.162,0.60,90),(-0.084,-0.162,0.65,0),(-0.084,-0.162,0.55,0)]
for x,y,z,yaw in tests:
    q=c.solve_ik((x,y,z),yaw)
    print((x,y,z,yaw),"->",None if q is None else np.round(q,3).tolist())
EOF
timeout 300 python3 iktest.py 2>&1 | grep -v XMLPARSER

# openrua op 18
cat > iktest2.py <<'EOF'
import sys, numpy as np, rclpy
sys.argv=[sys.argv[0]]
import ctl
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
c=ctl.Ctl()
def try_ik(pos, quat, link="", frame="", avoid=True, seed=True):
    req=GetPositionIK.Request()
    req.ik_request.group_name="panda_arm"
    req.ik_request.ik_link_name=link
    req.ik_request.pose_stamped.header.frame_id=frame
    p=req.ik_request.pose_stamped.pose
    p.position.x,p.position.y,p.position.z=pos
    p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=quat
    req.ik_request.avoid_collisions=avoid
    if seed:
        s=JointState(); s.name=list(ctl.ARM); s.position=[float(v) for v in c.arm_q()]
        req.ik_request.robot_state.joint_state=s
    req.ik_request.timeout.sec=2
    f=c.ik.call_async(req); rclpy.spin_until_future_complete(c.n,f,timeout_sec=60)
    r=f.result()
    print(pos,quat,link,frame,avoid,"-> code",r.error_code.val, np.round(r.solution.joint_state.position,3).tolist()[:9] if r.error_code.val==1 else "")
hb=(0.457,0.0,0.3576); q=(0.9996,0.0,-0.0284,0.0)
try_ik(hb,q)
try_ik(hb,q,avoid=False)
try_ik(hb,q,link="panda_hand")
try_ik(hb,q,link="panda_link8")
try_ik(hb,q,frame="panda_link0")
try_ik(hb,q,frame="world")
try_ik(hb,q,seed=False)
EOF
timeout 300 python3 iktest2.py 2>&1 | grep -v XMLPARSER

# openrua op 19
cat >> iktest2.py <<'EOF'
print("---- world coords with empty frame")
try_ik((-0.053,0.0,0.7776),q)
try_ik((-0.053,0.0,0.7776),q,frame="world")
try_ik((-0.053,0.0,0.7776),q,link="panda_hand")
try_ik((-0.053,0.0,0.7776),q,link="panda_link8")
EOF
timeout 300 python3 iktest2.py 2>&1 | grep -v XMLPARSER | tail -5

# openrua op 20
python3 - <<'EOF' 2>&1 | grep -v XMLPARSER
import re
s=open("iktest2.py").read()
s=s.replace('    r=f.result()\n','    r=f.result()\n    if r is None: print(pos,quat,link,frame,"-> NO ANSWER"); return\n')
# drop the first block of tests
head,tail=s.split('hb=(0.457')
s=head+'hb=(0.457'+tail.split('print("---- world')[0].split('\n')[0]+'\nprint("---- world'+tail.split('print("---- world')[1]
open("iktest2.py","w").write(s)
EOF
timeout 400 python3 iktest2.py 2>&1 | grep -v XMLPARSER | tail -6

# openrua op 21
python3 - <<'EOF'
s=open("/workspace/ctl.py").read()
s=s.replace('''        hb = hand_world - W2B
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        pp = req.ik_request.pose_stamped.pose
        pp.position.x, pp.position.y, pp.position.z = map(float, hb)''',
'''        # verified on this machine: world-frame coords resolve correctly with
        # frame_id "world"; naming the tip link gives the HAND (not link8) pose
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.ik_link_name = "panda_hand"
        req.ik_request.pose_stamped.header.frame_id = "world"
        pp = req.ik_request.pose_stamped.pose
        pp.position.x, pp.position.y, pp.position.z = map(float, hand_world)''')
s=s.replace('''        if res is None:
            raise SystemExit("IK no answer")''','''        if res is None:
            print("IK no answer within 60s")
            return None''')
open("/workspace/ctl.py","w").write(s)
EOF
timeout 300 python3 iktest.py 2>&1 | grep -v XMLPARSER

# openrua op 22
timeout 900 python3 ctl.py goto -0.084 -0.162 0.60 0 3 2>&1 | grep -v XMLPARSER

# openrua op 23
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih1.png 2>/dev/null; timeout 60 python3 scene3.py robot0_eye_in_hand 0.455 2>/dev/null

# openrua op 24
python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_world.npy'); z=P[...,2]
for (u,v) in [(320,200),(200,250),(500,300),(100,450),(320,130)]: print(u,v,P[v,u].round(3))
can=P[290:340,300:340]; print('can top z median',np.median(can[...,2]).round(3), 'xy', np.median(can[...,0]).round(3), np.median(can[...,1]).round(3))
"

# openrua op 25
timeout 900 python3 ctl.py line -0.080 -0.161 0.455 0 3 2>&1 | grep -v XMLPARSER

# openrua op 26
timeout 900 python3 ctl.py goto -0.080 -0.161 0.455 0 2 2>&1 | grep -v XMLPARSER

# openrua op 27
python3 - <<'EOF'
s=open("/workspace/ctl.py").read()
s=s.replace('''    def hand_pose(self):
        self.spin(0.3)
''','''    def hand_pose(self):
        self.spin(0.3)
        t0 = time.time()
        while not self.buf.can_transform("world", "panda_hand", rclpy.time.Time()) \\
                and time.time() - t0 < 20:
            rclpy.spin_once(self.n, timeout_sec=0.1)
''')
open("/workspace/ctl.py","w").write(s)
EOF
timeout 900 python3 ctl.py goto -0.080 -0.161 0.455 0 2 2>&1 | grep -v XMLPARSER

# openrua op 28
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih2.png 2>/dev/null; timeout 60 python3 tools/perception/cam_snap.py agentview agent2.png 2>/dev/null; timeout 20 ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once 2>/dev/null | grep -A8 wrench

# openrua op 29
python3 -c "
import cv2
im=cv2.imread('agent2.png'); cv2.imwrite('agent2_crop.png', cv2.resize(im[120:400,120:420],None,fx=2.5,fy=2.5,interpolation=cv2.INTER_CUBIC))"

# openrua op 30
timeout 60 python3 tools/perception/cam_snap.py sideview side2.png 2>/dev/null; timeout 60 python3 tools/perception/cam_snap.py frontview front2.png 2>/dev/null; python3 -c "
import cv2
im=cv2.imread('side2.png'); cv2.imwrite('side2_crop.png', cv2.resize(im[300:480,180:420],None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('front2.png'); cv2.imwrite('front2_crop.png', cv2.resize(im[150:300,180:400],None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))"

# openrua op 31
cat > dbg.py <<'EOF'
import sys, numpy as np
sys.argv=[sys.argv[0]]
import ctl
c=ctl.Ctl(); c.spin(1)
q=c.solve_ik((-0.080,-0.161,0.455),0)
cur=np.array(c.arm_q())
print("cur ",np.round(cur,3).tolist()); print("goal",np.round(q,3).tolist()); print("diff",np.round(np.array(q)-cur,3).tolist())
EOF
timeout 120 python3 dbg.py 2>&1 | grep -v XMLPARSER

# openrua op 32
timeout 60 python3 scene3.py sideview 0.9 2>/dev/null >/dev/null; python3 -c "
import numpy as np, cv2
P=np.load('sideview_world.npy'); c=cv2.imread('sideview.png')
# finger region: pixels whose world x within hand x +-0.05, y within -0.3..-0.05, z between 0.40 and 0.60 and greyish color
m=(np.abs(P[...,0]+0.08)<0.04)&(P[...,1]>-0.26)&(P[...,1]<-0.05)&(P[...,2]>0.40)&(P[...,2]<0.60)
g=c.astype(int); grey=(np.abs(g[...,0]-g[...,1])<12)&(np.abs(g[...,1]-g[...,2])<12)&(g[...,0]>90)
mm=m&grey
pts=P[mm]; print('grey pts',len(pts)); 
if len(pts): 
  print('z min',pts[:,2].min().round(3),'z pct',np.percentile(pts[:,2],[1,5,50]).round(3)); 
  low=pts[pts[:,2]<pts[:,2].min()+0.01]; print('lowest xy',low[:,:2].mean(0).round(3))
"


# openrua op 33
python3 -c "
import numpy as np, cv2
P=np.load('sideview_world.npy'); c=cv2.imread('sideview.png')
for name,(y0,y1) in {'near finger':(-0.135,-0.10),'far finger':(-0.225,-0.19)}.items():
  m=(np.abs(P[...,0]+0.08)<0.03)&(P[...,1]>y0)&(P[...,1]<y1)&(P[...,2]>0.43)&(P[...,2]<0.60)
  pts=P[m]; cols=c[m]
  if len(pts):
    order=np.argsort(pts[:,2]); 
    for i in order[:5]: print(name, pts[i].round(3), cols[i])
"


# openrua op 34
timeout 60 python3 scene3.py robot0_eye_in_hand 0.9 2>/dev/null >/dev/null; python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_world.npy')
for (u,v) in [(160,385),(165,380),(470,385),(465,380),(150,395),(480,395),(320,330),(320,300),(320,250)]: print(u,v,P[v,u].round(3))
"

# openrua op 35
timeout 900 python3 ctl.py goto -0.080 -0.161 0.62 0 2 2>&1 | grep -v XMLPARSER | head -2 && timeout 900 python3 ctl.py goto -0.080 -0.161 0.62 90 3 2>&1 | grep -v XMLPARSER | head -2 && timeout 900 python3 ctl.py line -0.080 -0.161 0.455 90 3 2>&1 | grep -v XMLPARSER

# openrua op 36
timeout 900 python3 ctl.py goto -0.080 -0.161 0.62 90 6 2>&1 | grep -v XMLPARSER

# openrua op 37
timeout 900 python3 ctl.py line -0.080 -0.161 0.455 90 4 2>&1 | grep -v XMLPARSER

# openrua op 38
timeout 900 python3 ctl.py goto -0.080 -0.161 0.60 90 3 2>&1 | grep -v XMLPARSER | head -2; timeout 900 python3 ctl.py goto -0.080 -0.40 0.60 90 3 2>&1 | grep -v XMLPARSER | head -2; timeout 900 python3 ctl.py line -0.080 -0.40 0.45 90 4 2>&1 | grep -v XMLPARSER; timeout 20 ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once 2>/dev/null | grep -A4 force

# openrua op 39
python3 -c "
import numpy as np
d=np.load('robot0_eye_in_hand_depth.npy'); P=np.load('robot0_eye_in_hand_world.npy')
np.set_printoptions(linewidth=250, precision=3, suppress=True)
print('raw depth rows 200..400 step 20, cols 100..540 step 40')
print(d[200:401:20,100:541:40])
print('world z same grid'); print(P[200:401:20,100:541:40,2])
"

# openrua op 40
sed -i 's|np.save(f"{cam}_world.npy",P); cv2.imwrite(f"{cam}.png",c)|np.save(f"{cam}_world.npy",P); np.save(f"{cam}_depth.npy",d); cv2.imwrite(f"{cam}.png",c)|' scene3.py
timeout 900 python3 ctl.py goto -0.080 -0.40 0.60 90 3 2>&1 | grep -v XMLPARSER | head -1; timeout 900 python3 ctl.py goto -0.080 -0.161 0.60 90 3 2>&1 | grep -v XMLPARSER | head -1; timeout 900 python3 ctl.py line -0.080 -0.161 0.455 90 4 2>&1 | grep -v XMLPARSER | head -2; timeout 60 python3 scene3.py robot0_eye_in_hand 0.9 2>/dev/null >/dev/null; python3 -c "
import numpy as np
d=np.load('robot0_eye_in_hand_depth.npy'); P=np.load('robot0_eye_in_hand_world.npy')
np.set_printoptions(linewidth=250, precision=3, suppress=True)
print('raw depth rows 200..400 step 20, cols 100..540 step 40')
print(d[200:401:20,100:541:40])
"

# openrua op 41
timeout 10 ros2 topic echo /robot0_eye_in_hand/color/camera_info --once 2>/dev/null | grep -A10 "^k:" | tr '\n' ' '; echo; python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_world.npy')
np.set_printoptions(linewidth=250, precision=3, suppress=True)
print('world x'); print(P[200:401:20,100:541:40,0]); print('world y'); print(P[200:401:20,100:541:40,1]); print('world z'); print(P[200:401:20,100:541:40,2])"

# openrua op 42
timeout 900 python3 ctl.py goto -0.080 -0.161 0.480 90 2 2>&1 | grep -v XMLPARSER | head -2; timeout 300 python3 ctl.py grip close 2>&1 | grep -v XMLPARSER

# openrua op 43
timeout 900 python3 ctl.py line -0.080 -0.161 0.72 90 3 2>&1 | grep -v XMLPARSER; timeout 60 python3 tools/perception/cam_snap.py agentview agent3.png 2>/dev/null

# openrua op 44
python3 - <<'EOF'
s=open("/workspace/ctl.py").read()
s=s.replace('''            q = self.solve_ik(wp, yaw, seed)
            if q is None:
                print(f"IK FAILED at waypoint {i}/{waypoints} {np.round(wp,3)}; no motion")
                return False''','''            q = self.solve_ik_near(wp, yaw, seed)
            if q is None:
                print(f"IK FAILED at waypoint {i}/{waypoints} {np.round(wp,3)}; no motion")
                return False''')
s=s.replace('''    def send_traj(self, points, secs):''','''    def solve_ik_near(self, tcp_world, yaw_deg, seed, tries=6, tol=0.5):
        """IK, but insist on the solution branch nearest the seed."""
        best = None
        for _ in range(tries):
            q = self.solve_ik(tcp_world, yaw_deg, seed)
            if q is None:
                continue
            d = float(np.abs(np.array(q) - np.array(seed)).max())
            if best is None or d < best[0]:
                best = (d, q)
            if d < tol:
                break
        if best is None:
            return None
        if best[0] >= tol:
            print(f"warning: nearest IK branch is {best[0]:.2f} rad from seed")
        return best[1]

    def send_traj(self, points, secs):''')
open("/workspace/ctl.py","w").write(s)
EOF
timeout 900 python3 ctl.py goto -0.080 -0.161 0.72 90 4 2>&1 | grep -v XMLPARSER

# openrua op 45
timeout 900 python3 ctl.py goto -0.005 0.253 0.72 90 5 2>&1 | grep -v XMLPARSER | head -2 && timeout 900 python3 ctl.py line -0.005 0.253 0.60 90 3 2>&1 | grep -v XMLPARSER | head -2

# openrua op 46
timeout 300 python3 ctl.py grip open 2>&1 | grep -v XMLPARSER; timeout 900 python3 ctl.py line -0.005 0.253 0.75 90 3 2>&1 | grep -v XMLPARSER | head -2; timeout 60 python3 tools/perception/cam_snap.py agentview agent4.png 2>/dev/null

# openrua op 47
timeout 900 python3 ctl.py goto 0.072 0.053 0.60 0 5 2>&1 | grep -v XMLPARSER | head -2 && timeout 60 python3 scene3.py robot0_eye_in_hand 0.437 2>/dev/null

# openrua op 48
timeout 120 python3 ctl.py state 2>&1 | grep -v XMLPARSER

# openrua op 49
timeout 900 python3 ctl.py goto 0.0735 0.0535 0.60 0 4 2>&1 | grep -v XMLPARSER | head -2 && timeout 900 python3 ctl.py line 0.0735 0.0535 0.432 0 4 2>&1 | grep -v XMLPARSER

# openrua op 50
timeout 900 python3 ctl.py goto 0.0735 0.0535 0.430 0 2 2>&1 | grep -v XMLPARSER | head -2; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih_butter.png 2>/dev/null

# openrua op 51
timeout 300 python3 ctl.py grip close 2>&1 | grep -v XMLPARSER; timeout 900 python3 ctl.py line 0.0735 0.0535 0.70 0 3 2>&1 | grep -v XMLPARSER | grep -E "traj|tcp now|fingers"

# openrua op 52
timeout 900 python3 ctl.py goto -0.005 0.305 0.72 0 5 2>&1 | grep -v XMLPARSER | grep -E "traj|tcp now|fingers" && timeout 900 python3 ctl.py line -0.005 0.305 0.60 0 3 2>&1 | grep -v XMLPARSER | grep -E "traj|tcp now|fingers"

# openrua op 53
timeout 300 python3 ctl.py grip open 2>&1 | grep -v XMLPARSER; timeout 900 python3 ctl.py line -0.005 0.305 0.78 0 3 2>&1 | grep -v XMLPARSER | grep -E "traj|tcp now"; timeout 900 python3 ctl.py goto -0.10 0.10 0.80 0 4 2>&1 | grep -v XMLPARSER | grep -E "traj|tcp now"; for c in agentview birdview frontview; do timeout 60 python3 tools/perception/cam_snap.py $c final_$c.png 2>/dev/null; done

# openrua op 54
timeout 60 python3 scene3.py birdview 0.44 2>/dev/null | grep -v "area 6" ; python3 -c "
import numpy as np, cv2
P=np.load('birdview_world.npy'); c=cv2.imread('birdview.png')
# basket interior region
m=(P[...,0]>-0.07)&(P[...,0]<0.055)&(P[...,1]>0.185)&(P[...,1]<0.32)
z=P[...,2][m]; print('basket interior z range',z.min().round(3),z.max().round(3))
crop=c[250:330,340:420]; cv2.imwrite('basket_top.png',cv2.resize(crop,None,fx=6,fy=6,interpolation=cv2.INTER_CUBIC))
# butter-colored pixels (orange/red) inside basket footprint
b,g,r=c[...,0].astype(int),c[...,1].astype(int),c[...,2].astype(int)
orange=(r>90)&(r>g+25)&(r>b+40)
mm=m&orange; pts=P[mm]; print('orange pixels in basket',mm.sum(), 'xy mean',pts[:,:2].mean(0).round(3) if len(pts) else None, 'z',pts[:,2].round(3)[:5] if len(pts) else None)
# check original butter spot is empty
m2=(P[...,0]>0.03)&(P[...,0]<0.115)&(P[...,1]>0.03)&(P[...,1]<0.08); print('old butter spot z max',P[...,2][m2].max().round(3))
m3=(P[...,0]>-0.12)&(P[...,0]<-0.04)&(P[...,1]>-0.20)&(P[...,1]<-0.12); print('old soup spot z max',P[...,2][m3].max().round(3))
"
