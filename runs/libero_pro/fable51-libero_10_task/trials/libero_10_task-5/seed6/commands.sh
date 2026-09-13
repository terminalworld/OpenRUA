#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la && find . -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
cat /workspace/tools/action/fjt_send.py; echo ----; cat /workspace/tools/action/gripper_cmd.py; echo ----; cat /workspace/tools/action/ik_move.py

# openrua op 3
ros2 topic list && echo ---- && ros2 action list && echo ---- && ros2 service list | grep -iE "ik|fk|plan|scene" && echo ---- && ros2 node list

# openrua op 4
for c in agentview frontview birdview sideview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c; done; ros2 topic echo /joint_states --once

# openrua op 5
cat > tfdump.py <<'EOF'
import rclpy, sys
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time
rclpy.init(); node = rclpy.create_node("tfdump")
buf = Buffer(); TransformListener(buf, node)
import time
for _ in range(30): rclpy.spin_once(node, timeout_sec=0.2)
print(buf.all_frames_as_string())
for f in sys.argv[1:]:
    try:
        t = buf.lookup_transform("world", f, Time())
        tr, q = t.transform.translation, t.transform.rotation
        print(f, "xyz=(%.4f %.4f %.4f) q=(%.4f %.4f %.4f %.4f)" % (tr.x,tr.y,tr.z,q.x,q.y,q.z,q.w))
    except Exception as e:
        print(f, "ERR", e)
rclpy.shutdown()
EOF
timeout 60 python3 tfdump.py panda_link0 panda_hand birdview_optical_frame agentview_optical_frame robot0_eye_in_hand_optical_frame

# openrua op 6
cat > scene.py <<'EOF'
import rclpy, numpy as np, struct, sys
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time
from cv_bridge import CvBridge
import cv2

def quat_R(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

class Cam:
    def __init__(self, node, buf, name):
        self.name=name; self.node=node; self.buf=buf
        self.depth=None; self.color=None; self.info=None
        node.create_subscription(Image, f"/{name}/depth/image_raw", self._d, 1)
        node.create_subscription(Image, f"/{name}/color/image_raw", self._c, 1)
        node.create_subscription(CameraInfo, f"/{name}/color/camera_info", self._i, 1)
    def _d(self,m): self.depth=CvBridge().imgmsg_to_cv2(m,"passthrough")
    def _c(self,m): self.color=CvBridge().imgmsg_to_cv2(m,"bgr8")
    def _i(self,m): self.info=m
    def ready(self): return self.depth is not None and self.color is not None and self.info is not None
    def T(self):
        t=self.buf.lookup_transform("world", f"{self.name}_optical_frame", Time())
        q=t.transform.rotation; T=np.eye(4); T[:3,:3]=quat_R(q.x,q.y,q.z,q.w)
        T[:3,3]=[t.transform.translation.x,t.transform.translation.y,t.transform.translation.z]; return T
    def cloud(self):
        k=self.info.k; fx,fy,cx,cy=k[0],k[4],k[2],k[5]
        h,w=self.depth.shape; u,v=np.meshgrid(np.arange(w),np.arange(h))
        z=self.depth.astype(float)
        pc=np.stack([(u-cx)*z/fx,(v-cy)*z/fy,z,np.ones_like(z)],-1)
        P=(self.T()@pc.reshape(-1,4).T).T[:,:3].reshape(h,w,3)
        return P
    def px2world(self,u,v):
        return self.cloud()[v,u]

def grab(names):
    rclpy.init(); node=rclpy.create_node("scene"); buf=Buffer(); TransformListener(buf,node)
    cams=[Cam(node,buf,n) for n in names]
    while not all(c.ready() for c in cams): rclpy.spin_once(node,timeout_sec=0.2)
    for _ in range(10): rclpy.spin_once(node,timeout_sec=0.1)
    for c in cams:
        while not buf.can_transform("world", f"{c.name}_optical_frame", Time()): rclpy.spin_once(node,timeout_sec=0.2)
    return node,cams

if __name__=="__main__":
    node,cams=grab([sys.argv[1]])
    c=cams[0]; P=c.cloud()
    np.save(f"{c.name}_cloud.npy",P); cv2.imwrite(f"{c.name}.png",c.color)
    for a in sys.argv[2:]:
        u,v=map(int,a.split(","))
        print(a, P[v,u], "depth", c.depth[v,u])
    rclpy.shutdown()
EOF
timeout 120 python3 scene.py birdview 330,262 258,183 235,183 285,170 285,195 320,180 366,240 100,200

# openrua op 7
cat > analyze.py <<'EOF'
import numpy as np, cv2
P=np.load("birdview_cloud.npy"); img=cv2.imread("birdview.png")
z=P[:,:,2]
print("table z median:", np.median(z[300:340, 250:400]))
# objects above table 0.89
mask=(z>0.895)&(P[:,:,0]>-0.6)&(P[:,:,0]<0.2)&(np.abs(P[:,:,1])<0.6)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask.astype(np.uint8))
for i in range(1,n):
    if stats[i,4]<20: continue
    m=lab==i
    pts=P[m]
    print(i,"px area",stats[i,4],"bbox",stats[i,:4],"x[%.3f %.3f] y[%.3f %.3f] zmax %.3f"%(pts[:,0].min(),pts[:,0].max(),pts[:,1].min(),pts[:,1].max(),pts[:,2].max()))
EOF
python3 analyze.py

# openrua op 8
cat > analyze2.py <<'EOF'
import numpy as np, cv2
P=np.load("birdview_cloud.npy"); img=cv2.imread("birdview.png")
z=P[:,:,2]
def rep(name, m):
    pts=P[m]; print(name, "n",m.sum(), "x[%.3f %.3f] y[%.3f %.3f] z pct 5/50/95/max %.3f %.3f %.3f %.3f"%(pts[:,0].min(),pts[:,0].max(),pts[:,1].min(),pts[:,1].max(),*np.percentile(pts[:,2],[5,50,95]),pts[:,2].max()))
    print("  centroid xy", pts[:,0].mean(), pts[:,1].mean())
# cup region
m=(z>0.895); sub=np.zeros_like(m); sub[248:280,305:350]=True; rep("cup", m&sub)
# cup body only (exclude handle): use color - cup body whitish/gray
hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
# caddy: brown box; restrict to region x in [-0.6,-0.25], y in [-0.36,0.13], z in [0.9,1.05]
m2=(z>0.93)&(z<1.1)&(P[:,:,0]>-0.62)&(P[:,:,0]<-0.2)&(P[:,:,1]>-0.4)&(P[:,:,1]<0.15)
rep("caddy walls", m2)
# print z map of caddy region coarsely
ys,xs=np.where(m2); print("caddy px bbox", xs.min(),xs.max(),ys.min(),ys.max())
# per row in caddy region print z
for v in range(160,210,4):
    print(v, " ".join("%.2f"%z[v,u] for u in range(210,310,4)))
EOF
python3 analyze2.py

# openrua op 9
timeout 60 python3 tools/perception/cam_snap.py robot0_robotview; timeout 60 python3 tools/perception/cam_snap.py galleryview; timeout 60 python3 tools/perception/cam_snap.py paperview

# openrua op 10
cat > heightmap.py <<'EOF'
import numpy as np, sys
from scene import grab
import rclpy
names=sys.argv[1:]
node,cams=grab(names)
res=0.005
x0,x1,y0,y1=-0.6,0.0,-0.45,0.25
W=int((x1-x0)/res); H=int((y1-y0)/res)
hm=np.full((H,W),np.nan)
for c in cams:
    P=c.cloud().reshape(-1,3)
    P=P[np.isfinite(P).all(1)]
    sel=(P[:,0]>x0)&(P[:,0]<x1)&(P[:,1]>y0)&(P[:,1]<y1)&(P[:,2]<1.2)
    P=P[sel]
    ix=((P[:,0]-x0)/res).astype(int); iy=((P[:,1]-y0)/res).astype(int)
    for a,b,z in zip(ix,iy,P[:,2]):
        if np.isnan(hm[b,a]) or z>hm[b,a]: hm[b,a]=z
np.save("hm.npy",hm)
# print coarse: rows=y (top=-0.45), cols=x
step=2
print("cols x from %.2f step %.3f"%(x0,res*step))
for j in range(0,H,step):
    row=hm[j,::step]
    s="".join(" ." if np.isnan(v) else ("%2d"%int(round((v-0.88)*100))) for v in row)
    print("y=%+.3f"%(y0+j*res), s)
rclpy.shutdown()
EOF
timeout 180 python3 heightmap.py agentview robot0_robotview 2>&1 | tail -80

# openrua op 11
cat > cup.py <<'EOF'
import numpy as np
P=np.load("birdview_cloud.npy"); z=P[:,:,2]
for v in range(244,284,2):
    print(v, "%.3f"%P[v,330,0], " ".join("%3d"%int(round((z[v,u]-0.88)*100)) for u in range(300,352,2)))
print("u->y:", " ".join("%.3f"%P[262,u,1] for u in range(300,352,2)))
EOF
python3 cup.py

# openrua op 12
cat > cupprofile.py <<'EOF'
import numpy as np, sys
from scene import grab
import rclpy
node,cams=grab(["sideview","frontview"])
for c in cams:
    P=c.cloud().reshape(-1,3); P=P[np.isfinite(P).all(1)]
    sel=(P[:,0]>-0.20)&(P[:,0]<-0.05)&(P[:,1]>-0.08)&(P[:,1]<0.12)&(P[:,2]>0.885)&(P[:,2]<1.06)
    Q=P[sel]
    print(c.name, len(Q))
    for lo in np.arange(0.885,1.05,0.01):
        s=Q[(Q[:,2]>=lo)&(Q[:,2]<lo+0.01)]
        if len(s)<3: continue
        print("  z %.3f-%.3f n=%4d x[%.3f %.3f] y[%.3f %.3f]"%(lo,lo+0.01,len(s),s[:,0].min(),s[:,0].max(),s[:,1].min(),s[:,1].max()))
rclpy.shutdown()
EOF
timeout 120 python3 cupprofile.py

# openrua op 13
timeout 300 python3 tools/action/gripper_cmd.py 0.04; ros2 topic echo /joint_states --once | grep -A9 position

# openrua op 14
timeout 120 python3 scene.py birdview 330,262 > /dev/null && python3 cup.py | head -3 && timeout 120 python3 cupprofile.py 2>&1 | head -20

# openrua op 15
cat > caddy.py <<'EOF'
import numpy as np
P=np.load("birdview_cloud.npy"); z=P[:,:,2]
print("table z", np.median(z[300:340,250:400]))
for v in range(158,212):
    print(v, " ".join("%.3f"%P[v,u,0] for u in (280,)), " ".join("%3d"%int(round((z[v,u]-0.88)*1000)) for u in range(214,302,3)))
print("u->y:", " ".join("%.3f"%P[180,u,1] for u in range(214,302,3)))
EOF
python3 caddy.py

# openrua op 16
cat > cupprofile2.py <<'EOF'
import numpy as np, sys
from scene import grab
import rclpy
node,cams=grab(["sideview","frontview","agentview","birdview"])
allP=[]
for c in cams:
    P=c.cloud().reshape(-1,3); P=P[np.isfinite(P).all(1)]
    sel=(P[:,0]>-0.20)&(P[:,0]<-0.05)&(P[:,1]>-0.08)&(P[:,1]<0.12)&(P[:,2]>0.882)&(P[:,2]<1.0)
    allP.append(P[sel])
Q=np.concatenate(allP)
# estimate axis: rim points z>0.975, fit circle center by mean of extremes
rim=Q[Q[:,2]>0.98]; 
# exclude handle: y< -0.0
body=rim[rim[:,1]>0.0]
cx=(body[:,0].min()+body[:,0].max())/2; cy=(body[:,1].min()+body[:,1].max())/2
print("rim center est", cx, cy, "rim diam x", body[:,0].max()-body[:,0].min(), "y", body[:,1].max()-body[:,1].min())
for lo in np.arange(0.882,1.0,0.01):
    s=Q[(Q[:,2]>=lo)&(Q[:,2]<lo+0.01)&(Q[:,1]>cy-0.02)]  # exclude handle side
    if len(s)<5: continue
    r=np.hypot(s[:,0]-cx,s[:,1]-cy)
    print("z %.3f n=%4d r pct50/90/98/max: %.3f %.3f %.3f %.3f"%(lo,len(s),*np.percentile(r,[50,90,98]),r.max()))
np.save("cupQ.npy",Q)
rclpy.shutdown()
EOF
timeout 180 python3 cupprofile2.py

# openrua op 17
ls /workspace/.ros/log/ | head; grep -il "caddy" /workspace/.ros/log/* 2>/dev/null | head; find / -iname "*caddy*" 2>/dev/null | grep -v proc | head -20

# openrua op 18
cat /workspace/.ros/log/python3_63_1789262167520.log | head -30; ros2 topic echo /planning_scene_world --once 2>/dev/null | head -20; timeout 20 ros2 topic echo /pipeline_state --once

# openrua op 19
head -c 3000 /workspace/.ros/log/python3_63_1789262167520.log; echo; ros2 topic info /pipeline_state; ros2 topic info /robot_description

# openrua op 20
pip list 2>/dev/null | grep -iE "libero|robosuite|mujoco|robocasa|bddl" ; python3 -c "import libero; print(libero.__file__)" 2>&1 | tail -1; find / -type d \( -iname "*libero*" -o -iname "*robosuite*" \) 2>/dev/null | grep -v proc | head

# openrua op 21
timeout 600 python3 tools/action/fjt_send.py 1.0,-0.161,0,-2.4446,0,2.2268,0.785 3 && timeout 120 python3 scene.py birdview 330,262 >/dev/null && echo saved

# openrua op 22
ros2 topic echo /joint_states --once | grep -A8 position

# openrua op 23
timeout 600 python3 tools/action/fjt_send.py 1.0,-0.161,0,-2.4446,0,2.2268,0.785 3; ros2 topic echo /joint_states --once | grep -A2 position

# openrua op 24
timeout 120 python3 scene.py birdview 330,262 >/dev/null; cat > caddy2.py <<'EOF'
import numpy as np
P=np.load("birdview_cloud.npy"); z=P[:,:,2]
us=list(range(214,380,3))
print("u->y:   ", " ".join("%4d"%int(P[180,u,1]*1000) for u in us))
for v in range(158,212,1):
    print(v, "%.3f"%P[v,280,0], " ".join("%4d"%int(round((z[v,u]-0.88)*1000)) for u in us))
EOF
python3 caddy2.py

# openrua op 25
python3 -c "
import cv2
im=cv2.imread('sideview.png'); crop=im[230:330,290:420]; crop=cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST); cv2.imwrite('side_crop.png',crop)
im=cv2.imread('frontview.png'); crop=im[280:360,280:400]; crop=cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST); cv2.imwrite('front_crop.png',crop)
"

# openrua op 26
ros2 interface show moveit_msgs/srv/GetPositionFK | head -30; ros2 interface show moveit_msgs/srv/GetPositionIK | grep -v "^\s*#" | head -40

# openrua op 27
mkdir -p "$(dirname /workspace/ctl.py)"
cat > /workspace/ctl.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Reusable robot controller: joint state, FK/IK (MoveIt), trajectories,
gripper, camera snapshots. Poses are WORLD frame; converted to the arm
base frame (machine.yaml planning_frame) for MoveIt."""
import sys, time
import numpy as np
import rclpy, yaml
from rclpy.action import ActionClient
from rclpy.time import Time
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState, Image, CameraInfo
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge
import cv2

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
TW = next(a for a in M["actuators"] if a["kind"] == "cartesian_twist")
ARM = FJT["joints"]
BASE_IN_WORLD = None  # filled from TF


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def R_quat(R):
    # returns x,y,z,w
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


def topdown_quat(yaw=0.0):
    """Hand z pointing down (world -z); yaw rotates finger axis about world z.
    yaw=0: fingers open along world y. yaw=pi/2: along world x."""
    Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
    R = Rz @ np.diag([1.0, -1.0, -1.0])
    return R_quat(R)


class Robot:
    def __init__(self, name="ctl"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self.js = None
        self.node.create_subscription(JointState, "/joint_states", self._js, 1)
        self.tfbuf = Buffer(); TransformListener(self.tfbuf, self.node)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.twist_pub = self.node.create_publisher(TwistStamped, TW["port"], 10)
        self.bridge = CvBridge()
        self.fjt.wait_for_server(10); self.grip.wait_for_server(10)
        self.ik.wait_for_service(10); self.fk.wait_for_service(10)
        while self.js is None:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        while not self.tfbuf.can_transform("world", "panda_link0", Time()):
            rclpy.spin_once(self.node, timeout_sec=0.2)
        t = self.tfbuf.lookup_transform("world", "panda_link0", Time()).transform.translation
        self.base = np.array([t.x, t.y, t.z])

    def _js(self, m):
        self.js = m

    def spin(self, n=5, dt=0.05):
        for _ in range(n):
            rclpy.spin_once(self.node, timeout_sec=dt)

    def joints(self):
        self.js = None
        while self.js is None:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        d = dict(zip(self.js.name, self.js.position))
        return d

    def arm_q(self):
        d = self.joints(); return [d[j] for j in ARM]

    def finger(self):
        d = self.joints(); return d["panda_finger_joint1"], d["panda_finger_joint2"]

    # ---------- kinematics ----------
    def hand_pose(self):
        """world pose of panda_hand from FK service (avoids stale TF)."""
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = ["panda_hand"]
        q = self.arm_q()
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = q
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + self.base
        return pos, np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])

    def ik_solve(self, pos_world, quat, seed=None):
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        pb = np.array(pos_world) - self.base
        p.position.x, p.position.y, p.position.z = map(float, pb)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        seed = seed if seed is not None else self.arm_q()
        req.ik_request.robot_state.joint_state.name = list(ARM)
        req.ik_request.robot_state.joint_state.position = list(map(float, seed))
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=1)
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in ARM]

    # ---------- motion ----------
    def move_joints(self, positions, seconds=3.0, retries=2):
        for attempt in range(retries + 1):
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = list(ARM)
            pt = JointTrajectoryPoint(positions=list(map(float, positions)))
            pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
            goal.trajectory.points = [pt]
            send = self.fjt.send_goal_async(goal)
            rclpy.spin_until_future_complete(self.node, send)
            res = send.result().get_result_async()
            rclpy.spin_until_future_complete(self.node, res)
            code = res.result().result.error_code
            q = np.array(self.arm_q()); err = np.abs(q - np.array(positions)).max()
            print(f"  move: code={code} max_joint_err={err:.4f}", flush=True)
            if err < 0.02:
                return True
        return False

    def move_pose(self, pos_world, quat, seconds=3.0, seed=None):
        q = self.ik_solve(pos_world, quat, seed)
        if q is None:
            print("  IK failed for", pos_world, flush=True); return False
        ok = self.move_joints(q, seconds)
        p, _ = self.hand_pose()
        print(f"  hand now at {p.round(4)} (target {np.array(pos_world).round(4)})", flush=True)
        return ok

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=120)
        r = rf.result().result
        f = self.finger()
        print(f"  gripper: reached={r.reached_goal} stalled={r.stalled} fingers={f[0]:.4f},{f[1]:.4f}", flush=True)
        return f

    def servo(self, lin, n=20, ang=(0, 0, 0)):
        msg = TwistStamped(); msg.header.frame_id = TW["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, lin)
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = map(float, ang)
        for _ in range(n):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.twist_pub.publish(msg); rclpy.spin_once(self.node, timeout_sec=0.05)

    # ---------- vision ----------
    def snap(self, cam, out=None):
        got = {}
        s1 = self.node.create_subscription(Image, f"/{cam}/color/image_raw", lambda m: got.setdefault("c", m), 1)
        s2 = self.node.create_subscription(Image, f"/{cam}/depth/image_raw", lambda m: got.setdefault("d", m), 1)
        s3 = self.node.create_subscription(CameraInfo, f"/{cam}/color/camera_info", lambda m: got.setdefault("i", m), 1)
        while len(got) < 3:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        for s in (s1, s2, s3):
            self.node.destroy_subscription(s)
        color = self.bridge.imgmsg_to_cv2(got["c"], "bgr8")
        depth = self.bridge.imgmsg_to_cv2(got["d"], "passthrough").astype(float)
        k = got["i"].k
        out = out or f"/workspace/{cam}.png"
        cv2.imwrite(out, color)
        frame = f"{cam}_optical_frame"
        while not self.tfbuf.can_transform("world", frame, Time()):
            rclpy.spin_once(self.node, timeout_sec=0.2)
        t = self.tfbuf.lookup_transform("world", frame, Time())
        q = t.transform.rotation
        T = np.eye(4); T[:3, :3] = quat_R(q.x, q.y, q.z, q.w)
        T[:3, 3] = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
        h, w = depth.shape; u, v = np.meshgrid(np.arange(w), np.arange(h))
        pc = np.stack([(u - k[2]) * depth / k[0], (v - k[5]) * depth / k[4], depth, np.ones_like(depth)], -1)
        P = (T @ pc.reshape(-1, 4).T).T[:, :3].reshape(h, w, 3)
        return color, depth, P, T
OPENRUA_EOF

# openrua op 28
cat > step1.py <<'EOF'
from ctl import *
r=Robot("step1")
print("base", r.base)
p,q=r.hand_pose(); print("hand", p, q)
print("topdown quat yaw0", topdown_quat(0), "yaw90", topdown_quat(np.pi/2))
# move above caddy middle column to inspect with eye-in-hand: camera ~5cm +x of hand
ok=r.move_pose([-0.48,-0.14,1.30], topdown_quat(0), 4)
color,depth,P,T=r.snap("robot0_eye_in_hand","/workspace/eih1.png")
np.save("/workspace/eih1_P.npy",P)
print("cam T", T[:3,3])
EOF
timeout 900 python3 -u step1.py

# openrua op 29
cat > step1b.py <<'EOF'
from ctl import *
r=Robot("step1b")
r.base=np.zeros(3)  # test hypothesis: MoveIt model frame == world
p,q=r.hand_pose(); print("FK hand", p, q)
t=r.tfbuf.lookup_transform("world","panda_hand",Time()).transform.translation; print("TF hand", t.x,t.y,t.z)
sol=r.ik_solve(p,q); print("IK at current pose:", None if sol is None else np.round(sol,3), "current", np.round(r.arm_q(),3))
sol=r.ik_solve([-0.48,-0.14,1.30], topdown_quat(0)); print("IK above caddy:", None if sol is None else np.round(sol,3))
EOF
timeout 600 python3 -u step1b.py

# openrua op 30
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/ctl.py", "old_string": "        t = self.tfbuf.lookup_transform(\\"world\\", \\"panda_link0\\", Time()).transform.translation\\n        self.base = np.array([t.x, t.y, t.z])", "new_string": "        # Verified on this machine: /compute_fk and /compute_ik with empty\\n        # frame_id use WORLD coordinates (FK of panda_hand == TF world->hand),\\n        # so no base offset is applied.\\n        self.base = np.zeros(3)", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 31
cat > step2.py <<'EOF'
from ctl import *
r=Robot("step2")
ok=r.move_pose([-0.48,-0.14,1.30], topdown_quat(0), 4)
color,depth,P,T=r.snap("robot0_eye_in_hand","/workspace/eih1.png")
np.save("/workspace/eih1_P.npy",P)
print("cam pos", T[:3,3].round(4))
EOF
timeout 900 python3 -u step2.py

# openrua op 32
cat > hm2.py <<'EOF'
import numpy as np, sys
P=np.load(sys.argv[1]); 
res=0.004
x0,x1,y0,y1=-0.50,-0.30,-0.40,0.10
W=int((x1-x0)/res); H=int((y1-y0)/res)
hm=np.full((H,W),np.nan)
Q=P.reshape(-1,3); Q=Q[np.isfinite(Q).all(1)]
sel=(Q[:,0]>x0)&(Q[:,0]<x1)&(Q[:,1]>y0)&(Q[:,1]<y1)&(Q[:,2]<1.2)
Q=Q[sel]
ix=((Q[:,0]-x0)/res).astype(int); iy=((Q[:,1]-y0)/res).astype(int)
for a,b,z in zip(ix,iy,Q[:,2]):
    if np.isnan(hm[b,a]) or z>hm[b,a]: hm[b,a]=z
print("cols x from %.3f step %.3f; values mm above table 0.880"%(x0,res))
print("       ", "".join("%4d"%int(round((x0+i*res)*1000)) for i in range(W)))
for j in range(H):
    row=hm[j]
    s="".join("   ." if np.isnan(v) else ("%4d"%int(round((v-0.88)*1000))) for v in row)
    print("y=%+.3f"%(y0+j*res), s)
EOF
python3 hm2.py eih1_P.npy 2>&1 | cut -c1-260

# openrua op 33
cat > step3.py <<'EOF'
from ctl import *
r=Robot("step3")
ok=r.move_pose([-0.122+0.035, 0.044-0.035, 1.25], topdown_quat(np.pi/2), 4)
p,q=r.hand_pose(); print("hand", p.round(4), q.round(4))
color,depth,P,T=r.snap("robot0_eye_in_hand","/workspace/eih2.png")
np.save("/workspace/eih2_P.npy",P)
print("cam pos", T[:3,3].round(4))
EOF
timeout 900 python3 -u step3.py

# openrua op 34
cat > step3b.py <<'EOF'
from ctl import *
r=Robot("step3b")
r.spin(20)
for f in ["panda_hand","panda_leftfinger","panda_rightfinger","robot0_eye_in_hand_optical_frame"]:
    t=r.tfbuf.lookup_transform("world",f,Time()).transform
    print(f, np.round([t.translation.x,t.translation.y,t.translation.z],4), np.round([t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w],4))
P=np.load("/workspace/eih2_P.npy")
Q=P.reshape(-1,3); Q=Q[np.isfinite(Q).all(1)]
# rim points: z between 0.975 and 1.0 near cup
sel=(Q[:,2]>0.97)&(Q[:,2]<1.01)&(Q[:,0]>-0.25)&(Q[:,0]<0.0)&(Q[:,1]>-0.1)&(Q[:,1]<0.15)
R=Q[sel]; print("rim pts", len(R), "z range", R[:,2].min(), R[:,2].max())
# fit circle to rim points (exclude handle: use algebraic fit with robust iteration)
def fit(pts):
    x,y=pts[:,0],pts[:,1]
    A=np.c_[2*x,2*y,np.ones_like(x)]; b=x*x+y*y
    c=np.linalg.lstsq(A,b,rcond=None)[0]
    cx,cy=c[0],c[1]; rad=np.sqrt(c[2]+cx*cx+cy*cy); return cx,cy,rad
pts=R[:,:2]
for it in range(5):
    cx,cy,rad=fit(pts)
    d=np.abs(np.hypot(pts[:,0]-cx,pts[:,1]-cy)-rad)
    pts=pts[d<np.percentile(d,80)]
print("rim circle center (%.4f %.4f) radius %.4f"%(cx,cy,rad))
# top-down height map around cup at 2mm
res=0.002; x0,x1,y0,y1=cx-0.08,cx+0.08,cy-0.09,cy+0.08
W=int((x1-x0)/res); H=int((y1-y0)/res); hm=np.full((H,W),np.nan)
sel=(Q[:,0]>x0)&(Q[:,0]<x1)&(Q[:,1]>y0)&(Q[:,1]<y1)&(Q[:,2]<1.1)
S=Q[sel]; ix=((S[:,0]-x0)/res).astype(int); iy=((S[:,1]-y0)/res).astype(int)
for a,b,z in zip(ix,iy,S[:,2]):
    if np.isnan(hm[b,a]) or z>hm[b,a]: hm[b,a]=z
print("x cols from %.3f step 2mm"%x0)
for j in range(0,H,2):
    print("y=%+.3f "%(y0+j*res)+"".join(" ." if np.isnan(v) else ("%2d"%min(99,int(round((v-0.88)*100)))) for v in hm[j]))
EOF
timeout 300 python3 -u step3b.py

# openrua op 35
python3 - <<'EOF'
import re
s=open("ctl.py").read()
s=s.replace('        req.ik_request.group_name = M["planning"]["group"]\n', '        req.ik_request.group_name = M["planning"]["group"]\n        req.ik_request.ik_link_name = "panda_hand"  # default tip is panda_link8 (yawed -45deg)\n')
open("ctl.py","w").write(s)
EOF
grep -n "ik_link_name" ctl.py
cat > rim.py <<'EOF'
import numpy as np, sys
P=np.load(sys.argv[1]); cx0,cy0=float(sys.argv[2]),float(sys.argv[3])
Q=P.reshape(-1,3); Q=Q[np.isfinite(Q).all(1)]
near=np.hypot(Q[:,0]-cx0,Q[:,1]-cy0)<0.075
sel=near&(Q[:,2]>0.975)&(Q[:,2]<1.01)
R=Q[sel]; print("rim pts", len(R), "z range %.4f %.4f"%(R[:,2].min(), R[:,2].max()))
def fit(pts):
    x,y=pts[:,0],pts[:,1]
    A=np.c_[2*x,2*y,np.ones_like(x)]; b=x*x+y*y
    c=np.linalg.lstsq(A,b,rcond=None)[0]
    cx,cy=c[0],c[1]; rad=np.sqrt(c[2]+cx*cx+cy*cy); return cx,cy,rad
pts=R[:,:2]
for it in range(6):
    cx,cy,rad=fit(pts)
    d=np.abs(np.hypot(pts[:,0]-cx,pts[:,1]-cy)-rad)
    pts=pts[d<max(np.percentile(d,85),0.002)]
print("rim circle center (%.4f %.4f) radius %.4f (n=%d)"%(cx,cy,rad,len(pts)))
# outer radius = max radial distance among fitted-in points
rr=np.hypot(R[:,0]-cx,R[:,1]-cy)
print("radial pct 50/90/99:", np.percentile(rr,[50,90,99]).round(4))
# handle direction: points within z band 0.93-0.97 and radial > rad+0.005
sel2=near&(Q[:,2]>0.90)&(Q[:,2]<0.975)
H=Q[sel2]; rh=np.hypot(H[:,0]-cx,H[:,1]-cy); Hh=H[rh>rad+0.003]
if len(Hh): 
    ang=np.degrees(np.arctan2(Hh[:,1].mean()-cy,Hh[:,0].mean()-cx))
    print("handle pts",len(Hh),"mean",Hh.mean(0).round(4),"angle deg",round(ang,1),"max radial %.4f"%rh.max())
res=0.002; x0,x1,y0,y1=cx-0.075,cx+0.075,cy-0.085,cy+0.075
W=int((x1-x0)/res); H2=int((y1-y0)/res); hm=np.full((H2,W),np.nan)
sel=(Q[:,0]>x0)&(Q[:,0]<x1)&(Q[:,1]>y0)&(Q[:,1]<y1)&(Q[:,2]<1.1)
S=Q[sel]; ix=np.clip(((S[:,0]-x0)/res).astype(int),0,W-1); iy=np.clip(((S[:,1]-y0)/res).astype(int),0,H2-1)
for a,b,z in zip(ix,iy,S[:,2]):
    if np.isnan(hm[b,a]) or z>hm[b,a]: hm[b,a]=z
print("x cols from %.3f step 2mm"%x0)
for j in range(0,H2,2):
    print("y=%+.3f "%(y0+j*res)+"".join(" ." if np.isnan(v) else ("%2d"%min(99,int(round((v-0.88)*100)))) for v in hm[j]))
EOF
python3 rim.py eih2_P.npy -0.122 0.044

# openrua op 36
cat > base.py <<'EOF'
import numpy as np
from ctl import *
r=Robot("base")
cx,cy=-0.1222,0.0429
for cam in ["frontview","sideview","agentview"]:
    _,_,P,T=r.snap(cam)
    print(cam, "cam at", T[:3,3].round(3))
    Q=P.reshape(-1,3); Q=Q[np.isfinite(Q).all(1)]
    near=np.hypot(Q[:,0]-cx,Q[:,1]-cy)<0.08
    for lo in np.arange(0.882,0.99,0.008):
        s=Q[near&(Q[:,2]>=lo)&(Q[:,2]<lo+0.008)]
        if len(s)<5: continue
        # radius on +y side, +x side, -x side
        ry=s[:,1].max()-cy; rxp=s[:,0].max()-cx; rxm=cx-s[:,0].min()
        print("  z %.3f n=%3d  r(+y)=%.4f r(+x)=%.4f r(-x)=%.4f"%(lo,len(s),ry,rxp,rxm))
EOF
timeout 300 python3 -u base.py

# openrua op 37
cat > slot.py <<'EOF'
import numpy as np
P=np.load("eih1_P.npy").reshape(-1,3); P=P[np.isfinite(P).all(1)]
# middle column y in [-0.19,-0.09]; profile along x at 2mm: max z and floor presence
sel=P[(P[:,1]>-0.19)&(P[:,1]<-0.09)&(P[:,0]>-0.50)&(P[:,0]<-0.30)]
xs=np.arange(-0.50,-0.30,0.002)
for x in xs:
    s=sel[(sel[:,0]>=x)&(sel[:,0]<x+0.002)]
    if len(s)==0: print("%.3f  -"%x); continue
    z=s[:,2]
    print("%.3f n=%4d zmax=%.3f zmin=%.3f pct90=%.3f  floor(<0.91)=%d"%(x,len(s),z.max(),z.min(),np.percentile(z,90),(z<0.91).sum()))
EOF
python3 slot.py

# openrua op 38
grep -ril "caddy\|compartment" --include=* . 2>/dev/null | grep -v "\.npy\|\.png" | head; ls /workspace; ls /workspace/docs

# openrua op 39
cat > walls.py <<'EOF'
import numpy as np
P=np.load("birdview_cloud.npy").reshape(-1,3); P=P[np.isfinite(P).all(1)]
xs=np.arange(-0.49,-0.31,0.01); ys=np.arange(-0.37,0.08,0.01)
print("maxz (cm above table 0.880), rows=x, cols=y")
print("      "+" ".join("%3d"%round(y*100) for y in ys))
for x in xs:
    row=[]
    for y in ys:
        s=P[(P[:,0]>=x)&(P[:,0]<x+0.01)&(P[:,1]>=y)&(P[:,1]<y+0.01)]
        row.append("%3d"%round((s[:,2].max()-0.88)*100) if len(s) else "  .")
    print("%5.2f "%x+" ".join(row))
EOF
python3 walls.py

# openrua op 40
cat > mugmap.py <<'EOF'
import numpy as np
P=np.load("birdview_cloud.npy").reshape(-1,3); P=P[np.isfinite(P).all(1)]
xs=np.arange(-0.20,-0.04,0.01); ys=np.arange(-0.05,0.12,0.01)
print("      "+" ".join("%3d"%round(y*100) for y in ys))
for x in xs:
    row=[]
    for y in ys:
        s=P[(P[:,0]>=x)&(P[:,0]<x+0.01)&(P[:,1]>=y)&(P[:,1]<y+0.01)]
        row.append("%3d"%round((s[:,2].max()-0.88)*100) if len(s) else "  .")
    print("%5.2f "%x+" ".join(row))
# birdview pixel scale
EOF
python3 mugmap.py

# openrua op 41
(find / -iname "*desk_caddy*" -o -iname "*libero*" 2>/dev/null | grep -v proc | head -20); pip list 2>/dev/null | grep -i -E "libero|robosuite|mujoco" ; ros2 node list 2>/dev/null | head -30

# openrua op 42
ros2 node info /robot_bridge 2>/dev/null | head -60; ros2 topic list | head -60; ros2 service list | grep -v -E "parameter|describe|get_type" | head -40

# openrua op 43
cat > g1.py <<'EOF'
import numpy as np
from ctl import *
r=Robot("g1")
cx,cy=-0.1222,0.0429
xh=cx+0.047; yh=cy
q=topdown_quat(np.pi/2)
print("target quat",q.round(3))
ok=r.move_pose([xh,yh,1.16],q,seconds=4)
p,qq=r.hand_pose(); print("hand pose",p.round(4),qq.round(3)); print("arm q",np.round(r.arm_q(),3))
c,d,P,T=r.snap("robot0_eye_in_hand","/workspace/g1_eih.png")
c2,_,_,_=r.snap("frontview","/workspace/g1_front.png")
EOF
timeout 300 python3 -u g1.py

# openrua op 44
cat > rimcheck.py <<'EOF'
import numpy as np, sys
from ctl import *
r=Robot("rimcheck")
c,d,P,T=r.snap("robot0_eye_in_hand","/workspace/rc_eih.png")
np.save("rc_P.npy",P)
p,qq=r.hand_pose(); print("hand",p.round(4),qq.round(3))
Q=P.reshape(-1,3); Q=Q[np.isfinite(Q).all(1)]
cx0,cy0=float(sys.argv[1]),float(sys.argv[2])
zlo=float(sys.argv[3]) if len(sys.argv)>3 else 0.975
sel=Q[(Q[:,2]>zlo)&(Q[:,2]<zlo+0.035)&(np.hypot(Q[:,0]-cx0,Q[:,1]-cy0)<0.075)]
print("rim pts",len(sel),"z range",sel[:,2].min().round(3),sel[:,2].max().round(3))
cx,cy=cx0,cy0
for it in range(6):
    A=np.c_[2*sel[:,0],2*sel[:,1],np.ones(len(sel))]; b=(sel[:,0]**2+sel[:,1]**2)
    s=np.linalg.lstsq(A,b,rcond=None)[0]; cx,cy=s[0],s[1]; rad=np.sqrt(s[2]+cx**2+cy**2)
    res=np.abs(np.hypot(sel[:,0]-cx,sel[:,1]-cy)-rad); sel=sel[res<max(0.004,np.percentile(res,80))]
print("rim center %.4f %.4f r=%.4f n=%d"%(cx,cy,rad,len(sel)))
rr=np.hypot(sel[:,0]-cx,sel[:,1]-cy); print("radial pct 10/50/90/99",np.percentile(rr,[10,50,90,99]).round(4))
print("+x rim point x=%.4f (r_mid) ; hand x=%.4f ; offset hand-wall=%.4f"%(cx+rad,p[0],p[0]-(cx+rad)))
EOF
timeout 200 python3 -u rimcheck.py -0.1222 0.0429

# openrua op 45
cat > g2.py <<'EOF'
import numpy as np
from ctl import *
r=Robot("g2")
xh,yh=-0.0740,0.0448
q=topdown_quat(np.pi/2)
for z in (1.11,1.07):
    r.move_pose([xh,yh,z],q,seconds=3)
p,qq=r.hand_pose(); print("hand",p.round(4),qq.round(3)); print("fingers",r.finger())
r.snap("robot0_eye_in_hand","/workspace/g2_eih.png"); r.snap("frontview","/workspace/g2_front.png")
EOF
timeout 300 python3 -u g2.py

# openrua op 46
cat > g3.py <<'EOF'
import numpy as np
from ctl import *
r=Robot("g3")
f=r.gripper(0.0)
r.spin(10); print("fingers after",r.finger())
EOF
timeout 300 python3 -u g3.py

# openrua op 47
cat > g4.py <<'EOF'
import numpy as np
from ctl import *
r=Robot("g4")
xh,yh=-0.0740,0.0448
q=topdown_quat(np.pi/2)
r.move_pose([xh,yh,1.20],q,seconds=3)
print("fingers",r.finger())
c,d,P,T=r.snap("frontview","/workspace/g4_front.png")
Q=P.reshape(-1,3); Q=Q[np.isfinite(Q).all(1)]
# points near cup xy above table: find cup body
sel=Q[(np.hypot(Q[:,0]+0.1207,Q[:,1]-0.0448)<0.08)&(Q[:,2]>0.885)&(Q[:,2]<1.2)]
print("pts near cup column: n=%d z range %.3f-%.3f"%(len(sel),sel[:,2].min(),sel[:,2].max()))
low=Q[(np.hypot(Q[:,0]+0.1207,Q[:,1]-0.0448)<0.06)&(Q[:,2]>0.885)&(Q[:,2]<0.95)]
print("pts still low (0.885-0.95):",len(low))
EOF
timeout 300 python3 -u g4.py

# openrua op 48
cat > g5.py <<'EOF'
import numpy as np, sys
from ctl import *
r=Robot("g5")
execute = "--go" in sys.argv
q90=topdown_quat(np.pi/2); q0=topdown_quat(0.0)
wps=[([-0.0738,0.045,1.32],q90),
     ([-0.0738,0.045,1.32],q0),
     ([-0.25,-0.05,1.32],q0),
     ([-0.444,-0.1366,1.32],q0)]
seed=r.arm_q()
sols=[]
for pos,q in wps:
    s=r.ik_solve(pos,q,seed)
    if s is None: print("IK FAIL",pos); sys.exit(1)
    d=np.abs(np.array(s)-np.array(seed)); print(pos, "max joint delta %.2f"%d.max(), np.round(s,2))
    sols.append(s); seed=s
if execute:
    for (pos,q),s in zip(wps,sols):
        r.move_joints(s,seconds=4)
        p,qq=r.hand_pose(); print("  hand",p.round(4),qq.round(3),"fingers",np.round(r.finger(),4))
    r.snap("frontview","/workspace/g5_front.png"); r.snap("agentview","/workspace/g5_agent.png")
EOF
timeout 300 python3 -u g5.py

# openrua op 49
timeout 400 python3 -u g5.py --go

# openrua op 50
cat > tilt.py <<'EOF'
import numpy as np
from ctl import *
r=Robot("tilt")
p,qq=r.hand_pose(); print("hand",p.round(4))
for cam in ["frontview","sideview","agentview"]:
    c,d,P,T=r.snap(cam,f"/workspace/tilt_{cam}.png")
    Q=P.reshape(-1,3); Q=Q[np.isfinite(Q).all(1)]
    # mug region: within 12 cm of expected mug center (-0.444,-0.090), z 1.05..1.25
    sel=Q[(np.hypot(Q[:,0]+0.444,Q[:,1]+0.090)<0.12)&(Q[:,2]>1.07)&(Q[:,2]<1.26)]
    print(cam,"n",len(sel))
    if len(sel)==0: continue
    print("  x %.3f..%.3f y %.3f..%.3f z %.3f..%.3f"%(sel[:,0].min(),sel[:,0].max(),sel[:,1].min(),sel[:,1].max(),sel[:,2].min(),sel[:,2].max()))
    # lowest points location
    low=sel[sel[:,2]<sel[:,2].min()+0.01]; print("  lowest pts mean xy",low[:,:2].mean(0).round(3),"n",len(low))
    # per z slice: x/y extents
    for lo in np.arange(1.07,1.26,0.02):
        s=sel[(sel[:,2]>=lo)&(sel[:,2]<lo+0.02)]
        if len(s)>3: print("  z %.2f n=%4d x %.3f..%.3f  y %.3f..%.3f"%(lo,len(s),s[:,0].min(),s[:,0].max(),s[:,1].min(),s[:,1].max()))
EOF
timeout 300 python3 -u tilt.py

# openrua op 51
cat > g6.py <<'EOF'
import numpy as np, sys
from ctl import *
from geometry_msgs.msg import WrenchStamped
r=Robot("g6")
w={}
r.node.create_subscription(WrenchStamped,"/franka_robot_state_broadcaster/external_wrench",lambda m: w.__setitem__("m",m),1)
def wrench():
    w.clear()
    while "m" not in w: rclpy.spin_once(r.node,timeout_sec=0.2)
    f=w["m"].wrench.force; t=w["m"].wrench.torque
    return np.array([f.x,f.y,f.z]).round(2), np.array([t.x,t.y,t.z]).round(2)
q0=topdown_quat(0.0)
xh,yh=-0.468,-0.1276
print("wrench before",wrench())
for z in (1.30,1.26,1.2445,1.239):
    r.move_pose([xh,yh,z],q0,seconds=3)
    print("  z",z,"fingers",np.round(r.finger(),4),"wrench",wrench())
c,d,P,T=r.snap("sideview","/workspace/g6_side.png")
Q=P.reshape(-1,3); Q=Q[np.isfinite(Q).all(1)]
sel=Q[(np.hypot(Q[:,0]+0.444,Q[:,1]+0.090)<0.12)&(Q[:,2]>1.02)&(Q[:,2]<1.2)]
for lo in np.arange(1.04,1.2,0.02):
    s=sel[(sel[:,2]>=lo)&(sel[:,2]<lo+0.02)]
    if len(s)>3: print("  z %.2f n=%4d x %.3f..%.3f  y %.3f..%.3f"%(lo,len(s),s[:,0].min(),s[:,0].max(),s[:,1].min(),s[:,1].max()))
EOF
timeout 400 python3 -u g6.py

# openrua op 52
cat > g7.py <<'EOF'
import numpy as np, sys
from ctl import *
r=Robot("g7")
q0=topdown_quat(0.0)
f=r.gripper(0.04)
p,qq=r.hand_pose(); print("hand",p.round(4))
r.move_pose([p[0],p[1],1.36],q0,seconds=3)
for cam in ["agentview","sideview","frontview","birdview"]:
    c,d,P,T=r.snap(cam,f"/workspace/g7_{cam}.png"); np.save(f"g7_{cam}_P.npy",P)
EOF
timeout 400 python3 -u g7.py

# openrua op 53
cat > find.py <<'EOF'
import numpy as np, cv2
for cam in ["sideview","frontview","birdview","agentview"]:
    P=np.load(f"g7_{cam}_P.npy"); img=cv2.imread(f"g7_{cam}.png")
    H,W,_=P.shape
    # yellow-ish pixels: B low, R,G high
    b,g,r=img[...,0].astype(int),img[...,1].astype(int),img[...,2].astype(int)
    yel=(r>150)&(g>130)&(b<110)&(r-b>80)
    Q=P[yel]; Q=Q[np.isfinite(Q).all(1)]
    # exclude table-colored? print clusters by rounding
    print(cam,"yellow px",yel.sum())
    if len(Q):
        # remove points at table level far away (wood is brownish, may match). bin xy at 2cm
        keys=np.round(Q[:,:2]/0.02).astype(int)
        u,cnt=np.unique(keys,axis=0,return_counts=True)
        idx=np.argsort(-cnt)[:8]
        for i in idx:
            m=(keys==u[i]).all(1)
            print("   xy %.2f %.2f n=%d z %.3f..%.3f"%(u[i][0]*0.02,u[i][1]*0.02,cnt[i],Q[m][:,2].min(),Q[m][:,2].max()))
EOF
python3 find.py

# openrua op 54
cat > hmall.py <<'EOF'
import numpy as np, sys
cams=sys.argv[1:] or ["birdview"]
P=np.concatenate([np.load(f"g7_{c}_P.npy").reshape(-1,3) for c in cams]); P=P[np.isfinite(P).all(1)]
xs=np.arange(-0.72,-0.24,0.02); ys=np.arange(-0.42,0.16,0.02)
print("      "+" ".join("%3d"%round(y*100) for y in ys))
for x in xs:
    row=[]
    for y in ys:
        s=P[(P[:,0]>=x)&(P[:,0]<x+0.02)&(P[:,1]>=y)&(P[:,1]<y+0.02)]
        row.append("%3d"%round((s[:,2].max()-0.88)*100) if len(s) else "  .")
    print("%5.2f "%x+" ".join(row))
EOF
python3 hmall.py birdview; python3 hmall.py sideview frontview

# openrua op 55
cat > g8.py <<'EOF'
import numpy as np, sys
from ctl import *
r=Robot("g8")
q0=topdown_quat(0.0)
r.move_pose([-0.30,0.15,1.35],q0,seconds=4)
for cam in ["birdview","sideview","robot0_robotview","frontview"]:
    c,d,P,T=r.snap(cam,f"/workspace/g8_{cam}.png"); np.save(f"g8_{cam}_P.npy",P)
P=np.concatenate([np.load(f"g8_{c}_P.npy").reshape(-1,3) for c in ["birdview","sideview"]]); P=P[np.isfinite(P).all(1)]
sel=P[(P[:,0]>-0.70)&(P[:,0]<-0.47)&(P[:,1]>-0.30)&(P[:,1]<0.05)&(P[:,2]>0.89)&(P[:,2]<1.1)]
print("mug candidate pts",len(sel),"x %.3f..%.3f y %.3f..%.3f z %.3f..%.3f"%(sel[:,0].min(),sel[:,0].max(),sel[:,1].min(),sel[:,1].max(),sel[:,2].min(),sel[:,2].max()))
xs=np.arange(-0.68,-0.48,0.01); ys=np.arange(-0.22,0.02,0.01)
print("      "+" ".join("%3d"%round(y*100) for y in ys))
for x in xs:
    row=[]
    for y in ys:
        s=P[(P[:,0]>=x)&(P[:,0]<x+0.01)&(P[:,1]>=y)&(P[:,1]<y+0.01)]
        row.append("%3d"%round((s[:,2].max()-0.88)*100) if len(s) else "  .")
    print("%5.2f "%x+" ".join(row))
EOF
timeout 400 python3 -u g8.py

# openrua op 56
cat > jam.py <<'EOF'
import numpy as np
# slot mouth: back wall inner face x=-0.456 (top z=1.056); divider back face x=-0.402 (top z=0.988); floor 0.90
rb,rt,H=0.033,0.050,0.105
def r(s): return rb+(rt-rb)*s/H
S=np.linspace(0,H,106)
best=None
for th in np.radians(np.arange(0,71,5)):
    u=np.array([np.sin(th),np.cos(th)]); v=np.array([np.cos(th),-np.sin(th)])
    # find max depth: minimize zb over xb s.t. constraints
    res=[]
    for xb in np.arange(-0.50,-0.36,0.001):
        # constraint: for each s, the -x extreme point (xm,zm) and +x extreme (xp,zp) and base disc edge points
        lo,hi=0.85,1.20
        # binary search zb: feasible if constraints hold
        def ok(zb):
            for s in S:
                c=np.array([xb,zb])+s*u
                pm=c-r(s)*v; pp=c+r(s)*v
                if pm[1]<1.056 and pm[0]<-0.456: return False
                if pp[1]<0.988 and pp[0]>-0.402: return False
                if pm[1]<0.90 or pp[1]<0.90: return False
                # also the -x point must not be inside the back wall below top; and the +x point inside divider (x>-0.402 and z<0.988) handled
                # also points over the divider top: if x in [-0.402,-0.386] z must be >0.988
                if -0.402<=pp[0]<=-0.386 and pp[1]<0.988: return False
            return True
        if not ok(hi): continue
        for _ in range(25):
            mid=(lo+hi)/2
            if ok(mid): hi=mid
            else: lo=mid
        res.append((hi,xb))
    if res:
        zb,xb=min(res)
        com=np.array([xb,zb])+0.05*u
        print("tilt %2d deg: base center z=%.3f x=%.3f  COM x=%.3f z=%.3f"%(np.degrees(th),zb,xb,com[0],com[1]))
EOF
python3 jam.py

# openrua op 57
cat > mugpose.py <<'EOF'
import numpy as np
P=np.concatenate([np.load(f"g8_{c}_P.npy").reshape(-1,3) for c in ["birdview","sideview","frontview","robot0_robotview"]]); P=P[np.isfinite(P).all(1)]
sel=P[(P[:,0]>-0.63)&(P[:,0]<-0.48)&(P[:,1]>-0.20)&(P[:,1]<-0.03)&(P[:,2]>0.885)&(P[:,2]<1.1)]
print("n",len(sel))
for x in np.arange(-0.63,-0.48,0.01):
    s=sel[(sel[:,0]>=x)&(sel[:,0]<x+0.01)]
    if len(s)<5: continue
    print("x %.2f n=%4d y %.3f..%.3f z %.3f..%.3f"%(x,len(s),s[:,1].min(),s[:,1].max(),s[:,2].min(),s[:,2].max()))
print("z slices:")
for z in np.arange(0.89,1.03,0.01):
    s=sel[(sel[:,2]>=z)&(sel[:,2]<z+0.01)]
    if len(s)<5: continue
    print("z %.2f n=%4d x %.3f..%.3f y %.3f..%.3f"%(z,len(s),s[:,0].min(),s[:,0].max(),s[:,1].min(),s[:,1].max()))
EOF
python3 mugpose.py

# openrua op 58
cat > s1.py <<'EOF'
import numpy as np, sys
from ctl import *
r=Robot("s1")
go="--go" in sys.argv
# hand pointing -y (approach from +y), fingers along z, hand x = +x world
R=np.array([[1,0,0],[0,0,-1],[0,1,0]],float)  # columns: x=(1,0,0), y=(0,0,1), z=(0,-1,0)
q=R_quat(R); print("quat",q.round(4))
ax_y=-0.1025; ax_z=0.978; gx=-0.508
pre=[gx, 0.09, ax_z]; grasp=[gx, ax_y+0.1034-0.010, ax_z]
seed=r.arm_q()
for name,pos in (("pre",pre),("grasp",grasp)):
    s=r.ik_solve(pos,q,seed)
    print(name,pos,"IK", None if s is None else np.round(s,2))
    if s is None: sys.exit(1)
    print("   max delta from seed %.2f"%np.abs(np.array(s)-np.array(seed)).max()); seed=s
if go:
    r.move_pose(pre,q,seconds=5)
    p,qq=r.hand_pose(); print("hand",p.round(4),qq.round(3))
    r.snap("robot0_eye_in_hand","/workspace/s1_eih.png"); r.snap("sideview","/workspace/s1_side.png")
EOF
timeout 300 python3 -u s1.py

# openrua op 59
cat > s1.py <<'EOF'
import numpy as np, sys
from ctl import *
r=Robot("s1")
go="--go" in sys.argv
R=np.array([[1,0,0],[0,0,-1],[0,1,0]],float)
q=R_quat(R)
ax_y=-0.1025; ax_z=0.978; gx=-0.508
wps=[([gx,0.09,1.25],q),([gx,0.09,1.10],q),([gx,0.09,ax_z],q)]
seed=r.arm_q()
sols=[]
for pos,qq in wps:
    s=r.ik_solve(pos,qq,seed)
    if s is None: print("IK fail",pos); sys.exit(1)
    print(pos,"delta %.2f"%np.abs(np.array(s)-np.array(seed)).max(),np.round(s,2)); seed=s; sols.append(s)
if go:
    for (pos,qq),s in zip(wps,sols):
        r.move_joints(s,seconds=4); p,q2=r.hand_pose(); print("  hand",p.round(4),q2.round(3))
    r.snap("robot0_eye_in_hand","/workspace/s1_eih.png"); r.snap("sideview","/workspace/s1_side.png")
EOF
timeout 300 python3 -u s1.py --go

# openrua op 60
python3 -c "
import numpy as np
P=np.load('g8_birdview_P.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,2]>1.2)&(np.abs(P[:,0]+0.30)<0.15)&(np.abs(P[:,1]-0.15)<0.2)
Q=P[m]; print(len(Q))
for z0 in np.arange(1.2,1.5,0.02):
    s=Q[(Q[:,2]>=z0)&(Q[:,2]<z0+0.02)]
    if len(s): print('%.2f n=%4d x %.3f..%.3f y %.3f..%.3f'%(z0,len(s),s[:,0].min(),s[:,0].max(),s[:,1].min(),s[:,1].max()))
"

# openrua op 61
timeout 20 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | grep -m1 -A1 Translation

# openrua op 62
cat > s2.py <<'EOF'
import numpy as np, sys
from ctl import *
r=Robot("s2")
go="--go" in sys.argv
q0=topdown_quat(0.0)
gx,gy=-0.505,-0.1025
back=[([-0.508,0.09,1.25],R_quat(np.array([[1,0,0],[0,0,-1],[0,1,0]],float)))]
wps=back+[([gx,gy,1.30],q0),([gx,gy,1.15],q0),([gx,gy,1.10],q0),([gx,gy,1.081],q0)]
seed=r.arm_q(); sols=[]
for pos,qq in wps:
    s=r.ik_solve(pos,qq,seed)
    if s is None: print("IK fail",pos); sys.exit(1)
    print(pos,"delta %.2f"%np.abs(np.array(s)-np.array(seed)).max(),np.round(s,2)); seed=s; sols.append(s)
if go:
    for i,((pos,qq),s) in enumerate(zip(wps,sols)):
        r.move_joints(s,seconds=4); p,q2=r.hand_pose(); print("  hand",p.round(4),q2.round(3))
        if i==2: r.snap("robot0_eye_in_hand","/workspace/s2_eih115.png")
    r.snap("robot0_eye_in_hand","/workspace/s2_eih.png"); r.snap("sideview","/workspace/s2_side.png")
EOF
timeout 300 python3 -u s2.py

# openrua op 63
cat > iksamp.py <<'EOF'
import numpy as np, sys
from ctl import *
r=Robot("iks")
lim=np.array(FJT["limits_rad"])
pos=[float(v) for v in sys.argv[1:4]]; yaw=float(sys.argv[4]) if len(sys.argv)>4 else 0.0
q=topdown_quat(yaw)
rng=np.random.default_rng(0); best=[]
for i in range(40):
    seed=rng.uniform(lim[:,0],lim[:,1]) if i else r.arm_q()
    s=r.ik_solve(pos,q,seed)
    if s is None: continue
    s=np.array(s); marg=np.minimum(s-lim[:,0],lim[:,1]-s).min()
    best.append((marg,s))
best.sort(key=lambda t:-t[0])
for m,s in best[:8]: print("margin %.2f"%m,np.round(s,2))
EOF
timeout 300 python3 -u iksamp.py -0.505 -0.1025 1.081 0

# openrua op 64
python3 - <<'EOF'
import re
s=open('ctl.py').read()
s=s.replace("def ik_solve(self, pos_world, quat, seed=None):","def ik_solve(self, pos_world, quat, seed=None, collide=False):")
s=s.replace("req.ik_request.avoid_collisions = False","req.ik_request.avoid_collisions = bool(collide)")
open('ctl.py','w').write(s)
EOF
cat > iksamp.py <<'EOF'
import numpy as np, sys
from ctl import *
r=Robot("iks")
lim=np.array(FJT["limits_rad"])
pos=[float(v) for v in sys.argv[1:4]]; yaw=float(sys.argv[4]) if len(sys.argv)>4 else 0.0
q=topdown_quat(yaw)
rng=np.random.default_rng(0); best=[]
for i in range(40):
    seed=rng.uniform(lim[:,0],lim[:,1]) if i else r.arm_q()
    s=r.ik_solve(pos,q,seed,collide=True)
    if s is None: continue
    s=np.array(s); marg=np.minimum(s-lim[:,0],lim[:,1]-s).min()
    best.append((marg,s))
best.sort(key=lambda t:-t[0])
print(len(best),"solutions")
for m,s in best[:8]: print("margin %.2f"%m,np.round(s,2))
EOF
timeout 300 python3 -u iksamp.py -0.505 -0.1025 1.081 0

# openrua op 65
timeout 120 python3 -c "
import numpy as np
from ctl import *
r=Robot('t')
for seed in ([0.59,-1.69,-1.87,-3.0,-1.88,1.5,1.24],[0.3,0.33,-0.89,-2.67,-2.34,3.5,2.59],[-1.03,0.32,0.82,-2.66,2.41,3.49,-1.89]):
    s=r.ik_solve([-0.505,-0.1025,1.081],topdown_quat(0),seed,collide=True)
    print(seed,'->',None if s is None else np.round(s,2))
"

# openrua op 66
cat > s2.py <<'EOF'
import numpy as np, sys
from ctl import *
r=Robot("s2")
go="--go" in sys.argv
q0=topdown_quat(0.0)
gx,gy=-0.505,-0.1025
qside=R_quat(np.array([[1,0,0],[0,0,-1],[0,1,0]],float))
# backward chain from the grasp config
final=[0.29,0.33,-0.89,-2.67,-2.34,3.5,2.59]
zs=[1.081,1.10,1.15,1.30]
sols={}; seed=final
for z in zs:
    s=r.ik_solve([gx,gy,z],q0,seed,collide=True)
    if s is None: print("IK fail",z); sys.exit(1)
    print(z,np.round(s,2),"delta %.2f"%np.abs(np.array(s)-np.array(seed)).max()); sols[z]=s; seed=s
back=r.ik_solve([-0.508,0.09,1.25],qside,r.arm_q(),collide=True); print("back",np.round(back,2))
print("transition delta",np.round(np.array(sols[1.30])-np.array(back),2))
plan=[("back",back)]+[(z,sols[z]) for z in [1.30,1.15,1.10,1.081]]
if go:
    for i,(lab,s) in enumerate(plan):
        r.move_joints(s,seconds=5 if i<2 else 4); p,q2=r.hand_pose(); print(lab,"hand",p.round(4),q2.round(3))
        if lab==1.15: r.snap("robot0_eye_in_hand","/workspace/s2_eih115.png"); r.snap("sideview","/workspace/s2_side115.png")
    r.snap("robot0_eye_in_hand","/workspace/s2_eih.png"); r.snap("sideview","/workspace/s2_side.png")
EOF
timeout 300 python3 -u s2.py

# openrua op 67
cat > iktilt.py <<'EOF'
import numpy as np, sys
from ctl import *
r=Robot("ikt")
lim=np.array(FJT["limits_rad"])
tip=np.array([-0.512,-0.1025,0.978])
def pose(th):
    zh=np.array([-np.sin(th),0,-np.cos(th)]); yh=np.array([0,-1,0.]); xh=np.cross(yh,zh)
    R=np.stack([xh,yh,zh],1); return tip-0.1034*zh, R_quat(R)
rng=np.random.default_rng(1)
for deg in [-30,-15,0,15]:
    pos,q=pose(np.radians(deg)); best=[]
    for i in range(30):
        seed=rng.uniform(lim[:,0],lim[:,1])
        s=r.ik_solve(pos,q,seed,collide=True)
        if s is None: continue
        s=np.array(s); marg=np.minimum(s-lim[:,0],lim[:,1]-s).min(); best.append((marg,s))
    best.sort(key=lambda t:-t[0])
    print("tilt",deg,"origin",pos.round(3),"nsol",len(best))
    for m,s in best[:3]: print("   margin %.2f"%m,np.round(s,2))
EOF
timeout 600 python3 -u iktilt.py

# openrua op 68
timeout 30 ros2 service call /get_planning_scene moveit_msgs/srv/GetPlanningScene "{components: {components: 12}}" 2>&1 | grep -E "id:|type|primitive_poses|dimensions|frame_id" | head -40; echo "---"; timeout 10 ros2 action list 2>&1 | head; timeout 10 ros2 service list | grep -i -E "scene|plan" 

# openrua op 69
cat > mp.py <<'EOF'
"""MoveIt planning helpers: collision objects + plan_kinematic_path + execute via FJT."""
import numpy as np, rclpy
from ctl import *
from moveit_msgs.srv import GetMotionPlan, ApplyPlanningScene
from moveit_msgs.msg import CollisionObject, PlanningScene, Constraints, JointConstraint, PositionConstraint, OrientationConstraint, BoundingVolume
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose, PoseStamped
from control_msgs.action import FollowJointTrajectory

def box(name, lo, hi):
    lo=np.array(lo,float); hi=np.array(hi,float)
    co=CollisionObject(); co.header.frame_id="world"; co.id=name
    sp=SolidPrimitive(); sp.type=SolidPrimitive.BOX; sp.dimensions=list(map(float,hi-lo))
    p=Pose(); p.position.x,p.position.y,p.position.z=map(float,(lo+hi)/2); p.orientation.w=1.0
    co.primitives=[sp]; co.primitive_poses=[p]; co.operation=CollisionObject.ADD
    return co

class Planner:
    def __init__(self, r):
        self.r=r; n=r.node
        self.aps=n.create_client(ApplyPlanningScene,"/apply_planning_scene"); self.aps.wait_for_service(10)
        self.gmp=n.create_client(GetMotionPlan,"/plan_kinematic_path"); self.gmp.wait_for_service(10)
    def scene(self, objs, remove=()):
        ps=PlanningScene(); ps.is_diff=True
        for o in objs: ps.world.collision_objects.append(o)
        for name in remove:
            co=CollisionObject(); co.header.frame_id="world"; co.id=name; co.operation=CollisionObject.REMOVE
            ps.world.collision_objects.append(co)
        req=ApplyPlanningScene.Request(); req.scene=ps
        f=self.aps.call_async(req); rclpy.spin_until_future_complete(self.r.node,f,timeout_sec=30)
        return f.result().success
    def _req(self, t=5.0, attempts=10):
        req=GetMotionPlan.Request(); mr=req.motion_plan_request
        mr.group_name=M["planning"]["group"]; mr.allowed_planning_time=t; mr.num_planning_attempts=attempts
        mr.max_velocity_scaling_factor=0.3; mr.max_acceleration_scaling_factor=0.3
        mr.start_state.joint_state.name=list(ARM); mr.start_state.joint_state.position=list(map(float,self.r.arm_q()))
        mr.start_state.is_diff=True
        return req
    def plan_joints(self, q, **kw):
        req=self._req(**kw); c=Constraints()
        for n_,v in zip(ARM,q):
            jc=JointConstraint(); jc.joint_name=n_; jc.position=float(v); jc.tolerance_above=0.01; jc.tolerance_below=0.01; jc.weight=1.0
            c.joint_constraints.append(jc)
        req.motion_plan_request.goal_constraints=[c]; return self._call(req)
    def plan_pose(self, pos, quat, **kw):
        req=self._req(**kw); c=Constraints()
        pc=PositionConstraint(); pc.header.frame_id="world"; pc.link_name="panda_hand"; pc.weight=1.0
        sp=SolidPrimitive(); sp.type=SolidPrimitive.SPHERE; sp.dimensions=[0.005]
        p=Pose(); p.position.x,p.position.y,p.position.z=map(float,pos); p.orientation.w=1.0
        pc.constraint_region.primitives=[sp]; pc.constraint_region.primitive_poses=[p]
        oc=OrientationConstraint(); oc.header.frame_id="world"; oc.link_name="panda_hand"; oc.weight=1.0
        oc.orientation.x,oc.orientation.y,oc.orientation.z,oc.orientation.w=map(float,quat)
        oc.absolute_x_axis_tolerance=oc.absolute_y_axis_tolerance=oc.absolute_z_axis_tolerance=0.02
        c.position_constraints=[pc]; c.orientation_constraints=[oc]
        req.motion_plan_request.goal_constraints=[c]; return self._call(req)
    def _call(self, req):
        f=self.gmp.call_async(req); rclpy.spin_until_future_complete(self.r.node,f,timeout_sec=120)
        res=f.result().motion_plan_response
        if res.error_code.val!=1: print("  plan failed code",res.error_code.val); return None
        tr=res.trajectory.joint_trajectory
        print(f"  plan ok: {len(tr.points)} pts, {tr.points[-1].time_from_start.sec + tr.points[-1].time_from_start.nanosec*1e-9:.1f}s, final {np.round(tr.points[-1].positions,2)}")
        return tr
    def execute(self, tr, retries=2):
        idx=[tr.joint_names.index(j) for j in ARM]
        goal=FollowJointTrajectory.Goal(); goal.trajectory.joint_names=list(ARM)
        for p in tr.points:
            pt=JointTrajectoryPoint(positions=[p.positions[i] for i in idx],velocities=[p.velocities[i] for i in idx] if p.velocities else [])
            pt.time_from_start=p.time_from_start; goal.trajectory.points.append(pt)
        send=self.r.fjt.send_goal_async(goal); rclpy.spin_until_future_complete(self.r.node,send)
        res=send.result().get_result_async(); rclpy.spin_until_future_complete(self.r.node,res)
        code=res.result().result.error_code
        target=[tr.points[-1].positions[i] for i in idx]
        err=np.abs(np.array(self.r.arm_q())-np.array(target)).max(); print(f"  exec: code={code} err={err:.4f}")
        if err>0.02: self.r.move_joints(target,seconds=3,retries=retries)
        return True

def scene_objects(with_mug=True):
    objs=[box("table",[-1.4,-1.0,0.80],[0.6,1.0,0.88]),
          box("caddy",[-0.472,-0.345,0.88],[-0.318,0.06,1.06])]
    if with_mug: objs.append(box("mug",[-0.60,-0.16,0.925],[-0.485,-0.045,1.03]))
    return objs
EOF
cat > s3.py <<'EOF'
import numpy as np, sys
from ctl import *; from mp import *
r=Robot("s3"); pl=Planner(r)
print("scene",pl.scene(scene_objects()))
go="--go" in sys.argv
q0=topdown_quat(0.0)
pre=[-0.512,-0.1025,1.03+0.1034+0.012]
tr=pl.plan_pose(pre,q0,t=10.0)
if tr is None: sys.exit(1)
P=np.array([p.positions for p in tr.points]); print("joint ranges min",P.min(0).round(2)," max",P.max(0).round(2))
if go:
    pl.execute(tr); p,q=r.hand_pose(); print("hand",p.round(4),q.round(3))
    r.snap("robot0_eye_in_hand","/workspace/s3_eih.png"); r.snap("sideview","/workspace/s3_side.png"); r.snap("birdview","/workspace/s3_bird.png")
EOF
timeout 300 python3 -u s3.py

# openrua op 70
timeout 30 ros2 service call /query_planner_interface moveit_msgs/srv/QueryPlannerInterfaces 2>&1 | grep -oE "pipeline_id='[^']*'|planner_ids=\[[^]]*\]" | head; timeout 10 ros2 service list | grep -i valid

# openrua op 71
timeout 30 ros2 service call /query_planner_interface moveit_msgs/srv/QueryPlannerInterfaces 2>&1 | grep -oE "pipeline_id='ompl'.{0,600}" | tr ',' '\n' | grep -i -E "panda_arm|RRT" | head -20

# openrua op 72
timeout 30 ros2 service call /query_planner_interface moveit_msgs/srv/QueryPlannerInterfaces 2>&1 | grep -oE "pipeline_id='ompl'.{0,3000}" | tr ',' '\n' | grep -i "panda_arm\[" | head -5

# openrua op 73
python3 - <<'EOF'
s=open('mp.py').read()
s=s.replace('mr.group_name=M["planning"]["group"]; mr.allowed_planning_time=t; mr.num_planning_attempts=attempts',
 'mr.group_name=M["planning"]["group"]; mr.allowed_planning_time=t; mr.num_planning_attempts=attempts\n        mr.pipeline_id="ompl"; mr.planner_id="RRTConnectkConfigDefault"')
open('mp.py','w').write(s)
EOF
timeout 300 python3 -u s3.py

# openrua op 74
timeout 60 python3 -c "
import rclpy
from ctl import *
from moveit_msgs.srv import GetStateValidity
r=Robot('v'); c=r.node.create_client(GetStateValidity,'/check_state_validity'); c.wait_for_service(10)
req=GetStateValidity.Request(); req.group_name='panda_arm'
req.robot_state.joint_state.name=list(ARM); req.robot_state.joint_state.position=list(map(float,r.arm_q()))
f=c.call_async(req); rclpy.spin_until_future_complete(r.node,f,timeout_sec=30); res=f.result()
print('valid',res.valid); [print(' contact',x.contact_body_1,x.contact_body_2) for x in res.contacts]
"; timeout 30 ros2 node list | head; 

# openrua op 75
timeout 300 python3 -u -c "
import numpy as np
from ctl import *; from mp import *
r=Robot('t'); pl=Planner(r)
tr=pl.plan_joints([0.29,-0.4,0.5,-2.82,-0.86,1.24,1.73],t=5.0)
qs=R_quat(np.array([[1,0,0],[0,0,-1],[0,1,0]],float))
tr=pl.plan_pose([-0.508,0.09,1.25],qs,t=5.0)
"

# openrua op 76
timeout 600 python3 -u -c "
import numpy as np
from ctl import *; from mp import *
r=Robot('t'); pl=Planner(r); lim=np.array(FJT['limits_rad']); rng=np.random.default_rng(3)
q0=topdown_quat(0)
for z in [1.145,1.12,1.10]:
    n=0; sols=[]
    for i in range(30):
        s=r.ik_solve([-0.512,-0.1025,z],q0,rng.uniform(lim[:,0],lim[:,1]),collide=True)
        if s is not None: n+=1; sols.append(np.round(s,2))
    print('z',z,'valid IK',n, sols[:2])
print('remove mug', pl.scene([],remove=['mug']))
tr=pl.plan_joints([0.29,0.33,-0.89,-2.67,-2.34,3.5,2.59],t=10.0)
if tr is not None:
    P=np.array([p.positions for p in tr.points]); print('min',P.min(0).round(2),'max',P.max(0).round(2))
"

# openrua op 77
python3 - <<'EOF'
s=open('mp.py').read()
s=s.replace('''    if with_mug: objs.append(box("mug",[-0.60,-0.16,0.925],[-0.485,-0.045,1.03]))''',
'''    if with_mug:
        objs += [box("mug_body",[-0.575,-0.139,0.942],[-0.485,-0.066,1.014]),
                 box("mug_rim",[-0.60,-0.16,0.925],[-0.575,-0.045,1.03]),
                 box("mug_handle",[-0.572,-0.12,0.88],[-0.521,-0.085,0.945])]''')
open('mp.py','w').write(s)
EOF
cat > s3.py <<'EOF'
import numpy as np, sys
from ctl import *; from mp import *
r=Robot("s3"); pl=Planner(r)
pl.scene([],remove=["mug","mug_body","mug_rim","mug_handle"]); print("scene",pl.scene(scene_objects()))
go="--go" in sys.argv
A=[0.29,0.33,-0.89,-2.67,-2.34,3.5,2.59]
tr=pl.plan_joints(A,t=15.0)
if tr is None: sys.exit(1)
P=np.array([p.positions for p in tr.points]); print("joint ranges min",P.min(0).round(2)," max",P.max(0).round(2))
# FK sweep of hand + fingertip
from moveit_msgs.srv import GetPositionFK
def fk(q):
    req=GetPositionFK.Request(); req.header.frame_id=""; req.fk_link_names=["panda_hand"]
    req.robot_state.joint_state.name=list(ARM); req.robot_state.joint_state.position=list(map(float,q))
    f=r.fk.call_async(req); rclpy.spin_until_future_complete(r.node,f,timeout_sec=30); p=f.result().pose_stamped[0].pose
    R=quat_R(p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w); o=np.array([p.position.x,p.position.y,p.position.z])
    return o, o+0.1034*R[:,2]
minz=9; 
for i in range(0,len(P),max(1,len(P)//20)):
    o,t=fk(P[i]); minz=min(minz,t[2]); print(i,"hand",o.round(3),"tip",t.round(3))
o,t=fk(P[-1]); print("final hand",o.round(4),"tip",t.round(4))
if go:
    pl.execute(tr); p,q=r.hand_pose(); print("hand",p.round(4),q.round(3),"fingers",r.finger())
    r.snap("robot0_eye_in_hand","/workspace/s3_eih.png"); r.snap("sideview","/workspace/s3_side.png"); r.snap("birdview","/workspace/s3_bird.png")
EOF
timeout 300 python3 -u s3.py

# openrua op 78
cat > valid.py <<'EOF'
import sys, numpy as np, rclpy
from ctl import *
from moveit_msgs.srv import GetStateValidity
def check(r, q, c=None):
    c = c or r.node.create_client(GetStateValidity,'/check_state_validity'); c.wait_for_service(10)
    req=GetStateValidity.Request(); req.group_name='panda_arm'
    req.robot_state.joint_state.name=list(ARM)+["panda_finger_joint1","panda_finger_joint2"]; req.robot_state.joint_state.position=list(map(float,q))+[0.04,0.04]
    f=c.call_async(req); rclpy.spin_until_future_complete(r.node,f,timeout_sec=30); res=f.result()
    return res.valid, [(x.contact_body_1,x.contact_body_2,round(x.depth,4)) for x in res.contacts]
if __name__=="__main__":
    r=Robot('v'); q=[float(v) for v in sys.argv[1:8]]; print(check(r,q))
EOF
timeout 60 python3 valid.py 0.29 0.33 -0.89 -2.67 -2.34 3.5 2.59

# openrua op 79
timeout 60 python3 -c "
from valid import *
r=Robot('v'); c=r.node.create_client(GetStateValidity,'/check_state_validity'); c.wait_for_service(10)
q=[0.29,0.33,-0.89,-2.67,-2.34,3.5,2.59]
for f2 in (0.04,-0.04,0.0):
    req=GetStateValidity.Request(); req.group_name='panda_arm'
    req.robot_state.joint_state.name=list(ARM)+['panda_finger_joint1','panda_finger_joint2']; req.robot_state.joint_state.position=q+[0.04,f2]
    f=c.call_async(req); rclpy.spin_until_future_complete(r.node,f,timeout_sec=30); res=f.result()
    print(f2, res.valid, [(x.contact_body_1,x.contact_body_2,round(x.depth,4)) for x in res.contacts])
"

# openrua op 80
python3 - <<'EOF'
s=open('mp.py').read()
s=s.replace('''mr.start_state.joint_state.name=list(ARM); mr.start_state.joint_state.position=list(map(float,self.r.arm_q()))''',
'''f1,f2=self.r.finger()  # bridge reports finger2 negative; MoveIt wants both >=0
        mr.start_state.joint_state.name=list(ARM)+["panda_finger_joint1","panda_finger_joint2"]
        mr.start_state.joint_state.position=list(map(float,self.r.arm_q()))+[abs(f1),abs(f2)]''')
open('mp.py','w').write(s)
EOF
timeout 300 python3 -u s3.py

# openrua op 81
timeout 600 python3 -u -c "
import numpy as np
from ctl import *; from mp import *; from valid import check
r=Robot('t'); pl=Planner(r)
print('start valid',check(r,r.arm_q()))
for A in ([0.29,0.33,-0.89,-2.67,-2.34,3.5,2.59],[-1.03,0.32,0.82,-2.66,2.41,3.49,-1.89]):
    print('goal valid',check(r,A))
    tr=pl.plan_joints(A,t=60.0,attempts=4)
    if tr is not None:
        P=np.array([p.positions for p in tr.points]); print('min',P.min(0).round(2),'max',P.max(0).round(2))
"

# openrua op 82
python3 - <<'EOF'
s=open('s3.py').read()
s=s.replace("tr=pl.plan_joints(A,t=15.0)","tr=pl.plan_joints(A,t=90.0,attempts=4)")
open('s3.py','w').write(s)
EOF
timeout 900 python3 -u s3.py --go

# openrua op 83
python3 - <<'EOF'
s=open('s3.py').read()
s=s.replace("tr=pl.plan_joints(A,t=90.0,attempts=4)\nif tr is None: sys.exit(1)",
"""tr=None
for attempt in range(6):
    tr=pl.plan_joints(A,t=60.0,attempts=4)
    if tr is not None: break
if tr is None: sys.exit(1)""")
open('s3.py','w').write(s)
EOF
timeout 1500 python3 -u s3.py --go

# openrua op 84
timeout 900 python3 -u -c "
import numpy as np, time
from ctl import *; from mp import *; from valid import check
r=Robot('t'); pl=Planner(r)
A=[0.29,0.33,-0.89,-2.67,-2.34,3.5,2.59]
print('start valid',check(r,r.arm_q()),'goal valid',check(r,A), 'fingers',r.finger())
for i in range(3):
    t0=time.time(); tr=pl.plan_joints(A,t=60.0,attempts=4); print('took %.1f'%(time.time()-t0))
    if tr: break
"

# openrua op 85
timeout 900 python3 -u -c "
import numpy as np, time
from ctl import *; from mp import *; from valid import check
r=Robot('t'); pl=Planner(r)
B=[0.44,0.3,-1.02,-2.66,-2.45,3.55,2.72]
print('B valid',check(r,B))
p,q=None,None
for i in range(2):
    t0=time.time(); tr=pl.plan_joints(B,t=60.0,attempts=4); print('took %.1f'%(time.time()-t0))
    if tr: break
if tr is None:
    q0=topdown_quat(0)
    for z in (1.10,1.12):
        t0=time.time(); tr=pl.plan_pose([-0.505,-0.1025,z],q0,t=60.0,attempts=4); print('pose',z,'took %.1f'%(time.time()-t0))
        if tr: break
"

# openrua op 86
timeout 900 python3 -u -c "
import numpy as np, time
from ctl import *; from mp import *; from valid import check
r=Robot('t'); pl=Planner(r)
r.move_joints([0.29,-0.4,0.5,-2.82,-0.86,1.24,1.73],seconds=4); print('hand',r.hand_pose()[0].round(3))
A=[0.29,0.33,-0.89,-2.67,-2.34,3.5,2.59]
for i in range(3):
    t0=time.time(); tr=pl.plan_joints(A,t=60.0,attempts=4); print('took %.1f'%(time.time()-t0))
    if tr: 
        P=np.array([p.positions for p in tr.points]); print('min',P.min(0).round(2),'max',P.max(0).round(2)); break
"

# openrua op 87
cat > iktilt.py <<'EOF'
import numpy as np, sys
from ctl import *
r=Robot("ikt")
lim=np.array(FJT["limits_rad"])
tipx=float(sys.argv[1]) if len(sys.argv)>1 else -0.508
tip=np.array([tipx,-0.1025,0.978])
def pose(th):
    zh=np.array([-np.sin(th),0,-np.cos(th)]); yh=np.array([0,-1,0.]); xh=np.cross(yh,zh)
    R=np.stack([xh,yh,zh],1); return tip-0.1034*zh, R_quat(R)
rng=np.random.default_rng(1)
for deg in [-20,-10,0,10,20,30]:
    pos,q=pose(np.radians(deg)); best=[]
    for i in range(40):
        seed=rng.uniform(lim[:,0],lim[:,1])
        s=r.ik_solve(pos,q,seed,collide=True)
        if s is None: continue
        s=np.array(s); marg=np.minimum(s-lim[:,0],lim[:,1]-s).min(); best.append((marg,s))
    best.sort(key=lambda t:-t[0])
    print("tilt",deg,"origin",pos.round(3),"nsol",len(best))
    for m,s in best[:3]: print("   margin %.2f"%m,np.round(s,2))
EOF
timeout 900 python3 -u iktilt.py -0.508

# openrua op 88
python3 - <<'EOF'
s=open('ctl.py').read()
s=s.replace('''        req.ik_request.robot_state.joint_state.name = list(ARM)
        req.ik_request.robot_state.joint_state.position = list(map(float, seed))''',
'''        f1, f2 = self.finger()  # bridge reports finger2 negative; MoveIt wants both >= 0
        req.ik_request.robot_state.joint_state.name = list(ARM) + ["panda_finger_joint1", "panda_finger_joint2"]
        req.ik_request.robot_state.joint_state.position = list(map(float, seed)) + [abs(f1), abs(f2)]''')
open('ctl.py','w').write(s)
EOF
timeout 900 python3 -u iktilt.py -0.508

# openrua op 89
cat > s4.py <<'EOF'
import numpy as np, sys
from ctl import *
r=Robot("s4"); go="--go" in sys.argv
lim=np.array(FJT["limits_rad"])
def pose(tip,deg):
    th=np.radians(deg); zh=np.array([-np.sin(th),0,-np.cos(th)]); yh=np.array([0,-1,0.]); xh=np.cross(yh,zh)
    return np.array(tip)-0.1034*zh, R_quat(np.stack([xh,yh,zh],1))
A=[0.26,0.32,-0.86,-2.67,-2.38,3.49,2.63]
# waypoints from the grasp upward: (tip z, tilt toward base deg)
wp=[(0.978,0),(0.995,4),(1.01,8),(1.03,13),(1.06,20),(1.10,28),(1.15,35),(1.20,40),(1.25,45)]
seed=A; chain=[]
for z,deg in wp:
    pos,q=pose([-0.508,-0.1025,z],deg)
    s=r.ik_solve(pos,q,seed,collide=True)
    if s is None:
        print("IK fail at",z,deg); break
    print("z %.3f tilt %2d origin %s delta %.2f  %s"%(z,deg,pos.round(3),np.abs(np.array(s)-np.array(seed)).max(),np.round(s,2)))
    chain.append((z,deg,s)); seed=s
EOF
timeout 600 python3 -u s4.py

# openrua op 90
sed -i 's/^wp=.*/wp=[(0.978,0),(0.995,-4),(1.01,-8),(1.03,-12),(1.06,-18),(1.10,-25),(1.15,-30),(1.20,-35),(1.25,-40)]/' s4.py && timeout 600 python3 -u s4.py

# openrua op 91
sed -i 's/^wp=.*/wp=[(0.978,0),(0.995,-4),(1.01,-8),(1.02,-10),(1.03,-12),(1.045,-15),(1.06,-18),(1.08,-22),(1.10,-25),(1.15,-30),(1.20,-35)]/' s4.py && timeout 600 python3 -u s4.py

# openrua op 92
cat > s4.py <<'EOF'
import numpy as np, sys, json
from ctl import *
r=Robot("s4"); go="--go" in sys.argv
def pose(tip,deg):
    th=np.radians(deg); zh=np.array([-np.sin(th),0,-np.cos(th)]); yh=np.array([0,-1,0.]); xh=np.cross(yh,zh)
    return np.array(tip)-0.1034*zh, R_quat(np.stack([xh,yh,zh],1))
top=[-0.44,-0.69,-0.33,-2.82,-2.77,3.7,2.9]
wp=[(1.20,-35),(1.15,-30),(1.10,-25),(1.08,-22),(1.06,-18),(1.045,-15),(1.03,-12),(1.02,-10),(1.01,-8),(0.995,-4),(0.985,-2),(0.978,0)]
seed=top; chain=[]
for z,deg in wp:
    pos,q=pose([-0.508,-0.1025,z],deg)
    s=r.ik_solve(pos,q,seed,collide=True)
    if s is None: print("IK fail at",z,deg); sys.exit(1)
    print("z %.3f tilt %3d origin %s delta %.2f  %s"%(z,deg,pos.round(3),np.abs(np.array(s)-np.array(seed)).max(),np.round(s,2)))
    chain.append((z,deg,list(map(float,s)))); seed=s
json.dump(chain,open("chain.json","w"))
EOF
timeout 600 python3 -u s4.py

# openrua op 93
cat > chk.py <<'EOF'
import numpy as np, json, rclpy
from ctl import *; from valid import check
from moveit_msgs.srv import GetStateValidity, GetPositionFK
r=Robot("chk"); chain=json.load(open("chain.json"))
c=r.node.create_client(GetStateValidity,'/check_state_validity')
def fk(q):
    req=GetPositionFK.Request(); req.header.frame_id=""; req.fk_link_names=["panda_hand"]
    req.robot_state.joint_state.name=list(ARM); req.robot_state.joint_state.position=list(map(float,q))
    f=r.fk.call_async(req); rclpy.spin_until_future_complete(r.node,f,timeout_sec=30); p=f.result().pose_stamped[0].pose
    R=quat_R(p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w); o=np.array([p.position.x,p.position.y,p.position.z])
    return o, o+0.1034*R[:,2]
for (z0,d0,q0),(z1,d1,q1) in zip(chain[:-1],chain[1:]):
    q0=np.array(q0); q1=np.array(q1); bad=[]; tips=[]
    for a in np.linspace(0,1,7):
        q=q0+(q1-q0)*a; v,cs=check(r,q,c); o,t=fk(q); tips.append(t)
        if not v: bad.append((round(a,2),cs))
    tips=np.array(tips); print("seg %.3f->%.3f tip y range %.3f..%.3f x %.3f..%.3f z %.3f..%.3f bad=%s"%(z0,z1,tips[:,1].min(),tips[:,1].max(),tips[:,0].min(),tips[:,0].max(),tips[:,2].min(),tips[:,2].max(),bad))
EOF
timeout 600 python3 -u chk.py

# openrua op 94
cat > s5.py <<'EOF'
import numpy as np, sys, json, time, pickle
from ctl import *; from mp import *
r=Robot("s5"); pl=Planner(r); go="--go" in sys.argv
pl.scene([],remove=["mug","mug_body","mug_rim","mug_handle"]); print("scene",pl.scene(scene_objects()))
chain=json.load(open("chain.json"))
goal=chain[1][2]  # (1.15,-30)
tr=None
for i in range(4):
    t0=time.time(); tr=pl.plan_joints(goal,t=40.0,attempts=4); print("took %.1f"%(time.time()-t0))
    if tr: break
if tr is None: sys.exit(1)
P=np.array([p.positions for p in tr.points]); print("min",P.min(0).round(2),"max",P.max(0).round(2))
pickle.dump(tr,open("tr_to_top.pkl","wb"))
if go:
    pl.execute(tr); p,q=r.hand_pose(); print("hand",p.round(4),q.round(3))
    r.snap("robot0_eye_in_hand","/workspace/s5_eih.png"); r.snap("sideview","/workspace/s5_side.png")
EOF
timeout 900 python3 -u s5.py

# openrua op 95
timeout 900 python3 -u s5.py --go

# openrua op 96
cat > s6.py <<'EOF'
import numpy as np, sys, json
from ctl import *
r=Robot("s6"); chain=json.load(open("chain.json"))
for z,deg,q in chain[2:]:
    r.move_joints(q,seconds=3); p,_=r.hand_pose(); print("z %.3f tilt %d hand %s"%(z,deg,p.round(4)))
r.snap("robot0_eye_in_hand","/workspace/s6_eih.png"); r.snap("sideview","/workspace/s6_side.png"); r.snap("frontview","/workspace/s6_front.png")
EOF
timeout 600 python3 -u s6.py

# openrua op 97
timeout 20 ros2 topic echo --once /franka_robot_state_broadcaster/external_wrench 2>&1 | grep -A7 "wrench:"

# openrua op 98
timeout 120 python3 -u -c "
import numpy as np
from ctl import *
r=Robot('m')
c,d,P,T=r.snap('robot0_eye_in_hand','/workspace/s6b_eih.png'); np.save('s6b_eih_P.npy',P)
Q=P.reshape(-1,3); Q=Q[np.isfinite(Q).all(1)]
m=(Q[:,0]>-0.60)&(Q[:,0]<-0.485)&(Q[:,1]>-0.20)&(Q[:,1]<-0.02)&(Q[:,2]>0.90)&(Q[:,2]<1.05)
S=Q[m]; print('mug pts',len(S))
for x0 in np.arange(-0.60,-0.485,0.01):
    s=S[(S[:,0]>=x0)&(S[:,0]<x0+0.01)]
    if len(s)<5: continue
    top=s[s[:,2]>s[:,2].max()-0.006]
    print('x %.2f n=%4d y %.3f..%.3f  ymid %.4f  ytop %.4f zmax %.3f'%(x0,len(s),s[:,1].min(),s[:,1].max(),(s[:,1].min()+s[:,1].max())/2,top[:,1].mean(),s[:,2].max()))
print('hand',r.hand_pose()[0].round(4),'fingers',r.finger())
"

# openrua op 99
timeout 60 python3 -c "
import numpy as np
P=np.load('s6b_eih_P.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,2]>1.03)&(P[:,2]<1.07)&(P[:,0]>-0.50)&(P[:,0]<-0.40)&(P[:,1]>-0.30)&(P[:,1]<0.0)
S=P[m]; print('wall-top pts',len(S),'x %.4f..%.4f z %.3f..%.3f'%(S[:,0].min(),S[:,0].max(),S[:,2].min(),S[:,2].max()))
for z0 in np.arange(1.03,1.07,0.005):
    s=S[(S[:,2]>=z0)&(S[:,2]<z0+0.005)]
    if len(s): print('z %.3f n=%4d xmin %.4f'%(z0,len(s),s[:,0].min()))
"; python3 -c "
import cv2; im=cv2.imread('/workspace/s6_side.png'); h,w=im.shape[:2]; cv2.imwrite('/workspace/s6_side_crop.png', cv2.resize(im[150:350, 380:640], None, fx=2.5, fy=2.5, interpolation=cv2.INTER_CUBIC))"

# openrua op 100
cat > wr.py <<'EOF'
import rclpy
from geometry_msgs.msg import WrenchStamped
class Wrench:
    def __init__(self,r):
        self.r=r; self.m=None; r.node.create_subscription(WrenchStamped,"/franka_robot_state_broadcaster/external_wrench",self._cb,1)
    def _cb(self,m): self.m=m
    def get(self):
        self.m=None
        while self.m is None: rclpy.spin_once(self.r.node,timeout_sec=0.2)
        f=self.m.wrench.force; return (round(f.x,2),round(f.y,2),round(f.z,2))
EOF
timeout 300 python3 -u -c "
import numpy as np
from ctl import *; from wr import Wrench
r=Robot('s7'); w=Wrench(r); q0=topdown_quat(0)
print('wrench before',w.get())
for pos in ([-0.509,-0.102,1.0883],[-0.509,-0.102,1.084],[-0.509,-0.102,1.081]):
    r.move_pose(pos,q0,seconds=2.5); print('   wrench',w.get(),'fingers',np.round(r.finger(),4))
r.snap('robot0_eye_in_hand','/workspace/s7_eih.png')
"

# openrua op 101
timeout 120 python3 -u -c "
import numpy as np
from ctl import *
r=Robot('m'); c,d,P,T=r.snap('robot0_eye_in_hand','/workspace/s7b_eih.png')
Q=P.reshape(-1,3); Q=Q[np.isfinite(Q).all(1)]
F=Q[(Q[:,0]>-0.515)&(Q[:,0]<-0.49)&(Q[:,2]>1.016)&(Q[:,2]<1.04)&(Q[:,1]>-0.2)&(Q[:,1]<0)]
print('finger pts',len(F))
h=np.histogram(F[:,1],bins=np.arange(-0.18,-0.02,0.005))
for n,e in zip(*h): print('y %.3f n=%d'%(e,n)) if n>20 else None
B=Q[(Q[:,0]>-0.495)&(Q[:,0]<-0.487)&(Q[:,2]>0.95)&(Q[:,2]<1.012)&(Q[:,1]>-0.2)&(Q[:,1]<0)]
print('base-end body y %.4f..%.4f mid %.4f zmax %.3f'%(B[:,1].min(),B[:,1].max(),(B[:,1].min()+B[:,1].max())/2,B[:,2].max()))
print('hand',r.hand_pose()[0].round(4))
"

# openrua op 102
timeout 120 python3 -u -c "
import numpy as np
from ctl import *
r=Robot('m'); c,d,P,T=r.snap('robot0_eye_in_hand','/workspace/s7b_eih.png'); np.save('s7b_eih_P.npy',P)
Q=P.reshape(-1,3); Q=Q[np.isfinite(Q).all(1)]
R=Q[(Q[:,0]>-0.62)&(Q[:,0]<-0.46)&(Q[:,1]>-0.2)&(Q[:,1]<-0.0)&(Q[:,2]>0.9)&(Q[:,2]<1.05)]
fl=R[R[:,1]>-0.07]; fr=R[R[:,1]<-0.138]
for name,F in (('+y finger',fl),('-y finger',fr)):
    print(name,len(F),'x %.4f..%.4f y %.4f..%.4f z %.3f..%.3f'%(F[:,0].min(),F[:,0].max(),F[:,1].min(),F[:,1].max(),F[:,2].min(),F[:,2].max()))
    # inner face: y closest to center at the pad region
    pad=F[(F[:,2]>0.98)&(F[:,2]<1.0)]
    if len(pad): print('   pad z0.98-1.0 y %.4f..%.4f x %.4f..%.4f'%(pad[:,1].min(),pad[:,1].max(),pad[:,0].min(),pad[:,0].max()))
body=R[(R[:,1]>-0.138)&(R[:,1]<-0.07)]
for x0 in np.arange(-0.60,-0.47,0.01):
    s=body[(body[:,0]>=x0)&(body[:,0]<x0+0.01)]
    if len(s)>10: print('body x %.2f n=%5d y %.4f..%.4f z %.3f..%.3f'%(x0,len(s),s[:,1].min(),s[:,1].max(),s[:,2].min(),s[:,2].max()))
"

# openrua op 103
timeout 300 python3 -u -c "
import numpy as np
from ctl import *; from wr import Wrench
r=Robot('p1'); w=Wrench(r); q0=topdown_quat(0)
p,_=r.hand_pose(); print('start',p.round(4),w.get())
for y in (-0.06,-0.02,0.02,0.06):
    r.move_pose([-0.503,y,1.0876],q0,seconds=3); print('   wrench',w.get())
r.snap('birdview','/workspace/p1_bird.png'); r.snap('sideview','/workspace/p1_side.png'); r.snap('robot0_eye_in_hand','/workspace/p1_eih.png')
"

# openrua op 104
timeout 120 python3 -u -c "
import numpy as np
from ctl import *
r=Robot('f')
for cam in ('sideview','frontview','birdview','agentview'):
    c,d,P,T=r.snap(cam,'/workspace/p1_%s.png'%cam); np.save('p1_%s_P.npy'%cam,P)
    Q=P.reshape(-1,3); Q=Q[np.isfinite(Q).all(1)]
    m=(Q[:,2]>0.89)&(Q[:,2]<1.06)&(Q[:,0]>-0.68)&(Q[:,0]<-0.30)&(Q[:,1]>-0.5)&(Q[:,1]<0.5)
    m&=~((Q[:,0]>-0.475)&(Q[:,0]<-0.315)&(Q[:,1]>-0.35)&(Q[:,1]<0.07))  # caddy
    S=Q[m]; print(cam,len(S))
    if len(S)==0: continue
    H,xe,ye=np.histogram2d(S[:,0],S[:,1],bins=[np.arange(-0.68,-0.30,0.02),np.arange(-0.5,0.5,0.02)])
    for i,x0 in enumerate(xe[:-1]):
        row=''.join('#' if H[i,j]>30 else ('+' if H[i,j]>5 else '.') for j in range(H.shape[1]))
        print('%.2f %s'%(x0,row))
    print('     y from -0.5 to 0.5 step 0.02')
" 2>&1 | head -90

# openrua op 105
timeout 60 python3 -c "
import numpy as np
for cam in ('sideview','agentview','frontview'):
    P=np.load('p1_%s_P.npy'%cam).reshape(-1,3); P=P[np.isfinite(P).all(1)]
    m=(P[:,2]>0.885)&(P[:,2]<1.06)&(P[:,0]>-0.62)&(P[:,0]<-0.48)&(P[:,1]>-0.05)&(P[:,1]<0.2)
    S=P[m]; print(cam,len(S))
    for x0 in np.arange(-0.62,-0.48,0.01):
        s=S[(S[:,0]>=x0)&(S[:,0]<x0+0.01)]
        if len(s)<5: continue
        top=s[s[:,2]>s[:,2].max()-0.005]
        print('  x %.2f n=%4d y %.3f..%.3f ytopmean %.4f z %.3f..%.3f'%(x0,len(s),s[:,1].min(),s[:,1].max(),top[:,1].mean(),s[:,2].min(),s[:,2].max()))
"

# openrua op 106
timeout 600 python3 -u -c "
import numpy as np, time
from ctl import *; from mp import *
r=Robot('p2'); pl=Planner(r); q0=topdown_quat(0)
pl.scene([],remove=['mug','mug_body','mug_rim','mug_handle'])
for z in (1.12,1.16,1.20):
    r.move_pose([-0.503,0.06,z],q0,seconds=2.5)
park=r.ik_solve([-0.30,0.15,1.35],q0,collide=True); print('park',np.round(park,2))
tr=None
for i in range(3):
    tr=pl.plan_joints(park,t=30.0,attempts=4)
    if tr: break
if tr: pl.execute(tr)
print('hand',r.hand_pose()[0].round(4))
for cam in ('birdview','sideview','frontview','agentview'):
    c,d,P,T=r.snap(cam,'/workspace/p2_%s.png'%cam); np.save('p2_%s_P.npy'%cam,P)
"

# openrua op 107
python3 - <<'EOF'
s=open('ctl.py').read()
s=s.replace('''    def move_pose(self, pos_world, quat, seconds=3.0, seed=None):
        q = self.ik_solve(pos_world, quat, seed)
        if q is None:
            print("  IK failed for", pos_world, flush=True); return False''',
'''    def move_pose(self, pos_world, quat, seconds=3.0, seed=None, max_delta=1.0, collide=True):
        q = self.ik_solve(pos_world, quat, seed, collide=collide)
        if q is None:
            print("  IK failed for", pos_world, flush=True); return False
        d = np.abs(np.array(q) - np.array(self.arm_q())).max()
        if d > max_delta:
            print(f"  REFUSED: IK solution is {d:.2f} rad away (max_delta={max_delta})", flush=True); return False''')
open('ctl.py','w').write(s)
EOF
timeout 120 python3 -u -c "
import numpy as np
from ctl import *
r=Robot('c'); print('q',np.round(r.arm_q(),2)); p,q=r.hand_pose(); print('hand',p.round(4),q.round(3))
for cam in ('birdview','sideview','frontview','agentview'):
    c,d,P,T=r.snap(cam,'/workspace/p2_%s.png'%cam); np.save('p2_%s_P.npy'%cam,P)
"

# openrua op 108
timeout 60 python3 -c "
import numpy as np
for cam in ('birdview','sideview','agentview','frontview'):
    P=np.load('p2_%s_P.npy'%cam).reshape(-1,3); P=P[np.isfinite(P).all(1)]
    m=(P[:,2]>0.89)&(P[:,2]<1.10)&(P[:,0]>-0.68)&(P[:,0]<-0.30)&(P[:,1]>-0.4)&(P[:,1]<0.4)
    m&=~((P[:,0]>-0.475)&(P[:,0]<-0.315)&(P[:,1]>-0.35)&(P[:,1]<0.062))
    S=P[m]
    if len(S)==0: print(cam,'none'); continue
    # top slice
    top=S[S[:,2]>S[:,2].max()-0.006]
    print(cam,'n',len(S),'zmax %.3f'%S[:,2].max(),'top x %.3f..%.3f y %.3f..%.3f  center (%.4f,%.4f)'%(top[:,0].min(),top[:,0].max(),top[:,1].min(),top[:,1].max(),(top[:,0].min()+top[:,0].max())/2,(top[:,1].min()+top[:,1].max())/2))
    for z0 in np.arange(0.89,1.0,0.02):
        s=S[(S[:,2]>=z0)&(S[:,2]<z0+0.02)]
        if len(s)>5: print('   z %.2f n=%4d x %.3f..%.3f y %.3f..%.3f'%(z0,len(s),s[:,0].min(),s[:,0].max(),s[:,1].min(),s[:,1].max()))
"

# openrua op 109
timeout 60 python3 -c "
import numpy as np
def fit(xy):
    A=np.c_[2*xy[:,0],2*xy[:,1],np.ones(len(xy))]; b=(xy**2).sum(1)
    c=np.linalg.lstsq(A,b,rcond=None)[0]; return c[0],c[1],np.sqrt(c[2]+c[0]**2+c[1]**2)
for cam in ('sideview','agentview','frontview','birdview'):
    P=np.load('p2_%s_P.npy'%cam).reshape(-1,3); P=P[np.isfinite(P).all(1)]
    m=(P[:,2]>0.885)&(P[:,2]<1.05)&(P[:,0]>-0.66)&(P[:,0]<-0.48)&(P[:,1]>-0.05)&(P[:,1]<0.25)
    S=P[m]
    if len(S)<10: print(cam,'few'); continue
    print(cam,'n',len(S),'zmax %.4f'%S[:,2].max())
    for z0 in np.arange(0.885,1.0,0.01):
        s=S[(S[:,2]>=z0)&(S[:,2]<z0+0.01)]
        if len(s)>5: print('   z %.3f n=%4d x %.3f..%.3f y %.3f..%.3f'%(z0,len(s),s[:,0].min(),s[:,0].max(),s[:,1].min(),s[:,1].max()))
    top=S[S[:,2]>S[:,2].max()-0.008]
    if len(top)>10: print('   rim fit',np.round(fit(top[:,:2]),4))
"

# openrua op 110
cat >> mp.py <<'EOF'

def cyl(name, cx, cy, z0, z1, rad):
    co=CollisionObject(); co.header.frame_id="world"; co.id=name
    sp=SolidPrimitive(); sp.type=SolidPrimitive.CYLINDER; sp.dimensions=[float(z1-z0), float(rad)]
    p=Pose(); p.position.x,p.position.y,p.position.z=float(cx),float(cy),float((z0+z1)/2); p.orientation.w=1.0
    co.primitives=[sp]; co.primitive_poses=[p]; co.operation=CollisionObject.ADD
    return co

MUG_C=(-0.548,0.056)
def mug_objects(cx=MUG_C[0], cy=MUG_C[1]):
    return [cyl("mug_body",cx,cy,0.88,0.995,0.052),
            box("mug_handle",[cx+0.03,cy-0.012,0.90],[cx+0.078,cy+0.012,0.97])]
EOF
cat > s8.py <<'EOF'
import numpy as np, sys, json
from ctl import *; from mp import *
r=Robot("s8"); pl=Planner(r)
pl.scene([],remove=["mug","mug_body","mug_rim","mug_handle"]); print("scene",pl.scene(scene_objects(with_mug=False)+mug_objects()))
lim=np.array(FJT["limits_rad"])
tip=np.array([MUG_C[0],MUG_C[1]+0.0485,0.972])
def pose(tipz,deg):
    th=np.radians(deg); zh=np.array([-np.sin(th),0,-np.cos(th)]); yh=np.array([0,-1,0.]); xh=np.cross(yh,zh)
    t=tip.copy(); t[2]=tipz; return t-0.1034*zh, R_quat(np.stack([xh,yh,zh],1))
rng=np.random.default_rng(5)
res={}
for deg in [-30,-20,-10,0,10,20]:
    pos,q=pose(0.972,deg); best=[]
    for i in range(30):
        s=r.ik_solve(pos,q,rng.uniform(lim[:,0],lim[:,1]),collide=True)
        if s is None: continue
        s=np.array(s); best.append((np.minimum(s-lim[:,0],lim[:,1]-s).min(),s))
    best.sort(key=lambda t:-t[0]); res[deg]=best
    print("tilt",deg,"origin",pos.round(3),"nsol",len(best),[ (round(m,2),np.round(s,2).tolist()) for m,s in best[:2]])
EOF
timeout 900 python3 -u s8.py

# openrua op 111
cat >> mp.py <<'EOF'

def ring_objects(cx, cy, z0, z1, rad=0.05, n=12, skip_angle=None, name="mugw"):
    """hollow mug wall as n chord boxes; omit segment nearest skip_angle (rad)."""
    objs=[]
    for k in range(n):
        a=2*np.pi*k/n
        if skip_angle is not None and abs((a-skip_angle+np.pi)%(2*np.pi)-np.pi)<np.pi/n*0.99: continue
        co=CollisionObject(); co.header.frame_id="world"; co.id=f"{name}{k}"
        sp=SolidPrimitive(); sp.type=SolidPrimitive.BOX; sp.dimensions=[0.012, float(2*rad*np.tan(np.pi/n)+0.004), float(z1-z0)]
        p=Pose(); p.position.x=float(cx+rad*np.cos(a)); p.position.y=float(cy+rad*np.sin(a)); p.position.z=float((z0+z1)/2)
        qz=np.array([0,0,np.sin(a/2),np.cos(a/2)]); p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=map(float,qz)
        co.primitives=[sp]; co.primitive_poses=[p]; co.operation=CollisionObject.ADD; objs.append(co)
    objs.append(box(name+"_bottom",[cx-0.04,cy-0.04,0.88],[cx+0.04,cy+0.04,0.90]))
    return objs
EOF
python3 - <<'EOF'
s=open('s8.py').read()
s=s.replace('''pl.scene([],remove=["mug","mug_body","mug_rim","mug_handle"]); print("scene",pl.scene(scene_objects(with_mug=False)+mug_objects()))''',
'''pl.scene([],remove=["mug","mug_body","mug_rim","mug_handle"]+[f"mugw{k}" for k in range(12)]+["mugw_bottom"])
print("scene",pl.scene(scene_objects(with_mug=False)+ring_objects(MUG_C[0],MUG_C[1],0.88,0.995,skip_angle=np.pi/2)+[mug_objects()[1]]))''')
open('s8.py','w').write(s)
EOF
timeout 900 python3 -u s8.py

# openrua op 112
cat > s9.py <<'EOF'
import numpy as np, sys, json
from ctl import *; from mp import *
r=Robot("s9")
tip=np.array([MUG_C[0],MUG_C[1]+0.0485,0.972])
def pose(tipz,deg):
    th=np.radians(deg); zh=np.array([-np.sin(th),0,-np.cos(th)]); yh=np.array([0,-1,0.]); xh=np.cross(yh,zh)
    t=tip.copy(); t[2]=tipz; return t-0.1034*zh, R_quat(np.stack([xh,yh,zh],1))
A=[-0.26,0.27,1.0,-2.83,2.2,3.42,-0.71]
wp=[(0.972,0),(0.985,-3),(1.00,-6),(1.02,-10),(1.04,-14),(1.06,-18),(1.08,-22),(1.10,-25),(1.13,-28),(1.16,-31),(1.20,-35)]
seed=A; chain=[]
for z,deg in wp:
    pos,q=pose(z,deg); s=r.ik_solve(pos,q,seed,collide=True)
    if s is None: print("IK fail",z,deg); break
    print("z %.3f tilt %3d origin %s delta %.2f %s"%(z,deg,pos.round(3),np.abs(np.array(s)-np.array(seed)).max(),np.round(s,2)))
    chain.append((z,deg,list(map(float,s)))); seed=s
chain=chain[::-1]; json.dump(chain,open("chain2.json","w"))
EOF
timeout 600 python3 -u s9.py

# openrua op 113
python3 - <<'EOF'
s=open('s9.py').read()
s=s.replace("A=[-0.26,0.27,1.0,-2.83,2.2,3.42,-0.71]","A=[0.04,-0.86,0.52,-2.92,-2.86,3.68,-2.07]")
s=s.replace("wp=[(0.972,0),(0.985,-3),(1.00,-6),(1.02,-10),(1.04,-14),(1.06,-18),(1.08,-22),(1.10,-25),(1.13,-28),(1.16,-31),(1.20,-35)]",
"wp=[(1.16,-31),(1.13,-28),(1.10,-25),(1.08,-22),(1.06,-18),(1.04,-14),(1.03,-12),(1.02,-10),(1.01,-8),(1.00,-6),(0.99,-4),(0.985,-3),(0.978,-1),(0.972,0)]")
s=s.replace("chain=chain[::-1]; json.dump","json.dump")
open('s9.py','w').write(s)
EOF
timeout 600 python3 -u s9.py

# openrua op 114
sed -i 's/chain=json.load(open("chain.json"))/chain=json.load(open(sys.argv[1] if len(sys.argv)>1 else "chain.json"))/; s/^import numpy as np, json, rclpy/import numpy as np, json, rclpy, sys/' chk.py && timeout 600 python3 -u chk.py chain2.json

# openrua op 115
python3 - <<'EOF'
s=open('mp.py').read()
s=s.replace('''    def execute(self, tr, retries=2):''','''    def execute_steps(self, tr, stride=4, seconds=1.0):
        """follow a planned path as a series of single-point goals (tracks the path, no lag pile-up)."""
        idx=[tr.joint_names.index(j) for j in ARM]
        pts=[[p.positions[i] for i in idx] for p in tr.points]
        sel=pts[stride::stride]
        if not sel or sel[-1] is not pts[-1]: sel.append(pts[-1])
        for k,q in enumerate(sel):
            self.r.move_joints(q, seconds=seconds if k<len(sel)-1 else 2.0, retries=0 if k<len(sel)-1 else 2)
        err=np.abs(np.array(self.r.arm_q())-np.array(pts[-1])).max(); print(f"  steps done err={err:.4f}"); return err<0.02
    def execute(self, tr, retries=2):''')
open('mp.py','w').write(s)
EOF
cat > s10.py <<'EOF'
import numpy as np, sys, json
from ctl import *; from mp import *
r=Robot("s10"); pl=Planner(r); go="--go" in sys.argv
chain=json.load(open("chain2.json")); top=chain[0][2]
tr=None
for i in range(4):
    tr=pl.plan_joints(top,t=30.0,attempts=4)
    if tr: break
if tr is None: sys.exit(1)
P=np.array([p.positions for p in tr.points]); print("min",P.min(0).round(2),"max",P.max(0).round(2))
if go:
    pl.execute_steps(tr); p,q=r.hand_pose(); print("hand",p.round(4),q.round(3))
    r.snap("robot0_eye_in_hand","/workspace/s10_eih.png"); r.snap("sideview","/workspace/s10_side.png")
EOF
timeout 900 python3 -u s10.py --go

# openrua op 116
timeout 120 python3 -u -c "
import numpy as np, json
from ctl import *; from valid import check
r=Robot('v'); print('start',check(r,r.arm_q())); print('goal',check(r,json.load(open('chain2.json'))[0][2]))
"

# openrua op 117
timeout 120 python3 -u -c "
import numpy as np, rclpy
from ctl import *; from valid import check
from moveit_msgs.srv import GetPositionFK
r=Robot('v'); q=np.array(r.arm_q())
def fk(q):
    req=GetPositionFK.Request(); req.header.frame_id=''; req.fk_link_names=['panda_hand']
    req.robot_state.joint_state.name=list(ARM); req.robot_state.joint_state.position=list(map(float,q))
    f=r.fk.call_async(req); rclpy.spin_until_future_complete(r.node,f,timeout_sec=30); p=f.result().pose_stamped[0].pose
    R=quat_R(p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w); o=np.array([p.position.x,p.position.y,p.position.z]); return o,o+0.1034*R[:,2]
print('now',fk(q))
for d4 in (0.1,0.2,0.3):
    for d2 in (0,-0.1,-0.2):
        qq=q.copy(); qq[3]+=d4; qq[1]+=d2; v=check(r,qq); o,t=fk(qq); print('d4 %.1f d2 %.1f'%(d4,d2),v[0],'hand',o.round(3),'tip',t.round(3))
"

# openrua op 118
timeout 900 python3 -u -c "
import numpy as np, json
from ctl import *; from mp import *; from valid import check
r=Robot('s10'); pl=Planner(r); q=np.array(r.arm_q()); q[3]+=0.1
r.move_joints(q,seconds=2); print('start valid',check(r,r.arm_q()))
chain=json.load(open('chain2.json')); top=chain[0][2]
tr=None
for i in range(4):
    tr=pl.plan_joints(top,t=30.0,attempts=4)
    if tr: break
if tr is None: raise SystemExit(1)
P=np.array([p.positions for p in tr.points]); print('min',P.min(0).round(2),'max',P.max(0).round(2))
pl.execute_steps(tr); p,qq=r.hand_pose(); print('hand',p.round(4),qq.round(3))
r.snap('robot0_eye_in_hand','/workspace/s10_eih.png'); r.snap('sideview','/workspace/s10_side.png')
"

# openrua op 119
cat > s11.py <<'EOF'
import json, numpy as np, cv2
from ctl import Robot
from wr import Wrench
r = Robot("s11"); w = Wrench(r)
chain = json.load(open("chain2.json"))
print("start", r.hand_pose()[0].round(4), "fingers", r.finger(), "wrench", w.get())
prev = np.array(r.arm_q())
for i,(z,deg,q) in enumerate(chain[1:], 1):
    d = np.abs(np.array(q)-prev).max(); sec = max(1.5, d/0.4)
    ok = r.move_joints(q, seconds=sec)
    p,_ = r.hand_pose(); prev = np.array(r.arm_q())
    print(f"[{i}] z={z} tilt={deg} ok={ok} hand={p.round(4)} wrench={w.get()}", flush=True)
    if i == len(chain)-3:
        r.snap("robot0_eye_in_hand", "/workspace/s11_eih_pre.png")
r.snap("robot0_eye_in_hand", "/workspace/s11_eih.png")
r.snap("sideview", "/workspace/s11_side.png")
EOF
timeout 900 python3 s11.py 2>&1 | grep -v "^  move" 

# openrua op 120
cat > s11b.py <<'EOF'
import numpy as np
from ctl import Robot
r = Robot("s11b")
c,d,P,T = r.snap("robot0_eye_in_hand","/workspace/s11b_eih.png")
np.save("/workspace/s11b_eih_P.npy", P)
pts = P.reshape(-1,3); pts = pts[np.isfinite(pts).all(1)]
# rim: z in 0.975..0.995, exclude fingers (fingers are near y 0.06-0.15 and z>0.97 too...)
rim = pts[(pts[:,2]>0.975)&(pts[:,2]<0.996)]
print("rim-ish pts", len(rim))
# points near x=-0.548 +-0.01
sl = rim[np.abs(rim[:,0]+0.548)<0.01]
if len(sl): 
    ys = np.sort(sl[:,1]); print("y range at x=-0.548:", ys.min().round(4), ys.max().round(4))
    h,_ = np.histogram(sl[:,1], bins=np.arange(-0.05,0.17,0.005)); print(list(zip(np.arange(-0.05,0.17,0.005).round(3), h)))
# fingers: dark gray, z in 0.96..1.0 near y ~0.06..0.15
fing = pts[(pts[:,2]>0.96)&(pts[:,2]<1.0)&(np.abs(pts[:,0]+0.548)<0.02)]
print("mug circle fit from rim pts:")
mask = (rim[:,2]>0.985)
xy = rim[mask][:,:2]
A = np.c_[2*xy, np.ones(len(xy))]; b = (xy**2).sum(1)
cx,cy,c0 = np.linalg.lstsq(A,b,rcond=None)[0]; rad = np.sqrt(c0+cx**2+cy**2)
print("center", round(cx,4), round(cy,4), "r", round(rad,4))
EOF
python3 s11b.py 2>&1 | tail -8

# openrua op 121
cat > s12.py <<'EOF'
import numpy as np
from ctl import Robot
from wr import Wrench
r = Robot("s12"); w = Wrench(r)
print("before", w.get(), r.finger())
f = r.gripper(0.0)
print("after", w.get(), r.hand_pose()[0].round(4))
r.snap("robot0_eye_in_hand","/workspace/s12_eih.png"); r.snap("sideview","/workspace/s12_side.png")
EOF
python3 s12.py 2>&1 | tail -4

# openrua op 122
sed -n 1,60p s9.py; cat valid.py | head -30

# openrua op 123
cat > s13.py <<'EOF'
import numpy as np, json
from ctl import *; from mp import *
from valid import check
r=Robot("s13"); pl=Planner(r)
print("scene rm:", pl.scene([], remove=[f"mugw{i}" for i in range(12)]+["mugw_bottom","mug_handle","mug_body","mug_rim"]))
tip=np.array([-0.548,0.1045,0.972])
def pose(tipz,deg):
    th=np.radians(deg); zh=np.array([-np.sin(th),0,-np.cos(th)]); yh=np.array([0,-1,0.]); xh=np.cross(yh,zh)
    t=tip.copy(); t[2]=tipz; return t-0.1034*zh, R_quat(np.stack([xh,yh,zh],1))
seed=r.arm_q(); print("cur", np.round(seed,2))
chain=[(0.972,0,list(map(float,seed)))]
for z in [1.00,1.03,1.06,1.09,1.12,1.16,1.20,1.25]:
    best=None
    for deg in [0,3,6,10,-3,-6]:
        pos,q=pose(z,deg); s=r.ik_solve(pos,q,seed,collide=True)
        if s is None: continue
        d=np.abs(np.array(s)-np.array(seed)).max()
        if d<0.6 and (best is None or d<best[0]): best=(d,deg,s)
    if best is None: print("no IK at",z); break
    d,deg,s=best; print("z %.3f tilt %3d delta %.2f %s"%(z,deg,d,np.round(s,2)))
    chain.append((z,deg,list(map(float,s)))); seed=s
json.dump(chain,open("chain3.json","w"))
EOF
python3 s13.py 2>&1 | tail -12

# openrua op 124
grep -n "def scene" -A 14 mp.py; cat > gs.py <<'EOF'
from ctl import *; import rclpy
from moveit_msgs.srv import GetPlanningScene
r=Robot("gs"); c=r.node.create_client(GetPlanningScene,"/get_planning_scene"); c.wait_for_service(5)
req=GetPlanningScene.Request(); req.components.components=req.components.WORLD_OBJECT_NAMES
f=c.call_async(req); rclpy.spin_until_future_complete(r.node,f,timeout_sec=20)
print([o.id for o in f.result().scene.world.collision_objects])
EOF
python3 gs.py 2>&1 | tail -1

# openrua op 125
cat > s14.py <<'EOF'
import numpy as np, json, sys
from ctl import *; from mp import *
from wr import Wrench
r=Robot("s14"); w=Wrench(r)
def pose(tip,deg):
    th=np.radians(deg); zh=np.array([-np.sin(th),0,-np.cos(th)]); yh=np.array([0,-1,0.]); xh=np.cross(yh,zh)
    return np.array(tip)-0.1034*zh, R_quat(np.stack([xh,yh,zh],1))
def go(tip, degs=(0,3,6,-3,-6,10,-10), maxd=0.6, sec=None):
    seed=r.arm_q(); best=None
    for deg in degs:
        pos,q=pose(tip,deg); s=r.ik_solve(pos,q,seed,collide=True)
        if s is None: continue
        d=np.abs(np.array(s)-np.array(seed)).max()
        if d<maxd and (best is None or d<best[0]): best=(d,deg,s)
    if best is None: print("NO IK for",tip); return False
    d,deg,s=best; ok=r.move_joints(s, seconds=sec or max(1.5,d/0.3))
    p,_=r.hand_pose(); print(f"tip {np.round(tip,3)} tilt {deg} delta {d:.2f} ok={ok} hand={p.round(4)} fing={np.round(r.finger(),4)} wr={w.get()}",flush=True)
    return ok
wps=json.loads(sys.argv[1])
for tip in wps:
    if not go(tip): break
EOF
python3 s14.py '[[-0.548,0.1045,0.985],[-0.548,0.1045,1.00]]' 2>&1 | grep -v "^  move"; python3 -c "
from ctl import Robot; r=Robot('x'); r.snap('sideview','/workspace/s14_side.png'); r.snap('agentview','/workspace/s14_agent.png')"

# openrua op 126
cat > mugz.py <<'EOF'
import numpy as np, sys
from ctl import Robot
r=Robot("mugz")
for cam in ["agentview","sideview"]:
    c,d,P,T=r.snap(cam,f"/workspace/mugz_{cam}.png")
    pts=P.reshape(-1,3); col=c.reshape(-1,3); ok=np.isfinite(pts).all(1); pts=pts[ok]; col=col[ok]
    # mug: near x -0.60..-0.45, y -0.05..0.2, z 0.885..1.15, bright/yellow (not gray gripper): exclude gray (r~g~b)
    m=(pts[:,0]>-0.62)&(pts[:,0]<-0.44)&(pts[:,1]>-0.05)&(pts[:,1]<0.2)&(pts[:,2]>0.882)&(pts[:,2]<1.12)
    b,g,rr=col[m].T.astype(int); yellow=(rr>150)&(g>130)&(b<170)&(rr-b>30)|((rr>200)&(g>200)&(b>200))
    q=pts[m][yellow]
    if len(q): print(cam, "mug pts",len(q),"zmin %.3f zmax %.3f xrange %.3f..%.3f yrange %.3f..%.3f"%(q[:,2].min(),q[:,2].max(),q[:,0].min(),q[:,0].max(),q[:,1].min(),q[:,1].max()))
    else: print(cam,"no mug pts")
    np.save(f"/workspace/mugz_{cam}_P.npy",P)
EOF
python3 mugz.py 2>&1 | tail -2

# openrua op 127
cat > mugz.py <<'EOF'
import numpy as np, sys
from ctl import Robot
r=Robot("mugz")
for cam in ["agentview","sideview","frontview"]:
    c,d,P,T=r.snap(cam,f"/workspace/mugz_{cam}.png")
    pts=P.reshape(-1,3); col=c.reshape(-1,3).astype(int); ok=np.isfinite(pts).all(1); pts=pts[ok]; col=col[ok]
    m=(pts[:,0]>-0.63)&(pts[:,0]<-0.44)&(pts[:,1]>-0.05)&(pts[:,1]<0.2)&(pts[:,2]>0.886)&(pts[:,2]<1.06)
    b,g,rr=col[m].T
    gray=(np.abs(rr-g)<20)&(np.abs(g-b)<20)&(rr<170)
    q=pts[m][~gray]
    if len(q):
        h,e=np.histogram(q[:,2],bins=np.arange(0.88,1.07,0.01))
        print(cam,len(q),"zmin %.3f"%q[:,2].min(), [(round(e[i],2),int(h[i])) for i in range(len(h)) if h[i]>0][:6])
        lo=q[q[:,2]<q[:,2].min()+0.015]; print("   lowest band xy mean",lo[:,:2].mean(0).round(3), "x %.3f..%.3f y %.3f..%.3f"%(lo[:,0].min(),lo[:,0].max(),lo[:,1].min(),lo[:,1].max()))
    else: print(cam,"none")
EOF
python3 mugz.py 2>&1 | tail -6

# openrua op 128
python3 s14.py '[[-0.548,0.14,1.02],[-0.548,0.18,1.04],[-0.548,0.22,1.06],[-0.548,0.26,1.09],[-0.548,0.30,1.12],[-0.548,0.30,1.16],[-0.548,0.30,1.20],[-0.548,0.30,1.24]]' 2>&1 | grep -v "^  move"

# openrua op 129
cat > s15.py <<'EOF'
import numpy as np, sys, json
from ctl import *; from mp import *
r=Robot("s15")
def pose(tip,deg,yaw=0):
    th=np.radians(deg); zh=np.array([-np.sin(th),0,-np.cos(th)]); yh=np.array([0,-1,0.]); xh=np.cross(yh,zh)
    R=np.stack([xh,yh,zh],1); c,s=np.cos(yaw),np.sin(yaw); Rz=np.array([[c,-s,0],[s,c,0],[0,0,1]])
    R=Rz@R; return np.array(tip)-0.1034*R[:,2], R_quat(R)
seed=r.arm_q(); print(np.round(seed,2))
tip=json.loads(sys.argv[1])
for deg in [0,5,10,15,-5,-10,-15,-20]:
  for yaw in [0,0.2,-0.2,0.4,-0.4]:
    pos,q=pose(tip,deg,yaw); s=r.ik_solve(pos,q,seed,collide=True)
    if s is None: print(deg,yaw,"none"); continue
    print(deg,yaw,"delta %.2f"%np.abs(np.array(s)-np.array(seed)).max(), np.round(s,2))
EOF
python3 s15.py '[-0.548,0.26,1.09]' 2>&1 | tail -41

# openrua op 130
sed -n 32,88p mp.py

# openrua op 131
python3 - <<'EOF'
import re
s=open("ctl.py").read()
s=s.replace("""        req.ik_request.avoid_collisions = bool(collide)""","""        req.ik_request.robot_state.is_diff = True  # keep scene-attached objects (held mug)
        req.ik_request.avoid_collisions = bool(collide)""")
open("ctl.py","w").write(s)
m=open("mp.py").read()
m+='''
def attach_mug(pl, hand_off=(0.0,0.04,0.148), rad=0.08, h=0.14, name="held_mug"):
    """Attach a cylinder (held mug, conservative) to panda_hand for planning."""
    from moveit_msgs.msg import AttachedCollisionObject
    aco=AttachedCollisionObject(); aco.link_name="panda_hand"
    co=CollisionObject(); co.header.frame_id="panda_hand"; co.id=name
    sp=SolidPrimitive(); sp.type=SolidPrimitive.CYLINDER; sp.dimensions=[float(h),float(rad)]
    p=Pose(); p.position.x,p.position.y,p.position.z=map(float,hand_off); p.orientation.w=1.0
    co.primitives=[sp]; co.primitive_poses=[p]; co.operation=CollisionObject.ADD
    aco.object=co; aco.touch_links=["panda_hand","panda_leftfinger","panda_rightfinger"]
    ps=PlanningScene(); ps.is_diff=True; ps.robot_state.is_diff=True; ps.robot_state.attached_collision_objects=[aco]
    req=ApplyPlanningScene.Request(); req.scene=ps
    f=pl.aps.call_async(req); rclpy.spin_until_future_complete(pl.r.node,f,timeout_sec=30); return f.result().success
def detach(pl, name="held_mug"):
    from moveit_msgs.msg import AttachedCollisionObject
    aco=AttachedCollisionObject(); aco.link_name="panda_hand"; aco.object.id=name; aco.object.operation=CollisionObject.REMOVE
    ps=PlanningScene(); ps.is_diff=True; ps.robot_state.is_diff=True; ps.robot_state.attached_collision_objects=[aco]
    # also drop the detached copy from the world
    co=CollisionObject(); co.id=name; co.header.frame_id="world"; co.operation=CollisionObject.REMOVE; ps.world.collision_objects=[co]
    req=ApplyPlanningScene.Request(); req.scene=ps
    f=pl.aps.call_async(req); rclpy.spin_until_future_complete(pl.r.node,f,timeout_sec=30); return f.result().success
'''
open("mp.py","w").write(m)
EOF
head -12 mp.py | grep import
cat > s16.py <<'EOF'
import numpy as np, json
from ctl import *; from mp import *
from valid import check
r=Robot("s16"); pl=Planner(r)
print("attach:", attach_mug(pl))
print("valid now:", check(r, r.arm_q()))
target=np.array([-0.417,-0.097,1.33]); q=topdown_quat(0.0)
sols=[]
for i in range(8):
    seed=list(np.random.uniform(-1,1,7)*np.array([2,1,2,0.5,2,1,2])+np.array([0,0,0,-2,0,2,0]))
    s=r.ik_solve(target,q,seed,collide=True)
    if s is not None: sols.append(s); print("IK", np.round(s,2), "valid", check(r,s)[0])
json.dump(sols,open("carry_sols.json","w"))
EOF
python3 s16.py 2>&1 | tail -12

# openrua op 132
cat > gs.py <<'EOF'
from ctl import *; import rclpy
from moveit_msgs.srv import GetPlanningScene
r=Robot("gs"); c=r.node.create_client(GetPlanningScene,"/get_planning_scene"); c.wait_for_service(5)
req=GetPlanningScene.Request(); req.components.components=req.components.WORLD_OBJECT_NAMES|req.components.ROBOT_STATE_ATTACHED_OBJECTS
f=c.call_async(req); rclpy.spin_until_future_complete(r.node,f,timeout_sec=20); sc=f.result().scene
print("world:", [o.id for o in sc.world.collision_objects]); print("attached:", [(a.object.id,a.link_name,a.touch_links) for a in sc.robot_state.attached_collision_objects])
EOF
python3 gs.py 2>&1 | tail -2

# openrua op 133
python3 - <<'EOF'
m=open("mp.py").read()
m=m.replace("""        for k,q in enumerate(sel):
            self.r.move_joints(q, seconds=seconds if k<len(sel)-1 else 2.0, retries=0 if k<len(sel)-1 else 2)""",
"""        prev=np.array(self.r.arm_q())
        for k,q in enumerate(sel):
            d=np.abs(np.array(q)-prev).max(); prev=np.array(q)
            self.r.move_joints(q, seconds=max(seconds, d/0.4) if k<len(sel)-1 else 2.0, retries=0 if k<len(sel)-1 else 2)""")
open("mp.py","w").write(m)
EOF
cat > s17.py <<'EOF'
import numpy as np, json
from ctl import *; from mp import *
from wr import Wrench
r=Robot("s17"); pl=Planner(r); w=Wrench(r)
goal=[-0.72,-0.68,0.41,-2.69,0.28,2.04,0.26]
tr=pl.plan_joints(goal, t=20.0, attempts=20)
if tr:
    idx=[tr.joint_names.index(j) for j in ARM]
    pts=np.array([[p.positions[i] for i in idx] for p in tr.points]); print("path len", len(pts), "total joint travel", np.abs(np.diff(pts,axis=0)).sum(0).round(2))
    pl.execute_steps(tr, stride=3, seconds=1.5)
    p,qq=r.hand_pose(); print("hand", p.round(4), qq.round(3), "fing", np.round(r.finger(),4), "wr", w.get())
    r.snap("agentview","/workspace/s17_agent.png"); r.snap("sideview","/workspace/s17_side.png")
EOF
timeout 1200 python3 s17.py 2>&1 | grep -v "^  move"

# openrua op 134
cat > mugpose.py <<'EOF'
import numpy as np, sys
from ctl import Robot
def fit_circle(xy):
    A=np.c_[2*xy,np.ones(len(xy))]; b=(xy**2).sum(1); cx,cy,c0=np.linalg.lstsq(A,b,rcond=None)[0]; return cx,cy,np.sqrt(max(c0+cx**2+cy**2,0))
def analyze(r, cams, box, tag):
    allp=[]
    for cam in cams:
        c,d,P,T=r.snap(cam,f"/workspace/{tag}_{cam}.png")
        pts=P.reshape(-1,3); col=c.reshape(-1,3).astype(int); ok=np.isfinite(pts).all(1); pts=pts[ok]; col=col[ok]
        (x0,x1),(y0,y1),(z0,z1)=box
        m=(pts[:,0]>x0)&(pts[:,0]<x1)&(pts[:,1]>y0)&(pts[:,1]<y1)&(pts[:,2]>z0)&(pts[:,2]<z1)
        b,g,rr=col[m].T; gray=(np.abs(rr-g)<20)&(np.abs(g-b)<20)&(rr<170); dark=(rr+g+b<150)
        q=pts[m][~gray&~dark]; print(cam,"pts",len(q)); allp.append(q)
    q=np.vstack(allp)
    if len(q)<20: print("too few"); return
    print("z %.3f..%.3f  x %.3f..%.3f  y %.3f..%.3f"%(q[:,2].min(),q[:,2].max(),q[:,0].min(),q[:,0].max(),q[:,1].min(),q[:,1].max()))
    zs=np.arange(q[:,2].min(),q[:,2].max(),0.01)
    for z in zs:
        s=q[(q[:,2]>=z)&(q[:,2]<z+0.01)]
        if len(s)>15:
            cx,cy,rad=fit_circle(s[:,:2]); print("  z %.3f n %4d  cxy (%.3f,%.3f) r %.3f  xrange %.3f..%.3f yrange %.3f..%.3f"%(z,len(s),cx,cy,rad,s[:,0].min(),s[:,0].max(),s[:,1].min(),s[:,1].max()))
    return q
if __name__=="__main__":
    r=Robot("mugpose"); box=eval(sys.argv[1]); tag=sys.argv[2]
    q=analyze(r,["agentview","sideview","birdview","frontview"],box,tag); np.save(f"/workspace/{tag}_pts.npy",q)
EOF
python3 mugpose.py "((-0.56,-0.28),(-0.26,0.06),(1.06,1.26))" mp17 2>&1 | grep -v "^\[INFO"

# openrua op 135
python3 - <<'EOF'
import numpy as np
from scipy.optimize import fsolve
P=np.array([-0.402,0.988]); C=np.array([-0.454,1.056])
def S(th,h):
    B=P+0.033*np.array([np.cos(th),np.sin(th)]); a=np.array([-np.sin(th),np.cos(th)]); r=0.033+0.162*h
    return B+h*a-r*np.array([np.cos(th),np.sin(th)])
sol=fsolve(lambda v: S(v[0],v[1])-C, [0.6,0.07]); th,h=sol
print("theta deg %.1f h %.3f  check"%(np.degrees(th),h), S(th,h).round(4))
B=P+0.033*np.array([np.cos(th),np.sin(th)]); a=np.array([-np.sin(th),np.cos(th)])
for hc in [0.045,0.05,0.055]:
    com=B+hc*a; print(" com h %.3f -> x %.4f (stable if in -0.454..-0.402)"%(hc,com[0]))
rim=B+0.11*a; print("base center",B.round(4),"rim center",rim.round(4),"rim top edge -x", (rim-0.05*np.array([np.cos(th),np.sin(th)])).round(4), "rim +x", (rim+0.05*np.array([np.cos(th),np.sin(th)])).round(4))
EOF

# openrua op 136
python3 - <<'EOF'
import numpy as np
from scipy.optimize import fsolve
P=np.array([-0.402,0.988]); C=np.array([-0.454,1.056])
for ov in [0.0,0.006,0.010,0.014,0.018]:
    def S(th,h):
        B=P+(0.033-ov)*np.array([np.cos(th),np.sin(th)]); a=np.array([-np.sin(th),np.cos(th)]); r=0.033+0.162*h
        return B+h*a-r*np.array([np.cos(th),np.sin(th)])
    th,h=fsolve(lambda v: S(v[0],v[1])-C,[0.5,0.07])
    B=P+(0.033-ov)*np.array([np.cos(th),np.sin(th)]); a=np.array([-np.sin(th),np.cos(th)])
    com=B+0.05*a; rim=B+0.11*a
    print("ov %.3f theta %.1f h %.3f com_x %.4f %s base %s rim %s"%(ov,np.degrees(th),h,com[0],"LEAN-STABLE" if com[0]<-0.402 else "rocks upright",B.round(4),rim.round(4)))
EOF

# openrua op 137
cat > s18.py <<'EOF'
import numpy as np, json, sys
from ctl import *; from mp import *
from wr import Wrench
r=Robot("s18"); w=Wrench(r)
def pose(tip,deg):
    th=np.radians(deg); zh=np.array([-np.sin(th),0,-np.cos(th)]); yh=np.array([0,-1,0.]); xh=np.cross(yh,zh)
    return np.array(tip)-0.1034*zh, R_quat(np.stack([xh,yh,zh],1))
TH=float(sys.argv[1]); XB=float(sys.argv[2]); YR=-0.143
def tip_for_base(zb):
    th=np.radians(TH); a=np.array([-np.sin(th),0,np.cos(th)]); B=np.array([XB,YR,zb])
    rim=B+0.11*a; pinch=rim+np.array([0,0.0485,0]); return pinch-0.02*a
def go(tip, deg, maxd=0.6):
    seed=r.arm_q(); pos,q=pose(tip,deg); s=r.ik_solve(pos,q,seed,collide=True)
    if s is None: print("NO IK",np.round(tip,3)); return False
    d=np.abs(np.array(s)-np.array(seed)).max()
    if d>maxd: print("too far",d); return False
    ok=r.move_joints(s,seconds=max(1.5,d/0.3)); p,_=r.hand_pose()
    print(f"tip {np.round(tip,3)} tilt {deg} d {d:.2f} ok={ok} hand={p.round(4)} fing={np.round(r.finger(),4)} wr={w.get()}",flush=True); return ok
zs=[float(v) for v in sys.argv[3].split(",")]
for zb in zs:
    if not go(tip_for_base(zb), -TH): break
    wz=w.get()[2]
    if wz>-4.6: print("CONTACT? fz",wz); break
EOF
python3 s18.py 10 -0.397 "1.13,1.09,1.05,1.03" 2>&1 | grep -v "^  move"

# openrua op 138
python3 -c "
from ctl import *; from mp import *
r=Robot('d'); pl=Planner(r); print('detach', detach(pl))" 2>&1 | tail -1; python3 gs.py 2>&1 | tail -2; python3 s18.py 10 -0.397 "1.09,1.05,1.03" 2>&1 | grep -v "^  move"

# openrua op 139
python3 mugpose.py "((-0.50,-0.30),(-0.26,-0.06),(0.99,1.16))" mp18 2>&1 | grep -v "^\[INFO"

# openrua op 140
python3 s18.py 10 -0.397 "1.015,1.003,0.995,0.988,0.982" 2>&1 | grep -v "^  move"

# openrua op 141
python3 mugpose.py "((-0.50,-0.30),(-0.26,-0.06),(0.95,1.13))" mp18b 2>&1 | grep -v "^\[INFO"

# openrua op 142
python3 s18.py 10 -0.397 "0.986" 2>&1 | grep -v "^  move"; python3 - <<'EOF' 2>&1 | grep -v "^\[INFO"
import numpy as np
from ctl import *; from wr import Wrench
r=Robot("rel"); w=Wrench(r)
print("pre-release wrench", w.get(), "fingers", np.round(r.finger(),4))
r.gripper(0.08)
print("post-release wrench", w.get())
r.snap("agentview","/workspace/rel_agent.png"); r.snap("sideview","/workspace/rel_side.png")
EOF

# openrua op 143
cat > s19.py <<'EOF'
import numpy as np
from ctl import *; from wr import Wrench
r=Robot("s19"); w=Wrench(r)
def pose(tip,deg):
    th=np.radians(deg); zh=np.array([-np.sin(th),0,-np.cos(th)]); yh=np.array([0,-1,0.]); xh=np.cross(yh,zh)
    return np.array(tip)-0.1034*zh, R_quat(np.stack([xh,yh,zh],1))
p,_=r.hand_pose(); tip=p+0.1034*np.array([-np.sin(np.radians(-10)),0,-np.cos(np.radians(-10))])
for dz,deg in [(0.03,-10),(0.07,-5),(0.12,0)]:
    t=tip+[0,0,dz]; pos,q=pose(t,deg); s=r.ik_solve(pos,q,r.arm_q(),collide=True)
    if s is None: print("noik"); break
    r.move_joints(s,seconds=2); print("hand", r.hand_pose()[0].round(4), "wr", w.get())
# let time pass: repeat current position twice (sim advances during commands)
q=r.arm_q()
for i in range(3): r.move_joints(q,seconds=2,retries=0)
print("settled wr", w.get())
EOF
python3 s19.py 2>&1 | grep -v "^  move"; python3 mugpose.py "((-0.50,-0.30),(-0.26,-0.06),(0.95,1.12))" mp19 2>&1 | grep -v "^\[INFO"

# openrua op 144
cat > s20.py <<'EOF'
import numpy as np, sys
from ctl import *; from wr import Wrench
r=Robot("s20"); w=Wrench(r)
def pose(tip,deg):
    th=np.radians(deg); zh=np.array([-np.sin(th),0,-np.cos(th)]); yh=np.array([0,-1,0.]); xh=np.cross(yh,zh)
    return np.array(tip)-0.1034*zh, R_quat(np.stack([xh,yh,zh],1))
def go(tip,deg=0,maxd=0.7,sec=None):
    seed=r.arm_q(); pos,q=pose(tip,deg); s=r.ik_solve(pos,q,seed,collide=True)
    if s is None: print("NO IK",np.round(tip,3)); return False
    d=np.abs(np.array(s)-np.array(seed)).max()
    if d>maxd: print("too far",d, np.round(s,2)); return False
    ok=r.move_joints(s,seconds=sec or max(1.5,d/0.3)); p,_=r.hand_pose()
    print(f"tip {np.round(tip,3)} d {d:.2f} ok={ok} hand={p.round(4)} fing={np.round(r.finger(),4)} wr={w.get()}",flush=True); return ok
RX,RY,RZTOP=-0.407,-0.129,1.097
print("fingers", r.finger()); r.gripper(0.08)
pin=np.array([RX,RY+0.0485,RZTOP-0.02])
for z in [1.16,1.12,1.095,pin[2]]:
    if not go([pin[0],pin[1],z]): sys.exit()
r.snap("robot0_eye_in_hand","/workspace/s20_eih.png")
EOF
python3 s20.py 2>&1 | grep -v "^  move"

# openrua op 145
cat > s21.py <<'EOF'
import numpy as np, sys
from ctl import *; from wr import Wrench
exec(open("s20.py").read().split("RX,RY,RZTOP")[0].replace('r=Robot("s20")','r=Robot("s21")'))
f=r.gripper(0.0)
if abs(f[0])>0.012 or abs(f[0])<0.002: print("grasp looks wrong", f); sys.exit()
p,_=r.hand_pose(); tip=p+[0,0,-0.1034]
# lift to base above wall tops, shift -x, lower until contact
for z in [1.10,1.13,1.16]: go([tip[0],tip[1],z])
X=tip[0]-0.007
for z in [1.16,1.12,1.09,1.08,1.075,1.07]:
    if not go([X,tip[1],z]): break
    if w.get()[2]>-4.7: print("contact"); break
EOF
python3 s21.py 2>&1 | grep -v "^  move"

# openrua op 146
cat > s22.py <<'EOF'
import numpy as np, sys
from ctl import *; from wr import Wrench
exec(open("s20.py").read().split("RX,RY,RZTOP")[0].replace('r=Robot("s20")','r=Robot("s22")'))
p,_=r.hand_pose(); tip=p+[0,0,-0.1034]; print("tip now", tip.round(4))
for z in [1.085,1.080,1.076]:
    go([tip[0],tip[1],z]); 
    if w.get()[2]>-2.0: print("pressing hard"); break
print("pre-release", w.get(), np.round(r.finger(),4)); r.gripper(0.08)
print("post-release", w.get())
p,_=r.hand_pose(); tip=p+[0,0,-0.1034]
for z in [1.11,1.16,1.22]: go([tip[0],tip[1],z])
q=r.arm_q()
for i in range(3): r.move_joints(q,seconds=2,retries=0)
print("settled", w.get())
EOF
python3 s22.py 2>&1 | grep -v "^  move"; python3 mugpose.py "((-0.50,-0.30),(-0.26,-0.06),(0.95,1.12))" mp22 2>&1 | grep -v "^\[INFO"

# openrua op 147
python3 - <<'EOF' 2>&1 | grep -v "^\[INFO"
import numpy as np
from ctl import *
r=Robot("settle"); q=r.arm_q()
for i in range(5): r.move_joints(q,seconds=2,retries=0)
EOF
python3 mugpose.py "((-0.50,-0.30),(-0.26,-0.06),(0.95,1.12))" mp23 2>&1 | grep -E "z 1.085|z 0.995|z 1.005|z 1.095"

# openrua op 148
python3 -c "
from ctl import Robot; r=Robot('f'); r.snap('robot0_eye_in_hand','/workspace/final_eih.png'); print(r.hand_pose()[0].round(3))" 2>&1 | tail -1
