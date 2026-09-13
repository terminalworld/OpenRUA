#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la && find . -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
cat tools/action/fjt_send.py; echo ----; cat tools/action/gripper_cmd.py; echo ----; cat tools/action/ik_move.py

# openrua op 3
timeout 30 ros2 topic list; echo ----; timeout 20 ros2 action list; echo ----; timeout 20 ros2 service list | head -50; echo ----; timeout 20 ros2 node list

# openrua op 4
timeout 60 ros2 topic echo /joint_states --once; for c in agentview frontview sideview birdview robot0_eye_in_hand robot0_robotview; do timeout 60 python3 tools/perception/cam_snap.py $c; done

# openrua op 5
timeout 60 ros2 topic echo /tf_static --once --qos-durability transient_local 2>&1 | grep -E "frame_id|translation|rotation|x:|y:|z:|w:" | head -120

# openrua op 6
cat > /workspace/tfdump.py <<'EOF'
import rclpy, yaml
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
rclpy.init(); node = rclpy.create_node("tfdump")
seen = {}
def cb(m):
    for t in m.transforms:
        seen[(t.header.frame_id, t.child_frame_id)] = t
qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
node.create_subscription(TFMessage, "/tf_static", cb, qos)
node.create_subscription(TFMessage, "/tf", cb, 100)
import time
t0=time.time()
while time.time()-t0 < 5: rclpy.spin_once(node, timeout_sec=0.2)
for (p,c),t in sorted(seen.items()):
    tr=t.transform.translation; q=t.transform.rotation
    print(f"{p} -> {c}: t=({tr.x:.4f},{tr.y:.4f},{tr.z:.4f}) q=({q.x:.4f},{q.y:.4f},{q.z:.4f},{q.w:.4f})")
EOF
timeout 60 python3 /workspace/tfdump.py

# openrua op 7
cat > /workspace/pxw.py <<'EOF'
"""Usage: pxw.py <camera> u,v [u,v ...]  -> world xyz for each pixel (uses TF from world->camera optical frame)."""
import sys, struct, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy
def quat_R(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
def main():
    cam=sys.argv[1]; pxs=[tuple(map(int,a.split(","))) for a in sys.argv[2:]]
    rclpy.init(); node=rclpy.create_node("pxw")
    got={}
    node.create_subscription(Image, f"/{cam}/depth/image_raw", lambda m: got.setdefault("d",m),1)
    node.create_subscription(CameraInfo, f"/{cam}/color/camera_info", lambda m: got.setdefault("i",m),1)
    qos=QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    def tfcb(m):
        for t in m.transforms:
            if t.child_frame_id==f"{cam}_optical_frame" and t.header.frame_id=="world": got["t"]=t
    node.create_subscription(TFMessage,"/tf_static",tfcb,qos); node.create_subscription(TFMessage,"/tf",tfcb,100)
    while not all(k in got for k in "dit"): rclpy.spin_once(node,timeout_sec=0.2)
    d=got["d"]; info=got["i"]; t=got["t"]
    depth=np.frombuffer(d.data,dtype=np.float32).reshape(d.height,d.width)
    fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
    R=quat_R(t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w)
    T=np.array([t.transform.translation.x,t.transform.translation.y,t.transform.translation.z])
    for u,v in pxs:
        z=depth[v,u]
        p=R@np.array([(u-cx)*z/fx,(v-cy)*z/fy,z])+T
        print(f"px({u},{v}) depth={z:.4f} world=({p[0]:.4f},{p[1]:.4f},{p[2]:.4f})")
    rclpy.shutdown()
main()
EOF
timeout 60 python3 /workspace/pxw.py robot0_eye_in_hand 235,235 180,250 320,400 500,300 100,100 150,80 60,150

# openrua op 8
cat > /workspace/cloud.py <<'EOF'
"""Usage: cloud.py <camera> -> saves <camera>_cloud.npy (H,W,3) world xyz, and <camera>_rgb.npy"""
import sys, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy
def quat_R(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
def main():
    cam=sys.argv[1]
    rclpy.init(); node=rclpy.create_node("cloud")
    got={}
    node.create_subscription(Image, f"/{cam}/depth/image_raw", lambda m: got.setdefault("d",m),1)
    node.create_subscription(Image, f"/{cam}/color/image_raw", lambda m: got.setdefault("c",m),1)
    node.create_subscription(CameraInfo, f"/{cam}/color/camera_info", lambda m: got.setdefault("i",m),1)
    qos=QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    def tfcb(m):
        for t in m.transforms:
            if t.child_frame_id==f"{cam}_optical_frame" and t.header.frame_id=="world": got["t"]=t
    node.create_subscription(TFMessage,"/tf_static",tfcb,qos); node.create_subscription(TFMessage,"/tf",tfcb,100)
    while not all(k in got for k in "dict"): rclpy.spin_once(node,timeout_sec=0.2)
    d=got["d"]; info=got["i"]; t=got["t"]; c=got["c"]
    depth=np.frombuffer(d.data,dtype=np.float32).reshape(d.height,d.width)
    rgb=np.frombuffer(c.data,dtype=np.uint8).reshape(c.height,c.width,-1)
    fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
    R=quat_R(t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w)
    T=np.array([t.transform.translation.x,t.transform.translation.y,t.transform.translation.z])
    v,u=np.mgrid[0:d.height,0:d.width]
    pc=np.stack([(u-cx)*depth/fx,(v-cy)*depth/fy,depth],-1)
    pw=pc@R.T+T
    np.save(f"{cam}_cloud.npy",pw); np.save(f"{cam}_rgb.npy",rgb)
    print(cam, pw.shape, "enc", c.encoding)
    rclpy.shutdown()
main()
EOF
for c in robot0_eye_in_hand agentview birdview sideview frontview; do timeout 60 python3 /workspace/cloud.py $c; done

# openrua op 9
python3 - <<'EOF'
import numpy as np
for cam in ["robot0_eye_in_hand","agentview","birdview"]:
    P=np.load(f"{cam}_cloud.npy"); ok=np.isfinite(P).all(-1)
    print("==",cam)
    # table height: mode of z
    z=P[ok][:,2]; h,e=np.histogram(z,bins=np.arange(0.85,1.4,0.01)); 
    print("z hist:", [(round(e[i],2),int(h[i])) for i in np.argsort(h)[::-1][:8]])
    # bottle: points with z between 0.95 and 1.2, x in [-0.3,0], y in [-0.1,0.2]
    m=ok&(P[...,2]>0.93)&(P[...,2]<1.25)&(P[...,0]>-0.35)&(P[...,0]<0.0)&(P[...,1]>-0.1)&(P[...,1]<0.2)
    Q=P[m]
    if len(Q):
        print("candidate bottle pts",len(Q),"xy range",Q[:,:2].min(0),Q[:,:2].max(0),"zmax",Q[:,2].max())
        for zlo in np.arange(0.93,1.2,0.03):
            s=Q[(Q[:,2]>=zlo)&(Q[:,2]<zlo+0.03)]
            if len(s): print(f"  z[{zlo:.2f},{zlo+0.03:.2f}) n={len(s)} xmean={s[:,0].mean():.3f} ymean={s[:,1].mean():.3f} xr=({s[:,0].min():.3f},{s[:,0].max():.3f}) yr=({s[:,1].min():.3f},{s[:,1].max():.3f})")
EOF

# openrua op 10
python3 - <<'EOF'
import numpy as np
for cam in ["robot0_eye_in_hand","agentview","sideview","frontview"]:
    P=np.load(f"{cam}_cloud.npy"); ok=np.isfinite(P).all(-1)
    m=ok&(P[...,2]>0.905)&(P[...,2]<1.25)&(P[...,0]>-0.24)&(P[...,0]<-0.10)&(P[...,1]>0.0)&(P[...,1]<0.13)
    Q=P[m]; print("==",cam,len(Q))
    for zlo in np.arange(0.90,1.16,0.02):
        s=Q[(Q[:,2]>=zlo)&(Q[:,2]<zlo+0.02)]
        if len(s): print(f"  z[{zlo:.2f},{zlo+0.02:.2f}) n={len(s)} xmean={s[:,0].mean():.3f} ymean={s[:,1].mean():.3f} xr=({s[:,0].min():.3f},{s[:,0].max():.3f}) yr=({s[:,1].min():.3f},{s[:,1].max():.3f})")
EOF

# openrua op 11
python3 - <<'EOF'
import numpy as np
for cam in ["sideview","agentview","frontview"]:
    P=np.load(f"{cam}_cloud.npy"); ok=np.isfinite(P).all(-1)
    m=ok&(P[...,2]>0.85)&(P[...,2]<0.97)&(P[...,0]>-0.20)&(P[...,0]<-0.13)&(P[...,1]>0.03)&(P[...,1]<0.10)
    Q=P[m]; print("==",cam,len(Q))
    for zlo in np.arange(0.85,0.97,0.01):
        s=Q[(Q[:,2]>=zlo)&(Q[:,2]<zlo+0.01)]
        if len(s): print(f"  z[{zlo:.2f},{zlo+0.01:.2f}) n={len(s)} xmean={s[:,0].mean():.3f} ymean={s[:,1].mean():.3f} xr=({s[:,0].min():.3f},{s[:,0].max():.3f}) yr=({s[:,1].min():.3f},{s[:,1].max():.3f})")
EOF

# openrua op 12
python3 - <<'EOF'
import numpy as np
np.set_printoptions(precision=3, suppress=True)
P=np.load("agentview_cloud.npy"); ok=np.isfinite(P).all(-1)
# region y>0.1 (cabinet side)
m=ok&(P[...,1]>0.08)&(P[...,1]<0.6)&(P[...,0]>-0.5)&(P[...,0]<0.6)&(P[...,2]>0.85)
Q=P[m]
print("pts",len(Q))
# 2D occupancy in x,y of max z
xs=np.arange(-0.45,0.55,0.05); ys=np.arange(0.08,0.6,0.05)
print("max z grid (rows=x, cols=y)")
print("      "+" ".join(f"{y:5.2f}" for y in ys))
for x in xs:
    row=[]
    for y in ys:
        s=Q[(Q[:,0]>=x)&(Q[:,0]<x+0.05)&(Q[:,1]>=y)&(Q[:,1]<y+0.05)]
        row.append(f"{s[:,2].max():5.2f}" if len(s) else "  .  ")
    print(f"{x:5.2f} "+" ".join(row))
EOF

# openrua op 13
python3 - <<'EOF'
import numpy as np
def quat_R(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
for name,q in [("agentview",(0.6380,0.6380,-0.3048,-0.3048)),("birdview",(0.7071,0.7071,0,0)),("eye",(0.7070,-0.7066,-0.0201,0.0201)),("sideview",(0.0099,0.8064,-0.5912,-0.0069))]:
    R=quat_R(*q); print(name,"optical x(img right)->world",R[:,0].round(3),"y(img down)->",R[:,1].round(3),"z(fwd)->",R[:,2].round(3))
EOF

# openrua op 14
python3 - <<'EOF'
import numpy as np
for cam in ["robot0_eye_in_hand","agentview"]:
    P=np.load(f"{cam}_cloud.npy"); ok=np.isfinite(P).all(-1)
    m=ok&(P[...,1]>0.0)&(P[...,1]<0.5)&(P[...,0]>-0.25)&(P[...,0]<0.25)&(P[...,2]>0.85)
    Q=P[m]
    print("==",cam,"max z grid (rows=x, cols=y), step 0.02")
    xs=np.arange(-0.22,0.22,0.02); ys=np.arange(0.0,0.46,0.02)
    print("      "+" ".join(f"{y:5.2f}" for y in ys))
    for x in xs:
        row=[]
        for y in ys:
            s=Q[(Q[:,0]>=x)&(Q[:,0]<x+0.02)&(Q[:,1]>=y)&(Q[:,1]<y+0.02)]
            row.append(f"{s[:,2].max():5.2f}" if len(s) else "  .  ")
        print(f"{x:5.2f} "+" ".join(row))
EOF

# openrua op 15
python3 - <<'EOF'
import numpy as np
P=np.load("robot0_eye_in_hand_cloud.npy"); ok=np.isfinite(P).all(-1)
# drawer floor
m=ok&(P[...,2]>0.915)&(P[...,2]<0.93)&(P[...,1]>0.0)&(P[...,1]<0.3)&(P[...,0]>-0.2)&(P[...,0]<0.2)
Q=P[m]; print("floor pts",len(Q),"x",Q[:,0].min(),Q[:,0].max(),"y",Q[:,1].min(),Q[:,1].max(), "z", Q[:,2].mean())
# drawer walls z 0.96-0.99
m=ok&(P[...,2]>0.96)&(P[...,2]<0.995)&(P[...,1]>0.0)&(P[...,1]<0.3)&(P[...,0]>-0.2)&(P[...,0]<0.2)
Q=P[m]; print("wall-top pts",len(Q))
for x in np.arange(-0.14,0.16,0.01):
    s=Q[(Q[:,0]>=x)&(Q[:,0]<x+0.01)]
    if len(s): print(f"x[{x:.2f}] n={len(s)} y=({s[:,1].min():.3f},{s[:,1].max():.3f})")
print("by y")
for y in np.arange(0.0,0.3,0.01):
    s=Q[(Q[:,1]>=y)&(Q[:,1]<y+0.01)]
    if len(s): print(f"y[{y:.2f}] n={len(s)} x=({s[:,0].min():.3f},{s[:,0].max():.3f})")
# handle
m=ok&(P[...,2]>0.93)&(P[...,2]<0.96)&(P[...,1]>0.0)&(P[...,1]<0.08)&(P[...,0]>-0.1)&(P[...,0]<0.1)
Q=P[m]; print("handle pts",len(Q),"x",Q[:,0].min(),Q[:,0].max(),"y",Q[:,1].min(),Q[:,1].max(),"z",Q[:,2].min(),Q[:,2].max())
# cabinet front face: points with y in 0.18-0.22 
m=ok&(P[...,1]>0.17)&(P[...,1]<0.23)&(P[...,0]>-0.15)&(P[...,0]<0.15)&(P[...,2]>0.9)
Q=P[m]; print("cabinet-front-ish pts",len(Q))
for z in np.arange(0.9,1.15,0.01):
    s=Q[(Q[:,2]>=z)&(Q[:,2]<z+0.01)]
    if len(s): print(f"z[{z:.2f}] n={len(s)} y=({s[:,1].min():.3f},{s[:,1].max():.3f}) x=({s[:,0].min():.3f},{s[:,0].max():.3f})")
EOF

# openrua op 16
timeout 300 python3 tools/action/gripper_cmd.py 0.04; timeout 60 ros2 topic echo /joint_states --once | grep -A9 position

# openrua op 17
timeout 60 python3 /workspace/cloud.py sideview && timeout 60 python3 /workspace/cloud.py robot0_eye_in_hand && python3 - <<'EOF'
import numpy as np
for cam in ["sideview","robot0_eye_in_hand"]:
    P=np.load(f"{cam}_cloud.npy"); ok=np.isfinite(P).all(-1)
    m=ok&(P[...,2]>0.85)&(P[...,2]<1.25)&(P[...,0]>-0.20)&(P[...,0]<-0.13)&(P[...,1]>0.03)&(P[...,1]<0.10)
    Q=P[m]; print("==",cam,len(Q))
    for zlo in np.arange(0.89,1.16,0.01):
        s=Q[(Q[:,2]>=zlo)&(Q[:,2]<zlo+0.01)]
        if len(s): print(f"  z[{zlo:.2f}] n={len(s)} xmean={s[:,0].mean():.3f} ymean={s[:,1].mean():.3f} xr=({s[:,0].min():.3f},{s[:,0].max():.3f}) yr=({s[:,1].min():.3f},{s[:,1].max():.3f})")
EOF

# openrua op 18
timeout 60 python3 - <<'EOF'
import rclpy
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy
rclpy.init(); node=rclpy.create_node("urdf")
got=[]
node.create_subscription(String,"/robot_description",got.append,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
while not got: rclpy.spin_once(node,timeout_sec=0.5)
open("robot.urdf","w").write(got[0].data); print(len(got[0].data))
EOF
grep -n -A12 'link name="panda_hand"' robot.urdf | head -60; grep -n -B2 -A12 'link name="panda_leftfinger"' robot.urdf | head -50

# openrua op 19
python3 - <<'EOF'
import numpy as np, struct
def load_stl(p):
    d=open(p,'rb').read()
    n=struct.unpack_from('<I',d,80)[0]
    arr=np.frombuffer(d[84:84+n*50],dtype=np.dtype([('n','<f4',3),('v','<f4',(3,3)),('a','<u2')]))
    return arr['v'].reshape(-1,3)
for f in ["hand","finger"]:
    v=load_stl(f"/opt/ros/jazzy/share/moveit_resources_panda_description/meshes/collision/{f}.stl")
    print(f, "min",v.min(0).round(4),"max",v.max(0).round(4))
EOF

# openrua op 20
find / -iname "*hand*.stl" -o -iname "*finger*.stl" -o -iname "hand.dae" -o -iname "finger.dae" 2>/dev/null | head; find / -path /proc -prune -o -iname "*panda*" -print 2>/dev/null | grep -v python | head -20

# openrua op 21
cat > /workspace/rob.py <<'EOF'
"""Reusable helpers: joint state, FK, IK, trajectory, gripper. Import or run functions."""
import math, time, sys
import numpy as np, rclpy, yaml
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import WrenchStamped

M = yaml.safe_load(open("/workspace/machine.yaml"))
ARM = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
JOINTS = ARM["joints"]
BASE = np.array([-0.66, 0.0, 0.912])  # world -> panda_link0

class Robot:
    def __init__(self):
        if not rclpy.ok(): rclpy.init()
        self.node = rclpy.create_node("rob_" + str(int(time.time()*1000) % 100000))
        self._js = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self._wr = None
        self.node.create_subscription(WrenchStamped, "/franka_robot_state_broadcaster/external_wrench", self._on_wr, 10)
        self.ik = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.traj = ActionClient(self.node, FollowJointTrajectory, ARM["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik.wait_for_service(10); self.fk.wait_for_service(10)
        self.traj.wait_for_server(10); self.grip.wait_for_server(10)
    def _on_js(self, m): self._js = m
    def _on_wr(self, m): self._wr = m
    def spin(self, t=0.1):
        end = time.time() + t
        while time.time() < end: rclpy.spin_once(self.node, timeout_sec=0.05)
    def joints(self, fresh=True):
        if fresh:
            self._js = None
            while self._js is None: rclpy.spin_once(self.node, timeout_sec=0.2)
        d = dict(zip(self._js.name, self._js.position))
        return d
    def arm_q(self):
        d = self.joints(); return [d[j] for j in JOINTS]
    def fingers(self):
        d = self.joints(); return d["panda_finger_joint1"], d["panda_finger_joint2"]
    def wrench(self):
        self._wr = None
        while self._wr is None: rclpy.spin_once(self.node, timeout_sec=0.2)
        w = self._wr.wrench
        return np.array([w.force.x, w.force.y, w.force.z, w.torque.x, w.torque.y, w.torque.z])
    def fk_hand(self, q=None, link="panda_hand"):
        """returns (pos_world, quat xyzw) of link for arm config q (default current)."""
        q = q if q is not None else self.arm_q()
        req = GetPositionFK.Request()
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = list(JOINTS)
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {None if res is None else res.error_code.val}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z])
        # planner frame is arm base; convert to world
        pos_w = pos + BASE
        return pos_w, np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]), res.pose_stamped[0].header.frame_id
    def ik_hand(self, pos_world, quat_xyzw, seed=None, attempts=3):
        """IK for hand frame at world pose. Returns joint list or None."""
        pos = np.asarray(pos_world, float) - BASE
        seed = seed if seed is not None else self.arm_q()
        for k in range(attempts):
            req = GetPositionIK.Request()
            req.ik_request.group_name = M["planning"]["group"]
            req.ik_request.pose_stamped.header.frame_id = ""
            p = req.ik_request.pose_stamped.pose
            p.position.x, p.position.y, p.position.z = map(float, pos)
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat_xyzw)
            req.ik_request.robot_state.joint_state.name = list(JOINTS)
            req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
            req.ik_request.avoid_collisions = False
            req.ik_request.timeout = Duration(sec=1)
            fut = self.ik.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            res = fut.result()
            if res is not None and res.error_code.val == 1:
                sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
                return [sol[j] for j in JOINTS]
            # perturb seed
            seed = list(np.array(seed) + np.random.uniform(-0.3, 0.3, len(seed)))
        return None
    def move_q(self, q, seconds=3.0, waypoints=None):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        pts = []
        wps = (waypoints or []) + [(q, seconds)]
        for qq, t in wps:
            pt = JointTrajectoryPoint(positions=[float(v) for v in qq])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            pts.append(pt)
        goal.trajectory.points = pts
        send = self.traj.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=600)
        code = rf.result().result.error_code
        qa = np.array(self.arm_q()); err = np.abs(qa - np.array(q)).max()
        return code, err
    def move_pose(self, pos_world, quat_xyzw, seconds=3.0, seed=None):
        q = self.ik_hand(pos_world, quat_xyzw, seed=seed)
        if q is None:
            return None, None, None
        code, err = self.move_q(q, seconds)
        return q, code, err
    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP.get("max_effort", 30.0))
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result
        return r.reached_goal, r.stalled, self.fingers()

def quat_from_R(R):
    """xyzw from rotation matrix"""
    m = R; t = np.trace(m)
    if t > 0:
        s = math.sqrt(t + 1) * 2; w = 0.25 * s; x = (m[2,1]-m[1,2])/s; y = (m[0,2]-m[2,0])/s; z = (m[1,0]-m[0,1])/s
    elif m[0,0] > m[1,1] and m[0,0] > m[2,2]:
        s = math.sqrt(1 + m[0,0]-m[1,1]-m[2,2])*2; w=(m[2,1]-m[1,2])/s; x=0.25*s; y=(m[0,1]+m[1,0])/s; z=(m[0,2]+m[2,0])/s
    elif m[1,1] > m[2,2]:
        s = math.sqrt(1 + m[1,1]-m[0,0]-m[2,2])*2; w=(m[0,2]-m[2,0])/s; x=(m[0,1]+m[1,0])/s; y=0.25*s; z=(m[1,2]+m[2,1])/s
    else:
        s = math.sqrt(1 + m[2,2]-m[0,0]-m[1,1])*2; w=(m[1,0]-m[0,1])/s; x=(m[0,2]+m[2,0])/s; y=(m[1,2]+m[2,1])/s; z=0.25*s
    return np.array([x, y, z, w])

def R_from_quat(q):
    x, y, z, w = q
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

def hand_R(approach, closing):
    """Rotation matrix with hand z = approach dir, hand y = closing dir (world vectors)."""
    a = np.asarray(approach, float); a /= np.linalg.norm(a)
    c = np.asarray(closing, float); c -= a * (c @ a); c /= np.linalg.norm(c)
    x = np.cross(c, a)
    return np.stack([x, c, a], axis=1)

if __name__ == "__main__":
    r = Robot()
    print("q", np.round(r.arm_q(), 4))
    print("fingers", r.fingers())
    pos, quat, frame = r.fk_hand()
    print("hand world pos", pos.round(4), "quat", quat.round(4), "frame", frame)
    R = R_from_quat(quat)
    print("hand x", R[:,0].round(3), "y", R[:,1].round(3), "z", R[:,2].round(3))
    print("wrench", r.wrench().round(3))
EOF
timeout 120 python3 /workspace/rob.py

# openrua op 22
timeout 120 python3 - <<'EOF'
import numpy as np
from rob import *
r=Robot()
q0=np.array(r.arm_q())
pos,quat,_=r.fk_hand()
raw=pos-BASE  # what FK actually returned
print("FK raw",raw.round(4))
# IK with raw pos (as returned), frame empty
r2=r.ik_hand(raw+BASE, quat)   # ik_hand subtracts BASE -> sends raw
print("IK(raw):", None if r2 is None else np.round(r2,3), "diff", None if r2 is None else np.abs(np.array(r2)-q0).max())
# IK with raw - BASE (i.e., interpreting raw as world and converting to base)
r3=r.ik_hand(raw, quat)
print("IK(raw-BASE):", None if r3 is None else np.round(r3,3), "diff", None if r3 is None else np.abs(np.array(r3)-q0).max())
EOF

# openrua op 23
timeout 120 python3 - <<'EOF'
import numpy as np
from rob import *
r=Robot()
q0=np.array(r.arm_q())
pos,quat,_=r.fk_hand()
sol=[0.058,-0.161,-0.058,-2.445,-0.012,2.227,0.009]
p2,q2,_=r.fk_hand(sol)
print("cur pose", (pos-BASE).round(4), quat.round(4)); print("sol pose", (p2-BASE).round(4), q2.round(4))
for L in ["panda_link8","panda_hand","panda_leftfinger","panda_rightfinger"]:
    p,q,_=r.fk_hand(link=L); print(L,(p-BASE).round(4),q.round(4))
EOF

# openrua op 24
python3 - <<'EOF'
import re
s=open("rob.py").read()
s=s.replace('req.ik_request.group_name = M["planning"]["group"]\n', 'req.ik_request.group_name = M["planning"]["group"]\n            req.ik_request.ik_link_name = "panda_hand"\n')
# FK returns world coords already: fix
s=s.replace("        pos_w = pos + BASE\n","        pos_w = pos  # service already answers in world coords (verified against camera TF)\n")
s=s.replace("        pos = np.asarray(pos_world, float) - BASE\n","        pos = np.asarray(pos_world, float)  # world coords (verified: FK/IK operate in world)\n")
open("rob.py","w").write(s)
EOF
timeout 120 python3 - <<'EOF'
import numpy as np
from rob import *
r=Robot()
q0=np.array(r.arm_q())
pos,quat,_=r.fk_hand()
sol=r.ik_hand(pos,quat)
print("sol",np.round(sol,3),"diff",np.abs(np.array(sol)-q0).max())
p2,q2,_=r.fk_hand(sol); print("sol pose",p2.round(4),q2.round(4))
EOF

# openrua op 25
python3 - <<'EOF'
import numpy as np
P=np.load("robot0_eye_in_hand_cloud.npy"); ok=np.isfinite(P).all(-1)
# handle bar: y<0.05, z 0.925-0.97
m=ok&(P[...,2]>0.925)&(P[...,2]<0.975)&(P[...,1]>0.0)&(P[...,1]<0.066)&(P[...,0]>-0.15)&(P[...,0]<0.15)
Q=P[m]; print("handle pts",len(Q))
for y in np.arange(0.03,0.07,0.005):
    s=Q[(Q[:,1]>=y)&(Q[:,1]<y+0.005)]
    if len(s): print(f"y[{y:.3f}] n={len(s)} x=({s[:,0].min():.3f},{s[:,0].max():.3f}) z=({s[:,2].min():.3f},{s[:,2].max():.3f})")
print("bar x-hist")
bar=Q[(Q[:,1]<0.048)]
h,e=np.histogram(bar[:,0],bins=np.arange(-0.12,0.12,0.01)); print([(round(e[i],2),int(h[i])) for i in range(len(h)) if h[i]>0])
# front panel top: z>0.97, y<0.09
m=ok&(P[...,2]>0.97)&(P[...,2]<1.0)&(P[...,1]>0.05)&(P[...,1]<0.09)
Q=P[m]; print("panel top", len(Q), "y",Q[:,1].min().round(3),Q[:,1].max().round(3),"x",Q[:,0].min().round(3),Q[:,0].max().round(3),"z",Q[:,2].mean().round(3))
EOF

# openrua op 26
python3 - <<'EOF'
import numpy as np
P=np.load("robot0_eye_in_hand_cloud.npy"); ok=np.isfinite(P).all(-1)
m=ok&(P[...,2]>1.10)&(P[...,2]<1.27)&(P[...,0]>-0.3)&(P[...,0]<-0.1)
Q=P[m]; print(len(Q))
for z in np.arange(1.10,1.27,0.01):
    s=Q[(Q[:,2]>=z)&(Q[:,2]<z+0.01)]
    if len(s): print(f"z[{z:.2f}] n={len(s)} x=({s[:,0].min():.3f},{s[:,0].max():.3f}) y=({s[:,1].min():.3f},{s[:,1].max():.3f})")
EOF

# openrua op 27
cat > /workspace/step1_hook.py <<'EOF'
import numpy as np, sys
from rob import *
r=Robot()
print("close gripper:", r.gripper(0.0))
R=hand_R((0,0,-1),(1,0,0)); q=quat_from_R(R)
print("target quat",q.round(4))
def go(p, secs=3.0, seed=None):
    sol=r.ik_hand(p,q,seed=seed)
    if sol is None: print("IK FAIL", p); sys.exit(1)
    code,err=r.move_q(sol,secs)
    pos,_,_=r.fk_hand()
    print(f"go {np.round(p,3)} -> code={code} jerr={err:.4f} hand={pos.round(4)}")
    return sol
go([0.007,0.105,1.20],4.0)
go([0.007,0.105,1.058],3.0)
w0=r.wrench(); print("wrench before pull",w0.round(2))
for y in [0.08,0.055,0.03,0.005,-0.02]:
    go([0.007,y,1.058],2.0)
    print("  wrench",r.wrench().round(2))
go([0.007,-0.02,1.20],3.0)
print("fingers",r.fingers())
EOF
timeout 1500 python3 -u /workspace/step1_hook.py

# openrua op 28
timeout 60 python3 tools/perception/cam_snap.py agentview && timeout 60 python3 /workspace/cloud.py agentview && python3 - <<'EOF'
import numpy as np
P=np.load("agentview_cloud.npy"); ok=np.isfinite(P).all(-1)
m=ok&(P[...,2]>0.97)&(P[...,2]<1.0)&(P[...,1]>0.0)&(P[...,1]<0.12)&(P[...,0]>-0.12)&(P[...,0]<0.13)
Q=P[m]; print("panel top", len(Q), "y",Q[:,1].min().round(3),Q[:,1].max().round(3))
m=ok&(P[...,2]>0.93)&(P[...,2]<0.96)&(P[...,1]>-0.05)&(P[...,1]<0.07)&(P[...,0]>-0.05)&(P[...,0]<0.06)
Q=P[m]; print("bar", len(Q), "y",Q[:,1].min().round(3),Q[:,1].max().round(3))
EOF

# openrua op 29
python3 - <<'EOF'
import numpy as np
P=np.load("agentview_cloud.npy"); ok=np.isfinite(P).all(-1)
# right wall: x 0.08..0.14, z 0.95-0.99
m=ok&(P[...,2]>0.95)&(P[...,2]<0.995)&(P[...,1]>0.09)&(P[...,1]<0.2)&(P[...,0]>0.06)&(P[...,0]<0.16)
Q=P[m]; print("right wall top pts",len(Q))
h,e=np.histogram(Q[:,0],bins=np.arange(0.06,0.16,0.005)); print([(round(e[i],3),int(h[i])) for i in range(len(h)) if h[i]>0])
# left wall
m=ok&(P[...,2]>0.95)&(P[...,2]<0.995)&(P[...,1]>0.09)&(P[...,1]<0.2)&(P[...,0]>-0.16)&(P[...,0]<-0.06)
Q=P[m]; print("left wall top pts",len(Q))
h,e=np.histogram(Q[:,0],bins=np.arange(-0.16,-0.06,0.005)); print([(round(e[i],3),int(h[i])) for i in range(len(h)) if h[i]>0])
# floor extents at z 0.915-0.935 for y in 0.09..0.2
m=ok&(P[...,2]>0.915)&(P[...,2]<0.935)&(P[...,1]>0.09)&(P[...,1]<0.2)
Q=P[m]; print("floor x range",Q[:,0].min().round(3),Q[:,0].max().round(3), "y max", Q[:,1].max().round(3))
# cabinet lip: points z 0.985-1.0 with y 0.18..0.26
m=ok&(P[...,2]>0.985)&(P[...,2]<1.02)&(P[...,1]>0.15)&(P[...,1]<0.3)&(P[...,0]>-0.1)&(P[...,0]<0.15)
Q=P[m]; print("lip pts",len(Q))
h,e=np.histogram(Q[:,1],bins=np.arange(0.15,0.3,0.005)); print([(round(e[i],3),int(h[i])) for i in range(len(h)) if h[i]>0])
# cabinet front face x extents at z 1.05-1.1
m=ok&(P[...,2]>1.05)&(P[...,2]<1.10)&(P[...,1]>0.15)&(P[...,1]<0.3)
Q=P[m]; print("cab front x",Q[:,0].min().round(3),Q[:,0].max().round(3),"y",Q[:,1].min().round(3),Q[:,1].max().round(3))
EOF

# openrua op 30
cat > /workspace/step2_grasp.py <<'EOF'
import numpy as np, sys
from rob import *
r=Robot()
print("open gripper:", r.gripper(0.04))
bottle=np.array([-0.165,0.061])
d=np.array([-0.823,-0.568,0.0])
Rg=hand_R((0,0,-1),d); qg=quat_from_R(Rg)
print("grasp R\n",Rg.round(3))
def go(p, q, secs=3.0):
    sol=r.ik_hand(p,q)
    if sol is None: print("IK FAIL", p); sys.exit(1)
    code,err=r.move_q(sol,secs)
    pos,quat,_=r.fk_hand()
    print(f"go {np.round(p,3)} -> code={code} jerr={err:.4f} hand={pos.round(4)} q={quat.round(3)}")
    return sol
go([bottle[0],bottle[1],1.30],qg,4.0)
go([bottle[0],bottle[1],1.14],qg,3.0)
print("close:", r.gripper(0.0))
r.spin(0.5); print("fingers", r.fingers())
EOF
timeout 1500 python3 -u /workspace/step2_grasp.py

# openrua op 31
python3 - <<'EOF'
s=open("/workspace/rob.py").read()
s=s.replace("""    def move_pose(self, pos_world, quat_xyzw, seconds=3.0, seed=None):""","""    def move_q_conv(self, q, seconds=3.0, tol=0.02, retries=4):
        code, err = self.move_q(q, seconds)
        n = 0
        while err > tol and n < retries:
            code, err = self.move_q(q, max(1.5, seconds/2)); n += 1
        return code, err, n
    def move_pose(self, pos_world, quat_xyzw, seconds=3.0, seed=None):""")
open("/workspace/rob.py","w").write(s)
EOF
cat > /workspace/step2b.py <<'EOF'
import numpy as np, sys
from rob import *
r=Robot()
q0=np.array(r.arm_q()); print("q now",q0.round(3))
pos,quat,_=r.fk_hand(); print("hand",pos.round(4),quat.round(3))
print("open:", r.gripper(0.04))
bottle=np.array([-0.165,0.061])
d=np.array([-0.823,-0.568,0.0])
Rg=hand_R((0,0,-1),d); qg=quat_from_R(Rg)
def go(p, q, secs=3.0):
    sol=r.ik_hand(p,q)
    if sol is None: print("IK FAIL", p); sys.exit(1)
    print("  sol",np.round(sol,3))
    code,err,n=r.move_q_conv(sol,secs)
    pos,quat,_=r.fk_hand()
    print(f"go {np.round(p,3)} -> code={code} jerr={err:.4f} retries={n} hand={pos.round(4)} q={quat.round(3)}")
    return sol
go([bottle[0],bottle[1],1.30],qg,4.0)
go([bottle[0],bottle[1],1.14],qg,3.0)
print("close:", r.gripper(0.0))
r.spin(0.5); print("fingers", r.fingers())
EOF
timeout 1700 python3 -u /workspace/step2b.py

# openrua op 32
cat > /workspace/step3_lift.py <<'EOF'
import numpy as np, sys
from rob import *
r=Robot()
d=np.array([-0.823,-0.568,0.0])
Rg=hand_R((0,0,-1),d); qg=quat_from_R(Rg)
sol=r.ik_hand([-0.165,0.061,1.30],qg)
code,err,n=r.move_q_conv(sol,3.0)
pos,quat,_=r.fk_hand(); print(f"lift code={code} jerr={err:.4f} retries={n} hand={pos.round(4)} fingers={r.fingers()}")
EOF
timeout 1200 python3 -u /workspace/step3_lift.py && timeout 60 python3 tools/perception/cam_snap.py agentview && timeout 60 python3 tools/perception/cam_snap.py sideview

# openrua op 33
cat > /workspace/step4_hold.py <<'EOF'
import numpy as np, sys, math
from rob import *
r=Robot()
d=np.array([-0.823,-0.568,0.0]); d/=np.linalg.norm(d)
z=np.array([0,0,1.0])
u=(d*math.sin(math.radians(45))+z*math.cos(math.radians(45)))  # base->cork axis when tilted
a1=-u
a0=np.array([0,0,-1.0])
# rotation taking a0 -> a1
n=np.cross(a0,a1); s=np.linalg.norm(n); n/=s; th=math.atan2(s,a0@a1)
K=np.array([[0,-n[2],n[1]],[n[2],0,-n[0]],[-n[1],n[0],0]])
Rt=np.eye(3)+math.sin(th)*K+(1-math.cos(th))*K@K
c1=Rt@d
print("u",u.round(3),"a1",a1.round(3),"c1",c1.round(3),"n",n.round(3),"th",math.degrees(th))
Rh=hand_R(a1,c1); qh=quat_from_R(Rh)
B=np.array([0.085,0.195,0.943])
H=B+0.235*u
print("H",H.round(4))
def go(p,q,secs):
    sol=r.ik_hand(p,q)
    if sol is None: print("IK FAIL",p); return None
    print("  sol",np.round(sol,3))
    code,err,n_=r.move_q_conv(sol,secs)
    pos,quat,_=r.fk_hand()
    print(f"go {np.round(p,3)} -> code={code} jerr={err:.4f} retries={n_} hand={pos.round(4)} q={quat.round(3)} fingers={np.round(r.fingers(),4)}")
    return sol
if go(H+[0,0,0.06],qh,5.0) is None: sys.exit(1)
EOF
timeout 1500 python3 -u /workspace/step4_hold.py

# openrua op 34
timeout 60 python3 tools/perception/cam_snap.py agentview && timeout 60 python3 tools/perception/cam_snap.py frontview

# openrua op 35
timeout 60 python3 /workspace/cloud.py agentview && python3 - <<'EOF'
import numpy as np
P=np.load("agentview_cloud.npy"); rgb=np.load("agentview_rgb.npy"); ok=np.isfinite(P).all(-1)
# bottle: dark green/black pixels inside drawer region
dark=(rgb.max(-1)<90)
m=ok&dark&(P[...,0]>-0.12)&(P[...,0]<0.14)&(P[...,1]>0.04)&(P[...,1]<0.25)&(P[...,2]>0.91)&(P[...,2]<1.0)
Q=P[m]; print("dark pts in drawer region",len(Q))
print("x",Q[:,0].min().round(3),Q[:,0].max().round(3),"y",Q[:,1].min().round(3),Q[:,1].max().round(3),"z",Q[:,2].min().round(3),Q[:,2].max().round(3))
# all points above floor in interior (excluding walls) 
m=ok&(P[...,0]>-0.085)&(P[...,0]<0.108)&(P[...,1]>0.085)&(P[...,1]<0.215)&(P[...,2]>0.93)&(P[...,2]<1.0)
Q=P[m]; print("interior pts above floor",len(Q),"x",Q[:,0].min().round(3),Q[:,0].max().round(3),"y",Q[:,1].min().round(3),Q[:,1].max().round(3),"z",Q[:,2].min().round(3),Q[:,2].max().round(3))
# cork (tan) pixels
tan=(rgb[...,0]>150)&(rgb[...,1]>100)&(rgb[...,2]<120)&(rgb[...,0]>rgb[...,2]+50)
m=ok&tan&(P[...,0]>-0.12)&(P[...,0]<0.14)&(P[...,1]>0.0)&(P[...,1]<0.25)&(P[...,2]>0.91)&(P[...,2]<1.0)
Q=P[m]; print("cork pts",len(Q), Q.mean(0).round(3) if len(Q) else None)
EOF

# openrua op 36
python3 - <<'EOF'
import numpy as np
P=np.load("agentview_cloud.npy"); rgb=np.load("agentview_rgb.npy"); ok=np.isfinite(P).all(-1)
m=ok&(P[...,0]>-0.085)&(P[...,0]<0.108)&(P[...,1]>0.083)&(P[...,1]<0.215)&(P[...,2]>0.93)&(P[...,2]<0.985)
Q=P[m]; C=rgb[m]
print(len(Q))
for z in np.arange(0.93,0.985,0.01):
    s=Q[(Q[:,2]>=z)&(Q[:,2]<z+0.01)]; c=C[(Q[:,2]>=z)&(Q[:,2]<z+0.01)]
    if len(s): print(f"z[{z:.2f}] n={len(s)} x=({s[:,0].min():.3f},{s[:,0].max():.3f}) y=({s[:,1].min():.3f},{s[:,1].max():.3f}) rgb_mean={c.mean(0).round(0)}")
# PCA of the bottle points (dark ones: max channel < 120)
dark=C.max(-1)<120
B=Q[dark]; print("dark",len(B),"x",B[:,0].min().round(3),B[:,0].max().round(3),"y",B[:,1].min().round(3),B[:,1].max().round(3))
c=B[:,:2].mean(0); u,s,vt=np.linalg.svd(B[:,:2]-c); print("center",c.round(3),"dir",vt[0].round(3),"extent along dir",((B[:,:2]-c)@vt[0]).min().round(3),((B[:,:2]-c)@vt[0]).max().round(3))
EOF

# openrua op 37
python3 - <<'EOF'
import numpy as np
P=np.load("agentview_cloud.npy"); ok=np.isfinite(P).all(-1)
m=ok&(P[...,2]>0.915)&(P[...,2]<1.0)&(P[...,1]>-0.4)&(P[...,1]<0.03)&(P[...,0]>-0.4)&(P[...,0]<0.4)
Q=P[m]; print("objects near y<0.03:",len(Q))
xs=np.arange(-0.4,0.4,0.05); ys=np.arange(-0.4,0.03,0.05)
print("      "+" ".join(f"{y:5.2f}" for y in ys))
for x in xs:
    row=[]
    for y in ys:
        s=Q[(Q[:,0]>=x)&(Q[:,0]<x+0.05)&(Q[:,1]>=y)&(Q[:,1]<y+0.05)]
        row.append(f"{s[:,2].max():5.2f}" if len(s) else "  .  ")
    print(f"{x:5.2f} "+" ".join(row))
EOF

# openrua op 38
python3 - <<'EOF'
import numpy as np
P=np.load("agentview_cloud.npy"); ok=np.isfinite(P).all(-1)
m=ok&(P[...,2]>0.915)&(P[...,2]<0.98)&(P[...,1]>-0.2)&(P[...,1]<0.03)&(P[...,0]>-0.08)&(P[...,0]<0.15)
Q=P[m]; print("bowl pts",len(Q),"x",Q[:,0].min().round(3),Q[:,0].max().round(3),"y",Q[:,1].min().round(3),Q[:,1].max().round(3),"zmax",Q[:,2].max().round(3))
c=Q[:,:2].mean(0); print("center approx",c.round(3))
for y in np.arange(-0.03,0.03,0.01):
    s=Q[(Q[:,1]>=y)&(Q[:,1]<y+0.01)]
    if len(s): print(f"y[{y:.2f}] n={len(s)} x=({s[:,0].min():.3f},{s[:,0].max():.3f}) z=({s[:,2].min():.3f},{s[:,2].max():.3f})")
EOF

# openrua op 39
cat > /workspace/step5_close.py <<'EOF'
import numpy as np, sys, math
from rob import *
r=Robot()
print("fingers",r.fingers())
a=np.array([0,math.sin(math.radians(45)),-math.cos(math.radians(45))])
Rp=hand_R(a,(1,0,0)); qp=quat_from_R(Rp)
TCP_OFF=0.1034
def hand_for_tcp(tcp): return np.array(tcp)-TCP_OFF*a
def go(p,q,secs,tol=0.02):
    sol=r.ik_hand(p,q)
    if sol is None: print("IK FAIL",p); return None
    code,err,n_=r.move_q_conv(sol,secs,tol=tol,retries=2)
    pos,quat,_=r.fk_hand()
    print(f"go {np.round(p,3)} -> code={code} jerr={err:.4f} retries={n_} hand={pos.round(4)}")
    return pos
# 1. lift straight up from current pose
pos,quat,_=r.fk_hand()
go(pos+[0,0,0.10],quat,3.0)
# 2. pre-push above start, pitched
x0=-0.025
go(hand_for_tcp([x0,0.02,1.10]),qp,5.0)
# 3. descend
go(hand_for_tcp([x0,0.02,0.9475]),qp,3.0)
w0=r.wrench(); print("wrench0",w0.round(2))
# 4. push in steps
for y in np.arange(0.05,0.215,0.03):
    hp=go(hand_for_tcp([x0,y,0.9475]),qp,2.5,tol=0.05)
    if hp is None: break
    tcp=hp+TCP_OFF*a
    w=r.wrench(); print(f"   tcp_y={tcp[1]:.4f} (cmd {y:.3f}) wrench={w.round(2)}")
    if y-tcp[1] > 0.02:
        print("   stalled -> drawer likely closed"); break
# 5. retreat: back and up
hp=r.fk_hand()[0]
go(hp+[0,-0.03,0.10],qp,3.0)
EOF
timeout 1700 python3 -u /workspace/step5_close.py

# openrua op 40
timeout 60 python3 tools/perception/cam_snap.py agentview && timeout 60 python3 tools/perception/cam_snap.py frontview

# openrua op 41
timeout 60 python3 /workspace/cloud.py agentview && timeout 60 python3 /workspace/cloud.py frontview && python3 - <<'EOF'
import numpy as np
for cam in ["agentview","frontview"]:
    P=np.load(f"{cam}_cloud.npy"); rgb=np.load(f"{cam}_rgb.npy"); ok=np.isfinite(P).all(-1)
    print("==",cam)
    # greenish/dark bottle pixels: g > r+5 and dark
    g=(rgb[...,1].astype(int)>rgb[...,0].astype(int)+3)&(rgb.max(-1)<110)
    m=ok&g&(P[...,0]>-0.15)&(P[...,0]<0.15)&(P[...,1]>0.0)&(P[...,1]<0.35)&(P[...,2]>0.9)&(P[...,2]<1.2)
    Q=P[m]; print("bottle-ish pts",len(Q))
    if len(Q):
        print(" x",Q[:,0].min().round(3),Q[:,0].max().round(3),"y",Q[:,1].min().round(3),Q[:,1].max().round(3),"z",Q[:,2].min().round(3),Q[:,2].max().round(3))
        for z in np.arange(0.9,1.2,0.02):
            s=Q[(Q[:,2]>=z)&(Q[:,2]<z+0.02)]
            if len(s)>5: print(f"  z[{z:.2f}] n={len(s)} x=({s[:,0].min():.3f},{s[:,0].max():.3f}) y=({s[:,1].min():.3f},{s[:,1].max():.3f})")
    # tan cork
    tan=(rgb[...,0]>140)&(rgb[...,1]>90)&(rgb[...,2]<120)&(rgb[...,0].astype(int)>rgb[...,2].astype(int)+50)
    m=ok&tan&(P[...,0]>-0.15)&(P[...,0]<0.15)&(P[...,1]>0.0)&(P[...,1]<0.35)&(P[...,2]>0.9)&(P[...,2]<1.2)
    Q=P[m]; print("cork pts",len(Q), Q.mean(0).round(3) if len(Q) else None, Q.min(0).round(3) if len(Q) else None, Q.max(0).round(3) if len(Q) else None)
    # drawer panel top (white/grey at z 0.97-0.99) y position
    m=ok&(P[...,2]>0.97)&(P[...,2]<0.99)&(P[...,0]>-0.09)&(P[...,0]<0.11)&(P[...,1]>0.0)&(P[...,1]<0.3)&(rgb.min(-1)>150)
    Q=P[m]; print("panel top pts",len(Q),"y",Q[:,1].min().round(3) if len(Q) else None, Q[:,1].max().round(3) if len(Q) else None)
    # bar
    m=ok&(P[...,2]>0.935)&(P[...,2]<0.96)&(P[...,0]>-0.04)&(P[...,0]<0.05)&(P[...,1]>0.0)&(P[...,1]<0.3)&(rgb.max(-1)<120)
    Q=P[m]; print("bar pts",len(Q),"y",Q[:,1].min().round(3) if len(Q) else None,Q[:,1].max().round(3) if len(Q) else None)
EOF

# openrua op 42
python3 - <<'EOF'
import numpy as np
for cam in ["agentview","frontview"]:
    P=np.load(f"{cam}_cloud.npy"); rgb=np.load(f"{cam}_rgb.npy"); ok=np.isfinite(P).all(-1)
    m=ok&(P[...,2]>0.99)&(P[...,2]<1.06)&(P[...,1]>0.15)&(P[...,1]<0.219)&(P[...,0]>-0.09)&(P[...,0]<0.12)
    Q=P[m]; print("==",cam,"top-handle candidates",len(Q))
    for z in np.arange(0.99,1.06,0.01):
        s=Q[(Q[:,2]>=z)&(Q[:,2]<z+0.01)]
        if len(s): print(f"  z[{z:.2f}] n={len(s)} x=({s[:,0].min():.3f},{s[:,0].max():.3f}) y=({s[:,1].min():.3f},{s[:,1].max():.3f})")
    # also the hole region: x 0.075-0.113, y 0.184-0.22: what's the max z there
    m=ok&(P[...,0]>0.06)&(P[...,0]<0.115)&(P[...,1]>0.18)&(P[...,1]<0.225)&(P[...,2]>0.9)&(P[...,2]<1.0)
    Q=P[m]; print("  hole region pts",len(Q))
    for x in np.arange(0.06,0.115,0.01):
        s=Q[(Q[:,0]>=x)&(Q[:,0]<x+0.01)]
        if len(s): print(f"  x[{x:.3f}] n={len(s)} y=({s[:,1].min():.3f},{s[:,1].max():.3f}) z=({s[:,2].min():.3f},{s[:,2].max():.3f})")
EOF

# openrua op 43
cat step5_close.py | head -80

# openrua op 44
mkdir -p "$(dirname /workspace/step6_press.py)"
cat > /workspace/step6_press.py <<'OPENRUA_EOF'
"""Press down on the jammed bottle so the drawer-front corner it rests on is
pushed back (-y); the bottle should then drop onto the drawer floor."""
import numpy as np, sys, math
from rob import *
r=Robot()
print("fingers",r.fingers())
a=np.array([0,math.sin(math.radians(45)),-math.cos(math.radians(45))])
Rp=hand_R(a,(1,0,0)); qp=quat_from_R(Rp)
TCP_OFF=0.1034
def hand_for_tcp(tcp): return np.array(tcp)-TCP_OFF*a
def go(p,q,secs,tol=0.02):
    sol=r.ik_hand(p,q)
    if sol is None: print("IK FAIL",p); return None
    code,err,n_=r.move_q_conv(sol,secs,tol=tol,retries=2)
    pos,quat,_=r.fk_hand()
    print(f"go {np.round(p,3)} -> code={code} jerr={err:.4f} retries={n_} hand={pos.round(4)}")
    return pos
xp=float(sys.argv[1]) if len(sys.argv)>1 else -0.02
yp=float(sys.argv[2]) if len(sys.argv)>2 else 0.20
pos,quat,_=r.fk_hand(); print("start hand",pos.round(3))
# above the bottle
go(hand_for_tcp([xp,yp,1.07]),qp,4.0)
w0=r.wrench(); print("wrench0",w0.round(2))
for z in [1.03,1.01,1.00,0.99,0.98,0.97,0.96]:
    hp=go(hand_for_tcp([xp,yp,z]),qp,2.0,tol=0.05)
    if hp is None: break
    tcp=hp+TCP_OFF*a
    w=r.wrench(); print(f"   tcp={tcp.round(4)} (cmd z {z:.3f}) wrench={w.round(2)}")
    if tcp[2]-z > 0.012:
        print("   blocked"); break
# retreat up
hp=r.fk_hand()[0]
go(hp+[0,0,0.08],qp,3.0)
print("done; fingers",r.fingers())
OPENRUA_EOF

# openrua op 45
timeout 600 python3 step6_press.py 2>&1 | grep -v "^\[" | tail -30

# openrua op 46
python3 tools/perception/cam_snap.py robot0_eye_in_hand eih2.png 2>&1 | grep -v "^\[" ; python3 tools/perception/cam_snap.py sideview side2.png 2>&1 | grep -v "^\["

# openrua op 47
(python3 cloud.py agentview; python3 cloud.py frontview; python3 cloud.py birdview) 2>&1 | grep -v "^\[" ; python3 - <<'EOF'
import numpy as np
P=np.load("agentview_cloud.npy"); rgb=np.load("agentview_rgb.npy"); ok=np.isfinite(P).all(-1)
# region of the slot: y 0.15-0.25, z 0.90-1.05, x -0.12..0.13
m=ok&(P[...,1]>0.15)&(P[...,1]<0.25)&(P[...,2]>0.90)&(P[...,2]<1.06)&(P[...,0]>-0.12)&(P[...,0]<0.13)
Q=P[m]; C=rgb[m]
print("n",len(Q))
for x in np.arange(-0.12,0.13,0.01):
    s=(Q[:,0]>=x)&(Q[:,0]<x+0.01)
    if s.sum()<3: continue
    q=Q[s]; c=C[s].mean(0)
    # split by z bands
    line=f"x[{x:+.2f}] n={s.sum():3d} rgb={c.astype(int)} "
    for z in np.arange(0.90,1.06,0.02):
        t=(q[:,2]>=z)&(q[:,2]<z+0.02)
        if t.sum(): line+=f" z{z:.2f}:y({q[t,1].min():.3f},{q[t,1].max():.3f})"
    print(line)
EOF

# openrua op 48
python3 tools/perception/cam_snap.py agentview agent2.png 2>&1 | grep -v "^\["; python3 tools/perception/cam_snap.py frontview front2.png 2>&1 | grep -v "^\["

# openrua op 49
python3 -c "
import cv2; im=cv2.imread('agent2.png'); c=im[180:400,380:640]; cv2.imwrite('agent2_crop.png', cv2.resize(c,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('front2.png'); cv2.imwrite('front2_small.png', im)"

# openrua op 50
python3 - <<'EOF'
import numpy as np
P=np.load("agentview_cloud.npy"); rgb=np.load("agentview_rgb.npy"); ok=np.isfinite(P).all(-1)
# bowl: points above table (z>0.915) in front of the drawer panel (y<0.168), x -0.15..0.15, excluding the robot (z<0.97)
m=ok&(P[...,2]>0.915)&(P[...,2]<0.97)&(P[...,1]<0.168)&(P[...,1]>-0.2)&(P[...,0]>-0.15)&(P[...,0]<0.15)
Q=P[m]; print("n",len(Q))
if len(Q):
    print("x",Q[:,0].min().round(3),Q[:,0].max().round(3),"y",Q[:,1].min().round(3),Q[:,1].max().round(3),"z",Q[:,2].min().round(3),Q[:,2].max().round(3))
    for y in np.arange(-0.2,0.17,0.02):
        s=Q[(Q[:,1]>=y)&(Q[:,1]<y+0.02)]
        if len(s)>3: print(f" y[{y:+.2f}] n={len(s)} x=({s[:,0].min():.3f},{s[:,0].max():.3f}) z=({s[:,2].min():.3f},{s[:,2].max():.3f})")
# panel handle: y 0.12-0.17, z 0.93-0.96
m=ok&(P[...,2]>0.93)&(P[...,2]<0.965)&(P[...,1]>0.12)&(P[...,1]<0.17)&(P[...,0]>-0.1)&(P[...,0]<0.1)
Q=P[m]; print("bar pts",len(Q))
for y in np.arange(0.12,0.17,0.005):
    s=Q[(Q[:,1]>=y)&(Q[:,1]<y+0.005)]
    if len(s)>3: print(f" y[{y:.3f}] n={len(s)} x=({s[:,0].min():.3f},{s[:,0].max():.3f}) z=({s[:,2].min():.3f},{s[:,2].max():.3f})")
EOF

# openrua op 51
python3 - <<'EOF'
import numpy as np
for cam in ["agentview","frontview"]:
    P=np.load(f"{cam}_cloud.npy"); ok=np.isfinite(P).all(-1)
    m=ok&(P[...,1]>0.165)&(P[...,1]<0.178)&(P[...,2]>0.93)&(P[...,2]<0.978)
    Q=P[m]; print(cam,"panel face pts",len(Q),"x",Q[:,0].min().round(3),Q[:,0].max().round(3))
    for x in np.arange(-0.14,-0.06,0.01):
        s=Q[(Q[:,0]>=x)&(Q[:,0]<x+0.01)]
        if len(s): print(f"  x[{x:.2f}] n={len(s)} y=({s[:,1].min():.3f},{s[:,1].max():.3f}) z=({s[:,2].min():.3f},{s[:,2].max():.3f})")
    # cabinet frame near left: y 0.20-0.23, z 0.93-0.98, x<-0.09
    m=ok&(P[...,1]>0.20)&(P[...,1]<0.23)&(P[...,2]>0.93)&(P[...,2]<0.978)&(P[...,0]<-0.09)
    Q=P[m]; print("  frame-left pts",len(Q), (Q[:,0].min().round(3),Q[:,0].max().round(3)) if len(Q) else "")
EOF

# openrua op 52
mkdir -p "$(dirname /workspace/step7_sidepush.py)"
cat > /workspace/step7_sidepush.py <<'OPENRUA_EOF'
"""Close the drawer: horizontal hand approaching along +x from the left of
the cabinet, closed fingers push the panel's front face with their +y side.
Hand body stays at x < -0.14 (outside the cabinet), fingers at the panel's
left end (clear of the bowl and handle)."""
import numpy as np, sys, math
from rob import *
r=Robot()
print("fingers",r.fingers())
TCP_OFF=0.1034
a=np.array([1.0,0,0])                      # approach +x (horizontal)
Rh=hand_R(a,(0,1,0)); qh=quat_from_R(Rh)    # closing axis y
def hand_for_tcp(tcp): return np.array(tcp)-TCP_OFF*a
def go(p,q,secs,tol=0.02,retries=3):
    sol=r.ik_hand(p,q)
    if sol is None: print("IK FAIL",p); return None
    code,err,n_=r.move_q_conv(sol,secs,tol=tol,retries=retries)
    pos,quat,_=r.fk_hand()
    print(f"go {np.round(p,3)} -> code={code} jerr={err:.4f} retries={n_} hand={pos.round(4)}")
    return pos
x0=-0.072; z0=0.963; HALF=0.0143           # finger block half-thickness along closing axis
pos,quat,_=r.fk_hand(); print("start hand",pos.round(3))
# 1. up from the current pose
go(pos+[0,0,0.10],quat,3.0)
# 2. above the pre-push point, horizontal orientation
hp=go(hand_for_tcp([x0,0.10,1.12]),qh,5.0)
if hp is None: sys.exit(1)
# 3. descend
hp=go(hand_for_tcp([x0,0.10,z0]),qh,3.0)
if hp is None: sys.exit(1)
w0=r.wrench(); print("wrench0",w0.round(2))
# 4. push +y in steps; panel face should end at ~0.222 -> tcp_y ~0.208
for y in np.arange(0.12,0.2181,0.02):
    hp=go(hand_for_tcp([x0,y,z0]),qh,2.5,tol=0.02,retries=3)
    if hp is None: break
    tcp=hp+TCP_OFF*a
    w=r.wrench(); print(f"   tcp_y={tcp[1]:.4f} (cmd {y:.3f}) panel~{tcp[1]+HALF:.4f} wrench={w.round(2)}")
    if y-tcp[1] > 0.012 or abs(w[1])>15:
        print("   stalled"); break
# 5. retreat: -y then up
hp=r.fk_hand()[0]
go(hp+[0,-0.04,0],qh,2.0)
go(hp+[0,-0.04,0.12],qh,3.0)
print("done; fingers",r.fingers())
OPENRUA_EOF

# openrua op 53
timeout 900 python3 step7_sidepush.py 2>&1 | grep -v "^\[" | tail -30

# openrua op 54
(python3 cloud.py agentview; python3 cloud.py frontview) 2>&1 | grep -v "^\["; python3 tools/perception/cam_snap.py agentview agent3.png 2>&1 | grep -v "^\["; python3 - <<'EOF'
import numpy as np
for cam in ["agentview","frontview"]:
    P=np.load(f"{cam}_cloud.npy"); rgb=np.load(f"{cam}_rgb.npy"); ok=np.isfinite(P).all(-1)
    # drawer panel front face: z 0.93-0.975, x -0.09..0.11, y in 0.15-0.25 -> histogram of y
    m=ok&(P[...,2]>0.955)&(P[...,2]<0.975)&(P[...,0]>-0.09)&(P[...,0]<-0.05)&(P[...,1]>0.15)&(P[...,1]<0.26)
    Q=P[m]; print(cam,"panel-left face y:",np.percentile(Q[:,1],[5,50,95]).round(4) if len(Q) else None, len(Q))
    m=ok&(P[...,2]>0.955)&(P[...,2]<0.975)&(P[...,0]>0.06)&(P[...,0]<0.11)&(P[...,1]>0.15)&(P[...,1]<0.26)
    Q=P[m]; print(cam,"panel-right face y:",np.percentile(Q[:,1],[5,50,95]).round(4) if len(Q) else None, len(Q))
    m=ok&(P[...,2]>0.99)&(P[...,2]<1.06)&(P[...,0]>-0.09)&(P[...,0]<0.11)&(P[...,1]>0.15)&(P[...,1]<0.26)
    Q=P[m]; print(cam,"top drawer face y:",np.percentile(Q[:,1],[5,50,95]).round(4), len(Q))
    # bar
    m=ok&(P[...,2]>0.935)&(P[...,2]<0.96)&(P[...,0]>-0.03)&(P[...,0]<0.04)&(P[...,1]>0.15)&(P[...,1]<0.26)
    Q=P[m]; print(cam,"bar y:",np.percentile(Q[:,1],[5,50,95]).round(4) if len(Q) else None, len(Q))
    # any green (bottle) visible? bottle rgb: check points with g > r+15 and g > b+15 above table
    g=rgb[...,1].astype(int); rr=rgb[...,0].astype(int); b=rgb[...,2].astype(int)
    m=ok&(g>rr+10)&(g>b+10)&(P[...,2]>0.91)
    Q=P[m]; print(cam,"greenish pts",len(Q), (Q.min(0).round(3),Q.max(0).round(3)) if len(Q) else "")
EOF

# openrua op 55
python3 -c "
import cv2; im=cv2.imread('agent3.png'); c=im[180:400,380:640]; cv2.imwrite('agent3_crop.png', cv2.resize(c,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))"

# openrua op 56
python3 - <<'EOF'
import numpy as np
for cam in ["agentview","frontview"]:
    P=np.load(f"{cam}_cloud.npy"); ok=np.isfinite(P).all(-1)
    print(cam)
    for z in np.arange(0.93,1.12,0.01):
        m=ok&(P[...,2]>=z)&(P[...,2]<z+0.01)&(P[...,0]>0.06)&(P[...,0]<0.11)&(P[...,1]>0.15)&(P[...,1]<0.26)
        Q=P[m]
        if len(Q)>3: print(f"  z[{z:.2f}] n={len(Q)} y med={np.median(Q[:,1]):.4f} min={Q[:,1].min():.4f}")
EOF

# openrua op 57
cat > step7b.py <<'EOF'
import numpy as np, sys, math
from rob import *
r=Robot()
TCP_OFF=0.1034
a=np.array([1.0,0,0]); qh=quat_from_R(hand_R(a,(0,1,0)))
def hand_for_tcp(tcp): return np.array(tcp)-TCP_OFF*a
def go(p,q,secs,tol=0.02,retries=3):
    sol=r.ik_hand(p,q)
    if sol is None: print("IK FAIL",p); return None
    code,err,n_=r.move_q_conv(sol,secs,tol=tol,retries=retries)
    pos,quat,_=r.fk_hand()
    print(f"go {np.round(p,3)} -> code={code} jerr={err:.4f} retries={n_} hand={pos.round(4)}")
    return pos
x0=-0.072; z0=0.963; HALF=0.0143
go(hand_for_tcp([x0,0.17,z0]),qh,3.0)
print("wrench0",r.wrench().round(2))
for y in [0.195,0.205,0.21,0.215,0.22]:
    hp=go(hand_for_tcp([x0,y,z0]),qh,2.0)
    if hp is None: break
    tcp=hp+TCP_OFF*a; w=r.wrench()
    print(f"   tcp_y={tcp[1]:.4f} (cmd {y:.3f}) panel~{tcp[1]+HALF:.4f} wrench={w.round(2)}")
    if y-tcp[1] > 0.008 or abs(w[1])>15: print("   stalled"); break
hp=r.fk_hand()[0]
go(hp+[0,-0.04,0],qh,2.0); go(hp+[0,-0.04,0.12],qh,3.0)
print("done")
EOF
timeout 600 python3 step7b.py 2>&1 | grep -v "^\[" | tail

# openrua op 58
(for c in birdview sideview galleryview frontview agentview; do python3 tools/perception/cam_snap.py $c final_$c.png; done) 2>&1 | grep -v "^\["; python3 cloud.py birdview 2>&1 | grep -v "^\["; python3 - <<'EOF'
import numpy as np
P=np.load("birdview_cloud.npy"); rgb=np.load("birdview_rgb.npy"); ok=np.isfinite(P).all(-1)
g=rgb[...,1].astype(int); rr=rgb[...,0].astype(int); b=rgb[...,2].astype(int)
m=ok&(g>rr+10)&(g>b+10)&(P[...,2]>0.905)
Q=P[m]; print("birdview greenish pts",len(Q), (Q.min(0).round(3),Q.max(0).round(3)) if len(Q) else "")
# objects on table above 0.915 excluding cabinet (x -0.13..0.13, y>0.2) and robot (x<-0.3)
m=ok&(P[...,2]>0.915)&(P[...,2]<1.0)&~((P[...,1]>0.2)&(P[...,0]>-0.14)&(P[...,0]<0.14))&(P[...,0]>-0.3)
Q=P[m]; print("table objects pts",len(Q))
if len(Q):
    for x in np.arange(-0.3,0.4,0.05):
        s=Q[(Q[:,0]>=x)&(Q[:,0]<x+0.05)]
        if len(s)>5: print(f" x[{x:+.2f}] n={len(s)} y=({s[:,1].min():.3f},{s[:,1].max():.3f}) z=({s[:,2].min():.3f},{s[:,2].max():.3f})")
EOF

# openrua op 59
mkdir -p "$(dirname /claude-config/projects/-workspace/memory/panda-sim-machine-quirks.md)"
cat > /claude-config/projects/-workspace/memory/panda-sim-machine-quirks.md <<'OPENRUA_EOF'
---
name: panda-sim-machine-quirks
description: Hard-won facts about the /workspace Panda simulator (FK/IK frames, controller lag, gripper geometry, scene layout) not stated in machine.yaml or docs
metadata:
  type: project
---

Facts learned on 2026-09-12/13 while doing "bottle into bottom drawer" on the /workspace Panda sim:

- `/compute_fk` returns WORLD coordinates (not panda_link0) despite manifest saying planning_frame panda_link0; `/compute_ik` with empty frame_id also takes world poses. Set `ik_link_name="panda_hand"` — default tip panda_link8 is yawed 45° from the hand.
- FollowJointTrajectory often returns error_code -5 with large joint error (controller lag); resending the same goal until joint error < 0.02 rad converges (see /workspace/rob.py `move_q_conv`). Never trust a single goal result.
- Hand frame: z = approach, y = finger-closing axis, x = thin axis. TCP = hand + 0.1034·z. Closed finger block ≈ ±0.014 m along closing axis, ±0.008 m along x; hand body ≈ ±0.10 along closing axis, ±0.03 along x, palm ≈ 0.066 from hand origin.
- Scene: table z 0.905; cabinet (3 drawers) front face y≈0.222, x −0.125..0.125; bottom drawer floor z 0.924, panel top 0.982, handle bar protrudes 3.3 cm at z 0.94–0.955. Drawer closed stop puts its face at y≈0.217 (5 mm behind frame — that is fully closed). Bottom drawer needed ~17 N to pull open and didn't move — push it closed from the side instead: horizontal hand approaching along +x with the hand body outside the cabinet (x < −0.14) worked cleanly.
- Paused-clock sim: world advances only during commands; verify by re-reading cameras/point clouds (cloud.py) after each move.

**Why:** these cost many failed attempts (wrong-orientation grasp, drawer push stalls) to discover.
**How to apply:** reuse /workspace/rob.py and cloud.py; plan hand-body clearance against cabinet front/handles before every push.
OPENRUA_EOF

# openrua op 60
cd /claude-config/projects/-workspace/memory && (test -f MEMORY.md && cat MEMORY.md || echo "(no index)")

# openrua op 61
mkdir -p "$(dirname /claude-config/projects/-workspace/memory/MEMORY.md)"
cat > /claude-config/projects/-workspace/memory/MEMORY.md <<'OPENRUA_EOF'
- [Panda sim machine quirks](panda-sim-machine-quirks.md) — FK/IK are world-frame, ik_link_name=panda_hand, resend trajectories until converged, finger/hand geometry, cabinet/drawer layout
OPENRUA_EOF
