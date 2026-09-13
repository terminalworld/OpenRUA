#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat tools/README.md && echo ---- && cat tools/action/fjt_send.py && echo ---- && cat tools/action/ik_move.py && echo ---- && cat tools/action/gripper_cmd.py && echo ---- && cat tools/perception/cam_snap.py && echo ---- && cat tools/perception/px2world.py

# openrua op 2
timeout 30 ros2 topic list; echo ----; timeout 20 ros2 action list; echo ----; timeout 20 ros2 service list | head -60; echo ----; timeout 20 ros2 node list

# openrua op 3
for c in agentview birdview frontview sideview galleryview paperview robot0_eye_in_hand robot0_robotview; do timeout 60 python3 tools/perception/cam_snap.py $c & done; wait; ls *.png; timeout 20 ros2 topic echo /joint_states --once

# openrua op 4
timeout 20 ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id|x:|y:|z:|w:" | head -80; echo ----; timeout 15 ros2 topic echo /tf --once 2>&1 | grep -E "frame_id" | head

# openrua op 5
cat > /workspace/state.py <<'EOF'
#!/usr/bin/env python3
"""Print world->panda_link0, hand FK (via /compute_fk), finger gap."""
import rclpy, yaml, sys
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionFK
from tf2_msgs.msg import TFMessage
import numpy as np

def main():
    rclpy.init(); node = rclpy.create_node("state_probe")
    js = {}
    node.create_subscription(JointState, "/joint_states", lambda m: js.setdefault("m", m), 1)
    tfs = {}
    def tfcb(m):
        for t in m.transforms:
            tfs[t.child_frame_id] = t
    node.create_subscription(TFMessage, "/tf", tfcb, 10)
    for _ in range(40):
        rclpy.spin_once(node, timeout_sec=0.2)
        if "m" in js and "panda_link0" in tfs: break
    m = js["m"]
    d = dict(zip(m.name, m.position))
    print("joints:", {k: round(v,4) for k,v in d.items()})
    if "panda_link0" in tfs:
        t = tfs["panda_link0"].transform
        print("world->panda_link0: t=(%.4f %.4f %.4f) q=(%.4f %.4f %.4f %.4f)" % (t.translation.x,t.translation.y,t.translation.z,t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w))
    cli = node.create_client(GetPositionFK, "/compute_fk")
    cli.wait_for_service(timeout_sec=10)
    req = GetPositionFK.Request()
    req.header.frame_id = ""
    req.fk_link_names = ["panda_hand"]
    arm = [f"panda_joint{i}" for i in range(1,8)]
    req.robot_state.joint_state.name = arm
    req.robot_state.joint_state.position = [d[a] for a in arm]
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=30)
    r = fut.result()
    if r is None: print("FK no answer"); return
    p = r.pose_stamped[0].pose
    print("hand (base frame): p=(%.4f %.4f %.4f) q=(%.4f %.4f %.4f %.4f) frame=%s err=%d" % (p.position.x,p.position.y,p.position.z,p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w, r.pose_stamped[0].header.frame_id, r.error_code.val))
    rclpy.shutdown()
main()
EOF
timeout 60 python3 /workspace/state.py

# openrua op 6
python3 - <<'EOF'
import numpy as np
def T(a,d,al,th):
    ca,sa,ct,st=np.cos(al),np.sin(al),np.cos(th),np.sin(th)
    return np.array([[ct,-st,0,a],[st*ca,ct*ca,-sa,-d*sa],[st*sa,ct*sa,ca,d*ca],[0,0,0,1]])
q=[0.0,-0.161,0.0,-2.4446,0.0,2.2268,0.7854]
a=[0,0,0,0.0825,-0.0825,0,0.088]; d=[0.333,0,0.316,0,0.384,0,0]; al=[0,-np.pi/2,np.pi/2,np.pi/2,-np.pi/2,np.pi/2,np.pi/2]
M=np.eye(4)
for i in range(7): M=M@T(a[i],d[i],al[i],q[i])
M=M@T(0,0.107,0,0)  # flange link8
# hand = link8 rotated -45deg about z
M=M@T(0,0,0,-np.pi/4)
print("hand in base:", M[:3,3].round(4))
print("hand z axis in base:", M[:3,2].round(4))
EOF

# openrua op 7
cat > /workspace/cloud.py <<'EOF'
#!/usr/bin/env python3
"""Grab depth+info+TF for a camera, save world-frame point cloud as npy (H,W,3)."""
import sys, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage
from cv_bridge import CvBridge

def grab(node, topic, typ):
    got={}
    sub=node.create_subscription(typ, topic, lambda m: got.setdefault("m",m), 1)
    for _ in range(100):
        rclpy.spin_once(node, timeout_sec=0.2)
        if "m" in got: break
    node.destroy_subscription(sub)
    return got["m"]

def quat_R(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

def main():
    cam=sys.argv[1]
    rclpy.init(); node=rclpy.create_node("cloud")
    tfs={}
    def cb(m):
        for t in m.transforms: tfs[t.child_frame_id]=t
    node.create_subscription(TFMessage,"/tf",cb,10)
    depth=grab(node,f"/{cam}/depth/image_raw",Image)
    info=grab(node,f"/{cam}/color/camera_info",CameraInfo)
    frame=f"{cam}_optical_frame"
    for _ in range(50):
        if frame in tfs: break
        rclpy.spin_once(node,timeout_sec=0.2)
    t=tfs[frame].transform
    R=quat_R(t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w)
    p0=np.array([t.translation.x,t.translation.y,t.translation.z])
    D=CvBridge().imgmsg_to_cv2(depth,"passthrough").astype(np.float64)
    H,W=D.shape
    fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
    u,v=np.meshgrid(np.arange(W),np.arange(H))
    X=(u-cx)*D/fx; Y=(v-cy)*D/fy
    P=np.stack([X,Y,D],-1)@R.T+p0
    np.save(f"{cam}_cloud.npy",P)
    print(cam, "cam pos", p0.round(3), "depth range", np.nanmin(D), np.nanmax(D), "shape", D.shape)
    rclpy.shutdown()
main()
EOF
for c in birdview agentview sideview frontview; do timeout 90 python3 /workspace/cloud.py $c & done; wait

# openrua op 8
python3 - <<'EOF'
import numpy as np
P=np.load("birdview_cloud.npy")
Z=P[...,2]
# table height: mode of z in the central region
tbl=np.median(Z[200:450,180:460]); print("table z ~", round(tbl,4))
# sample pixels of interest
for name,(u,v) in {"pot1":(262,238),"pot2":(390,278),"knob":(330,305),"stove":(330,350),"table":(220,400)}.items():
    print(name,(u,v),P[v,u].round(4))
# objects above table by >1.5cm in the table region
mask=(Z>tbl+0.015)&(Z<tbl+0.4)
mask[:190,:]=False  # exclude robot
import scipy.ndimage as ndi
lab,n=ndi.label(mask)
for i in range(1,n+1):
    sel=lab==i
    if sel.sum()<30: continue
    pts=P[sel]
    vs,us=np.nonzero(sel)
    print(f"blob{i}: n={sel.sum()} px_center=({us.mean():.0f},{vs.mean():.0f}) x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] ztop={pts[:,2].max():.3f}")
EOF

# openrua op 9
python3 - <<'EOF'
import numpy as np
for cam in ["agentview","sideview","frontview"]:
    P=np.load(f"{cam}_cloud.npy")
    for name,(x0,y0) in {"pot1":(-0.204,-0.19),"pot2":(-0.075,0.235)}.items():
        sel=(np.abs(P[...,0]-x0)<0.09)&(np.abs(P[...,1]-y0)<0.09)&(P[...,2]>0.905)&(P[...,2]<1.1)
        pts=P[sel]
        print(cam,name,"n=",len(pts))
        for zlo in np.arange(0.90,1.06,0.01):
            s=pts[(pts[:,2]>=zlo)&(pts[:,2]<zlo+0.01)]
            if len(s)<3: continue
            print(f"  z={zlo:.2f} n={len(s):3d} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}] cx={s[:,0].mean():.3f} cy={s[:,1].mean():.3f}")
EOF

# openrua op 10
python3 - <<'EOF'
import numpy as np
for cam in ["agentview","frontview","birdview"]:
    P=np.load(f"{cam}_cloud.npy")
    # pot1 handle: y < -0.238, x near -0.199
    sel=(P[...,1]<-0.236)&(P[...,1]>-0.30)&(np.abs(P[...,0]+0.199)<0.05)&(P[...,2]>0.92)&(P[...,2]<1.06)
    pts=P[sel]; print(cam,"pot1 handle pts",len(pts))
    for zlo in np.arange(0.94,1.05,0.01):
        s=pts[(pts[:,2]>=zlo)&(pts[:,2]<zlo+0.01)]
        if len(s)<2: continue
        print(f"  z={zlo:.3f} n={len(s):3d} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]")
    # by y bins: z range
    print("  by y:")
    for ylo in np.arange(-0.29,-0.235,0.005):
        s=pts[(pts[:,1]>=ylo)&(pts[:,1]<ylo+0.005)]
        if len(s)<2: continue
        print(f"  y={ylo:.3f} n={len(s):3d} z[{s[:,2].min():.3f},{s[:,2].max():.3f}] x[{s[:,0].min():.3f},{s[:,0].max():.3f}]")
EOF

# openrua op 11
python3 - <<'EOF'
import numpy as np
P=np.load("agentview_cloud.npy")
sel=(P[...,1]<-0.236)&(P[...,1]>-0.30)&(np.abs(P[...,0]+0.199)<0.05)&(P[...,2]>0.92)&(P[...,2]<1.06)
pts=P[sel]
# 2D occupancy in y-z with 4mm bins
ys=np.arange(-0.285,-0.235,0.004); zs=np.arange(0.955,1.045,0.004)
H,_,_=np.histogram2d(pts[:,1],pts[:,2],bins=[ys,zs])
print("rows: z from top (1.045) down; cols: y from -0.285 to -0.235")
for j in range(len(zs)-2,-1,-1):
    print(f"z={zs[j]:.3f} "+"".join("#" if H[i,j]>0 else "." for i in range(len(ys)-1)))
EOF

# openrua op 12
timeout 60 python3 - <<'EOF'
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
rclpy.init(); n=rclpy.create_node("rd")
qos=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
got=[]
n.create_subscription(String,"/robot_description",got.append,qos)
for _ in range(50):
    rclpy.spin_once(n,timeout_sec=0.2)
    if got: break
open("robot.urdf","w").write(got[0].data); print(len(got[0].data))
rclpy.shutdown()
EOF
grep -n -A12 'link name="panda_hand"' robot.urdf | head -60; grep -n -B2 -A14 'link name="panda_leftfinger"' robot.urdf | head -60; grep -n "mesh filename" robot.urdf | head -20

# openrua op 13
ls /opt/ros/jazzy/share/moveit_resources_panda_description/meshes/collision/ && python3 - <<'EOF'
import numpy as np, struct
def stl_bounds(path):
    data=open(path,'rb').read()
    if data[:5]==b'solid' and b'facet' in data[:300]:
        import re
        v=np.array([list(map(float,m)) for m in re.findall(rb'vertex\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)',data)])
    else:
        n=struct.unpack('<I',data[80:84])[0]
        arr=np.frombuffer(data[84:84+n*50],dtype=np.dtype([('n','<3f4'),('v','<9f4'),('a','<u2')]))
        v=arr['v'].reshape(-1,3)
    return v.min(0),v.max(0)
for m in ["hand","finger"]:
    lo,hi=stl_bounds(f"/opt/ros/jazzy/share/moveit_resources_panda_description/meshes/collision/{m}.stl")
    print(m,"min",lo.round(4),"max",hi.round(4))
EOF
grep -o '<joint name="panda_finger_joint1"[^>]*>.*\?</joint>' robot.urdf | head -c 800; echo; grep -o 'panda_hand_joint.\{0,400\}' robot.urdf | head -c 600

# openrua op 14
find / -iname "hand*.stl" -o -iname "finger*.stl" 2>/dev/null | head; find / -ipath "*panda*" -name "*.stl" 2>/dev/null | head; find / -iname "*.xml" -path "*panda*" 2>/dev/null | head

# openrua op 15
python3 - <<'EOF'
import numpy as np
for cam in ["agentview","sideview","frontview"]:
    P=np.load(f"{cam}_cloud.npy")
    sel=(np.abs(P[...,0]+0.203)<0.12)&(np.abs(P[...,1])<0.15)&(P[...,2]>1.15)&(P[...,2]<1.30)
    pts=P[sel]; print(cam,len(pts))
    for zlo in np.arange(1.15,1.30,0.01):
        s=pts[(pts[:,2]>=zlo)&(pts[:,2]<zlo+0.01)]
        if len(s)<3: continue
        print(f"  z={zlo:.2f} n={len(s):3d} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]")
EOF

# openrua op 16
cat > /workspace/ikprobe.py <<'EOF'
#!/usr/bin/env python3
"""IK feasibility probe: python3 ikprobe.py x y z qx qy qz qw [x y z qx qy qz qw ...]  (world coords, hand frame). No motion."""
import sys, rclpy, numpy as np
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK
ARM=[f"panda_joint{i}" for i in range(1,8)]
def main():
    vals=list(map(float,sys.argv[1:])); poses=[vals[i:i+7] for i in range(0,len(vals),7)]
    rclpy.init(); node=rclpy.create_node("ikprobe")
    js={}
    node.create_subscription(JointState,"/joint_states",lambda m: js.setdefault("m",m),1)
    while "m" not in js: rclpy.spin_once(node,timeout_sec=0.2)
    d=dict(zip(js["m"].name,js["m"].position))
    seed=[d[a] for a in ARM]
    cli=node.create_client(GetPositionIK,"/compute_ik"); cli.wait_for_service(timeout_sec=10)
    for p in poses:
        req=GetPositionIK.Request(); r=req.ik_request
        r.group_name="panda_arm"; r.pose_stamped.header.frame_id=""
        r.pose_stamped.pose.position.x,r.pose_stamped.pose.position.y,r.pose_stamped.pose.position.z=p[:3]
        r.pose_stamped.pose.orientation.x,r.pose_stamped.pose.orientation.y,r.pose_stamped.pose.orientation.z,r.pose_stamped.pose.orientation.w=p[3:]
        r.robot_state.joint_state.name=ARM; r.robot_state.joint_state.position=seed
        r.avoid_collisions=False
        r.timeout.sec=2
        fut=cli.call_async(req); rclpy.spin_until_future_complete(node,fut,timeout_sec=60)
        res=fut.result()
        if res is None: print(p[:3],"NO ANSWER"); continue
        if res.error_code.val!=1: print(np.round(p[:3],3),"FAIL",res.error_code.val); continue
        sol=dict(zip(res.solution.joint_state.name,res.solution.joint_state.position))
        q=[sol[a] for a in ARM]
        print(np.round(p[:3],3),"OK",",".join(f"{v:.4f}" for v in q))
    rclpy.shutdown()
main()
EOF
# sanity: current pose should solve to ~current joints; then vertical-hand poses (q=(0.7071,0.7071,0,0)) at increasing x
timeout 120 python3 /workspace/ikprobe.py  -0.203 0 1.2696 0.9996 0 -0.0284 0   \
  -0.199 -0.27 1.10 0.7071 0.7071 0 0   -0.07 0.17 1.10 0.7071 0.7071 0 0 \
  0.075 -0.01 1.13 0.7071 0.7071 0 0   0.15 -0.085 1.13 0.7071 0.7071 0 0   0.15 0.155 1.13 0.7071 0.7071 0 0  0.19 0.0 1.13 0.7071 0.7071 0 0  0.23 0.0 1.13 0.7071 0.7071 0 0

# openrua op 17
cat > /workspace/geom.py <<'EOF'
import numpy as np
def R_to_q(R):
    """rotation matrix -> (x,y,z,w)"""
    m=R; t=np.trace(m)
    if t>0:
        s=np.sqrt(t+1)*2; w=0.25*s; x=(m[2,1]-m[1,2])/s; y=(m[0,2]-m[2,0])/s; z=(m[1,0]-m[0,1])/s
    elif m[0,0]>m[1,1] and m[0,0]>m[2,2]:
        s=np.sqrt(1+m[0,0]-m[1,1]-m[2,2])*2; w=(m[2,1]-m[1,2])/s; x=0.25*s; y=(m[0,1]+m[1,0])/s; z=(m[0,2]+m[2,0])/s
    elif m[1,1]>m[2,2]:
        s=np.sqrt(1+m[1,1]-m[0,0]-m[2,2])*2; w=(m[0,2]-m[2,0])/s; x=(m[0,1]+m[1,0])/s; y=0.25*s; z=(m[1,2]+m[2,1])/s
    else:
        s=np.sqrt(1+m[2,2]-m[0,0]-m[1,1])*2; w=(m[1,0]-m[0,1])/s; x=(m[0,2]+m[2,0])/s; y=(m[1,2]+m[2,1])/s; z=0.25*s
    return np.array([x,y,z,w])
def Ry(t): c,s=np.cos(t),np.sin(t); return np.array([[c,0,s],[0,1,0],[-s,0,c]])
def Rx(t): c,s=np.cos(t),np.sin(t); return np.array([[1,0,0],[0,c,-s],[0,s,c]])
def Rz(t): c,s=np.cos(t),np.sin(t); return np.array([[c,-s,0],[s,c,0],[0,0,1]])
R_PICK=np.array([[0,1,0],[1,0,0],[0,0,-1]],float)      # fingers close along world x, z down
R_PLACE=np.array([[1,0,0],[0,-1,0],[0,0,-1]],float)    # fingers close along world y, z down
def lean(R,theta):
    """tilt hand z from -z toward +x by theta (about world y), keeping closing axis y horizontal"""
    return Ry(-theta)@R
def hand_from_tips(tips,R,off=0.1034):
    """hand frame origin given desired fingertip point and orientation"""
    return np.array(tips)-off*R[:,2]
EOF
python3 - <<'EOF'
import numpy as np, subprocess
from geom import *
poses=[]
for tips,R in [((0.075,-0.015,1.045),R_PLACE),((0.075,0.085,1.045),R_PLACE),
               ((0.075,-0.015,1.045),lean(R_PLACE,np.radians(20))),((0.075,0.085,1.045),lean(R_PLACE,np.radians(20))),
               ((0.10,-0.015,1.045),lean(R_PLACE,np.radians(25))),((0.10,0.085,1.045),lean(R_PLACE,np.radians(25))),
               ((0.075,-0.015,1.20),R_PLACE),((0.075,0.085,1.20),R_PLACE),
               ((-0.199,-0.269,1.197),R_PICK),((-0.069,0.170,1.197),R_PICK),
               ((-0.199,-0.269,0.999),R_PICK),((-0.069,0.170,0.999),R_PICK)]:
    h=hand_from_tips(tips,R); q=R_to_q(R); poses+=list(h)+list(q)
    print("tips",tips,"-> hand",h.round(4),"q",q.round(4))
out=subprocess.run(["python3","ikprobe.py"]+[f"{v:.5f}" for v in poses],capture_output=True,text=True,timeout=300)
print(out.stdout,out.stderr[-500:])
EOF

# openrua op 18
mkdir -p "$(dirname /workspace/rob.py)"
cat > /workspace/rob.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Reusable robot client: joint state, IK, trajectory, gripper. Builds clients once."""
import time
import numpy as np
import rclpy
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from geom import R_to_q

ARM = [f"panda_joint{i}" for i in range(1, 8)]
FJT = "/panda_arm_controller/follow_joint_trajectory"
GRIP = "/franka_gripper/gripper_action"


def log(*a):
    print(time.strftime("%H:%M:%S"), *a, flush=True)


class Robot:
    def __init__(self, name="rob"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT)
        self.grip = ActionClient(self.node, GripperCommand, GRIP)
        self.ik_cli = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        assert self.fjt.wait_for_server(timeout_sec=20), "no FJT server"
        assert self.grip.wait_for_server(timeout_sec=20), "no gripper server"
        assert self.ik_cli.wait_for_service(timeout_sec=20), "no IK"
        assert self.fk_cli.wait_for_service(timeout_sec=20), "no FK"
        while self._js is None:
            rclpy.spin_once(self.node, timeout_sec=0.2)

    def _on_js(self, m):
        self._js = m

    def spin(self, n=3):
        for _ in range(n):
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def joints(self):
        self.spin(5)
        return dict(zip(self._js.name, self._js.position))

    def arm_q(self):
        d = self.joints()
        return np.array([d[a] for a in ARM])

    def fingers(self):
        d = self.joints()
        return d["panda_finger_joint1"], d["panda_finger_joint2"]

    def fk(self, q=None):
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = ARM
        req.robot_state.joint_state.position = list(q if q is not None else self.arm_q())
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        r = fut.result()
        p = r.pose_stamped[0].pose
        return (np.array([p.position.x, p.position.y, p.position.z]),
                np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]))

    def ik(self, pos, R, seed=None):
        q = R_to_q(R)
        req = GetPositionIK.Request()
        r = req.ik_request
        r.group_name = "panda_arm"
        r.pose_stamped.header.frame_id = ""
        r.pose_stamped.pose.position.x, r.pose_stamped.pose.position.y, r.pose_stamped.pose.position.z = map(float, pos)
        r.pose_stamped.pose.orientation.x, r.pose_stamped.pose.orientation.y, r.pose_stamped.pose.orientation.z, r.pose_stamped.pose.orientation.w = map(float, q)
        r.robot_state.joint_state.name = ARM
        r.robot_state.joint_state.position = list(map(float, seed if seed is not None else self.arm_q()))
        r.avoid_collisions = False
        r.timeout.sec = 2
        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            log("IK FAIL", None if res is None else res.error_code.val, "at", np.round(pos, 4))
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return np.array([sol[a] for a in ARM])

    def move_joints(self, q, secs, via=None):
        """One trajectory; optional list of intermediate joint vectors `via` spread evenly."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM
        pts = list(via or []) + [q]
        for i, p in enumerate(pts):
            t = secs * (i + 1) / len(pts)
            pt = JointTrajectoryPoint(positions=list(map(float, p)))
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=120)
        gh = send.result()
        if gh is None or not gh.accepted:
            log("FJT goal rejected")
            return False
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=1800)
        if res.result() is None:
            log("FJT no result (timeout)")
        else:
            code = res.result().result.error_code
            if code != 0:
                log("FJT error_code", code)
        err = np.abs(self.arm_q() - np.array(q)).max()
        log(f"move done; max joint err {err:.4f} rad")
        return err < 0.02

    def move_pose(self, pos, R, secs, seed=None, via_poses=None):
        """IK for pos/R (world, hand frame), execute. via_poses: list of (pos,R) intermediate."""
        seed = self.arm_q() if seed is None else seed
        via = []
        for vp, vR in (via_poses or []):
            qv = self.ik(vp, vR, seed)
            if qv is None:
                return False
            via.append(qv)
            seed = qv
        q = self.ik(pos, R, seed)
        if q is None:
            return False
        ok = self.move_joints(q, secs, via=via)
        p, _ = self.fk()
        log(f"hand now at {np.round(p, 4)} (target {np.round(pos, 4)}) d={np.linalg.norm(p - pos):.4f}")
        return ok

    def gripper(self, width, timeout=300):
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = 30.0
        fut = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        gh = fut.result()
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=timeout)
        r = rf.result()
        f = self.fingers()
        log(f"gripper -> {width}: reached={None if r is None else r.result.reached_goal} "
            f"stalled={None if r is None else r.result.stalled} fingers={f[0]:.4f},{f[1]:.4f}")
        return f
OPENRUA_EOF

# openrua op 19
python3 - <<'EOF'
import numpy as np
P=np.load("agentview_cloud.npy")
sel=(P[...,1]<0.207)&(P[...,1]>0.14)&(np.abs(P[...,0]+0.069)<0.05)&(P[...,2]>0.92)&(P[...,2]<1.06)
pts=P[sel]; print("pot2 handle pts",len(pts),"x range",pts[:,0].min().round(3),pts[:,0].max().round(3), "x median",np.median(pts[:,0]).round(4))
ys=np.arange(0.155,0.209,0.004); zs=np.arange(0.955,1.045,0.004)
H,_,_=np.histogram2d(pts[:,1],pts[:,2],bins=[ys,zs])
print("cols y from 0.155 to 0.205 step 4mm")
for j in range(len(zs)-2,-1,-1):
    print(f"z={zs[j]:.3f} "+"".join("#" if H[i,j]>0 else "." for i in range(len(ys)-1)))
EOF

# openrua op 20
mkdir -p "$(dirname /workspace/pick.py)"
cat > /workspace/pick.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Pick a moka pot by its handle plate (handle pointing -y).
Usage: python3 -u pick.py <hx> <hy>   (world x,y of the handle outer-arm grasp point)
Fingers close along world x; hand leaned 25 deg about x so palm sits outboard of the pot.
"""
import sys
import numpy as np
from geom import R_PICK, Rx
from rob import Robot, log

hx, hy = float(sys.argv[1]), float(sys.argv[2])
LEAN = np.radians(25)
R = Rx(LEAN) @ R_PICK                      # hand z -> (0, sin, -cos): down and toward +y (pot body)
Z_TIPS_GRASP = 0.999
Z_TIPS_HIGH = 1.20


def hand(tips):
    return np.array(tips) - 0.1034 * R[:, 2]


r = Robot("pick")
log("start joints", np.round(r.arm_q(), 3), "fingers", r.fingers())
r.gripper(0.04)

# 1. pre-grasp high above the handle
assert r.move_pose(hand((hx, hy, Z_TIPS_HIGH)), R, 4.0), "pre-grasp move failed"
# 2. descend in two legs
assert r.move_pose(hand((hx, hy, 1.06)), R, 2.5), "descend-1 failed"
assert r.move_pose(hand((hx, hy, Z_TIPS_GRASP)), R, 2.0), "descend-2 failed"
# 3. close
f = r.gripper(0.0)
gap = f[0] - f[1]
log(f"finger gap after close = {gap:.4f} m")
if gap < 0.003:
    log("GRASP FAILED: fingers closed on air")
    sys.exit(2)
# 4. lift straight up
assert r.move_pose(hand((hx, hy, Z_TIPS_HIGH)), R, 3.0), "lift failed"
f = r.fingers()
log(f"after lift finger gap = {f[0]-f[1]:.4f}")
log("PICK DONE")
OPENRUA_EOF

# openrua op 21
nohup python3 -u pick.py -0.1995 -0.267 > pick1.log 2>&1 &
sleep 45; cat /workspace/pick1.log

# openrua op 22
timeout 120 python3 - <<'EOF'
import numpy as np
from geom import R_PICK, Rx
from rob import Robot, log
R=Rx(np.radians(25))@R_PICK
r=Robot("probe")
q=r.arm_q(); print("now",np.round(q,3))
p,_=r.fk(); print("hand",np.round(p,4))
tgt=np.array((-0.1995,-0.267,1.20))-0.1034*R[:,2]
sol=r.ik(tgt,R); print("sol",np.round(sol,3)); print("diff",np.round(sol-q,3))
EOF

# openrua op 23
python3 - <<'EOF'
import re
s=open("rob.py").read()
s=s.replace('''    def move_pose(self, pos, R, secs, seed=None, via_poses=None):
        """IK for pos/R (world, hand frame), execute. via_poses: list of (pos,R) intermediate."""
        seed = self.arm_q() if seed is None else seed
        via = []
        for vp, vR in (via_poses or []):
            qv = self.ik(vp, vR, seed)
            if qv is None:
                return False
            via.append(qv)
            seed = qv
        q = self.ik(pos, R, seed)
        if q is None:
            return False
        ok = self.move_joints(q, secs, via=via)
''','''    def ik_best(self, pos, R, seed, flip_ok=True):
        """IK; with flip_ok also try R rotated 180deg about hand z (symmetric gripper) and
        return the solution closest to seed in joint space."""
        cands = [R]
        if flip_ok:
            cands.append(R @ np.diag([-1.0, -1.0, 1.0]))
        best = None
        for Rc in cands:
            q = self.ik(pos, Rc, seed)
            if q is None:
                continue
            d = np.abs(q - seed).max()
            if best is None or d < best[1]:
                best = (q, d)
        return None if best is None else best[0]

    def move_pose(self, pos, R, secs, seed=None, via_poses=None, flip_ok=True, retries=1):
        """IK for pos/R (world, hand frame), execute. via_poses: list of (pos,R) intermediate."""
        seed = self.arm_q() if seed is None else seed
        via = []
        for vp, vR in (via_poses or []):
            qv = self.ik_best(vp, vR, seed, flip_ok)
            if qv is None:
                return False
            via.append(qv)
            seed = qv
        q = self.ik_best(pos, R, seed, flip_ok)
        if q is None:
            return False
        log(f"joint travel {np.round(q - self.arm_q(), 2)}")
        ok = self.move_joints(q, secs, via=via)
        for _ in range(retries):
            if ok:
                break
            log("retrying same goal (slower)")
            ok = self.move_joints(q, secs * 1.5)
''')
open("rob.py","w").write(s)
EOF
nohup python3 -u pick.py -0.1995 -0.267 > pick1.log 2>&1 &
sleep 60; cat /workspace/pick1.log

# openrua op 24
sleep 20; cat /workspace/pick1.log | tail -4; timeout 60 python3 tools/perception/cam_snap.py agentview agent_after_pick1.png; timeout 60 python3 tools/perception/cam_snap.py sideview side_after_pick1.png; timeout 20 ros2 topic echo /joint_states --once | grep -A9 position

# openrua op 25
timeout 90 python3 cloud.py sideview & timeout 90 python3 cloud.py agentview & wait
python3 - <<'EOF'
import numpy as np
# pot2 fine profile (2mm bins) from sideview (sees +y face, x-silhouette) and agentview
for cam in ["sideview","agentview"]:
    P=np.load(f"{cam}_cloud.npy")
    sel=(np.abs(P[...,0]+0.069)<0.06)&(np.abs(P[...,1]-0.243)<0.045)&(P[...,2]>0.93)&(P[...,2]<1.07)
    pts=P[sel]
    print(cam,"pot2 body profile (2mm bins)")
    for zlo in np.arange(0.935,1.065,0.002):
        s=pts[(pts[:,2]>=zlo)&(pts[:,2]<zlo+0.002)]
        if len(s)<2: continue
        print(f"  z={zlo:.3f} n={len(s):3d} xw={s[:,0].max()-s[:,0].min():.3f} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] yw={s[:,1].max()-s[:,1].min():.3f} cx={s[:,0].mean():.4f} cy={s[:,1].mean():.4f}")
EOF

# openrua op 26
timeout 90 python3 cloud.py frontview & timeout 90 python3 cloud.py sideview & wait
python3 - <<'EOF'
import numpy as np
for cam in ["frontview","sideview","agentview"]:
    P=np.load(f"{cam}_cloud.npy")
    sel=(P[...,0]>-0.36)&(P[...,0]<-0.04)&(P[...,1]>-0.5)&(P[...,1]<-0.05)&(P[...,2]>0.95)&(P[...,2]<1.19)
    pts=P[sel]
    # exclude hand/fingers: fingers are near x=-0.1995±0.05, y<-0.25, z>1.17 ... just print z-binned extents
    print(cam,len(pts))
    for zlo in np.arange(0.95,1.19,0.01):
        s=pts[(pts[:,2]>=zlo)&(pts[:,2]<zlo+0.01)]
        if len(s)<3: continue
        print(f"  z={zlo:.2f} n={len(s):3d} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}] cx={s[:,0].mean():.3f} cy={s[:,1].mean():.3f}")
EOF

# openrua op 27
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih1.png; timeout 60 python3 tools/perception/cam_snap.py frontview front1.png; python3 -c "
import cv2; im=cv2.imread('front1.png'); cv2.imwrite('front1_crop.png', cv2.resize(im[60:300,120:400],None,fx=2,fy=2,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('side_after_pick1.png'); cv2.imwrite('side1_crop.png', cv2.resize(im[60:260,260:460],None,fx=2.5,fy=2.5,interpolation=cv2.INTER_CUBIC))"

# openrua op 28
cat > /workspace/setdown.py <<'EOF'
import numpy as np, sys
from geom import R_PICK, Rx
from rob import Robot, log
R = Rx(np.radians(25)) @ R_PICK
def hand(t): return np.array(t) - 0.1034 * R[:, 2]
hx, hy = -0.1995, -0.267
r = Robot("setdown")
assert r.move_pose(hand((hx, hy, 1.06)), R, 3.0)
log("fingers", r.fingers())
assert r.move_pose(hand((hx, hy, 1.015)), R, 2.0)
log("fingers", r.fingers())
assert r.move_pose(hand((hx, hy, 1.000)), R, 1.5)
log("fingers", r.fingers())
r.gripper(0.04)
assert r.move_pose(hand((hx, hy, 1.20)), R, 3.0)
log("SETDOWN DONE")
EOF
nohup python3 -u setdown.py > setdown.log 2>&1 &
sleep 75; cat setdown.log

# openrua op 29
timeout 60 python3 tools/perception/cam_snap.py sideview side2.png; timeout 60 python3 tools/perception/cam_snap.py agentview agent2.png; python3 -c "
import cv2; im=cv2.imread('side2.png'); cv2.imwrite('side2_crop.png', cv2.resize(im[100:330,240:460],None,fx=2.5,fy=2.5,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('agent2.png'); cv2.imwrite('agent2_crop.png', cv2.resize(im[80:300,80:320],None,fx=2.5,fy=2.5,interpolation=cv2.INTER_CUBIC))"

# openrua op 30
timeout 90 python3 cloud.py frontview & timeout 90 python3 cloud.py sideview & timeout 90 python3 cloud.py agentview & wait
python3 - <<'EOF'
import numpy as np
allp=[]
for cam in ["frontview","sideview","agentview"]:
    P=np.load(f"{cam}_cloud.npy")
    sel=(P[...,0]>-0.30)&(P[...,0]<-0.02)&(P[...,1]>-0.40)&(P[...,1]<-0.10)&(P[...,2]>0.902)&(P[...,2]<1.02)
    allp.append(P[sel])
pts=np.concatenate(allp); print("pts",len(pts))
for zlo in np.arange(0.90,1.02,0.01):
    s=pts[(pts[:,2]>=zlo)&(pts[:,2]<zlo+0.01)]
    if len(s)<3: continue
    print(f"  z={zlo:.2f} n={len(s):3d} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}] cx={s[:,0].mean():.3f} cy={s[:,1].mean():.3f}")
low=pts[pts[:,2]<0.915]; print("lowest pts centroid (E):",low.mean(0).round(4), "n",len(low))
EOF

# openrua op 31
timeout 90 python3 cloud.py birdview; python3 - <<'EOF'
import numpy as np
P=np.load("birdview_cloud.npy")
sel=(P[...,0]>-0.32)&(P[...,0]<-0.05)&(P[...,1]>-0.40)&(P[...,1]<-0.12)&(P[...,2]>0.905)&(P[...,2]<1.30)
pts=P[sel]
for zlo in np.arange(0.90,1.30,0.01):
    s=pts[(pts[:,2]>=zlo)&(pts[:,2]<zlo+0.01)]
    if len(s)<3: continue
    print(f"  z={zlo:.2f} n={len(s):3d} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}] cx={s[:,0].mean():.3f} cy={s[:,1].mean():.3f}")
EOF

# openrua op 32
python3 tools/perception/cam_snap.py sideview side3.png && python3 tools/perception/cam_snap.py frontview front3.png && python3 tools/perception/cam_snap.py eye_in_hand eih3.png 2>&1 | tail -2; ros2 topic list | grep image_raw

# openrua op 33
cat /tmp/claude-1000/-workspace/0bb68966-1811-4752-ba66-0c1acd96a050/tasks/b2km3xnlw.output; ls -la *.png | tail -5; ros2 topic list | grep image_raw

# openrua op 34
mkdir -p "$(dirname /workspace/nudge.py)"
cat > /workspace/nudge.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Move the hand by a world-frame delta keeping its current orientation (from FK).
Usage: python3 -u nudge.py dx dy dz [secs]
"""
import sys
import numpy as np
from rob import Robot, log

d = np.array([float(a) for a in sys.argv[1:4]])
secs = float(sys.argv[4]) if len(sys.argv) > 4 else 3.0
r = Robot("nudge")
p, q = r.fk()
x, y, z, w = q
R = np.array([
    [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
    [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
    [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
])
log("hand", np.round(p, 4), "fingers", r.fingers())
ok = r.move_pose(p + d, R, secs, flip_ok=False, retries=0)
log("ok" if ok else "STALLED", "fingers", r.fingers())
OPENRUA_EOF

# openrua op 35
timeout 900 python3 -u nudge.py 0.03 0.012 -0.005 3 2>&1 | tail -8 && python3 tools/perception/cam_snap.py frontview front4.png && python3 tools/perception/cam_snap.py sideview side4.png

# openrua op 36
timeout 900 python3 -u nudge.py 0.02 0.008 -0.004 3 2>&1 | tail -4 && python3 tools/perception/cam_snap.py frontview front5.png && python3 tools/perception/cam_snap.py sideview side5.png

# openrua op 37
timeout 300 python3 cloud.py sideview >/dev/null 2>&1; timeout 300 python3 cloud.py frontview >/dev/null 2>&1; python3 - <<'EOF'
import numpy as np
for cam in ["sideview","frontview"]:
    P=np.load(f"{cam}_cloud.npy").reshape(-1,3); P=P[np.isfinite(P).all(1)]
    m=(P[:,0]>-0.32)&(P[:,0]<-0.02)&(P[:,1]>-0.42)&(P[:,1]<-0.15)&(P[:,2]>0.895)&(P[:,2]<1.10)
    Q=P[m]; print(cam,len(Q))
    for z0 in np.arange(0.90,1.10,0.01):
        s=Q[(Q[:,2]>=z0)&(Q[:,2]<z0+0.01)]
        if len(s)>3: print(f" z{z0:.2f} n{len(s):4d} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}] xm{s[:,0].mean():.3f} ym{s[:,1].mean():.3f}")
EOF

# openrua op 38
timeout 900 python3 - <<'EOF' 2>&1 | tail -5
from rob import Robot, log
import numpy as np
r=Robot("rel")
r.gripper(0.04)
EOF
python3 tools/perception/cam_snap.py sideview side6.png && python3 tools/perception/cam_snap.py frontview front6.png

# openrua op 39
timeout 900 python3 -u nudge.py 0 -0.02 0.12 3 2>&1 | tail -3; timeout 300 python3 cloud.py birdview >/dev/null 2>&1; timeout 300 python3 cloud.py sideview >/dev/null 2>&1; python3 - <<'EOF'
import numpy as np
for cam in ["birdview","sideview"]:
    P=np.load(f"{cam}_cloud.npy").reshape(-1,3); P=P[np.isfinite(P).all(1)]
    m=(P[:,0]>-0.32)&(P[:,0]<-0.02)&(P[:,1]>-0.42)&(P[:,1]<-0.12)&(P[:,2]>0.905)&(P[:,2]<1.10)
    Q=P[m]; print(cam,len(Q))
    for z0 in np.arange(0.90,1.08,0.01):
        s=Q[(Q[:,2]>=z0)&(Q[:,2]<z0+0.01)]
        if len(s)>3: print(f" z{z0:.2f} n{len(s):4d} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}] xm{s[:,0].mean():.3f} ym{s[:,1].mean():.3f}")
EOF

# openrua op 40
python3 - <<'EOF'
import numpy as np
P=np.concatenate([np.load(f"{c}_cloud.npy").reshape(-1,3) for c in ["birdview","sideview"]]); P=P[np.isfinite(P).all(1)]
m=(P[:,0]>-0.25)&(P[:,0]<0.02)&(P[:,1]>-0.42)&(P[:,1]<-0.12)&(P[:,2]>0.906)&(P[:,2]<1.0)
Q=P[m]
for x0 in np.arange(-0.20,-0.01,0.01):
    s=Q[(Q[:,0]>=x0)&(Q[:,0]<x0+0.01)]
    if len(s)>3: print(f" x{x0:.2f} n{len(s):4d} y[{s[:,1].min():.3f},{s[:,1].max():.3f}] zmax{s[:,2].max():.3f}  y@ztop{s[np.argmax(s[:,2]),1]:.3f}")
EOF
python3 tools/perception/cam_snap.py birdview bird7.png >/dev/null; python3 tools/perception/cam_snap.py agentview agent7.png

# openrua op 41
grep -n -A12 '<link name="panda_hand"' robot.urdf | head -40; grep -n -B2 -A10 'panda_leftfinger' robot.urdf | grep -n -E 'box|cylinder|mesh|origin' | head

# openrua op 42
cat geom.py; cat ikprobe.py | head -30

# openrua op 43
mkdir -p "$(dirname /workspace/reach.py)"
cat > /workspace/reach.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Reach survey (no motion): for several hand orientations, find IK feasibility of
fingertip positions along x at given y,z. Uses a few seeds."""
import numpy as np
from geom import R_PICK, R_PLACE, Ry, Rz, hand_from_tips
from rob import Robot, log

r = Robot("reach")
seeds = [r.arm_q(),
         np.array([0, -0.785, 0, -2.356, 0, 1.571, 0.785]),
         np.array([0, 0.3, 0, -1.6, 0, 1.9, 0.785]),
         np.array([0, 0.6, 0, -1.2, 0, 1.8, 0.785])]


def feasible(pos, R):
    for s in seeds:
        for Rc in (R, R @ np.diag([-1.0, -1.0, 1.0])):
            q = r.ik(pos, Rc, s)
            if q is not None:
                return q
    return None


R_HORIZ = np.array([[0, 0, 1], [0, -1, 0], [1, 0, 0]], float)  # hand z -> +x, fingers close along y
cases = {
    "vertical R_PLACE tips z1.044": (R_PLACE, 1.044),
    "vertical R_PICK tips z1.044": (R_PICK, 1.044),
    "lean25 tips z1.044": (Ry(np.radians(-25)) @ R_PLACE, 1.044),
    "lean45 tips z1.044": (Ry(np.radians(-45)) @ R_PLACE, 1.044),
    "vertical R_PLACE tips z0.95": (R_PLACE, 0.95),
    "horiz +x, hand z0.963": (R_HORIZ, None),
    "horiz +x, hand z1.00": (R_HORIZ, None),
}
import sys
sys.stdout = open("/dev/stdout", "w")
for name, (R, ztip) in cases.items():
    ok = []
    for x in np.arange(0.0, 0.26, 0.02):
        for y in (0.0, 0.08, -0.02):
            if ztip is None:
                pos = np.array([x, y, 0.963 if "0.963" in name else 1.00])
            else:
                pos = hand_from_tips((x, y, ztip), R)
            q = feasible(pos, R)
            ok.append((round(x, 2), y, q is not None))
    good = [(x, y) for x, y, f in ok if f]
    print(name, "feasible x per y:", {y: [x for x, yy, f in ok if f and yy == y] for y in (0.0, 0.08, -0.02)}, flush=True)
OPENRUA_EOF

# openrua op 44
timeout 1500 python3 -u reach.py 2>&1 | grep -v "IK FAIL"

# openrua op 45
timeout 600 python3 - <<'EOF' 2>&1 | grep -v "^\[" 
import numpy as np
from rob import Robot
from geom import R_PICK, Rx
r=Robot("meas")
p,q=r.fk()
x,y,z,w=q
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
print("hand",p, "fingers", r.fingers())
P=np.concatenate([np.load(f"{c}_cloud.npy").reshape(-1,3) for c in ["sideview","frontview","birdview"]]); P=P[np.isfinite(P).all(1)]
L=(P-p)@R   # points in hand frame
m=(np.abs(L[:,0])<0.15)&(np.abs(L[:,1])<0.15)&(L[:,2]>-0.25)&(L[:,2]<0.13)
L=L[m]
for z0 in np.arange(-0.25,0.13,0.01):
    s=L[(L[:,2]>=z0)&(L[:,2]<z0+0.01)]
    if len(s)>5: print(f"hz{z0:+.2f} n{len(s):4d} hx[{s[:,0].min():+.3f},{s[:,0].max():+.3f}] hy[{s[:,1].min():+.3f},{s[:,1].max():+.3f}]")
EOF

# openrua op 46
python3 - <<'EOF'
import numpy as np
P=np.concatenate([np.load(f"{c}_cloud.npy").reshape(-1,3) for c in ["sideview","frontview","birdview"]]); P=P[np.isfinite(P).all(1)]
cx,cy=-0.069,0.243
m=(np.abs(P[:,0]-cx)<0.08)&(np.abs(P[:,1]-cy)<0.12)&(P[:,2]>0.9)&(P[:,2]<1.07)
Q=P[m]
print("pot2 slices (x-width uses all points incl handle at -y; handle excluded for y via y>cy-0.05)")
for z0 in np.arange(0.90,1.07,0.005):
    s=Q[(Q[:,2]>=z0)&(Q[:,2]<z0+0.005)]
    if len(s)>3:
        sb=s[s[:,1]>cy-0.045]
        print(f" z{z0:.3f} n{len(s):4d} x[{s[:,0].min()-cx:+.3f},{s[:,0].max()-cx:+.3f}] y[{sb[:,1].min()-cy:+.3f},{sb[:,1].max()-cy:+.3f}] handle_ymin{s[:,1].min()-cy:+.3f}")
EOF

# openrua op 47
python3 - <<'EOF'
import numpy as np
P=np.concatenate([np.load(f"{c}_cloud.npy").reshape(-1,3) for c in ["sideview","frontview","birdview"]]); P=P[np.isfinite(P).all(1)]
m=(P[:,0]>-0.05)&(P[:,0]<0.30)&(P[:,1]>-0.15)&(P[:,1]<0.2)&(P[:,2]>0.903)&(P[:,2]<1.0)
Q=P[m]
print("knob obstacle + stove region, by x slice")
for x0 in np.arange(-0.05,0.30,0.01):
    s=Q[(Q[:,0]>=x0)&(Q[:,0]<x0+0.01)]
    if len(s)>3: print(f" x{x0:+.2f} n{len(s):4d} y[{s[:,1].min():+.3f},{s[:,1].max():+.3f}] z[{s[:,2].min():.3f},{s[:,2].max():.3f}]")
print("knob by z")
K=Q[(Q[:,0]<0.095)]
for z0 in np.arange(0.90,0.97,0.01):
    s=K[(K[:,2]>=z0)&(K[:,2]<z0+0.01)]
    if len(s)>3: print(f" z{z0:.2f} n{len(s):4d} x[{s[:,0].min():+.3f},{s[:,0].max():+.3f}] y[{s[:,1].min():+.3f},{s[:,1].max():+.3f}]")
EOF

# openrua op 48
timeout 600 python3 - <<'EOF' 2>&1 | grep -v "^\[" 
import numpy as np
from rob import Robot
r=Robot("meas")
p,q=r.fk()
x,y,z,w=q
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
print("hand",p,"R z-axis",R[:,2])
P=np.concatenate([np.load(f"{c}_cloud.npy").reshape(-1,3) for c in ["sideview","frontview","birdview"]]); P=P[np.isfinite(P).all(1)]
P=P[P[:,2]>1.0]
L=(P-p)@R
m=(np.abs(L[:,0])<0.12)&(np.abs(L[:,1])<0.13)&(L[:,2]>-0.16)&(L[:,2]<0.12)
L=L[m]
for z0 in np.arange(-0.16,0.12,0.01):
    s=L[(L[:,2]>=z0)&(L[:,2]<z0+0.01)]
    if len(s)>5: print(f"hz{z0:+.2f} n{len(s):4d} hx[{s[:,0].min():+.3f},{s[:,0].max():+.3f}] hy[{s[:,1].min():+.3f},{s[:,1].max():+.3f}]")
EOF

# openrua op 49
timeout 600 python3 - <<'EOF' 2>&1 | grep -v "^\[" 
import numpy as np
from rob import Robot
r=Robot("meas")
p,q=r.fk()
x,y,z,w=q
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
for cam in ["sideview","frontview","birdview","agentview"]:
    try: P=np.load(f"{cam}_cloud.npy").reshape(-1,3)
    except Exception as e: print(cam,e); continue
    P=P[np.isfinite(P).all(1)]; P=P[P[:,2]>1.0]
    L=(P-p)@R
    m=(np.abs(L[:,0])<0.12)&(np.abs(L[:,1])<0.13)&(L[:,2]>0.0)&(L[:,2]<0.05)
    h,_=np.histogram(L[m,0],bins=np.arange(-0.12,0.121,0.01))
    print(cam, "hx hist (1cm bins from -0.12):", h)
    m2=(np.abs(L[:,0])<0.12)&(np.abs(L[:,1])<0.13)&(L[:,2]>0.06)&(L[:,2]<0.10)
    h2,_=np.histogram(L[m2,0],bins=np.arange(-0.12,0.121,0.01))
    print(cam, "fingers hz.06-.10 hx hist:", h2)
    h3,_=np.histogram(L[m2,1],bins=np.arange(-0.13,0.131,0.01))
    print(cam, "fingers hy hist:", h3)
EOF

# openrua op 50
python3 -c "
import cv2; im=cv2.imread('bird7.png'); print(im.shape)
cv2.imwrite('bird7_crop.png', cv2.resize(im[0:300, 300:640], None, fx=2, fy=2))
"

# openrua op 51
mkdir -p "$(dirname /workspace/standup.py)"
cat > /workspace/standup.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Stand up a moka pot lying along +x (base at -x end, lid at +x end).
Usage: python3 -u standup.py <gx> <gy> <gz_axis>
  gx,gy: world x,y of the grasp point on the lying pot's axis (lower chamber)
  gz_axis: world z of the pot axis
Top-down grasp with fingers closing along y, lift, rotate -90deg about world y so the
lid points up, lower until the base is ~3 mm above the table, release, retreat.
"""
import sys
import numpy as np
from geom import R_PLACE, Ry
from rob import Robot, log

gx, gy, gz = map(float, sys.argv[1:4])
BASE_X = float(sys.argv[4]) if len(sys.argv) > 4 else -0.19   # x of the base face
TABLE_Z = 0.90
TIP = 0.1034
R_G = R_PLACE                        # hand z down, fingers close along y
R_F = Ry(np.radians(-90)) @ R_PLACE  # hand z -> +x, fingers still close along y
h_above_base = gx - BASE_X           # grasp height above base once upright
log(f"grasp {h_above_base*100:.1f} cm above base; R_F z-axis {R_F[:,2]} x-axis {R_F[:,0]}")

r = Robot("standup")
q0 = r.arm_q()

# poses (hand frame): fingertips at the axis point for the grasp
tips_hover = np.array([gx, gy, gz + 0.16])
tips_grasp = np.array([gx, gy, gz])
p_hover = tips_hover - TIP * R_G[:, 2]
p_grasp = tips_grasp - TIP * R_G[:, 2]
G_high = np.array([gx, gy, gz + 0.18])          # axis point carried high
p_high_v = G_high - TIP * R_G[:, 2]              # still vertical
p_high_h = G_high - TIP * R_F[:, 2]              # rotated: hand behind (-x) the pot
G_down = np.array([gx, gy, TABLE_Z + h_above_base + 0.003])
p_down = G_down - TIP * R_F[:, 2]
p_retreat = p_down + np.array([-0.06, 0, 0.12])

# feasibility check before moving anything
seed = q0
plan = [("hover", p_hover, R_G), ("grasp", p_grasp, R_G), ("high_v", p_high_v, R_G),
        ("high_h", p_high_h, R_F), ("down", p_down, R_F), ("retreat", p_retreat, R_F)]
for name, p, R in plan:
    q = r.ik_best(p, R, seed, flip_ok=True)
    if q is None:
        log(f"IK infeasible for {name} at {np.round(p,3)}; abort")
        sys.exit(1)
    log(f"{name}: hand {np.round(p,3)} ok, joint travel {np.round(np.abs(q-seed).max(),2)}")
    seed = q

r.gripper(0.04)
assert r.move_pose(p_hover, R_G, 4.0), "hover failed"
assert r.move_pose(p_grasp, R_G, 3.0), "descend failed"
f = r.gripper(0.0)
gap = f[0] - f[1]
log(f"gap after close {gap:.4f}")
if gap < 0.04:
    log("GRASP FAILED (gap too small for a pot body)")
    r.gripper(0.04)
    r.move_pose(p_hover, R_G, 3.0)
    sys.exit(2)
assert r.move_pose(p_high_v, R_G, 3.0), "lift failed"
log("rotating pot upright")
ok = r.move_pose(p_high_h, R_F, 6.0, flip_ok=False)
log(f"rotate ok={ok} fingers {r.fingers()}")
assert r.move_pose(p_down, R_F, 4.0, flip_ok=False, retries=0) or True
log(f"fingers before release {r.fingers()}")
r.gripper(0.04)
r.move_pose(p_retreat, R_F, 3.0, flip_ok=False, retries=0)
log("STANDUP DONE")
OPENRUA_EOF

# openrua op 52
python3 - <<'EOF'
import re
s=open("standup.py").read()
old=s[s.index("# feasibility check before moving anything"):s.index("r.gripper(0.04)\nassert")]
new='''# feasibility check before moving anything; the pot rotation must be exactly Ry(-90) applied
# to the ACTUAL grasp orientation, so pick one flip (about hand z) and keep it for every pose.
FLIP = np.diag([-1.0, -1.0, 1.0])
chain = None
for fl in (np.eye(3), FLIP):
    RG, RF = R_G @ fl, R_F @ fl
    seed, qs, ok = q0, [], True
    plan = [("hover", p_hover, RG), ("grasp", p_grasp, RG), ("high_v", p_high_v, RG),
            ("high_h", p_high_h, RF), ("down", p_down, RF), ("retreat", p_retreat, RF)]
    for name, p, R in plan:
        q = r.ik(p, R, seed)
        if q is None:
            log(f"flip={fl[0,0]}: IK infeasible for {name} at {np.round(p,3)}")
            ok = False
            break
        log(f"flip={fl[0,0]} {name}: hand {np.round(p,3)} ok, joint travel {np.round(np.abs(q-seed).max(),2)}")
        seed = q
    if ok:
        chain = (RG, RF)
        break
if chain is None:
    log("no feasible chain; abort")
    sys.exit(1)
R_G, R_F = chain

'''
s=s.replace(old,new)
open("standup.py","w").write(s)
EOF
grep -n "flip_ok" standup.py

# openrua op 53
sed -i 's/assert r.move_pose(p_hover, R_G, 4.0), "hover failed"/assert r.move_pose(p_hover, R_G, 4.0, flip_ok=False), "hover failed"/; s/assert r.move_pose(p_grasp, R_G, 3.0), "descend failed"/assert r.move_pose(p_grasp, R_G, 3.0, flip_ok=False), "descend failed"/; s/assert r.move_pose(p_high_v, R_G, 3.0), "lift failed"/assert r.move_pose(p_high_v, R_G, 3.0, flip_ok=False), "lift failed"/; s/    r.move_pose(p_hover, R_G, 3.0)$/    r.move_pose(p_hover, R_G, 3.0, flip_ok=False)/' standup.py && grep -n "move_pose" standup.py

# openrua op 54
nohup python3 -u standup.py -0.14 -0.222 0.937 -0.19 > standup.log 2>&1 &
sleep 240; cat standup.log | grep -v "^\["

# openrua op 55
timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih8.png; timeout 120 python3 tools/perception/cam_snap.py sideview side8.png; timeout 120 python3 tools/perception/cam_snap.py agentview agent8.png

# openrua op 56
timeout 300 python3 cloud.py birdview >/dev/null 2>&1; timeout 300 python3 cloud.py sideview >/dev/null 2>&1; timeout 300 python3 cloud.py agentview >/dev/null 2>&1; python3 - <<'EOF'
import numpy as np
P=np.concatenate([np.load(f"{c}_cloud.npy").reshape(-1,3) for c in ["birdview","sideview","agentview"]]); P=P[np.isfinite(P).all(1)]
m=(P[:,0]>-0.25)&(P[:,0]<0.02)&(P[:,1]>-0.42)&(P[:,1]<-0.12)&(P[:,2]>0.906)&(P[:,2]<1.0)
Q=P[m]
print("pot region by x (z<1.0)")
for x0 in np.arange(-0.22,0.0,0.01):
    s=Q[(Q[:,0]>=x0)&(Q[:,0]<x0+0.01)]
    if len(s)>3: print(f" x{x0:+.2f} n{len(s):4d} y[{s[:,1].min():+.3f},{s[:,1].max():+.3f}] zmax{s[:,2].max():.3f} y@ztop{s[np.argmax(s[:,2]),1]:+.3f}")
m=(P[:,0]>-0.25)&(P[:,0]<0.02)&(P[:,1]>-0.42)&(P[:,1]<-0.12)&(P[:,2]>0.906)&(P[:,2]<1.1)
Q=P[m]
print("by z")
for z0 in np.arange(0.90,1.10,0.01):
    s=Q[(Q[:,2]>=z0)&(Q[:,2]<z0+0.01)]
    if len(s)>3: print(f" z{z0:.2f} n{len(s):4d} x[{s[:,0].min():+.3f},{s[:,0].max():+.3f}] y[{s[:,1].min():+.3f},{s[:,1].max():+.3f}]")
EOF

# openrua op 57
python3 - <<'EOF'
import numpy as np
P=np.concatenate([np.load(f"{c}_cloud.npy").reshape(-1,3) for c in ["birdview","sideview","agentview"]]); P=P[np.isfinite(P).all(1)]
m=(P[:,0]>-0.25)&(P[:,0]<0.02)&(P[:,1]>-0.42)&(P[:,1]<-0.12)&(P[:,2]>0.906)&(P[:,2]<0.975)
Q=P[m]
for x0 in np.arange(-0.22,0.0,0.01):
    s=Q[(Q[:,0]>=x0)&(Q[:,0]<x0+0.01)]
    if len(s)>3:
        sb=s[s[:,1]>-0.29]
        print(f" x{x0:+.2f} n{len(s):4d} y[{s[:,1].min():+.3f},{s[:,1].max():+.3f}] body_y[{sb[:,1].min():+.3f},{sb[:,1].max():+.3f}] zmax{s[:,2].max():.3f} ymid_top{np.mean(sb[sb[:,2]>sb[:,2].max()-0.01,1]):+.3f}")
EOF

# openrua op 58
python3 - <<'EOF'
s=open("standup.py").read()
old_head=s[:s.index("r = Robot(\"standup\")")]
new_head='''#!/usr/bin/env python3
"""Stand up a moka pot lying on the table.
Usage: python3 -u standup.py <bx> <by> <kx> <ky> <z_axis> [d_grasp=0.05]
  (bx,by): world xy of the base face centre; (kx,ky): xy of the lid knob (defines the axis
  direction); z_axis: height of the pot axis; d_grasp: grasp distance from the base.
Top-down grasp with fingers closing perpendicular to the axis, lift, rotate -90deg about the
horizontal axis perpendicular to the pot so the lid points up, lower until the base is ~3 mm
above the table, release, retreat.
"""
import sys
import numpy as np
from geom import R_PLACE, Ry, Rz
from rob import Robot, log

bx, by, kx, ky, gz = map(float, sys.argv[1:6])
d_grasp = float(sys.argv[6]) if len(sys.argv) > 6 else 0.05
u = np.array([kx - bx, ky - by]); u /= np.linalg.norm(u)
phi = np.arctan2(u[1], u[0])
gx, gy = bx + d_grasp * u[0], by + d_grasp * u[1]
TABLE_Z = 0.90
TIP = 0.1034
R_G = Rz(phi) @ R_PLACE                        # hand z down, fingers close perpendicular to axis
R_F = Rz(phi) @ Ry(np.radians(-90)) @ R_PLACE  # hand z -> along axis dir (lid side), pot upright
h_above_base = d_grasp
log(f"axis dir {np.round(u,3)} phi {np.degrees(phi):.1f} deg; grasp at {np.round((gx,gy,gz),3)}")
log(f"R_F z-axis {np.round(R_F[:,2],3)} x-axis {np.round(R_F[:,0],3)}")

'''
s=new_head+s[s.index("r = Robot(\"standup\")"):]
s=s.replace("p_retreat = p_down + np.array([-0.06, 0, 0.12])","p_retreat = p_down + np.array([-0.06*u[0], -0.06*u[1], 0.12])")
open("standup.py","w").write(s)
EOF
sed -n 28,45p standup.py

# openrua op 59
nohup python3 -u standup.py -0.188 -0.222 -0.04 -0.272 0.938 0.05 > standup2.log 2>&1 &
sleep 200; grep -v "^\[" standup2.log

# openrua op 60
timeout 60 ros2 run tf2_ros tf2_echo world robot0_eye_in_hand_optical_frame 2>&1 | head -8; timeout 20 ros2 topic echo /robot0_eye_in_hand/color/camera_info --once 2>&1 | head -20

# openrua op 61
timeout 600 python3 -u nudge.py 0 0 0.13 3 2>&1 | grep -v "^\[" | tail -2; timeout 300 python3 cloud.py robot0_eye_in_hand 2>&1 | tail -1; python3 tools/perception/cam_snap.py robot0_eye_in_hand eih9.png

# openrua op 62
python3 - <<'EOF'
import numpy as np
P=np.load("robot0_eye_in_hand_cloud.npy").reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,2]>0.906)&(P[:,2]<0.99)&(P[:,0]>-0.3)&(P[:,0]<0.05)&(P[:,1]>-0.4)&(P[:,1]<-0.1)
Q=P[m]; print(len(Q), "pts; z range", Q[:,2].min(), Q[:,2].max())
# ridge: for x-slices, take highest points
b=np.array([-0.188,-0.222]); u=np.array([0.947,-0.32]); v=np.array([0.32,0.947])
s=(Q[:,:2]-b)@u; t=(Q[:,:2]-b)@v
for s0 in np.arange(-0.03,0.19,0.01):
    k=(s>=s0)&(s<s0+0.01)
    if k.sum()<5: continue
    q=Q[k]; tt=t[k]
    top=q[:,2]>q[:,2].max()-0.004
    print(f" s{s0:+.2f} n{k.sum():4d} zmax{q[:,2].max():.3f} t_ridge{tt[top].mean():+.4f} t[{tt.min():+.3f},{tt.max():+.3f}] width{tt.max()-tt.min():.3f}")
EOF

# openrua op 63
python3 - <<'EOF'
import numpy as np
P=np.load("robot0_eye_in_hand_cloud.npy").reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,2]>0.906)&(P[:,2]<0.99)&(P[:,0]>-0.3)&(P[:,0]<0.05)&(P[:,1]>-0.4)&(P[:,1]<-0.1)
Q=P[m]
for zt in (0.965,0.970,0.973):
    R=Q[Q[:,2]>zt][:,:2]
    c=R.mean(0); w,v=np.linalg.eigh(np.cov((R-c).T)); u=v[:,1]
    if u[0]<0: u=-u
    proj=(R-c)@u; perp=(R-c)@np.array([-u[1],u[0]])
    print(f"z>{zt}: n{len(R)} centre {np.round(c,4)} dir {np.round(u,3)} angle {np.degrees(np.arctan2(u[1],u[0])):.1f} deg; along [{proj.min():+.3f},{proj.max():+.3f}] perp std {perp.std():.4f} perp[{perp.min():+.3f},{perp.max():+.3f}]")
# full extent along that axis using all points
R=Q[Q[:,2]>0.970][:,:2]; c=R.mean(0); w,v=np.linalg.eigh(np.cov((R-c).T)); u=v[:,1]; u=u if u[0]>0 else -u
vv=np.array([-u[1],u[0]])
s=(Q[:,:2]-c)@u; t=(Q[:,:2]-c)@vv
print("axis c",np.round(c,4),"u",np.round(u,3))
for s0 in np.arange(-0.10,0.10,0.01):
    k=(s>=s0)&(s<s0+0.01)
    if k.sum()<5: continue
    q=Q[k]; tt=t[k]; top=q[:,2]>q[:,2].max()-0.003
    body=tt[np.abs(tt)<0.045]
    print(f" s{s0:+.2f} n{k.sum():4d} zmax{q[:,2].max():.3f} t_ridge{tt[top].mean():+.4f} t[{tt.min():+.3f},{tt.max():+.3f}] body_t[{body.min():+.3f},{body.max():+.3f}]")
EOF

# openrua op 64
nohup python3 -u standup.py -0.155 -0.203 -0.0725 -0.340 0.939 0.068 > standup3.log 2>&1 &
sleep 150; grep -v "^\[" standup3.log

# openrua op 65
timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih10.png; timeout 120 python3 tools/perception/cam_snap.py agentview agent10.png; python3 -c "
import cv2; im=cv2.imread('agent10.png'); cv2.imwrite('agent10_crop.png', cv2.resize(im[150:400, 50:350], None, fx=2, fy=2))"

# openrua op 66
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rob.py", "old_string": "    def gripper(self, width, timeout=300):", "new_string": "    def move_line(self, pos, R, secs, step=0.015, retries=1):\\n        \\"\\"\\"Straight-line Cartesian move (fixed orientation R) from the current hand pose:\\n        waypoints every `step` m, each IK-solved with the previous as seed, sent as one\\n        trajectory so the hand does not swing sideways like a joint-space interpolation.\\"\\"\\"\\n        p0, _ = self.fk()\\n        pos = np.asarray(pos, float)\\n        n = max(1, int(np.ceil(np.linalg.norm(pos - p0) / step)))\\n        seed = self.arm_q()\\n        via = []\\n        for i in range(1, n + 1):\\n            p = p0 + (pos - p0) * i / n\\n            q = self.ik(p, R, seed)\\n            if q is None:\\n                log(f\\"move_line: IK failed at waypoint {i}/{n} {np.round(p,4)}\\")\\n                return False\\n            if np.abs(q - seed).max() > 0.5:\\n                log(f\\"move_line: joint jump {np.abs(q - seed).max():.2f} at waypoint {i}; abort\\")\\n                return False\\n            via.append(q)\\n            seed = q\\n        q = via.pop()\\n        ok = self.move_joints(q, secs, via=via)\\n        for _ in range(retries):\\n            if ok:\\n                break\\n            log(\\"move_line: retrying remaining path (slower)\\")\\n            ok = self.move_line(pos, R, secs, step, retries=0)\\n        p, _ = self.fk()\\n        log(f\\"hand now at {np.round(p, 4)} (target {np.round(pos, 4)}) d={np.linalg.norm(p - pos):.4f}\\")\\n        return ok\\n\\n    def gripper(self, width, timeout=300):", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 67
sed -i 's/ok = r.move_pose(p + d, R, secs, flip_ok=False, retries=0)/ok = r.move_line(p + d, R, secs)/' nudge.py && python3 - <<'EOF'
s=open("standup.py").read()
s=s.replace('assert r.move_pose(p_grasp, R_G, 3.0, flip_ok=False), "descend failed"',
 'assert r.move_line(p_grasp, R_G, 5.0), "descend failed"')
s=s.replace('assert r.move_pose(p_high_v, R_G, 3.0, flip_ok=False), "lift failed"',
 'assert r.move_line(p_high_v, R_G, 4.0), "lift failed"')
s=s.replace('assert r.move_pose(p_down, R_F, 4.0, flip_ok=False, retries=0) or True',
 'r.move_line(p_down, R_F, 5.0, retries=0)')
s=s.replace('r.move_pose(p_retreat, R_F, 3.0, flip_ok=False, retries=0)',
 'r.move_line(p_retreat, R_F, 3.0, retries=0)')
open("standup.py","w").write(s)
EOF
grep -n "move_" standup.py; timeout 600 python3 -u nudge.py 0 0 0.12 4 2>&1 | grep -v "^\[" | tail -2

# openrua op 68
timeout 600 python3 - <<'EOF' 2>&1 | grep -v "^\["
import numpy as np
from rob import Robot, log
r=Robot("dbg")
p,q=r.fk(); x,y,z,w=q
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
q0=r.arm_q(); print("q0",np.round(q0,3))
for i in range(5):
    s=r.ik(p+[0,0,0.015],R,q0)
    print("ik",np.round(s,3),"diff",np.round(s-q0,2))
s=r.ik(p,R,q0); print("ik same pose",np.round(s-q0,2))
EOF

# openrua op 69
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rob.py", "old_string": "    def move_joints(self, q, secs, via=None):", "new_string": "    def ik_local(self, pos, R, seed, iters=12, tol=5e-4):\\n        \\"\\"\\"Damped-least-squares IK from `seed` using a numerical Jacobian (FK service), so the\\n        result stays on the seed\'s solution branch (the IK service sometimes jumps branches).\\"\\"\\"\\n        from geom import R_to_q  # noqa\\n        q = np.array(seed, float)\\n        pos = np.asarray(pos, float)\\n\\n        def pose_err(qq):\\n            p, quat = self.fk(qq)\\n            x, y, z, w = quat\\n            Rc = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],\\n                           [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],\\n                           [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])\\n            Re = R @ Rc.T                     # rotation taking current to target\\n            ang = np.arccos(np.clip((np.trace(Re) - 1) / 2, -1, 1))\\n            axis = np.array([Re[2, 1] - Re[1, 2], Re[0, 2] - Re[2, 0], Re[1, 0] - Re[0, 1]])\\n            rot = np.zeros(3) if ang < 1e-9 else axis / np.linalg.norm(axis) * ang\\n            return np.concatenate([pos - p, rot])\\n\\n        for _ in range(iters):\\n            e = pose_err(q)\\n            if np.linalg.norm(e[:3]) < tol and np.linalg.norm(e[3:]) < 4 * tol:\\n                return q\\n            J = np.zeros((6, 7))\\n            h = 1e-4\\n            for j in range(7):\\n                dq = np.zeros(7); dq[j] = h\\n                J[:, j] = (pose_err(q) - pose_err(q + dq)) / h   # d(target - fk)/dq = -d fk/dq\\n            J = -J\\n            lam = 1e-3\\n            q = q + J.T @ np.linalg.solve(J @ J.T + lam * np.eye(6), e)\\n        e = pose_err(q)\\n        log(f\\"ik_local: not converged, pos err {np.linalg.norm(e[:3]):.4f} rot err {np.linalg.norm(e[3:]):.4f}\\")\\n        return None\\n\\n    def move_joints(self, q, secs, via=None):", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 70
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rob.py", "old_string": "            for j in range(7):\\n                dq = np.zeros(7); dq[j] = h\\n                J[:, j] = (pose_err(q) - pose_err(q + dq)) / h   # d(target - fk)/dq = -d fk/dq\\n            J = -J\\n            lam = 1e-3", "new_string": "            for j in range(7):\\n                dq = np.zeros(7); dq[j] = h\\n                J[:, j] = (e - pose_err(q + dq)) / h   # = d fk/dq (e = target - fk)\\n            lam = 1e-3", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 71
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rob.py", "old_string": "            q = self.ik(p, R, seed)\\n            if q is None:\\n                log(f\\"move_line: IK failed at waypoint {i}/{n} {np.round(p,4)}\\")\\n                return False\\n            if np.abs(q - seed).max() > 0.5:\\n                log(f\\"move_line: joint jump {np.abs(q - seed).max():.2f} at waypoint {i}; abort\\")\\n                return False", "new_string": "            q = self.ik(p, R, seed)\\n            if q is None or np.abs(q - seed).max() > 0.3:\\n                q = self.ik_local(p, R, seed)      # stay on the current branch\\n            if q is None:\\n                log(f\\"move_line: IK failed at waypoint {i}/{n} {np.round(p,4)}\\")\\n                return False\\n            if np.abs(q - seed).max() > 0.5:\\n                log(f\\"move_line: joint jump {np.abs(q - seed).max():.2f} at waypoint {i}; abort\\")\\n                return False", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 72
timeout 900 python3 - <<'EOF' 2>&1 | grep -v "^\["
import numpy as np, time
from rob import Robot, log
r=Robot("dbg")
p,q=r.fk(); x,y,z,w=q
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
q0=r.arm_q(); t=time.time()
s=r.ik_local(p+[0,0,0.015],R,q0)
print("ik_local",None if s is None else np.round(s-q0,3), "t",round(time.time()-t,1))
if s is not None:
    pp,_=r.fk(s); print("fk of sol", np.round(pp-p,4))
EOF

# openrua op 73
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rob.py", "old_string": "            rot = np.zeros(3) if ang < 1e-9 else axis / np.linalg.norm(axis) * ang", "new_string": "            na = np.linalg.norm(axis)\\n            rot = np.zeros(3) if na < 1e-9 else axis / na * ang", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 74
timeout 900 python3 - <<'EOF' 2>&1 | grep -v "^\["
import numpy as np, time
from rob import Robot, log
r=Robot("dbg")
p,q=r.fk(); x,y,z,w=q
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
q0=r.arm_q(); t=time.time()
s=r.ik_local(p+[0,0,0.015],R,q0)
print("ik_local",None if s is None else np.round(s-q0,3), "t",round(time.time()-t,1))
if s is not None:
    pp,_=r.fk(s); print("fk of sol", np.round(pp-p,4))
EOF

# openrua op 75
timeout 900 python3 -u nudge.py 0 0 0.12 4 2>&1 | grep -v "^\["

# openrua op 76
ls *.py && grep -l "PCA\|ridge\|eigh\|svd" *.py 2>/dev/null; sed -n 1,40p cloud.py

# openrua op 77
timeout 300 python3 cloud.py robot0_eye_in_hand 2>&1 | grep -v "^\[" | tail -3; timeout 300 python3 - <<'EOF'
import numpy as np
C=np.load("robot0_eye_in_hand_cloud.npy").reshape(-1,3)
C=C[np.isfinite(C).all(1)]
# pot1 region: above table, within region around previous estimate
m=(C[:,2]>0.905)&(C[:,2]<1.05)&(C[:,0]>-0.3)&(C[:,0]<0.05)&(C[:,1]>-0.5)&(C[:,1]<-0.1)
P=C[m]; print("pts",len(P), "z max",P[:,2].max().round(4))
# ridge: top points (within 8mm of the max in their xy neighborhood) -> approximate by z > zmax-0.012
zm=P[:,2].max()
top=P[P[:,2]>zm-0.012]
c=top[:,:2].mean(0); U,S,Vt=np.linalg.svd(top[:,:2]-c)
u=Vt[0]; print("centre",c.round(4),"u",u.round(3),"spread",S.round(3), "ridge z mean", top[:,2].mean().round(4), "n",len(top))
# project all pot points onto axis to get extents
t=(P[:,:2]-c)@u; print("t range",t.min().round(3),t.max().round(3))
# radial profile along axis
for a in np.arange(t.min(),t.max(),0.01):
    s=P[(t>=a)&(t<a+0.01)]
    if len(s): 
        w=(s[:,:2]-c)@np.array([-u[1],u[0]])
        print(f"t {a:+.3f} n {len(s):4d} width {w.min():+.3f}..{w.max():+.3f} zmax {s[:,2].max():.3f}")
EOF

# openrua op 78
timeout 300 python3 - <<'EOF'
import numpy as np
C=np.load("robot0_eye_in_hand_cloud.npy").reshape(-1,3)
C=C[np.isfinite(C).all(1)]
m=(C[:,2]>0.905)&(C[:,2]<1.05)&(C[:,0]>-0.3)&(C[:,0]<0.05)&(C[:,1]>-0.5)&(C[:,1]<-0.1)
P=C[m]
c=np.array([-0.1135,-0.2701]); u=np.array([0.483,-0.876]); n=np.array([-u[1],u[0]])
for it in range(3):
    t=(P[:,:2]-c)@u; w=(P[:,:2]-c)@n
    pts=[]
    for a in np.arange(-0.07,0.06,0.005):
        s=(t>=a)&(t<a+0.005)
        if s.sum()<20: continue
        zz=P[s,2]; k=zz>zz.max()-0.003
        pts.append([t[s][k].mean(), w[s][k].mean(), zz[k].mean()])
    pts=np.array(pts)
    A=np.c_[pts[:,0],np.ones(len(pts))]; slope,off=np.linalg.lstsq(A,pts[:,1],rcond=None)[0]
    # update axis direction
    du=u+slope*n; du/=np.linalg.norm(du); c=c+off*n; u=du; n=np.array([-u[1],u[0]])
    print("iter",it,"c",c.round(4),"u",u.round(4),"slope",round(slope,4),"off",round(off,4),"ridge z", pts[:,2].mean().round(4), "resid", np.abs(pts[:,1]-(A@[slope,off])).max().round(4))
t=(P[:,:2]-c)@u; w=(P[:,:2]-c)@n
# body extent: bins where |w|<0.045 and zmax>0.96
for a in np.arange(-0.09,0.10,0.005):
    s=(t>=a)&(t<a+0.005)&(np.abs(w)<0.045)
    if s.sum()>5: print(f"t {a:+.3f} n {s.sum():4d} zmax {P[s,2].max():.3f} wmin {w[s].min():+.3f} wmax {w[s].max():+.3f}")
EOF

# openrua op 79
nohup python3 -u standup.py -0.1513 -0.2076 -0.0728 -0.3435 0.939 0.072 > standup4.log 2>&1 &
sleep 60; grep -v "^\[" /workspace/standup4.log

# openrua op 80
sleep 45; grep -v "^\[" /workspace/standup4.log | tail -n +16

# openrua op 81
timeout 300 python3 - <<'EOF' 2>&1 | grep -v "^\["
import numpy as np
from rob import Robot
r=Robot("chk")
p,q=r.fk(); x,y,z,w=q
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
print("hand",p.round(4)); print("R\n",R.round(3))
c=np.array([-0.1148,-0.2708]); u=np.array([0.5,-0.866]); n=np.array([-u[1],u[0]])
for s in (+1,-1):
    f=p+s*0.04*R[:,1]+0.095*R[:,2]
    print("finger",s,f.round(4),"t",((f[:2]-c)@u).round(4),"w",((f[:2]-c)@n).round(4))
d=r.joints(); print({k:round(v,4) for k,v in d.items() if 'finger' in k})
EOF

# openrua op 82
cat geom.py

# openrua op 83
timeout 300 python3 - <<'EOF' 2>&1 | grep -v "^\["
import numpy as np
from rob import Robot
from geom import R_PLACE, Rz, R_to_q
r=Robot("chk")
R_G=Rz(np.radians(-60))@R_PLACE
print("R_G\n",R_G.round(3), "q",R_to_q(R_G).round(3))
q=r.ik([-0.1153,-0.2699,1.2024],R_G,r.arm_q())
print("q sol",q.round(3))
p,qu=r.fk(q); print("fk quat",qu.round(3),"pos",p.round(4))
x,y,z,w=qu
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
print("R fk\n",R.round(3))
EOF

# openrua op 84
timeout 600 python3 - <<'EOF' 2>&1 | grep -v "^\["
import numpy as np
from rob import Robot
from geom import R_PLACE, Rz, R_to_q
r=Robot("chk")
def yaw_of(qu):
    x,y,z,w=qu
    R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
    return np.degrees(np.arctan2(R[1,0],R[0,0]))
pos=[-0.1153,-0.2699,1.2024]
q0=r.arm_q()
seeds=[q0, np.array([0,-0.785,0,-2.356,0,1.571,0.785]), np.array([-0.5,0.3,0,-1.6,0,1.9,0.0])]
for t in range(-90,91,15):
    R=Rz(np.radians(t))@R_PLACE
    out=[]
    for s in seeds:
        q=r.ik(pos,R,s)
        if q is None: out.append("fail"); continue
        p,qu=r.fk(q); out.append(f"{yaw_of(qu):.0f}(j7={q[6]:.2f},j1={q[0]:.2f})")
    print("req yaw",t,"->",out)
EOF

# openrua op 85
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rob.py", "old_string": "    def ik(self, pos, R, seed=None):\\n        q = R_to_q(R)", "new_string": "    # /compute_ik solves for the panda_arm tip link (panda_link8); panda_hand is link8 rotated\\n    # -45deg about z (same origin). All poses in this class are panda_hand poses (as /compute_fk\\n    # reports), so convert before asking IK.\\n    HAND_TO_LINK8 = np.array([[np.cos(np.pi / 4), -np.sin(np.pi / 4), 0],\\n                              [np.sin(np.pi / 4), np.cos(np.pi / 4), 0], [0, 0, 1]])\\n\\n    def ik(self, pos, R, seed=None):\\n        q = R_to_q(np.asarray(R) @ self.HAND_TO_LINK8)", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 86
timeout 600 python3 - <<'EOF' 2>&1 | grep -v "^\["
import numpy as np
from rob import Robot
from geom import R_PLACE, R_PICK, Rz, Ry
r=Robot("chk")
def R_of(qu):
    x,y,z,w=qu
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
pos=[-0.1153,-0.2699,1.2024]
for name,R in [("R_G",Rz(np.radians(-60))@R_PLACE),("R_PICK",R_PICK),("R_F",Rz(np.radians(-60))@Ry(np.radians(-90))@R_PLACE)]:
    q=r.ik(pos,R,r.arm_q())
    if q is None: print(name,"fail"); continue
    p,qu=r.fk(q); print(name,"max |R_fk-R| =",np.abs(R_of(qu)-R).max().round(4))
EOF

# openrua op 87
timeout 900 python3 -u nudge.py 0 0 0.13 4 2>&1 | grep -v "^\["

# openrua op 88
mkdir -p "$(dirname /workspace/fitpot.py)"
cat > /workspace/fitpot.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Fit the axis of a moka pot lying on the table from the eye-in-hand cloud (ridge fit).
Usage: python3 fitpot.py <cx> <cy> <ux> <uy>   (initial guess of axis centre and direction)
Prints refined centre/direction, ridge height and the body extent along the axis."""
import sys
import numpy as np

C = np.load("robot0_eye_in_hand_cloud.npy").reshape(-1, 3)
C = C[np.isfinite(C).all(1)]
c = np.array([float(sys.argv[1]), float(sys.argv[2])])
u = np.array([float(sys.argv[3]), float(sys.argv[4])]); u /= np.linalg.norm(u)
m = (C[:, 2] > 0.905) & (C[:, 2] < 1.05) & (np.linalg.norm(C[:, :2] - c, axis=1) < 0.2)
P = C[m]
for it in range(4):
    n = np.array([-u[1], u[0]])
    t = (P[:, :2] - c) @ u; w = (P[:, :2] - c) @ n
    pts = []
    for a in np.arange(-0.07, 0.06, 0.005):
        s = (t >= a) & (t < a + 0.005) & (np.abs(w) < 0.05)
        if s.sum() < 20:
            continue
        zz = P[s, 2]; k = zz > zz.max() - 0.003
        pts.append([t[s][k].mean(), w[s][k].mean(), zz[k].mean()])
    pts = np.array(pts)
    A = np.c_[pts[:, 0], np.ones(len(pts))]
    slope, off = np.linalg.lstsq(A, pts[:, 1], rcond=None)[0]
    c = c + off * n
    u = u + slope * n; u /= np.linalg.norm(u)
n = np.array([-u[1], u[0]])
t = (P[:, :2] - c) @ u; w = (P[:, :2] - c) @ n
body = (np.abs(w) < 0.045) & (P[:, 2] > 0.955)
print(f"centre {c.round(4)} u {u.round(4)} ridge z {pts[:,2].mean():.4f} resid {np.abs(pts[:,1]-A@[slope,off]).max():.4f}")
print(f"body t range {t[body].min():+.3f} .. {t[body].max():+.3f}")
for a in np.arange(-0.10, 0.11, 0.01):
    s = (t >= a) & (t < a + 0.01) & (np.abs(w) < 0.05)
    if s.sum() > 5:
        print(f"t {a:+.2f} n {s.sum():4d} zmax {P[s,2].max():.3f} w {w[s].min():+.3f}..{w[s].max():+.3f}")
OPENRUA_EOF

# openrua op 89
timeout 300 python3 cloud.py robot0_eye_in_hand 2>&1 | grep -v "^\[" | tail -1; python3 fitpot.py -0.1148 -0.2708 0.5 -0.866

# openrua op 90
nohup python3 -u standup.py -0.1532 -0.2100 -0.0670 -0.3412 0.939 0.068 > standup5.log 2>&1 &
sleep 75; grep -v "^\[" /workspace/standup5.log

# openrua op 91
sleep 60; grep -v "^\[" /workspace/standup5.log | tail -n +16

# openrua op 92
sleep 50; grep -v "^\[" /workspace/standup5.log | tail -n +22

# openrua op 93
ls *.py; grep -l "imwrite\|cam_snap" *.py; ls *.png | tail -5

# openrua op 94
ls /workspace; ls /workspace/tools 2>/dev/null; grep -rl "cam_snap" /workspace --include=* 2>/dev/null | head

# openrua op 95
sed -n 1,30p tools/perception/cam_snap.py | head -40; grep -n "argv\|argparse\|add_argument" tools/perception/cam_snap.py | head

# openrua op 96
timeout 120 python3 tools/perception/cam_snap.py sideview side9.png 2>&1 | grep -v "^\[" ; timeout 120 python3 tools/perception/cam_snap.py frontview front7.png 2>&1 | grep -v "^\["; timeout 120 python3 tools/perception/cam_snap.py agentview agent11.png 2>&1 | grep -v "^\["

# openrua op 97
timeout 300 python3 - <<'EOF' 2>&1 | grep -v "^\["
import numpy as np
from rob import Robot
r=Robot("chk")
print("q",r.arm_q().round(3))
p,qu=r.fk(); x,y,z,w=qu
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
print("hand",p.round(4)); print("R\n",R.round(3)); print("tips",(p+0.1034*R[:,2]).round(4)); print("fingers",r.fingers())
EOF

# openrua op 98
timeout 900 python3 -u nudge.py 0 0.05 0.12 4 2>&1 | grep -v "^\["

# openrua op 99
timeout 900 python3 - <<'EOF' 2>&1 | grep -v "^\["
import numpy as np
from rob import Robot, log
from geom import R_PLACE
r=Robot("mv")
ok=r.move_pose([-0.15,-0.25,1.25],R_PLACE,6.0,flip_ok=True)
log("ok",ok,"q",r.arm_q().round(2))
EOF

# openrua op 100
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rob.py", "old_string": "        log(f\\"joint travel {np.round(q - self.arm_q(), 2)}\\")\\n        ok = self.move_joints(q, secs, via=via)", "new_string": "        travel = np.abs(q - self.arm_q()).max()\\n        secs = max(secs, travel / 0.4)      # controller aborts (-5) on fast large joint moves\\n        log(f\\"joint travel {np.round(q - self.arm_q(), 2)} in {secs:.1f}s\\")\\n        ok = self.move_joints(q, secs, via=via)", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 101
timeout 300 python3 cloud.py robot0_eye_in_hand 2>&1 | grep -v "^\[" | tail -1; timeout 300 python3 - <<'EOF'
import numpy as np
C=np.load("robot0_eye_in_hand_cloud.npy").reshape(-1,3); C=C[np.isfinite(C).all(1)]
m=(C[:,2]>0.905)&(C[:,2]<1.05)&(C[:,0]<0.0)&(C[:,1]<-0.05)
P=C[m]; print("n",len(P),"x",P[:,0].min().round(3),P[:,0].max().round(3),"y",P[:,1].min().round(3),P[:,1].max().round(3),"zmax",P[:,2].max().round(3))
top=P[P[:,2]>P[:,2].max()-0.012]; c=top[:,:2].mean(0); U,S,Vt=np.linalg.svd(top[:,:2]-c); print("c",c.round(4),"u",Vt[0].round(3),"S",S.round(2))
EOF

# openrua op 102
python3 fitpot.py -0.1871 -0.3362 -0.999 -0.045; python3 fitpot.py -0.1871 -0.3362 0.999 0.045 | head -3

# openrua op 103
sed -i 's/np.arange(-0.07, 0.06, 0.005)/np.arange(-0.12, 0.12, 0.005)/; s/np.arange(-0.10, 0.11, 0.01)/np.arange(-0.14, 0.14, 0.01)/' fitpot.py && python3 fitpot.py -0.1871 -0.3362 -0.999 -0.045

# openrua op 104
timeout 900 python3 - <<'EOF' 2>&1 | grep -v "^\["
import numpy as np
from rob import Robot, log
from geom import R_PLACE
r=Robot("mv")
ok=r.move_pose([-0.15,-0.34,1.28],R_PLACE,4.0,flip_ok=True); log("ok",ok)
EOF
timeout 300 python3 cloud.py robot0_eye_in_hand 2>&1 | grep -v "^\[" | tail -1
sed -i 's/np.arange(-0.12, 0.12, 0.005)/np.arange(-0.09, 0.07, 0.005)/' fitpot.py
python3 fitpot.py -0.1871 -0.3369 -0.999 -0.045

# openrua op 105
python3 -c "import scipy; print(scipy.__version__)"; grep -n "limit" robot.urdf | grep -i "panda_joint[1-7]" | head; grep -n -A3 'name="panda_joint7"' robot.urdf | head; grep -n "<limit" robot.urdf | head -9

# openrua op 106
grep -o '<joint name="panda_joint[1-7]"[^§]*\?<limit[^/]*/>' robot.urdf | grep -o 'name="panda_joint[0-9]"\|lower="[^"]*" upper="[^"]*"' | paste - - 

# openrua op 107
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rob.py", "old_string": "    def move_joints(self, q, secs, via=None):", "new_string": "    Q_LO = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])\\n    Q_HI = np.array([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973])\\n\\n    def within_limits(self, q, margin=0.05):\\n        return bool(np.all(q > self.Q_LO + margin) and np.all(q < self.Q_HI - margin))\\n\\n    def plan_line(self, q0, p1, R, step=0.015):\\n        \\"\\"\\"Joint waypoints for a straight hand path from fk(q0) to p1 at fixed R (on-branch).\\"\\"\\"\\n        p0, _ = self.fk(q0)\\n        n = max(1, int(np.ceil(np.linalg.norm(np.asarray(p1) - p0) / step)))\\n        path, q = [], np.array(q0)\\n        for i in range(1, n + 1):\\n            q = self.ik_local(p0 + (np.asarray(p1) - p0) * i / n, R, q)\\n            if q is None or not self.within_limits(q):\\n                log(f\\"plan_line: fail at waypoint {i}/{n}\\" + (\\"\\" if q is None else f\\" (limits) {np.round(q,2)}\\"))\\n                return None\\n            path.append(q)\\n        return path\\n\\n    def plan_rot(self, q0, G, R1, step_deg=10.0):\\n        \\"\\"\\"Joint waypoints rotating the hand about the fixed world point G (rigidly attached to the\\n        hand) from its pose at q0 to orientation R1.\\"\\"\\"\\n        from scipy.spatial.transform import Rotation as Rot\\n        p0, qu = self.fk(q0)\\n        R0 = Rot.from_quat(qu).as_matrix()\\n        off = R0.T @ (p0 - np.asarray(G))          # G->hand offset in hand frame\\n        dR = Rot.from_matrix(R1 @ R0.T)\\n        ang = np.degrees(dR.magnitude())\\n        n = max(1, int(np.ceil(ang / step_deg)))\\n        path, q = [], np.array(q0)\\n        for i in range(1, n + 1):\\n            Ri = (Rot.from_rotvec(dR.as_rotvec() * i / n)).as_matrix() @ R0\\n            pi_ = np.asarray(G) + Ri @ off\\n            q = self.ik_local(pi_, Ri, q)\\n            if q is None or not self.within_limits(q):\\n                log(f\\"plan_rot: fail at step {i}/{n}\\" + (\\"\\" if q is None else f\\" (limits) {np.round(q,2)}\\"))\\n                return None\\n            path.append(q)\\n        return path\\n\\n    def move_path(self, path, secs):\\n        \\"\\"\\"Execute a precomputed joint path (from the current configuration) as one trajectory;\\n        time is stretched so no joint exceeds ~0.4 rad/s.\\"\\"\\"\\n        qs = [self.arm_q()] + list(path)\\n        travel = sum(np.abs(np.array(b) - np.array(a)).max() for a, b in zip(qs[:-1], qs[1:]))\\n        secs = max(secs, travel / 0.4)\\n        ok = self.move_joints(path[-1], secs, via=path[:-1])\\n        p, _ = self.fk()\\n        log(f\\"move_path ({len(path)} pts, {secs:.1f}s) ok={ok}; hand now {np.round(p, 4)}\\")\\n        return ok\\n\\n    def move_joints(self, q, secs, via=None):", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 108
mkdir -p "$(dirname /workspace/standup2.py)"
cat > /workspace/standup2.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Stand up a moka pot lying on the table (pre-planned, on-branch joint paths).
Usage: python3 -u standup2.py <bx> <by> <kx> <ky> <z_axis> [d_grasp=0.068]
  (bx,by): world xy of the base face centre; (kx,ky): xy of the lid knob (axis direction);
  z_axis: height of the pot axis; d_grasp: grasp distance from the base (neck ~0.068).
Top-down grasp with fingers closing perpendicular to the axis, lift, rotate the hand about the
grasp point until the lid points up, lower until the base is ~3 mm above the table, release.
"""
import sys
import numpy as np
from geom import R_PLACE, Ry, Rz
from rob import Robot, log

bx, by, kx, ky, gz = map(float, sys.argv[1:6])
d_grasp = float(sys.argv[6]) if len(sys.argv) > 6 else 0.068
u = np.array([kx - bx, ky - by]); u /= np.linalg.norm(u)
phi = np.arctan2(u[1], u[0])
gx, gy = bx + d_grasp * u[0], by + d_grasp * u[1]
TABLE_Z = 0.90
TIP = 0.1034
R_G0 = Rz(phi) @ R_PLACE                        # hand z down, fingers close perpendicular to axis
R_F0 = Rz(phi) @ Ry(np.radians(-90)) @ R_PLACE  # hand z -> +u (lid side), pot upright
log(f"axis dir {np.round(u,3)} phi {np.degrees(phi):.1f} deg; grasp at {np.round((gx,gy,gz),3)}")

r = Robot("standup2")
q_now = r.arm_q()

G = np.array([gx, gy, gz])
G_high = G + [0, 0, 0.18]
G_down = np.array([gx, gy, TABLE_Z + d_grasp + 0.003])
FLIP = np.diag([-1.0, -1.0, 1.0])
seeds = [q_now, np.array([0, -0.785, 0, -2.356, 0, 1.571, 0.785]),
         np.array([-0.5, 0.3, 0, -1.6, 0, 1.9, 0.0]), np.array([-0.8, 0.5, 0.3, -1.8, 0, 2.2, 0.5])]


def plan(fl):
    R_G, R_F = R_G0 @ fl, R_F0 @ fl
    p_hover = G + [0, 0, 0.16] - TIP * R_G[:, 2]
    p_grasp = G - TIP * R_G[:, 2]
    p_high = G_high - TIP * R_G[:, 2]
    p_down = G_down - TIP * R_F[:, 2]
    p_retreat = p_down + np.array([-0.06 * u[0], -0.06 * u[1], 0.12])
    for s in seeds:
        q_h = r.ik(p_hover, R_G, s)
        if q_h is not None and r.within_limits(q_h):
            break
    else:
        log(f"flip={fl[0,0]}: no hover IK"); return None
    segs = {}
    segs["descend"] = r.plan_line(q_h, p_grasp, R_G)
    if segs["descend"] is None: return None
    segs["lift"] = r.plan_line(segs["descend"][-1], p_high, R_G)
    if segs["lift"] is None: return None
    segs["rot"] = r.plan_rot(segs["lift"][-1], G_high, R_F)
    if segs["rot"] is None: return None
    segs["down"] = r.plan_line(segs["rot"][-1], p_down, R_F)
    if segs["down"] is None: return None
    segs["retreat"] = r.plan_line(segs["down"][-1], p_retreat, R_F)
    if segs["retreat"] is None: return None
    rot_travel = np.abs(segs["rot"][-1] - segs["lift"][-1]).max()
    log(f"flip={fl[0,0]} feasible; hover q {np.round(q_h,2)}; rot joint travel {rot_travel:.2f}")
    return dict(R_G=R_G, R_F=R_F, q_h=q_h, p_grasp=p_grasp, p_high=p_high, p_down=p_down,
                p_retreat=p_retreat, rot_travel=rot_travel)


plans = [p for p in (plan(np.eye(3)), plan(FLIP)) if p is not None]
if not plans:
    log("no feasible plan; abort"); sys.exit(1)
P = min(plans, key=lambda p: p["rot_travel"])
log(f"chosen flip with rot travel {P['rot_travel']:.2f}")

r.gripper(0.04)
assert r.move_path([P["q_h"]], 4.0), "hover failed"
assert r.move_path(r.plan_line(r.arm_q(), P["p_grasp"], P["R_G"]), 5.0), "descend failed"
f = r.gripper(0.0)
gap = f[0] - f[1]
log(f"gap after close {gap:.4f}")
if gap < 0.04:
    log("GRASP FAILED (gap too small for a pot body)")
    r.gripper(0.04)
    r.move_path(r.plan_line(r.arm_q(), P["p_grasp"] + [0, 0, 0.16], P["R_G"]), 4.0)
    sys.exit(2)
assert r.move_path(r.plan_line(r.arm_q(), P["p_high"], P["R_G"]), 4.0), "lift failed"
log("rotating pot upright")
rot = r.plan_rot(r.arm_q(), G_high, P["R_F"])
assert rot is not None, "rotation plan failed live"
ok = r.move_path(rot, 8.0)
log(f"rotate ok={ok} fingers {r.fingers()}")
down = r.plan_line(r.arm_q(), P["p_down"], P["R_F"])
assert down is not None, "down plan failed"
r.move_path(down, 5.0)
log(f"fingers before release {r.fingers()}")
r.gripper(0.04)
ret = r.plan_line(r.arm_q(), P["p_retreat"], P["R_F"])
if ret is not None:
    r.move_path(ret, 3.0)
log("STANDUP DONE")
OPENRUA_EOF

# openrua op 109
nohup python3 -u standup2.py -0.1321 -0.3351 -0.2868 -0.3439 0.939 0.068 > standup6.log 2>&1 &
sleep 45; grep -v "^\[" /workspace/standup6.log

# openrua op 110
timeout 600 python3 - <<'EOF' 2>&1 | grep -v "^\["
import numpy as np
from rob import Robot, log
from geom import R_PLACE, Ry, Rz
r=Robot("dbg")
u=np.array([-0.998,-0.057]); u/=np.linalg.norm(u); phi=np.arctan2(u[1],u[0])
R_G0=Rz(phi)@R_PLACE; FLIP=np.diag([-1.,-1.,1.])
G=np.array([-0.2,-0.339,0.939])
seeds=[r.arm_q(), np.array([0,-0.785,0,-2.356,0,1.571,0.785]), np.array([-0.5,0.3,0,-1.6,0,1.9,0.0]), np.array([-0.8,0.5,0.3,-1.8,0,2.2,0.5])]
for fl in (np.eye(3),FLIP):
    R=R_G0@fl
    for name,p in (("hover",G+[0,0,0.16]-0.1034*R[:,2]),("grasp",G-0.1034*R[:,2])):
        for s in seeds:
            q=r.ik(p,R,s)
            print(fl[0,0],name,None if q is None else (np.round(q,2), r.within_limits(q)))
EOF

# openrua op 111
timeout 600 python3 - <<'EOF' 2>&1 | grep -v "^\["
import numpy as np
from rob import Robot, log
from geom import R_PLACE, Rz
r=Robot("dbg")
u=np.array([-0.998,-0.057]); u/=np.linalg.norm(u); phi=np.arctan2(u[1],u[0])
R=Rz(phi)@R_PLACE@np.diag([-1.,-1.,1.])
G=np.array([-0.2,-0.339,0.939]); p_h=G+[0,0,0.16]-0.1034*R[:,2]; p_g=G-0.1034*R[:,2]
q=np.array([-0.79,0.26,0.15,-2.13,-0.06,2.38,0.13])
q=r.ik(p_h,R,q); print("q_h",q.round(3)); 
p,qu=r.fk(q); print("fk",p.round(4),qu.round(4))
# step by step
for i in range(1,4):
    tgt=p_h+(p_g-p_h)*i/10
    qq=q.copy()
    for it in range(12):
        e=r.ik_local.__func__  # placeholder
        break
    q2=r.ik_local(tgt,R,q)
    if q2 is None:
        # inspect FK of q
        p,qu=r.fk(q); print("stuck at", i, "fk pos",p.round(4),"tgt",tgt.round(4)); break
    print(i, np.round(q2-q,3)); q=q2
EOF

# openrua op 112
grep -n "def ik_local" -A 40 rob.py | sed -n 1,45p

# openrua op 113
python3 - <<'EOF'
import re
s=open('rob.py').read()
old=s[s.index("    def ik_local"):s.index("    Q_LO = ")]
new='''    def ik_local(self, pos, R, seed, iters=30, tol=5e-4):
        """Damped-least-squares IK from `seed` using a numerical Jacobian (FK service), so the
        result stays on the seed's solution branch (the IK service sometimes jumps branches)."""
        q = np.array(seed, float)
        pos = np.asarray(pos, float)

        def pose_err(qq):
            p, quat = self.fk(qq)
            x, y, z, w = quat
            Rc = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                           [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                           [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
            Re = R @ Rc.T                     # rotation taking current to target
            ang = np.arccos(np.clip((np.trace(Re) - 1) / 2, -1, 1))
            axis = np.array([Re[2, 1] - Re[1, 2], Re[0, 2] - Re[2, 0], Re[1, 0] - Re[0, 1]])
            na = np.linalg.norm(axis)
            rot = np.zeros(3) if na < 1e-9 else axis / na * ang
            return np.concatenate([pos - p, rot])

        e = pose_err(q)
        for _ in range(iters):
            if np.linalg.norm(e[:3]) < tol and np.linalg.norm(e[3:]) < 4 * tol:
                return q
            J = np.zeros((6, 7))
            h = 1e-4
            for j in range(7):
                dq = np.zeros(7); dq[j] = h
                J[:, j] = (pose_err(q + dq) - e) / -h   # = d fk/dq (e = target - fk)
            step = J.T @ np.linalg.solve(J @ J.T + 1e-4 * np.eye(6), e)
            alpha = 1.0
            while alpha > 0.05:                         # backtracking so the error never grows
                qn = q + alpha * step
                en = pose_err(qn)
                if np.linalg.norm(en) < np.linalg.norm(e):
                    q, e = qn, en
                    break
                alpha *= 0.5
            else:
                break
        log(f"ik_local: not converged, pos err {np.linalg.norm(e[:3]):.4f} rot err {np.linalg.norm(e[3:]):.4f}")
        return None

'''
open('rob.py','w').write(s.replace(old,new))
EOF
nohup python3 -u standup2.py -0.1321 -0.3351 -0.2868 -0.3439 0.939 0.068 > standup6.log 2>&1 &
sleep 60; grep -v "^\[" /workspace/standup6.log

# openrua op 114
timeout 600 python3 - <<'EOF' 2>&1 | grep -v "^\["
import numpy as np
from rob import Robot, log
from geom import R_PLACE, Rz
r=Robot("dbg")
bx,by,kx,ky=-0.1321,-0.3351,-0.2868,-0.3439
u=np.array([kx-bx,ky-by]); u/=np.linalg.norm(u); phi=np.arctan2(u[1],u[0])
R=Rz(phi)@R_PLACE@np.diag([-1.,-1.,1.])
G=np.array([bx+0.068*u[0],by+0.068*u[1],0.939]); p_h=G+[0,0,0.16]-0.1034*R[:,2]; p_g=G-0.1034*R[:,2]
q_h=r.ik(p_h,R,r.arm_q()); print("q_h",q_h.round(3), r.within_limits(q_h))
p,qu=r.fk(q_h); print("fk",p.round(5),qu.round(5)); print("R\n",R.round(4))
x,y,z,w=qu
Rc=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
print("Rc\n",Rc.round(4))
q=q_h
for i in range(1,4):
    tgt=p_h+(p_g-p_h)*i/10
    q2=r.ik_local(tgt,R,q)
    print(i, None if q2 is None else np.round(q2-q,3))
    if q2 is None: break
    q=q2
EOF

# openrua op 115
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rob.py", "old_string": "        e = pose_err(q)\\n        for _ in range(iters):\\n            if np.linalg.norm(e[:3]) < tol and np.linalg.norm(e[3:]) < 4 * tol:\\n                return q", "new_string": "        e = pose_err(q)\\n        hist = []\\n        for _ in range(iters):\\n            hist.append(round(float(np.linalg.norm(e)), 5))\\n            if np.linalg.norm(e[:3]) < tol and np.linalg.norm(e[3:]) < 4 * tol:\\n                return q", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 116
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rob.py", "old_string": "        log(f\\"ik_local: not converged, pos err {np.linalg.norm(e[:3]):.4f} rot err {np.linalg.norm(e[3:]):.4f}\\")\\n        return None", "new_string": "        log(f\\"ik_local: not converged, pos err {np.linalg.norm(e[:3]):.4f} rot err {np.linalg.norm(e[3:]):.4f}\\"\\n            f\\" seed {np.round(seed,3)} target {np.round(pos,4)} hist {hist}\\")\\n        return None", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 117
nohup python3 -u standup2.py -0.1321 -0.3351 -0.2868 -0.3439 0.939 0.068 > standup6.log 2>&1 &
sleep 50; grep -v "^\[" /workspace/standup6.log

# openrua op 118
python3 - <<'EOF'
s=open('standup2.py').read()
old=s[s.index("def plan(fl):"):s.index("r.gripper(0.04)\n")]
new='''def plan(fl, phi2):
    """fl: hand flip about its z; phi2: azimuth the pot axis is yawed to (about the vertical
    through the grasp point) before the righting rotation, so the arm ends in a comfortable pose."""
    R_G, R_F = R_G0 @ fl, R_F0 @ fl
    u2 = np.array([np.cos(phi2), np.sin(phi2)])
    R_G2 = Rz(phi2) @ R_PLACE @ fl
    R_F2 = Rz(phi2) @ Ry(np.radians(-90)) @ R_PLACE @ fl
    p_hover = G + [0, 0, 0.16] - TIP * R_G[:, 2]
    p_grasp = G - TIP * R_G[:, 2]
    p_high = G_high - TIP * R_G[:, 2]
    p_down = G_down - TIP * R_F2[:, 2]
    p_retreat = p_down + np.array([-0.06 * u2[0], -0.06 * u2[1], 0.12])
    tag = f"flip={fl[0,0]:+.0f} phi2={np.degrees(phi2):.0f}"
    for s in seeds:
        q_h = r.ik(p_hover, R_G, s)
        if q_h is not None and r.within_limits(q_h):
            break
    else:
        log(f"{tag}: no hover IK"); return None
    segs = {}
    steps = [("descend", lambda q: r.plan_line(q, p_grasp, R_G)),
             ("lift", lambda q: r.plan_line(q, p_high, R_G)),
             ("yaw", lambda q: r.plan_rot(q, G_high, R_G2)),
             ("rot", lambda q: r.plan_rot(q, G_high, R_F2)),
             ("down", lambda q: r.plan_line(q, p_down, R_F2)),
             ("retreat", lambda q: r.plan_line(q, p_retreat, R_F2))]
    q = q_h
    for name, fn in steps:
        seg = fn(q)
        if seg is None:
            log(f"{tag}: segment {name} infeasible"); return None
        segs[name] = seg
        q = seg[-1]
    travel = sum(np.abs(np.diff(np.array([q_h] + segs["yaw"] + segs["rot"]), axis=0)).max(1).sum() for _ in [0])
    log(f"{tag} feasible; hover q {np.round(q_h,2)}; down q {np.round(segs['down'][-1],2)}; travel {travel:.2f}")
    return dict(R_G=R_G, R_G2=R_G2, R_F2=R_F2, q_h=q_h, p_grasp=p_grasp, p_high=p_high,
                p_down=p_down, p_retreat=p_retreat, travel=travel)


plans = []
for phi2_deg in (-60, -90, -30, -120, 0, -150):
    for fl in (np.eye(3), FLIP):
        pl = plan(fl, np.radians(phi2_deg))
        if pl is not None:
            plans.append(pl)
    if plans:
        break
if not plans:
    log("no feasible plan; abort"); sys.exit(1)
P = min(plans, key=lambda p: p["travel"])
log(f"chosen plan with travel {P['travel']:.2f}")

'''
s=s.replace(old,new)
old2=s[s.index('log("rotating pot upright")'):s.index('down = r.plan_line')]
new2='''log("yawing pot")
yaw = r.plan_rot(r.arm_q(), G_high, P["R_G2"])
assert yaw is not None, "yaw plan failed live"
assert r.move_path(yaw, 5.0), "yaw failed"
log("rotating pot upright")
rot = r.plan_rot(r.arm_q(), G_high, P["R_F2"])
assert rot is not None, "rotation plan failed live"
ok = r.move_path(rot, 8.0)
log(f"rotate ok={ok} fingers {r.fingers()}")
'''
s=s.replace(old2,new2)
s=s.replace('P["p_down"], P["R_F"]','P["p_down"], P["R_F2"]').replace('P["p_retreat"], P["R_F"]','P["p_retreat"], P["R_F2"]')
open('standup2.py','w').write(s)
EOF
grep -n "R_F\"\]\|R_F\b" standup2.py | head; sed -n 95,140p standup2.py

# openrua op 119
nohup python3 -u standup2.py -0.1321 -0.3351 -0.2868 -0.3439 0.939 0.068 > standup6.log 2>&1 &
sleep 90; grep -v "^\[" /workspace/standup6.log

# openrua op 120
sleep 75; grep -v "^\[" /workspace/standup6.log | tail -n +15

# openrua op 121
timeout 600 python3 - <<'EOF' 2>&1 | grep -v "^\["
import numpy as np
from rob import Robot, log
from geom import R_PLACE, Rz
r=Robot("dbg")
q0=r.arm_q(); print("q0",q0.round(3), "fingers", r.fingers())
p,qu=r.fk(); print("hand",p.round(4))
G_high=np.array([-0.2,-0.339,0.939+0.18])
FLIP=np.diag([-1.,-1.,1.])
R_G2=Rz(np.radians(-60))@R_PLACE@FLIP
path=r.plan_rot(q0,G_high,R_G2)
prev=q0
for q in path:
    print(np.round(q-prev,3)); prev=q
EOF

# openrua op 122
timeout 120 python3 tools/perception/cam_snap.py agentview agent12.png 2>&1 | grep -v "^\["; timeout 120 python3 tools/perception/cam_snap.py sideview side10.png 2>&1 | grep -v "^\["

# openrua op 123
timeout 300 python3 - <<'EOF' 2>&1 | grep -v "^\["
import numpy as np, time
from rob import Robot, log
from geom import R_PLACE, Rz
r=Robot("dbg")
q0=r.arm_q(); t=time.time()
p,qu=r.fk(); print("fk time",round(time.time()-t,2))
t=time.time(); q=r.ik_local(p+[0,0,0.01],Rz(0)@np.eye(3)@np.array([[1,0,0],[0,-1,0],[0,0,-1]]),q0); print("ik_local time",round(time.time()-t,2), q is not None)
EOF

# openrua op 124
timeout 300 python3 - <<'EOF' 2>&1 | grep -v "^\["
import numpy as np, time
from rob import Robot, log
r=Robot("dbg")
q0=r.arm_q()
for dq in (0,0.1,0.2):
    t=time.time(); p,qu=r.fk(q0+dq); print(dq, p.round(4), qu.round(3), round(time.time()-t,3))
EOF

# openrua op 125
timeout 200 python3 - <<'EOF' 2>&1 | grep -v "^\["
import numpy as np, time
t=time.time()
from scipy.spatial.transform import Rotation as Rot
print("scipy import", round(time.time()-t,2))
from rob import Robot, log
from geom import R_PLACE, Rz
r=Robot("dbg")
q0=r.arm_q(); print("q0",q0.round(3))
G_high=np.array([-0.2,-0.339,0.939+0.18])
FLIP=np.diag([-1.,-1.,1.])
R_G2=Rz(np.radians(-60))@R_PLACE@FLIP
t=time.time()
path=r.plan_rot(q0,G_high,R_G2)
print("plan time",round(time.time()-t,2), None if path is None else len(path))
prev=q0
for q in path or []:
    print(np.round(q-prev,3)); prev=q
EOF

# openrua op 126
timeout 200 python3 - <<'EOF' 2>&1 | grep -v "^\[INFO\|^\[WARN"
import numpy as np, time
from rob import Robot, log
from geom import R_PLACE, Rz
r=Robot("dbg")
q0=r.arm_q(); print("q0",q0.round(3))
G_high=np.array([-0.2,-0.339,0.939+0.18])
FLIP=np.diag([-1.,-1.,1.])
R_G2=Rz(np.radians(-60))@R_PLACE@FLIP
path=r.plan_rot(q0,G_high,R_G2)
prev=q0
for q in path or []:
    p,_=r.fk(q); print("dq",np.round(q-prev,3),"hand",p.round(4)); prev=q
EOF

# openrua op 127
timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih11.png 2>&1 | grep -v "^\[INFO"; timeout 120 python3 tools/perception/cam_snap.py frontview front8.png 2>&1 | grep -v "^\[INFO"

# openrua op 128
timeout 600 python3 - <<'EOF' 2>&1 | grep -v "^\[INFO\|^\[WARN"
import numpy as np, time
from rob import Robot, log
from geom import R_PLACE, Rz
r=Robot("dbg")
q0=r.arm_q(); print("q0",q0.round(3),"fingers",r.fingers())
G_high=np.array([-0.2,-0.339,0.939+0.18])
FLIP=np.diag([-1.,-1.,1.])
R_G2=Rz(np.radians(-60))@R_PLACE@FLIP
path=r.plan_rot(q0,G_high,R_G2)
ok=r.move_path(path[:2],4.0)
print("ok",ok,"q",r.arm_q().round(3),"fingers",r.fingers())
EOF

# openrua op 129
timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih12.png 2>&1 | grep -v "^\[INFO"

# openrua op 130
timeout 300 python3 cloud.py robot0_eye_in_hand 2>&1 | grep -v "^\[INFO" | tail -1; timeout 300 python3 - <<'EOF'
import numpy as np
C=np.load("robot0_eye_in_hand_cloud.npy").reshape(-1,3); C=C[np.isfinite(C).all(1)]
m=(C[:,2]>1.02)&(C[:,2]<1.19)
P=C[m]; print("n",len(P),"z",P[:,2].min().round(3),P[:,2].max().round(3))
c=P[:,:2].mean(0); U,S,Vt=np.linalg.svd(P[:,:2]-c); u=Vt[0]; print("c",c.round(4),"u",u.round(3))
t=(P[:,:2]-c)@u; n=np.array([-u[1],u[0]]); w=(P[:,:2]-c)@n
G=np.array([-0.2,-0.339]); print("t of G", ((G-c)@u).round(4), "w of G", ((G-c)@n).round(4))
for a in np.arange(t.min(),t.max(),0.01):
    s=(t>=a)&(t<a+0.01)
    if s.sum()>5: print(f"t {a:+.3f} n {s.sum():4d} z {P[s,2].min():.3f}..{P[s,2].max():.3f} w {w[s].min():+.3f}..{w[s].max():+.3f}")
EOF

# openrua op 131
timeout 300 python3 - <<'EOF' 2>&1 | grep -v "^\[INFO"
import numpy as np
from rob import Robot
r=Robot("chk")
p,qu=r.fk(); x,y,z,w=qu
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
print("hand",p.round(4)); print("hand x",R[:,0].round(3),"hand y (fingers)",R[:,1].round(3),"hand z",R[:,2].round(3))
C=np.load("robot0_eye_in_hand_cloud.npy").reshape(-1,3); C=C[np.isfinite(C).all(1)]
# express cloud in hand frame
H=(C-p)@R
m=(H[:,2]>0.05)&(H[:,2]<0.2)   # below the hand origin along z (down)
Q=H[m]; print("n",len(Q))
for name,i in (("hx",0),("hy",1),("hz",2)): print(name, Q[:,i].min().round(3), Q[:,i].max().round(3))
# histogram along hand x (perp to fingers, expected pot axis direction)
for a in np.arange(-0.12,0.12,0.01):
    s=(Q[:,0]>=a)&(Q[:,0]<a+0.01)
    if s.sum()>5: print(f"hx {a:+.2f} n {s.sum():5d} hy {Q[s,1].min():+.3f}..{Q[s,1].max():+.3f} hz {Q[s,2].min():.3f}..{Q[s,2].max():.3f}")
EOF

# openrua op 132
mkdir -p "$(dirname /workspace/finish_standup.py)"
cat > /workspace/finish_standup.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Finish righting a pot already held horizontally in the gripper (top-down neck grasp).
Usage: python3 -u finish_standup.py <phi2_deg> <d_eff> [hz_pad=0.105]
  phi2_deg: azimuth the pot axis (base->lid) is yawed to before righting; the hand's -x axis
            currently points to the lid.
  d_eff:    distance from the pad contact point to the pot base (m).
"""
import sys
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from geom import R_PLACE, Ry, Rz
from rob import Robot, log

phi2 = np.radians(float(sys.argv[1]))
d_eff = float(sys.argv[2])
HZ = float(sys.argv[3]) if len(sys.argv) > 3 else 0.105
TABLE_Z = 0.90
FLIP = np.diag([-1.0, -1.0, 1.0])
u2 = np.array([np.cos(phi2), np.sin(phi2), 0.0])

r = Robot("finish")


def hand_R():
    p, qu = r.fk()
    return p, Rot.from_quat(qu).as_matrix()


p, R = hand_R()
log(f"hand {np.round(p,4)} x {np.round(R[:,0],3)} z {np.round(R[:,2],3)} fingers {r.fingers()}")
# pick the flip so that the hand's -x axis (lid direction) matches u2 after the yaw
R_G2 = Rz(phi2) @ R_PLACE @ FLIP
if np.dot(R_G2[:, 0], -u2) < 0.9:
    R_G2 = Rz(phi2) @ R_PLACE
assert np.dot(R_G2[:, 0], -u2) > 0.9, "flip logic"
R_F2 = R_G2 @ np.linalg.inv(R_G2) @ Rz(phi2) @ Ry(np.radians(-90)) @ np.linalg.inv(Rz(phi2)) @ R_G2
log(f"R_F2 z {np.round(R_F2[:,2],3)} (should be u2 {np.round(u2,3)}), x {np.round(R_F2[:,0],3)} (should be +z)")

# 1. yaw about the vertical through the pad point, in chunks
Gp = p + HZ * R[:, 2]
yaw = r.plan_rot(r.arm_q(), Gp, R_G2, step_deg=8.0)
assert yaw is not None, "yaw plan failed"
log(f"yaw: {len(yaw)} steps")
for i in range(0, len(yaw), 3):
    chunk = yaw[i:i + 3]
    ok = r.move_path(chunk, 6.0)
    log(f"yaw chunk ok={ok} fingers {r.fingers()}")
    if not ok:
        log("yaw stalled; abort"); sys.exit(1)

# 2. right the pot: rotate about the pad point
p, R = hand_R()
Gp = p + HZ * R[:, 2]
rot = r.plan_rot(r.arm_q(), Gp, R_F2, step_deg=6.0)
assert rot is not None, "rot plan failed"
for i in range(0, len(rot), 3):
    ok = r.move_path(rot[i:i + 3], 6.0)
    log(f"rot chunk ok={ok} fingers {r.fingers()}")
    if not ok:
        log("rotation stalled; abort"); sys.exit(1)
p, R = hand_R()
log(f"after rotation hand {np.round(p,4)} z {np.round(R[:,2],3)} x {np.round(R[:,0],3)}")

# 3. lower until the base is 3 mm above the table (pad point z = table + d_eff + 0.003)
Gp = p + HZ * R[:, 2]
p_down = p + np.array([0, 0, TABLE_Z + d_eff + 0.003 - Gp[2]])
down = r.plan_line(r.arm_q(), p_down, R)
assert down is not None, "down plan failed"
ok = r.move_path(down, 6.0)
log(f"down ok={ok} fingers {r.fingers()}")
r.gripper(0.04)
p, R = hand_R()
ret = r.plan_line(r.arm_q(), p - 0.07 * u2 + [0, 0, 0.10], R)
if ret is not None:
    r.move_path(ret, 4.0)
log("FINISH DONE")
OPENRUA_EOF

# openrua op 133
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/finish_standup.py", "old_string": "R_F2 = R_G2 @ np.linalg.inv(R_G2) @ Rz(phi2) @ Ry(np.radians(-90)) @ np.linalg.inv(Rz(phi2)) @ R_G2", "new_string": "dR = Rz(phi2) @ Ry(np.radians(-90)) @ Rz(-phi2)   # -90deg about n2: maps u2 -> +z, -z -> +u2\\nR_F2 = dR @ R_G2", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 134
nohup python3 -u finish_standup.py -60 0.058 0.105 > finish1.log 2>&1 &
sleep 90; grep -v "^\[INFO\|^\[WARN" /workspace/finish1.log

# openrua op 135
pkill -f finish_standup.py; sleep 1; grep -v "^\[INFO\|^\[WARN" /workspace/finish1.log | tail -3; timeout 120 python3 tools/perception/cam_snap.py agentview agent13.png 2>&1 | grep -v "^\[INFO"

# openrua op 136
pgrep -fa finish_standup; grep -v "^\[INFO\|^\[WARN" /workspace/finish1.log | tail -3

# openrua op 137
timeout 120 python3 tools/perception/cam_snap.py agentview agent13.png 2>&1 | grep -v "^\[INFO"; timeout 120 python3 tools/perception/cam_snap.py frontview front9.png 2>&1 | grep -v "^\[INFO"

# openrua op 138
timeout 900 python3 - <<'EOF' 2>&1 | grep -v "^\[INFO\|^\[WARN"
import numpy as np
from rob import Robot, log
from geom import R_PLACE
r=Robot("mv")
r.gripper(0.04)
ok=r.move_pose([-0.15,-0.30,1.30],R_PLACE,5.0,flip_ok=True); log("ok",ok)
EOF
timeout 300 python3 cloud.py robot0_eye_in_hand 2>&1 | grep -v "^\[INFO" | tail -1
timeout 300 python3 - <<'EOF'
import numpy as np
C=np.load("robot0_eye_in_hand_cloud.npy").reshape(-1,3); C=C[np.isfinite(C).all(1)]
m=(C[:,2]>0.905)&(C[:,2]<1.05)&(C[:,0]<0.0)&(C[:,1]<-0.05)
P=C[m]; print("n",len(P),"x",P[:,0].min().round(3),P[:,0].max().round(3),"y",P[:,1].min().round(3),P[:,1].max().round(3),"zmax",P[:,2].max().round(3))
top=P[P[:,2]>P[:,2].max()-0.012]; c=top[:,:2].mean(0); U,S,Vt=np.linalg.svd(top[:,:2]-c); print("c",c.round(4),"u",Vt[0].round(3),"S",S.round(2))
EOF

# openrua op 139
sed -i 's|travel / 0.4|travel / 0.25|' rob.py; grep -n "0.25" rob.py | head -3; python3 fitpot.py -0.2371 -0.368 0.45 -0.893

# openrua op 140
timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih13.png 2>&1 | grep -v "^\[INFO"

# openrua op 141
timeout 900 python3 - <<'EOF' 2>&1 | grep -v "^\[INFO\|^\[WARN"
import numpy as np
from rob import Robot, log
from geom import R_PLACE
r=Robot("mv")
ok=r.move_pose([-0.29,-0.37,1.25],R_PLACE,5.0,flip_ok=True); log("ok",ok)
EOF
timeout 300 python3 cloud.py robot0_eye_in_hand 2>&1 | grep -v "^\[INFO" | tail -1
timeout 300 python3 - <<'EOF'
import numpy as np, cv2
C=np.load("robot0_eye_in_hand_cloud.npy").reshape(-1,3); C=C[np.isfinite(C).all(1)]
m=(C[:,2]>0.903)&(C[:,2]<1.10)&(C[:,0]>-0.40)&(C[:,0]<-0.08)&(C[:,1]<-0.20)&(C[:,1]>-0.52)
P=C[m]; print("n",len(P),"zmax",P[:,2].max().round(3))
# heightmap 1mm/px: x -> rows (x increasing downward), y -> cols
x0,y0=-0.40,-0.52; H=np.zeros((320,320)); 
ix=((P[:,0]-x0)*1000).astype(int); iy=((P[:,1]-y0)*1000).astype(int)
for a,b,z in zip(ix,iy,P[:,2]):
    if 0<=a<320 and 0<=b<320: H[a,b]=max(H[a,b],z)
img=np.zeros((320,320,3),np.uint8)
v=np.clip((H-0.90)/0.10,0,1); img[...,1]=(v*255).astype(np.uint8); img[H>0,2]=80
img=cv2.resize(img,(640,640),interpolation=cv2.INTER_NEAREST)
for k in range(0,320,50):
    cv2.line(img,(k*2,0),(k*2,639),(60,60,60),1); cv2.line(img,(0,k*2),(639,k*2),(60,60,60),1)
    cv2.putText(img,f"y{y0+k/1000:.2f}",(k*2,12),cv2.FONT_HERSHEY_SIMPLEX,0.35,(255,255,255),1)
    cv2.putText(img,f"x{x0+k/1000:.2f}",(0,k*2+12),cv2.FONT_HERSHEY_SIMPLEX,0.35,(255,255,255),1)
cv2.imwrite("hmap1.png",img)
EOF

# openrua op 142
timeout 300 python3 - <<'EOF'
import numpy as np
C=np.load("robot0_eye_in_hand_cloud.npy").reshape(-1,3); C=C[np.isfinite(C).all(1)]
m=(C[:,2]>0.903)&(C[:,2]<1.10)&(C[:,0]>-0.31)&(C[:,0]<-0.225)&(C[:,1]<-0.28)&(C[:,1]>-0.48)
P=C[m]
pts=[]
for y in np.arange(-0.47,-0.29,0.005):
    s=(P[:,1]>=y)&(P[:,1]<y+0.005)
    if s.sum()<20: continue
    zz=P[s,2]; k=zz>zz.max()-0.003
    pts.append([P[s][k][:,0].mean(), y+0.0025, zz.max(), P[s,0].min(), P[s,0].max()])
pts=np.array(pts)
for p in pts: print(f"y {p[1]:+.4f} ridge x {p[0]:+.4f} zmax {p[2]:.3f} x range {p[3]:+.3f}..{p[4]:+.3f} width {p[4]-p[3]:.3f}")
# fit line x = a*y + b over body slices with zmax>0.965
b=pts[pts[:,2]>0.965]; A=np.c_[b[:,1],np.ones(len(b))]; a,bb=np.linalg.lstsq(A,b[:,0],rcond=None)[0]
print("axis: x = %.4f*y + %.4f ; resid %.4f"%(a,bb,np.abs(b[:,0]-A@[a,bb]).max()))
EOF

# openrua op 143
mkdir -p "$(dirname /workspace/knobpick.py)"
cat > /workspace/knobpick.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Pinch a moka pot's lid knob from above and lift it (the pot then hangs upright).
Usage: python3 -u knobpick.py <kx> <ky> <kz_tips> <phi_deg> [lift=0.22]
  (kx,ky): world xy of the knob cap centre; kz_tips: fingertip height for the pinch;
  phi_deg: azimuth of the pot axis (fingers close perpendicular to it; irrelevant for an upright pot).
"""
import sys
import numpy as np
from geom import R_PLACE, Rz
from rob import Robot, log

kx, ky, kz, phi = map(float, sys.argv[1:5])
lift = float(sys.argv[5]) if len(sys.argv) > 5 else 0.22
TIP = 0.1034
R = Rz(np.radians(phi)) @ R_PLACE
r = Robot("knobpick")
FLIP = np.diag([-1.0, -1.0, 1.0])
p_grasp = np.array([kx, ky, kz]) - TIP * R[:, 2]
p_hover = p_grasp + [0, 0, 0.15]
p_lift = p_grasp + [0, 0, lift]

best = None
for fl in (np.eye(3), FLIP):
    for s in (r.arm_q(), np.array([0, -0.785, 0, -2.356, 0, 1.571, 0.785]),
              np.array([-0.5, 0.3, 0, -1.6, 0, 1.9, 0.0])):
        q = r.ik(p_hover, R @ fl, s)
        if q is None or not r.within_limits(q):
            continue
        d = r.plan_line(q, p_grasp, R @ fl)
        if d is None:
            continue
        u = r.plan_line(d[-1], p_lift, R @ fl)
        if u is None:
            continue
        travel = np.abs(q - r.arm_q()).max()
        if best is None or travel < best[0]:
            best = (travel, q, R @ fl)
assert best is not None, "no feasible hover/descent"
_, q_h, Rg = best
log(f"hover q {np.round(q_h,2)} fingers dir {np.round(Rg[:,1],3)}")
r.gripper(0.04)
assert r.move_path([q_h], 4.0), "hover failed"
assert r.move_path(r.plan_line(r.arm_q(), p_grasp, Rg), 5.0), "descend failed"
p, _ = r.fk()
log(f"tips at {np.round(p + TIP * Rg[:,2], 4)}")
f = r.gripper(0.0)
gap = f[0] - f[1]
log(f"gap after close {gap:.4f}")
if gap < 0.004:
    log("PINCH FAILED (closed on air)")
    r.gripper(0.04)
    r.move_path(r.plan_line(r.arm_q(), p_hover, Rg), 4.0)
    sys.exit(2)
assert r.move_path(r.plan_line(r.arm_q(), p_lift, Rg), 6.0), "lift failed"
f = r.fingers()
log(f"after lift gap {f[0]-f[1]:.4f}")
log("KNOBPICK DONE")
OPENRUA_EOF

# openrua op 144
nohup python3 -u knobpick.py -0.239 -0.443 0.930 -80.8 0.22 > knob1.log 2>&1 &
sleep 100; grep -v "^\[INFO\|^\[WARN" /workspace/knob1.log

# openrua op 145
grep -n "def move_joints" -A 12 rob.py

# openrua op 146
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rob.py", "old_string": "        pts = list(via or []) + [q]\\n        for i, p in enumerate(pts):\\n            t = secs * (i + 1) / len(pts)", "new_string": "        # densify (linear in joint space, <=0.05 rad per segment) and time points by arc length so\\n        # the controller sees a uniform-speed path; single big jumps otherwise abort with -5.\\n        raw = [self.arm_q()] + list(via or []) + [q]\\n        pts = []\\n        for a, b in zip(raw[:-1], raw[1:]):\\n            a, b = np.asarray(a, float), np.asarray(b, float)\\n            k = max(1, int(np.ceil(np.abs(b - a).max() / 0.05)))\\n            pts += [a + (b - a) * j / k for j in range(1, k + 1)]\\n        seg = np.array([np.abs(np.asarray(b) - np.asarray(a)).max() for a, b in zip(raw[:1] + pts[:-1], pts)])\\n        cum = np.cumsum(seg); cum = cum / cum[-1] if cum[-1] > 0 else np.linspace(1 / len(pts), 1, len(pts))\\n        for i, p in enumerate(pts):\\n            t = max(0.1, secs * cum[i])", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 147
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rob.py", "old_string": "        seg = np.array([np.abs(np.asarray(b) - np.asarray(a)).max() for a, b in zip(raw[:1] + pts[:-1], pts)])\\n        cum = np.cumsum(seg); cum = cum / cum[-1] if cum[-1] > 0 else np.linspace(1 / len(pts), 1, len(pts))\\n        for i, p in enumerate(pts):\\n            t = max(0.1, secs * cum[i])", "new_string": "        seg = np.array([np.abs(np.asarray(b) - np.asarray(a)).max() for a, b in zip(raw[:1] + pts[:-1], pts)])\\n        keep = seg > 1e-6\\n        pts = [p for p, k in zip(pts, keep) if k] or [np.asarray(q, float)]\\n        seg = seg[keep] if keep.any() else np.array([1.0])\\n        cum = np.cumsum(seg) / seg.sum()\\n        for i, p in enumerate(pts):\\n            t = max(0.2, secs * cum[i])", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 148
nohup python3 -u knobpick.py -0.239 -0.443 0.930 -80.8 0.22 > knob1.log 2>&1 &
sleep 120; grep -v "^\[INFO\|^\[WARN" /workspace/knob1.log

# openrua op 149
timeout 120 python3 tools/perception/cam_snap.py frontview front10.png 2>&1 | grep -v "^\[INFO"; timeout 120 python3 tools/perception/cam_snap.py agentview agent14.png 2>&1 | grep -v "^\[INFO"

# openrua op 150
python3 -c "
import cv2; im=cv2.imread('front10.png'); c=im[150:300,60:260]; cv2.imwrite('front10_crop.png', cv2.resize(c,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('agent14.png'); cv2.imwrite('agent14_crop.png', cv2.resize(im[0:260,0:320],None,fx=2,fy=2,interpolation=cv2.INTER_CUBIC))"

# openrua op 151
mkdir -p "$(dirname /workspace/rightpot.py)"
cat > /workspace/rightpot.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Rotate a knob-pinched pot about the pinch point so its axis becomes vertical (base down).
Usage: python3 -u rightpot.py <ax> <ay> [hz_pinch=0.095]
  (ax,ay): world direction from the knob toward the pot base (horizontal), from the cloud fit.
"""
import sys
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from rob import Robot, log

a = np.array([float(sys.argv[1]), float(sys.argv[2]), 0.0]); a /= np.linalg.norm(a)
HZ = float(sys.argv[3]) if len(sys.argv) > 3 else 0.095
r = Robot("rightpot")
p, qu = r.fk()
R0 = Rot.from_quat(qu).as_matrix()
n = R0[:, 1]                                  # finger (pinch) axis
a = a - n * (a @ n); a /= np.linalg.norm(a)   # make sure a is perpendicular to the pinch axis
b = np.array([0, 0, -1.0]); b = b - n * (b @ n); b /= np.linalg.norm(b)
ang = np.arctan2(np.cross(a, b) @ n, a @ b)
dR = Rot.from_rotvec(n * ang).as_matrix()
R1 = dR @ R0
log(f"pinch axis {np.round(n,3)}; rotate {np.degrees(ang):.1f} deg; hand z after: {np.round(R1[:,2],3)}")
Gp = p + HZ * R0[:, 2]
path = r.plan_rot(r.arm_q(), Gp, R1, step_deg=6.0)
assert path is not None, "rotation plan failed"
for i in range(0, len(path), 4):
    ok = r.move_path(path[i:i + 4], 4.0)
    f = r.fingers()
    log(f"chunk ok={ok} gap {f[0]-f[1]:.4f}")
    if not ok or f[0] - f[1] < 0.004:
        log("ABORT (stall or lost pot)"); sys.exit(1)
p, qu = r.fk()
log(f"hand {np.round(p,4)} R\n{np.round(Rot.from_quat(qu).as_matrix(),3)}")
log("RIGHT DONE")
OPENRUA_EOF

# openrua op 152
nohup python3 -u rightpot.py -0.16 0.987 0.095 > right1.log 2>&1 &
sleep 75; grep -v "^\[INFO\|^\[WARN" /workspace/right1.log

# openrua op 153
timeout 900 python3 - <<'EOF' 2>&1 | grep -v "^\[INFO\|^\[WARN"
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from rob import Robot, log
r=Robot("setdown")
p,qu=r.fk(); R=Rot.from_quat(qu).as_matrix()
tips=p+0.1034*R[:,2]; log("tips",tips.round(4),"gap",r.fingers())
tgt=p+[0,0,0.937-tips[2]]
ok=r.move_path(r.plan_line(r.arm_q(),tgt,R),8.0); log("down ok",ok,"gap",r.fingers())
r.gripper(0.04)
ok=r.move_path(r.plan_line(r.arm_q(),tgt+[0,0,0.22],R),6.0); log("up ok",ok)
EOF

# openrua op 154
timeout 120 python3 tools/perception/cam_snap.py agentview agent15.png 2>&1 | grep -v "^\[INFO"; timeout 120 python3 tools/perception/cam_snap.py frontview front11.png 2>&1 | grep -v "^\[INFO"

# openrua op 155
timeout 900 python3 - <<'EOF' 2>&1 | grep -v "^\[INFO\|^\[WARN"
import numpy as np
from rob import Robot, log
from geom import R_PLACE
r=Robot("mv")
ok=r.move_pose([-0.29,-0.40,1.25],R_PLACE,5.0,flip_ok=True); log("ok",ok)
EOF
timeout 300 python3 cloud.py robot0_eye_in_hand 2>&1 | grep -v "^\[INFO" | tail -1
cat > hmap.py <<'EOF'
import sys, numpy as np, cv2
x0,x1,y0,y1=map(float,sys.argv[1:5]); out=sys.argv[5]
C=np.load("robot0_eye_in_hand_cloud.npy").reshape(-1,3); C=C[np.isfinite(C).all(1)]
m=(C[:,2]>0.903)&(C[:,2]<1.10)&(C[:,0]>x0)&(C[:,0]<x1)&(C[:,1]>y0)&(C[:,1]<y1)
P=C[m]; print("n",len(P),"zmax",P[:,2].max().round(3) if len(P) else None)
nx,ny=int((x1-x0)*1000),int((y1-y0)*1000); H=np.zeros((nx,ny))
ix=((P[:,0]-x0)*1000).astype(int); iy=((P[:,1]-y0)*1000).astype(int)
for a,b,z in zip(ix,iy,P[:,2]):
    if 0<=a<nx and 0<=b<ny: H[a,b]=max(H[a,b],z)
img=np.zeros((nx,ny,3),np.uint8); v=np.clip((H-0.90)/0.10,0,1); img[...,1]=(v*255).astype(np.uint8); img[H>0,2]=80
img=cv2.resize(img,(ny*2,nx*2),interpolation=cv2.INTER_NEAREST)
for k in range(0,max(nx,ny),50):
    cv2.line(img,(k*2,0),(k*2,nx*2-1),(60,60,60),1); cv2.line(img,(0,k*2),(ny*2-1,k*2),(60,60,60),1)
    cv2.putText(img,f"y{y0+k/1000:.2f}",(k*2,12),cv2.FONT_HERSHEY_SIMPLEX,0.35,(255,255,255),1)
    cv2.putText(img,f"x{x0+k/1000:.2f}",(0,k*2+12),cv2.FONT_HERSHEY_SIMPLEX,0.35,(255,255,255),1)
cv2.imwrite(out,img)
EOF
python3 hmap.py -0.45 -0.10 -0.60 -0.25 hmap2.png

# openrua op 156
sed -i 's|travel / 0.25|travel / 0.15|g' rob.py; grep -c "0.15" rob.py

# openrua op 157
python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_cloud.npy')
m=(P[:,0]>-0.30)&(P[:,0]<-0.10)&(P[:,1]>-0.62)&(P[:,1]<-0.40)&(P[:,2]>0.905)
P=P[m]
for y in np.arange(-0.61,-0.41,0.01):
    s=P[(P[:,1]>=y)&(P[:,1]<y+0.01)]
    if len(s)<5: print(f'y {y:.2f}: -'); continue
    i=s[:,2].argmax()
    hi=s[s[:,2]>0.915]
    print(f'y {y:.2f}: n {len(s):4d} zmax {s[i,2]:.3f} at x {s[i,0]:.3f}  x-range(z>.915) {hi[:,0].min():.3f}..{hi[:,0].max():.3f} w {hi[:,0].max()-hi[:,0].min():.3f}' if len(hi) else f'y {y:.2f}: low')
"


# openrua op 158
python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_cloud.npy'); print(P.shape); P=P.reshape(-1,P.shape[-1]) if P.shape[-1]==3 else P.reshape(3,-1).T
P=P[np.isfinite(P).all(1)]
m=(P[:,0]>-0.30)&(P[:,0]<-0.10)&(P[:,1]>-0.62)&(P[:,1]<-0.40)&(P[:,2]>0.905)
P=P[m]
for y in np.arange(-0.61,-0.41,0.01):
    s=P[(P[:,1]>=y)&(P[:,1]<y+0.01)]
    if len(s)<5: print(f'y {y:.2f}: -'); continue
    i=s[:,2].argmax()
    hi=s[s[:,2]>0.915]
    print(f'y {y:.2f}: n {len(s):4d} zmax {s[i,2]:.3f} at x {s[i,0]:.3f}  x-range(z>.915) {hi[:,0].min():.3f}..{hi[:,0].max():.3f} w {hi[:,0].max()-hi[:,0].min():.3f}' if len(hi) else f'y {y:.2f}: low')
"


# openrua op 159
cat > look.py <<'EOF'
#!/usr/bin/env python3
"""Move the hand to (x,y,z) looking straight down, then capture the eye-in-hand cloud."""
import sys, subprocess
import numpy as np
from geom import R_PLACE
from rob import Robot, log
p = np.array(list(map(float, sys.argv[1:4])))
r = Robot("look")
q = r.ik(p, R_PLACE)
assert q is not None, "no IK"
ok = r.move_path([q], 6.0)
log(f"look ok={ok} hand {np.round(r.fk()[0],4)}")
subprocess.run(["python3", "cloud.py", "robot0_eye_in_hand"], check=True)
EOF
python3 -u look.py -0.20 -0.50 1.32 2>&1 | grep -v "^\[INFO\|^\[WARN" | tail -5

# openrua op 160
python3 hmap.py -0.35 -0.05 -0.65 -0.35 hmap3.png; python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_cloud.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,0]>-0.32)&(P[:,0]<-0.08)&(P[:,1]>-0.64)&(P[:,1]<-0.38)&(P[:,2]>0.905)&(P[:,2]<1.05)
P=P[m]
for y in np.arange(-0.63,-0.38,0.01):
    s=P[(P[:,1]>=y)&(P[:,1]<y+0.01)]
    if len(s)<5: print(f'y {y:.2f}: -'); continue
    i=s[:,2].argmax()
    hi=s[s[:,2]>0.915]
    print(f'y {y:.2f}: n {len(s):4d} zmax {s[i,2]:.3f} at x {s[i,0]:.3f}  x-range(z>.915) {hi[:,0].min():.3f}..{hi[:,0].max():.3f} w {hi[:,0].max()-hi[:,0].min():.3f}' if len(hi) else f'y {y:.2f}: low')
"

# openrua op 161
python3 fitpot.py -0.195 -0.495 -0.34 0.94 2>&1 | tail -30

# openrua op 162
cat geom.py | head -40; ls tools/perception; grep -n "def gripper\|def move_path\|def plan_rot" -A3 rob.py | head -40

# openrua op 163
mkdir -p "$(dirname /workspace/pot1.py)"
cat > /workspace/pot1.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Pick a LYING moka pot by its lid knob, right it (rotate about the pinch axis), carry it to the
stove and set it down. Everything is pre-planned on one IK branch before touching the pot.
Usage: python3 -u pot1.py <kx> <ky> <kz_axis> <ax> <ay> <tx> <ty> [phase]
  (kx,ky,kz_axis): knob cap centre; (ax,ay): horizontal direction knob -> pot base;
  (tx,ty): where the pot base should stand on the stove. phase: 'plan' (default) / 'go'.
"""
import sys
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from geom import R_PLACE, Rz
from rob import Robot, log

kx, ky, kz, ax, ay, tx, ty = map(float, sys.argv[1:8])
GO = len(sys.argv) > 8 and sys.argv[8] == "go"
TIP, HZ = 0.1034, 0.093            # fingertip / pad-centre distance along hand z
LIFT = 0.21
PLATE_Z = 0.932
KNOB_ABOVE_BASE = 0.150
CARRY_Z = 1.16                    # knob height while carrying (pot base ~1.01, clears everything)
a = np.array([ax, ay, 0.0]); a /= np.linalg.norm(a)
phi = np.arctan2(a[1], a[0])
FLIP = np.diag([-1.0, -1.0, 1.0])
r = Robot("pot1")
q_now = r.arm_q()
K = np.array([kx, ky, kz])                       # knob cap centre (pinch point)
K_lift = K + [0, 0, LIFT]


def right_R(R, a_now):
    """Orientation after rotating about the finger axis so a_now -> -z."""
    n = R[:, 1]
    aa = a_now - n * (a_now @ n); aa /= np.linalg.norm(aa)
    b = np.array([0, 0, -1.0]); b = b - n * (b @ n); b /= np.linalg.norm(b)
    ang = np.arctan2(np.cross(aa, b) @ n, aa @ b)
    return Rot.from_rotvec(n * ang).as_matrix() @ R


def plan(fl, yaw_deg):
    tag = f"flip={fl[0,0]:+.0f} yaw={yaw_deg:+.0f}"
    R_G = Rz(phi) @ R_PLACE @ fl                  # z down, fingers perpendicular to the pot axis
    p_grasp = K - HZ * R_G[:, 2]
    p_hover = p_grasp + [0, 0, 0.15]
    p_lift = p_grasp + [0, 0, LIFT]
    R_Y = Rz(np.radians(yaw_deg)) @ R_G
    a_y = Rz(np.radians(yaw_deg)) @ a
    R_F = right_R(R_Y, a_y)                       # hand horizontal, z = -a_y
    q_h = None
    for s in (q_now, np.array([0, -0.785, 0, -2.356, 0, 1.571, 0.785]),
              np.array([-0.5, 0.3, 0, -1.6, 0, 1.9, 0.0]), np.array([-0.8, 0.5, 0.3, -1.8, 0, 2.2, 0.5])):
        q = r.ik(p_hover, R_G, s)
        if q is not None and r.within_limits(q):
            q_h = q; break
    if q_h is None:
        log(f"{tag}: no hover IK"); return None
    segs, q = {}, q_h
    K_carry = np.array([tx, ty, CARRY_Z])
    K_down = np.array([tx, ty, PLATE_Z + KNOB_ABOVE_BASE + 0.003])
    steps = [("descend", lambda q: r.plan_line(q, p_grasp, R_G)),
             ("lift", lambda q: r.plan_line(q, p_lift, R_G)),
             ("yaw", lambda q: r.plan_rot(q, K_lift, R_Y, step_deg=8)),
             ("rot", lambda q: r.plan_rot(q, K_lift, R_F, step_deg=6)),
             ("up", lambda q: r.plan_line(q, K_lift + [0, 0, CARRY_Z - K_lift[2]] - HZ * R_F[:, 2], R_F, step=0.03)),
             ("carry", lambda q: r.plan_line(q, K_carry - HZ * R_F[:, 2], R_F, step=0.03)),
             ("down", lambda q: r.plan_line(q, K_down - HZ * R_F[:, 2], R_F)),
             ("retreat", lambda q: r.plan_line(q, K_down - HZ * R_F[:, 2] - 0.08 * R_F[:, 2] + [0, 0, 0.06], R_F))]
    for name, fn in steps:
        seg = fn(q)
        if seg is None:
            log(f"{tag}: segment {name} infeasible"); return None
        segs[name] = seg
        q = seg[-1]
    allq = np.array([q_now, q_h] + sum((segs[n] for n, _ in steps), []))
    travel = np.abs(np.diff(allq, axis=0)).max(1).sum()
    log(f"{tag}: feasible, travel {travel:.2f}; hover q {np.round(q_h,2)}; final hand z {np.round(R_F[:,2],2)}")
    return dict(tag=tag, q_h=q_h, segs=segs, travel=travel, R_G=R_G, R_Y=R_Y, R_F=R_F,
                p_grasp=p_grasp, p_hover=p_hover, p_lift=p_lift, K_carry=K_carry, K_down=K_down)


plans = []
for yaw_deg in (np.degrees(np.pi - phi), 0.0, np.degrees(np.pi / 2 - phi), np.degrees(-np.pi / 2 - phi)):
    yaw_deg = (yaw_deg + 180) % 360 - 180
    for fl in (np.eye(3), FLIP):
        pl = plan(fl, yaw_deg)
        if pl is not None:
            plans.append(pl)
    if plans:
        break
if not plans:
    log("NO FEASIBLE PLAN"); sys.exit(1)
P = min(plans, key=lambda p: p["travel"])
log(f"chosen {P['tag']}")
if not GO:
    sys.exit(0)


def chunks(path, n, secs, what):
    for i in range(0, len(path), n):
        ok = r.move_path(path[i:i + n], secs)
        f = r.fingers()
        log(f"{what} chunk ok={ok} gap {f[0]-f[1]:.4f}")
        if not ok or f[0] - f[1] < 0.004:
            log(f"ABORT during {what}"); sys.exit(1)


r.gripper(0.04)
assert r.move_path([P["q_h"]], 5.0), "hover failed"
assert r.move_path(r.plan_line(r.arm_q(), P["p_grasp"], P["R_G"]), 5.0), "descend failed"
p, _ = r.fk()
log(f"pads at {np.round(p + HZ * P['R_G'][:,2], 4)} (knob {np.round(K,4)})")
f = r.gripper(0.0)
gap = f[0] - f[1]
log(f"gap after close {gap:.4f}")
if gap < 0.004:
    log("PINCH FAILED (closed on air)")
    r.gripper(0.04)
    r.move_path(r.plan_line(r.arm_q(), P["p_hover"], P["R_G"]), 4.0)
    sys.exit(2)
assert r.move_path(r.plan_line(r.arm_q(), P["p_lift"], P["R_G"]), 5.0), "lift failed"
# re-plan the remaining segments from the actual configuration (same branch)
q = r.arm_q()
yaw = r.plan_rot(q, K_lift, P["R_Y"], step_deg=8); assert yaw is not None, "yaw replan"
chunks(yaw, 4, 4.0, "yaw")
q = r.arm_q()
rot = r.plan_rot(q, K_lift, P["R_F"], step_deg=6); assert rot is not None, "rot replan"
chunks(rot, 4, 4.0, "rot")
p, qu = r.fk()
log(f"righted: hand {np.round(p,4)} z {np.round(Rot.from_quat(qu).as_matrix()[:,2],3)} fingers {r.fingers()}")
log("RIGHTED -- pausing here for a visual check")
OPENRUA_EOF

# openrua op 164
timeout 1500 python3 -u pot1.py -0.172 -0.566 0.937 -0.334 0.943 0.19 0.09 plan 2>&1 | grep -v "^\[INFO\|^\[WARN" | tail -30

# openrua op 165
python3 - <<'EOF'
import re
s=open('pot1.py').read()
s=s.replace('''def plan(fl, yaw_deg):
    tag = f"flip={fl[0,0]:+.0f} yaw={yaw_deg:+.0f}"''','''def plan(fl, yaw_deg, rot_first=False):
    tag = f"flip={fl[0,0]:+.0f} yaw={yaw_deg:+.0f} rot_first={rot_first}"''')
s=s.replace('''    R_F = right_R(R_Y, a_y)                       # hand horizontal, z = -a_y
''','''    R_F = right_R(R_Y, a_y)                       # hand horizontal, z = -a_y
    R_F0 = right_R(R_G, a)                        # righted before yawing (rot_first)
''')
s=s.replace('''             ("yaw", lambda q: r.plan_rot(q, K_lift, R_Y, step_deg=8)),
             ("rot", lambda q: r.plan_rot(q, K_lift, R_F, step_deg=6)),
''','''             ("rot", lambda q: r.plan_rot(q, K_lift, R_F0, step_deg=6)),
             ("yaw", lambda q: r.plan_rot(q, K_lift, R_F, step_deg=6)),
''' )
s=s.replace('''    for name, fn in steps:
        seg = fn(q)''','''    if not rot_first:
        steps[2:4] = [("yaw", lambda q: r.plan_rot(q, K_lift, R_Y, step_deg=8)),
                      ("rot", lambda q: r.plan_rot(q, K_lift, R_F, step_deg=6))]
    for name, fn in steps:
        seg = fn(q)''')
s=s.replace('''    return dict(tag=tag, q_h=q_h, segs=segs, travel=travel, R_G=R_G, R_Y=R_Y, R_F=R_F,''','''    return dict(tag=tag, q_h=q_h, segs=segs, travel=travel, R_G=R_G, R_Y=R_Y, R_F=R_F, R_F0=R_F0,
                rot_first=rot_first, order=[n for n, _ in steps],''')
s=s.replace('''    for fl in (np.eye(3), FLIP):
        pl = plan(fl, yaw_deg)
        if pl is not None:
            plans.append(pl)''','''    for fl in (np.eye(3), FLIP):
        for rf in (False, True):
            pl = plan(fl, yaw_deg, rf)
            if pl is not None:
                plans.append(pl)''')
s=s.replace('''q = r.arm_q()
yaw = r.plan_rot(q, K_lift, P["R_Y"], step_deg=8); assert yaw is not None, "yaw replan"
chunks(yaw, 4, 4.0, "yaw")
q = r.arm_q()
rot = r.plan_rot(q, K_lift, P["R_F"], step_deg=6); assert rot is not None, "rot replan"
chunks(rot, 4, 4.0, "rot")''','''if P["rot_first"]:
    seq = [("rot", P["R_F0"]), ("yaw", P["R_F"])]
else:
    seq = [("yaw", P["R_Y"]), ("rot", P["R_F"])]
for name, RR in seq:
    seg = r.plan_rot(r.arm_q(), K_lift, RR, step_deg=6); assert seg is not None, f"{name} replan"
    chunks(seg, 4, 4.0, name)''')
open('pot1.py','w').write(s)
EOF
timeout 1500 python3 -u pot1.py -0.172 -0.566 0.937 -0.334 0.943 0.19 0.09 plan 2>&1 | grep -v "^\[INFO\|^\[WARN" | grep -v ik_local | tail -30

# openrua op 166
python3 - <<'EOF' 2>&1 | grep -v "^\[INFO\|^\[WARN"
import rclpy, numpy as np
from rob import Robot, ARM
from moveit_msgs.srv import GetPositionFK
r=Robot("basefind")
req=GetPositionFK.Request(); req.header.frame_id=""; req.fk_link_names=["panda_link0","panda_link1","panda_hand"]
req.robot_state.joint_state.name=ARM; req.robot_state.joint_state.position=list(r.arm_q())
fut=r.fk_cli.call_async(req); rclpy.spin_until_future_complete(r.node,fut,timeout_sec=30)
for ps in fut.result().pose_stamped:
    p=ps.pose.position; print(ps.header.frame_id, round(p.x,3),round(p.y,3),round(p.z,3))
EOF

# openrua op 167
python3 - <<'EOF'
s=open('pot1.py').read()
s=s.replace('''K_lift = K + [0, 0, LIFT]
''','''K_lift = K + [0, 0, LIFT]
BASE = np.array([-0.66, 0.0])


def azim(K):
    """Azimuth of the hand direction pointing radially away from the robot base at K."""
    return np.arctan2(K[1] - BASE[1], K[0] - BASE[0])


def plan_carry(q0, K0, K1, R0, step=0.03):
    """Carry the pinch point from K0 to K1 keeping the hand horizontal and pointing radially away
    from the base (the pot stays vertical since the hand only yaws about the vertical)."""
    n = max(1, int(np.ceil(np.linalg.norm(K1 - K0) / step)))
    path, q = [], np.array(q0)
    for i in range(1, n + 1):
        Ki = K0 + (K1 - K0) * i / n
        Ri = Rz(azim(Ki) - azim(K0)) @ R0
        q = r.ik_local(Ki - HZ * Ri[:, 2], Ri, q)
        if q is None or not r.within_limits(q):
            log(f"plan_carry: fail at {i}/{n}" + ("" if q is None else f" (limits) {np.round(q,2)}"))
            return None, None
        path.append(q)
    return path, Ri
''')
s=s.replace('''    K_carry = np.array([tx, ty, CARRY_Z])
    K_down = np.array([tx, ty, PLATE_Z + KNOB_ABOVE_BASE + 0.003])
''','''    K_carry = np.array([tx, ty, CARRY_Z])
    K_down = np.array([tx, ty, PLATE_Z + KNOB_ABOVE_BASE + 0.003])
    K_up = np.array([K_lift[0], K_lift[1], CARRY_Z])
    R_T = Rz(azim(K_carry) - azim(K_lift)) @ R_F     # hand orientation at the stove
''')
s=s.replace('''             ("up", lambda q: r.plan_line(q, K_lift + [0, 0, CARRY_Z - K_lift[2]] - HZ * R_F[:, 2], R_F, step=0.03)),
             ("carry", lambda q: r.plan_line(q, K_carry - HZ * R_F[:, 2], R_F, step=0.03)),
             ("down", lambda q: r.plan_line(q, K_down - HZ * R_F[:, 2], R_F)),
             ("retreat", lambda q: r.plan_line(q, K_down - HZ * R_F[:, 2] - 0.08 * R_F[:, 2] + [0, 0, 0.06], R_F))]''','''             ("up", lambda q: r.plan_line(q, K_up - HZ * R_F[:, 2], R_F, step=0.03)),
             ("carry", lambda q: plan_carry(q, K_up, K_carry, R_F)[0]),
             ("down", lambda q: r.plan_line(q, K_down - HZ * R_T[:, 2], R_T)),
             ("retreat", lambda q: r.plan_line(q, K_down - HZ * R_T[:, 2] - 0.08 * R_T[:, 2] + [0, 0, 0.06], R_T))]''')
s=s.replace('''    return dict(tag=tag, q_h=q_h, segs=segs, travel=travel, R_G=R_G, R_Y=R_Y, R_F=R_F, R_F0=R_F0,''','''    return dict(tag=tag, q_h=q_h, segs=segs, travel=travel, R_G=R_G, R_Y=R_Y, R_F=R_F, R_F0=R_F0, R_T=R_T,
                K_up=K_up,''')
s=s.replace('''for yaw_deg in (np.degrees(np.pi - phi), 0.0, np.degrees(np.pi / 2 - phi), np.degrees(-np.pi / 2 - phi)):''',
'''yaw_radial = np.degrees(azim(K) + np.pi - phi)     # hand ends pointing radially away from the base
for yaw_deg in (yaw_radial, yaw_radial - 30, yaw_radial + 30, np.degrees(np.pi - phi), 0.0):''')
open('pot1.py','w').write(s)
EOF
timeout 1500 python3 -u pot1.py -0.172 -0.566 0.937 -0.334 0.943 0.19 0.09 plan 2>&1 | grep -v "^\[INFO\|^\[WARN" | grep -v ik_local | tail -30

# openrua op 168
timeout 1700 python3 -u pot1.py -0.172 -0.566 0.937 -0.334 0.943 0.19 0.09 go > pot1a.log 2>&1; grep -v "^\[INFO\|^\[WARN" pot1a.log | grep -v "ik_local\|plan_rot: fail\|plan_line: fail\|infeasible" | tail -40

# openrua op 169
grep -n "cam\|argv" tools/perception/cam_snap.py | head; python3 tools/perception/cam_snap.py frontview front11.png 2>&1 | tail -1; python3 tools/perception/cam_snap.py agentview agent16.png 2>&1 | tail -1

# openrua op 170
ros2 topic list 2>/dev/null | grep -i "image\|camera\|depth" | head -20

# openrua op 171
for c in birdview galleryview robot0_robotview; do python3 tools/perception/cam_snap.py $c ${c}_1.png 2>&1 | tail -1; done

# openrua op 172
mkdir -p "$(dirname /workspace/pot1b.py)"
cat > /workspace/pot1b.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Carry a knob-pinched, hanging-vertical pot to (tx,ty) on the stove and set it down.
Usage: python3 -u pot1b.py <tx> <ty> [knob_above_base=0.150]
"""
import sys
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from geom import Rz
from rob import Robot, log

tx, ty = map(float, sys.argv[1:3])
KAB = float(sys.argv[3]) if len(sys.argv) > 3 else 0.150
HZ = 0.093
PLATE_Z = 0.932
CARRY_Z = 1.16
BASE = np.array([-0.66, 0.0])
r = Robot("pot1b")


def azim(K):
    return np.arctan2(K[1] - BASE[1], K[0] - BASE[0])


def plan_carry(q0, K0, K1, R0, step=0.03):
    n = max(1, int(np.ceil(np.linalg.norm(K1 - K0) / step)))
    path, q = [], np.array(q0)
    for i in range(1, n + 1):
        Ki = K0 + (K1 - K0) * i / n
        Ri = Rz(azim(Ki) - azim(K0)) @ R0
        q = r.ik_local(Ki - HZ * Ri[:, 2], Ri, q)
        if q is None or not r.within_limits(q):
            log(f"plan_carry: fail at {i}/{n}" + ("" if q is None else f" (limits) {np.round(q,2)}"))
            return None, None
        path.append(q)
    return path, Ri


def chunks(path, n, secs, what):
    for i in range(0, len(path), n):
        ok = r.move_path(path[i:i + n], secs)
        f = r.fingers()
        log(f"{what} chunk ok={ok} gap {f[0]-f[1]:.4f}")
        if not ok or f[0] - f[1] < 0.004:
            log(f"ABORT during {what}"); sys.exit(1)


p, qu = r.fk()
R0 = Rot.from_quat(qu).as_matrix()
assert abs(R0[2, 2]) < 0.1, "hand is not horizontal"
K0 = p + HZ * R0[:, 2]
K_up = np.array([K0[0], K0[1], CARRY_Z])
K_carry = np.array([tx, ty, CARRY_Z])
K_down = np.array([tx, ty, PLATE_Z + KAB + 0.003])
q = r.arm_q()
up = r.plan_line(q, K_up - HZ * R0[:, 2], R0, step=0.03); assert up is not None, "up plan"
carry, R_T = plan_carry(up[-1], K_up, K_carry, R0); assert carry is not None, "carry plan"
down = r.plan_line(carry[-1], K_down - HZ * R_T[:, 2], R_T); assert down is not None, "down plan"
p_ret = K_down - HZ * R_T[:, 2] - 0.08 * R_T[:, 2] + [0, 0, 0.06]
ret = r.plan_line(down[-1], p_ret, R_T); assert ret is not None, "retreat plan"
log(f"plans ok: up {len(up)} carry {len(carry)} down {len(down)} ret {len(ret)}; hand z at stove {np.round(R_T[:,2],3)}")

chunks(up, 6, 4.0, "up")
chunks(carry, 6, 4.0, "carry")
chunks(down, 6, 4.0, "down")
p, qu = r.fk(); Rn = Rot.from_quat(qu).as_matrix()
log(f"before release: pinch at {np.round(p + HZ * Rn[:,2],4)} fingers {r.fingers()}")
r.gripper(0.04)
ret = r.plan_line(r.arm_q(), p_ret, R_T); assert ret is not None, "retreat replan"
ok = r.move_path(ret, 4.0)
log(f"retreat ok={ok}")
log("PLACE DONE")
OPENRUA_EOF

# openrua op 173
timeout 1700 python3 -u pot1b.py 0.19 0.09 > pot1b.log 2>&1; grep -v "^\[INFO\|^\[WARN" pot1b.log | grep -v "ik_local" | tail -40

# openrua op 174
python3 tools/perception/cam_snap.py agentview agent17.png 2>&1 | tail -1; python3 tools/perception/cam_snap.py frontview front12.png 2>&1 | tail -1

# openrua op 175
python3 -u look.py 0.14 0.03 1.36 2>&1 | grep -v "^\[INFO\|^\[WARN" | tail -2; python3 hmap.py 0.02 0.36 -0.15 0.21 hmap4.png

# openrua op 176
python3 -u look.py 0.10 0.03 1.30 2>&1 | grep -v "^\[INFO\|^\[WARN" | tail -2; python3 hmap.py 0.02 0.36 -0.15 0.21 hmap4.png

# openrua op 177
python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_cloud.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,2]>0.915)&(P[:,2]<0.95)&(P[:,0]>0.0)&(P[:,0]<0.4)&(P[:,1]>-0.2)&(P[:,1]<0.25)
S=P[m]; print('plate-ish pts',len(S)); 
for lo,hi in [(0.915,0.925),(0.925,0.935),(0.935,0.95)]:
    s=S[(S[:,2]>=lo)&(S[:,2]<hi)]
    if len(s): print(lo,hi,len(s),'x',s[:,0].min().round(3),s[:,0].max().round(3),'y',s[:,1].min().round(3),s[:,1].max().round(3))
# burner ring: heights along y at x=0.19
for y in np.arange(-0.12,0.12,0.02):
    s=P[(abs(P[:,0]-0.19)<0.005)&(abs(P[:,1]-y)<0.005)]
    if len(s): print(f'y {y:.2f} z med {np.median(s[:,2]):.4f} max {s[:,2].max():.4f}')
"


# openrua op 178
python3 fitpot.py 0.16 0.08 0.77 0.64 2>&1 | tail -20; grep -n "text\|label" hmap.py | head

# openrua op 179
python3 fitpot.py 0.16 0.08 0.77 0.64 2>&1 | head -4; python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_cloud.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
P=P[(P[:,2]>0.94)&(P[:,0]>0.05)&(P[:,0]<0.35)&(P[:,1]>-0.1)&(P[:,1]<0.2)]
# slices along the fitted axis direction
c=np.array([0.16,0.08]); u=np.array([0.77,0.64]); u/=np.linalg.norm(u); v=np.array([-u[1],u[0]])
t=(P[:,:2]-c)@u; s=(P[:,:2]-c)@v
for tt in np.arange(-0.10,0.12,0.01):
    m=(t>=tt)&(t<tt+0.01)
    if m.sum()<5: print(f't {tt:+.2f}: -'); continue
    i=P[m][:,2].argmax()
    print(f't {tt:+.2f}: n {m.sum():4d} zmax {P[m][i,2]:.3f} at s {s[m][i]:+.3f}  s-range {s[m].min():+.3f}..{s[m].max():+.3f}')
"

# openrua op 180
python3 -u look.py 0.02 -0.06 1.30 2>&1 | grep -v "^\[INFO\|^\[WARN" | tail -1; python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_cloud.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
P=P[(P[:,2]>0.94)&(P[:,2]<1.1)&(P[:,0]>0.05)&(P[:,0]<0.35)&(P[:,1]>-0.1)&(P[:,1]<0.2)]
print('pts',len(P))
c=np.array([0.16,0.08]); u=np.array([0.70,0.715]); u/=np.linalg.norm(u); v=np.array([-u[1],u[0]])
t=(P[:,:2]-c)@u; s=(P[:,:2]-c)@v
for tt in np.arange(-0.14,0.12,0.01):
    m=(t>=tt)&(t<tt+0.01)
    if m.sum()<5: print(f't {tt:+.2f}: -'); continue
    i=P[m][:,2].argmax()
    print(f't {tt:+.2f}: n {m.sum():4d} zmax {P[m][i,2]:.3f} at s {s[m][i]:+.3f}  s-range {s[m].min():+.3f}..{s[m].max():+.3f}')
"

# openrua op 181
python3 fitpot.py 0.18 0.081 0.81 0.59 2>&1 | head -6; python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_cloud.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
P=P[(P[:,2]>0.94)&(P[:,2]<1.1)&(P[:,0]>0.08)&(P[:,0]<0.35)&(P[:,1]>-0.1)&(P[:,1]<0.2)]
# knob candidates: points z>0.95 with x<0.12
K=P[(P[:,0]<0.125)&(P[:,2]>0.95)]
print('knob pts',len(K)); print(np.round(K.mean(0),4), np.round(K.min(0),4), np.round(K.max(0),4))
"

# openrua op 182
cat hmap.py

# openrua op 183
python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_cloud.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
S=P[(P[:,2]>0.94)&(P[:,2]<1.1)&(P[:,0]>0.085)&(P[:,0]<0.14)&(P[:,1]>-0.02)&(P[:,1]<0.07)]
print(len(S))
for x in np.arange(0.085,0.14,0.005):
    s=S[(S[:,0]>=x)&(S[:,0]<x+0.005)]
    if len(s): print(f'x {x:.3f}: n {len(s):4d} y {s[:,1].min():.3f}..{s[:,1].max():.3f} zmax {s[:,2].max():.3f}')
"

# openrua op 184
python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_cloud.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
S=P[(P[:,2]>0.94)&(P[:,2]<1.1)&(P[:,0]>0.10)&(P[:,0]<0.30)&(P[:,1]>-0.05)&(P[:,1]<0.20)]
K=np.array([0.110,0.046])
best=None
for az in np.arange(20,55,1.0):
    u=np.array([np.cos(np.radians(az)),np.sin(np.radians(az))]); v=np.array([-u[1],u[0]])
    t=(S[:,:2]-K)@u; s=(S[:,:2]-K)@v
    # body slices t 0.03..0.13 excluding handle side (s<0.045): symmetric extent about axis?
    m=(t>0.03)&(t<0.13)&(S[:,2]>0.99)   # top ridge points only
    if m.sum()<50: continue
    err=np.abs(s[m]).mean()
    if best is None or err<best[0]: best=(err,az,m.sum())
print('best ridge az',best)
az=best[1]; u=np.array([np.cos(np.radians(az)),np.sin(np.radians(az))]); v=np.array([-u[1],u[0]])
t=(S[:,:2]-K)@u; s=(S[:,:2]-K)@v
for tt in np.arange(0.0,0.18,0.01):
    m=(t>=tt)&(t<tt+0.01)
    if m.sum()<3: print(f't {tt:.2f}: -'); continue
    hi=m&(S[:,2]>0.99)
    print(f't {tt:.2f}: n {m.sum():4d} zmax {S[m][:,2].max():.3f} s-range {s[m].min():+.3f}..{s[m].max():+.3f}', f'ridge s {s[hi].mean():+.3f}' if hi.sum() else '')
"

# openrua op 185
python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_cloud.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
D=P[(P[:,2]>0.928)&(P[:,2]<0.94)&(P[:,0]>0.08)&(P[:,0]<0.30)&(P[:,1]>-0.07)&(P[:,1]<0.14)]
print('disk pts',len(D),'x',D[:,0].min().round(3),D[:,0].max().round(3),'y',D[:,1].min().round(3),D[:,1].max().round(3), 'z',np.median(D[:,2]).round(4))
F=P[(P[:,2]>0.92)&(P[:,2]<0.928)&(P[:,0]>0.0)&(P[:,0]<0.35)&(P[:,1]>-0.1)&(P[:,1]<0.2)]
print('flat pts',len(F),'x',F[:,0].min().round(3),F[:,0].max().round(3),'y',F[:,1].min().round(3),F[:,1].max().round(3), 'z',np.median(F[:,2]).round(4))
# disk fit circle via extreme points per y row
for y in np.arange(-0.02,0.09,0.01):
    d=D[abs(D[:,1]-y)<0.003]
    if len(d): print(f'y {y:.2f}: x {d[:,0].min():.3f}..{d[:,0].max():.3f}')
"

# openrua op 186
python3 - <<'EOF'
s=open('pot1.py').read()
s=s.replace('''Usage: python3 -u pot1.py <kx> <ky> <kz_axis> <ax> <ay> <tx> <ty> [phase]''','''Usage: python3 -u pot1.py <kx> <ky> <kz_axis> <ax> <ay> <tx> <ty> <surface_z> [phase]''')
s=s.replace('''kx, ky, kz, ax, ay, tx, ty = map(float, sys.argv[1:8])
GO = len(sys.argv) > 8 and sys.argv[8] == "go"''','''kx, ky, kz, ax, ay, tx, ty, PLATE_Z = map(float, sys.argv[1:9])
GO = len(sys.argv) > 9 and sys.argv[9] == "go"''')
s=s.replace('''PLATE_Z = 0.932
''','')
s=s.replace('''K_down = np.array([tx, ty, PLATE_Z + KNOB_ABOVE_BASE + 0.003])''','''K_down = np.array([tx, ty, PLATE_Z + KNOB_ABOVE_BASE + 0.002])''')
s=s.replace('''for yaw_deg in (yaw_radial, yaw_radial - 30, yaw_radial + 30, np.degrees(np.pi - phi), 0.0):''',
'''for yaw_deg in (yaw_radial, yaw_radial - 30, yaw_radial + 30, np.degrees(np.pi - phi), yaw_radial - 60, yaw_radial + 60):''')
open('pot1.py','w').write(s)
EOF
timeout 1500 python3 -u pot1.py 0.110 0.046 0.964 0.875 0.485 0.196 0.034 0.9304 plan 2>&1 | grep -v "^\[INFO\|^\[WARN" | grep -v ik_local | tail -12

# openrua op 187
timeout 1500 python3 -u pot1.py 0.110 0.046 0.964 0.875 0.485 0.196 0.034 0.9304 plan 2>&1 | grep -v "^\[INFO\|^\[WARN" | grep -v ik_local | grep "infeasible\|feasible\|no hover" 

# openrua op 188
timeout 1750 python3 -u pot1.py 0.110 0.046 0.964 0.875 0.485 0.196 0.034 0.9304 go > pot1c.log 2>&1; grep -v "^\[INFO\|^\[WARN" pot1c.log | grep -v "ik_local\|plan_rot: fail\|plan_line: fail\|infeasible\|move done" | tail -30

# openrua op 189
python3 - <<'EOF'
s=open('pot1.py').read()
s=s.replace('''def plan(fl, yaw_deg, rot_first=False):
    tag = f"flip={fl[0,0]:+.0f} yaw={yaw_deg:+.0f} rot_first={rot_first}"''','''def hover_solutions(fl):
    R_G = Rz(phi) @ R_PLACE @ fl
    p_hover = K - HZ * R_G[:, 2] + [0, 0, 0.15]
    sols = []
    for s in (q_now, np.array([0, -0.785, 0, -2.356, 0, 1.571, 0.785]),
              np.array([-0.5, 0.3, 0, -1.6, 0, 1.9, 0.0]), np.array([-0.8, 0.5, 0.3, -1.8, 0, 2.2, 0.5]),
              np.array([0.3, 0.6, -0.3, -1.6, 0.3, 2.0, -1.5]), np.array([-0.3, 0.9, 0.6, -1.1, -0.5, 1.9, -2.5])):
        for _ in range(2):
            q = r.ik(p_hover, R_G, s)
            if q is not None and r.within_limits(q) and all(np.abs(q - o).max() > 0.05 for o in sols):
                sols.append(q)
    log(f"flip={fl[0,0]:+.0f}: {len(sols)} hover solutions")
    return sols


def plan(fl, yaw_deg, rot_first=False, q_h=None):
    tag = f"flip={fl[0,0]:+.0f} yaw={yaw_deg:+.0f} rot_first={rot_first} j7h={q_h[6]:+.2f}"''')
s=s.replace('''    q_h = None
    for s in (q_now, np.array([0, -0.785, 0, -2.356, 0, 1.571, 0.785]),
              np.array([-0.5, 0.3, 0, -1.6, 0, 1.9, 0.0]), np.array([-0.8, 0.5, 0.3, -1.8, 0, 2.2, 0.5])):
        q = r.ik(p_hover, R_G, s)
        if q is not None and r.within_limits(q):
            q_h = q; break
    if q_h is None:
        log(f"{tag}: no hover IK"); return None
''','')
s=s.replace('''plans = []
yaw_radial''','''plans = []
HOV = {+1: hover_solutions(np.eye(3)), -1: hover_solutions(FLIP)}
yaw_radial''')
s=s.replace('''    for fl in (np.eye(3), FLIP):
        for rf in (False, True):
            pl = plan(fl, yaw_deg, rf)
            if pl is not None:
                plans.append(pl)''','''    for fl in (np.eye(3), FLIP):
        for q_h in HOV[int(fl[0, 0])]:
            for rf in (False, True):
                pl = plan(fl, yaw_deg, rf, q_h)
                if pl is not None:
                    plans.append(pl)''')
open('pot1.py','w').write(s)
EOF
timeout 1750 python3 -u pot1.py 0.110 0.046 0.964 0.875 0.485 0.196 0.034 0.9304 plan 2>&1 | grep -v "^\[INFO\|^\[WARN" | grep "hover sol\|feasible,\|chosen\|NO FEAS"

# openrua op 190
timeout 1750 python3 -u pot1.py 0.110 0.046 0.964 0.875 0.485 0.196 0.034 0.9304 go > pot1c.log 2>&1; grep -v "^\[INFO\|^\[WARN" pot1c.log | grep -v "ik_local\|plan_rot: fail\|plan_line: fail\|infeasible\|move done\|hover sol" | tail -30

# openrua op 191
sed -i 's|^PLATE_Z = 0.932|PLATE_Z = float(sys.argv[4]) if len(sys.argv) > 4 else 0.9304|; s|PLATE_Z + KAB + 0.003|PLATE_Z + KAB + 0.002|' pot1b.py; grep -n "PLATE_Z" pot1b.py; python3 tools/perception/cam_snap.py galleryview gallery_2.png | tail -1

# openrua op 192
python3 - <<'EOF'
s=open('pot1b.py').read()
s=s.replace('''def chunks(path, n, secs, what):
    for i in range(0, len(path), n):
        ok = r.move_path(path[i:i + n], secs)
        f = r.fingers()
        log(f"{what} chunk ok={ok} gap {f[0]-f[1]:.4f}")
        if not ok or f[0] - f[1] < 0.004:''','''def chunks(path, n, secs, what, fatal=True):
    for i in range(0, len(path), n):
        ok = r.move_path(path[i:i + n], secs)
        f = r.fingers()
        log(f"{what} chunk ok={ok} gap {f[0]-f[1]:.4f}")
        if not ok and not fatal:
            log(f"{what}: stalled, continuing"); return
        if not ok or f[0] - f[1] < 0.004:''')
s=s.replace('''chunks(down, 6, 4.0, "down")''','''chunks(down, 6, 4.0, "down", fatal=False)''')
open('pot1b.py','w').write(s)
EOF
timeout 1700 python3 -u pot1b.py 0.196 0.034 0.158 0.9304 > pot1d.log 2>&1; grep -v "^\[INFO\|^\[WARN" pot1d.log | grep -v "ik_local\|move done" | tail -20

# openrua op 193
python3 tools/perception/cam_snap.py agentview agent18.png | tail -1

# openrua op 194
python3 -u look.py -0.12 0.20 1.33 2>&1 | grep -v "^\[INFO\|^\[WARN" | tail -1; python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_cloud.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
S=P[(P[:,2]>1.0)&(P[:,2]<1.1)&(P[:,0]>-0.2)&(P[:,0]<0.05)&(P[:,1]>0.1)&(P[:,1]<0.4)]
print('pts',len(S),'zmax',S[:,2].max().round(4))
for lo,hi in [(1.045,1.07),(1.03,1.045),(1.0,1.03)]:
    s=S[(S[:,2]>=lo)&(S[:,2]<hi)]
    if len(s): print(lo,hi,len(s),'mean',np.round(s.mean(0),4),'x',s[:,0].min().round(3),s[:,0].max().round(3),'y',s[:,1].min().round(3),s[:,1].max().round(3))
"

# openrua op 195
python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_cloud.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
S=P[(P[:,2]>0.95)&(P[:,2]<1.1)&(P[:,0]>-0.2)&(P[:,0]<0.05)&(P[:,1]>0.1)&(P[:,1]<0.4)]
for y in np.arange(0.15,0.32,0.01):
    s=S[(S[:,1]>=y)&(S[:,1]<y+0.01)]
    if len(s): print(f'y {y:.2f}: n {len(s):4d} x {s[:,0].min():.3f}..{s[:,0].max():.3f} z {s[:,2].min():.3f}..{s[:,2].max():.3f}')
"

# openrua op 196
mkdir -p "$(dirname /workspace/pot2.py)"
cat > /workspace/pot2.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Pick an UPRIGHT moka pot by its lid knob from above, carry it, set it down.
Usage: python3 -u pot2.py <kx> <ky> <kz_cap_centre> <tx> <ty> <surface_z> <handx_azim_deg>
"""
import sys
import numpy as np
from geom import R_PLACE, Rz
from rob import Robot, log

kx, ky, kz, tx, ty, SZ, psi = map(float, sys.argv[1:8])
HZ = 0.093
LIFT = 0.12
R = Rz(np.radians(psi)) @ R_PLACE
r = Robot("pot2")
q_now = r.arm_q()
K = np.array([kx, ky, kz])
KAB = kz - 0.90                                   # pinch point height above the pot base (table at 0.90)
K_lift = K + [0, 0, LIFT]
K_carry = np.array([tx, ty, kz + LIFT])
K_down = np.array([tx, ty, SZ + KAB + 0.002])
hp = lambda Kp: Kp - HZ * R[:, 2]                 # hand origin for a given pad-centre point

# hover solutions, pre-plan the whole chain on one branch
plans = []
for s in (q_now, np.array([0, -0.785, 0, -2.356, 0, 1.571, 0.785]), np.array([-0.5, 0.3, 0, -1.6, 0, 1.9, 0.0]),
          np.array([0.3, 0.6, -0.3, -1.6, 0.3, 2.0, -1.5]), np.array([-0.3, 0.9, 0.6, -1.1, -0.5, 1.9, -2.5])):
    q_h = r.ik(hp(K_lift), R, s)
    if q_h is None or not r.within_limits(q_h) or any(np.abs(q_h - p["q_h"]).max() < 0.05 for p in plans):
        continue
    segs, q, ok = {}, q_h, True
    for name, fn in [("descend", lambda q: r.plan_line(q, hp(K), R)),
                     ("lift", lambda q: r.plan_line(q, hp(K_lift), R)),
                     ("carry", lambda q: r.plan_line(q, hp(K_carry), R, step=0.03)),
                     ("down", lambda q: r.plan_line(q, hp(K_down), R)),
                     ("retreat", lambda q: r.plan_line(q, hp(K_down) + [0, 0, 0.12], R))]:
        seg = fn(q)
        if seg is None:
            log(f"hover j7={q_h[6]:+.2f}: {name} infeasible"); ok = False; break
        segs[name] = seg; q = seg[-1]
    if ok:
        travel = np.abs(np.diff(np.array([q_now, q_h] + sum(segs.values(), [])), axis=0)).max(1).sum()
        log(f"hover q {np.round(q_h,2)} feasible, travel {travel:.2f}")
        plans.append(dict(q_h=q_h, travel=travel))
assert plans, "no feasible plan"
P = min(plans, key=lambda p: p["travel"])

r.gripper(0.04)
assert r.move_path([P["q_h"]], 5.0), "hover failed"
assert r.move_path(r.plan_line(r.arm_q(), hp(K), R), 5.0), "descend failed"
p, _ = r.fk()
log(f"pads at {np.round(p + HZ * R[:,2], 4)} (target {np.round(K,4)})")
f = r.gripper(0.0)
gap = f[0] - f[1]
log(f"gap after close {gap:.4f}")
if gap < 0.004:
    log("PINCH FAILED"); r.gripper(0.04)
    r.move_path(r.plan_line(r.arm_q(), hp(K_lift), R), 4.0); sys.exit(2)
for name, target in [("lift", K_lift), ("carry", K_carry)]:
    seg = r.plan_line(r.arm_q(), hp(target), R, step=0.03); assert seg is not None, f"{name} replan"
    for i in range(0, len(seg), 6):
        ok = r.move_path(seg[i:i + 6], 4.0)
        f = r.fingers()
        log(f"{name} chunk ok={ok} gap {f[0]-f[1]:.4f}")
        if not ok or f[0] - f[1] < 0.004:
            log(f"ABORT during {name}"); sys.exit(1)
seg = r.plan_line(r.arm_q(), hp(K_down), R); assert seg is not None, "down replan"
ok = r.move_path(seg, 4.0)
p, _ = r.fk()
log(f"down ok={ok}; pads at {np.round(p + HZ * R[:,2], 4)} (target {np.round(K_down,4)}) fingers {r.fingers()}")
r.gripper(0.04)
seg = r.plan_line(r.arm_q(), hp(K_down) + [0, 0, 0.12], R); assert seg is not None, "retreat replan"
log(f"retreat ok={r.move_path(seg, 4.0)}")
log("POT2 DONE")
OPENRUA_EOF

# openrua op 197
timeout 1750 python3 -u pot2.py -0.0702 0.2438 1.046 0.13 0.09 0.925 140 > pot2a.log 2>&1; grep -v "^\[INFO\|^\[WARN" pot2a.log | grep -v "ik_local\|move done\|plan_line: fail" | tail -25

# openrua op 198
grep "plan_line: fail\|ik_local" pot2a.log | tail -4

# openrua op 199
sed -i 's|\[0, 0, 0.12\], R|[0, 0, 0.06], R|g' pot2.py; timeout 1750 python3 -u pot2.py -0.0702 0.2438 1.046 0.13 0.09 0.925 140 > pot2a.log 2>&1; grep -v "^\[INFO\|^\[WARN" pot2a.log | grep -v "ik_local\|move done\|plan_line: fail" | tail -25

# openrua op 200
python3 -u look.py 0.02 0.03 1.30 2>&1 | grep -v "^\[INFO\|^\[WARN" | tail -1; python3 hmap.py 0.05 0.32 -0.10 0.20 hmap5.png; for c in agentview frontview birdview; do python3 tools/perception/cam_snap.py $c ${c}_final.png | tail -1; done

# openrua op 201
python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_cloud.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
S=P[(P[:,2]>1.04)&(P[:,2]<1.12)&(P[:,0]>0.05)&(P[:,0]<0.32)&(P[:,1]>-0.1)&(P[:,1]<0.2)]
# knob caps: two clusters of the highest points
from collections import defaultdict
for name,(cx,cy) in {'pot1 (target 0.196,0.034)':(0.196,0.034),'pot2 (target 0.13,0.09)':(0.13,0.09)}.items():
    c=S[(np.hypot(S[:,0]-cx,S[:,1]-cy)<0.03)]
    top=c[c[:,2]>c[:,2].max()-0.012]
    print(name,'cap pts',len(top),'cap centre xy',np.round(top[:,:2].mean(0),4),'cap top z',top[:,2].max().round(4))
    # lid ring radius check: points z 1.03-1.06 within 6cm
    lid=P[(np.hypot(P[:,0]-cx,P[:,1]-cy)<0.06)&(P[:,2]>1.02)&(P[:,2]<1.05)]
    print('   lid-level pts centre',np.round(lid[:,:2].mean(0),4),'n',len(lid))
"


# openrua op 202
python3 -u nudge.py 0 0 0.04 4 2>&1 | grep -v "^\[INFO\|^\[WARN" | tail -1; python3 cloud.py robot0_eye_in_hand | tail -1; python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_cloud.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
for name,(cx,cy) in {'pot1':(0.20,0.03),'pot2':(0.13,0.09)}.items():
    c=P[(np.hypot(P[:,0]-cx,P[:,1]-cy)<0.03)&(P[:,2]>1.04)]
    top=c[c[:,2]>c[:,2].max()-0.012]
    print(name,'cap centre xy',np.round(top[:,:2].mean(0),4),'cap top z',top[:,2].max().round(4))
"; python3 state.py 2>/dev/null | head -5
